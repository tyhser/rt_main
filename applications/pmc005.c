#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "rs485.h"
#include "pmc005.h"
#include "app_modbus_slave.h"
#include "easyblink.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "pmc"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define MBP 0

#define PMC_SERIAL	"uart3"
#define PMC_BAUDRATE	9600
#define PMC_PARITY	0
#define PMC_MODE_CONTROL_PIN 58
#define PMC_LVL		1

enum slave_online_state {
	SLAVE_ONLINE_OK,
	SLAVE_ONLINE_CHECKING,
	SLAVE_ONLINE_CONNECTION_TIMEOUT,
	SLAVE_ONLINE_ERROR,
};

void set_valve(int id, int val);
int pmc_is_robot_busy(uint8_t station_addr, enum axis_id id);
extern uint16_t usSRegInBuf[S_REG_INPUT_NREGS];
extern ebled_t led0;

#define REG_XY_AXIS_STATE		(usSRegInBuf[0])
#define REG_Z_AXIS_STATE		(usSRegInBuf[1])
#define REG_SYRING_STATE		(usSRegInBuf[2])

#define REG_PMC_STATE			(usSRegInBuf[3])

#define REG_MOTOR1_POSITION_INDEX	22
#define REG_MOTOR2_POSITION_INDEX	24
#define REG_MOTOR3_POSITION_INDEX	26
#define REG_MOTOR4_POSITION_INDEX	28

#define REG_SLAVE_AXIS_ONLINE		(usSRegInBuf[4])
#define BREAK_OPEN			set_valve(22, 1)
#define BREAK_CLOSE			set_valve(22, 0)

#define MODBUS_SET_INT32_TO_INT16(index, value) \
    do { \
        usSRegInBuf[(index)    ] = (value) >> 16; \
        usSRegInBuf[(index) + 1] = (value); \
    } while (0)

enum motor_status {
	MOTOR_OK,
	MOTOR_ERROR,
	MOTOR_BUSY
};

enum pmc_status {
	PMC_OK				= 0,
	PMC_INIT_ERROR			= 1,
	PMC_INVAILD_CMD			= 2,
	PMC_INVAILD_OPERATION_DATA	= 3,
	PMC_COMMUNICATION_ERROR		= 5,
	PMC_UNINIT			= 7,
	PMC_OVERLOAD			= 9,
	PMC_CMD_OVERFLOW		= 15
};

struct cmd_line_info {
	uint8_t addr;
	int8_t cmd[4];
	int32_t data;
};

static uint8_t buf[256];
static rs485_inst_t *hinst = NULL;

void mb_set_device_online_state(uint8_t slave_addr, enum slave_online_state state)
{
	/*axis slave address is 1, deliver address is 2, sampler address is from 3*/
	*(&REG_SLAVE_AXIS_ONLINE + slave_addr - 1) = state;
}

int pmc_send_then_recv(uint8_t *read_cmd, int len, void *buf, int recv_size)
{
	uint8_t station_addr = read_cmd[1] - '0';

	if (station_addr < 1 || station_addr > 15)
		return -1;

	mb_set_device_online_state(station_addr, SLAVE_ONLINE_CHECKING);
	int recv_len = rs485_send_then_recv(hinst, (void *)read_cmd, len, buf, recv_size);
        if (recv_len < 0) {
		LOG_E("rs485 station:%d send datas error.", station_addr);
		easyblink(led0, 10, 200, 400);
		mb_set_device_online_state(station_addr, SLAVE_ONLINE_ERROR);
		return recv_len;
        }

        if (recv_len == 0) {
		LOG_E("rs485 station:%d recv timeout.", station_addr);
		easyblink(led0, 10, 200, 400);
		mb_set_device_online_state(station_addr, SLAVE_ONLINE_CONNECTION_TIMEOUT);
		return recv_len;
        }

	mb_set_device_online_state(station_addr, SLAVE_ONLINE_OK);
        
        *((uint8_t *)buf+recv_len) = 0;
	return recv_len;
}

uint8_t get_hex_ch(uint8_t num)
{
	switch (num) {
	case 0 ... 9:
		return ('0' + num);
	case 0x0a ... 0x0f:
		return ('A' + num - 0x0a);
	default:
		LOG_E("not number");
		return 0;
	}
}

int pmc_make_cmd_line(struct cmd_line_info *cmd_info, uint8_t *cmd_line, int len)
{
	uint8_t data_ch[25] = {0};

	if (len < 5) {
		LOG_E("cmd line buffer too small");
		return 0;
	}

	*cmd_line = '/';
	*(cmd_line + 1) = get_hex_ch(cmd_info->addr);
	rt_memcpy(cmd_line + 2, cmd_info->cmd, strlen((char *)(cmd_info->cmd)));
	sprintf((char *)data_ch, "%ld", cmd_info->data);
	rt_memcpy(cmd_line + strlen((char *)cmd_line), data_ch, strlen((char *)data_ch));
	*(cmd_line + strlen((char *)cmd_line)) = 'R';
	*(cmd_line + strlen((char *)cmd_line)) = '\r';
	*(cmd_line + strlen((char *)cmd_line)) = '\0';
	return strlen((char *)cmd_line);
}

static inline int32_t motor_route_in_pulse(enum motor_id id, int32_t route)
{
	int32_t pulses = 0;

	switch (id) {
	case MOTOR_1:
		pulses = X_AXIS_PULSE(route);
	break;
	case MOTOR_2:
		pulses = Y_AXIS_PULSE(route);
	break;
	case MOTOR_3:
		pulses = Z_AXIS_PULSE(route);
	break;
	case MOTOR_4:
		pulses = SYRING_PULSE(route);
	break;
	default:
	break;
	}
	return pulses;
}

void pmc_set_current_motor_speed(int station_addr, uint32_t speed)
{
	uint8_t cmd[25] = {0};
	uint8_t recv[128] = {0};
	struct cmd_line_info cmd_info = {
		.addr = station_addr,
		.cmd = "V",
		.data = speed,
	};

	pmc_make_cmd_line(&cmd_info, cmd, 25);
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

void pmc_select_motor(enum motor_id id, int station_addr)
{
	uint8_t cmd[25] = {0};
	uint8_t recv[128] = {0};
	struct cmd_line_info cmd_info = {
		.addr = station_addr,
		.cmd = "aM",
		.data = id + 1,
	};

	pmc_make_cmd_line(&cmd_info, cmd, 25);
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

int pmc_motor_absolute_position(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	uint8_t cmd[25] = {0};
	uint8_t recv[128] = {0};
	struct cmd_line_info cmd_line = {
		.addr = station_addr,
		.cmd = "A",
		.data = pos,
	};

	pmc_make_cmd_line(&cmd_line, cmd, 25);

	pmc_select_motor(motor_id, station_addr);
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	return 0;
}

int pmc_get_response_info(struct response_info *info, const uint8_t *recv, int len)
{
	int ret = 0;
	uint8_t pos = 0;

	if (len < 4) {
		LOG_E("response pkg bad");
		return -1;
        }
        if ((recv[0] == 0xff) && (recv[1] == '/')) {
		pos = 1;
        } else if (recv[0] == '/') {
		pos = 0;
        } else {
		LOG_E("response pkg bad");
		return -1;
        }

	info->host_addr = recv[++pos];
	info->status = recv[++pos];
	pos++;
	for (int i = 0; recv[pos] != 0x03; i++) {
		info->data[i] = recv[pos];
		pos++;
		if (pos >= len) {
			LOG_E("response pkg bad");
			return -1;
		}
	}
	return ret;
}

enum pmc_status get_pmc_status(uint8_t *recv, int len)
{
	struct response_info info = {0};

	pmc_get_response_info(&info, recv, len);

	return (info.status & 0x0F);
}

void pmc_mb_set_motor_state(enum motor_id id, enum motor_status state)
{
	switch (id) {
	case MOTOR_1:
		REG_XY_AXIS_STATE = state;
	break;
	case MOTOR_2:
		REG_XY_AXIS_STATE = state;
	break;
	case MOTOR_3:
		REG_Z_AXIS_STATE = state;
		LOG_I("z axis state:%d", REG_Z_AXIS_STATE);
	break;
	case MOTOR_4:
		REG_SYRING_STATE = state;
	break;
	default:
		LOG_E("Unknow motor when set MB state");
	break;
	}
}

void pmc_update_motor_state(struct response_info *info)
{
	uint8_t busy_state = info->data[0] - '0';
	uint8_t pmc_status = info->status;

	//LOG_I("busy state:%c", info->data[0]);

	rt_enter_critical();

	if ((busy_state & 0x01) || (busy_state & 0x02))
		REG_XY_AXIS_STATE = MOTOR_BUSY;
	else
		REG_XY_AXIS_STATE = MOTOR_OK;
	if (busy_state & 0x04)
		REG_Z_AXIS_STATE = MOTOR_BUSY;
	else
		REG_Z_AXIS_STATE = MOTOR_OK;
	if (busy_state & 0x08)
		REG_SYRING_STATE = MOTOR_BUSY;
	else
		REG_SYRING_STATE = MOTOR_OK;

	REG_PMC_STATE = pmc_status;

	rt_exit_critical();
}

void pmc_stop(uint8_t station_addr)
{
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};

	cmd[0] = '/';
	cmd[1] = get_hex_ch(station_addr);
	cmd[2] = 'T';
	cmd[3] = 'R';
	cmd[4] = '\r';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	REG_PMC_STATE = get_pmc_status(recv, strlen((char *)recv));

}

void pmc_robot_home(uint8_t station_addr)
{
	uint8_t cmd[] = "/1aM1Z60000aM2Z60000aM3Z60000R\r";
	uint8_t recv[128] = {0};
	cmd[1] = get_hex_ch(station_addr);
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

void pmc_robot_init(uint8_t station_addr)
{
	int32_t pos = 0;
#if MBP
	uint8_t cmd[] = "/1n3aM3Z60000aM1j32m120L30h50V4000Z60000V16000aM2j2m120V20000D25000z0aM4m100L1000V64000Z50000R\r";
#else
	uint8_t cmd[] = "/1n3aM3f0j16m120L120h50V16000Z60000V64000aM1f2j16m120L100h20V16000Z120000V64000aM2f0j16m120L110h20V16000Z100000V64000aM4f0j16m125L900V30000Z120000R\r";
#endif
	uint8_t recv[128] = {0};
	cmd[1] = get_hex_ch(station_addr);
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, XY_AXIS) &&
				!pmc_is_robot_busy(station_addr, Z_AXIS) &&
				!pmc_is_robot_busy(station_addr, SYRING))
			break;
		rt_thread_mdelay(300);
	}
	pos = pmc_get_motor_position(MOTOR_1);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX, pos);
	pos = pmc_get_motor_position(MOTOR_2);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR2_POSITION_INDEX, pos);
	pos = pmc_get_motor_position(MOTOR_3);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR3_POSITION_INDEX, pos);
	pos = pmc_get_motor_position(MOTOR_4);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR4_POSITION_INDEX, pos);
}

int pmc_is_motor_busy(uint8_t station_addr, enum motor_id id)
{
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	struct response_info info = {0};
	uint32_t len = 0;

	cmd[0] = '/';
	cmd[1] = get_hex_ch(station_addr);
	cmd[2] = '?';
	cmd[3] = 'a';
	cmd[4] = 'S';
	cmd[5] = '\r';

	len = pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, len);
	pmc_update_motor_state(&info);
	REG_PMC_STATE = info.status & 0x0F;
	if (info.data[0] & (0x01 << id))
		return 1;
	else
		return 0;
}

int pmc_is_robot_busy(uint8_t station_addr, enum axis_id id)
{
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	struct response_info info = {0};
	uint32_t len = 0;

	cmd[0] = '/';
	cmd[1] = get_hex_ch(station_addr);
	cmd[2] = '?';
	cmd[3] = 'a';
	cmd[4] = 'S';
	cmd[5] = '\r';

	len = pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, len);
	pmc_update_motor_state(&info);
	switch (id) {
	case XY_AXIS:
		return REG_XY_AXIS_STATE;
		break;
	case Z_AXIS:
		return REG_Z_AXIS_STATE;
		break;
	case SYRING:
		return REG_SYRING_STATE;
		break;
	default:
		return 0;
		break;
	}
}

void pmc_block_wait_motor_free(uint8_t station_addr, enum motor_id id)
{
	for (int i = 0; i < 100; i++) {
		if (!pmc_is_motor_busy(station_addr, id))
			break;
		rt_thread_mdelay(300);
	}
}

int32_t pmc_get_current_motor_position(void)
{
	uint8_t cmd[] = "/1?0\r";
	uint8_t recv[128] = {0};
	struct response_info info = {0};
	uint32_t position = 0;

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	position = atol((char *)info.data);
	return position;
}

int32_t pmc_get_motor_position(enum motor_id id)
{
	int32_t current_position = 0;

	pmc_select_motor(id, ROBOT_ADDR);
	current_position = pmc_get_current_motor_position();
	return current_position;
}

uint32_t pmc_get_current_motor_max_speed(void)
{
	uint8_t cmd[] = "/1?2\r";
	uint8_t recv[128] = {0};
	struct response_info info = {0};
	uint32_t speed = 0;

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	speed = atol((char *)info.data);
	return speed;
}

static int get_limit_data(char *s, int buf_out[2])
{
	char *delim = ",";

	buf_out[0] = atoi(strtok(s, delim));
	buf_out[1] = atoi(strtok(NULL, delim));
	return 0;
}

enum axis_tristate_pos pmc_get_motor_tristate_pos(uint32_t addr, enum motor_id id)
{
	uint8_t cmd[] = "/1?aa1\r";
	uint8_t recv[128] = {0};
	struct response_info info = {0};
	int limit_buf[2] = {0};

	cmd[1] = get_hex_ch(addr);
	cmd[5] = id + '1';
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);
	get_limit_data((char *)info.data, limit_buf);

	if (!limit_buf[0] && !limit_buf[1])
		return MIDDLE_POS;
	if (limit_buf[0] != 0 && !limit_buf[1])
		return LOW_POS;
	if (!limit_buf[0] && limit_buf[1] != 0)
		return HIGH_POS;
	return UNKNOW_POS;
}

void pmc_motor_tristate(int argc, char *argv[])
{
	int tristate_pos = 0;

	if ((argc < 2) || (argv[1][0] < '1') || (argv[1][0] > '4')) {
		rt_kprintf("usage: pmc_motor_tristate [1/2/3/4]\n");
		return;
	}

	tristate_pos = pmc_get_motor_tristate_pos(1, argv[1][0] - '1');
	switch (tristate_pos)
	{
	case LOW_POS:
		rt_kprintf("LOW\n");
	break;
	case MIDDLE_POS:
		rt_kprintf("MIDDLE\n");
	break;
	case HIGH_POS:
		rt_kprintf("HIGH\n");
	break;
	default:
		rt_kprintf("UNKNOW\n");
	break;
	}
}
MSH_CMD_EXPORT(pmc_motor_tristate, pmc get tristate);

void pmc_motor_home(uint8_t station_addr, enum motor_id id)
{
	if (id > 4) {
		LOG_E("unkown motor id home");
		easyblink(led0, 10, 200, 400);
		return;
	}
	uint8_t cmd[] = "/1aM1V12000Z120000R\r";
	uint8_t recv[128] = {0};
	uint32_t prev_speed = 0;
	int ret = 0;
	int32_t pos = 0;

	pmc_select_motor(id, station_addr);
	prev_speed = pmc_get_current_motor_max_speed();
	if (id == MOTOR_3) {
		BREAK_OPEN;
	}
	pmc_mb_set_motor_state(id, MOTOR_BUSY);
	cmd[1] = get_hex_ch(station_addr);
	cmd[4] = (uint8_t)(1 + id + '0');
	ret = pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	if (ret == 0)
		pmc_mb_set_motor_state(id, MOTOR_ERROR);

	pmc_block_wait_motor_free(station_addr, id);
	if (id == MOTOR_3) {
		BREAK_CLOSE;
	}
	pmc_set_current_motor_speed(station_addr, prev_speed);
	pos = pmc_get_motor_position(id);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX + 2 * id, pos);
}

uint32_t get_x_axis_max_speed_by_length(int32_t x)
{
	uint32_t speed = ((x * 320 / 335) + 32000);
	LOG_I("x:%d must speed:%d", x, speed);

	if (speed > 64000)
		return 64000;
	else if (speed < 32000)
		return 32000;
	else
		return speed;
}

void pmc_motor_xy_abs(uint8_t station_addr, int32_t x, int32_t y)
{
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	struct response_info info = {0};
	uint32_t prev_speed = 0;
	uint32_t x_axis_speed = 0;
	int32_t x_pos = 0;
	int32_t y_pos = 0;

	x_pos = pmc_get_motor_position(MOTOR_1);
	y_pos = pmc_get_motor_position(MOTOR_2);

	if ((x != x_pos) || (y != y_pos)) {
		pmc_motor_z_abs(ROBOT_ADDR, 0);
	}

	if (x > X_AXIS_PULSE(X_AXIS_LENGTH)) {
		LOG_W("X axis move over length");
		return;
	}
	if (y > Y_AXIS_PULSE(Y_AXIS_LENGTH)) {
		LOG_W("Y axis move over length");
		return;
	}

	pmc_select_motor(MOTOR_1, ROBOT_ADDR);
	prev_speed = pmc_get_current_motor_max_speed();
	x_axis_speed = get_x_axis_max_speed_by_length(abs(X_AXIS_PULSE_TO_LEN(x_pos)-x));
	LOG_I("x axis speed:%ld", x_axis_speed);

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM1", strlen("aM1"));

	*(cmd_pos + strlen((char *)cmd_pos)) = 'V';
	sprintf((char *)num_str, "%ld", x_axis_speed);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), num_str, strlen((char *)num_str));
	rt_memset(num_str, 0, 25);

	*(cmd_pos + strlen((char *)cmd_pos)) = 'B';
	sprintf((char *)num_str, "%ld", x);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), num_str, strlen((char *)num_str));
	rt_memset(num_str, 0, 25);

	*(cmd_pos + strlen((char *)cmd_pos)) = 'V';
	sprintf((char *)num_str, "%ld", prev_speed);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), num_str, strlen((char *)num_str));
	rt_memset(num_str, 0, 25);

	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM2", strlen("aM2"));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'B';
	sprintf((char *)num_str, "%ld", y);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, XY_AXIS))
			break;
		rt_thread_mdelay(300);
	}
	x_pos = pmc_get_motor_position(MOTOR_1);
	y_pos = pmc_get_motor_position(MOTOR_2);

	MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX, x_pos);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR2_POSITION_INDEX, y_pos);
}

void pmc_motor_z_abs(uint8_t station_addr, int32_t pos)
{
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	struct response_info info = {0};
	int32_t cpos = 0;

	cpos = pmc_get_motor_position(MOTOR_3);

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM3", strlen("aM3"));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'A';
	sprintf((char *)num_str, "%ld", motor_route_in_pulse(MOTOR_3, pos));
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	if (cpos != 0 || pos != 0)
		BREAK_OPEN;

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, Z_AXIS))
			break;
		rt_thread_mdelay(300);
	}
	BREAK_CLOSE;

	cpos = pmc_get_motor_position(MOTOR_3);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR3_POSITION_INDEX, cpos);
}

void pmc_motor_syring_abs(uint8_t station_addr, int32_t pos)
{
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	struct response_info info = {0};
	int32_t cpos = 0;

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM4", strlen("aM4"));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'A';
	sprintf((char *)num_str, "%ld", pos);
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, SYRING))
			break;
		rt_thread_mdelay(300);
	}
	cpos = pmc_get_motor_position(MOTOR_4);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR4_POSITION_INDEX, cpos);
}

void pmc_robot_syring_pp(uint8_t station_addr, uint16_t times)
{
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	struct response_info info = {0};
	uint32_t prev_speed = 0;
	int32_t pos = 0;

	pmc_select_motor(MOTOR_4, station_addr);
	prev_speed = pmc_get_current_motor_max_speed();

#ifdef APP_USING_PISTON_PUMP
#define PP_CMD "V34000Z64000gP32000D32000G"
#else
#define PP_CMD "V45000Z64000gP46000D46000G"
#endif

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM4", strlen("aM4"));
	rt_memcpy(cmd_pos + strlen((char *)cmd), PP_CMD, strlen(PP_CMD));

	sprintf((char *)num_str, "%u", times);
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_get_response_info(&info, recv, 128);

	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, SYRING))
			break;
		rt_thread_mdelay(300);
	}
	pmc_set_current_motor_speed(station_addr, prev_speed);
	pos = pmc_get_motor_position(MOTOR_4);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR4_POSITION_INDEX, pos);
#undef PP_CMD
}

int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	if (pos == 0)
		return 0;
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	uint32_t limit_length = 0;

	switch (motor_id)
	{
	case MOTOR_1:
		limit_length = X_AXIS_LENGTH;
	break;
	case MOTOR_2:
		limit_length = Y_AXIS_LENGTH;
	break;
	case MOTOR_3:
		limit_length = Z_AXIS_LENGTH;
	break;
	case MOTOR_4:
		limit_length = SYRING_LENGTH;
	break;
	default:
	break;
	}

	int32_t current_position = pmc_get_motor_position(motor_id);

	if (motor_route_in_pulse(motor_id, pos) + current_position > motor_route_in_pulse(motor_id, limit_length)) {
		LOG_W("motor:%d fwd ERROR move to:%d", motor_id + 1, motor_route_in_pulse(motor_id, pos) + current_position);
		return 0;
	}

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM", strlen("aM"));
	*(cmd_pos + strlen((char *)cmd_pos)) = '1' + motor_id;
	*(cmd_pos + strlen((char *)cmd_pos)) = 'P';
	sprintf((char *)num_str, "%ld", motor_route_in_pulse(motor_id, pos));
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	LOG_I("%s", cmd);
	if (motor_id == MOTOR_3) {
		BREAK_OPEN;
	}
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_block_wait_motor_free(station_addr, motor_id);
	if (motor_id == MOTOR_3) {
		BREAK_CLOSE;
	}
	current_position = pmc_get_motor_position(motor_id);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX + 2 * motor_id, current_position);
	return 0;
}

int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	if (pos == 0)
		return 0;

	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];

	int32_t current_position = pmc_get_motor_position(motor_id);

	if (current_position < motor_route_in_pulse(motor_id, pos)) {
		LOG_W("rcv ERROR current pos:%d, motor_route:%d", current_position, motor_route_in_pulse(motor_id, pos));
		if (motor_id == MOTOR_3) {
			BREAK_OPEN;
		}
		pmc_motor_absolute_position(station_addr, motor_id, 0);
		pmc_block_wait_motor_free(station_addr, motor_id);
		if (motor_id == MOTOR_3) {
			BREAK_CLOSE;
		}
		current_position = pmc_get_motor_position(motor_id);
		MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX + 2 * motor_id, current_position);
		return 0;
	}

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM", strlen("aM"));
	*(cmd_pos + strlen((char *)cmd_pos)) = '1' + motor_id;
	*(cmd_pos + strlen((char *)cmd_pos)) = 'D';
	sprintf((char *)num_str, "%ld", motor_route_in_pulse(motor_id, pos));
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));
	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	if (motor_id == MOTOR_3) {
		BREAK_OPEN;
	}
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	pmc_block_wait_motor_free(station_addr, motor_id);
	if (motor_id == MOTOR_3) {
		BREAK_CLOSE;
	}
	current_position = pmc_get_motor_position(motor_id);
	MODBUS_SET_INT32_TO_INT16(REG_MOTOR1_POSITION_INDEX + 2 * motor_id, current_position);
	return 0;
}

int pmc_motor_jog(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	if (pos == 0)
		return 0;
	if (pos > 0)
		pmc_motor_fwd(station_addr, motor_id, pos);
	else
		pmc_motor_rev(station_addr, motor_id, abs(pos));
	return 0;
}

void pmc_motor_speed_mode_stop(uint8_t addr, uint8_t id)
{
	uint8_t cmd[] = "/1T1\r";
	uint8_t recv[128] = {0};

	cmd[1] = get_hex_ch(addr);
	cmd[3] = id + '0';
	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

int pmc_motor_speed_mode(uint8_t station_addr, uint8_t motor_id, int16_t speed)
{
	LOG_I("enter pmc_motor_speed_mode:(%d %d %d)", station_addr, motor_id, speed);
	uint8_t num_str[25] = {0};
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t *cmd_pos = &cmd[0];
	int32_t send_result = 0;

	*(cmd_pos + strlen((char *)cmd_pos)) = '/';
	*(cmd_pos + strlen((char *)cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen((char *)cmd_pos), "aM", strlen("aM"));
	*(cmd_pos + strlen((char *)cmd_pos)) = motor_id + '0';
	*(cmd_pos + strlen((char *)cmd_pos)) = 'V';
#ifdef APP_USING_PMC_SAMPLER
	sprintf((char *)num_str, "%u", (uint16_t)abs(speed * 160 / 3));
#else
	sprintf((char *)num_str, "%u", (uint16_t)abs(speed));
#endif
	rt_memcpy(cmd_pos + strlen((char *)cmd), num_str, strlen((char *)num_str));

	if (speed > 0)
		*(cmd_pos + strlen((char *)cmd_pos)) = 'C';
	else
		*(cmd_pos + strlen((char *)cmd_pos)) = 'E';

	*(cmd_pos + strlen((char *)cmd_pos)) = '0';

	*(cmd_pos + strlen((char *)cmd_pos)) = 'O';
	if (speed == 0)
		*(cmd_pos + strlen((char *)cmd_pos)) = '0';
	else
		*(cmd_pos + strlen((char *)cmd_pos)) = '1';

	*(cmd_pos + strlen((char *)cmd_pos)) = 'R';
	*(cmd_pos + strlen((char *)cmd_pos)) = '\r';

	rt_thread_mdelay(60);
	send_result = pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
	return send_result;
}

void pmc_set_valve(uint8_t station_addr, enum pmc_valve value)
{
	uint8_t cmd[128] = "/1J0R\r";
	uint8_t recv[128] = {0};

	cmd[1] = get_hex_ch(station_addr);
	cmd[3] = value + '0';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

void deliver_set_valve(uint8_t station_addr, uint8_t value)
{
	uint8_t cmd[128] = {0};
	uint8_t recv[128] = {0};
	uint8_t num_str[25] = {0};
	uint8_t pos = 0;

	cmd[pos++] = (uint8_t)'/';
	cmd[pos++] = (uint8_t)'2';
	cmd[pos++] = (uint8_t)'J';
	sprintf((char *)num_str, "%u", (uint16_t)value);
	memcpy(&cmd[pos], num_str, strlen((char *)num_str));

	*(cmd + strlen((char *)cmd)) = (uint8_t)'R';
	*(cmd + strlen((char *)cmd)) = (uint8_t)'\r';

	pmc_send_then_recv(cmd, strlen((char *)cmd), recv, 128);
}

void PMC(int argc, char *argv[])
{
	uint8_t cmd[256] = {0};
	int recv_len = 0;

	rt_memcpy(&cmd[0], &argv[1][0], strlen((char *)argv[1]) + 1);
	cmd[strlen((char *)argv[1])] = '\r';
	recv_len  = pmc_send_then_recv(cmd, strlen((char *)cmd), buf, 128);
	if (recv_len > 0)
		LOG_HEX("pmc recv", 16, buf, recv_len);
	rt_memset(buf, 0, 256);
}
MSH_CMD_EXPORT(PMC, test pmc);

int pmc_init(void)
{
	hinst = rs485_create(PMC_SERIAL, PMC_BAUDRATE, PMC_PARITY, PMC_MODE_CONTROL_PIN, PMC_LVL);
	if (hinst == RT_NULL) {
        	LOG_E("create rs485 instance fail.");
		easyblink(led0, -1, 200, 400);
        	return -1;
    	}
	rs485_set_recv_tmo(hinst, 500);
	rs485_set_byte_tmo(hinst, 45000 / PMC_BAUDRATE);
    	if (rs485_connect(hinst) != RT_EOK) {
		rs485_destory(hinst);
		LOG_E("rs485 connect fail.");
		easyblink(led0, -1, 200, 400);
		return -1;
    	}
	rt_thread_mdelay(500);
	BREAK_OPEN;
	pmc_robot_init(1);
	BREAK_CLOSE;
	return 0;
}

void set_baudrate_for_pmc(int argc, char *argv[])
{
	rs485_config(hinst, atol((char *)argv[1]), 8, PMC_PARITY, 1);
}
MSH_CMD_EXPORT(set_baudrate_for_pmc, set pmc baudrate);
