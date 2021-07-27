#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdio.h>
#include "rs485.h"
#include "pmc005.h"
#include "app_modbus_slave.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "pmc"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define PMC_SERIAL	"uart3"
#define PMC_BAUDRATE	9600
#define PMC_PARITY	0
#define PMC_MODE_CONTROL_PIN 58
#define PMC_LVL		1

extern uint16_t usSRegInBuf[S_REG_INPUT_NREGS];

#define REG_XY_AXIS_STATE		(usSRegInBuf[0])
#define REG_Z_AXIS_STATE		(usSRegInBuf[1])
#define REG_SYRING_STATE		(usSRegInBuf[2])

#define REG_PMC_STATE			(usSRegInBuf[3])

#define PMC_BUSY	1
#define PMC_FREE	0

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
	char cmd[4];
	uint32_t data;
};

static uint8_t buf[256];
static rs485_inst_t *hinst = NULL;

int pmc_send_then_recv(char *read_cmd, int len, void *buf, int recv_size)
{
	int recv_len = rs485_send_then_recv(hinst, (void *)read_cmd, len, buf, recv_size);
        if (recv_len < 0) {
		LOG_E("rs485 send datas error.");
		return len;
        }

        if (recv_len == 0) {
		LOG_D("rs485 recv timeout.");
		return 0;
        }
        
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

int pmc_make_cmd_line(struct cmd_line_info *cmd_info, char *cmd_line, int len)
{
	char data_ch[25] = {0};

	if (len < 5) {
		LOG_E("cmd line buffer too small");
		return 0;
	}

	*cmd_line = '/';
	*(cmd_line + 1) = get_hex_ch(cmd_info->addr);
	rt_memcpy(cmd_line + 2, cmd_info->cmd, strlen(cmd_info->cmd));
	sprintf(data_ch, "%lu", cmd_info->data);
	rt_memcpy(cmd_line + strlen(cmd_line), data_ch, strlen(data_ch));
	*(cmd_line + strlen(cmd_line)) = 'R';
	*(cmd_line + strlen(cmd_line)) = '\r';
	*(cmd_line + strlen(cmd_line)) = '\0';
	return strlen(cmd_line);
}

void pmc_select_motor(enum motor_id id, int station_addr)
{
	char cmd[25] = {0};
	char recv[128] = {0};
	struct cmd_line_info cmd_info = {
		.addr = station_addr,
		.cmd = "aM",
		.data = id + 1,
	};

	pmc_make_cmd_line(&cmd_info, cmd, 25);
	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
}

int pmc_motor_absolute_position(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	char cmd[25] = {0};
	char recv[128] = {0};
	struct cmd_line_info cmd_line = {
		.addr = station_addr,
		.cmd = "A",
		.data = pos,
	};

	pmc_make_cmd_line(&cmd_line, cmd, 25);

	pmc_select_motor(motor_id, station_addr);
	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	return 0;
}

int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	char cmd[25] = {0};
	char recv[128] = {0};
	struct cmd_line_info cmd_info = {
		.addr = station_addr,
		.cmd = "P",
		.data = pos,
	};

	pmc_make_cmd_line(&cmd_info, cmd, 25);

	pmc_select_motor(motor_id, station_addr);
	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	return 0;
}

enum pmc_status get_pmc_status(char *recv, int len)
{
	if (len < 3) {
		LOG_E("payload too short");
		return 0;
	}
	return (*(recv + 2) & 0x0F);
}

void pmc_update_motor_state(char *recv, int len)
{
	if (len < 4) {
		LOG_E("payload too short");
		return;
	}

	rt_enter_critical();

	if ((*(recv + 3) & 0x01) || (*(recv + 3) & 0x02))
		REG_XY_AXIS_STATE = PMC_BUSY;
	else
		REG_XY_AXIS_STATE = PMC_FREE;
	if (*(recv + 3) & 0x04)
		REG_Z_AXIS_STATE = PMC_BUSY;
	else
		REG_Z_AXIS_STATE = PMC_FREE;
	if (*(recv + 3) & 0x08)
		REG_SYRING_STATE = PMC_BUSY;
	else
		REG_SYRING_STATE = PMC_FREE;

	REG_PMC_STATE = get_pmc_status(recv, strlen(recv));

	rt_exit_critical();
}

void pmc_stop(uint8_t station_addr)
{
	char cmd[128] = {0};
	char recv[128] = {0};

	cmd[0] = '/';
	cmd[1] = get_hex_ch(station_addr);
	cmd[2] = 'T';
	cmd[3] = 'R';
	cmd[4] = '\r';

	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	REG_PMC_STATE = get_pmc_status(recv, strlen(recv));
}

int pmc_is_robot_busy(uint8_t station_addr, enum axis_id id)
{
	char cmd[128] = {0};
	char recv[128] = {0};

	cmd[0] = '/';
	cmd[1] = get_hex_ch(station_addr);
	cmd[2] = '?';
	cmd[3] = 'a';
	cmd[4] = 'S';
	cmd[5] = '\r';

	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	pmc_update_motor_state(recv, strlen(recv));
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

void pmc_motor_xy_pose(uint8_t station_addr, uint16_t x, uint16_t y)
{
	char num_str[25] = {0};
	char cmd[128] = {0};
	char recv[128] = {0};
	char *cmd_pos = &cmd[0];

	*(cmd_pos + strlen(cmd_pos)) = '/';
	*(cmd_pos + strlen(cmd_pos)) = get_hex_ch(station_addr);
	rt_memcpy(cmd_pos + strlen(cmd_pos), "aM1", strlen("aM1"));
	*(cmd_pos + strlen(cmd_pos)) = 'B';
	sprintf(num_str, "%u", x);
	rt_memcpy(cmd_pos + strlen(cmd_pos), num_str, strlen(num_str));
	rt_memset(num_str, 0, 25);

	rt_memcpy(cmd_pos + strlen(cmd_pos), "aM2", strlen("aM2"));
	*(cmd_pos + strlen(cmd_pos)) = 'B';
	sprintf(num_str, "%u", y);
	rt_memcpy(cmd_pos + strlen(cmd_pos), num_str, strlen(num_str));
	*(cmd_pos + strlen(cmd_pos)) = 'R';
	*(cmd_pos + strlen(cmd_pos)) = '\r';

	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	pmc_update_motor_state(recv, strlen(recv));
	for (int i = 0; i < 100; i++) {
		if (!pmc_is_robot_busy(station_addr, XY_AXIS))
			break;
		rt_thread_mdelay(300);
	}
}

int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos)
{
	char cmd[25] = {0};
	char recv[128] = {0};
	struct cmd_line_info cmd_info = {
		.addr = station_addr,
		.cmd = "D",
		.data = pos,
	};

	pmc_make_cmd_line(&cmd_info, cmd, 25);

	pmc_select_motor(motor_id, station_addr);
	pmc_send_then_recv(cmd, strlen(cmd), recv, 128);
	return 0;
}

void PMC(int argc, char *argv[])
{
	char cmd[128] = {0};
	int recv_len = 0;

	rt_memcpy(&cmd[0], &argv[1][0], strlen(argv[1]) + 1);
	cmd[strlen(argv[1])] = '\r';
	cmd[strlen(argv[1]) + 1] = '\n';
	recv_len  = pmc_send_then_recv(cmd, strlen(cmd), buf, 128);
	LOG_HEX("pmc recv", 16, buf, recv_len);
	rt_memset(buf, 0, 256);
}
MSH_CMD_EXPORT(PMC, test pmc);

int pmc_init(void)
{
	hinst = rs485_create(PMC_SERIAL, PMC_BAUDRATE, PMC_PARITY, PMC_MODE_CONTROL_PIN, PMC_LVL);
	if (hinst == RT_NULL) {
        	LOG_E("create rs485 instance fail.");
        	return -1;
    	}
	rs485_set_recv_tmo(hinst, 1000);
    	if (rs485_connect(hinst) != RT_EOK) {
		rs485_destory(hinst);
		LOG_E("rs485 connect fail.");
		return -1;
    	}
	return 0;
}
