#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "app_modbus_slave.h"
#include "modbus_event.h"
#include "pmc005.h"
#include "easyblink.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "event"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define ROBOT_ADDR		1

#define HOLD_REG_X_AXIS		180
#define HOLD_REG_Y_AXIS		181
#define HOLD_REG_XY_CMD		182

#define HOLD_REG_Z_AXIS		183
#define HOLD_REG_Z_CMD		184

#define HOLD_REG_SYRING		185
#define HOLD_REG_SYRING_CMD	186

#define HOLD_REG_SAMPLER_ADDR	188

struct pmc_pumb {
	uint32_t modbus_addr;
	uint32_t pmc_addr;
	uint8_t pmc_motor_id;
} pmc_pumb_tab[] = {
	{190, 2, 1},
	{191, 3, 1},
	{192, 3, 2},
	{193, 3, 3},
	{194, 4, 1},
	{195, 4, 2},
	{196, 4, 3},
	{197, 5, 1},
	{198, 5, 2},
	{199, 5, 3},
	{200, 6, 1},
	{201, 6, 2},
	{202, 6, 3},
	{203, 7, 1},
	{204, 7, 2},
	{205, 7, 3},
	{206, 8, 1},
	{207, 8, 2},
	{208, 8, 3},
	{209, 9, 1},
	{210, 9, 2},
	{211, 9, 3},
	{212, 10, 1},
	{213, 11, 2},
	{214, 12, 3},
};

#define SYRING_LEAD_UL		334.8309
#define SYRING_SUB_PULSE	16
#define Z_LEAD_MM		18.8495
#define Z_SUB_PULSE		16
#define X_SUB_PULSE		16
#define Y_SUB_PULSE		16
#define X_LEAD_MM		21.2057
#define Y_LEAD_MM		14.1371
#define X_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * X_SUB_PULSE * 200 / (X_LEAD_MM * 10))
#define Y_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * Y_SUB_PULSE * 200 / (Y_LEAD_MM * 10))
#define Z_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * Z_SUB_PULSE * 200 / (Z_LEAD_MM * 10))
#define SYRING_PULSE(ul) (int32_t)((float)(ul) * SYRING_SUB_PULSE * 200 / (SYRING_LEAD_UL))
#define X_AXIS_LENGTH		3350
#define Y_AXIS_LENGTH		1835
#define Z_AXIS_LENGTH		1055
#define SYRING_LENGTH		10500

enum robot_cmd {
	ROBOT_READY,
	ROBOT_ABS,
	ROBOT_STOP,
	ROBOT_HOME,
	ROBOT_FWD,
	ROBOT_RCV,
	ROBOT_PUSH_PULL
};

enum robot_status {
	ROBOT_OK,
	ROBOT_ERROR,
	ROBOT_BUSY
};

extern void set_valve(int id, int val);
extern void temperature_control_enable_disable(int val);
extern struct rt_mailbox modbus_ind_mailbox;
static struct rt_thread *event_thread;

extern ebled_t red;
extern ebled_t green;
extern ebled_t yellow;
extern ebled_t beep;

struct pmc_pumb *get_pmc_pumb_struct(uint32_t md_addr)
{
	for (int i = 0; i < sizeof(pmc_pumb_tab) / sizeof(pmc_pumb_tab[0]); i++) {
		if (pmc_pumb_tab[i].modbus_addr == md_addr)
			return &pmc_pumb_tab[i];
	}
	return NULL;
}

void red_alarm_handler(int bit)
{
	if (bit) {
		easyblink(red, 16, 100, 150);
		easyblink(beep, 16, 100, 150);
	} else {
		easyblink_stop(red);
		easyblink_stop(beep);
	}
}

void yellow_alarm_handler(int bit)
{
	if (bit) {
		easyblink(yellow, 16, 250, 300);
		easyblink(beep, 16, 250, 300);
	} else {
		easyblink_stop(yellow);
		easyblink_stop(beep);
	}
}

void green_alarm_handler(int bit)
{
	if (bit) {
		easyblink(green, 16, 500, 1000);
		easyblink(beep, 16, 500, 1000);
	} else {
		easyblink_stop(green);
		easyblink_stop(beep);
	}
}

int modbus_addr_to_pmc_addr(int addr)
{
	if (addr < 48) {
		LOG_E("invalid modbus addr");
		return 0;
	}
	/*sampler pmc addr from 3, deliver addr is 2*/
	return (2 + (addr - 48) / 2);
}

int find_valve_modbus_addr_seat(int addr)
{
	static int sampler_modbus_start_addr = 48;
	static int valve_num_per_pmc = 2;
	int pmc_valve_start_addr = 0;

	if (addr < sampler_modbus_start_addr) {
		LOG_E("invalid modbus addr");
		return 0;
	}

	pmc_valve_start_addr = (addr - sampler_modbus_start_addr) / valve_num_per_pmc;
	if (pmc_valve_start_addr == addr)
		return 0;
	else
		return 1;
}

void md_coil_write_handle(uint32_t addr, ssize_t cnt, uint8_t *reg)
{
	//LOG_HEX("coil", 16, reg, 64);
	uint16_t reg_index = (uint16_t)(addr - 16) / 8;
	uint16_t reg_bit_index = (uint16_t)(addr - 16) % 8;
	uint8_t bit_value = 0;
	uint8_t valve_index = 0;
	uint8_t deliver_valve_value = 0;

#define COIL_REG_INDEX(md_addr, offset) ((((md_addr) + (offset)) - S_COIL_START) / 8)
#define COIL_REG_BIT_INDEX(md_addr, offset) (((md_addr) + (offset) - S_COIL_START) % 8)
#define COIL_VALUE(reg_index, reg_bit_index) ((reg[(reg_index)] & (0x01 << (reg_bit_index))) \
						>> (reg_bit_index))
#define COIL_ADDR_VALUE(md_addr, offset) ((reg[COIL_REG_INDEX((md_addr), (offset))] & (0x01 << (COIL_REG_BIT_INDEX((md_addr), (offset))))) \
					>> (COIL_REG_BIT_INDEX((md_addr), (offset))))

	for (int i = 0; i < cnt; i++) {
		reg_index	= COIL_REG_INDEX(addr, i);
		reg_bit_index	= COIL_REG_BIT_INDEX(addr, i);
		bit_value	= COIL_VALUE(reg_index, reg_bit_index);
		valve_index	= addr - 16 + i;

		set_valve(valve_index, bit_value);
		//LOG_I("valve%d value%d", valve_index, bit_value);

		if (valve_index == 40)
			red_alarm_handler(bit_value);
		if (valve_index == 41)
			green_alarm_handler(bit_value);
		if (valve_index == 42)
			yellow_alarm_handler(bit_value);

		if (valve_index == 43)
			temperature_control_enable_disable(bit_value);

		/*for deliver*/
		if (valve_index >= 44 && valve_index <= 47) {
			deliver_valve_value =
			((uint8_t)COIL_ADDR_VALUE(44+16, 0)) |
			((uint8_t)COIL_ADDR_VALUE(44+16, 1) << 1) |
			((uint8_t)COIL_ADDR_VALUE(44+16, 2) << 2)|
			((uint8_t)COIL_ADDR_VALUE(44+16, 3) << 3);

			deliver_set_valve(2, deliver_valve_value);
		}
		if ((valve_index >= 48) && (valve_index <= 63)) {
			uint8_t station_addr = modbus_addr_to_pmc_addr(valve_index);
			uint16_t reg_index_friend = 0;
			uint16_t reg_bit_friend_index = 0;
			uint8_t friend_bit_value = 0;
			uint8_t v1 = 0;
			uint8_t v2 = 0;

			if (find_valve_modbus_addr_seat(valve_index) == 0) {
				reg_index_friend	= COIL_REG_INDEX(addr, i + 1);
				reg_bit_friend_index	= COIL_REG_BIT_INDEX(addr, i + 1);
				friend_bit_value	= COIL_VALUE(reg_index_friend, reg_bit_friend_index);
				v1 = bit_value;
				v2 = friend_bit_value;
			} else if (find_valve_modbus_addr_seat(valve_index) == 1) {
				reg_index_friend	= COIL_REG_INDEX(addr, i - 1);
				reg_bit_friend_index	= COIL_REG_BIT_INDEX(addr, i - 1);
				friend_bit_value	= COIL_VALUE(reg_index_friend, reg_bit_friend_index);
				v1 = friend_bit_value;
				v2 = bit_value;
			} else {
			}
			pmc_set_valve(station_addr, (v1 & 0x01) | ((v2 & 0x01) << 1));
		}
	}
#undef COIL_REG_INDEX
#undef COIL_REG_BIT_INDEX
#undef COIL_VALUE
}

void print_hold_reg(uint32_t addr, ssize_t cnt, uint16_t *reg)
{
	rt_kprintf("\nwrite: ");
	for (int i = addr; i < addr + cnt; i++) {
		rt_kprintf("%d:%d ", i, reg[(i) - S_REG_HOLDING_START]);
	}
	rt_kprintf("\n");
}

/*
 * holding reg:
 * 180 181 182
 *  |   |   |
 * [x,  y, cmd] xy axis
 * -------------
 * 183   184
 *  |     |
 * [P/D, cmd] z axis
 * ------------
 * 185   186
 *  |     |
 * [P/D, cmd] syring
 */
void md_hold_reg_write_handle(uint32_t addr, ssize_t cnt, uint16_t *reg)
{
	int32_t current_position = 0;

	//LOG_I("hold reg: addr:%d cnt:%d reg:%p value0x%x", addr, cnt, reg, reg[addr]);
	print_hold_reg(addr, cnt, reg);
#define REG_VALUE(mb_addr) (reg[(mb_addr) - S_REG_HOLDING_START])
	switch (addr) {
	case HOLD_REG_X_AXIS ... HOLD_REG_XY_CMD:
		switch (REG_VALUE(HOLD_REG_XY_CMD)) {
		case ROBOT_ABS:
			pmc_motor_z_abs(ROBOT_ADDR, 0);
			if (REG_VALUE(HOLD_REG_X_AXIS) > X_AXIS_LENGTH) {
				LOG_W("X axis move over length");
				break;
			}
			if (REG_VALUE(HOLD_REG_Y_AXIS) > Y_AXIS_LENGTH) {
				LOG_W("Y axis move over length");
				break;
			}
			pmc_motor_xy_abs(ROBOT_ADDR,
					X_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS)),
					Y_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS)));
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			pmc_motor_z_abs(ROBOT_ADDR, 0);
			if (REG_VALUE(HOLD_REG_X_AXIS) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_1);
			if (REG_VALUE(HOLD_REG_Y_AXIS) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_2);
			break;
		case ROBOT_READY:
			break;
		case ROBOT_FWD:
			pmc_motor_z_abs(ROBOT_ADDR, 0);
			pmc_select_motor(MOTOR_1, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if (X_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS)) + current_position
					> X_AXIS_PULSE(X_AXIS_LENGTH))
				LOG_W("X axis move over length");
			else
				pmc_motor_fwd(ROBOT_ADDR, MOTOR_1, X_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS)));

			pmc_select_motor(MOTOR_2, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if (Y_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS)) + current_position
					> Y_AXIS_PULSE(Y_AXIS_LENGTH))
				LOG_W("Y axis move over length");
			else
				pmc_motor_fwd(ROBOT_ADDR, MOTOR_2, Y_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS)));
			break;
		case ROBOT_RCV:
			pmc_motor_z_abs(ROBOT_ADDR, 0);
			pmc_select_motor(MOTOR_1, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if ((current_position - X_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS))) < 0)
				LOG_W("X axis invalid position");
			else
				pmc_motor_rev(ROBOT_ADDR, MOTOR_1, X_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS)));

			pmc_select_motor(MOTOR_2, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if ((current_position - Y_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS))) < 0)
				LOG_W("Y axis invalid position");
			else
				pmc_motor_rev(ROBOT_ADDR, MOTOR_2, Y_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS)));

			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		REG_VALUE(HOLD_REG_XY_CMD) = ROBOT_READY;
		REG_VALUE(HOLD_REG_X_AXIS) = 0;
		REG_VALUE(HOLD_REG_Y_AXIS) = 0;
		//REG_VALUE(HOLD_REG_Z_AXIS) = 0;
		break;
	case HOLD_REG_Z_AXIS ... HOLD_REG_Z_CMD:
		switch (REG_VALUE(HOLD_REG_Z_CMD)) {
		case ROBOT_FWD:
			pmc_select_motor(MOTOR_3, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if ((current_position + Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS)))
					> Z_AXIS_PULSE(Z_AXIS_LENGTH))
				LOG_W("Z axis over length");
			else
				pmc_motor_fwd(ROBOT_ADDR, MOTOR_3, Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS)));
			break;
		case ROBOT_RCV:
			pmc_select_motor(MOTOR_3, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if ((current_position - Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS))) < 0)
				LOG_W("Z axis invalid position");
			else
				pmc_motor_rev(ROBOT_ADDR, MOTOR_3, Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS)));
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			if (REG_VALUE(HOLD_REG_Z_AXIS) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_3);
			break;
		case ROBOT_READY:
			break;
		case ROBOT_ABS:
			if (REG_VALUE(HOLD_REG_Z_AXIS) > Z_AXIS_LENGTH)
				LOG_W("Z axis move over length");
			else
				pmc_motor_z_abs(ROBOT_ADDR, Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS)));

			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		REG_VALUE(HOLD_REG_Z_AXIS) = 0;
		REG_VALUE(HOLD_REG_Z_CMD) = ROBOT_READY;
		break;
	case HOLD_REG_SYRING ... HOLD_REG_SYRING_CMD:
		switch (REG_VALUE(HOLD_REG_SYRING_CMD)) {
		case ROBOT_ABS:
			if (REG_VALUE(HOLD_REG_SYRING) > SYRING_LENGTH)
				LOG_W("Syring move over length");
			else
				pmc_motor_syring_abs(ROBOT_ADDR, SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)));
			break;
		case ROBOT_FWD:
			pmc_select_motor(MOTOR_4, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if (SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)) + current_position
					> SYRING_PULSE(SYRING_LENGTH))
				LOG_W("Syring move over length");
			else
				pmc_motor_fwd(ROBOT_ADDR, MOTOR_4, SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)));
			break;
		case ROBOT_RCV:
			pmc_select_motor(MOTOR_4, ROBOT_ADDR);
			current_position = pmc_get_current_motor_position();

			if (current_position - SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)) < 0)
				LOG_W("Syring invalid position");
			else
				pmc_motor_rev(ROBOT_ADDR, MOTOR_4, SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)));
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			if (REG_VALUE(HOLD_REG_SYRING) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_4);
			break;
		case ROBOT_READY:
			break;
		case ROBOT_PUSH_PULL:
			pmc_robot_syring_pp(ROBOT_ADDR, REG_VALUE(HOLD_REG_SYRING));
			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		REG_VALUE(HOLD_REG_SYRING_CMD) = ROBOT_READY;
		REG_VALUE(HOLD_REG_SYRING) = 0;
		break;
		/*pmc pumb speed control*/
	case 190 ... 214:
	{
		struct pmc_pumb *pmc_pumb = get_pmc_pumb_struct(addr);
		pmc_motor_speed_mode(pmc_pumb->pmc_addr, pmc_pumb->pmc_motor_id, REG_VALUE(addr));
	}
		break;
	default:
		LOG_E("UNKNOW HOLD REG");
		break;
	}
#undef REG_ARRAY_INDEX
}

static void event_thread_entry(void *parameter)
{
	struct md_event *event = NULL;
	enum md_cmd_type type = MD_NONE;
	uint32_t addr = 0;
	uint16_t *reg = NULL;
	size_t cnt = 0;

	while (1) {
		event = md_event_recv();
		if (event == NULL)
			continue;
		type = md_event_get_cmd_type(event);
		addr = md_event_get_start_addr(event);
		reg = md_event_get_reg_pointer(event);
		cnt = md_event_get_reg_cnt(event);

		switch (type) {
		case MD_COIL:
			md_coil_write_handle(addr, cnt, (uint8_t *)reg);
			break;
		case MD_HOLDING_REG:
			md_hold_reg_write_handle(addr, cnt, (uint16_t *)reg);
			break;
		default:
			break;
		}
		md_event_release_msg(event);
	}
}

rt_err_t event_init(void)
{
	rt_err_t ret = RT_EOK;

	event_thread = rt_thread_create("event",
			event_thread_entry,
			RT_NULL,
			3072,
			15, 5);
	rt_thread_startup(event_thread);
	return ret;
}
