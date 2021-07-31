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

#define SYRING_LEAD_UL		164.388
#define SYRING_SUB_PULSE	16
#define Z_LEAD_MM		4
#define Z_SUB_PULSE		16
#define XY_SUB_PULSE		32
#define XY_LEAD_MM		48
#define XY_AXIS_PULSE(mm_10) ((mm_10) * XY_SUB_PULSE * 200 / (XY_LEAD_MM * 10))
#define Z_AXIS_PULSE(mm_10) ((mm_10) * Z_SUB_PULSE * 200 / (Z_LEAD_MM * 10))
#define SYRING_PULSE(ul) ((float)(ul) * SYRING_SUB_PULSE * 200 / (SYRING_LEAD_UL))

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
extern struct rt_mailbox modbus_ind_mailbox;
static struct rt_thread *event_thread;

extern ebled_t red;
extern ebled_t green;
extern ebled_t yellow;
extern ebled_t beep;

void md_coil_write_handle(uint32_t addr, ssize_t cnt, uint8_t *reg)
{
	//LOG_HEX("coil", 16, reg, 64);
	uint16_t reg_index = (uint16_t)(addr - 16) / 8;
	uint16_t reg_bit_index = (uint16_t)(addr - 16) % 8;
	uint8_t bit_value = 0;
	uint8_t valve_index = 0;

	for (int i = 0; i < cnt; i++) {
		reg_index = (i + addr - 16) / 8;
		reg_bit_index = (i + addr - 16) % 8;
		bit_value = (reg[reg_index] & (0x01 << reg_bit_index)) >> reg_bit_index;
		valve_index = addr - 16 + i;

		set_valve(valve_index, bit_value);
		//LOG_I("valve%d value%d", valve_index, bit_value);

		if (valve_index == 41) {
			LOG_I("valve 41");
			if (bit_value) {
				easyblink(red, 16, 50, 100);
				easyblink(beep, 16, 50, 100);
			} else {
				easyblink_stop(red);
				easyblink_stop(beep);
			}
		}
		if (valve_index == 42) {
			LOG_I("valve 42");
			if (bit_value) {
				easyblink(yellow, 16, 50, 300);
				easyblink(beep, 16, 50, 300);
			} else {
				easyblink_stop(yellow);
				easyblink_stop(beep);
			}
		}
		if (valve_index == 43) {
			LOG_I("valve 43");
			if (bit_value) {
				easyblink(green, 16, 50, 300);
				easyblink(beep, 16, 50, 300);
			} else {
				easyblink_stop(green);
				easyblink_stop(beep);
			}
		}
	}
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
	LOG_I("hold reg: addr:%d cnt:%d reg:%p value0x%x", addr, cnt, reg, reg[addr]);
#define REG_VALUE(mb_addr) (reg[(mb_addr) - S_REG_HOLDING_START])
	switch (addr) {
	case HOLD_REG_X_AXIS ... HOLD_REG_XY_CMD:
		switch (REG_VALUE(HOLD_REG_XY_CMD)) {
		case ROBOT_ABS:
			pmc_motor_xy_abs(ROBOT_ADDR, XY_AXIS_PULSE(REG_VALUE(HOLD_REG_X_AXIS)), XY_AXIS_PULSE(REG_VALUE(HOLD_REG_Y_AXIS)));
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			if (REG_VALUE(HOLD_REG_X_AXIS) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_1);
			if (REG_VALUE(HOLD_REG_Y_AXIS) == 0)
				pmc_motor_home(ROBOT_ADDR, MOTOR_2);
			break;
		case ROBOT_READY:
			break;
		case ROBOT_FWD:
			pmc_motor_fwd(ROBOT_ADDR, REG_VALUE(HOLD_REG_X_AXIS), MOTOR_1);
			pmc_motor_fwd(ROBOT_ADDR, REG_VALUE(HOLD_REG_Y_AXIS), MOTOR_2);
			break;
		case ROBOT_RCV:
			pmc_motor_rev(ROBOT_ADDR, REG_VALUE(HOLD_REG_X_AXIS), MOTOR_1);
			pmc_motor_rev(ROBOT_ADDR, REG_VALUE(HOLD_REG_Y_AXIS), MOTOR_2);
			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		REG_VALUE(HOLD_REG_XY_CMD) = ROBOT_READY;
		REG_VALUE(HOLD_REG_X_AXIS) = 0;
		REG_VALUE(HOLD_REG_Y_AXIS) = 0;
		break;
	case HOLD_REG_Z_AXIS ... HOLD_REG_Z_CMD:
		switch (REG_VALUE(HOLD_REG_Z_CMD)) {
		case ROBOT_FWD:
			pmc_motor_fwd(ROBOT_ADDR, MOTOR_3, Z_AXIS_PULSE(REG_VALUE(HOLD_REG_Z_AXIS)));
			break;
		case ROBOT_RCV:
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
		case ROBOT_FWD:
			pmc_motor_fwd(ROBOT_ADDR, MOTOR_4, SYRING_PULSE(REG_VALUE(HOLD_REG_SYRING)));
			break;
		case ROBOT_RCV:
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
