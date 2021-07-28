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

#define ROBOT_ADDR		2

#define HOLD_REG_X_AXIS		180
#define HOLD_REG_Y_AXIS		181
#define HOLD_REG_XY_CMD		182

#define HOLD_REG_Z_AXIS		183
#define HOLD_REG_Z_CMD		184

#define HOLD_REG_SYRING		185
#define HOLD_REG_SYRING_CMD	186

enum robot_cmd {
	ROBOT_READY,
	ROBOT_RUN,
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
		LOG_I("valve%d value%d", valve_index, bit_value);

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
	switch (addr) {
	case HOLD_REG_X_AXIS ... HOLD_REG_XY_CMD:
		switch (reg[HOLD_REG_XY_CMD]) {
		case ROBOT_RUN:
			pmc_motor_xy_pose(ROBOT_ADDR, reg[HOLD_REG_X_AXIS], reg[HOLD_REG_Y_AXIS]);
			reg[HOLD_REG_XY_CMD] = ROBOT_READY;
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			reg[HOLD_REG_XY_CMD] = ROBOT_READY;
			break;
		case ROBOT_HOME:
			reg[HOLD_REG_XY_CMD] = ROBOT_READY;
			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		break;
	case HOLD_REG_Z_AXIS ... HOLD_REG_Z_CMD:
		switch (reg[HOLD_REG_Z_CMD]) {
		case ROBOT_FWD:
			pmc_motor_fwd(ROBOT_ADDR, MOTOR_2, reg[HOLD_REG_Z_AXIS]);
			break;
		case ROBOT_RCV:
			pmc_motor_rev(ROBOT_ADDR, MOTOR_2, reg[HOLD_REG_Z_AXIS]);
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		break;
	case HOLD_REG_SYRING ... HOLD_REG_SYRING_CMD:
		switch (reg[HOLD_REG_SYRING_CMD]) {
		case ROBOT_FWD:
			pmc_motor_fwd(ROBOT_ADDR, MOTOR_3, reg[HOLD_REG_SYRING]);
			break;
		case ROBOT_RCV:
			pmc_motor_rev(ROBOT_ADDR, MOTOR_3, reg[HOLD_REG_SYRING]);
			break;
		case ROBOT_STOP:
			pmc_stop(ROBOT_ADDR);
			break;
		case ROBOT_HOME:
			break;
		default:
			LOG_E("Unkow cmd");
			break;
		}
		break;
	default:
		LOG_E("UNKNOW HOLD REG");
		break;
	}
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
