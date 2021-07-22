#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "app_modbus_slave.h"
#include "modbus_event.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "event"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

enum modbus_cmd {
	MD_CMD_DISCRETE,
	MD_CMD_COIL,
	MD_CMD_INPUT,
	MD_CMD_HOLD,

	MD_CMD_ERROR,
};

enum control_cmd {
	CMD_VALVE,
	CMD_MOVING_PLANE,
};

extern void set_valve(int id, int val);
extern struct rt_mailbox modbus_ind_mailbox;
static struct rt_thread *event_thread;

void md_coil_write_handle(uint32_t addr, ssize_t cnt, uint8_t *reg)
{
	//LOG_HEX("coil", 16, reg, 64);
	uint16_t reg_index = (uint16_t)(addr - 16) / 8;
	uint16_t reg_bit_index = (uint16_t)(addr - 16) % 8;

	for (int i = 0; i < cnt; i++) {
		reg_index = (i + addr - 16) / 8;
		reg_bit_index = (i + addr - 16) % 8;
		set_valve(addr - 16 + i, (reg[reg_index] & (0x01 << reg_bit_index)) >> reg_bit_index);
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
