#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "electrode.h"
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

extern struct rt_mailbox modbus_ind_mailbox;
static struct rt_thread *event_thread;

void reg_work_state_notify(void)
{
	rt_enter_critical();
	REG_WORK_STATE = 0;
	rt_exit_critical();
}

md_coil_write_handle(uint32_t addr, ssize_t cnt, uint16_t *reg)
{
	LOG_I("D");
	for (int i = 0; i < cnt; i++) {
		rt_kprintf("");
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
			md_coil_write_handle(addr, cnt, reg);
			break;
		case MD_HOLDING_REG:
			break;
		default:
			break;
		}
		md_event_release_msg(event);
	}
}

rt_err_t reactor_init(void)
{
	rt_err_t ret = RT_EOK;

	ret = electrode_init();
	if (!electrode_get_data(0))
		ret = RT_ERROR;

	rt_list_init(&reactor.raw_data_list);
	reactor.state = READY;

	reactor_thread = rt_thread_create("reactor",
		       reactor_thread_entry,
		       RT_NULL,
		       3072,
		       REACTOR_PRI, APP_THREAD_TIMESLICE);
	rt_thread_startup(reactor_thread);
	return ret;
}
