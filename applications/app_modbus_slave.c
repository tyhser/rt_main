/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-21     flybreak     first version
 */

#include <rtthread.h>
#include <string.h>
#include "easyblink.h"
#include "app_modbus_slave.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define SLAVE_ADDR      0x01
#define PORT_NUM        6
#define PORT_BAUDRATE   115200

#define PORT_PARITY     MB_PAR_NONE

#define MB_POLL_THREAD_PRIORITY  10
#define MB_SEND_THREAD_PRIORITY  (RT_THREAD_PRIORITY_MAX - 1)

#define MB_POLL_CYCLE_MS 1
#define MODBUS_POLL_PRI         RT_MAIN_THREAD_PRIORITY - 1
#define APP_THREAD_TIMESLICE    5

extern ebled_t led0;

struct rt_mailbox modbus_ind_mailbox;
static char mailbox_pool[128];

ALIGN(RT_ALIGN_SIZE)
static char modbus_poll_thread_stack[2048];
static struct rt_thread modbus_poll_thread;

static void mb_slave_poll(void *parameter)
{
	if (rt_strstr(parameter, "RTU")) {
#ifdef PKG_MODBUS_SLAVE_RTU
		eMBInit(MB_RTU, SLAVE_ADDR, PORT_NUM, PORT_BAUDRATE,
			PORT_PARITY);
#else
		rt_kprintf("Error: Please open RTU mode first");
#endif
	} else if (rt_strstr(parameter, "ASCII")) {
#ifdef PKG_MODBUS_SLAVE_ASCII
		eMBInit(MB_ASCII, SLAVE_ADDR, PORT_NUM, PORT_BAUDRATE,
			PORT_PARITY);
#else
		rt_kprintf("Error: Please open ASCII mode first");
#endif
	} else if (rt_strstr(parameter, "TCP")) {
#ifdef PKG_MODBUS_SLAVE_TCP
		eMBTCPInit(0);
#else
		rt_kprintf("Error: Please open TCP mode first");
#endif
	} else {
		rt_kprintf("Error: unknown parameter");
	}
	eMBEnable();
	while (1) {
		eMBPoll();
		rt_thread_mdelay(MB_POLL_CYCLE_MS);
	}
}

void app_md_slave_init(void)
{
	rt_err_t result;

	rt_memcpy(&REG_VERSION, GIT_DESC, strlen(GIT_DESC) + 1);

	result = rt_mb_init(&modbus_ind_mailbox,
			    "modbus_ind",
			    &mailbox_pool[0],
			    (rt_size_t) (sizeof(mailbox_pool) / 4),
			    RT_IPC_FLAG_FIFO);

	if (result != RT_EOK) {
		LOG_E("init mailbox failed.");
		easyblink(led0, -1, 200, 400);
	}

	rt_thread_init(&modbus_poll_thread,
		       "modbus_poll",
		       mb_slave_poll,
		       "RTU",
		       &modbus_poll_thread_stack[0],
		       sizeof(modbus_poll_thread_stack),
		       MODBUS_POLL_PRI, APP_THREAD_TIMESLICE);
	rt_thread_startup(&modbus_poll_thread);
}
