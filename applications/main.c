/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_modbus_slave.h"
#include "pmc005.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 14)

int main(void)
{
	int count = 1;
	/* set LED0 pin mode to output */
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	app_md_slave_init();
	pmc_init();
	LOG_I(GIT_DESC);

	while (count++)
	{
	    rt_pin_write(LED0_PIN, PIN_HIGH);
	    rt_thread_mdelay(500);
	    rt_pin_write(LED0_PIN, PIN_LOW);
	    rt_thread_mdelay(500);
	}

	return RT_EOK;
}
