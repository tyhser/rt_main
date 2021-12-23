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
#include "modbus_event.h"
#include "easyblink.h"
#include "digital_ctrl.h"
#include "dac121s101.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

/* defined the LED0 pin: PB1 */
#define LED0	GET_PIN(B, 14)
#define RED	GET_PIN(D, 15)
#define YELLOW	GET_PIN(D, 13)
#define GREEN	GET_PIN(D, 14)
#define BEEP	GET_PIN(C, 2)
#define PC_ON	GET_PIN(B, 11)

extern rt_err_t event_init(void);
extern void valve_init(void);
extern int sw_input_init(void);

void pc_on_off(void)
{

	rt_pin_mode(PC_ON, PIN_MODE_OUTPUT);
	rt_pin_write(PC_ON, PIN_HIGH);
	rt_thread_mdelay(300);
	rt_pin_write(PC_ON, PIN_LOW);
}
MSH_CMD_EXPORT(pc_on_off, pc on off);

ebled_t led0	= RT_NULL;
ebled_t red	= RT_NULL;
ebled_t green	= RT_NULL;
ebled_t yellow	= RT_NULL;
ebled_t beep	= RT_NULL;

int main(void)
{
	led0	= easyblink_init_led(LED0, PIN_HIGH);
	red	= easyblink_init_led(RED, PIN_HIGH);
	green	= easyblink_init_led(GREEN, PIN_HIGH);
	yellow	= easyblink_init_led(YELLOW, PIN_HIGH);
	beep	= easyblink_init_led(BEEP, PIN_HIGH);
	easyblink(led0, -1, 500, 1000);

	app_md_slave_init();
	md_event_init();
	event_init();
	valve_init();
	pmc_init();
	rt_kprintf("application version: "GIT_DESC"\n");
	pc_on_off();
	sw_input_init();
	DAC121S101_Init();

	DAC121S101_WriteDAC(1, PD_OPERATION_NORMAL, 700);
	while (1) {
		if(!__EASYBLINK_IS_FLAG(led0, PKG_EASYBLINK_ACTIVE))
			easyblink(led0, -1, 500, 1000);
		rt_thread_mdelay(1000);
	}
	return RT_EOK;
}
