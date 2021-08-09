#include <rtthread.h>
#include <rtdevice.h>
#include "electrode.h"
#include "digital_ctrl.h"
#include "app_modbus_slave.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "temperature"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define HWTIMER_NAME "timer2"
#define PWM_NAME "pwm2"
#define HWTIMER_CH 3
//#define PERIOD 10000000
#define PERIOD 20000000

#define DC_P -0.05
#define DC_I 0
#define DC_D 0

extern uint16_t usSRegInBuf[S_REG_INPUT_NREGS];

#define REG_TEMPERATURE_L (usSRegInBuf[20])
#define REG_TEMPERATURE_H (usSRegInBuf[21])

struct rt_device_pwm *pwm_dev = NULL;
rt_thread_t temperature_control_thread = NULL;
struct dc_pid pid_info = {0};

void modbus_set_float(float f, uint16_t *dest)
{
    uint32_t i;

    rt_memcpy(&i, &f, sizeof(uint32_t));
    dest[0] = (uint16_t)i;
    dest[1] = (uint16_t)(i >> 16);
}

void temperature_contro_entry(void *parameter)
{
	dc_t out = 0;

	while (1) {
		pid_info.feed_back = electrode_get_temperature();
		//LOG_I("temperature:%0.3f", pid_info.feed_back);
		modbus_set_float(pid_info.feed_back, &REG_TEMPERATURE_L);
		out = dc_pid_calc(&pid_info);
		//LOG_I("Duty:%0.3f%", out * 100);
		//rt_pwm_set(pwm_dev, HWTIMER_CH, PERIOD, PERIOD * out * 100);
		rt_pwm_set(pwm_dev, HWTIMER_CH, PERIOD, PERIOD * out);
		rt_thread_mdelay(200);
	}
}

int temperature_controller_init(void)
{
	pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_NAME);
	if (pwm_dev == RT_NULL) {
		rt_kprintf("pwm init failed!");
		return 0;
	}
	rt_pwm_set(pwm_dev, HWTIMER_CH, PERIOD, 0);
	rt_pwm_enable(pwm_dev, HWTIMER_CH);
	temperature_control_thread  = rt_thread_create("temp_control",
		temperature_contro_entry, NULL, 1024, 20, 10);
	dc_pid_init(&pid_info, 4.0, DC_P, DC_I, DC_D, 100, 0);
	rt_thread_startup(temperature_control_thread);
	return 0;
}
INIT_APP_EXPORT(temperature_controller_init);
