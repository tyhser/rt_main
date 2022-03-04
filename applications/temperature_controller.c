#include <rtthread.h>
#include <rtdevice.h>
#include "electrode.h"
#include "app_modbus_slave.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "temperature"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

void set_valve(int id, int val);
extern uint16_t usSRegInBuf[S_REG_INPUT_NREGS];

#define REG_TEMPERATURE_L (usSRegInBuf[20])
#define REG_TEMPERATURE_H (usSRegInBuf[21])

#define COOLER_ON set_valve(21, 1)
#define COOLER_OFF set_valve(21, 0)

rt_thread_t temperature_control_thread = NULL;
int temperature_control_enable = 0;

void modbus_set_float(float f, uint16_t *dest)
{
    uint32_t i;

    rt_memcpy(&i, &f, sizeof(uint32_t));
    dest[0] = (uint16_t)i;
    dest[1] = (uint16_t)(i >> 16);
}

void temperature_control_enable_disable(int val)
{
	temperature_control_enable = val;
}

void temperature_contro_entry(void *parameter)
{
	float temp = 0;

	COOLER_ON;
	while (1) {
		temp = electrode_get_temperature();
		//LOG_I("temperature:%0.3f", temp);
		modbus_set_float(temp, &REG_TEMPERATURE_L);
		if (temp <= -11) {
			COOLER_OFF;
		}
		if (temp >= -10) {
			if (temperature_control_enable)
				COOLER_ON;
			else
				COOLER_OFF;
		}
		rt_thread_mdelay(300);
	}
}

int temperature_controller_init(void)
{
	temperature_control_thread  = rt_thread_create("temp_control",
		temperature_contro_entry, NULL, 1024, 20, 50);
	rt_thread_startup(temperature_control_thread);
	return 0;
}
INIT_APP_EXPORT(temperature_controller_init);
