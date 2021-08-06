#include <rtthread.h>
#include <rtdevice.h>

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "electrode"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define OFFSET                0.651
#define ELECTRODE_DEV_NAME    "adc0"
#define REFER_VOLTAGE		1250000

#define CONVERT_BITS    ((1<<23) - 1)

static struct {
	rt_adc_device_t adc_dev;
	struct rt_semaphore adc_sem;
} adc_context = { 0 };

extern double get_temperature_by_resistance(uint32_t r);

rt_err_t electrode_init(void)
{
	adc_context.adc_dev =
	    (rt_adc_device_t) rt_device_find(ELECTRODE_DEV_NAME);
	if (adc_context.adc_dev == RT_NULL) {
		LOG_I("electrode measure failed, can't find %s device!",
		      ELECTRODE_DEV_NAME);
		return RT_ERROR;
	} else {
		return RT_EOK;
	}
}

rt_int32_t electrode_get_data(rt_uint32_t ch)
{
	if (adc_context.adc_dev)
		return (rt_int32_t) rt_adc_read(adc_context.adc_dev, ch);
	else
		return 0;
}

float get_resistance_by_adc_data(int64_t data)
{
	return (float)(data * 33000) / (2 * CONVERT_BITS - data);
}

static int electrode_vol_sample(int argc, char *argv[])
{
	rt_adc_device_t adc_dev;
	rt_int64_t value;
	float resistance = 0;
	float temperature = 0;
	rt_err_t ret = RT_EOK;

	adc_dev = (rt_adc_device_t)rt_device_find(ELECTRODE_DEV_NAME);
	if (adc_dev == RT_NULL) {
		LOG_I("electrode measure failed, can't find %s device!",
		      ELECTRODE_DEV_NAME);
		return RT_ERROR;
	}
	if (argc > 1) {
		value = (int)rt_adc_read(adc_dev, argv[1][0] - '0');
		resistance = get_resistance_by_adc_data(value);
		temperature = get_temperature_by_resistance(resistance);

		LOG_I("channel %d resistance: %.3f degree Celsius",
		      (argv[1][0] - '0') + 1, temperature);
	} else {
		value = (int)rt_adc_read(adc_dev, 0);
		resistance = get_resistance_by_adc_data(value);
		temperature = get_temperature_by_resistance(resistance);
		LOG_I("channel 0 resistance: %.3f degree Celsius", temperature);
	}
	return ret;
}

MSH_CMD_EXPORT(electrode_vol_sample, electrode voltage convert sample);

static int electrode_vol_list(int argc, char *argv[])
{
	rt_adc_device_t adc_dev;
	float resistance = 0;
	float temperature = 0;
	rt_int32_t value0;
	rt_err_t ret = RT_EOK;

	adc_dev = (rt_adc_device_t) rt_device_find(ELECTRODE_DEV_NAME);
	if (adc_dev == RT_NULL) {
		LOG_I("electrode measure failed, can't find %s device!",
	      		ELECTRODE_DEV_NAME);
		return RT_ERROR;
	}
	while (1) {
		value0 = (int)rt_adc_read(adc_dev, 0);
		rt_thread_mdelay(5000);
		resistance = get_resistance_by_adc_data(value0);
		temperature = get_temperature_by_resistance(resistance);
		LOG_I("%.3f", temperature);
	}
	return ret;
}
MSH_CMD_EXPORT(electrode_vol_list, electrode voltage convert list);
