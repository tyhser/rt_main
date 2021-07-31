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
#define REFER_VOLTAGE         900000000

#define CONVERT_BITS    ((1<<23) - 1)

static struct {
	rt_adc_device_t adc_dev;
	struct rt_semaphore adc_sem;
} adc_context = { 0 };

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

float electrode_data_to_voltagemv(int64_t data)
{
	int64_t tmp = (data * REFER_VOLTAGE) / CONVERT_BITS;
	float result = (float)tmp / 1000000;
	return result;
}

int64_t voltagemv_to_electrode_data(float mv)
{
	return (mv * 1000000 * CONVERT_BITS / REFER_VOLTAGE);
}

static int electrode_vol_sample(int argc, char *argv[])
{
	rt_adc_device_t adc_dev;
	rt_int64_t value;
	rt_int64_t vol;
	rt_err_t ret = RT_EOK;

	adc_dev = (rt_adc_device_t)rt_device_find(ELECTRODE_DEV_NAME);
	if (adc_dev == RT_NULL) {
		LOG_I("electrode measure failed, can't find %s device!",
		      ELECTRODE_DEV_NAME);
		return RT_ERROR;
	}
	if (argc > 1) {
		value = (int)rt_adc_read(adc_dev, argv[1][0] - '0');
		vol = (value * REFER_VOLTAGE) / CONVERT_BITS;
		LOG_I("electrode channel %d voltage: %.12fmv",
		      (argv[1][0] - '0') + 1, (float)vol / 1000000);
	} else {
		value = (int)rt_adc_read(adc_dev, 0);
		vol = (value * REFER_VOLTAGE) / CONVERT_BITS;
		LOG_I("electrode channel %d voltage: %.12fmv", 1,
		      (float)vol / 1000000);
	}
	return ret;
}

MSH_CMD_EXPORT(electrode_vol_sample, electrode voltage convert sample);

static int electrode_vol_list(int argc, char *argv[])
{
	rt_adc_device_t adc_dev;
	rt_int32_t value0;
	rt_int32_t value1;
	rt_int32_t value2;
	rt_int32_t value3;
	rt_err_t ret = RT_EOK;

	adc_dev = (rt_adc_device_t) rt_device_find(ELECTRODE_DEV_NAME);
	if (adc_dev == RT_NULL) {
		LOG_I("electrode measure failed, can't find %s device!",
	      		ELECTRODE_DEV_NAME);
		return RT_ERROR;
	}
	while (1) {
		value0 = (int)rt_adc_read(adc_dev, 0);
		rt_thread_mdelay(4);
		value1 = (int)rt_adc_read(adc_dev, 1);
		rt_thread_mdelay(4);
		value2 = (int)rt_adc_read(adc_dev, 2);
		rt_thread_mdelay(4);
		value3 = (int)rt_adc_read(adc_dev, 3);
		rt_thread_mdelay(4);
		rt_kprintf("%d\t%d\t%d\t%d\n", value0, value1, value2, value3);
	}
	return ret;
}
MSH_CMD_EXPORT(electrode_vol_list, electrode voltage convert list);
