#include <rtthread.h>
#if defined(BSP_USING_EXTERNED_ADC) && defined(RT_USING_ADC)
#include "cs1237.h"
#include "drivers/adc.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "exdevice.adc"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

struct ex_adc {
	struct rt_adc_device ex_adc_device;
};

static struct ex_adc ex_adc_obj;

static rt_err_t ex_adc_enabled(struct rt_adc_device *device,
			       rt_uint32_t channel, rt_bool_t enabled)
{
	RT_ASSERT(device != RT_NULL);

	return RT_EOK;
}

static rt_err_t ex_get_adc_value(struct rt_adc_device *device,
				 rt_uint32_t channel, rt_uint32_t *value)
{
	RT_ASSERT(device != RT_NULL);
	RT_ASSERT(value  != RT_NULL);

	if (channel > 1)
		LOG_E("ADC channel not supply!");

	*value = cs1237_convert(channel);
	return RT_EOK;
}

static const struct rt_adc_ops ex_adc_ops = {
	.enabled = ex_adc_enabled,
	.convert = ex_get_adc_value,
};

static int ex_adc_init(void)
{
	int result = RT_EOK;
	/* save adc name */
	char name_buf[5] = { 'a', 'd', 'c', '0', 0 };
	/* ADC init */
	if (cs1237_init() != 0) {
		LOG_E("%s init failed", name_buf);
		result = -RT_ERROR;
	} else {
		/* register ADC device */
		if (rt_hw_adc_register
		    (&(ex_adc_obj.ex_adc_device), name_buf, &ex_adc_ops,
		     NULL) == RT_EOK) {
			LOG_D("%s init success", name_buf);
		} else {
			LOG_E("%s register failed", name_buf);
			result = -RT_ERROR;
		}
	}

	return result;
}
INIT_BOARD_EXPORT(ex_adc_init);

#endif /* BSP_USING_EXTERN_ADC */
