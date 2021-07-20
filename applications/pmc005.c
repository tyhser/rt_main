#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "pmc"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define PMC_BUS_NAME "uart3"
#define PMC_BUS_CONFIG_DEFAULT           \
{                                          \
    BAUD_RATE_9600, /* 9600 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}

static rt_device_t pmc_bus = NULL;

int pmc_write(int addr, char *buf, ssize_t len)
{
	rt_device_write(pmc_bus, 0, buf, len);
}

int pmc_init(void)
{
	rt_err_t err = RT_EOK;
	struct serial_configure config = PMC_BUS_CONFIG_DEFAULT;

	pmc_bus = rt_device_find(PMC_BUS_NAME);
	if (pmc_bus == NULL) {
		LOG_E("pmc bus device didn't found");
		return -1;
	}
	err = rt_device_control(pmc_bus, RT_DEVICE_CTRL_CONFIG, &config);
	if (err != RT_EOK) {
		LOG_E("pmc bus setting failed, err:%d", err);
		return -1
	}
	err = rt_device_open(pmc_bus, RT_DEVICE_FLAG_INT_RX);
	if (err != RT_EOK) {
		LOG_E("pmc bus open failed, err:%d", err);
		return -1
	}
}
