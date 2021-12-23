#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include "dac121s101.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "DAC"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define DAC121S101_SYNC_0 GET_PIN(A, 15)
#define DAC121S101_SYNC_1 GET_PIN(D, 0)
#define DAC121S101_SYNC_2 GET_PIN(D, 1)
#define DAC121S101_SCLK   GET_PIN(C, 10)
#define DAC121S101_DIN    GET_PIN(C, 12)
#define DAC121S101_DELAY() delay_us(1)

#define DAC121S101_SCLK_HIGH() rt_pin_write(DAC121S101_SCLK, 1)
#define DAC121S101_SCLK_LOW() rt_pin_write(DAC121S101_SCLK, 0)
#define DAC121S101_DIN_HIGH() rt_pin_write(DAC121S101_DIN, 1)
#define DAC121S101_DIN_LOW() rt_pin_write(DAC121S101_DIN, 0)

void delay_us(uint32_t nus)
{
	uint32_t delay = nus * 168 / 4;

	do {
		__NOP();
	} while (delay--);
}

void DAC121S101_Init(void)
{
	rt_pin_mode(DAC121S101_SYNC_0, PIN_MODE_OUTPUT);
	rt_pin_mode(DAC121S101_SYNC_1, PIN_MODE_OUTPUT);
	rt_pin_mode(DAC121S101_SYNC_2, PIN_MODE_OUTPUT);
	rt_pin_mode(DAC121S101_SCLK, PIN_MODE_OUTPUT);
	rt_pin_mode(DAC121S101_DIN, PIN_MODE_OUTPUT);
	rt_pin_write(DAC121S101_SYNC_0, 1);
	rt_pin_write(DAC121S101_SYNC_1, 1);
	rt_pin_write(DAC121S101_SYNC_2, 1);
}

//12 BIT DA
int DAC121S101_WriteDAC(int addr, unsigned char op, unsigned short DAValue)
{
	int i = 0;
	int sync;

	switch (addr) {
	case 0:
		sync = DAC121S101_SYNC_0;
	break;
	case 1:
		sync = DAC121S101_SYNC_1;
	break;
	case 2:
		sync = DAC121S101_SYNC_2;
	break;
	default:
		LOG_E("dac address unknow");
	break;
	}

	unsigned short Command = (op & 0x03);
	Command <<= 12;
	Command |= (DAValue & 0x0fff);

	rt_pin_write(sync, 1);
	DAC121S101_DELAY();

	for (i = 0; i < 16; i++) {
		if (Command & 0x8000) {
			DAC121S101_DIN_HIGH();
		} else {
			DAC121S101_DIN_LOW();
		}
		DAC121S101_SCLK_HIGH();
		DAC121S101_DELAY();
		Command <<= 1;
		DAC121S101_SCLK_LOW();
		DAC121S101_DELAY();
	}

	rt_pin_write(sync, 0);
	DAC121S101_DELAY();
	DAC121S101_SCLK_LOW();
	DAC121S101_DELAY();
	DAC121S101_DIN_LOW();
	DAC121S101_DELAY();
	return 0;
}

void dac_set(int argc, char *argv[])
{
	uint32_t addr = 0;
	uint16_t value = 0;

	if (argc == 3) {
		addr = atol((char *)argv[1]);
		value = atol((char *)argv[2]);
		DAC121S101_WriteDAC(addr, 0, value);
	} else {
		rt_kprintf("usage: dac_set [addr:0|1|2] value");
	}
}
MSH_CMD_EXPORT(dac_set, set dac);
