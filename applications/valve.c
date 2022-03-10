#include <board.h>
#include <stdlib.h>
#include "app_modbus_slave.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "switch"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

int valve_map[] = {
	[1] = GET_PIN(E, 15),
	[0] = GET_PIN(B, 10),
	[3] = GET_PIN(E, 13),
	[2] = GET_PIN(E, 14),
	[5] = GET_PIN(E, 11),
	[4] = GET_PIN(E, 12),
	[7] = GET_PIN(E, 9),
	[6] = GET_PIN(E, 10),
	[9] = GET_PIN(E, 7),
	[8] = GET_PIN(E, 8),
	[11] = GET_PIN(B, 0),
	[10] = GET_PIN(B, 1),
	[13] = GET_PIN(C, 4),
	[12] = GET_PIN(C, 5),
	[15] = GET_PIN(A, 6),
	[14] = GET_PIN(A, 7),
	[17] = GET_PIN(A, 4),
	[16] = GET_PIN(A, 5),
	[19] = GET_PIN(A, 2),
	[18] = GET_PIN(A, 3),
	[21] = GET_PIN(A, 0),
	[20] = GET_PIN(A, 1),
	[23] = GET_PIN(C, 2),
	[22] = GET_PIN(C, 3),
	[25] = GET_PIN(B, 7),
	[24] = GET_PIN(B, 8),
	[27] = GET_PIN(B, 5),
	[26] = GET_PIN(B, 6),
	[29] = GET_PIN(B, 3),
	[28] = GET_PIN(B, 4),
	[31] = GET_PIN(D, 6),
	[30] = GET_PIN(D, 7),
	[32] = GET_PIN(B, 12),
	[33] = GET_PIN(B, 13),
	[34] = GET_PIN(B, 15),
	[35] = GET_PIN(D, 11),
	[36] = GET_PIN(D, 12),
	[37] = GET_PIN(D, 13),
	[38] = GET_PIN(D, 14),
	[39] = GET_PIN(D, 15),
};

int sw_input_map[] = {
	[0] = GET_PIN(C, 14),
	[1] = GET_PIN(C, 13),
	[2] = GET_PIN(E, 6),
	[3] = GET_PIN(E, 5),
	[4] = GET_PIN(E, 4),
	[5] = GET_PIN(E, 3),
	[6] = GET_PIN(E, 2),
	[7] = GET_PIN(E, 1),
	[8] = GET_PIN(E, 0),
	[9] = GET_PIN(B, 9),
};

void valve_init(void)
{
	for (int i = 0; i < sizeof(valve_map) / sizeof(valve_map[0]); i++)
		rt_pin_mode(valve_map[i], PIN_MODE_OUTPUT);
}

void set_valve(int id, int val)
{
	if (id < sizeof(valve_map) / sizeof(valve_map[0])) {
		rt_pin_write(valve_map[id], val);
	}
}

void valve_set(int argc, char *argv[])
{
	uint32_t id = 0;
	int value = 0;

	if (argc == 3) {
		id = atol((char *)argv[1]);
		value = argv[2][0] - '0';
		set_valve(id, value);

	} else {
		rt_kprintf("usage: valve_set [num] [1|0]\n");
	}
}
MSH_CMD_EXPORT(valve_set, set valve);

void sw_input_change(void *args)
{
	int value = rt_pin_read(sw_input_map[(int)args]);;

	rt_kprintf("[In:%d]= [%d]\n", (int)args, value);
	if (value) {
		ucSDiscInBuf[(int)args / 8] |= ((uint8_t)1 << ((int)args % 8));
	} else {
		ucSDiscInBuf[(int)args / 8] &= ~((uint8_t)1 << ((int)args % 8));
	}
}

int sw_input_init(void)
{
	int value =  0;

	for (int i = 0; i < sizeof(sw_input_map) / sizeof(sw_input_map[0]); i++) {
		rt_pin_mode(sw_input_map[i], PIN_MODE_INPUT);
		rt_pin_attach_irq(sw_input_map[i], PIN_IRQ_MODE_RISING_FALLING, sw_input_change, (void *)i);
		rt_pin_irq_enable(sw_input_map[i], PIN_IRQ_ENABLE);
		value = rt_pin_read(sw_input_map[i]);

		if (value) {
			ucSDiscInBuf[i / 8] |= ((uint8_t)1 << (i % 8));
		} else {
			ucSDiscInBuf[i / 8] &= ~((uint8_t)1 << (i % 8));
		}
	}
	return 0;
}
