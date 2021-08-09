#include <board.h>

int valve_map[] = {
	[0] = GET_PIN(E, 15),
	[1] = GET_PIN(E, 15), /*is for pwm temperature control*/
	[2] = GET_PIN(E, 13),
	[3] = GET_PIN(E, 14),
	[4] = GET_PIN(E, 11),
	[5] = GET_PIN(E, 12),
	[6] = GET_PIN(E, 9),
	[7] = GET_PIN(E, 10),
	[8] = GET_PIN(E, 7),
	[9] = GET_PIN(E, 8),
	[10] = GET_PIN(B, 0),
	[11] = GET_PIN(B, 1),
	[12] = GET_PIN(C, 4),
	[13] = GET_PIN(C, 5),
	[14] = GET_PIN(A, 6),
	[15] = GET_PIN(A, 7),
	[16] = GET_PIN(A, 4),
	[17] = GET_PIN(A, 5),
	[18] = GET_PIN(A, 2),
	[19] = GET_PIN(A, 3),
	[20] = GET_PIN(A, 0),
	[21] = GET_PIN(A, 1),
	[22] = GET_PIN(C, 2),
	[23] = GET_PIN(C, 3),
	[24] = GET_PIN(B, 7),
	[25] = GET_PIN(B, 8),
	[26] = GET_PIN(B, 5),
	[27] = GET_PIN(B, 6),
	[28] = GET_PIN(B, 3),
	[29] = GET_PIN(B, 4),
	[30] = GET_PIN(D, 6),
	[31] = GET_PIN(D, 7),
	[32] = GET_PIN(B, 12),
	[33] = GET_PIN(B, 13),
	[34] = GET_PIN(B, 15),
	[35] = GET_PIN(D, 11),
	[36] = GET_PIN(D, 12),
	[37] = GET_PIN(D, 13),
	[38] = GET_PIN(D, 14),
	[39] = GET_PIN(D, 15),
};

void valve_init(void)
{
	for (int i = 0; i < sizeof(valve_map) / sizeof(valve_map[0]); i++)
		rt_pin_mode(valve_map[i], PIN_MODE_OUTPUT);
}

void set_valve(int id, int val)
{
	if (id < sizeof(valve_map) / sizeof(valve_map[0])) {
		/*is for PWM temperature control*/
		if (id == 1)
			return;
		rt_pin_write(valve_map[id], val);
	}
}
