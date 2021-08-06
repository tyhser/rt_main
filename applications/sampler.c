#include "pmc005.h"

void sampler_set_pumb_speed(uint16_t addr, int pumb_index, int16_t speed)
{
	pmc_motor_speed_mode(addr, pumb_index, speed);
}

void sampler_set_valve(uint16_t addr, int 
