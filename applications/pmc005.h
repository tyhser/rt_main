#ifndef _PMC005_H_
#define _PMC005_H_
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

enum axis_id {
	XY_AXIS,
	Z_AXIS,
	SYRING,
};

enum motor_id {
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
};

int pmc_init(void);
void pmc_motor_xy_pose(uint8_t station_addr, uint16_t x, uint16_t y);
int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos);

int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos);
void pmc_stop(uint8_t station_addr);
#ifdef __cplusplus
}
#endif
#endif
