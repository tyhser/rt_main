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

struct response_info {
	uint8_t host_addr;
	uint8_t status;
	uint8_t data[128];
};

int pmc_init(void);
void pmc_motor_xy_abs(uint8_t station_addr, uint16_t x, uint16_t y);
void pmc_motor_z_abs(uint8_t station_addr, uint16_t pos);
int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos);

int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos);
void pmc_stop(uint8_t station_addr);
void pmc_robot_home(uint8_t station_addr);
void pmc_motor_home(uint8_t station_addr, enum motor_id id);

void pmc_robot_syring_pp(uint8_t station_addr, uint16_t times);
#ifdef __cplusplus
}
#endif
#endif
