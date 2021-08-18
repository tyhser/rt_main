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

enum pmc_valve {
	PMC_VALVE_ALL_OFF,
	PMC_VALVE_1_ON_2_OFF,
	PMC_VALVE_1_OFF_2_ON,
	PMC_VALVE_ALL_ON
};

struct response_info {
	uint8_t host_addr;
	uint8_t status;
	uint8_t data[128];
};

int pmc_init(void);
void pmc_motor_xy_abs(uint8_t station_addr, int32_t x, int32_t y);
void pmc_motor_z_abs(uint8_t station_addr, int32_t pos);
void pmc_motor_syring_abs(uint8_t station_addr, int32_t pos);
int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos);

int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos);
void pmc_stop(uint8_t station_addr);
void pmc_robot_home(uint8_t station_addr);
void pmc_motor_home(uint8_t station_addr, enum motor_id id);

void pmc_robot_syring_pp(uint8_t station_addr, uint16_t times);

int pmc_motor_speed_mode(uint8_t station_addr, uint8_t motor_id, int16_t speed);
void pmc_set_valve(uint8_t station_addr, enum pmc_valve value);

void deliver_set_valve(uint8_t station_addr, uint8_t valve);
#ifdef __cplusplus
}
#endif
#endif
