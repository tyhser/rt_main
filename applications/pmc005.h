#ifndef _PMC005_H_
#define _PMC005_H_
#include <rtthread.h>

#ifdef APP_USING_PISTON_PUMP
#define SYRING_LEAD_UL		500
#else
#define SYRING_LEAD_UL		(1000/3)
#endif

#define SYRING_SUB_PULSE	16
#define Z_LEAD_MM		18.8495
#define Z_SUB_PULSE		16
#define X_SUB_PULSE		16
#define Y_SUB_PULSE		16
#define X_LEAD_MM		21.2057
#define Y_LEAD_MM		14.1371
#define X_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * X_SUB_PULSE * 200 / (X_LEAD_MM * 10))
#define Y_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * Y_SUB_PULSE * 200 / (Y_LEAD_MM * 10))
#define Z_AXIS_PULSE(mm_10) (int32_t)(((float)(mm_10)) * Z_SUB_PULSE * 200 / (Z_LEAD_MM * 10))
#define SYRING_PULSE(ul) (int32_t)((float)(ul) * SYRING_SUB_PULSE * 200 / (SYRING_LEAD_UL))
#define X_AXIS_LENGTH		3350
#define Y_AXIS_LENGTH		1835
#define Z_AXIS_LENGTH		1055
#define SYRING_LENGTH		10500
#define X_AXIS_PULSE_TO_LEN(pulse) (int32_t)((float)(pulse) * X_LEAD_MM * 10 / (200 * X_SUB_PULSE))

#define ROBOT_ADDR		1
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

enum axis_tristate_pos {
	LOW_POS,
	MIDDLE_POS,
	HIGH_POS,
	UNKNOW_POS
};

struct response_info {
	uint8_t host_addr;
	uint8_t status;
	uint8_t data[128];
};

int pmc_init(void);
void pmc_select_motor(enum motor_id id, int station_addr);
int32_t pmc_get_motor_position(enum motor_id id);
uint32_t pmc_get_current_motor_max_speed(void);
void pmc_motor_xy_abs(uint8_t station_addr, int32_t x, int32_t y);
void pmc_motor_z_abs(uint8_t station_addr, int32_t pos);
void pmc_motor_syring_abs(uint8_t station_addr, int32_t pos);
int pmc_motor_rev(uint8_t station_addr, uint8_t motor_id, int32_t pos);
int pmc_motor_fwd(uint8_t station_addr, uint8_t motor_id, int32_t pos);
int pmc_motor_jog(uint8_t station_addr, uint8_t motor_id, int32_t pos);
void pmc_stop(uint8_t station_addr);
void pmc_robot_home(uint8_t station_addr);
void pmc_motor_home(uint8_t station_addr, enum motor_id id);

void pmc_robot_syring_pp(uint8_t station_addr, uint16_t times);

int pmc_motor_speed_mode(uint8_t station_addr, uint8_t motor_id, int16_t speed);
void pmc_set_valve(uint8_t station_addr, enum pmc_valve value);

void deliver_set_valve(uint8_t station_addr, uint8_t valve);

enum axis_tristate_pos pmc_get_motor_tristate_pos(uint32_t addr, enum motor_id id);
#ifdef __cplusplus
}
#endif
#endif
