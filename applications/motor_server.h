#ifndef _MOTOR_SERVER_H_
#define _MOTOR_SERVER_H_
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_ser_event_id {
	XY_ABS,
	Z_ABS,
	SYRING_ABS,
	STOP_ALL,
	MOTOR_JOG,
	MOTOR_HOME,
};

typedef struct {
	uint8_t station_addr;
	int32_t x;
	int32_t y;

} xy_abs_t;

typedef struct {
	uint8_t station_addr;
	uint8_t motor_id;
	int32_t pos;

} motor_move_t;

typedef union {
	xy_abs_t xy_abs;
	motor_move_t motor_move;

} motor_server_param_t;

struct motor_ser_event {
    enum motor_ser_event_id id;
    motor_server_param_t parameters;
    post_result_t post_callback;

};
typedef struct motor_ser_event *motor_ser_event_t;

#ifdef __cplusplus
}
#endif
#endif
