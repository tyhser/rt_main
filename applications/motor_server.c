#include <rtthread.h>
#include <rtdevice.h>
#include "pmc005.h"
#include "motor_server.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_md_eventAG      "m_ser"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

rt_mq_t m_ser_queue;
rt_thread_t m_ser_thread;

void motor_server_z_abs(int32_t pos)
{
	motor_server_param_t param = {0};

	param.motor_move.station_addr = ROBOT_ADDR;
	param.motor_move.pos = pos;
	param.motor_move.motor_id = MOTOR_3;
	motor_server_post(Z_ABS, &param);
}

void motor_server_syring_abs(int32_t pos)
{
	motor_server_param_t param = {0};

	param.motor_move.station_addr = ROBOT_ADDR;
	param.motor_move.pos = pos;
	param.motor_move.motor_id = MOTOR_4;

	motor_server_post(SYRING_ABS, &param);
}

void motor_server_post(enum motor_ser_id id, motor_server_param_t *param)
{
	int result = 0;
	struct motor_ser_event event = {0};

	event.id = id;
	event.post_callback = NULL;
	rt_memcpy(&event.parameters, param, sizeof(*param));

	result = rt_mq_send(m_ser_queue, &event, sizeof(event));
	if (result != RT_EOK) {
		LOG_E("motor server post failed");
	}
}

void set_robot_busy(enum axis_id id);

void motor_server_process(motor_ser_event_t event)
{
	motor_server_param_t param = {0};
	rt_memcpy(&param, &event->parameters, sizeof(param));

	switch (event->id) {
	case XY_ABS:
		set_robot_busy(XY_AXIS);
		pmc_motor_xy_abs(param.xy_abs.station_addr, param.xy_abs.x, param.xy_abs.y);
	break;
	case Z_ABS:
		set_robot_busy(Z_AXIS);
		pmc_motor_z_abs(param.motor_move.station_addr, param.motor_move.pos);
	break;
	case SYRING_ABS:
		set_robot_busy(SYRING);
		pmc_motor_syring_abs(param.motor_move.station_addr, param.motor_move.pos);
	break;
	case STOP_ALL:
		pmc_stop(param.motor_move.station_addr);
	break;
	case MOTOR_JOG:
		set_robot_busy(XY_AXIS);
		set_robot_busy(Z_AXIS);
		set_robot_busy(SYRING);
		pmc_motor_jog(param.motor_move.station_addr, param.motor_move.motor_id, param.motor_move.pos);
	break;
	case MOTOR_HOME:
		set_robot_busy(XY_AXIS);
		set_robot_busy(Z_AXIS);
		set_robot_busy(SYRING);
		pmc_motor_home(param.motor_move.station_addr, param.motor_move.motor_id);
	break;
	case SYRING_PP:
		set_robot_busy(SYRING);
		pmc_robot_syring_pp(param.motor_move.station_addr, param.motor_move.pos);
		break;
	default:
		LOG_E("unknow motor server id");
	break;
	}
}

void motor_server_thread_entry(void *parameter)
{
	rt_err_t ret = RT_EOK;
	struct motor_ser_event event;

	while (1) {
		ret = rt_mq_recv(m_ser_queue,
				&event,
				sizeof(event),
				RT_WAITING_FOREVER);
		if (ret != RT_EOK) {
			LOG_E("m_ser_queue recv error");
			continue;
		}
		motor_server_process(&event);
	}
}

int motor_ser_init(void)
{
	m_ser_queue = rt_mq_create("m_ser_q",
			sizeof(struct motor_ser_event),
			8, RT_IPC_FLAG_FIFO);

	if (m_ser_queue == NULL)
		rt_kprintf("init motor server queue failed.\n");

	m_ser_thread = rt_thread_create("m_ser",
			motor_server_thread_entry,
			RT_NULL,
			3072,
			11, 5);
	rt_thread_startup(m_ser_thread);
	return 0;
}
INIT_APP_EXPORT(motor_ser_init);
