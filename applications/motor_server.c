#include <rtthread.h>
#include <rtdevice.h>

#ifndef ULOG_USING_SYSLOG
#define LOG_md_eventAG      "m_ser"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

rt_mq_t m_ser_queue;
rt_thread_t m_ser_thread;

void motor_server_process(motor_ser_event_t event)
{
	motor_server_param_t param = {0};
	rt_memcpy(param, event->parameters, sizeof(param));

	switch (event->id) {
	case XY_ABS:
		pmc_motor_xy_abs(event->station_addr, event->x, event->y);
	break;
	case Z_ABS:
	break;
	case SYRING_ABS:
	break;
	case STOP_ALL:
	break;
	case MOTOR_JOG:
	break;
	case MOTOR_HOME:
	break;
	default:
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
	}
}

void motor_ser_init(void)
{
	m_ser_queue = rt_mq_create("m_ser_q",
			sizeof(struct motor_ser_event),
			64, RT_IPC_FLAG_FIFO);

	if (m_ser_queue == NULL)
		rt_kprintf("init motor server queue failed.\n");

	m_ser_thread = rt_thread_create("m_ser", RT_NULL, 2048, 10, 5);
}
