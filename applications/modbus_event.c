#include <rtthread.h>
#include <rtdevice.h>
#include "modbus_event.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_md_eventAG      "md_event"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

static struct rt_messagequeue mq;
static rt_uint8_t msg_pool[2048];

void md_event_init(void)
{
	rt_err_t result;

	result = rt_mq_init(&mq, "md_mqt",
			    &msg_pool[0],
			    sizeof(struct md_event),
			    sizeof(msg_pool), RT_IPC_FLAG_FIFO);

	if (result != RT_EOK)
		rt_kprintf("init message queue failed.\n");
}

int md_event_send(enum md_rw rw, enum md_reg_type reg_type, uint32_t start_addr,
		  size_t reg_cnt)
{
	int result = 0;
	struct md_event msg = {
		.rw = rw,
		.reg_type = reg_type,
		.start_addr = start_addr,
		.reg_cnt = reg_cnt,
	};
	result = rt_mq_send(&mq, &msg, sizeof(msg));
	if (result != RT_EOK)
		LOG_E("md event send ERR");
	return result;
}

struct md_event *md_event_recv(void)
{
	rt_err_t ret = RT_EOK;
	struct md_event *msg;

	msg = rt_malloc(sizeof(struct md_event));
	if (!msg)
		LOG_E("new md msg failed");

	ret = rt_mq_recv(&mq, msg, sizeof(*msg), RT_WAITING_FOREVER);
	if (ret == RT_EOK)
		return msg;
	else
		return NULL;
}

void md_event_release_msg(struct md_event *msg)
{
	rt_free(msg);
}

enum md_rw md_event_get_rw(struct md_event *msg)
{
	return msg->rw;
}

enum md_reg_type md_event_get_reg_type(struct md_event *msg)
{
	return msg->reg_type;
}

uint32_t md_event_get_start_addr(struct md_event *msg)
{
	return msg->start_addr;
}

size_t md_event_get_reg_cnt(struct md_event *msg)
{
	return msg->reg_cnt;
}
