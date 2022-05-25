#include <rtthread.h>
#include <rtdevice.h>
#include "modbus_event.h"
#include "app_modbus_slave.h"
#include "easyblink.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_md_eventAG      "md_event"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

extern ebled_t led0;
static rt_mq_t mq;

void md_event_init(void)
{
	mq = rt_mq_create("md_mqt", sizeof(struct md_event), 40, RT_IPC_FLAG_FIFO);

	if (mq == NULL)
		rt_kprintf("init message queue failed.\n");
}

int md_event_send(enum md_rw rw, enum md_cmd_type reg_type, uint32_t start_addr,
		  size_t reg_cnt, void *reg)
{
	int result = 0;
	struct md_event msg = {
		.rw = rw,
		.reg_type = reg_type,
		.start_addr = start_addr,
		.reg_cnt = reg_cnt,
		.reg = reg,
	};

	if (reg_type == MD_HOLDING_REG) {
		for (int i = 0; i < reg_cnt; i++) {
			msg.holding_reg_obj[i].md_addr = start_addr + i;
			msg.holding_reg_obj[i].value = ((uint16_t *)reg)[start_addr - S_REG_HOLDING_START + i];
		}
	}

	result = rt_mq_send_wait(mq, &msg, sizeof(msg), 1);
	if (result != RT_EOK) {
		LOG_E("md event send ERR");
		//easyblink(led0, -1, 200, 400);
	}
	return result;
}

struct md_event *md_event_recv(void)
{
	rt_err_t ret = RT_EOK;
	struct md_event *msg;

	msg = rt_malloc(sizeof(struct md_event));
	if (!msg) {
		LOG_E("new md msg failed");
		easyblink(led0, -1, 200, 400);
		return NULL;
	}
	if (!mq) {
		LOG_E("mq null");
		easyblink(led0, -1, 200, 400);
		return NULL;
	}

	ret = rt_mq_recv(mq, msg, sizeof(*msg), RT_WAITING_FOREVER);
	if (ret == RT_EOK)
		return msg;
	else
		return NULL;
}

void md_event_release_msg(struct md_event *msg)
{
	if (msg != NULL)
		rt_free(msg);
}

enum md_rw md_event_get_rw(struct md_event *msg)
{
	if (msg != NULL)
		return msg->rw;
	else
		return MD_EVENT_REG_READ;
}

enum md_cmd_type md_event_get_cmd_type(struct md_event *msg)
{
	if (msg != NULL)
		return msg->reg_type;
	else
		return MD_NONE;
}

uint32_t md_event_get_start_addr(struct md_event *msg)
{
	if (msg != NULL) {
		return msg->start_addr;
	} else {
		LOG_E("msg is NULL");
		easyblink(led0, -1, 200, 400);
		return 0;
	}
}

uint16_t *md_event_get_reg_pointer(struct md_event *msg)
{
	if (msg != NULL) {
		return msg->reg;
	} else {
		LOG_E("msg is NULL");
		easyblink(led0, -1, 200, 400);
		return 0;
	}
}

size_t md_event_get_reg_cnt(struct md_event *msg)
{
	if (msg != NULL)
		return msg->reg_cnt;
	else
		return 0;
}
