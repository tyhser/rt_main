#ifndef _MODBUS_EVENT_H_
#define _MODBUS_EVENT_H_
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

enum md_rw {
	MD_EVENT_REG_WRITE,
	MD_EVENT_REG_READ,
};

enum md_cmd_type {
	MD_HOLDING_REG,
	MD_COIL,

	MD_NONE
};

struct md_event {
	enum md_rw rw;
	enum md_cmd_type reg_type;
	uint32_t start_addr;
	size_t reg_cnt;
	uint16_t *reg;
};

void md_event_init(void);
int md_event_send(enum md_rw rw, enum md_cmd_type reg_type, uint32_t start_addr,
		  size_t reg_cnt, void *reg);

/*md_event_recv and md_event_release_msg must pairing use*/
struct md_event *md_event_recv(void);
void md_event_release_msg(struct md_event *msg);

enum md_rw md_event_get_rw(struct md_event *msg);
enum md_cmd_type md_event_get_cmd_type(struct md_event *msg);
uint32_t md_event_get_start_addr(struct md_event *msg);
size_t md_event_get_reg_cnt(struct md_event *msg);

uint16_t *md_event_get_reg_pointer(struct md_event *msg);
#ifdef __cplusplus
}
#endif
#endif
