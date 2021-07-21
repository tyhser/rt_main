#ifndef _PMC005_H_
#define _PMC005_H_
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int pmc_init(void);
int pmc_send_then_recv(char *read_cmd, int len, void *buf, int recv_size);

#ifdef __cplusplus
}
#endif
#endif
