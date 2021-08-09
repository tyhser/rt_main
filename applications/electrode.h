#ifndef _ELECTRODE_H_
#define _ELECTRODE_H_

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

extern rt_err_t electrode_init(void);
extern rt_uint32_t electrode_get_data(rt_uint32_t ch);
extern float electrode_data_to_voltagemv(uint64_t data);
extern int64_t voltagemv_to_electrode_data(float mv);
float electrode_get_temperature(void);

#ifdef __cplusplus
}
#endif

#endif
