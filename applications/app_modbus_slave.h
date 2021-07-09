#ifndef _APP_MODBUS_SLAVE_H_
#define _APP_MODBUS_SLAVE_H_
#include <rtthread.h>
#include "user_mb_app.h"
#include "mb.h"

#ifdef __cplusplus
extern "C" {
#endif

extern USHORT usSRegHoldBuf[S_REG_HOLDING_NREGS];
extern USHORT usSRegInBuf[S_REG_INPUT_NREGS];

#define REG_ACTIVE_CH_ADDR	(0)
#define REG_INPUT_CMD_ADDR	(1)

#define REG_ACTIVE_CH	(usSRegHoldBuf[REG_ACTIVE_CH_ADDR])
#define REG_INPUT_CMD	(usSRegHoldBuf[REG_INPUT_CMD_ADDR])

#define REG_WORK_STATE	(usSRegInBuf[0])
#define REG_CH_DATA	(usSRegInBuf[1])
#define CH_DATA_LEN_MAX	(24 * 4)

enum md_ind {
        MD_ACTIVE_CH_CHANGED_IND,
        MD_NEW_INPUT_CMD_IND,
};

void app_md_slave_init(void);
#ifdef __cplusplus
}
#endif
#endif
