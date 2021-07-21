#ifndef _APP_MODBUS_SLAVE_H_
#define _APP_MODBUS_SLAVE_H_
#include <rtthread.h>
#include "user_mb_app.h"
#include "mb.h"

#ifdef __cplusplus
extern "C" {
#endif

#if S_COIL_NCOILS%8
extern UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1];
#else
extern UCHAR    ucSCoilBuf[S_COIL_NCOILS/8];
#endif
extern USHORT usSRegHoldBuf[S_REG_HOLDING_NREGS];
extern USHORT usSRegInBuf[S_REG_INPUT_NREGS];

/*
 *	 0    16    80   180   279      (modbus reg map)
 *	 |<16>|<64 >|<100>|<100>|               |
 *	 +----+-----+-----+-----+---------------+
 *	 |d-In|coil |RegIn|Hold |
 *	 +----+-----+-----+-----+
 */

#define REG_VERSION     (usSRegInBuf[90])

enum md_ind {
        MD_ACTIVE_CH_CHANGED_IND,
        MD_NEW_INPUT_CMD_IND,
};

void app_md_slave_init(void);
#ifdef __cplusplus
}
#endif
#endif
