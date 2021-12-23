#ifndef __DAC121S101_H__
#define __DAC121S101_H__

#define PD_OPERATION_NORMAL       0
#define PD_OPERATION_1KTOGND      1
#define PD_OPERATION_100KToGND    2
#define PD_OPERATION_HIGHZ        3

void DAC121S101_Init(void);
int DAC121S101_WriteDAC(int addr, unsigned char op, unsigned short DAValue);
#endif
