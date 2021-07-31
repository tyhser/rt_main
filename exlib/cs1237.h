#ifndef __CS1237_H__
#define __CS1237_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CS_CONFIG   0X50		//芯片配置寄存器地址
#define ADC_BIT     24			//ADC有效位数，带符号位，最高24位
#define FILTER		  0.00001	//滤波系数，小于1

enum CS1237_CHANNEL
{
    channel1,
    channel2,
    channel_cnt
};

extern int cs1237_init(void);
extern rt_int32_t cs1237_convert(enum CS1237_CHANNEL ch);
extern void cs1237_power_down(enum CS1237_CHANNEL ch);







#ifdef __cplusplus
}
#endif

#endif
