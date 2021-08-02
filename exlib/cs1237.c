#include <rtthread.h>
#if defined(BSP_USING_CS1237)
#include <board.h>
#include <rtdevice.h>
#include "cs1237.h"
#include "drivers/pin.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "cs1237"
#define LOG_LVL              LOG_LVL_ERROR
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

#define CS1237_BITS 24

#define DELAY_MS(t) cs1237_delay_us(1000*t)
#define DELAY_US(t) cs1237_delay_us(t)

#define SCLK1_PIN       GET_PIN(D,  0)
#define DRDYDOUT1_PIN   GET_PIN(D,  1)

#define SCLK2_PIN       GET_PIN(C, 10)
#define DRDYDOUT2_PIN   GET_PIN(C, 11)

enum CS1237_RW {
	CS1237_R = PIN_MODE_INPUT,
	CS1237_W = PIN_MODE_OUTPUT
};

struct ic_pin {
	rt_base_t sclk_pin;
	rt_base_t drdydout_pin;
} boad_pin[2] = {
	{ SCLK1_PIN, DRDYDOUT1_PIN },
	{ SCLK2_PIN, DRDYDOUT2_PIN },
};

#define SCLK_P(ch) boad_pin[(ch)].sclk_pin
#define DRDYDOUT_P(ch) boad_pin[(ch)].drdydout_pin

void cs1237_delay_us(uint32_t nus)
{
	uint32_t Delay = nus * 168 / 4;

	do {
		__NOP();
	} while (Delay--);
}

void drdydout_set_mode(enum CS1237_CHANNEL ch, enum CS1237_RW rw)
{
	rt_pin_mode(DRDYDOUT_P(ch), rw);
}

int drdydout_read(enum CS1237_CHANNEL ch)
{
	return rt_pin_read(DRDYDOUT_P(ch));
}

void drdydout_write(enum CS1237_CHANNEL ch, int val)
{
	rt_pin_write(DRDYDOUT_P(ch), val);
}

void sclk_write(enum CS1237_CHANNEL ch, int val)
{
	rt_pin_write(SCLK_P(ch), val);
}

int is_cs1237_ready(enum CS1237_CHANNEL ch)
{
	if (drdydout_read(ch) == 0)
		return 1;
	else
		return 0;
}

void cs1237_sclk_up(enum CS1237_CHANNEL ch)
{
	sclk_write(ch, 0);
	DELAY_US(1);
	sclk_write(ch, 1);
	DELAY_US(1);
}

void cs1237_one_clk(enum CS1237_CHANNEL ch)
{
	sclk_write(ch, 1);
	DELAY_US(1);
	sclk_write(ch, 0);
	DELAY_US(1);
}

/*sclk hold on high level 100us, enter low power mode*/
void cs1237_power_down(enum CS1237_CHANNEL ch)
{
	sclk_write(ch, 1);
	DELAY_US(100);
	sclk_write(ch, 1);
	DELAY_US(100);
}

/*cs1237 power on*/
void cs1237_restart(enum CS1237_CHANNEL ch)
{
	sclk_write(ch, 0);
	DELAY_US(20);
}

/*
 * 名称：cs1237_write_config
 * 功能：向控制寄存器中写控制字
 * 入口：需要写入的控制字
 * 出口：无
 * 说明： 写控制寄存器地址：0X65
 *        写入默认控制字：0X0C
 */
void cs1237_write_config(enum CS1237_CHANNEL ch, unsigned char config)
{

	unsigned char _dat = 0x80;
	unsigned char count_i = 0;

	sclk_write(ch, 0);
	drdydout_set_mode(ch, CS1237_R);

	while (drdydout_read(ch) == 1) {
		DELAY_MS(5);
		count_i++;
		if (count_i > 150) {
			sclk_write(ch, 1);
			drdydout_write(ch, 1);
			return;
		}
	}

	for (int i = 0; i < 29; i++)
		cs1237_one_clk(ch);

	drdydout_set_mode(ch, CS1237_W);

#define DRDYOUT_OUT(ch, dout) do {\
	sclk_write(ch, 1);\
	DELAY_US(30);\
	drdydout_write(ch, dout);\
	sclk_write(ch, 0);\
	DELAY_US(30);\
	} while (0)

	//第30-36个时钟周期，输入寄存器的写或读命令字数据（高位先输入），这里是写，应输入0x65
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 0);
	DRDYOUT_OUT(ch, 0);
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 0);
	DRDYOUT_OUT(ch, 1);

	cs1237_one_clk(ch);

	for (int i = 0; i < 8; i++) {

		sclk_write(ch, 1);
		DELAY_US(40);
		if ((config & _dat) != 0)
			drdydout_write(ch, 1);
		else
			drdydout_write(ch, 0);

		sclk_write(ch, 0);
		DELAY_US(1);
		_dat >>= 1;
	}
	/*46个脉冲，切换DOUT引脚，并且拉高DOUT引脚 */
	cs1237_one_clk(ch);
}

/*
 * 名称：cs1237_read_config
 * 功能：读取CS1237的控制寄存器中的数据
 * 入口：无
 * 出口：读出的控制字
 * 说明：
读控制寄存器地址：0X56
读出刚才写的控制字0X0C，如果读出的控制字和刚才写入的一样，说明通信成功
 */
unsigned char cs1237_read_config(enum CS1237_CHANNEL ch)
{
	unsigned char i = 0;
	unsigned char dat = 0;
	unsigned char count_i = 0;

	sclk_write(ch, 0);

	drdydout_set_mode(ch, CS1237_R);
	/*芯片准备好数据输出  时钟已经为0，数据也需要等CS1237全部拉低为0才算都准备好*/
	while (drdydout_read(ch) == 1) {
		DELAY_MS(5);
		count_i++;
		if (count_i > 150) {
			sclk_write(ch, 1);
			drdydout_write(ch, 1);
			return 1;
		}
	}

	for (i = 0; i < 29; i++)
		cs1237_one_clk(ch);

	drdydout_set_mode(ch, CS1237_W);

	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 0);
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 0);
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 1);
	DRDYOUT_OUT(ch, 0);

	drdydout_write(ch, 1);

	cs1237_one_clk(ch);

	drdydout_set_mode(ch, CS1237_R);
	for (i = 0; i < 8; i++) {
		cs1237_one_clk(ch);
		dat <<= 1;
		if (drdydout_read(ch) == 1)
			dat++;
	}
	cs1237_one_clk(ch);

	return dat;
}

/*
 * 名称：cs1237_read_adc
 * 功能：读取CS1237的ADC数据
 * 入口：无
 * 出口：20位的ADC数据
 * 说明：
 */
int cs1237_read_adc(enum CS1237_CHANNEL ch)
{
	unsigned char i = 0;
	int dat = 0;
	unsigned char count_i = 0;

	sclk_write(ch, 0);

	drdydout_set_mode(ch, CS1237_R);

	while (drdydout_read(ch) == 1) {
		DELAY_MS(5);
		count_i++;
		if (count_i > 150) {
			sclk_write(ch, 1);
			drdydout_write(ch, 1);
			return 0;
		}
	}

	for (i = 0; i < 24; i++) {
		sclk_write(ch, 1);
		DELAY_US(1);
		dat <<= 1;
		if (drdydout_read(ch) == 1)
			dat++;
		sclk_write(ch, 0);
		DELAY_US(1);
	}
	//一共需要输入27个脉冲
	for (i = 0; i < 3; i++)
		cs1237_one_clk(ch);

	drdydout_set_mode(ch, CS1237_W);
	drdydout_write(ch, 1);

	i = 24 - ADC_BIT;
	dat >>= i;		//丢弃多余的位数

	/*24 bit complement to 32 bit int */
	dat = ((dat << 8) >> 8);
	return dat;
}

/*
 *名称：CS1237_Read_18bit_ADC
 *功能：对原始的ADC数据进行低通滤波
 *入口：无
 *出口：18位的ADC数据
 *说明：
一阶数字滤波器（一阶低通滤波器）
本次滤波数据  C = B*FILTER + C*(1-FILTER)
优点：
      对周期性干扰具有良好的抑制作用
      适用于波动频率较高的场合
缺点：
      相位滞后，灵敏度低
      滞后程度取决于a值大小
      不能消除滤波频率高于采样频率的1/2的干扰信号
 */
long cs1237_read_18bit_adc(enum CS1237_CHANNEL ch)
{
	static struct {
		int now;
		int last;
	} filter_tmp[4] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
	//本次数据
	filter_tmp[ch].now = cs1237_read_adc(ch);
	//读取到正确的数据
	if (filter_tmp[ch].now != 0) {
		filter_tmp[ch].last =
		    filter_tmp[ch].last * FILTER + filter_tmp[ch].now * (1 -
									 FILTER);
	}
	return filter_tmp[ch].last;
}

int cs1237_init(void)
{
	rt_pin_mode(SCLK1_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(SCLK2_PIN, PIN_MODE_OUTPUT);

	for (int i = channel1; i < channel_cnt; i++) {
		sclk_write(i, 0);
		drdydout_set_mode(i, CS1237_R);
	}
	for (int i = channel1; i < channel_cnt; i++)
		cs1237_write_config(i, CS_CONFIG);

	for (int i = channel1; i < channel_cnt; i++) {
		while (cs1237_read_config(i) != CS_CONFIG) {
			rt_kprintf("\n\tCS1237%d read error...\n", i);
			cs1237_write_config(i, CS_CONFIG);
			DELAY_MS(10);
			continue;
		}
	}
	return 0;
}

void cs1237_enabled(rt_uint32_t channel)
{

}

rt_uint32_t cs1237_low_level_read(rt_uint32_t channel)
{
	rt_uint32_t data = 0;
	rt_uint32_t tmp = 0;

	while (!is_cs1237_ready(channel))
		;

	for (int i = CS1237_BITS - 1; i >= 0; i--) {

		sclk_write(channel, 0);
		DELAY_MS(1);
		sclk_write(channel, 1);
		DELAY_MS(1);

		tmp = drdydout_read(channel);
		data |= (tmp << i);
		sclk_write(channel, 0);
	}
	sclk_write(channel, 0);
	cs1237_sclk_up(channel);
	cs1237_sclk_up(channel);
	cs1237_sclk_up(channel);
	sclk_write(channel, 0);
	return data;
}

rt_int32_t mean_sample(rt_uint32_t channel, rt_uint32_t mean_range)
{
	rt_int32_t sample_total = 0;

	for (int i = 0; i < mean_range; i++)
		sample_total += (int)cs1237_read_adc(channel);

	return sample_total / mean_range;
}

int32_t average_filter(rt_uint32_t channel)
{
	int32_t data = 0;
	int32_t sum = 0;
#define N 15
	static int32_t data_buf[4][N] = {0};

	if (data_buf[channel][0] == 0) {
		data = (int)cs1237_read_adc(channel);
		for (int i = 0; i < N - 1; i++)
			data_buf[channel][i] = data;
	}
	data = cs1237_read_adc(channel);
	for (int i = 0; i < N - 1; i++) {
		data_buf[channel][i] = data_buf[channel][i + 1];
		sum = sum + data_buf[channel][i];
	}
	data_buf[channel][N - 1] = data;
	sum += data_buf[channel][N - 1];
	return (sum / N);
#undef N
}

rt_int32_t cs1237_convert(enum CS1237_CHANNEL channel)
{
	return mean_sample(channel, 1);
}
#endif
