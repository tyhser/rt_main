#include "modbus_rtu_test.h"
#include "modbus.h"
#include "stdio.h"
#include "string.h"

#define RS485_RE GET_PIN(D, 10)
#define TEST 1

static void modbus_master_thread(void *param)
{
#if TEST
    uint16_t tab_reg[64] = {0};
#endif
    modbus_t *ctx = RT_NULL;
    ctx = modbus_new_rtu("/dev/uart3", 115200, 'N', 8, 1);
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    modbus_rtu_set_rts(ctx, RS485_RE, MODBUS_RTU_RTS_UP);
    modbus_set_slave(ctx, 1);
    modbus_set_debug(ctx, 0);
    modbus_connect(ctx);
    modbus_set_response_timeout(ctx, 0, 1000000);

#if TEST
    int num = 0;
    while (1)
    {
	//modbus_write_register(ctx, 0, 0x03);
        memset(tab_reg, 0, 64 * 2);
        int regs = modbus_read_input_registers(ctx, 0, 20, tab_reg);
        rt_kprintf("[%4d][read num = %d]", num, regs);
        num++;
        int i;
        for (i = 0; i < 20; i++)
        {
            rt_kprintf("<%#x>", tab_reg[i]);
        }
        rt_kprintf("\n");
        rt_thread_mdelay(30);
    }
#endif
    modbus_close(ctx);
    modbus_free(ctx);
}

static int rtu_test_init(void)
{
    rt_pin_mode(RS485_RE, PIN_MODE_OUTPUT);
    rt_thread_t tid;
    tid = rt_thread_create("modbus",
                           modbus_master_thread, RT_NULL,
                           2048,
                           12, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(rtu_test_init);
