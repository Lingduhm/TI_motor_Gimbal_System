#include "ti_msp_dl_config.h"
#include "motor.h"
#include "sensor.h"
#include "key.h"
#include "task.h"
#include "SPI.h"
#include "IIC.h"
#include <stdio.h>

//#define abs(x) ((x) > 0 ? (x) : -(x))
#define delay_ms(X) delay_cycles((CPUCLK_FREQ / 1000) * (X));

//串口服务相关变量
volatile unsigned char uart_data = 0;
//串口服务函数
void uart0_send_char(char ch);
void uart0_send_string(char* str);

int main(void)
{
    SYSCFG_DL_init();
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
   
	// 延时等待系统稳定
    delay_ms(100);
    
    // 初始化任务管理器
    TaskManager_Init();
    car_init();

    // IIC实验
    // float TEMP = SHT20_Read(0xf3);
    // float PH   = SHT20_Read(0xf5);
    // printf("TEMP = %f\r\n",TEMP);
    // printf("PH = %f\r\n",PH);
	
    // SPI实验
	// //读取W25Q128的ID
    // uint8_t buf[10] = {0};
	// printf("ID = %X\r\n",W25Q128_readID());

	// //读取0地址的5个字节数据到buff
	// W25Q128_read(buff, 0, 5);
	// //串口输出读取的数据
	// printf("buff = %s\r\n",buff);

	// //往0地址写入5个字节长度的数据 ABCD
	// W25Q128_write((uint8_t *)"ABCD", 0, 5); // 要用指针进行类型的强转 char->uint8_t，否则无法正确传入函数，函数的参数是一个指针

	// delay_ms(1);//等待稳定

	// //读取0地址的5个字节数据到buff
	// W25Q128_read(buff, 0, 5);

	// //串口输出读取的数据 //若从 Flash 读出的数据不含 \0，printf 会继续打印后面的内存垃圾，直到碰巧遇到 0。
	// printf("buff = %s\r\n",buff);

    while(1)
    {
        TaskManager_Update();   
    }
}

//串口发送单个字符
void uart0_send_char(char ch)
{
   //当串口0忙的时候等待，不忙的时候再发送传进来的字符
   while( DL_UART_isBusy(UART_0_INST) == true );
   //发送单个字符
   DL_UART_Main_transmitData(UART_0_INST, ch);
}

//串口发送字符串
void uart0_send_string(char* str)
{
   //当前字符串地址不在结尾 并且 字符串首地址不为空
   while(*str != 0 && str != NULL)
   {
       //发送字符串首地址中的字符，并且在发送完成之后首地址自增
       uart0_send_char(*str ++);
   }
}

// /**
//  * @brief      获取当前优先级最高的挂起定时器中断
//  *
//  * 检查定时器是否有任何中断正在等待处理。
//  * 即使中断之前没有被开启，也可以检查。
//  *
//  * @param[in]  gptimer    指向定时器外设的寄存器指针
//  *
//  * @return     返回当前优先级最高的挂起中断编号
//  *             是 DL_TIMER_IIDX 枚举中的一个值
//  */
// __STATIC_INLINE DL_TIMER_IIDX DL_Timer_getPendingInterrupt(
//     GPTIMER_Regs *gptimer)
// {
//     return ((DL_TIMER_IIDX) gptimer->CPU_INT.IIDX);
// }

/*! Timer interrupt index for zero interrupt */
// DL_TIMER_IIDX_ZERO = GPTIMER_CPU_INT_IIDX_STAT_Z
// 理解为：当有中断触发需要处理时，判断中断类型为溢出中断而非其他类型如比较中断，则执行任务管理器更新，执行完成读取 IIDX 寄存器后 → 硬件自动清中断标志
    
// 定时器的中断服务函数 已配置为0.005秒的周期，1s触发200次中断
// 全局volatile标记（中断和主循环共享，volatile防止编译器优化）
volatile uint8_t g_flag_need_update = 0;  // 标记需要更新控制逻辑
volatile uint8_t g_raw_key = 0;           // 中断采样的原始按键值
volatile uint32_t g_isr_tick = 0;         // 中断计时tick（用于主循环计时）

// 定时器中断服务函数
void TIMER_0_INST_IRQHandler(void)
{
    switch(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {   
        case DL_TIMER_IIDX_ZERO:
            g_isr_tick++;                // 仅计时（暂时未启用）
            g_raw_key = read_key();      // 仅采样原始按键值（无去抖）
            g_flag_need_update = 1;      // 标记主循环需要处理逻辑
            break;
        default:
            break;
    }
}

// 接收一个发一个，不做字符串的存储
// 串口中断服务函数
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //接发送过来的数据保存在变量中
            uart_data = DL_UART_Main_receiveData(UART_0_INST);
            //将保存的数据再发送出去
            uart0_send_char(uart_data);
            break;

        default://其他的串口中断
            break;
    }
}

int fputc(int ch, FILE *stream)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    DL_UART_Main_transmitData(UART_0_INST, ch);
 
   return ch;
}

// 半主机模式,取消掉keil中的微库，减少内存占用。
// #if !defined(__MICROLIB)
// //不使用微库的话就需要添加下面的函数
// #if (__ARMCLIB_VERSION <= 6000000)
// //如果编译器是AC5  就定义下面这个结构体
// struct __FILE
// {
//         int handle;
// };
// #endif
// FILE __stdout;
// //定义_sys_exit()以避免使用半主机模式
// void _sys_exit(int x)
// {
//         x = x;
// }
// #endif