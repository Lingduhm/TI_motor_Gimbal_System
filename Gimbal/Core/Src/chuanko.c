#include <string.h>
#include "chuanko.h"
#include "main.h"
#include "usart.h"
// 定义接收缓冲区大小
uint16_t xx,yy;
float xxx,yyy;
char RxBuffer[RX_BUFFER_SIZE];  // 接收数据的缓冲区
volatile uint8_t RxIndex = 0;   // 接收数据的索引
uint8_t uart_rx_byte;              // 单字节接收缓冲区

// 串口接收状态机定义
typedef enum {
    STATE_WAIT_HEADER,          //等待帧头
    STATE_RECEIVE_DATA,			//接收数据部分
    STATE_WAIT_TAIL     		//等待帧尾
} ReceiveState;

ReceiveState state = STATE_WAIT_HEADER;			//当前接收状态
uint8_t data_received = 0;		//接受完数据帧标志位

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t received = uart_rx_byte;  // 获取接收到的数据
        
        switch (state) {
            case STATE_WAIT_HEADER:      // 等待帧头
                if (received == 0x2C) {  // 检测到帧头
                    state = STATE_RECEIVE_DATA; // 设置状态为开始接收数据
                    RxIndex = 0;
                }
                break;
                
            case STATE_RECEIVE_DATA:     // 开始接收数据
                if (received == 0x5B) {  // 检测到帧尾
                    state = STATE_WAIT_TAIL; // 设置状态为等待帧尾
                    data_received = 1;   // 接收完数据标志位置1
                } else {
                    if (RxIndex < RX_BUFFER_SIZE - 1) {
                        RxBuffer[RxIndex++] = received;  // 如果接收到的数据不是帧尾,那就一直把数据存在缓冲区里
                    }
                }
                break;
                
            case STATE_WAIT_TAIL:        // 接收到等待帧尾就重新设置为等待帧头,如此循环接收
                if (received == 0x5B) {
                    state = STATE_WAIT_HEADER;
                } else {
                    state = STATE_WAIT_HEADER;
                }
                break;
        }
        
        // 重新启动接收中断
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

// 处理接收到的数据
void Process_Received_Data(void)
{
    if (data_received) {
        // 解析数据
        if (RxIndex >= 4) {  // 至少有4个字节的数据(x的高字节、低字节,y的高字节、低字节)
            // 合成x坐标
            uint16_t x_high = RxBuffer[0];
            uint16_t x_low = RxBuffer[1];
            xx = (x_high << 8) | x_low;
            xxx=xx/100.0;
            // 合成y坐标
            uint16_t y_high = RxBuffer[2];
            uint16_t y_low = RxBuffer[3];
            yy = (y_high << 8) | y_low;
					  yyy=yy/100.0;
        }
        
        // 重置接收状态
        data_received = 0;
        RxIndex = 0;
        state = STATE_WAIT_HEADER;
    }
}
