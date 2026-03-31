#ifndef __IIC_H__
#define __IIC_H__

#include "ti_msp_dl_config.h"
#include "stdio.h"

// 设置SDA输出模式
#define SDA_OUT()   {                                                \
    /* 更改引脚功能为gpio输出  I2C_SDA_IOMUX => 引脚的寄存器地址 */ \
    DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);                      \
    /* 预设置引脚电平 */                                             \
    DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);                        \
    /* 使能输出引脚电平，此后输出缓冲器一直开启，除非更改引脚功能 */   \
    DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN);                   \
}

//设置SDA输入模式
#define SDA_IN()    { DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); } // 更改引脚电平功能为gpio输入

//获取SDA引脚的电平变化
#define SDA_GET()   ( ( ( DL_GPIO_readPins(I2C_PORT,I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 ) //防御性编程

// /**
//  *  @brief      Read a group of GPIO pins
//  *
//  *  @param[in]  gpio  Pointer to the register overlay for the peripheral
//  *  @param[in]  pins  Pins to read. Bitwise OR of @ref DL_GPIO_PIN.
//  *
//  *  @return     The pins (from the selection) that are currently high
//  *
//  *  @retval     Bitwise OR of @ref DL_GPIO_PIN of pins that are currently high
//  *              from the input selection.
//  */
// __STATIC_INLINE uint32_t DL_GPIO_readPins(GPIO_Regs* gpio, uint32_t pins)
// {
//     return (gpio->DIN31_0 & pins);// 已经只能输出传入的引脚的电平
// }

//SDA与SCL输出
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SDA_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SCL_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SCL_PIN)) )

//函数声明
float SHT20_Read(uint8_t regaddr);

#endif