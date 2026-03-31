/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for Motor_A */
#define Motor_A_INST                                                       TIMG7
#define Motor_A_INST_IRQHandler                                 TIMG7_IRQHandler
#define Motor_A_INST_INT_IRQN                                   (TIMG7_INT_IRQn)
#define Motor_A_INST_CLK_FREQ                                            2000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_A_C0_PORT                                               GPIOA
#define GPIO_Motor_A_C0_PIN                                       DL_GPIO_PIN_26
#define GPIO_Motor_A_C0_IOMUX                                    (IOMUX_PINCM59)
#define GPIO_Motor_A_C0_IOMUX_FUNC                   IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_Motor_A_C0_IDX                                  DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_A_C1_PORT                                               GPIOA
#define GPIO_Motor_A_C1_PIN                                       DL_GPIO_PIN_24
#define GPIO_Motor_A_C1_IOMUX                                    (IOMUX_PINCM54)
#define GPIO_Motor_A_C1_IOMUX_FUNC                   IOMUX_PINCM54_PF_TIMG7_CCP1
#define GPIO_Motor_A_C1_IDX                                  DL_TIMER_CC_1_INDEX

/* Defines for Motor_B */
#define Motor_B_INST                                                       TIMA1
#define Motor_B_INST_IRQHandler                                 TIMA1_IRQHandler
#define Motor_B_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define Motor_B_INST_CLK_FREQ                                            2000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_B_C0_PORT                                               GPIOA
#define GPIO_Motor_B_C0_PIN                                       DL_GPIO_PIN_17
#define GPIO_Motor_B_C0_IOMUX                                    (IOMUX_PINCM39)
#define GPIO_Motor_B_C0_IOMUX_FUNC                   IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_Motor_B_C0_IDX                                  DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_B_C1_PORT                                               GPIOA
#define GPIO_Motor_B_C1_PIN                                       DL_GPIO_PIN_16
#define GPIO_Motor_B_C1_IOMUX                                    (IOMUX_PINCM38)
#define GPIO_Motor_B_C1_IOMUX_FUNC                   IOMUX_PINCM38_PF_TIMA1_CCP1
#define GPIO_Motor_B_C1_IDX                                  DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (199U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            4000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_0_FBRD_4_MHZ_115200_BAUD                                       (11)




/* Defines for SPI */
#define SPI_INST                                                           SPI1
#define SPI_INST_IRQHandler                                     SPI1_IRQHandler
#define SPI_INST_INT_IRQN                                         SPI1_INT_IRQn
#define GPIO_SPI_PICO_PORT                                                GPIOA
#define GPIO_SPI_PICO_PIN                                        DL_GPIO_PIN_18
#define GPIO_SPI_IOMUX_PICO                                     (IOMUX_PINCM40)
#define GPIO_SPI_IOMUX_PICO_FUNC                     IOMUX_PINCM40_PF_SPI1_PICO
#define GPIO_SPI_POCI_PORT                                                GPIOB
#define GPIO_SPI_POCI_PIN                                         DL_GPIO_PIN_7
#define GPIO_SPI_IOMUX_POCI                                     (IOMUX_PINCM24)
#define GPIO_SPI_IOMUX_POCI_FUNC                     IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI */
#define GPIO_SPI_SCLK_PORT                                                GPIOB
#define GPIO_SPI_SCLK_PIN                                         DL_GPIO_PIN_9
#define GPIO_SPI_IOMUX_SCLK                                     (IOMUX_PINCM26)
#define GPIO_SPI_IOMUX_SCLK_FUNC                     IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED1_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LED1_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group M0 */
#define M0_PORT                                                          (GPIOA)

/* Defines for PIN_12: GPIOA.12 with pinCMx 34 on package pin 5 */
#define M0_PIN_12_PIN                                           (DL_GPIO_PIN_12)
#define M0_PIN_12_IOMUX                                          (IOMUX_PINCM34)
/* Port definition for Pin Group L1 */
#define L1_PORT                                                          (GPIOB)

/* Defines for PIN_23: GPIOB.23 with pinCMx 51 on package pin 22 */
#define L1_PIN_23_PIN                                           (DL_GPIO_PIN_23)
#define L1_PIN_23_IOMUX                                          (IOMUX_PINCM51)
/* Port definition for Pin Group L2 */
#define L2_PORT                                                          (GPIOB)

/* Defines for PIN_27: GPIOB.27 with pinCMx 58 on package pin 29 */
#define L2_PIN_27_PIN                                           (DL_GPIO_PIN_27)
#define L2_PIN_27_IOMUX                                          (IOMUX_PINCM58)
/* Port definition for Pin Group R1 */
#define R1_PORT                                                          (GPIOB)

/* Defines for PIN_8: GPIOB.8 with pinCMx 25 on package pin 60 */
#define R1_PIN_8_PIN                                             (DL_GPIO_PIN_8)
#define R1_PIN_8_IOMUX                                           (IOMUX_PINCM25)
/* Port definition for Pin Group R2 */
#define R2_PORT                                                          (GPIOB)

/* Defines for PIN_6: GPIOB.6 with pinCMx 23 on package pin 58 */
#define R2_PIN_6_PIN                                             (DL_GPIO_PIN_6)
#define R2_PIN_6_IOMUX                                           (IOMUX_PINCM23)
/* Port definition for Pin Group KEY_4 */
#define KEY_4_PORT                                                       (GPIOB)

/* Defines for PIN_3: GPIOB.3 with pinCMx 16 on package pin 51 */
#define KEY_4_PIN_3_PIN                                          (DL_GPIO_PIN_3)
#define KEY_4_PIN_3_IOMUX                                        (IOMUX_PINCM16)
/* Port definition for Pin Group KEY_3 */
#define KEY_3_PORT                                                       (GPIOB)

/* Defines for PIN_2: GPIOB.2 with pinCMx 15 on package pin 50 */
#define KEY_3_PIN_2_PIN                                          (DL_GPIO_PIN_2)
#define KEY_3_PIN_2_IOMUX                                        (IOMUX_PINCM15)
/* Port definition for Pin Group KEY_2 */
#define KEY_2_PORT                                                       (GPIOB)

/* Defines for PIN_16: GPIOB.16 with pinCMx 33 on package pin 4 */
#define KEY_2_PIN_16_PIN                                        (DL_GPIO_PIN_16)
#define KEY_2_PIN_16_IOMUX                                       (IOMUX_PINCM33)
/* Port definition for Pin Group KEY_1 */
#define KEY_1_PORT                                                       (GPIOB)

/* Defines for PIN_15: GPIOB.15 with pinCMx 32 on package pin 3 */
#define KEY_1_PIN_15_PIN                                        (DL_GPIO_PIN_15)
#define KEY_1_PIN_15_IOMUX                                       (IOMUX_PINCM32)
/* Port definition for Pin Group KEY_5 */
#define KEY_5_PORT                                                       (GPIOB)

/* Defines for PIN_21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_5_PIN_21_PIN                                        (DL_GPIO_PIN_21)
#define KEY_5_PIN_21_IOMUX                                       (IOMUX_PINCM49)
/* Port definition for Pin Group CS */
#define CS_PORT                                                          (GPIOB)

/* Defines for PIN: GPIOB.13 with pinCMx 30 on package pin 1 */
#define CS_PIN_PIN                                              (DL_GPIO_PIN_13)
#define CS_PIN_IOMUX                                             (IOMUX_PINCM30)
/* Port definition for Pin Group I2C */
#define I2C_PORT                                                         (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define I2C_SDA_PIN                                              (DL_GPIO_PIN_0)
#define I2C_SDA_IOMUX                                             (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define I2C_SCL_PIN                                              (DL_GPIO_PIN_1)
#define I2C_SCL_IOMUX                                             (IOMUX_PINCM2)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_A_init(void);
void SYSCFG_DL_Motor_B_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
