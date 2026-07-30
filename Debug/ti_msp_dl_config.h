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



/* Defines for PWM_MOTOR_AB */
#define PWM_MOTOR_AB_INST                                                  TIMG0
#define PWM_MOTOR_AB_INST_IRQHandler                            TIMG0_IRQHandler
#define PWM_MOTOR_AB_INST_INT_IRQN                              (TIMG0_INT_IRQn)
#define PWM_MOTOR_AB_INST_CLK_FREQ                                        100000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_AB_C0_PORT                                          GPIOB
#define GPIO_PWM_MOTOR_AB_C0_PIN                                  DL_GPIO_PIN_10
#define GPIO_PWM_MOTOR_AB_C0_IOMUX                               (IOMUX_PINCM27)
#define GPIO_PWM_MOTOR_AB_C0_IOMUX_FUNC              IOMUX_PINCM27_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_AB_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_AB_C1_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_AB_C1_PIN                                  DL_GPIO_PIN_24
#define GPIO_PWM_MOTOR_AB_C1_IOMUX                               (IOMUX_PINCM54)
#define GPIO_PWM_MOTOR_AB_C1_IOMUX_FUNC              IOMUX_PINCM54_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_AB_C1_IDX                             DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_CD */
#define PWM_MOTOR_CD_INST                                                  TIMA0
#define PWM_MOTOR_CD_INST_IRQHandler                            TIMA0_IRQHandler
#define PWM_MOTOR_CD_INST_INT_IRQN                              (TIMA0_INT_IRQn)
#define PWM_MOTOR_CD_INST_CLK_FREQ                                        100000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_CD_C0_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_CD_C0_PIN                                   DL_GPIO_PIN_0
#define GPIO_PWM_MOTOR_CD_C0_IOMUX                                (IOMUX_PINCM1)
#define GPIO_PWM_MOTOR_CD_C0_IOMUX_FUNC               IOMUX_PINCM1_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_CD_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_CD_C1_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_CD_C1_PIN                                   DL_GPIO_PIN_1
#define GPIO_PWM_MOTOR_CD_C1_IOMUX                                (IOMUX_PINCM2)
#define GPIO_PWM_MOTOR_CD_C1_IOMUX_FUNC               IOMUX_PINCM2_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_CD_C1_IDX                             DL_TIMER_CC_1_INDEX



/* Defines for TIMER_PID */
#define TIMER_PID_INST                                                   (TIMA1)
#define TIMER_PID_INST_IRQHandler                               TIMA1_IRQHandler
#define TIMER_PID_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define TIMER_PID_INST_LOAD_VALUE                                       (63999U)




/* Defines for I2C_MPU6050 */
#define I2C_MPU6050_INST                                                    I2C0
#define I2C_MPU6050_INST_IRQHandler                              I2C0_IRQHandler
#define I2C_MPU6050_INST_INT_IRQN                                  I2C0_INT_IRQn
#define I2C_MPU6050_BUS_SPEED_HZ                                          400000
#define GPIO_I2C_MPU6050_SDA_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SDA_PIN                                  DL_GPIO_PIN_28
#define GPIO_I2C_MPU6050_IOMUX_SDA                                (IOMUX_PINCM3)
#define GPIO_I2C_MPU6050_IOMUX_SDA_FUNC                 IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_MPU6050_SCL_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SCL_PIN                                  DL_GPIO_PIN_31
#define GPIO_I2C_MPU6050_IOMUX_SCL                                (IOMUX_PINCM6)
#define GPIO_I2C_MPU6050_IOMUX_SCL_FUNC                 IOMUX_PINCM6_PF_I2C0_SCL

/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
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
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_18
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM40)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM40_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_1_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Defines for AIN_1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_MOTOR_A_AIN_1_PORT                                          (GPIOB)
#define GPIO_MOTOR_A_AIN_1_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_MOTOR_A_AIN_1_IOMUX                                 (IOMUX_PINCM13)
/* Defines for AIN_2: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_MOTOR_A_AIN_2_PORT                                          (GPIOB)
#define GPIO_MOTOR_A_AIN_2_PIN                                  (DL_GPIO_PIN_11)
#define GPIO_MOTOR_A_AIN_2_IOMUX                                 (IOMUX_PINCM28)
/* Defines for EA_1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_MOTOR_A_EA_1_PORT                                           (GPIOA)
#define GPIO_MOTOR_A_EA_1_PIN                                    (DL_GPIO_PIN_7)
#define GPIO_MOTOR_A_EA_1_IOMUX                                  (IOMUX_PINCM14)
/* Defines for EB_1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_MOTOR_A_EB_1_PORT                                           (GPIOB)
// groups represented: ["GPIO_MOTOR_C","GPIO_MOTOR_D","GPIO_MOTOR_A"]
// pins affected: ["EB_3","EB_4","EB_1"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_MOTOR_A_EB_1_IIDX                              (DL_GPIO_IIDX_DIO14)
#define GPIO_MOTOR_A_EB_1_PIN                                   (DL_GPIO_PIN_14)
#define GPIO_MOTOR_A_EB_1_IOMUX                                  (IOMUX_PINCM31)
/* Defines for BIN_1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_MOTOR_B_BIN_1_PORT                                          (GPIOB)
#define GPIO_MOTOR_B_BIN_1_PIN                                  (DL_GPIO_PIN_24)
#define GPIO_MOTOR_B_BIN_1_IOMUX                                 (IOMUX_PINCM52)
/* Defines for BIN_2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_MOTOR_B_BIN_2_PORT                                          (GPIOA)
#define GPIO_MOTOR_B_BIN_2_PIN                                  (DL_GPIO_PIN_22)
#define GPIO_MOTOR_B_BIN_2_IOMUX                                 (IOMUX_PINCM47)
/* Defines for EA_2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_MOTOR_B_EA_2_PORT                                           (GPIOA)
#define GPIO_MOTOR_B_EA_2_PIN                                   (DL_GPIO_PIN_15)
#define GPIO_MOTOR_B_EA_2_IOMUX                                  (IOMUX_PINCM37)
/* Defines for EB_2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_MOTOR_B_EB_2_PORT                                           (GPIOA)
// pins affected by this interrupt request:["EB_2"]
#define GPIO_MOTOR_B_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GPIO_MOTOR_B_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_MOTOR_B_EB_2_IIDX                              (DL_GPIO_IIDX_DIO17)
#define GPIO_MOTOR_B_EB_2_PIN                                   (DL_GPIO_PIN_17)
#define GPIO_MOTOR_B_EB_2_IOMUX                                  (IOMUX_PINCM39)
/* Port definition for Pin Group GPIO_MOTOR_C */
#define GPIO_MOTOR_C_PORT                                                (GPIOB)

/* Defines for CIN_1: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_MOTOR_C_CIN_1_PIN                                   (DL_GPIO_PIN_5)
#define GPIO_MOTOR_C_CIN_1_IOMUX                                 (IOMUX_PINCM18)
/* Defines for CIN_2: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_MOTOR_C_CIN_2_PIN                                   (DL_GPIO_PIN_4)
#define GPIO_MOTOR_C_CIN_2_IOMUX                                 (IOMUX_PINCM17)
/* Defines for EA_3: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_MOTOR_C_EA_3_PIN                                   (DL_GPIO_PIN_12)
#define GPIO_MOTOR_C_EA_3_IOMUX                                  (IOMUX_PINCM29)
/* Defines for EB_3: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_MOTOR_C_EB_3_IIDX                              (DL_GPIO_IIDX_DIO13)
#define GPIO_MOTOR_C_EB_3_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_MOTOR_C_EB_3_IOMUX                                  (IOMUX_PINCM30)
/* Defines for DIN_1: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_MOTOR_D_DIN_1_PORT                                          (GPIOA)
#define GPIO_MOTOR_D_DIN_1_PIN                                   (DL_GPIO_PIN_8)
#define GPIO_MOTOR_D_DIN_1_IOMUX                                 (IOMUX_PINCM19)
/* Defines for DIN_2: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_MOTOR_D_DIN_2_PORT                                          (GPIOA)
#define GPIO_MOTOR_D_DIN_2_PIN                                   (DL_GPIO_PIN_9)
#define GPIO_MOTOR_D_DIN_2_IOMUX                                 (IOMUX_PINCM20)
/* Defines for EA_4: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_MOTOR_D_EA_4_PORT                                           (GPIOB)
#define GPIO_MOTOR_D_EA_4_PIN                                   (DL_GPIO_PIN_15)
#define GPIO_MOTOR_D_EA_4_IOMUX                                  (IOMUX_PINCM32)
/* Defines for EB_4: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_MOTOR_D_EB_4_PORT                                           (GPIOB)
#define GPIO_MOTOR_D_EB_4_IIDX                              (DL_GPIO_IIDX_DIO16)
#define GPIO_MOTOR_D_EB_4_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_MOTOR_D_EB_4_IOMUX                                  (IOMUX_PINCM33)
/* Defines for AD0: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_GRAYSCALE_AD0_PORT                                          (GPIOB)
#define GPIO_GRAYSCALE_AD0_PIN                                  (DL_GPIO_PIN_27)
#define GPIO_GRAYSCALE_AD0_IOMUX                                 (IOMUX_PINCM58)
/* Defines for AD1: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_GRAYSCALE_AD1_PORT                                          (GPIOB)
#define GPIO_GRAYSCALE_AD1_PIN                                  (DL_GPIO_PIN_26)
#define GPIO_GRAYSCALE_AD1_IOMUX                                 (IOMUX_PINCM57)
/* Defines for AD2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_GRAYSCALE_AD2_PORT                                          (GPIOB)
#define GPIO_GRAYSCALE_AD2_PIN                                  (DL_GPIO_PIN_23)
#define GPIO_GRAYSCALE_AD2_IOMUX                                 (IOMUX_PINCM51)
/* Defines for OUT: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_GRAYSCALE_OUT_PORT                                          (GPIOA)
#define GPIO_GRAYSCALE_OUT_PIN                                  (DL_GPIO_PIN_12)
#define GPIO_GRAYSCALE_OUT_IOMUX                                 (IOMUX_PINCM34)
/* Port definition for Pin Group GPIO_BTN */
#define GPIO_BTN_PORT                                                    (GPIOA)

/* Defines for PIN_START: GPIOA.29 with pinCMx 4 on package pin 36 */
#define GPIO_BTN_PIN_START_PIN                                  (DL_GPIO_PIN_29)
#define GPIO_BTN_PIN_START_IOMUX                                  (IOMUX_PINCM4)
/* Defines for PIN_CHANGE: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_BTN_PIN_CHANGE_PIN                                 (DL_GPIO_PIN_13)
#define GPIO_BTN_PIN_CHANGE_IOMUX                                (IOMUX_PINCM35)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_AB_init(void);
void SYSCFG_DL_PWM_MOTOR_CD_init(void);
void SYSCFG_DL_TIMER_PID_init(void);
void SYSCFG_DL_I2C_MPU6050_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
