/**
 ******************************************************************************
 * File     : VarmintInit.cpp
 * Date     : June 3, 2024
 ******************************************************************************
 *
 * Copyright (c) 2024, AeroVironment, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3.Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 **/
#include <Varmint.h>

#include <BoardConfig.h>
#include <Spi.h>
#include <Time64.h>
#include <misc.h>

#include <usb_device.h>
#include <usbd_cdc_if.h>

#include <ctime>
#include <main.h>

#include <Callbacks.h>
#include <Polling.h>
#include <sandbox.h>

bool verbose = true;

Time64 time64;

////////////////////////////////////////////////////////////////////////////////////////
//
// Varmint Board
//
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @fn void init_board(void)
 * @brief Board Initialization
 *
 */

void Varmint::init_board(void)
{
  uint32_t init_status;

  MPU_Config();
  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C1_Init();
  //	MX_SDMMC1_SD_Init(); // initialized elsewhere
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_SPI6_Init();

  //	MX_TIM1_Init();  // PWM, initialized elsewhere
  //	MX_TIM2_Init();	 // PWM (Buzzer) not used as such
  //	MX_TIM3_Init();  // RC PPM input, not used as such
  //	MX_TIM4_Init();  // PWM, initialized elsewhere
  //	MX_TIM8_Init();  // PWM, initialized elsewhere
  //  MX_TIM7_Init();  // Poll, initialized elsewhere
  //	MX_TIM5_Init();  // Time64, initialized elsewhere
  //	MX_TIM15_Init(); // Time64, initialized elsewhere

  //	MX_UART4_Init(); // GPS, initialized elsewhere
  //	MX_UART7_Init(); // Serial 5&6 connector, not used
  //	MX_UART8_Init(); // Telem "FRSKY" connector
  //	MX_USART1_UART_Init(); // Serial 5&6 connector, not used
  MX_USART2_UART_Init(); // Telem/Serial 1
                         //	MX_USART3_UART_Init(); // Serial 2 connector, not used
                         //	MX_USART6_UART_Init();

  //	MX_ADC3_Init(); // initialized elsewhere
  //	MX_ADC1_Init(); // initialized elsewhere

  MX_FDCAN1_Init(); // not used yet
  MX_FDCAN2_Init(); // not used yet

  //	MX_CRC_Init(); // not used
  //	MX_RNG_Init(); // not used
  //	MX_RTC_Init(); // not used

#if _USBD_USE_HS // USB HS (480MB/s
  MX_USB_OTG_HS_PCD_Init();
#else // USB FS (48 MB/s)
  MX_USB_OTG_FS_PCD_Init();
#endif
  MX_USB_DEVICE_Init();

  //// Startup Chained Timestamp Timers 1us rolls over in 8.9 years.
  misc_printf("\nStarted Timestamp Timer\n");
  init_status = time64.init(HTIM_LOW, HTIM_LOW_INSTANCE, HTIM_HIGH, HTIM_HIGH_INSTANCE);

#define ASCII_ESC 27
  misc_printf("%c[H", ASCII_ESC);  // home
  misc_printf("%c[2J", ASCII_ESC); // clear screen

  misc_printf("\nStarted Timestamp Timer\n");
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // IMU initialization

  HAL_NVIC_DisableIRQ(BMI088_INT4_GYRO_EXTI_IRQn);  // EXTI4_IRQn Gyro DRDY Feedback
  HAL_NVIC_DisableIRQ(BMI088_INT1_ACCEL_EXTI_IRQn); // EXTI1_IRQn ACCEL DRDY

  misc_printf("\n\nBMI088 (imu1) Initialization\n");
  init_status =
    imu0_.init(BMI088_HZ, BMI088_ACCEL_DRDY_GPIO_Port, BMI088_ACCEL_DRDY_Pin, BMI088_SPI, BMI088_ACCEL_CSn_GPIO_Port,
               BMI088_ACCEL_CSn_Pin, BMI088_GYRO_CSn_GPIO_Port, BMI088_GYRO_CSn_Pin, BMI088_RANGE_A, BMI088_RANGE_G);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pitot/Baro initialization

  misc_printf("\n\nMS4525 (Pitot) Initialization\n"); // I2C must already be initialized
  init_status = pitot_.init(PITOT_HZ, PITOT_I2C, PITOT_I2C_ADDRESS);
  misc_exit_status(init_status);

  misc_printf("\n\nDPS310 (baro) Initialization\n");
  init_status = baro_.init(DPS310_HZ, DPS310_SPI, DPS310_CSn_GPIO_Port, DPS310_CSn_Pin // SPI
  );

  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Mag initialization

  misc_printf("\n\nIST3808 (mag) Initialization\n");
  init_status = mag_.init(IST3808_HZ, IST3808_I2C, IST3808_I2C_ADDRESS);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // GPS initialization

  misc_printf("\n\nUbx (gps) Initialization\n");
  init_status = gps_.init(GPS_HZ, GPS_PPS_PORT, GPS_PPS_PIN, GPS_HAS_PPS, GPS_UART, GPS_UART_INSTANCE, GPS_UART_DMA,
                          GPS_BAUD, UBX_PROTOCOL);

  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RC/S.Bus initialization

  misc_printf("\n\nS.Bus (rc) Initialization\n");
  init_status = rc_.init(RC_HZ, RC_UART, RC_UART_INSTANCE, RC_UART_DMA, RC_BAUD);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ADC initialization

  misc_printf("\n\nAdc (adc) Initialization\n");
  init_status =
    adc_.init(ADC_HZ, ADC_ADC_EXTERNAL, ADC_ADC_INSTANCE_EXTERNAL, ADC_ADC_INTERNAL, ADC_ADC_INSTANCE_INTERNAL);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // COM initialization

  misc_printf("\n\nVcp (vcp) Initialization\n");
  init_status = vcp_.init(VCP_HZ);
  misc_exit_status(init_status);

  misc_printf("\n\nTelem (telem) Initialization\n");
  init_status = telem_.init(TELEM_HZ, TELEM_UART, TELEM_UART_INSTANCE, TELEM_UART_DMA, TELEM_BAUD);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PWM initialization

  misc_printf("\n\nPWM (PWM) Initialization\n");
  init_status = pwm_.init();

  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Servo Power Supply initialization

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // uSD Card initialization
  misc_printf("\n\nSDMMC Initialization\n");
  init_status = sd_.init(SD_HSD, SD_HSD_INSTANCE);
  misc_exit_status(init_status);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Interrupt initializations

  misc_printf("\n\nSet-up EXTI IRQ's\n");
  verbose = false;

  HAL_NVIC_EnableIRQ(BMI088_INT4_GYRO_EXTI_IRQn);  // EXTI4_IRQn Gyro DRDY Feedback
  HAL_NVIC_EnableIRQ(BMI088_INT1_ACCEL_EXTI_IRQn); // EXTI1_IRQn ACCEL DRDY

  __HAL_UART_ENABLE_IT(gps_.huart(), UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(rc_.huart(), UART_IT_IDLE);

  telem_.rxStart(); // Also enables its interrupts.

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // High Rate Timer initialization

  misc_printf("\n\nPolling Timer Initialization\n");
  init_status = InitPollTimer(POLL_HTIM, POLL_HTIM_INSTANCE, POLL_TIM_CHANNEL);
  misc_exit_status(init_status);

  RED_LO;
  GRN_LO;
  BLU_LO;

  PROBE1_LO;
  PROBE2_LO;
  PROBE3_LO;
  PROBE4_HI;

#if SANDBOX
  misc_printf("\n\nStarting Sandbox\n");
  sandbox();
#else
  misc_printf("\n\nStarting Rosflight\n");
  verbose = false;
#endif
}
