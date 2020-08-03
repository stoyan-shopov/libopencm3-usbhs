/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "usb_device.h"
//#include "stdbool.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/timer.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

volatile int CUBE = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int yyy, zzz;
extern bool use_libopencm3_usb_code;
int main(void)
{
	//HAL_Init();
	rcc_periph_clock_enable(RCC_APB1ENR_PWREN);
	rcc_periph_clock_enable(RCC_APB2ENR_SYSCFGEN);
	rcc_clock_setup_hse(rcc_3v3 + RCC_CLOCK_3V3_216MHZ, 25000000);
	xMX_GPIO_Init();
	if (CUBE)
		MX_USB_DEVICE_Init();
	else
		xmain();

	init_timer2();
	xEnableUsbInterrupt();
	while (1)
	{
		if (!yyy ++)
			zzz ++;
	}
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
