/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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

static const uint16_t adc_offset = 1973;
//static const uint32_t dac_offset = 2048;
#define dac_offset	2048

#define N_SAMPLES 	16000

volatile uint16_t adc_buffer[N_SAMPLES];
volatile uint16_t dac_buffer[N_SAMPLES];

extern volatile int anc_enabled;
volatile int anc_acquire_cnt = N_SAMPLES;
volatile static float   u1_[2];
volatile static float	u_[2] = {0};
volatile static int16_t u[3] = {0};
volatile static int16_t e[3] = {0};

volatile static int16_t adc[2] = {1973, 1973};

volatile static int16_t sendbuf[2];

#define SENDBUF_SIZE 	(2 * sizeof(sendbuf[0]))

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

#define MEAN_UPCNT	 8
	static uint8_t mean_cnt			= MEAN_UPCNT;
	static int32_t e_mean 			= 0;
	static int32_t u_mean 			= 0;

	static const float b[2] = {0.9941, -0.9941};
	static const float a = {0.9883};

	static const float c[3] = {0.9681, -1.6481, 0.8425};
	static const float d[2] = {-0.3146, 0.2502};

	static const int16_t u_max = 4095 - dac_offset;
	static const int16_t u_min = (int16_t)0 - (int16_t)dac_offset;

	LL_GPIO_SetOutputPin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin);

	/* End of ADC2 conversion interrupt */
	if (LL_ADC_IsActiveFlag_EOCS(ADC2))
	{
		/* High-pass filter and error calculation */
		adc[0] = - ((int16_t)LL_ADC_REG_ReadConversionData12(ADC2));
		e[0] = (int16_t)(b[0] * adc[0] + b[1] * adc[1] + a * e[1]);

		if (anc_enabled)
		{
			/* calculate u from coefficients */
			u[0] = c[0] * e[0] + c[1] * e[1] + c[2] * e[2] + d[0] * u[1] + d[1] * u[2];
			/* Saturate u */
			if (u[0] > u_max)
				u[0] = u_max;
			else if (u[0] < u_min)
				u[0] = u_min;

			/* Set output DAC with offset */
			LL_DAC_ConvertData12RightAligned(DAC, LL_DAC_CHANNEL_2, (uint32_t)(u[0] + dac_offset));
		}
		else
		{
			u[0] = 0;
		}
		if (anc_acquire_cnt < N_SAMPLES)
		{
			adc_buffer[anc_acquire_cnt] = (uint16_t)adc[0];
			dac_buffer[anc_acquire_cnt] = (uint16_t)(u[0] + dac_offset);
			anc_acquire_cnt++;
		}
		/* Sending via UART every 8 averaged samples */
		e_mean += e[0];
		u_mean += u[0];
		--mean_cnt;
		if (mean_cnt == 0)
		{
			/* Calculate mean from 8 samples */
			sendbuf[0] = e_mean / MEAN_UPCNT;
			sendbuf[1] = u_mean / MEAN_UPCNT;
			/* Send via UART with DMA */
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)sendbuf, SENDBUF_SIZE);
			/* Clean averaging operation */
			e_mean = 0;
			u_mean = 0;
			mean_cnt = MEAN_UPCNT;
		}

		/* Save last e and u */
		adc[1] = adc[0];

		e[2] = e[1];
		e[1] = e[0];

		u[2] = u[1];
		u[1] = u[0];
	}
  /* USER CODE END ADC_IRQn 0 */
  
  /* USER CODE BEGIN ADC_IRQn 1 */

	LL_GPIO_ResetOutputPin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin);

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
