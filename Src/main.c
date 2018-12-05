
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include "ssd1306.h"
#include <string.h>
#include "New_CalibriLight14.h"

#define DRIVE HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==GPIO_PIN_SET
#define REAR  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)==GPIO_PIN_SET
#define LEFT  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)==GPIO_PIN_SET
#define RIGHT HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==GPIO_PIN_SET

#define LED_MAIN_ON				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_MAIN_OFF			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)

#define LED_YELLOW_ON			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_YELLOW_OFF  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_YELLOW_TOG   	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

#define REED_BTN_1 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==GPIO_PIN_RESET
#define REED_BTN_2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_RESET

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char str1[20]={0};
uint8_t buf1[20]={0}; // Буфер на отправку
extern uint8_t RX_BUF[TX_PLOAD_WIDTH+1]; // Буфер приема

volatile uint16_t ADC_Data[10]; // Для хранения данных из DMA

float ADC_1_Percent;  // АЦП Руля
float ADC_2_Percent;	// АЦП Газа
float ADC_3_Percent;
float ADC_4_Percent;
char str[20];
float Volt = 0.0; 		// Переменная напряжения в пульте
uint16_t MenuDelay=0;	// Задержка работы меню, обновления дисплея
uint8_t OptionFlag = 0; // 1 - Если перешли в режим настройки

struct But {
	uint8_t ButEvent; // 1-Короткий клик, 2-Длиный клик.
	uint8_t ButDown; // 1-Если кнопка сейчас нажата.
	uint8_t ButFlag; // 1-Если сработало нажатие
	uint8_t ButEventTime; // Время, которое висит собитек кнопки.
	uint16_t ButDownTime;//время удержания кнопки.
	uint16_t ButUpTime;
};
extern struct But But1; // Машина состояний кнопки энкодера
uint8_t Menu = 21;			// Переменная для уровней меню
uint8_t OldParam, NewParam; //Старые и новые значения настраиваемых данных
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void MainMenu (void);
uint8_t SetResBits (uint8_t set, uint8_t value, uint8_t pos);
uint8_t CheckBits (uint8_t value, uint8_t pos);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint8_t dt; // Переменная для количества попыток отправления.
	static uint16_t i=1;
	uint16_t ADC_1, ADC_1_sr, ADC_1_max=0, ADC_1_min=4096;
	uint16_t ADC_2, ADC_2_sr, ADC_2_max=0, ADC_2_min=4096;
	
	uint8_t LedFlashNumb = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);
	
	
	i=0;
	while ((LCD_init() !=1)||i<50){
		i++;
		LedFlashNumb++;
		if (LedFlashNumb == 1) {
			LED_GREEN_ON;
			LED_YELLOW_OFF;
			LED_MAIN_OFF;
		} else if (LedFlashNumb == 2) {
			LED_GREEN_OFF;
			LED_YELLOW_ON;
			LED_MAIN_OFF;
		} else if (LedFlashNumb == 3) {
			LED_GREEN_OFF;
			LED_YELLOW_OFF;
			LED_MAIN_ON;
		} else if (LedFlashNumb > 3) LedFlashNumb = 0;
	}
		
	LED_MAIN_ON;
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data,5);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	TIM1->CNT = 2; // Стартовое положение энкодера;
	
	NRF24_ini();
	HAL_Delay(10);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	
	ADC_1_max=3700;	
	ADC_1_min=205;
	ADC_1_sr = 50; //50%
	
	
	ADC_2_max=2800;	 
	ADC_2_min=1150;
	ADC_2_sr = 65; // 44%

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
		if (MenuDelay>5){
			MainMenu();
			MenuDelay = 0;
		}
		MenuDelay++;
		
		ADC_1 = ADC_Data[1];
		ADC_1_Percent=100-((100.0/(ADC_1_max-ADC_1_min))*(ADC_1-ADC_1_min));
		
		ADC_2 = ((float)ADC_Data[0]);
		ADC_2_Percent=(100.0/(ADC_2_max-ADC_2_min))*(ADC_2-ADC_2_min);
		
		Volt = 0.00278 * ADC_Data[2];
		
		if (OptionFlag == 0) { // если не находимся в режиме настроек.
		
			if (ADC_1_Percent > 100) ADC_1_Percent = 100;
			if (ADC_1_Percent < 0) ADC_1_Percent = 0;
			
			if (ADC_1_Percent < ADC_1_sr - 5){
					buf1[0] = 100  + ((ADC_1_sr - 5 - ADC_1_Percent)*1.12*2);
			}
			if (ADC_1_Percent > ADC_1_sr + 5){
					buf1[0] = 100 - 0 + ((ADC_1_sr + 5 - ADC_1_Percent)*1.12*2);
			}
			if (ADC_1_Percent >ADC_1_sr - 5 && ADC_1_Percent<ADC_1_sr + 5){
				buf1[0] = 100;
			}
			
			if (ADC_2_Percent > 100) ADC_2_Percent = 100;
			
			if (ADC_2_Percent < 0) ADC_2_Percent = 0;
			
			if (ADC_2_Percent < ADC_2_sr - 5){
					buf1[1] = 100 + ((ADC_2_sr - 5 - ADC_2_Percent)*0.835*2);
				}
			if (ADC_2_Percent > ADC_2_sr + 5){
					buf1[1] = 100 + ((ADC_2_sr + 5 - ADC_2_Percent)*1.68*2);
					if (buf1[1] <= 0) buf1[1] = 0;
				}		
			if (ADC_2_Percent >ADC_2_sr - 5 && ADC_2_Percent<ADC_2_sr + 5){
				buf1[1] = 100;
			}
			
			if (REED_BTN_1) {
				buf1[2] = SetResBits(1,buf1[2],1);
			} else {
				buf1[2] = SetResBits(0,buf1[2],1);
			}
			if (REED_BTN_2) {
				buf1[2] = SetResBits(1,buf1[2],2);
			} else {
				buf1[2] = SetResBits(0,buf1[2],2);
			}
			dt = NRF24L01_Send(buf1);
			LED_YELLOW_OFF;
		}
		else {
			HAL_Delay(10);
			LED_YELLOW_ON;
		}
		

		if ((Menu>20 && Menu<30)||OptionFlag==1)
			NRF24L01_Receive();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 127;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_YELLOW_Pin|LED_RED_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_YELLOW_Pin LED_RED_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin|LED_RED_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 BTN_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_2_Pin */
  GPIO_InitStruct.Pin = BTN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin== GPIO_PIN_10)
  {
    IRQ_Callback();
  }
  else
  {
    __NOP();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim==&htim2)
  {
    TIM1_Callback();
  }
}

void MainMenu (void){
	char str[20];
	float VoltRx = 0;
	uint8_t i=0;
	
	if (But1.ButEvent == 1 && (Menu == 11 || Menu == 21 || Menu == 31)) {
		Menu += 10;
		if (Menu > 40)
			Menu -= 30;
		But1.ButEvent = 0;
	}
//======================= Первый уровень меню ==========================
	if (Menu>10 && Menu<20) {
	
		LCD_RectFill(1,1,32,128,0x00);

		LCD_BarHorizont(1,1,5,50,buf1[0]/2,0x01,0x00);
		sprintf(str,"Ch1 = %d", buf1[0]);
		StringCalibriLight14(6,1,str,0x01,0x02);
		
		LCD_BarHorizont(1,60,5,50,buf1[1]/2,0x01,0x00);
		sprintf(str,"Ch2 = %d", buf1[1]);
		StringCalibriLight14(6,60,str,0x01,0x02);
		
		sprintf(str,"Длит. наж - Настройка");
		StringCalibriLight14(20,1,str,0x01,0x02);
		buf1[4] = 0;
//---------------------- Переход в подменю -------------------------------		
		if (But1.ButEvent == 2 && Menu == 11) {
			LCD_RectFill(1,1,32,128,0x00);
			StringCalibriLight14(1,1,"Для выхода длинно",0x01,0x02);	
			StringCalibriLight14(14,1,"нажмите кнопку",0x01,0x02);
			PrintBuf();
			Menu++;
			OptionFlag = 1; // Перешли в режим настройки
			HAL_Delay(1500);
			if (Menu>14) Menu = 12;
			But1.ButEvent = 0;
		}
//---------------------- Переход в подменю -------------------------------
//-------------------------- Подменю 1 -----------------------------------		
		if (Menu == 12){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Max влево=%d", RX_BUF[0]);
			StringCalibriLight14(6,1,str,0x01,0x02);
			buf1[4] = 2; // 4 - настройка центра рулевой машинки.
			NRF24L01_Send(buf1);
			HAL_Delay(3);
			
			if (RX_BUF[0] > 0 && RX_BUF[4] == 2) { // если данные от приемника пришли.
				if (TIM1->CNT == 1) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] + 1;
				}
				if (TIM1->CNT == 3) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] - 1;
				}
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 11;
				OptionFlag = 0;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 13;
				RX_BUF[0] = 0;
				RX_BUF[4] = 0;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 1 -----------------------------------
//-------------------------- Подменю 2 -----------------------------------		
		if (Menu == 13){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Max вправо=%d", RX_BUF[0]);
			StringCalibriLight14(6,1,str,0x01,0x02);
			buf1[4] = 3; // 4 - настройка центра рулевой машинки.
			NRF24L01_Send(buf1);
			HAL_Delay(3);
			
			if (RX_BUF[0] > 0 && RX_BUF[4] == 3) { // если данные от приемника пришли.
				if (TIM1->CNT == 1) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] + 1;
				}
				if (TIM1->CNT == 3) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] - 1;
				}
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 11;
				OptionFlag = 0;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 14;
				RX_BUF[0] = 0;
				RX_BUF[4] = 0;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 2 -----------------------------------
//-------------------------- Подменю 3 -----------------------------------		
		if (Menu == 14){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Центр - %d", RX_BUF[0]);
			StringCalibriLight14(6,60,str,0x01,0x02);
			buf1[4] = 5; // 5 - настройка центра рулевой машинки.
			NRF24L01_Send(buf1);
			HAL_Delay(3);
			
			if (RX_BUF[0] > 0 && RX_BUF[4] == 5) { // если данные от приемника пришли.
				if (TIM1->CNT == 1) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] + 1;
				}
				if (TIM1->CNT == 3) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] - 1;
				}
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 11;
				OptionFlag = 0;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 15;
				RX_BUF[0] = 0;
				RX_BUF[4] = 0;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 3 -----------------------------------
//-------------------------- Подменю 4 -----------------------------------		
		if (Menu == 15){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Макс. мощ - %d", RX_BUF[0]);
			StringCalibriLight14(6,1,str,0x01,0x02);
			buf1[4] = 6; // 4 - настройка центра рулевой машинки.
			NRF24L01_Send(buf1);
			HAL_Delay(3);
			
			if (RX_BUF[0] > 0 && RX_BUF[4] == 6) { // если данные от приемника пришли.
				if (TIM1->CNT == 1) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] + 1;
				}
				if (TIM1->CNT == 3) {
					TIM1->CNT = 2;
					buf1[0] = RX_BUF[0] - 1;
				}
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 11;
				OptionFlag = 0;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 12;
				RX_BUF[0] = 0;
				RX_BUF[4] = 0;
				But1.ButEvent = 0;
			}
		}
	}
//-------------------------- Подменю 4 -----------------------------------
//======================= Первый уровень меню ==========================

//======================= Второй уровень меню ==========================
	if (Menu>20 && Menu<30) {
		LCD_RectFill(1,1,32,128,0x00);
		sprintf(str,"Батарея пульта = %.1f В", Volt);
		StringCalibriLight14(1,1,str,0x01,0x02);
		
//		LCD_BarHorizont(1,60,5,50,(uint8_t)100 - (100/1.7)*(6 - Volt),0x01,0x00);
		
		memcpy(&VoltRx, RX_BUF, 4);
		sprintf(str,"Батарея борта = %.1f В", VoltRx);
		StringCalibriLight14(20,1,str,0x01,0x02);
		buf1[4] = 1;
	}
//======================= Второй уровень меню ==========================	

//======================= Третий уровень меню ==========================
	if (Menu>30 && Menu<40) {
		LCD_RectFill(1,1,32,128,0x00);
		sprintf(str,"Дополнительные каналы");
		StringCalibriLight14(1,1,str,0x01,0x02);
		sprintf(str,"Длит. наж - Настройка");
		StringCalibriLight14(20,1,str,0x01,0x02);
		buf1[4] = 0;
//---------------------- Переход в подменю -------------------------------		
		if (But1.ButEvent == 2 && Menu == 31) {
			LCD_RectFill(1,1,32,128,0x00);
			Menu++;
			if (Menu>35) Menu = 32;
			But1.ButEvent = 0;
		}
//---------------------- Переход в подменю -------------------------------
//-------------------------- Подменю 1 -----------------------------------		
		if (Menu == 32){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Ch1 = %d", CheckBits(buf1[2],0));
			StringCalibriLight14(2,3,str,0x01,0x02);
			LCD_Rect(1,1,15,41,0x01);
			sprintf(str," Для выхода");
			StringCalibriLight14(1,50,str,0x01,0x02);
			sprintf(str,"длит. нажатие");
			StringCalibriLight14(14,50,str,0x01,0x02);
			
			if (TIM1->CNT == 1) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(1,buf1[2],0);
			}
			if (TIM1->CNT == 3) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(0,buf1[2],0);
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 31;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 33;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 1 -----------------------------------
//-------------------------- Подменю 2 -----------------------------------		
		if (Menu == 33){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Ch2 = %d", CheckBits(buf1[2],1));
			StringCalibriLight14(2,3,str,0x01,0x02);
			LCD_Rect(1,1,15,41,0x01);
			sprintf(str," Для выхода");
			StringCalibriLight14(1,50,str,0x01,0x02);
			sprintf(str,"длит. нажатие");
			StringCalibriLight14(14,50,str,0x01,0x02);
			
			if (TIM1->CNT == 1) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(1,buf1[2],1);
			}
			if (TIM1->CNT == 3) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(0,buf1[2],1);
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 31;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 34;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 2 -----------------------------------
//-------------------------- Подменю 3 -----------------------------------		
		if (Menu == 34){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Ch3 = %d", CheckBits(buf1[2],2));
			StringCalibriLight14(2,3,str,0x01,0x02);
			LCD_Rect(1,1,15,41,0x01);
			sprintf(str," Для выхода");
			StringCalibriLight14(1,50,str,0x01,0x02);
			sprintf(str,"длит. нажатие");
			StringCalibriLight14(14,50,str,0x01,0x02);
			
			if (TIM1->CNT == 1) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(1,buf1[2],2);
			}
			if (TIM1->CNT == 3) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(0,buf1[2],2);
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 31;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 35;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 3 -----------------------------------
//-------------------------- Подменю 4 -----------------------------------		
		if (Menu == 35){
			LCD_RectFill(1,1,32,128,0x00);
			sprintf(str,"Ch4 = %d", CheckBits(buf1[2],3));
			StringCalibriLight14(2,3,str,0x01,0x02);
			LCD_Rect(1,1,15,41,0x01);
			sprintf(str," Для выхода");
			StringCalibriLight14(1,50,str,0x01,0x02);
			sprintf(str,"длит. нажатие");
			StringCalibriLight14(14,50,str,0x01,0x02);
			
			if (TIM1->CNT == 1) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(1,buf1[2],3);
			}
			if (TIM1->CNT == 3) {
				TIM1->CNT = 2;
				buf1[2] = SetResBits(0,buf1[2],3);
			}
			if (TIM1->CNT<1 || TIM1->CNT>3) TIM1->CNT = 2;
			
			if (But1.ButEvent==2){
				Menu = 31;
				But1.ButEvent = 0;
			}
			if (But1.ButEvent==1){
				Menu = 32;
				But1.ButEvent = 0;
			}
		}
//-------------------------- Подменю 4 -----------------------------------
	}
	PrintBuf();
}
//======================= Третий уровень меню ==========================

void Display (void) {
	
};

uint8_t SetResBits (uint8_t set, uint8_t value, uint8_t pos){ // set==1 - Установить бит
	if (set) {
		return  value |(1 << pos);
	}
	else {
		return value & (~(1 << pos));
	}
}

uint8_t CheckBits (uint8_t value, uint8_t pos){
	if (value & (1<<pos))
		return 1;
	else 
		return 0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
