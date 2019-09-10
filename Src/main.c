
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
/* Added name here: Luke Mills to test git*/
/*fuck git*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t A_Flag = 0;
uint8_t B_Flag = 0;
uint8_t HM_Flag = 0;
uint8_t Display_State = 0;
uint8_t HMTicks = 0;
uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours = 12;
uint8_t tx_buffer[10];
uint8_t rx_buffer[10];
uint8_t acc_buffer[6];
uint8_t gyro_buffer[6];
uint8_t success;
uint8_t readIMU_Flag = 0;

volatile uint8_t numbers[10][10] = {
								{1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								{0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
								{0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
								{0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
								{0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
								{0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
								{0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
								{0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
								{0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
								{0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
								};

struct tube {

	uint8_t NUM0;
	uint8_t NUM1;
	uint8_t NUM2;
	uint8_t NUM3;
	uint8_t NUM4;
	uint8_t NUM5;
	uint8_t NUM6;
	uint8_t NUM7;
	uint8_t NUM8;
	uint8_t NUM9;
};

struct tube tube1;	//initializes structs for tubes
struct tube tube2;

uint8_t sleep_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void update_time(void);
void update_tubes(void);
void update_display(void);
uint8_t accelerometer_initialization(void);
void enter_Standby(void);
uint8_t write_Reg(uint8_t Reg, uint8_t data);
uint8_t read_Reg(uint8_t Reg, uint8_t *data, uint16_t length);
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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim21);
  accelerometer_initialization();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  update_tubes();
	  update_display();

	  if(sleep_flag == 1)
	  {
		  sleep_flag = 0;
		  enter_Standby();
	  }

	  if(readIMU_Flag == 1)
	  {
		  success = read_Reg(0x0C, &gyro_buffer[0], 6);	//reads gyro xx yy zz
		  HAL_Delay(20);
		  success = read_Reg(0x12, &acc_buffer[0], 6);	//reads accelerometer xx yy zz
		  HAL_Delay(20);
		  readIMU_Flag = 0;
	  }

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00200C28;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 24000;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 999;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Tube1_Pin|Number7_Pin|Number2_Pin|Number4_Pin 
                          |Number3_Pin|Number0_Pin|Number6_Pin|Number8_Pin 
                          |Number5_Pin|Tube2_Pin|Neon2_Pin|Neon1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Number1_Pin|Number9_Pin|Charge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Wakeup_Pin */
  GPIO_InitStruct.Pin = Wakeup_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Wakeup_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Tube1_Pin Number7_Pin Number2_Pin Number4_Pin 
                           Number3_Pin Number0_Pin Number6_Pin Number8_Pin 
                           Number5_Pin Tube2_Pin Neon2_Pin */
  GPIO_InitStruct.Pin = Tube1_Pin|Number7_Pin|Number2_Pin|Number4_Pin 
                          |Number3_Pin|Number0_Pin|Number6_Pin|Number8_Pin 
                          |Number5_Pin|Tube2_Pin|Neon2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Number1_Pin Number9_Pin */
  GPIO_InitStruct.Pin = Number1_Pin|Number9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCSW_Pin */
  GPIO_InitStruct.Pin = ENCSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Neon1_Pin */
  GPIO_InitStruct.Pin = Neon1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Neon1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCB_Pin ENCA_Pin */
  GPIO_InitStruct.Pin = ENCB_Pin|ENCA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Charge_Pin */
  GPIO_InitStruct.Pin = Charge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Charge_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t toggle = 0;
	static uint8_t sleepTicks = 0;
	update_time();

	if (htim->Instance == TIM21)
	{

		if (Display_State == 1){
			sleepTicks++;	//"timer" to go back to sleep after wakeup just to view time
		}
		else{
			sleepTicks = 0;
		}

		if(Display_State == 1){

			toggle++;

			if(toggle >= 2){		//switch back and forth between hours and minutes in the
				HM_Flag = !HM_Flag;
				toggle = 0;
			}

			if(sleepTicks >= 20){	//implementation of sleep timer
				sleep_flag = 1;
				HAL_GPIO_WritePin(GPIOB, Charge_Pin, 0);
				Display_State = 0;
				sleepTicks = 0;
			}
		}

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static volatile uint8_t A_Flag = 0;
	static volatile uint8_t B_Flag = 0;
	static volatile uint8_t A_Previous = 0;
	static volatile uint8_t B_Previous = 0;
	RTC_TimeTypeDef sTimeSet;

    if (GPIO_Pin == ENCSW_Pin)
    {
    	Display_State++;
    }

	if(Display_State == 1){			//display hours and minutes
		HAL_GPIO_WritePin(GPIOB, Charge_Pin, 1);
	}
	else if(Display_State == 2){	//set hours
		HAL_GPIO_WritePin(GPIOB, Charge_Pin, 1);
		HM_Flag = 1;
	}
	else if(Display_State == 3){	//set minutes
		HAL_GPIO_WritePin(GPIOB, Charge_Pin, 1);
		HM_Flag = 0;
	}
	else{							//tubes off, MCU asleep
		Display_State = 0;
		HAL_GPIO_WritePin(GPIOB, Charge_Pin, 0);
		sleep_flag = 1;
	}

    if (Display_State != 1 && Display_State != 0){
		if (GPIO_Pin == ENCA_Pin)
		{
			A_Flag = HAL_GPIO_ReadPin(GPIOB, ENCA_Pin);

			if (A_Flag == 0 && B_Flag == 1 && A_Previous == 1 && B_Previous == 0){

				if(Display_State == 2){
					hours--;
					if(hours < 1){
						hours = 12;
					}
				}else if(Display_State == 3){
					if(minutes == 0){
						minutes = 60;	//just a quick hack to deal with not going negative
					}
					minutes--;
					if(minutes < 0){
						minutes = 59;
					}
				}else{
					//do nothing
				}

			}
			else if (A_Flag == 1 && B_Flag == 0 && A_Previous == 0 && B_Previous == 1){

				if(Display_State == 2){
					hours--;
					if(hours < 1){	//23
						hours = 12;	//23
					}
				}else if(Display_State == 3){
					if(minutes == 0){
						minutes = 60;	//just a quick hack to deal with not going negative
					}
					minutes--;
					if(minutes < 0){
						minutes = 59;
					}
				}else{
					//do nothing
				}
			}
			else if (A_Flag == 1 && B_Flag == 1 && A_Previous == 0 && B_Previous == 0){

				if(Display_State == 2){
					hours++;
					if(hours > 12){
						hours = 1;
					}
				}else if(Display_State == 3){
					minutes++;
					if(minutes > 59){
						minutes = 0;
					}
				}else{
					//do nothing
				}

			}
			else if (A_Flag == 0 && B_Flag == 0 && A_Previous == 1 && B_Previous == 1){

				if(Display_State == 2){
					hours++;
					if(hours > 12){
						hours = 1;
					}
				}else if(Display_State == 3){
					minutes++;
					if(minutes > 59){
						minutes = 0;
					}
				}else{
					//do nothing
				}
			}

			A_Previous = A_Flag;
			B_Previous = B_Flag;
			sTimeSet.Seconds = 0;
			sTimeSet.Minutes = minutes;
			sTimeSet.Hours = hours;
			HAL_RTC_SetTime(&hrtc, &sTimeSet, RTC_FORMAT_BIN);
		}
    }

    if (GPIO_Pin == ENCB_Pin)
    {
    	B_Flag = HAL_GPIO_ReadPin(GPIOB, ENCB_Pin);
    	if (A_Flag == 1 && B_Flag == 0 && A_Previous == 0 && B_Previous == 1){


    	}
    	else if (A_Flag == 0 && B_Flag == 0 && A_Previous == 1 && B_Previous == 1){


    	}
    }

}

void update_tubes(void)
{

	if(HM_Flag == 0){

		tube2.NUM0 = numbers[0][minutes%10];	//output values are that of the array in the minutes%10 position
		tube2.NUM1 = numbers[1][minutes%10];
		tube2.NUM2 = numbers[2][minutes%10];
		tube2.NUM3 = numbers[3][minutes%10];
		tube2.NUM4 = numbers[4][minutes%10];
		tube2.NUM5 = numbers[5][minutes%10];
		tube2.NUM6 = numbers[6][minutes%10];
		tube2.NUM7 = numbers[7][minutes%10];
		tube2.NUM8 = numbers[8][minutes%10];
		tube2.NUM9 = numbers[9][minutes%10];

		tube1.NUM0 = numbers[0][minutes/10];
		tube1.NUM1 = numbers[1][minutes/10];
		tube1.NUM2 = numbers[2][minutes/10];
		tube1.NUM3 = numbers[3][minutes/10];
		tube1.NUM4 = numbers[4][minutes/10];
		tube1.NUM5 = numbers[5][minutes/10];
		tube1.NUM6 = numbers[6][minutes/10];
		tube1.NUM7 = numbers[7][minutes/10];
		tube1.NUM8 = numbers[8][minutes/10];
		tube1.NUM9 = numbers[9][minutes/10];

		HAL_GPIO_WritePin(GPIOA, Neon2_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, Neon1_Pin, 0);
	}
	else{

		tube2.NUM0 = numbers[0][hours%10];
		tube2.NUM1 = numbers[1][hours%10];
		tube2.NUM2 = numbers[2][hours%10];
		tube2.NUM3 = numbers[3][hours%10];
		tube2.NUM4 = numbers[4][hours%10];
		tube2.NUM5 = numbers[5][hours%10];
		tube2.NUM6 = numbers[6][hours%10];
		tube2.NUM7 = numbers[7][hours%10];
		tube2.NUM8 = numbers[8][hours%10];
		tube2.NUM9 = numbers[9][hours%10];

		tube1.NUM0 = numbers[0][hours/10];	//multiplexer values is that of the array in that location
		tube1.NUM1 = numbers[1][hours/10];
		tube1.NUM2 = numbers[2][hours/10];
		tube1.NUM3 = numbers[3][hours/10];
		tube1.NUM4 = numbers[4][hours/10];
		tube1.NUM5 = numbers[5][hours/10];
		tube1.NUM6 = numbers[6][hours/10];
		tube1.NUM7 = numbers[7][hours/10];
		tube1.NUM8 = numbers[8][hours/10];
		tube1.NUM9 = numbers[9][hours/10];

		HAL_GPIO_WritePin(GPIOA, Neon1_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, Neon2_Pin, 0);
	}
}

void update_display(void)
{
	HAL_GPIO_WritePin(GPIOA, Tube2_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, Tube1_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, Number0_Pin, tube1.NUM0);
	HAL_GPIO_WritePin(GPIOB, Number1_Pin, tube1.NUM1);
	HAL_GPIO_WritePin(GPIOA, Number2_Pin, tube1.NUM2);
	HAL_GPIO_WritePin(GPIOA, Number3_Pin, tube1.NUM3);
	HAL_GPIO_WritePin(GPIOA, Number4_Pin, tube1.NUM4);
	HAL_GPIO_WritePin(GPIOA, Number5_Pin, tube1.NUM5);
	HAL_GPIO_WritePin(GPIOA, Number6_Pin, tube1.NUM6);
	HAL_GPIO_WritePin(GPIOA, Number7_Pin, tube1.NUM7);
	HAL_GPIO_WritePin(GPIOA, Number8_Pin, tube1.NUM8);
	HAL_GPIO_WritePin(GPIOB, Number9_Pin, tube1.NUM9);
	HAL_Delay(4);

	HAL_GPIO_WritePin(GPIOA, Tube1_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, Tube2_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, Number0_Pin, tube2.NUM0);
	HAL_GPIO_WritePin(GPIOB, Number1_Pin, tube2.NUM1);
	HAL_GPIO_WritePin(GPIOA, Number2_Pin, tube2.NUM2);
	HAL_GPIO_WritePin(GPIOA, Number3_Pin, tube2.NUM3);
	HAL_GPIO_WritePin(GPIOA, Number4_Pin, tube2.NUM4);
	HAL_GPIO_WritePin(GPIOA, Number5_Pin, tube2.NUM5);
	HAL_GPIO_WritePin(GPIOA, Number6_Pin, tube2.NUM6);
	HAL_GPIO_WritePin(GPIOA, Number7_Pin, tube2.NUM7);
	HAL_GPIO_WritePin(GPIOA, Number8_Pin, tube2.NUM8);
	HAL_GPIO_WritePin(GPIOB, Number9_Pin, tube2.NUM9);
	HAL_Delay(4);

}

void update_time(void)
{
	RTC_TimeTypeDef sTimeGet;
	RTC_TimeTypeDef sTimeSet;
	HAL_RTC_GetTime(&hrtc, &sTimeGet, RTC_FORMAT_BIN);
	seconds = sTimeGet.Seconds;
	minutes = sTimeGet.Minutes;
	hours = sTimeGet.Hours;

	if(hours>12){
		hours = 0;
		sTimeSet.Hours = hours;
		HAL_RTC_SetTime(&hrtc, &sTimeSet, RTC_FORMAT_BIN);
	}
}

uint8_t write_Reg(uint8_t Reg, uint8_t data)
{
	static uint8_t IMUAddress = 0b01101000;
	uint8_t packet[2] = {0};
	packet[0] = Reg;
	packet[1] = data;

	if(HAL_I2C_Master_Transmit(&hi2c1, IMUAddress<<1, packet, 2, 100) != HAL_OK)
	{
		return 0;
	}

	return 1;
}

uint8_t read_Reg(uint8_t Reg, uint8_t *data, uint16_t length)
{
	static uint8_t IMUAddress = 0b01101000;
	if(HAL_I2C_Master_Transmit(&hi2c1, IMUAddress<<1, &Reg, 1, 100) != HAL_OK)
	{
		return 0;
	}
	if(HAL_I2C_Master_Receive(&hi2c1, IMUAddress<<1, data, length, 100) != HAL_OK)
	{
		return 0;
	}
	return 1;

}

uint8_t accelerometer_initialization(void){

	uint8_t acc_pwr = 0x11;
	write_Reg(0x7E, acc_pwr);	//put acc into normal mode
	HAL_Delay(100);

	uint8_t gyro_pwr = 0x15;
	write_Reg(0x7E, gyro_pwr);	//put gyro into normal mode
	HAL_Delay(100);

	/*uint8_t mag_pwr = 0x18;	//put mag into normal mode
	write_Reg(0x7E, mag_pwr);
	HAL_Delay(100);*/

	write_Reg(0x40, 0x28);	//acc config
	HAL_Delay(100);

	write_Reg(0x41, 0x03);	//acc range
	HAL_Delay(100);

	write_Reg(0x42, 0x28);	//gyro config
	HAL_Delay(100);

	write_Reg(0x43, 0x03);	//gyro range
	HAL_Delay(100);

	write_Reg(0x75, 0xFF);	//gyro yy offset
	HAL_Delay(100);

	write_Reg(0x76, 0xFF);	//gyro zz offset
	HAL_Delay(100);

	read_Reg(0x03, &rx_buffer[1], 1);	//read PMU register
	HAL_Delay(100);

	write_Reg(0x50, 0x04);	//sets anymotion z enabled
	HAL_Delay(100);

	write_Reg(0x53, 0x08);	//sets INT1 enabled and active lo
	HAL_Delay(100);

	write_Reg(0x55, 0x04);	//sets INT1 to any motion/significant motion
	HAL_Delay(100);

	write_Reg(0x5F, 0x01);	//sets INT Motion[0] any motion trigger to 2 samples over threshold
	HAL_Delay(100);

	write_Reg(0x60, 0xFF);	//sets anymotion acc to trigger on ~1G
	HAL_Delay(100);

	write_Reg(0x69, 0x40);	//enables fast offset compensation for gyro
	HAL_Delay(100);

	return 1;

}

void enter_Standby( void )
{
    /* Enable Clocks */
    //RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Prepare for Standby */
    // if WKUP pins are already high, the WUF bit will be set
    //PWR->CSR |= PWR_CSR_EWUP1 | PWR_CSR_EWUP2;

    //PWR->CR |= PWR_CR_CWUF; // clear the WUF flag after 2 clock cycles
    //PWR->CR |= PWR_CR_ULP;   // V_{REFINT} is off in low-power mode
    //PWR->CR |= PWR_CR_PDDS; // Enter Standby mode when the CPU enters deepsleep

    //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // low-power mode = stop mode
    //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // reenter low-power mode after ISR
    //__WFI(); // enter low-power mode

	HAL_PWR_DisableSleepOnExit();
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

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
