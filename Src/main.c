
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t adcRead(void);
void adcEnableChannel(int channel);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t converted1;
uint32_t converted2;
uint32_t converted3;
uint32_t converted4;
uint32_t converted5;
static int msCount = 0;
int convertIT = 0;
int rsp[15] = {0, 2, 2, 1, 0, 1, 0, 0, 2, 1, 1, 2, 0, 2, 1};
int rspi = 0;
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  // CLONE SCRIPT
	  // finger 5
	  adcEnableChannel(1);
	  converted1 = adcRead();
	  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, converted1/25);
	  if (converted1 > 2100) {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 120);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	  }
	  // finger 4
	  adcEnableChannel(2);
	  converted2 = adcRead();
	  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, converted2/25);
	  if (converted2 > 2200) {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 120);
	  }
	  // finger 3
	  adcEnableChannel(3);
	  converted3 = adcRead();
	  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, converted3/25);
	  if (converted3 > 2300) {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 120);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 30);
	  }
	  // finger 2
	  adcEnableChannel(4);
	  converted4 = adcRead();
	  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, converted4/25);
	  if (converted4 > 2200) {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 70);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
	  }
	  // finger 1
	  adcEnableChannel(5);
	  converted5 = adcRead();
	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 120);
	  if (converted5 > 2200) {
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 120);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 30);
	  }
	  // TRANSLATION SCRIPT
	  if (convertIT == 1) {
		  convertIT = 0;
		  if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  // NOTHING
		  }
		  // a
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: a\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // b
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 > 2500) ) {
			  char alphabet[20] = "You entered: b\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // c
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: c\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // d
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: d\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // e
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: e\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // f
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: f\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // g
		  else if ( (converted1 > 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: g\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // h
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: h\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // i
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: i\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // j
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: j\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // k
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: k\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // l
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: l\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // m
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: m\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // n
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: n\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // o
		  else if ( (converted1 > 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: o\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
          // p
          else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
              char alphabet[20] = "You entered: p\n\r";
              HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
          }
		  // q
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: q\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // r
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: r\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // s
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: s\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // t
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: t\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // u
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: u\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // v
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: v\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // w
		  else if ( (converted1 < 2100) && (converted2 > 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: w\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // x
		  else if ( (converted1 < 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: x\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // y
		  else if ( (converted1 < 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 > 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "You entered: y\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // z
		  else if ( (converted1 < 2100) && (converted2 < 2200) && (converted3 > 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = "You entered: z\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // space
		  else if ( (converted1 < 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 > 2200) ) {
			  char alphabet[20] = " ";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
		  // newline
		  else if ( (converted1 < 2100) && (converted2 < 2200) && (converted3 < 2300) && (converted4 < 2200) && (converted5 < 2200) ) {
			  char alphabet[20] = "\n\r";
			  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
		  }
	  }
	  /*
	  // Rock Scissors Paper
	  if (convertIT == 1) {
		  convertIT = 0;
		  adcEnableChannel(1);
		  converted1 = adcRead();
		  adcEnableChannel(2);
		  converted2 = adcRead();
		  adcEnableChannel(3);
		  converted3 = adcRead();
		  adcEnableChannel(4);
		  converted4 = adcRead();
		  adcEnableChannel(5);
		  converted5 = adcRead();
		  if (rsp[rspi] == 0) {
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 120);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 30);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 30);
			  if ( converted1 > 2200 && converted2 > 2200 && converted3 > 2200 && converted4 > 2200 && converted5 > 2200 ) {
				  char alphabet[20] = "You won!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
			  else {
				  char alphabet[20] = "You lost!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
		  }
		  else if (rsp[rspi] == 1) {
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 120);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 120);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 120);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 30);
			  if ( converted1 < 2200 && converted2 < 2200 && converted3 < 2200 && converted4 < 2200 && converted5 < 2200 ) {
				  char alphabet[20] = "You won!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
			  else {
				  char alphabet[20] = "You lost!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
		  }
		  else if (rsp[rspi] == 2) {
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 120);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 30);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 120);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 120);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 120);
			  if ( converted1 < 2200 && converted2 < 2200 && converted3 > 2000 && converted4 > 2000 && converted5 < 2200 ) {
				  char alphabet[20] = "You won!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
			  else {
				  char alphabet[20] = "You lost!!\n\r";
				  HAL_UART_Transmit(&huart1, (uint8_t *)alphabet, strlen(alphabet), 10);
			  }
		  }
	  }
	  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 959;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 959;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// TIM3_ISR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		if (msCount == 3) {
			convertIT = 1;
			rspi = rspi + 1;
			if (rspi == 20) {
				rspi = 0;
			}
			GPIOC -> ODR ^= 0x00000300;
			msCount = 0;
		}
		else if (msCount == 2) {
			GPIOC -> ODR ^= 0x00000300;
			msCount = msCount + 1;
		}
		else {
			msCount = msCount + 1;
		}
	}
}
// ADC channel enable function
void adcEnableChannel(int channel) {
	// Wait for ADC to be ready
	while ((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
	// Ensure ADCStart = 0
	while ((ADC1 -> CR & ADC_CR_ADSTART) == 1);
	ADC1 -> CHSELR = 0;
	ADC1 -> CHSELR |= 1 << channel;
}
// ADC read function
uint32_t adcRead(void) {
	uint32_t adcValue = 0;
	// Wait for ADC to be ready
	while ((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
	// Start the ADC (ADCStart = 1)
	ADC1 -> CR |= ADC_CR_ADSTART;
	// Wait for end of conversion
	while ((ADC1 -> ISR & ADC_ISR_EOC) == 0);
	adcValue = ADC1 -> DR;

	return adcValue;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n\r", file, line) */
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
