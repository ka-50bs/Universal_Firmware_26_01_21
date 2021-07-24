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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

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

/* Size of 'DMA_Buf' */
#define DMA_Size 200

/* Half of size of 'DMA_Buf' */
#define DMA_Half 100

/* Buffer for DMA */
uint16_t DMA_Buf[DMA_Size];

/* Size of 'CDC_Buf' */
#define CDC_Size 5000

/* Number of parts of 'DMA_Buf' in 'CDC_Buf' */
#define CDC_Parts CDC_Size / DMA_Half

/* Buffer for CDC transmission */
uint16_t CDC_Buf[CDC_Size];

/* Part of 'CDC_Buf' for memcpy in Callback */
uint16_t CDC_part = 0;

/**
 * Flag of ADC status.
 * If == 0 <=> Ready for sending in VCP
 * If == 1 <=> ADC is busy, conversion
 */
uint8_t ADC_Status = 0;

/**
 * Flag of CDC status.
 * If == 0 <=> 'CDC_Buf' isn't full, transmission denied
 * If == 1 <=> 'CDC_Buf' is full, transmission allow
 */
uint8_t CDC_Status = 0;

/**
 * Flag of CDC enable transmission status, marked from PC VCP.
 * If == 0 <=> Sending data isn't called
 * If == 1 <=> Asking for sending data in VCP
 */
uint8_t CDC_Request = 0;

/* Size of 'RX_Buf' */
#define RX_Buf_Size 64

/* VCP RX input buffer */
char RX_Buf[RX_Buf_Size];

/* Level of triggering in trigger mode (0-4095) */
uint16_t Trig_level = 500;

/**
 * Flag of trigger status in trigger mode
 * If == 0 <=> Trigger module isn't triggered, writing 0 CDC_part of CDC_Buf
 * If == 1 <=> Trigger is triggered, writing 1-2 CDC_part of CDC_Buf
 */
uint16_t Trig_status = 0;

/**
 * Modes of module working
 * If == 0 <=> Sleep mode
 * If == 1 <=> Trace mode
 * If == 2 <=> True Trigger mode
 * If == 3 <=> Auto Trigger mode
 */
uint8_t Mode = 0;

/* Mods constants */
#define Sleep_Mode_Num 			0
#define Trace_Mode_Num 			1
#define True_Trigger_Mode_Num 	2
#define Auto_Trigger_Mode_Num 	3
#define Set_Gain_Num			4
#define Set_Trigger_Level_Num	5
#define Request_Num				6

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Function of start symbol transmission */
void Start_Symbol()
{
	uint16_t mess = 0xFFFF;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
}

/* Function of stop symbol transmission */
void Stop_Symbol()
{
	uint16_t mess = 0xFFFE;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
}

/**
 * @brief	Function of True Trigger Mode
 * If module is triggered in "For" cycle, It fills CDC_Buf with ADC conversion to transmit by VCP.
 * If module isn't triggered in "For"  cycle, It do nothing.
 */
void True_Trigger_Mode()
{
	Trig_status = 0;
	ADC_Status = 1;
	for (int i = 0; i<100; i++){}
	for (int i = 0; i<10000;i++)
	{
		if(HAL_ADC_GetValue(&hadc1) > Trig_level)
		{
			Trig_status = 1;
			while(Trig_status == 1){}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			return ;
		}
	}
}

/**
 * @brief	Function of Auto Trigger Mode
 * If module is triggered in "For" cycle, It fills CDC_Buf with ADC conversion to transmit by VCP.
 * If module isn't triggered in "For"  cycle, It triggered forced after cycle.
 * This mode is useful to setting modules.
 */
void Auto_Trigger_Mode()
{
	ADC_Status = 1;
	Trig_status = 0;
	for (int i = 0; i<100; i++){}
	for (int i = 0; i<10000;i++)
	{
		if(HAL_ADC_GetValue(&hadc1) > Trig_level)
		{
			Trig_status = 1;
			while(Trig_status == 1){}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			return ;
		}
	}
	Trig_status = 1;
	while(Trig_status == 1){}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	return ;
}

/**
 * @brief	Function of transmitting data by VCP
 * Send start symbol, CDC_Transmit_Size amount of dots, stop symbol
 * @param CDC_Transmit_Size: number of dots of transmitting data
 */
void CDC_Send_Data(uint16_t CDC_Transmit_Size)
{
	if((CDC_Status == 1)&&(CDC_Request == 1))
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		Start_Symbol();
		while(CDC_Transmit_FS((uint8_t*)&CDC_Buf, sizeof(uint16_t)*CDC_Transmit_Size) == USBD_BUSY);
		Stop_Symbol();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		CDC_Status = 0;
		CDC_Request = 0;
	 }
}

/**
 * @brief	Function of setting voltage of DAC MCP4921
 * @param	val: value of gain on DAC, val = 0..4095
 */
void DAC_Write(uint16_t val)
{
	/* Init transmittion data: 4 register bits + 12 value bits */
	/* See MCP4921 datasheet */
	uint16_t dac_data = 0x7000 | (val&0x0FFF);
	uint8_t data[2];
	memcpy(&data, &dac_data, sizeof(data));

	/* SPI CS Low, SPI transmitt, SPI CS High */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, sizeof(data), 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

/**
 * @brief	Function of parser in "while" cycle of firmware
 * Mode is main variable for choosing function_mode
 */
void While_Parser()
{
	switch(Mode)
	{
	case Sleep_Mode_Num:
		__NOP();
		break;

	case Trace_Mode_Num:
		CDC_Send_Data(CDC_Size);
		break;

	case True_Trigger_Mode_Num:
		True_Trigger_Mode();
		CDC_Send_Data(DMA_Half*3);
		Mode = 0;
		break;

	case Auto_Trigger_Mode_Num:
		Auto_Trigger_Mode();
		CDC_Send_Data(DMA_Half*3);
		Mode = 0;
		break;
	}
}

/**
 * @brief	Function of CDC_ReciveCallBack parsing
 * @param	mode: mode bit
 * @param	val: value for set mode
 */
void CDC_Callback_Parser(uint8_t mode, uint16_t val)
{
	switch(mode)
	{
	case Sleep_Mode_Num:
		Mode = mode;
		break;

	case Trace_Mode_Num:
		Mode = mode;
		break;

	case True_Trigger_Mode_Num:
		Mode = mode;
		break;

	case Auto_Trigger_Mode_Num:
		Mode = mode;
		break;

	case Set_Gain_Num:
		DAC_Write(val);
		break;

	case Set_Trigger_Level_Num:
		Trig_level = val;
		break;

	case Request_Num:
		CDC_Request = 1;
		break;
	}
}

/**
 * @brief	Function of HAL_ADC_ConvCpltCallback(DMA) for Auto and True Trigger mode
 * @param	half: half of active DMA buffer fo copy
 */
void Trigger_DMA_Callback(uint8_t half)
{
	if(ADC_Status == 1)
	{
		if(half == 0)
			memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[0],sizeof(uint16_t)*DMA_Half );
		else
			memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[DMA_Half],sizeof(uint16_t)*DMA_Half );

		if(Trig_status == 1)
			CDC_part = CDC_part + 1;
		else
			CDC_part = 0;
		if(CDC_part == 3)
		{
			CDC_Status = 1;
			ADC_Status = 0;
			CDC_part = 0;
			Trig_status = 0;
		}
	}
}

/**
 * @brief	Function of HAL_ADC_ConvCpltCallback(DMA) for Trace mode
 * @param	half: half of active DMA buffer fo copy
 */
void Trace_DMA_Callback(uint8_t half)
{
	if(ADC_Status == 1)
	{
		if(half == 0)
			memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[0],sizeof(uint16_t)*DMA_Half );
		else
			memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[DMA_Half],sizeof(uint16_t)*DMA_Half );

		CDC_part = CDC_part + 1;
		if(CDC_part == CDC_Parts)
		{
			CDC_Status = 1;
			ADC_Status = 0;
			CDC_part = 0;
			HAL_ADC_Stop_DMA(&hadc1);
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Calibration of ADC (hadc1) for extra accuracy */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Start first DMA cycle */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) DMA_Buf, DMA_Size);

  /* Init PC13 State. Light is off. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  While_Parser();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == GPIO_PIN_3)&&(ADC_Status == 0)&&(CDC_Status == 0))
	{
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) DMA_Buf, DMA_Size);
		ADC_Status = 1;
	}
	else
	{
		__NOP();
	}
}

void CDC_ReciveCallBack(uint8_t *Buf, uint32_t *Len)
{
	memset(RX_Buf, 0, RX_Buf_Size);
	memcpy(RX_Buf, Buf, Len);

	uint16_t CDC_RX_Len = Len;
	uint8_t Mode_Byte = RX_Buf[0];
	uint16_t Value_Bytes = atoi(&RX_Buf[1]);
	CDC_Callback_Parser(Mode_Byte, Value_Bytes);
	RX_Buf[CDC_RX_Len] = '\n';
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(Mode == Trace_Mode_Num)
		Trace_DMA_Callback(1);
	if((Mode == Auto_Trigger_Mode_Num)||(Mode == True_Trigger_Mode_Num))
		Trigger_DMA_Callback(1);
	/*
	if(ADC_Status == 1)
	{
		memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[DMA_Half],sizeof(uint16_t)*DMA_Half );

		if((Mode == Trace_Mode_Num)||(Trig_status == 1))
			CDC_part = CDC_part + 1;

		if((Trig_status == 1)&&(CDC_part == 3))
			CDC_part = CDC_Parts;

		if(CDC_part == CDC_Parts)
		{
			if(Mode == Trace_Mode_Num)
				HAL_ADC_Stop_DMA(&hadc1);

			CDC_Status = 1;
			ADC_Status = 0;
			CDC_part = 0;
			if(Trig_status == 1)
				Trig_status = 0;
		}
	}
	*/
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(Mode == Trace_Mode_Num)
			Trace_DMA_Callback(0);
	if((Mode == Auto_Trigger_Mode_Num)||(Mode == True_Trigger_Mode_Num))
		Trigger_DMA_Callback(0);
	/*
	if(ADC_Status == 1)
	{
		memcpy(&CDC_Buf[CDC_part*DMA_Half], &DMA_Buf[0],sizeof(uint16_t)*DMA_Half );

		if((Mode == Trace_Mode_Num)||(Trig_status == 1))
			CDC_part = CDC_part + 1;

		if((Trig_status == 1)&&(CDC_part == 3))
			CDC_part = CDC_Parts;

		if(CDC_part == CDC_Parts)
		{
			if(Mode == Trace_Mode_Num)
				HAL_ADC_Stop_DMA(&hadc1);

			CDC_Status = 1;
			ADC_Status = 0;
			CDC_part = 0;
			if(Trig_status == 1)
				Trig_status = 0;
		}
	}
	*/
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
