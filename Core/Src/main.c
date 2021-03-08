/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
//#include "stdlib.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "ds18b20.h"
#include "gfx.h"
#include "oled.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ZAMERS 126
#define RXBUFFLEN 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t ledCounter = 0;
volatile uint32_t screenCounter = 0;
volatile bool f_updTemperat = true;
volatile bool f_nextScreen = false;
volatile uint8_t f_graph = 2;
volatile bool f_second_start = true;
volatile bool f_firsStart = false;
volatile bool f_startChecksum = false;
volatile uint8_t checksum = 0;

static bool f_RX = false;
static bool f_temperValid = false;
static float FifoTemperOf24H[ZAMERS];		 //температура
static int measureTemperat = 0;
static float maxTemper = -100.0, minTemper = 200.0;
volatile static uint32_t lastMeasurTime = 0;
extern Ds18b20Sensor_t ds18b20[_DS18B20_MAX_SENSORS];
char strokeLCD[70] =
{ 0 };
char strokeTo8266[16] =
{ 0 };
char daysOfTheWeek[7][10] =
{ "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
char mounts[12][4] =
{ "jan", "feb", "mar", "apr", "jun", "jul", "aug", "sep", "oct", "nov", "dec" };

volatile int RXindex = 0;
uint8_t RX;
union
{
	uint8_t RXbuff90[RXBUFFLEN];
} data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void init();
void searchEXTR();
void getTemperat();
void lcdGraph();
void time_lcd();
void lcdTemperat();
char *ftoa(float f);
void sendTemper();
void esp8266_usartRX();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct
{
	RTC_DateTypeDef DATE;
	RTC_TimeTypeDef TIME;
	bool f_setTime;
} TIME_DATE =
{ 0 };

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(SENS_POWER_GPIO_Port, SENS_POWER_Pin, SET);
	oled_init();
	Ds18b20_Init();
	init();
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (f_updTemperat)
		{
			getTemperat();
			sendTemper();
			f_updTemperat = false;
		}

		if (f_nextScreen && !f_second_start)
		{
			if (f_graph == 1)
			{
				lcdGraph();
				f_graph = 2;
				f_nextScreen = false;
			}
			else if (f_graph == 2)
			{
				lcdTemperat();
				f_graph = 3;
				f_nextScreen = false;
			}
			else if (f_graph == 3)
			{
				time_lcd();
				f_graph = 1;  ///////// ----------------  2
				f_nextScreen = false;
			}
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void init()
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart2, &RX, 1);

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
		esp8266_usartRX();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//------------------------------------------------------------------------------------------------
{
	if (htim == &htim2)
	{
		//ledCounter++;
		if (ledCounter++ >= 300)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			f_updTemperat = true;
			ledCounter = 0;
		}

		if (screenCounter++ == 250)
		{
			f_nextScreen = true;
			screenCounter = 0;
		}

		if (lastMeasurTime++ > 68572)  //68572при 100в сек //685714 1000в сек
		{
			lastMeasurTime = 0;
			if (measureTemperat++ == (ZAMERS - 1))
				measureTemperat = 0;
		}
	}
}
//------------------------------------------------------------------------------------------------
void lcdTemperat()
{
	searchEXTR();
	clear();
	graphics_text(0, 0, FONT_NINE_DOT, "MAX: ");
	memset(data.RXbuff90, 0, sizeof(data.RXbuff90));
	graphics_text(63, 0, FONT_NINE_DOT, ftoa(maxTemper));
	graphics_text(0, 11, FONT_NINE_DOT, "MIN: ");
	memset(data.RXbuff90, 0, sizeof(data.RXbuff90));
	graphics_text(63, 11, FONT_NINE_DOT, ftoa(minTemper));
	graphics_text(0, 22, FONT_NINE_DOT, "CURR: ");
	memset(data.RXbuff90, 0, sizeof(data.RXbuff90));
	graphics_text(63, 22, FONT_NINE_DOT, ftoa(FifoTemperOf24H[measureTemperat]));
	oled_update();
}
//------------------------------------------------------------------------------------------------
void time_lcd()
{
	clear();
	uint32_t a = HAL_GetTick();
	static uint32_t hours = 0, min = 0, sec = 0;
	do
	{
		if (a >= 3600000)
		{
			hours++;
			a -= 3600000;
			continue;
		}
		else if (a >= 60000)
		{
			min++;
			a -= 60000;
			continue;
		}/*  в жопу секунды
		 else if (a >= 1000)
		 {
		 sec++;
		 a -= 1000;
		 continue;
		 }
		 */
	} while (a >= 60000);

	memset(strokeLCD, 0, sizeof strokeLCD);
	sprintf(strokeLCD, "%s%u%c%u", "Uptime: ", hours, ':', min);
	hours = min = sec = 0;

	graphics_text(0, 0, FONT_NINE_DOT, strokeLCD);
	oled_update();

	if (TIME_DATE.f_setTime)
	{
		HAL_RTC_GetDate(&hrtc, &TIME_DATE.DATE, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &TIME_DATE.TIME, RTC_FORMAT_BIN);
		graphics_text(0, 11, FONT_NINE_DOT, daysOfTheWeek[TIME_DATE.DATE.WeekDay]);

		memset(strokeLCD, 0, sizeof strokeLCD);
		sprintf(strokeLCD, "%u%c%s%c%u", TIME_DATE.DATE.Date, ':', mounts[TIME_DATE.DATE.Month-1], ':',	TIME_DATE.DATE.Year+2000);
 		graphics_text(0, 22, FONT_NINE_DOT, strokeLCD);
		oled_update();
	}
}
//------------------------------------------------------------------------------------------------
void set_time()
{
	uint8_t mountday = 0;
	if(sscanf(data.RXbuff90+1, "%u;%u;%u;%u;%u;%u;%u;%u", &TIME_DATE.DATE.WeekDay, &mountday, &TIME_DATE.DATE.Month, &TIME_DATE.DATE.Year, &TIME_DATE.TIME.Hours, &TIME_DATE.TIME.Minutes, &TIME_DATE.TIME.Seconds)> 1)
	{
		TIME_DATE.DATE.Date = mountday;
		HAL_RTC_SetDate(&hrtc, &TIME_DATE.DATE, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &TIME_DATE.TIME, RTC_FORMAT_BIN);
		TIME_DATE.f_setTime = true;
	}
}
//------------------------------------------------------------------------------------------------
void sendTemper()
{
	f_temperValid = 1;
	if(f_temperValid)
	{
		memset(strokeTo8266, 0, sizeof(strokeTo8266));
		sprintf(strokeTo8266, "%c%s%c", '$', ftoa(FifoTemperOf24H[measureTemperat]), '\n');
		//HAL_UART_Transmit(&huart2, (uint8_t*) strokeTo8266, sizeof(strokeTo8266), 10);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)strokeTo8266, sizeof(strokeTo8266));
		f_temperValid = false;
	}
}
//------------------------------------------------------------------------------------------------
void esp8266_usartRX()
{
	if (RX == '$' || f_RX)
	{
		f_RX = true;
		data.RXbuff90[RXindex++] = RX;

		if (RXindex == (RXBUFFLEN - 1) && data.RXbuff90[RXindex - 1] != '\n')
		{
			f_RX = false;
			RXindex = 0;
		}
		/*
		 if (RXindex <= (RXBUFFLEN - 1) || data.RXbuff90[RXindex - 1] != '\n')
		 {
		 if (data.RXbuff90[RXindex - 2] == '$' && data.RXbuff90[RXindex - 1] != '$')
		 f_startChecksum = true;

		 if (f_startChecksum && data.RXbuff90[RXindex - 1] != '*')
		 checksum ^= RX;
		 else
		 f_startChecksum = false;
		 }

		 else
		 {
		 f_RX = false;
		 RXindex = 0;
		 if (data.dataRAW.crc == checksum)
		 {
		 set_time();
		 checksum = 0;
		 memset(data.RXbuff90, 0, sizeof(data.RXbuff90));
		 }
		 }*/

		else if (data.RXbuff90[RXindex - 1] == '\n')
		{
			set_time();
			memset(data.RXbuff90, 0, sizeof(data.RXbuff90));
		}

	}
	HAL_UART_Receive_IT(&huart2, &RX, 1);
}

//------------------------------------------------------------------------------------------------
void lcdGraph()
{
	clear();
	line_v(0, 31, 0, 1, add);
	line_v(0, 31, 127, 1, add);
	line_h(0, 127, 0, 1, add);
	line_h(0, 127, 31, 1, add);

	if (minTemper <= 0)
	{
		uint8_t Y = (uint8_t) round(map(0.0, minTemper, maxTemper, 30.0, 1.0));
		line_h(1, 126, Y, 1, add);
	}
	if (maxTemper == minTemper)
	{
		line_h(1, 126, 16, 1, add);
	}
	else
	{
		static int last_y;
		static int yy;
		for (int i = measureTemperat, idxLCDbuff = ZAMERS + 1; idxLCDbuff > 0; idxLCDbuff--, i--) //!
		{
			if (i < 0)
				i = ZAMERS;
			float a = map(FifoTemperOf24H[i], minTemper, maxTemper, 29.0, 2.0);
			uint8_t y = (uint8_t) round((double) a);

			if (idxLCDbuff < ZAMERS + 1)
			{
				yy = last_y - y;
				yy = abs(yy);
				if (yy != 0)
				{
					for (; yy > 0; yy--)
						if (last_y > y)
						{
							draw_pixel(idxLCDbuff, last_y - yy, add);
						}
						else
						{
							draw_pixel(idxLCDbuff, last_y + yy, add);
						}
				}
			}
			last_y = y;
			draw_pixel(idxLCDbuff, y, add);
		}
	}
	oled_update();
}
//------------------------------------------------------------------------------------------------
void getTemperat()
{
	Ds18b20_ManualConvert();
	if (f_firsStart)
	{
		f_firsStart = false;
		f_second_start = true;
	}
	else if (f_second_start)
	{
		FifoTemperOf24H[measureTemperat] = ds18b20[0].Temperature;
		for (int i = 0; i < ZAMERS; i++)
			FifoTemperOf24H[i] = FifoTemperOf24H[measureTemperat];
		f_second_start = false;
	}
	else
		FifoTemperOf24H[measureTemperat] = ds18b20[0].Temperature;

	if(FifoTemperOf24H[measureTemperat]> -40 && FifoTemperOf24H[measureTemperat]<70)
		f_temperValid = true;
}
//------------------------------------------------------------------------------------------------
void searchEXTR()
{
	maxTemper = -100.0;
	minTemper = 200.0;
	for (int q = 0; q < ZAMERS; q++) //находим минимум и максимум
	{
		if (maxTemper < FifoTemperOf24H[q])
			maxTemper = FifoTemperOf24H[q];
		if (minTemper > FifoTemperOf24H[q])
			minTemper = FifoTemperOf24H[q];
	}
}
//------------------------------------------------------------------------------------------------
char *ftoa(float f)
{
	static char buf[10];
	char * cp = buf;
	unsigned long l, rem;

	if (f < 0)
	{
		*cp++ = '-';
		f = -f;
	}
	l = (unsigned long) f;
	f -= (float) l;
	rem = (unsigned long) (f * 1e1);
	sprintf(cp, "%lu.%1.1lu", l, rem);
	return buf;
}
//------------------------------------------------------------------------------------------------

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
