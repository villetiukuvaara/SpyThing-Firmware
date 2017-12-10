/**
 ******************************************************************************
 * @file    FatFs/FatFs_uSD/Src/main.c
 * @author  MCD Application Team
 * @version V1.8.0
 * @date    21-April-2017
 * @brief   Main program body
 *          This sample code shows how to use FatFs with uSD card drive.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright(c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "logger.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	FRESULT res;                                          /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	//uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
	uint8_t rtext[100];                                   /* File read buffer */

	/* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
	 */
	HAL_Init();

	/* Configure the system clock to 80 MHz */
	SystemClock_Config();

	/* Configure LED1 and LED3 */
	BSP_LED_Init(LED1);
	//BSP_LED_Init(LED3);

	/*for(;;)
	{
		BSP_LED_Toggle(LED1);
		HAL_Delay(300);
	}*/

	/*##-1- Link the micro SD disk I/O driver ##################################*/
	if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	{
		/*##-2- Register the file system object to the FatFs module ##############*/
		if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
		{
			/* FatFs Initialization Error */
			Error_Handler();
		}
		else
		{
			/*##-3- Create a FAT file system (format) on the logical drive #########*/
			/* WARNING: Formatting the uSD card will delete all content on the device */
			if(f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK)
			{
				/* FatFs Format Error */
				Error_Handler();
			}
			else
			{
				/*##-4- Create and Open a new text file object with write access #####*/
				if(f_open(&MyFile, "AUDIO.WAV", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
				{
					/* 'STM32.TXT' file Open for write Error */
					Error_Handler();
				}
				else
				{
					/*##-5- Write data to the text file ################################*/
					//res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
					wave_sample_t samples_0[] = {0x1122, 0x3344};
					wave_sample_t samples_1[] = {0x5566, 0x7788};
					wave_sample_t* samples[2];
					samples[0] = samples_0;
					samples[1] = samples_1;

					logger_wav_write_header(&MyFile, 22050, 2, 512);
					res = logger_wav_append_nchannels(&MyFile, 2, 2, samples);

					byteswritten = f_tell(&MyFile);

					/*##-6- Close the open text file #################################*/
					if (f_close(&MyFile) != FR_OK )
					{
						Error_Handler();
					}

					if((byteswritten == 0) || (res != FR_OK))
					{
						/* 'STM32.TXT' file Write or EOF Error */
						Error_Handler();
					}
					else
					{
						/*##-7- Open the text file object with read access ###############*/
						if(f_open(&MyFile, "AUDIO.WAV", FA_READ) != FR_OK)
						{
							/* 'STM32.TXT' file Open for read Error */
							Error_Handler();
						}
						else
						{
							/*##-8- Read data from the text file ###########################*/
							res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

							if((bytesread == 0) || (res != FR_OK))
							{
								/* 'STM32.TXT' file Read or EOF Error */
								Error_Handler();
							}
							else
							{
								/*##-9- Close the open text file #############################*/
								f_close(&MyFile);

								/*##-10- Compare read data with the expected data ############*/
								if((bytesread != byteswritten))
								{
									/* Read data is different from the expected data */
									Error_Handler();
								}
								else
								{
									/* Success of the demo: no error occurrence */
									BSP_LED_On(LED1);
								}
							}
						}
					}
				}
			}
		}
	}

	/*##-11- Unlink the RAM disk I/O driver ####################################*/
	FATFS_UnLinkDriver(SDPath);


	/* Infinite loop */
	while (1)
	{
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follows :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 80000000
 *            HCLK(Hz)                       = 80000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 1
 *            PLL_N                          = 20
 *            PLL_P                          = 7
 *            PLL_Q                          = 4
 *            PLL_R                          = 2
 *            Flash Latency(WS)              = 4
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
	//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	//
	//  /* Enable HSE Oscillator and activate PLL with HSE as source   */
	//  /* (Default MSI Oscillator enabled at system reset remains ON) */
	//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	//  RCC_OscInitStruct.PLL.PLLM = 1;
	//  RCC_OscInitStruct.PLL.PLLN = 20;
	//  RCC_OscInitStruct.PLL.PLLR = 2;
	//  RCC_OscInitStruct.PLL.PLLP = 7;
	//  RCC_OscInitStruct.PLL.PLLQ = 4;
	//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	//  {
	//    /* Initialization Error */
	//    while(1);
	//  }
	//
	//  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	//     clocks dividers */
	//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	//  {
	//    /* Initialization Error */
	//    while(1);
	//  }

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure LSE Drive Capability
	 */
	//__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
	//		|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		while(1);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		while(1);
	}

	//PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
	//		|RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_SAI1
	//		|RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
	//		|RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
	//		|RCC_PERIPHCLK_SDMMC1;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
	//PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
	//PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_HSI;
	//PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	//PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
	//PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	//PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
	//PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	//PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		while(1);
	}

	//HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);

	/**Enables the Clock Security System
	 */
	 //HAL_RCCEx_EnableLSECSS();

	/**Configure the main internal regulator output voltage
	 */
	 if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	 {
		 //_Error_Handler(__FILE__, __LINE__);
		 while(1);
	 }

	 /**Configure the Systick interrupt time
	  */
	 HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	 /**Configure the Systick
	  */
	 HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	 /**Enable MSI Auto calibration
	  */
	 HAL_RCCEx_EnableMSIPLLMode();

	 /* SysTick_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	while(1)
	{
		/* Toggle LED3 fast */
		BSP_LED_Toggle(LED1);
		HAL_Delay(40);
	}
}




#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}

#endif


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
