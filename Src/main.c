/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************

  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lgs_bluetooth.h"
#include "lgs_datamanagement.h"


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void checkOutputActiveFlag(void);
void setDefaultValues(void);


#define TESTBOARD


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  //1 - Init GPIO:
  MX_GPIO_Init();
  //2 - Init BLE:
  LGS_BLE_Init();
  //3 - Init Datamanagement (EEPROM-Interface):
  LGS_DATAMANAGEMENT_Init(DATAMANAGEMENT_INTERVAL_TEST);


  setDefaultValues(); //TODO: Inhalt der Funktion in EEPROM verschieben

  while (1)
  {

#ifdef TESTBOARD
	  //Für Test Bluetooth- und EEPROM-Schnittstelle ohne Sensoren am Board:
	  LGS_GenerateRandomData(); //Schreibe zufällige neue Sensordaten in Datenstruktur
#endif

	  checkOutputActiveFlag();

	  LGS_BLE_Process();		 	//Nach checkOutputActiveFlag() aufrufen!

	  LGS_DATAMANAGEMENT_Process(); //WICHTIG: Als letztes in MainWhile aufrufen!
  }
}

/**
 * Setzt Default-Values
 * -> TODO: In EEPROM verschieben, sodass nicht mehr notwendig
 */
void setDefaultValues()
{
	//Set Default Values:
	m_environmentData.m_repRateBT 			= LGS_CYCLIC_SEND_INTERVAL_DEFAULT;
	m_environmentData.m_outputActive 		= LGS_DEFAULT_OUTPUT_ACTIVE;
	m_environmentData.m_criticTemperature 	= LGS_DEFAULT_CRITIC_TEMPERATURE;
	m_environmentData.m_criticVOC 			= LGS_DEFAULT_CRITIC_VOC;
	m_environmentData.m_criticCo2 			= LGS_DEFAULT_CRITIC_CO2;
	m_environmentData.m_criticHumidity 		= LGS_DEFAULT_CRITIC_HUMIDITY;
	m_environmentData.m_criticPressure 		= LGS_DEFAULT_CRITIC_PRESSURE;
}

/**
 * Prüft, ob das Flag für "Ausgang aktiv" gesetzt werden muss
 */
void checkOutputActiveFlag(void)
{
	if(		(m_environmentData.m_environmentTemperature 	> m_environmentData.m_criticTemperature)
			|| 	(m_environmentData.m_environmentVOC 		> m_environmentData.m_criticVOC)
			|| 	(m_environmentData.m_environmentCO2 		> m_environmentData.m_criticCo2)
			|| 	(m_environmentData.m_environmentAirHumidity > m_environmentData.m_criticHumidity)
			|| 	(m_environmentData.m_environmentAirPressure > m_environmentData.m_criticPressure))
	{
		if(m_environmentData.m_outputActive == 0U)
		{
			m_environmentData.m_isUpdateAvailable = 1U; 	//Update Available Flag
		}
		m_environmentData.m_outputActive = 1U;
	}
	else
	{
		if(m_environmentData.m_outputActive == 1U)
		{
			m_environmentData.m_isUpdateAvailable = 1U;	//Update Available Flag
		}
		m_environmentData.m_outputActive = 0U;
	}
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number*/
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
