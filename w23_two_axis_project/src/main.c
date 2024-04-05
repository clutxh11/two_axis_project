/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

//test git

 
#include "example.h"
#include "example_usart.h"
#include "L6470.h"
#include "xnucleoihm02a1_interface.h"

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();
  
#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
  	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
#endif
  
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();

  void ADC1_Config(void) {
  // configure ADC
  ADC_ChannelConfTypeDef Config = {0};
  Config.Channel = ADC_CHANNEL_1;
  Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  Config.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &Config);
}

void setADC1_Value(uint16_t rawADCValue) {  
  // Define the output range
  const uint16_t outputMin = 4000;
  const uint16_t outputMax = 26000;

   L6470_Run(1, L6470_DIR_REV_ID, 0);

  if (rawADCValue > 152) {
      // Define the input range
      const uint16_t inputMin = 152;
      const uint16_t inputMax = 255;
      uint16_t modifedValue = ((rawADCValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)) + outputMin; 

      L6470_Run(0, L6470_DIR_FWD_ID, modifedValue); // Toggle LED 1
    } 
  else if (rawADCValue < 107) {
      // Define the input range
    const uint16_t inputMin = 0;
    const uint16_t inputMax = 107;
    uint16_t modifedValue = ((rawADCValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)) + outputMin; 

    L6470_Run(0, L6470_DIR_REV_ID, modifedValue); // Turn off LED 1
  } 
  else {
    L6470_Run(0, L6470_DIR_REV_ID, 0); // Turn off LED 1
  }
}

void setADC2_Value(uint16_t rawADCValue) {
 // Define the output range
  const uint16_t outputMin = 4000;
  const uint16_t outputMax = 26000;

   L6470_Run(0, L6470_DIR_REV_ID, 0);

  if (rawADCValue > 152) {
      // Define the input range
      const uint16_t inputMin = 152;
      const uint16_t inputMax = 255;
      uint16_t modifedValue = ((rawADCValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)) + outputMin; 

      L6470_Run(1, L6470_DIR_FWD_ID, modifedValue); // Toggle LED 1
    } 
  else if (rawADCValue < 107) {
      // Define the input range
    const uint16_t inputMin = 0;
    const uint16_t inputMax = 107;
    uint16_t modifedValue = ((rawADCValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)) + outputMin; 

    L6470_Run(1, L6470_DIR_REV_ID, modifedValue); // Turn off LED 1
  } 
  else {
    L6470_Run(1, L6470_DIR_REV_ID, 0); // Turn off LED 1
  }
}

  uint16_t rawPoten;
  char uart_msg1[20];
  char uart_msg2[20];

  MX_ADC1_Init();

  /* Configure the GPIO_ADC pin */
  GPIO_InitTypeDef  GPIO_InitStruct_ADC;

  __HAL_RCC_ADC1_CLK_ENABLE();  //ADC1
  __HAL_RCC_GPIOA_CLK_ENABLE();  //GPIOA

  GPIO_InitStruct_ADC.Pin = GPIO_PIN_1;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_ADC.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_ADC
  );

  // Motor switch
  GPIO_InitTypeDef  GPIO_InitStruct_M_SW;

  GPIO_InitStruct_M_SW.Pin = GPIO_PIN_5;
  GPIO_InitStruct_M_SW.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_M_SW.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_M_SW
  );

  // Y - Axis and Motor 1
  GPIO_InitTypeDef  GPIO_InitStruct_Limit_Switch1;
  GPIO_InitStruct_Limit_Switch1.Pin = GPIO_PIN_7;
  GPIO_InitStruct_Limit_Switch1.Pull = GPIO_PULLUP;
  GPIO_InitStruct_Limit_Switch1.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Limit_Switch1);

  GPIO_InitTypeDef  GPIO_InitStruct_Limit_Switch4;
  GPIO_InitStruct_Limit_Switch4.Pin = GPIO_PIN_8;
  GPIO_InitStruct_Limit_Switch4.Pull = GPIO_PULLUP;
  GPIO_InitStruct_Limit_Switch4.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Limit_Switch4);

  // X - Axis and Motor 0
  GPIO_InitTypeDef  GPIO_InitStruct_Limit_Switch2;
  GPIO_InitStruct_Limit_Switch2.Pin = GPIO_PIN_6;
  GPIO_InitStruct_Limit_Switch2.Pull = GPIO_PULLUP;
  GPIO_InitStruct_Limit_Switch2.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Limit_Switch2);
  
  
  GPIO_InitTypeDef  GPIO_InitStruct_Limit_Switch3;
  GPIO_InitStruct_Limit_Switch3.Pin = GPIO_PIN_9;
  GPIO_InitStruct_Limit_Switch3.Pull = GPIO_PULLUP;
  GPIO_InitStruct_Limit_Switch3.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Limit_Switch3);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));


  /* Infinite loop */
  while (1)
  {
    // Get Potentiometer 1 value
    ADC1_Config();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    rawPoten = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);


    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1) 
    {
      setADC2_Value(rawPoten);

      sprintf(uart_msg1, "Raw ADC1: %lu\r\n", rawPoten); // Assuming %lu is for unsigned long
      // Send UART message
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg1, strlen(uart_msg1), HAL_MAX_DELAY);
    }
    else 
    {
      // Get Potentiometer 2 value
      setADC1_Value(rawPoten);

      sprintf(uart_msg2, "Raw ADC2: %lu\r\n", rawPoten); // Assuming %lu is for unsigned long
      // Send UART message
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg2, strlen(uart_msg2), HAL_MAX_DELAY);
    }

    USART_CheckAppCmd();
  }
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
