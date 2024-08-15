 /******************************************************************************
  * @file    v.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    30-Novermber-2018
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023  SmartSite Limited  
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

#include "vm201.h"
#include "timeServer.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
//bool flags_command_check=0;
//uint8_t rxdatacheck[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
extern UART_HandleTypeDef UartHandle1;
extern uint8_t aRxBuffer[1];

void VM201_read(VM201_reading_t *tfsensor_reading)
{
  uint8_t rxdata[19] ={
		0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00};
	int retry = 0;
	VM201_IoInit();
	HAL_Delay(500);	
	vm201_data_receive(rxdata,500);
	HAL_Delay(500);	
  VM201_IoDeInit();
	tfsensor_reading->checkBit = (int)rxdata[2];
	tfsensor_reading->frequency = (int)((rxdata[3]<<8)+rxdata[4]);
	tfsensor_reading->frequency_detail_high = (int)((rxdata[5]<<8)+rxdata[6]);
	tfsensor_reading->frequency_detail_low = (int)((rxdata[7]<<8)+rxdata[8]);
	tfsensor_reading->temperature = (int)(rxdata[15]*256+rxdata[16]);
}

void VM201_Init(void)
{
	VM201_uart1_Init();
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1);	
	HAL_Delay(1000); 
}
/************************ (C) COPYRIGHT SmartSite Limited *****END OF FILE****/