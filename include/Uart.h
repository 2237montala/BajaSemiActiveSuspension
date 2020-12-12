#pragma once
/*
 * HEADER NAME : Uart.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/09/2020
 * DESCRIPTION : Contains function headers for the UART device
 */

#include "targetCommon.h"


int UART_Init(UART_HandleTypeDef *UartHandle);

/*
 * PURPOSE
 *      Puts a arry of uint8 onto the UART interface. This function is blocking
 * PARAMETERS
 *      huart - A pointer to the hardware uart interface that will be used to send the data
 *      ptr   - A pointer to the data that will be sent
 *      len   - The number of bytes to be sent
 * RETURNS
 *      int - the number of bytes sent
 */
int UART_putData(UART_HandleTypeDef *huart,uint8_t ptr, int len);

/*
 * PURPOSE
 *      Sends a null terminated string over the UART interface. This function is blocking
 * PARAMETERS
 *      huart - A pointer to the hardware uart interface that will be used to send the data
 *      str   - A pointer to a null terminated string
 * RETURNS
 *      int - the number of bytes sent 
 */
int UART_putString(UART_HandleTypeDef *huart,char *str);

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart);