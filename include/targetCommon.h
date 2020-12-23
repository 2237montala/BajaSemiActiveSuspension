#pragma once
/*
 * HEADER NAME : targetCommon.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/03/2020
 * DESCRIPTION :
 *      Holds all the includes that are special for the system being compiled
 *      for
 */

#include "nucleo_f446re.h"

/*
 * These settings enable the ST-Link on the nucleo board to be a serial adapter
 * Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for CANx clock resources */
// TX is pb9 , rx is pb8
#define CANx                           CAN1
#define CANx_CLK_ENABLE()              __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()

#define CANx_FORCE_RESET()             __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()           __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for CANx Pins */
#define CANx_TX_PIN                    GPIO_PIN_9
#define CANx_TX_GPIO_PORT              GPIOB
#define CANx_TX_AF                     GPIO_AF9_CAN1
#define CANx_RX_PIN                    GPIO_PIN_8
#define CANx_RX_GPIO_PORT              GPIOB
#define CANx_RX_AF                     GPIO_AF9_CAN1
