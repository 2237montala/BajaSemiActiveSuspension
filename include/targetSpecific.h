#pragma once

#include <Bsp.h>

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

// Definition for UART used for software testing data
// Tx : PC12 , Rx : PD2
#define SOFTWARE_TEST_UART                           UART4
#define SOFTWARE_TEST_UART_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE();
#define SOFTWARE_TEST_UART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define SOFTWARE_TEST_UART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 

#define SOFTWARE_TEST_UART_FORCE_RESET()             __HAL_RCC_UART4_FORCE_RESET()
#define SOFTWARE_TEST_UART_RELEASE_RESET()           __HAL_RCC_UART4_RELEASE_RESET()

/* Definition for UARTx Pins */
#define SOFTWARE_TEST_UART_TX_PIN                    GPIO_PIN_10
#define SOFTWARE_TEST_UART_TX_GPIO_PORT              GPIOC  
#define SOFTWARE_TEST_UART_TX_AF                     GPIO_AF8_UART4
#define SOFTWARE_TEST_UART_RX_PIN                    GPIO_PIN_11
#define SOFTWARE_TEST_UART_RX_GPIO_PORT              GPIOC 
#define SOFTWARE_TEST_UART_RX_AF                     GPIO_AF8_UART4

/* Definition for CANx clock resources */
// TX is pb9 , rx is pb8
#define CANx                           CAN1
#define CANx_CLK_ENABLE()              __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define CANx_FORCE_RESET()             __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()           __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for CANx Pins */
#define CANx_TX_PIN                    GPIO_PIN_9
#define CANx_TX_GPIO_PORT              GPIOB
#define CANx_TX_AF                     GPIO_AF9_CAN1
#define CANx_RX_PIN                    GPIO_PIN_8
#define CANx_RX_GPIO_PORT              GPIOB
#define CANx_RX_AF                     GPIO_AF9_CAN1

// Interrupt priorities for the devices used in the project
#define TIM6_IRQ_PRIORITY 0xA
#define CAN1_TX_IRQ_PRIORITY 0x8
#define CAN1_RX0_IRQ_PRIORITY 0x8
#define CAN1_RX1_IRQ_PRIORITY 0x8