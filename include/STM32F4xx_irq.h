#pragma once
/* Includes ------------------------------------------------------------------*/
#include "targetCommon.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
