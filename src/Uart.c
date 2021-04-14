#include "Uart.h"
#include "string.h"

int UART_Init(UART_HandleTypeDef *UartHandle) {
    return HAL_UART_Init(UartHandle);
}

int UART_putData(UART_HandleTypeDef *huart,uint8_t *ptr, int len) {
    return HAL_UART_Transmit(huart, ptr, len, 0xFFFF); 
}

int UART_putString(UART_HandleTypeDef *huart,char *str) {
    return HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 0xFFFF); 
}

int UART_putStringNL(UART_HandleTypeDef *huart,char *str) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 0xFFFF); 
    status += HAL_UART_Transmit(huart, (uint8_t *)NL, strlen(NL), 0xFFFF); 

    return status;
}

int UART_readData(UART_HandleTypeDef *huart,uint8_t *ptr, uint32_t len) {
    HAL_StatusTypeDef status = HAL_UART_Receive(huart,ptr,len,0xFFFF);
    return status;
}

int UART_readLine(UART_HandleTypeDef *huart,uint8_t *ptr, uint32_t len, char newLine, uint32_t *bytesRead) {
    // This function is a modified version of HAL_UART_Receive
    // Since we need to check every character from the new line char
    uint16_t *tmp;
    uint32_t tickstart = 0U;
    uint32_t bytesReadIn = 0;
    HAL_StatusTypeDef statusReturn = HAL_ERROR;

    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if ((ptr == NULL) || (len == 0U))
        {
            statusReturn = HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);

        // Check if we are using parity because we do not support that currently
        if(huart->Init.WordLength == UART_WORDLENGTH_8B) {
            huart->ErrorCode = HAL_UART_ERROR_NONE;
            huart->RxState = HAL_UART_STATE_BUSY_RX;

            /* Init tickstart for timeout managment */
            tickstart = HAL_GetTick();

            huart->RxXferSize = len;
            huart->RxXferCount = len;

            /* Process Unlocked */
            __HAL_UNLOCK(huart);

            /* Check the remain data to be received */
            while (huart->RxXferCount > 0U)
            {
                huart->RxXferCount--;
                if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart, 0xFFFF) != HAL_OK) {
                    statusReturn = HAL_TIMEOUT;
                }

                if (huart->Init.Parity == UART_PARITY_NONE) {
                    *ptr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
                    bytesReadIn++;

                    // Check if new char is the new line char
                    uint8_t *tempPtr = ptr;
                    if(--*tempPtr == newLine) {
                        // Stop reading in new chars
                        huart->RxXferCount = 0;
                    }
                }
            }

            /* At end of Rx process, restore huart->RxState to Ready */
            huart->RxState = HAL_UART_STATE_READY;

            *bytesRead = bytesReadIn;
            statusReturn = HAL_OK;
        } else {
            *bytesRead = 0;
            statusReturn = HAL_ERROR;
        }
    } else {
        *bytesRead = 0;
        statusReturn = HAL_BUSY;
    }

    return statusReturn;
}
