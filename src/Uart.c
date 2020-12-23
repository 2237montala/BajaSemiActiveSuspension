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
