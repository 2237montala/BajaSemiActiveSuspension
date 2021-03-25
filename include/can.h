// #include "targetCommon.h"
// #include "targetSpecific.h"


// #define CAN_MESSAGE_MAX_LEN 8

// /*
//  * HEADER NAME : can.h
//  * CREATOR     : Anthony Montalbano
//  * CREATE DATE : 01/16/2021
//  * DESCRIPTION :
//  */

// HAL_StatusTypeDef CAN_Init(void);

// /*
//  * PURPOSE
//  *      Sends a message over the CAN bus. It adds the message to the CAN fifo
//  *      and will send it when it has its turn. The function is blocking until
//  *      the message has been sent
//  * PARAMETERS
//  * 
//  * RETURNS
//  *      HAL_StatusTyepDef : status of the HAL CAN device
//  */
// HAL_StatusTypeDef CAN_SendMessage(CAN_TxHeaderTypeDef *messageHeader, uint8_t *message, uint32_t len);

// HAL_StatusTypeDef CAN_SendMessageNb(CAN_TxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);

// HAL_StatusTypeDef CAN_ReceiveMessage(CAN_RxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);

// HAL_StatusTypeDef CAN_ReceiveMessageNb(CAN_RxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);