// #include "can.h"
// #include "string.h"

// CAN_HandleTypeDef     CanHandle;
// uint8_t               TxData[8];
// uint8_t               RxData[8];
// uint32_t              TxMailbox;

// HAL_StatusTypeDef CAN_Init(void) {
//   HAL_StatusTypeDef error = HAL_OK;
//   CAN_FilterTypeDef  sFilterConfig;
  
//   /*##-1- Configure the CAN peripheral #######################################*/
//   CanHandle.Instance = CANx;
    
//   CanHandle.Init.TimeTriggeredMode = DISABLE;
//   CanHandle.Init.AutoBusOff = DISABLE;
//   CanHandle.Init.AutoWakeUp = DISABLE;
//   CanHandle.Init.AutoRetransmission = ENABLE;
//   CanHandle.Init.ReceiveFifoLocked = DISABLE;
//   CanHandle.Init.TransmitFifoPriority = DISABLE;
//   CanHandle.Init.Mode = CAN_MODE_NORMAL;
//   CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
//   CanHandle.Init.TimeSeg1 = CAN_BS1_12TQ;
//   CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
//   CanHandle.Init.Prescaler = 6;
  
//   // Used this website to get config values
//   // http://www.bittiming.can-wiki.info/
//   // Used a APB2 clock of 45 MHz
//   // 500kbps use Sync of 1, Seg1 of 12, Seg2 of 2, and prescale of 6
//   // 100kbps use Sync of 1, Seg1 of 15, Seg2 of 2, and prescale of 25

//   error = HAL_CAN_Init(&CanHandle);

//   // /*##-2- Configure the CAN Filter ###########################################*/
//   // The default has the filter blocking nothing
//   sFilterConfig.FilterBank = 0;
//   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//   sFilterConfig.FilterIdHigh = 0x0000;
//   sFilterConfig.FilterIdLow = 0x0000;
//   sFilterConfig.FilterMaskIdHigh = 0x0000;
//   sFilterConfig.FilterMaskIdLow = 0x0000;
//   sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//   sFilterConfig.FilterActivation = ENABLE;
//   sFilterConfig.SlaveStartFilterBank = 14;
  
//   if(error == HAL_OK) {
//     error = HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig);
//   }

//   /*##-3- Start the CAN peripheral ###########################################*/
//   if(error == HAL_OK) {
//     error = HAL_CAN_Start(&CanHandle);
//   }
//   return error;
// }

// HAL_StatusTypeDef CAN_SendMessage(CAN_TxHeaderTypeDef *messageHeader, uint8_t *message,
//                                   uint32_t len) {
//   HAL_StatusTypeDef error = HAL_CAN_ERROR_NONE; 
//   uint32_t bytesSent = 0;
//   while(error == HAL_CAN_ERROR_NONE && bytesSent < len) {
//       uint32_t diff = len - bytesSent;

//       if(diff < 8) {
//         messageHeader->DLC = diff;
//       } else {
//         messageHeader->DLC = CAN_MESSAGE_MAX_LEN;
//       }

//       // Copy the next set of bytes into the message buffer
//       memcpy(TxData, &message[bytesSent],messageHeader->DLC);

//       error = HAL_CAN_AddTxMessage(&CanHandle, messageHeader, TxData, &TxMailbox);

//       bytesSent += messageHeader->DLC;


//   }

//   return error;

// }

// HAL_StatusTypeDef CAN_SendMessageNb(CAN_TxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);

// HAL_StatusTypeDef CAN_ReceiveMessage(CAN_RxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);

// HAL_StatusTypeDef CAN_ReceiveMessageNb(CAN_RxHeaderTypeDef messageHeader, uint8_t *message, uint32_t len);