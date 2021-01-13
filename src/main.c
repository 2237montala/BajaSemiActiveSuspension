#include "nucleo_f446re.h"
#include "stdio.h"
#include "stdbool.h"
//#include "config.h"
//#include "Delay.h"
#include "Uart.h"
#include "targetCommon.h"
#include "ControlSys.h"


// Include arm math libraies
#ifndef _ARM_MATH_H
    #include "arm_math.h"
#endif

/* Private function prototypes -----------------------------------------------*/
void createInitialDamperProfiles();
static void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
static HAL_StatusTypeDef CAN_Polling(void);

/* UART handler declaration */
UART_HandleTypeDef debugUartHandle;

CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;


/* PID systems for the four shock controllers */

volatile uint32_t micros = 0;

int setup() {
  // Sets up the systick and other mcu functions
  // Needs to be first if using HAL libraries
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  setupDebugUart(&debugUartHandle,115200);

  BSP_LED_Init(LED2);

  return 0;
}

int main(void)
{
    // Run code to set up the system
    setup();

    createInitialDamperProfiles();
    
    char *msg = "Hi from uart\r\n";

    UART_putString(&debugUartHandle,msg);
    /* Output a message on Hyperterminal using printf function */
    printf("\r\nUART Printf Example: retarget the C library printf function to the UART\r\n");

    /* Infinite loop */ 

    //__TIM6_CLK_ENABLE();
    // TIM_HandleTypeDef usTimer = {.Instance = TIM6};

    // usTimer.Init.Prescaler = 0;
    // usTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    // usTimer.Init.Period = 176;
    // usTimer.Init.AutoReloadPreload = 0;
    // usTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // usTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // // usTimer.Init.Prescaler = 0;
    // // usTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    // // usTimer.Init.Period = 180000;
    // // usTimer.Init.AutoReloadPreload = 0;
    // // usTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // // usTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // printf("test2\r\n");

     CAN_Polling();

    // Set up the timer registers
    //HAL_TIM_Base_Init(&usTimer);

    // Start the timer with interrupt callbacks 
    // TIM6->CNT = 0u;
    // HAL_TIM_Base_Start(&usTimer);

    // Wait a second
    // for(int i = 0; i < 10; i++) {
    //   TIM6->CNT = 0u;
    //   HAL_TIM_Base_Start(&usTimer);


    //   uint32_t startUs = TIM6->CNT;
    //   delay(10);
    //   uint32_t currUs = TIM6->CNT;

    //   printf("Start count: %lu\r\n",startUs);
    //   printf("End count: %lu\r\n",currUs);
    //   printf("Current microseconds: %lu\r\n",(currUs - startUs));

    //   HAL_TIM_Base_Stop(&usTimer);

    // }

    // need to set up the irq handler again
    ///HAL_TIM_Base_Start_IT(&usTimer);

    // for(int i = 0; i < 10; i++) {
    //   uint32_t startUs = micros;
    //   delay(10);
    //   uint32_t currUs = micros;
    //   printf("Start count: %lu\r\n",startUs);
    //   printf("End count: %lu\r\n",currUs);
    //   printf("Current microseconds: %lu\r\n",(currUs - startUs));

    // }

    BSP_LED_On(LED2);

}

// void appendData(struct ShockSensorData *dataIn, struct ShockControlSystem *shockUnits, int numShocks) {

//   for(int i = 0; i < numShocks; i++) {
//       struct ShockControlSystem *currShockController = &shockUnits[i];

//       int dataBufIndex = currShockController->shockData.mostRecentDataIndex;
//       if(dataBufIndex >= SHOCK_DATA_BUFFER_LEN) {
//         // If we are at the end of the buffer then we wrap around
//         dataBufIndex = 0;


//       }
      
//       // Save the new sample at next index
//       currShockController->shockData.dataBuffer[dataBufIndex] = dataIn[i];

//       // Update the index
//       currShockController->shockData.mostRecentDataIndex++;
//     }




// }

void createInitialDamperProfiles() {
  // This function should be edited by the user to set the shock damper
  // profiles. All of the coefficients should be defined here or in the 
  // config header file

  struct ShockDamperProfile tempProfiles[NUM_SHOCK_PROFILES] = {
    {
        // Define the normal shock damping values
        .PID_P = PID_P_NORMAL,
        .PID_I = PID_I_NORMAL,
        .PID_D = PID_D_NORMAL
    }
  };

  SdpInit(tempProfiles,NUM_SHOCK_PROFILES);

}

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
HAL_StatusTypeDef CAN_Polling(void)
{
  CAN_FilterTypeDef  sFilterConfig;
  
  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;
    
  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = DISABLE;
  CanHandle.Init.AutoWakeUp = DISABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
  CanHandle.Init.Prescaler = 25;
  
  // 500kbps use Sync of 1, Seg1 of 12, Seg2 of 2, and prescale of 6
  // 100kbps use Sync of 1, Seg1 of 15, Seg2 of 2, and prescale of 25

  // Used this website to get config values
  // http://www.bittiming.can-wiki.info/
  // Used a APB2 clock of 45 MHz and a CAN baud rate of 500kbps

  UART_putString(&debugUartHandle, "Running CAN Init\r\n");
  if(HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  // /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }


  
  /*##-3- Start the CAN peripheral ###########################################*/
  UART_putString(&debugUartHandle, "Running CAN start\r\n");
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  UART_putString(&debugUartHandle,"Sending request\r\n");
  /*##-4- Start the Transmission process #####################################*/
  TxHeader.StdId = 0x20;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;
  
  // char *dataToSend = "Hi\r\n";

  // // TxData[0] = 0xCA;
  // // TxData[1] = 0xFE;

  // memcpy(TxData,dataToSend,8);
  
  // // Set Data Length Code
  // TxHeader.DLC = strlen(dataToSend);

  TxData[0] = 0xCA;
  TxData[1] = 0xFE;
  TxHeader.DLC = 2;

  /* Request transmission */
  if(HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
  
  /* Wait transmission complete */
  while(HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) != 3) {}

  /*##-5- Start the Reception process ########################################*/

  // Wait for the shock controller to respond
  UART_putStringNL(&debugUartHandle, "Waiting for request");
  while(HAL_CAN_GetRxFifoFillLevel(&CanHandle, CAN_RX_FIFO0) == 0) {
    uint32_t tecError = CanHandle.Instance->ESR & CAN_ESR_TEC;
    if(tecError > 0) {
      printf("%d\r\n",tecError);
    }
  }

  UART_putStringNL(&debugUartHandle, "Got request");
  if(HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  if((RxHeader.StdId != 0x12)                     ||
     (RxHeader.RTR != CAN_RTR_DATA)               ||
     (RxHeader.IDE != CAN_ID_STD))
  {
    /* Rx message Error */
    return HAL_ERROR;
  }

  UART_putString(&debugUartHandle, "Got message from shock controller\r\n");
  printf("%d\r\n",RxData[0]);

  return HAL_OK; /* Test Passed */
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows: 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
   /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

}

void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate) {
  /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follows:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = ODD parity
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    debugUartHandle.Instance          = USARTx;
    
    debugUartHandle.Init.BaudRate     = buadRate;
    debugUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    debugUartHandle.Init.StopBits     = UART_STOPBITS_1;
    debugUartHandle.Init.Parity       = UART_PARITY_NONE;
    debugUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_RTS_CTS;
    debugUartHandle.Init.Mode         = UART_MODE_TX_RX;
    debugUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    UART_Init(huart);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(true) {
    BSP_LED_On(LED2);
    HAL_Delay(500);
    BSP_LED_Off(LED2);
    HAL_Delay(500);
  }
}

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  //#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define PUTCHAR_PROTOTYPE int _write(int fd, char * ptr, int len)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&debugUartHandle, (uint8_t *)ptr, len, 0xFFFF); 

  return len;
}