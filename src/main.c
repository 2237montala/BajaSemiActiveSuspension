#include "nucleo_f446re.h"
#include "stdio.h"
#include "stdbool.h"
//#include "config.h"
#include "Delay.h"
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

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* PID systems for the four shock controllers */

volatile uint32_t micros = 0;

int setup() {
  // Sets up the systick and other mcu functions
  // Needs to be first if using HAL libraries
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  setupDebugUart(&UartHandle,115200);

  return 0;
}

int main(void)
{
    // Run code to set up the system
    setup();

    createInitialDamperProfiles();
    
    char *msg = "Hi from uart\r\n";

    UART_putString(&UartHandle,msg);
    /* Output a message on Hyperterminal using printf function */
    printf("\r\nUART Printf Example: retarget the C library printf function to the UART\r\n");

    /* Infinite loop */ 

    //__TIM6_CLK_ENABLE();
    TIM_HandleTypeDef usTimer = {.Instance = TIM6};

    usTimer.Init.Prescaler = 0;
    usTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    usTimer.Init.Period = 176;
    usTimer.Init.AutoReloadPreload = 0;
    usTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    usTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // usTimer.Init.Prescaler = 0;
    // usTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    // usTimer.Init.Period = 180000;
    // usTimer.Init.AutoReloadPreload = 0;
    // usTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // usTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    printf("test2\r\n");

    // Set up the timer registers
    HAL_TIM_Base_Init(&usTimer);

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

    HAL_TIM_Base_Start_IT(&usTimer);

    // for(int i = 0; i < 10; i++) {
    //   uint32_t startUs = micros;
    //   delay(10);
    //   uint32_t currUs = micros;
    //   printf("Start count: %lu\r\n",startUs);
    //   printf("End count: %lu\r\n",currUs);
    //   printf("Current microseconds: %lu\r\n",(currUs - startUs));

    // }

    while(true) {
        BSP_LED_On(LED2);
        delay(500);
        BSP_LED_Off(LED2);
        delay(500);
    }
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


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  // Enable the clock that TIM6 is connected to. page 59 of ref manual
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Enable clock for the timer itself
  __HAL_RCC_TIM6_CLK_ENABLE();

  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  NVIC_SetPriority(TIM6_DAC_IRQn,14);
}

void TIM6_DAC_IRQHandler() {
  micros++;

  // Clear the interrupt
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  TIM6->SR = ~TIM_SR_UIF;
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
    UartHandle.Instance          = USARTx;
    
    UartHandle.Init.BaudRate     = buadRate;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_RTS_CTS;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    UART_Init(huart);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
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
  HAL_UART_Transmit(&UartHandle, (uint8_t *)ptr, len, 0xFFFF); 

  return len;
}