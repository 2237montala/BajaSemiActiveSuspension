#include "config.h"
#include "targetSpecific.h"
#include "stdio.h"
#include "stdbool.h"
#include "Uart.h"
#include "targetCommon.h"

#include "ControlSys.h"
#include "CANopen.h"


// Include arm math libraies
#ifndef _ARM_MATH_H
    #include "arm_math.h"
#endif

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
void ShockControllerBooted(uint8_t nodeId, uint8_t idx, void *object);
void ShockControllerNMTChange(uint8_t nodeId,uint8_t idx, CO_NMT_internalState_t state, void *object);
void ShockControllerHBReceived(uint8_t nodeId, uint8_t idx, void *object);
void ShockControllerHBStopped(uint8_t nodeId, uint8_t idx, void *object);

void enableHBCallBacks(uint8_t *nodeIds, uint8_t numNodes);
void resetAllNodes(uint8_t *nodeIds, uint8_t numNodes);
void enableHBForAllNodes(uint8_t *nodeIds, uint8_t numNodes);

// Private functions for control system
void createInitialDamperProfiles();
void calculateAllDampingValues(ShockVelocitiesStruct_t *velocityData);

// CANOpen variable/settings -------------------------------------------------------------
/* Global variables and objects */
volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */
uint8_t LED_red, LED_green;
volatile uint32_t tempTimer = 0;

// Timer for running CANOpen background tasks
TIM_HandleTypeDef msTimer = {.Instance = TIM6};
#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

// CAN Open shock controller variables
uint8_t shockControllersNodes[NUM_SHOCKS];

// Sensor Data Fifos -------------------------------------------------------------------------
// These need to be in here as they are referenced using the ID passed into the structure macro
#define SHOCK_SENSOR_DATA_FIFO_NAME shockSensorDataFifo
#define SHOCK_VELOCITY_FIFO_NAME shockVelocityFifo

_fff_declare_a(ShockSensorDataStruct_t,SHOCK_SENSOR_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);
_fff_declare_a(ShockVelocitiesStruct_t,SHOCK_VELOCITY_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);

// Initalize the data fifos
_fff_init_a(SHOCK_SENSOR_DATA_FIFO_NAME,NUM_SHOCKS);
_fff_init_a(SHOCK_VELOCITY_FIFO_NAME,NUM_SHOCKS);


// Create a shock control system struct to be used
struct ShockControlSystem shockControlSystems; 


// LED values and pins
#define GREEN_LED_PIN D8
#define RED_LED_PIN D9
#define DEBUG_GPIO_PIN D4

uint32_t nmtRetryCounter = 0;
bool nmtChanged = false;

//-----------------------------------------------------------------------------------------
// Handle for CAN object linked with CO and Hal
CAN_HandleTypeDef     CanHandle;

/* UART handler declaration */
UART_HandleTypeDef debugUartHandle;

#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)

int setup() {
  // Sets up the systick and other mcu functions
  // Needs to be first if using HAL libraries
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  setupDebugUart(&debugUartHandle,115200);

  BSP_LED_Init(LED2);
  BspGpioInitOutput(GREEN_LED_PIN);
  BspGpioInitOutput(RED_LED_PIN);
  BspGpioInitOutput(DEBUG_GPIO_PIN);

  return 0;
}

int main (void){
  /* Configure microcontroller. */
  setup();

  // Create temporary shockDamperProfile
  struct ShockDamperProfile defaultProfile = { 0 };
  defaultProfile.PID_P = PID_P_NORMAL;
  defaultProfile.PID_I = PID_I_NORMAL;
  defaultProfile.PID_D = PID_D_NORMAL;

  // Set up the control system
  ControlSystemInit(&shockControlSystems,NUM_SHOCKS,defaultProfile);
  //calculateAllDampingValues(&newestData);

  // CAN Open Variables
  CO_ReturnError_t err;
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  uint32_t heapMemoryUsed;
  void *CANmoduleAddress = &CanHandle; /* CAN module address */
  uint8_t activeNodeId = MAIN_CONTROLLER_ID;
  uint16_t pendingBitRate = CAN_BAUD_RATE; 

  /* Allocate memory but these are statically allocated so no malloc */
  err = CO_new(&heapMemoryUsed);
  if (err != CO_ERROR_NO) {
      log_printf("Error: Can't allocate memory\r\n");
      return 0;
  }
  else {
      log_printf("Allocated %d bytes for CANopen objects\r\n", heapMemoryUsed);
  }

  while(reset != CO_RESET_APP){
/* CANopen communication reset - initialize CANopen objects *******************/
      uint16_t timer1msPrevious;

      //Add one shock controller to the list of monitored notes
      shockControllersNodes[0] = SHOCK_CONTROLLER_ONE_ID;
      shockControllersNodes[1] = SHOCK_CONTROLLER_TWO_ID;

      enableHBForAllNodes(shockControllersNodes,NUM_SHOCKS);

      log_printf("CANopenNode - Reset communication...\r\n");

      /* disable CAN and CAN interrupts */
      CO_CANmodule_disable(CO->CANmodule[0]);

      /* initialize CANopen */
      err = CO_CANinit(CANmoduleAddress, pendingBitRate);
      if (err != CO_ERROR_NO) {
          log_printf("Error: CAN initialization failed: %d\r\n", err);
          return 0;
      }
      // err = CO_LSSinit(&pendingNodeId, &pendingBitRate);
      // if(err != CO_ERROR_NO) {
      //     log_printf("Error: LSS slave initialization failed: %d\n", err);
      //     return 0;
      // }
      err = CO_CANopenInit(activeNodeId);
      if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
          log_printf("Error: CANopen initialization failed: %d\r\n", err);
          return 0;
      }

      /* Configure Timer interrupt function for execution every 1 millisecond */

      HAL_TIM_Base_DeInit(&msTimer);
      HAL_TIM_Base_Stop_IT(&msTimer);

      // Using timer 6
      // Timer 4 input clock is APB1
      // As of now APB1 is 45Mhz
      // The Timer 4 clock input is multiplied by 2 so 90 Mhz.


      __TIM6_CLK_ENABLE();

      // Input = 90 Mhz
      // Interal divider = 1
      // Prescaler = 90
      // Clock rate = (Input)/(Interal divider * prescaler)
      //            = (90 MHz)/(90) = 1 Mhz

      msTimer.Init.Prescaler = 91-1;
      msTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
      msTimer.Init.Period = 1000-1;
      msTimer.Init.AutoReloadPreload = 0;
      msTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      msTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

      /* Configure CAN transmit and receive interrupt */

      // Initalize timer device
      HAL_TIM_Base_Init(&msTimer);

      // Enable interrupts for timer
      HAL_TIM_Base_Start_IT(&msTimer);

      // Set up callbacks for certain functions
      enableHBCallBacks(shockControllersNodes,NUM_SHOCKS);

      /* start CAN */
      CO_CANsetNormalMode(CO->CANmodule[0]);

      reset = CO_RESET_NOT;
      timer1msPrevious = CO_timer1ms;

      log_printf("CANopenNode - Running...\r\n");
      fflush(stdout);

      // Run startup sequence
      bool startUpComplete = false;
      uint16_t timer1msCopy, timer1msDiff;
      uint32_t nmtCommandDelay = 5000;
      uint32_t lastNMTCommandTime = HAL_GetTick();
      bool onBoot = true;
      while(reset == CO_RESET_NOT && !startUpComplete){
          

          timer1msCopy = CO_timer1ms;
          timer1msDiff = timer1msCopy - timer1msPrevious;
          timer1msPrevious = timer1msCopy;

          /* CANopen process */
          reset = CO_process(CO, (uint32_t)timer1msDiff*1000, NULL);

          // Handle LED Updates
          LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
          LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

          BspGpioWrite(GREEN_LED_PIN,LED_green);
          BspGpioWrite(RED_LED_PIN,LED_red);

          if(onBoot) {
            // Run some code in the beginning to reset network
            printf("Reseting all shock controllers\r\n");
            resetAllNodes(shockControllersNodes,NUM_SHOCKS);
            onBoot = false;
          }

          /* Nonblocking application code may go here. */
          if(CO->HBcons->allMonitoredOperational) {
            //startUpComplete = true;
            //BSP_LED_On(LED2);
          }

          if(HAL_GetTick() - lastNMTCommandTime >= nmtCommandDelay) {
            lastNMTCommandTime = HAL_GetTick();
            //CO->TPDO[0]->sendRequest = true;
          }

          /* optional sleep for short time */
      }


      printf("All nodes are ready\r\n");

      while(reset == CO_RESET_NOT){
/* loop for normal program execution ******************************************/
          timer1msCopy = CO_timer1ms;
          timer1msDiff = timer1msCopy - timer1msPrevious;
          timer1msPrevious = timer1msCopy;

          /* CANopen process */
          reset = CO_process(CO, (uint32_t)timer1msDiff*1000, NULL);

          /* Nonblocking application code may go here. */

          /* Process EEPROM */

          /* optional sleep for short time */
      }
  }

}

/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void){
  /* sleep for interval */
  BspGpioToggle(DEBUG_GPIO_PIN);
  INCREMENT_1MS(CO_timer1ms);
  if(CO->CANmodule[0]->CANnormal) {
      bool_t syncWas;

      /* Process Sync */
      syncWas = CO_process_SYNC(CO, TMR_TASK_INTERVAL, NULL);

      /* Read inputs */
      CO_process_RPDO(CO, syncWas);

      /* Further I/O or nonblocking application code may go here. */

      /* Write outputs */
      CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL, NULL);

      /* verify timer overflow */
      if(0) {
          CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
      }
  }
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

void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&msTimer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        tmrTask_thread();
    }
    
    
}

void resetAllNodes(uint8_t *nodeIds, uint8_t numNodes) {
  for(uint8_t i = 0; i < numNodes; i++) {
    CO_NMT_sendCommand(CO->NMT,CO_NMT_RESET_NODE,nodeIds[i]);
  }
}

void enableHBForAllNodes(uint8_t *nodeIds, uint8_t numNodes) {
  for(uint8_t i = 0; i < numNodes; i++) {
      uint32_t temp = (nodeIds[i]) << 16U;
      temp |= SHOCK_CONTROLLER_HEARTBEAT_INTERVAL + SHOCK_CONTROLLER_HEARTBEAT_INTERVAL_OFFSET;
      OD_consumerHeartbeatTime[i] = temp;
  }
}

void enableHBCallBacks(uint8_t *nodeIds, uint8_t numNodes) {
  for(uint8_t i = 0; i < numNodes; i++) {
    CO_HBconsumer_initCallbackRemoteReset(CO->HBcons,i,
                                          NULL,ShockControllerBooted);

    CO_HBconsumer_initCallbackNmtChanged(CO->HBcons,i,
                                          NULL,ShockControllerNMTChange);

    CO_HBconsumer_initCallbackHeartbeatStarted(CO->HBcons, i,
                                              NULL,ShockControllerHBReceived);

    CO_HBconsumer_initCallbackTimeout(CO->HBcons,i,NULL,ShockControllerHBStopped);
  }
  
}

void SetRemoteNodeToOperational(uint8_t nodeId) {
  switch(nodeId) {
    case SHOCK_CONTROLLER_ONE_ID:
      CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
      break;

    case SHOCK_CONTROLLER_TWO_ID:
      CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
      break;

    default:
      break;
  }
  // if(nodeId == (SHOCK_CONTROLLER_ONE_ID | SHOCK_CONTROLLER_TWO_ID)) {
  //   if(nmtRetryCounter++ < 5) {
  //     //printf("Putting back into operational state\r\n");
  //     CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
  //   } else if(nmtRetryCounter == 6) {
  //     printf("Too many retry attempts\r\n");
  //   }
  // }
}

void calculateAllDampingValues(ShockVelocitiesStruct_t *velocityData) {
  for(int i = 0; i < NUM_SHOCKS; i++) {
    float32_t tempDy, tempDLinearPos;
    tempDLinearPos = velocityData->dLinearPos;
    tempDy = velocityData->dy;
    calculateDampingValue(&shockControlSystems, i, tempDLinearPos, 
                          tempDy, SHOCK_DATA_COLLECTION_RATE_SEC);

    // Take new damper value and convert to valve position
    // TODO: write this function
  }
}

void ShockControllerBooted(uint8_t nodeId, uint8_t idx, void *object) {
  printf("%lu: Shock controller node %d booted\r\n",HAL_GetTick(),nodeId);
  //CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
}


void ShockControllerNMTChange(uint8_t nodeId,uint8_t idx, CO_NMT_internalState_t state, void *object) {
  printf("%lu: Shock controller node %d NMT changed to %d\r\n",HAL_GetTick(),nodeId,state);
  if(state != CO_NMT_INITIALIZING) {
    SetRemoteNodeToOperational(nodeId);
  }
  
}

void ShockControllerHBReceived(uint8_t nodeId, uint8_t idx, void *object) {
  //printf("Shock controller node %d HB started\r\n",nodeId);
}

void ShockControllerHBStopped(uint8_t nodeId, uint8_t idx, void *object) {
  //printf("Shock controller node %d HB stopped\r\n",nodeId);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    printf("CAN TEC Error: %lu\r\n", (hcan->Instance->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    printf("CAN REC Error: %lu\r\n", (hcan->Instance->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    printf("CAN error: 0x%lx\r\n", hcan->ErrorCode);
}

