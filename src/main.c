#include "config.h"
#include "targetSpecific.h"
#include "stdio.h"
#include "stdbool.h"
#include "Uart.h"
#include "targetCommon.h"
#include "DataCollection.h"
#include "DataProcessing.h"

#include "ControlSys.h"

#ifndef CANopen_H
  #include "CANopen.h"
#endif



// Struct definitions 
struct ShockControllerNodeStatus {
  uint8_t canOpenId;
  bool isActive;
  bool hasNewData;
};

struct ShockControllerNodes {
  struct ShockControllerNodeStatus status;
  struct ShockControlSystem controlSystem;
  struct VariableToOdMappingStruct dampingValueMapping;
  struct VariableToOdMappingStruct canIdMapping;
};

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
void ShockControllerBooted(uint8_t nodeId, uint8_t idx, void *object);
void ShockControllerNMTChange(uint8_t nodeId,uint8_t idx, CO_NMT_internalState_t state, void *object);
void ShockControllerHBReceived(uint8_t nodeId, uint8_t idx, void *object);
void ShockControllerHBStopped(uint8_t nodeId, uint8_t idx, void *object);

void enableHBCallBacksForAllNodes(uint8_t numNodes);
void resetAllNodes(struct ShockControllerNodes *nodes, uint8_t numNodes);
void enableHBForAllNodes(struct ShockControllerNodes *nodes, uint8_t numNodes);

// Private functions for control system
void createInitialDamperProfiles();
bool DoAllNodesHaveNewData(struct ShockControllerNodes *nodeArray, uint32_t arrayLen);

uint8_t DisableCoTimerIrq();
void EnableCoTimerIrq(uint8_t basePri);
void SetupShockControllerNodeMappings();
void LoadNewDataIntoFifo();
void ComputeVelocities(bool firstRun, uint32_t dt);
void ComputeAllDampingValue(bool firstRun, uint32_t dt);
void LoadNewDampingValuesToOD(struct ShockControllerNodes *nodes, uint32_t len);



// CANOpen variable/settings -------------------------------------------------------------
/* Global variables and objects */
volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */
uint8_t LED_red, LED_green;
volatile uint32_t tempTimer = 0;

// Timer for running CANOpen background tasks
TIM_HandleTypeDef msTimer = {.Instance = TIM6};
#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define TMR_TASK_INTERVAL_MS (TMR_TASK_INTERVAL/1000)
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

struct ShockControllerNodes shockControllerNodes[NUM_SHOCKS];

// LED values and pins
#define GREEN_LED_PIN D8
#define RED_LED_PIN D9
#define DEBUG_GPIO_PIN D4

uint32_t nmtRetryCounter = 0;
bool nmtChanged = false;

//-----------------------------------------------------------------------------------------
// Handle for CAN object linked with CO and Hal
CAN_HandleTypeDef     CanHandle;

/* debug UART handler declaration */
UART_HandleTypeDef debugUartHandle;

#ifdef SOFTWARE_TEST
#define SOFTWATE_TEST_UART_BAUDRATE 115200
UART_HandleTypeDef STUartHandle;
void ST_SetupUart();
void ST_SetupTestDataArray();
void ST_LoadNewTestData();
void ST_PrintControlSystemOutput(struct ShockControllerNodes *nodes, uint32_t len);
bool ST_LoadNewDataFromUart(uint32_t *simulationDtInMs);
uint32_t controlSystemStartComputeMs = 0;
uint32_t controlSystemEndComputeMs = 0;
#endif


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

  #ifdef SOFTWARE_TEST
  ST_SetupUart();
  #endif

  return 0;
}

int main (void){
  /* Configure microcontroller. */
  setup();

  // Initialize global variables
  // Configure the controller node array based on which nodes are active
  uint8_t shockNodesAdded = 0;
  if(SHOCK_CONTROLLER_ONE_ID != 0) {
    shockControllerNodes[shockNodesAdded++].status.canOpenId = SHOCK_CONTROLLER_ONE_ID;
  }

  if(SHOCK_CONTROLLER_TWO_ID != 0) {
    shockControllerNodes[shockNodesAdded++].status.canOpenId = SHOCK_CONTROLLER_TWO_ID;
  }

  if(SHOCK_CONTROLLER_THREE_ID != 0) {
    shockControllerNodes[shockNodesAdded++].status.canOpenId = SHOCK_CONTROLLER_THREE_ID;
  }

  if(SHOCK_CONTROLLER_FOUR_ID != 0) {
    shockControllerNodes[shockNodesAdded++].status.canOpenId = SHOCK_CONTROLLER_FOUR_ID;
  }



  // Create temporary shockDamperProfile
  struct ShockDamperProfile defaultProfile = { 0 };
  defaultProfile.PID_P = PID_P_NORMAL;
  defaultProfile.PID_I = PID_I_NORMAL;
  defaultProfile.PID_D = PID_D_NORMAL;

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
      log_printf("Allocated %lu bytes for CANopen objects\r\n", heapMemoryUsed);
  }

  while(reset != CO_RESET_APP){
    /* CANopen communication reset - initialize CANopen objects *******************/
    uint16_t timer1msPrevious;

    enableHBForAllNodes(shockControllerNodes,NUM_SHOCKS);

    log_printf("CANopenNode - Reset communication...\r\n");

    /* disable CAN and CAN interrupts */
    CO_CANmodule_disable(CO->CANmodule[0]);

    /* initialize CANopen */
    err = CO_CANinit(CANmoduleAddress, pendingBitRate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\r\n", err);
        return 0;
    }

    err = CO_CANopenInit(activeNodeId);
    if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        log_printf("Error: CANopen initialization failed: %d\r\n", err);
        return 0;
    }

    // Map the node array damping values to the OD
    SetupShockControllerNodeMappings();

    // Set up the data collection functions
    DataCollectionInit(CO,OD_6000_readShockAccel,OD_6050_readShockAccelStatus,
                     OD_6100_readAccelRPY,OD_6060_readShockDataSenderID,OD_6150_readShockPosition);

    // Set up the control system
    struct ShockControlSystem *tempControlSysArr[NUM_SHOCKS];
    for(int i = 0; i < NUM_SHOCKS; i++) {
      tempControlSysArr[i] = &(shockControllerNodes[i].controlSystem);
    }
    ControlSystemInit(tempControlSysArr,NUM_SHOCKS,defaultProfile);

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
    #ifndef SOFTWARE_TEST
    HAL_TIM_Base_Start_IT(&msTimer);
    #endif
    // Set up callbacks for certain functions
    enableHBCallBacksForAllNodes(NUM_SHOCKS);

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule[0]);

    reset = CO_RESET_NOT;
    timer1msPrevious = CO_timer1ms;

    log_printf("CANopenNode - Running...\r\n");
    fflush(stdout);

    // Run startup sequence
    bool startUpComplete = false;
    uint16_t timer1msCopy, timer1msDiff;
    bool onBoot = true;
    bool firstSetOfData = true;
    bool newVelocityData = false;
    bool dampingValuesReady = false;
    uint32_t lastVelocityComputeMs = 0;
    uint32_t controlSystemDt = HAL_GetTick();

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
          log_printf("Reseting all shock controllers\r\n");

          resetAllNodes(shockControllerNodes,NUM_SHOCKS);
          onBoot = false;
          log_printf("Ready\r\n");
        }

        /* Nonblocking application code may go here. */
        // //TODO : Change this code to only set start up complete once
        // // all the nodes are here
        // if(CO->HBcons->allMonitoredOperational) {
        //   //startUpComplete = true;
        //   //BSP_LED_On(LED2);
        // }

        // Calculate velocites based on previous data
        // Check if all the nodes have new data
        if(DoAllNodesHaveNewData(shockControllerNodes,NUM_SHOCKS)) {
          #ifdef SOFTWARE_TEST
          controlSystemStartComputeMs = HAL_GetTick();
          #endif
          // If we are not testing the software then compute the dt based on the current
          // system time. Otherwise we have to send the dt from the test script
          #ifndef SOFTWARE_TEST
          controlSystemDt = (HAL_GetTick() - lastVelocityComputeMs);
          #endif
          ComputeVelocities(firstSetOfData,controlSystemDt);
          
          newVelocityData = true;
        }

        // Run the control system if we have new data
        if(newVelocityData) {
          ComputeAllDampingValue(firstSetOfData,controlSystemDt);

          // Reset state
          newVelocityData = false;
          dampingValuesReady = true;
        }

        // If we have new damping values then send those values out
        if(dampingValuesReady) {
          // Send the damping values to the shock controllers
          // TODO: Write this function
          #ifdef SOFTWARE_TEST
          controlSystemEndComputeMs = HAL_GetTick();
          #endif
          dampingValuesReady = false;
          firstSetOfData = false;

          // Copy the new damping values to the OD
          LoadNewDampingValuesToOD(shockControllerNodes,NUM_SHOCKS);

          // Load in new testing data
          #ifdef SOFTWARE_TEST
          // Print out the time to calculate damping values
          log_printf("Time to compute %d damping values: %lu\r\n",
                NUM_SHOCKS,(controlSystemEndComputeMs - controlSystemStartComputeMs));

          // Send computed value back to testing pc
          ST_PrintControlSystemOutput(shockControllerNodes,NUM_SHOCKS);
          #endif
        }

        #ifdef SOFTWARE_TEST
          // This is blocking code
          // Send a ready command to let the software know we are ready for new data
          UART_putString(&STUartHandle,"rdy");
          ST_LoadNewDataFromUart(&controlSystemDt);
        #endif

        /* optional sleep for short time */
    }

    log_printf("All nodes are ready\r\n");

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

  /* program exit ***************************************************************/
  /* stop threads */
  HAL_TIM_Base_Stop_IT(&msTimer);

  /* delete objects from memory */
  CO_delete((void*) 0/* CAN module address */);

  log_printf("CANopenNode finished\r\n");

  /* reset */
  return 0;
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

      // Check if there was new sensor data added to the OD
      if(DoesOdContainNewData()) {
        LoadNewDataIntoFifo();
      }

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
static void Error_Handler(void){
  /* Turn LED2 on */
  const int NUM_BLINKS = 2;
  while (1)
  {
    BSP_LED_Off(LED2);
    // Number of blinks has to be multiplied by 2
    for(int i = 0; i < 2 * NUM_BLINKS; i++) {
      BSP_LED_Toggle(LED2);
      HAL_Delay(250);
    }
    BSP_LED_Off(LED2);
    HAL_Delay(1000);

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        tmrTask_thread();
    }
}

void resetAllNodes(struct ShockControllerNodes *nodes, uint8_t numNodes) {
  for(uint8_t i = 0; i < numNodes; i++) {
    CO_NMT_sendCommand(CO->NMT,CO_NMT_RESET_NODE,nodes[i].status.canOpenId);
  }
}

void enableHBForAllNodes(struct ShockControllerNodes *nodes, uint8_t numNodes)
{
  for(uint8_t i = 0; i < numNodes; i++) {
    uint32_t temp = nodes[i].status.canOpenId << 16U;
      temp |= SHOCK_CONTROLLER_HEARTBEAT_INTERVAL + SHOCK_CONTROLLER_HEARTBEAT_INTERVAL_OFFSET;
      OD_consumerHeartbeatTime[i] = temp;
  }

}

void enableHBCallBacksForAllNodes(uint8_t numNodes) {
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

    #ifdef SHOCK_CONTROLLER_TWO_ID
      case SHOCK_CONTROLLER_TWO_ID:
        CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
        break;
    #endif

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

void ShockControllerBooted(uint8_t nodeId, uint8_t idx, void *object) {
  log_printf("%lu: Shock controller node %d booted\r\n",HAL_GetTick(),nodeId);
  //CO_NMT_sendCommand(CO->NMT,CO_NMT_ENTER_OPERATIONAL,nodeId);
}

void ShockControllerNMTChange(uint8_t nodeId,uint8_t idx, CO_NMT_internalState_t state, void *object) {
  log_printf("%lu: Shock controller node %d NMT changed to %d\r\n",HAL_GetTick(),nodeId,state);
  if(state != CO_NMT_INITIALIZING) {
    SetRemoteNodeToOperational(nodeId);
  }

  // TODO: If the node changes from operational to non operational set the active flag
  // in shockControllerNodes to false;
}

void ShockControllerHBReceived(uint8_t nodeId, uint8_t idx, void *object) {
  //log_printf("Shock controller node %d HB started\r\n",nodeId);
}

void ShockControllerHBStopped(uint8_t nodeId, uint8_t idx, void *object) {
  //log_printf("Shock controller node %d HB stopped\r\n",nodeId);
  
  // TODO: If the node changes from operational to non operational set the active flag
  // in shockControllerNodes to false;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  uint8_t tecErrors = (hcan->Instance->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
  uint8_t recErrors = (hcan->Instance->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
  uint32_t errorCode = hcan->ErrorCode;

  // Reset tx error if we have exited the warning state
  if(CO_isError(CO->em,CO_EM_CAN_BUS_WARNING) && tecErrors < 127) {
    CO_errorReset(CO->em,CO_EM_CAN_BUS_WARNING, 0);
  }

  if(CO_isError(CO->em,CO_EM_CAN_BUS_WARNING) && recErrors < 127) {
    CO_errorReset(CO->em,CO_EM_CAN_BUS_WARNING, 0);
  }

  log_printf("CAN TEC/REC/Error: %d/%d/%lx\r\n", tecErrors,recErrors,errorCode);
}

bool DoAllNodesHaveNewData(struct ShockControllerNodes *nodeArray, uint32_t arrayLen) {
  uint32_t i = 0;
  uint32_t newDataCount = 0;
  while(i < arrayLen) {
    if(nodeArray[i].status.hasNewData) {
      newDataCount++;
    }
    i++;
  }

  return newDataCount == arrayLen;
}

uint8_t DisableCoTimerIrq() {
  uint8_t oldBasePri = __get_BASEPRI();

  // Set the minimum priority for interrupts to be higher than
  // the CO timer interrupt
  // Value needs to be shifted over by 4 following table 9 of Cortex M3
  // processor programming guide
  __set_BASEPRI((TIM6_IRQ_PRIORITY << 4));
  return oldBasePri;
}

void EnableCoTimerIrq(uint8_t basePri) {
  __set_BASEPRI(basePri);
}

void LoadNewDataIntoFifo() {
  log_printf("Got new data\r\n");
  // Copy the data out of the OD
  uint8_t fifoIndex = PushNewDataOntoFifo();
  if(fifoIndex >= 0) {
    // Notify the main loop that a shock controller sent new data and to process it
    shockControllerNodes[fifoIndex].status.hasNewData = true;

  } else {
    // Something went wrong with the data collection
    //CO_errorReport(CO->em,CO_EM_GENERIC_SOFTWARE_ERROR,CO_EMC_SOFTWARE_INTERNAL,0U);

    // Enter into stop mode?
  }
}

void ComputeVelocities(bool firstRun, uint32_t dt) {
  // Disable CO thread interrupt as we will be accessing the data fifos
  // We need to prevent the fifo changing mid calculation
  uint8_t oldBasePri = DisableCoTimerIrq();

  // Compute all the velocities
  if(DataProcessingComputeVelocities(firstRun, dt) > 0) {
    // Error with computing values
    // One of the fifos must have been empty somehow
    // Stop the program as this is a code problem
  }

  // Enable CO thread interrupt 
  EnableCoTimerIrq(oldBasePri);
}

void ComputeAllDampingValue(bool firstRun, uint32_t dt) {
  for(int i = 0; i < NUM_SHOCKS; i++) {
    // We want to remove the old value after the velocity was calculated becuase it uses the previous value
    // This function will only delete if there is more than 1 item in the fifo
    if(!firstRun) {
      DataProcessingRemoveFifoHead(i);
    }
    
    ControlSystemShockData_t shockData = DataProcessingGetFifoHead(i);
    ControlSystemComputeDampingValue(&(shockControllerNodes[i].controlSystem),&shockData,dt);
    
  }
}

void LoadNewDampingValuesToOD(struct ShockControllerNodes *nodes, uint32_t len) {
  // for each shock load it new damping value into the TPDO
  for(int i = 0; i < len; i++) {
    // Copy data to OD
    memcpy(nodes[i].dampingValueMapping.odDataPtr,
           &(nodes[i].controlSystem.previousDamperValue),
           nodes[i].dampingValueMapping.dataLengthInBytes);

    memcpy(nodes[i].canIdMapping.odDataPtr,
           &(nodes[i].status.canOpenId),
           nodes[i].canIdMapping.dataLengthInBytes);
  }
}

void SetupShockControllerNodeMappings() {
  // Get the data mapping from local structure to OD for sending out damping values
  FillOdMapping(CO->SDO[0],&(shockControllerNodes[0].dampingValueMapping),OD_6500_sendShock1Damping);

  #if NUM_SHOCKS > 1
  FillOdMapping(CO->SDO[0],&(shockControllerNodes[1].dampingValueMapping),OD_6510_sendShock2Damping);
  #endif
  #if NUM_SHOCKS > 2
  FillOdMapping(CO->SDO[0],&(shockControllerNodes[2].dampingValueMapping),OD_6520_sendShock3Damping);
  #endif
  #if NUM_SHOCKS > 3
  FillOdMapping(CO->SDO[0],&(shockControllerNodes[3].dampingValueMapping),OD_6530_sendShock4Damping);
  #endif

  // Get the mapping for the shock controller can Ids
  struct VariableToOdMappingStruct canIdMapping;
  FillOdMapping(CO->SDO[0],&canIdMapping,OD_6540_sendConnectedShockIDs);

  // From the array of CAN ids assign each one an ID from the config file
  uint8_t shockNodesAdded = 0;
  if(SHOCK_CONTROLLER_ONE_ID > 0) {
    shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr = canIdMapping.odDataPtr;
    shockControllerNodes[shockNodesAdded].canIdMapping.dataLengthInBytes = sizeof(uint8_t);

    // Assign fill the OD with the CAN ID
    memcpy(shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr,
           &(shockControllerNodes[shockNodesAdded].status.canOpenId),sizeof(uint8_t));
    
    // Each time we add assign a node a OD index we move the pointer up one
    canIdMapping.odDataPtr++;
    shockNodesAdded++; // Move which index we are at
  }

  if(SHOCK_CONTROLLER_TWO_ID > 0) {
    shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr = canIdMapping.odDataPtr;
    shockControllerNodes[shockNodesAdded].canIdMapping.dataLengthInBytes = sizeof(uint8_t);

    // Assign fill the OD with the CAN ID
    memcpy(shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr,
           &(shockControllerNodes[shockNodesAdded].status.canOpenId),sizeof(uint8_t));
    
    // Each time we add assign a node a OD index we move the pointer up one
    canIdMapping.odDataPtr++;
    shockNodesAdded++; // Move which index we are at
  }

  if(SHOCK_CONTROLLER_THREE_ID > 0) {
    shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr = canIdMapping.odDataPtr;
    shockControllerNodes[shockNodesAdded].canIdMapping.dataLengthInBytes = sizeof(uint8_t);

    // Assign fill the OD with the CAN ID
    memcpy(shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr,
           &(shockControllerNodes[shockNodesAdded].status.canOpenId),sizeof(uint8_t));
    
    // Each time we add assign a node a OD index we move the pointer up one
    canIdMapping.odDataPtr++;
    shockNodesAdded++; // Move which index we are at
  }

  if(SHOCK_CONTROLLER_FOUR_ID > 0) {
    shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr = canIdMapping.odDataPtr;
    shockControllerNodes[shockNodesAdded].canIdMapping.dataLengthInBytes = sizeof(uint8_t);

    // Assign fill the OD with the CAN ID
    memcpy(shockControllerNodes[shockNodesAdded].canIdMapping.odDataPtr,
           &(shockControllerNodes[shockNodesAdded].status.canOpenId),sizeof(uint8_t));
    
    // Each time we add assign a node a OD index we move the pointer up one
    canIdMapping.odDataPtr++;
    shockNodesAdded++; // Move which index we are at
  }
}

#ifdef SOFTWARE_TEST
void ST_SetupUart() {
  /* UART1 configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  STUartHandle.Instance          = SOFTWARE_TEST_UART;
  
  STUartHandle.Init.BaudRate     = SOFTWATE_TEST_UART_BAUDRATE;
  STUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  STUartHandle.Init.StopBits     = UART_STOPBITS_1;
  STUartHandle.Init.Parity       = UART_PARITY_NONE;
  STUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  STUartHandle.Init.Mode         = UART_MODE_TX_RX;
  STUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  UART_Init(&STUartHandle);
}

void ST_PrintControlSystemOutput(struct ShockControllerNodes *nodes, uint32_t len) {
  // Convert each of the shock damping values into byte form to be sent back to testing pc
  // uint8_t dataToSendBackLen = sizeof(float32_t) * NUM_SHOCKS;
  // uint8_t dataToSendBack[dataToSendBackLen];

  for(int i = 0; i < len; i++) {
    // Convert each shock damping value to bytes and send it
    UART_putData(&STUartHandle, (uint8_t *) &(nodes[i].controlSystem.previousDamperValue),sizeof(float32_t));
  }

}

bool ST_LoadNewDataFromUart(uint32_t *simulationDtInMs) {
  // Loads in new data from the UART 
  // Expects the same amount of data as number of shocks
  // So if we have 2 shock it expects sensor data for 2 shocks
  uint32_t uartDataInLen = sizeof(struct ShockSensorDataOdStruct);
  uint8_t uartDataIn[uartDataInLen];
  memset(uartDataIn,0x0,uartDataInLen);

  int completedTransfers = 0;
  if(simulationDtInMs != NULL) {
    bool error = false;
    // Read in the delta t for the computations
    HAL_StatusTypeDef status = UART_readData(&STUartHandle, uartDataIn, sizeof(uint32_t));

    if(!error && status == HAL_OK) {
      // Read enough data to make 3 float32s
      memcpy(simulationDtInMs,uartDataIn,sizeof(uint32_t));
    } else {
      error = true;
    }
    
    for(int i = 0; i < NUM_SHOCKS; i++) {
      // Read in the accels
      struct ShockSensorDataOdStruct tempData = {0};

      status = UART_readData(&STUartHandle, uartDataIn, sizeof(float32_t)*NUMBER_OF_AXIS);

      if(!error && status == HAL_OK) {
        // Read enough data to make 3 float32s
        memcpy(tempData.sensorData.accels,uartDataIn,sizeof(float32_t)*NUMBER_OF_AXIS);
      } else {
        error = true;
      }

      // Read in shock len
      status = UART_readData(&STUartHandle, uartDataIn,sizeof(float32_t)); 

      if(!error && status == HAL_OK) {
        // Read enough data to make 3 float32s
        memcpy(&(tempData.sensorData.linearPos),uartDataIn,sizeof(float32_t));
      } else {
        error = true;
      }

      // Read in freefall
      status = UART_readData(&STUartHandle, uartDataIn,sizeof(uint8_t)); 

      if(!error && status == HAL_OK) {
        // Read enough data to make 3 float32s
        memcpy(&(tempData.sensorData.inFreefall),uartDataIn,sizeof(uint8_t));
      } else {
        error = true;
      }

      // Read in canId
      status = UART_readData(&STUartHandle, uartDataIn,sizeof(uint8_t)); 

      if(!error && status == HAL_OK) {
        // Read enough data to make 3 float32s
        memcpy(&(tempData.senderCanId),uartDataIn,sizeof(uint8_t));
      } else {
        error = true;
      }

      if(!error) {
        // Copy the data to the OD 
        bool success = DataCollectionLoadNewTestValues(&tempData);
        if(!success) {
          // Error copying over data
        }

        // Move the data out of the OD
        if(DoesOdContainNewData()) {
          //UART_putStringNL(&debugUartHandle,"Got data");
          LoadNewDataIntoFifo();
          completedTransfers++;
        } else {
          // This function should always return true since we just loaded data into the OD
          // Unless the data sent was the same which shouldn't happen during testing
        }
      }   
    }
  }

  return completedTransfers > 0;
}


#endif