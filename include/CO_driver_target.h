// /*
//  * CAN module object for ST STM32F334 microcontroller.
//  *
//  * @file        CO_driver_target.h
//  * @author      Janez Paternoster
//  * @author      Ondrej Netik
//  * @author      Vijayendra
//  * @author      Jan van Lienden
//  * @author      Petteri Mustonen
//  * @copyright   2013 Janez Paternoster
//  *
//  * This file is part of CANopenNode, an opensource CANopen Stack.
//  * Project home page is <https://github.com/CANopenNode/CANopenNode>.
//  * For more information on CANopen see <http://www.can-cia.org/>.
//  *
//  * CANopenNode is free and open source software: you can redistribute
//  * it and/or modify it under the terms of the GNU General Public License
//  * as published by the Free Software Foundation, either version 2 of the
//  * License, or (at your option) any later version.
//  *
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.
//  *
//  * Following clarification and special exception to the GNU General Public
//  * License is included to the distribution terms of CANopenNode:
//  *
//  * Linking this library statically or dynamically with other modules is
//  * making a combined work based on this library. Thus, the terms and
//  * conditions of the GNU General Public License cover the whole combination.
//  *
//  * As a special exception, the copyright holders of this library give
//  * you permission to link this library with independent modules to
//  * produce an executable, regardless of the license terms of these
//  * independent modules, and to copy and distribute the resulting
//  * executable under terms of your choice, provided that you also meet,
//  * for each linked independent module, the terms and conditions of the
//  * license of that module. An independent module is a module which is
//  * not derived from or based on this library. If you modify this
//  * library, you may extend this exception to your version of the
//  * library, but you are not obliged to do so. If you do not wish
//  * to do so, delete this exception statement from your version.
//  */


// #ifndef CO_DRIVER_TARGET_H
// #define CO_DRIVER_TARGET_H


// /* For documentation see file drvTemplate/CO_driver.h */


// /* Includes ------------------------------------------------------------------*/
// #include <stdbool.h>
// #include <stddef.h>         /* for 'NULL' */
// #include <stdint.h>         /* for 'int8_t' to 'uint64_t' */
// #include "targetSpecific.h"

// #define bool_t bool
// #define CO_LITTLE_ENDIAN

// /* Exported define -----------------------------------------------------------*/
// #define PACKED_STRUCT               __attribute__((packed))
// #define ALIGN_STRUCT_DWORD          __attribute__((aligned(4)))

// /* Peripheral addresses */
// #define ADDR_CAN1                   CAN1

// /* Critical sections */
// #define CO_LOCK_CAN_SEND()          __set_PRIMASK(1);
// #define CO_UNLOCK_CAN_SEND()        __set_PRIMASK(0);

// #define CO_LOCK_EMCY()              __set_PRIMASK(1);
// #define CO_UNLOCK_EMCY()            __set_PRIMASK(0);

// #define CO_LOCK_OD()                __set_PRIMASK(1);
// #define CO_UNLOCK_OD()              __set_PRIMASK(0);

// #define CLOCK_CAN                   RCC_APB1Periph_CAN1

// #define CAN_REMAP_1                 /* Select CAN1 remap 1 */
// #ifdef CAN1_NO_REMAP                /* CAN1 not remapped */
// #define CLOCK_GPIO_CAN              RCC_APB2Periph_GPIOA
// #define GPIO_Remapping_CAN          (0)
// #define GPIO_CAN                    GPIOA
// #define GPIO_Pin_CAN_RX             GPIO_Pin_11
// #define GPIO_Pin_CAN_TX             GPIO_Pin_12
// #define GPIO_PinSource_CAN_RX       GPIO_PinSource11
// #define GPIO_PinSource_CAN_TX       GPIO_PinSource12
// #define GPIO_CAN_Remap_State        DISABLE
// #endif
// #ifdef CAN_REMAP_1                  /* CAN1 remap 1 */
// #define CLOCK_GPIO_CAN              RCC_AHBPeriph_GPIOB
// #define GPIO_Remapping_CAN          GPIO_Remap1_CAN1
// #define GPIO_CAN                    GPIOB
// #define GPIO_Pin_CAN_RX             CANx_RX_PIN
// #define GPIO_Pin_CAN_TX             CANx_TX_PIN
// #define GPIO_PinSource_CAN_RX       GPIO_PinSource8
// #define GPIO_PinSource_CAN_TX       GPIO_PinSource9
// #define GPIO_CAN_Remap_State        ENABLE
// #endif

// #define CAN1_TX_INTERRUPTS          CAN1_TX_IRQn
// #define CAN1_RX0_INTERRUPTS         CAN1_RX0_IRQn

// #define CO_CAN_TXMAILBOX   ((uint8_t)0x00)

// /* Timeout for initialization */

// #define INAK_TIMEOUT        ((uint32_t)0x0000FFFF)
// /* Data types */
// typedef float                   float32_t;
// typedef long double             float64_t;
// typedef char                    char_t;
// typedef unsigned char           oChar_t;
// typedef unsigned char           domain_t;


// /* CAN receive message structure as aligned in CAN module.
//  * prevzato z stm32f10_can.h - velikostne polozky a poradi odpovidaji. */
// typedef struct{
//     uint32_t    ident;          /* Standard Identifier */
//     uint32_t    ExtId;          /* Specifies the extended identifier */
//     uint8_t     IDE;            /* Specifies the type of identifier for the
//                                    message that will be received */
//     uint8_t     RTR;            /* Remote Transmission Request bit */
//     uint8_t     DLC;            /* Data length code (bits 0...3) */
//     uint8_t     data[8];        /* 8 data bytes */
//     uint8_t     FMI;            /* Specifies the index of the filter the message
//                                    stored in the mailbox passes through */
// }CO_CANrxMsg_t;


// /* Received message object */
// typedef struct{
//     uint16_t           ident;
//     uint16_t           mask;
//     void               *object;
//     void              (*pFunct)(void *object, const CO_CANrxMsg_t *message); // Changed by VJ
// }CO_CANrx_t;


// /* Transmit message object. */
// typedef struct{
//     uint32_t            ident;
//     uint8_t             DLC;
//     uint8_t             data[8];
//     volatile uint8_t    bufferFull;
//     volatile uint8_t    syncFlag;
// }CO_CANtx_t;/* ALIGN_STRUCT_DWORD; */


// /* CAN module object. */
// typedef struct{
//     CAN_TypeDef        *CANdriverState;         /* STM32F4xx specific */
//     CO_CANrx_t         *rxArray;
//     uint16_t            rxSize;
//     CO_CANtx_t         *txArray;
//     uint16_t            txSize;
//     volatile bool     CANnormal;
//     volatile bool     useCANrxFilters;
//     volatile uint8_t    bufferInhibitFlag;
//     volatile uint8_t    firstCANtxMessage;
//     volatile uint16_t   CANtxCount;
//     uint32_t            errOld;
//     void               *em;
// }CO_CANmodule_t;


// /* CAN interrupts receives and transmits CAN messages. */
// void CO_CANinterrupt_Rx(CO_CANmodule_t *CANmodule);
// void CO_CANinterrupt_Tx(CO_CANmodule_t *CANmodule);

// #endif /* CO_DRIVER_TARGET_H */

/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#ifndef CO_CONFIG_NMT
#define CO_CONFIG_NMT (CO_CONFIG_NMT_CALLBACK_CHANGE | \
                       CO_CONFIG_NMT_MASTER | \
                       CO_CONFIG_FLAG_CALLBACK_PRE | \
                       CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | \
                           CO_CONFIG_HB_CONS_CALLBACK_MULTI | \
                           CO_CONFIG_HB_CONS_QUERY_FUNCT | \
                           CO_CONFIG_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_EM
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | \
                      CO_CONFIG_EM_HISTORY | \
                      CO_CONFIG_EM_CONSUMER | \
                      CO_CONFIG_FLAG_CALLBACK_PRE | \
                      CO_CONFIG_FLAG_TIMERNEXT | \
                      CO_CONFIG_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_SDO_SRV
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | \
                           CO_CONFIG_SDO_SRV_BLOCK | \
                           CO_CONFIG_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_FLAG_TIMERNEXT | \
                           CO_CONFIG_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_SDO_SRV_BUFFER_SIZE
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE (127*7)
#endif

#ifndef CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE | \
                           CO_CONFIG_SDO_CLI_SEGMENTED | \
                           CO_CONFIG_SDO_CLI_BLOCK | \
                           CO_CONFIG_SDO_CLI_LOCAL | \
                           CO_CONFIG_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_FLAG_TIMERNEXT | \
                           CO_CONFIG_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_TIME
#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE | \
                        CO_CONFIG_TIME_PRODUCER | \
                        CO_CONFIG_FLAG_CALLBACK_PRE)
#endif

#ifndef CO_CONFIG_SYNC
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | \
                        CO_CONFIG_SYNC_PRODUCER | \
                        CO_CONFIG_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_PDO
#define CO_CONFIG_PDO (CO_CONFIG_RPDO_ENABLE | \
                       CO_CONFIG_TPDO_ENABLE | \
                       CO_CONFIG_PDO_SYNC_ENABLE | \
                       CO_CONFIG_RPDO_CALLS_EXTENSION | \
                       CO_CONFIG_TPDO_CALLS_EXTENSION | \
                       CO_CONFIG_FLAG_CALLBACK_PRE | \
                       CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_LEDS
#define CO_CONFIG_LEDS (CO_CONFIG_LEDS_ENABLE | \
                        CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_GFC
#define CO_CONFIG_GFC (CO_CONFIG_GFC_ENABLE | \
                       CO_CONFIG_GFC_CONSUMER | \
                       CO_CONFIG_GFC_PRODUCER)
#endif

#ifndef CO_CONFIG_SRDO
#define CO_CONFIG_SRDO (CO_CONFIG_SRDO_ENABLE | \
                        CO_CONFIG_SRDO_CHECK_TX | \
                        CO_CONFIG_RSRDO_CALLS_EXTENSION | \
                        CO_CONFIG_TSRDO_CALLS_EXTENSION | \
                        CO_CONFIG_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_SRDO_MINIMUM_DELAY
#define CO_CONFIG_SRDO_MINIMUM_DELAY 0
#endif

#ifndef CO_CONFIG_LSS
#define CO_CONFIG_LSS (CO_CONFIG_LSS_SLAVE | \
                       CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND | \
                       CO_CONFIG_LSS_MASTER | \
                       CO_CONFIG_FLAG_CALLBACK_PRE)
#endif

#ifndef CO_CONFIG_GTW
#define CO_CONFIG_GTW (CO_CONFIG_GTW_ASCII | \
                       CO_CONFIG_GTW_ASCII_SDO | \
                       CO_CONFIG_GTW_ASCII_NMT | \
                       CO_CONFIG_GTW_ASCII_LSS | \
                       CO_CONFIG_GTW_ASCII_LOG | \
                       CO_CONFIG_GTW_ASCII_ERROR_DESC | \
                       CO_CONFIG_GTW_ASCII_PRINT_HELP | \
                       CO_CONFIG_GTW_ASCII_PRINT_LEDS)
#define CO_CONFIG_GTW_BLOCK_DL_LOOP 1
#define CO_CONFIG_GTWA_COMM_BUF_SIZE 2000
#define CO_CONFIG_GTWA_LOG_BUF_SIZE 2000
#endif

#ifndef CO_CONFIG_CRC16
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE)
#endif

#ifndef CO_CONFIG_FIFO
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | \
                        CO_CONFIG_FIFO_ALT_READ | \
                        CO_CONFIG_FIFO_CRC16_CCITT | \
                        CO_CONFIG_FIFO_ASCII_COMMANDS | \
                        CO_CONFIG_FIFO_ASCII_DATATYPES)
#endif

#ifndef CO_CONFIG_TRACE
#define CO_CONFIG_TRACE (CO_CONFIG_TRACE_ENABLE)
#endif


/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef unsigned char           bool_t;
typedef float                   float32_t;
typedef double                  float64_t;
typedef char                    char_t;
typedef unsigned char           oChar_t;
typedef unsigned char           domain_t;


/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)0)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)0)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)NULL)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
    int32_t errinfo;
} CO_CANmodule_t;


/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND()
#define CO_UNLOCK_CAN_SEND()

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY()
#define CO_UNLOCK_EMCY()

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD()
#define CO_UNLOCK_OD()

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
