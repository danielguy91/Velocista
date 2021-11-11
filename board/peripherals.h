/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_uart.h"
#include "fsl_clock.h"
#include "fsl_pit.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define UART3_PERIPHERAL UART3
/* Definition of the clock source frequency */
#define UART3_CLOCK_SOURCE 60000000UL
/* UART3 interrupt vector ID (number). */
#define UART3_SERIAL_RX_TX_IRQN UART3_RX_TX_IRQn
/* UART3 interrupt handler identifier. */
#define UART3_SERIAL_RX_TX_IRQHANDLER UART3_RX_TX_IRQHandler
/* UART3 interrupt vector ID (number). */
#define UART3_SERIAL_ERROR_IRQN UART3_ERR_IRQn
/* UART3 interrupt handler identifier. */
#define UART3_SERIAL_ERROR_IRQHANDLER UART3_ERR_IRQHandler
/* BOARD_InitPeripherals defines for PIT */
/* Definition of peripheral ID. */
#define PIT_PERIPHERAL PIT
/* Definition of clock source. */
#define PIT_CLOCK_SOURCE kCLOCK_BusClk
/* Definition of clock source frequency. */
#define PIT_CLK_FREQ 60000000UL
/* Definition of ticks count for channel 0 - deprecated. */
#define PIT_0_TICKS 599999U
/* PIT interrupt vector ID (number) - deprecated. */
#define PIT_0_IRQN PIT0_IRQn
/* PIT interrupt handler identifier - deprecated. */
#define PIT_0_IRQHANDLER PIT0_IRQHandler
/* Definition of channel number for channel 0. */
#define PIT_CHANNEL_0 kPIT_Chnl_0
/* Definition of ticks count for channel 0. */
#define PIT_CHANNEL_0_TICKS 599999U
/* PIT interrupt vector ID (number). */
#define PIT_CHANNEL_0_IRQN PIT0_IRQn
/* PIT interrupt handler identifier. */
#define PIT_CHANNEL_0_IRQHANDLER PIT0_IRQHandler

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const uart_config_t UART3_config;
extern const pit_config_t PIT_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/

void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
