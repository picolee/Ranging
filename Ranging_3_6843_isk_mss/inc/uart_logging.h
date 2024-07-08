/*
 * uart_logging.h
 *
 *  Created on: Jun 25, 2024
 *      Author: LeeLemay
 */

#ifndef INC_UART_LOGGING_H_
#define INC_UART_LOGGING_H_


/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  INCLUDES
 * ----------------------------------------------------------------------------------------------------------------- */
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Function Declarations
 * ----------------------------------------------------------------------------------------------------------------- */
void Log_To_Uart(UART_Handle uartHandle, const char* format, ...);
void Priority_Log_To_Uart(UART_Handle uartHandle, const char* format, ...);


#endif /* INC_UART_LOGGING_H_ */
