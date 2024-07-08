/*
 * uart_logging.c
 *
 *  Created on: Jun 25, 2024
 *      Author: LeeLemay
 */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  INCLUDES
 * ----------------------------------------------------------------------------------------------------------------- */
#include <inc/uart_logging.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <stdio.h>
#include <string.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  DEFINES
 * ----------------------------------------------------------------------------------------------------------------- */

//#define ENABLE_UART_LOGGING

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Local Variables
 * ----------------------------------------------------------------------------------------------------------------- */

char logString[128];
char buffer[128];

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Functions
 * ----------------------------------------------------------------------------------------------------------------- */

void Log_To_Uart(UART_Handle uartHandle, const char* format, ...)
{
#ifdef ENABLE_UART_LOGGING
    // Make sure we're not in an ISR
    if(BIOS_getThreadType() == BIOS_ThreadType_Task)
    {
        if (uartHandle != NULL)
        {
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);

            snprintf(logString, sizeof(logString), "%u %s", Cycleprofiler_getTimeStamp(), buffer);

            UART_writePolling(uartHandle, (uint8_t*)logString, strlen(logString));
        }
    }
#endif
}

void Priority_Log_To_Uart(UART_Handle uartHandle, const char* format, ...)
{
    // Make sure we're not in an ISR
    if(BIOS_getThreadType() == BIOS_ThreadType_Task)
    {
        if (uartHandle != NULL)
        {
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);

            snprintf(logString, sizeof(logString), "%u %s", Cycleprofiler_getTimeStamp(), buffer);

            UART_writePolling(uartHandle, (uint8_t*)logString, strlen(logString));
        }
    }
}
