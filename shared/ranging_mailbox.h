/*
 * ranging_mailbox.h
 *
 *  Created on: May 24, 2024
 *      Author: LeeLemay
 */

#ifndef INC_RANGING_MAILBOX_H_
#define INC_RANGING_MAILBOX_H_

#include <stdint.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/sysbios/knl/Task.h>

///////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////

extern Semaphore_Handle g_semaphoreHandle;
extern Mbox_Handle      g_mboxHandle;


////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////

// Enum for message IDs
typedef enum ipcMessageId
{
    CMD_DSS_TO_START_SENSOR = 0,
    DSS_REPORTS_SUCCESS,
    DSS_REPORTS_FAILURE
    // Add other message IDs here
}ipcMessageId_t;

/**
 * @brief
 *  Message passed between MSS and DSS
 */
typedef struct Ranging_MSS_DSS_Message_t
{
    /*! @brief Message ID */
    ipcMessageId_t    messageId;
}Ranging_MSS_DSS_Message;

////////////////////////////////////////////////////
//  FUNCTIONS
////////////////////////////////////////////////////

void initializeMailboxWithRemote(uint8_t taskPriority,
                                 Mailbox_Type localMailboxType,
                                 Mailbox_Type remoteMailboxType,
                                 Task_Handle* task,
                                 ti_sysbios_knl_Task_FuncPtr taskFunction);

void dssReportsSuccess();
void dssReportsFailure();
void ranging_mssMboxReadTask(UArg arg0, UArg arg1);
void cmdDssToStartSensor();

#endif /* INC_RANGING_MAILBOX_H_ */
