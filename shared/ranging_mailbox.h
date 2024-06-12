/*
 * ranging_mailbox.h
 *
 *  Created on: May 24, 2024
 *      Author: LeeLemay
 */

#ifndef INC_RANGING_MAILBOX_H_
#define INC_RANGING_MAILBOX_H_

#include <inc/ranging_output.h>
#include <shared/ranging_timeslot.h>
#include <stdint.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

///////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////

extern Semaphore_Handle g_readSemaphore;
extern Mbox_Handle      g_mboxHandle;


////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////

// Enum for message IDs
typedef enum ipcMessageId
{
    CMD_DSS_TO_START_SENSOR_NOW = 0,
    CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT,
    CMD_DSS_TO_START_SENSOR_AT_SPECIFIC_TX_TIME,
    CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT,
    MSS_SENDS_CFG_DATA_TO_DSS,
    SET_CURRENT_TIMESLOT,
    SET_NEXT_TIMESLOT,
    DSS_REPORTS_NEXT_TIMESLOT_STARTED,
    DSS_REPORTS_SENSOR_STARTED,
    DSS_REPORTS_RESULT,
    DSS_REPORTS_SUCCESS,
    DSS_REPORTS_FAILURE,
    DSS_SEND_STRING_MESSAGE
}ipcMessageId_t;

/**
 * @brief
 *  Message passed between MSS and DSS
 */
typedef struct Ranging_MSS_DSS_Message_t
{
    ipcMessageId_t                  messageId;
    union
    {
        rangingTimeSlot_t timeSlot;
        DPC_Ranging_Data_t rangingData;
        char stringData[128];
    } data;
}Ranging_MSS_DSS_Message;

////////////////////////////////////////////////////
//  FUNCTIONS
////////////////////////////////////////////////////

void initializeMailboxWithRemote(uint8_t taskPriority,
                                 Mailbox_Type localMailboxType,
                                 Mailbox_Type remoteMailboxType,
                                 Task_Handle* readTask,
                                 ti_sysbios_knl_Task_FuncPtr taskFunction,
                                 Task_Handle* writeTask);

void ranging_mssMboxReadTask(UArg arg0, UArg arg1);

void dssReportsResult();
void dssReportsSuccess();
void dssReportsFailure();
void dssReportsTimeslotStart();
void dssReportsSensorStart();
void dssSendStringToMss(const char *string);
void sendCfgDataToDSS();
void setNextTimeSlotOnDss(rangingTimeSlot_Ptr_t   p_timeSlot);
void cmdDssToStartSensorNow(rangingTimeSlot_Ptr_t   p_timeSlot);
void cmdDssToStartSensorAtNextTimeslot(rangingTimeSlot_Ptr_t   p_timeSlot);
void cmdDssToStartSensorAtSpecificTxTime(rangingTimeSlot_Ptr_t   p_timeSlot);
void cmdDssToMsgMssAtNextTimeslot();

#endif /* INC_RANGING_MAILBOX_H_ */
