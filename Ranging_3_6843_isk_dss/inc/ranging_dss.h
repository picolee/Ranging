/**
 *   @file  mmw_dss.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef RANGING_DSS_H
#define RANGING_DSS_H

#include <ti/sysbios/knl/Task.h>

#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <inc/ranging_output.h>
#include <inc/ranging_dpc.h>
#include <inc/ranging_dpc_internal.h>
#include <shared/ranging_timeslot.h>

/* This is used to resolve RL_MAX_SUBFRAMES, TODO: wired */
#include <ti/control/mmwavelink/mmwavelink.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Ranging_DataPathObj_t
{
    /*! @brief dpm Handle */
    DPM_Handle          objDetDpmHandle;

    /*! @brief   Handle and channel configuration of the EDMA driver. */
    DPU_RangingDSP_EDMAConfig     edmaCfg;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t    EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief          Processing Stats */
    Ranging_output_message_stats   subFrameStats[RL_MAX_SUBFRAMES];
} Ranging_DataPathObj;

/** @}*/ /* configStoreOffsets */

/**
 * @brief
 *  Millimeter Wave Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in mmwDemo
 */
typedef enum Ranging_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    Ranging_SensorState_INIT = 0,

    /*!  @brief Inital state after sensor is post RF init.
     */
    Ranging_SensorState_OPENED,

    /*!  @brief Indicates sensor is started */
    Ranging_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    Ranging_SensorState_STOPPED
}Ranging_SensorState;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_DSS_STATS_t
{
    /*! @brief   Counter which tracks the number of config events
                 The config event is triggered in mmwave config callback function
                 when remote sends configuration */
    uint16_t configEvt;

    /*! @brief   Counter which tracks the number of open events
                 The open event is triggered in mmwave open callback function
                 when remote calls mmwave_open() */
    uint16_t openEvt;

    /*! @brief   Counter which tracks the number of close events
                 The close event is triggered in mmwave close callback function
                 when remote calls mmwave_close() */
    uint16_t closeEvt;

    /*! @brief   Counter which tracks the number of start events
                 The start event is triggered in mmwave start callback function
                 when remote calls mmwave_start() or when the local controller opens the sensor*/
    uint16_t sensorStartCount;

    /*! @brief   Counter which tracks the number of stop events
                 The start event is triggered in mmwave stop callback function
                 when remote calls mmwave_stop() or when the local controller closes the sensor*/
    uint16_t sensorStopCount;

    /*! @brief   Counter which tracks the number of chirp interrupt detected */
    uint32_t chirpIntCounter;

    /*! @brief   Counter which tracks the number of frame start interrupt  detected */
    uint32_t frameStartIntCounter;

    /*! @brief   Counter which tracks the number of chirp event detected
                 The chirp event is triggered in the ISR for chirp interrupt */
    uint32_t chirpEvt;

    /*! @brief   Counter which tracks the number of frame start event detected
                 The frame start event is triggered in the ISR for frame start interrupt */
    uint32_t frameStartEvt;

    /*! @brief   Counter which tracks the number of Failed Timing Reports received from BSS  */
    uint32_t numFailedTimingReports;

    /*! @brief   Counter which tracks the number of Calibration Report received from BSS  */
    uint32_t numCalibrationReports;

    /*! @brief   Counter which tracks the number of frames triggered in BSS detected
                 The frame trigger event is triggered in the mmwave async event callback function */
    uint32_t frameTrigEvt;

    /*! @brief   Counter which tracks the number of times saving detected objects in
                 logging buffer is skipped */
    uint32_t detObjLoggingSkip;

    /*! @brief   Counter which tracks the number of times saving detected objects in
                 logging buffer has an error */
    uint32_t detObjLoggingErr;

    /*! @brief   Counter which tracks the number of times ARM fails to ship out UART info on time */
    uint32_t detObjLoggingBufNotRdyCnt;
} Ranging_DSS_STATS;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct Ranging_DSS_MCB_t
{
    /*! * @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief     DPM Handle */
    Task_Handle                 objDetDpmTaskHandle;

    /*! @brief     init Task Handle */
    Task_Handle                 initTaskHandle;

    /*! @brief     init Task Handle */
    Task_Handle                 mboxReadTaskHandle;

    /*! @brief     init Task Handle */
    Task_Handle                 mboxWriteTaskHandle;

    /*! @brief     init Task Handle */
    Task_Handle                 precisionTimerTaskHandle;

    /*! @brief   DSS event handle */
    Event_Handle                eventHandle;

    // Used to ensure we don't start the sensor before it is configured
    Semaphore_Handle            sensorConfigSemaphore;

    /*! @brief     Data Path object */
    Ranging_DataPathObj         edmaContainer;

    /*! @brief     Data Path objec */
    rangingDataPathObject_t     dataPathObject;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg              ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg              openCfg;

    /*! @brief   Current time slot */
    rangingTimeSlot_t           currentTimeslot;

    /*! @brief   Next up time slot */
    rangingTimeSlot_t           nextTimeslot;

    /*! @brief    Sensor state */
    Ranging_SensorState         sensorState;


    /*! @brief    Sensor state */
    Ranging_DSS_STATS           stats;

    /*! @brief   Counter which tracks the number of dpm stop events received
                 The event is triggered by DPM_Report_DPC_STOPPED from DPM */
    uint32_t                    dpmStopEvents;

    /*! @brief   Counter which tracks the number of dpm start events received
                 The event is triggered by DPM_Report_DPC_STARTED from DPM */
    uint32_t                    dpmStartEvents;
} Ranging_DSS_MCB;


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern void Ranging_dataPathInit(Ranging_DataPathObj *obj);
extern void Ranging_dataPathOpen(Ranging_DataPathObj *obj);
extern void Ranging_dataPathClose(Ranging_DataPathObj *obj);

/* Sensor Management Module Exported API */
extern void _Ranging_debugAssert(int32_t expression, const char *file, int32_t line);
#define Ranging_debugAssert(expression) {                                      \
                                         DebugP_assert(expression);             \
                                        }
                                        
#ifdef __cplusplus
}
#endif

#endif /* RANGING_DSS_H */

