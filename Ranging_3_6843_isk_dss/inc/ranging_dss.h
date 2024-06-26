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

#include <inc/ranging_output.h>
#include <inc/ranging_dpc.h>

/* This is used to resolve RL_MAX_SUBFRAMES, TODO: wired */
#include <ti/control/mmwavelink/mmwavelink.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Ranging_DataPathObj_t
{
    /*! @brief dpm Handle */
    DPM_Handle          objDetDpmHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle         edmaHandle;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t    EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief          Processing Stats */
    Ranging_output_message_stats   subFrameStats[RL_MAX_SUBFRAMES];
} Ranging_DataPathObj;

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

    /*! @brief     Data Path object */
    Ranging_DataPathObj         dataPathObj;

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

