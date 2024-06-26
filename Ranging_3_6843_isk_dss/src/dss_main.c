/**
 *   @file  dss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/osal/DebugP.h>

/* Data path Include Files */
#include <inc/ranging_dpc.h>

/* Demo Include Files */
#include <inc/ranging_output.h>
#include <inc/ranging_dss.h>
#include <inc/ranging_res.h>

#include<inc/countdown_timer.h>

/* Demo Profiling Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/**
 * @brief Task Priority settings:
 */
#define RANGING_DPC_OBJDET_DPM_TASK_PRIORITY      5

/*! L3 RAM buffer for results */
// A chunk is reserved for the twiddle factor, precalculated in "computed_twiddle_factor.h"
#define RANGING_OBJDET_L3RAM_SIZE (SOC_L3RAM_SIZE - 16U * 1024U)
uint8_t gMmwL3[RANGING_OBJDET_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

 /*! L2 RAM buffer for object detection DPC */
#define RANGING_OBJDET_L2RAM_SIZE (89U * 1024U)
uint8_t gDPC_ObjDetL2Heap[RANGING_OBJDET_L2RAM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetL2Heap, ".dpc_l2Heap");

 /*! L1DSRAM RAM buffer - used to hold ADC IN*/
#define RANGING_OBJDET_L1RAM_SIZE (16U * 1024U)
uint8_t gDPC_ObjDetL1Heap[RANGING_OBJDET_L1RAM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetL1Heap, ".dpc_l1Heap");

 /*! HSRAM for processing results */
#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);

#define DPC_OBJDET_DSP_INSTANCEID       (0xDEEDDEED)

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
Ranging_DSS_MCB    gMmwDssMCB;

/**
 * @brief
 *  Global Variable for DPM result buffer
 */
DPM_Buffer  resultBuffer;

/**
 * @brief
 *  Global Variable for HSRAM buffer used to share results to remote
 */
Ranging_HSRAM gHSRAM;

/**************************************************************************
 ******************* Millimeter Wave Demo Functions Prototype *******************
 **************************************************************************/
static void Ranging_dssInitTask(UArg arg0, UArg arg1);
static void Ranging_DPC_Ranging_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
static void Ranging_DPC_Ranging_processFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void Ranging_DPC_Ranging_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void Ranging_updateObjectDetStats
(
    DPC_Ranging_Stats       *currDpcStats,
    Ranging_output_message_stats    *outputMsgStats
);

static int32_t Ranging_copyResultToHSRAM
(
    Ranging_HSRAM           *ptrHsramBuffer,
    DPC_Ranging_ExecuteResult *result,
    Ranging_output_message_stats *outStats
);
static void Ranging_DPC_Ranging_dpmTask(UArg arg0, UArg arg1);
static void Ranging_sensorStopEpilog(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_edmaInit(Ranging_DataPathObj *obj, uint8_t instance)
{
    int32_t errorCode;

    errorCode = EDMA_init(instance);
    if (errorCode != EDMA_NO_ERROR)
    {
        //System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
        Ranging_debugAssert (0);
        return;
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void Ranging_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gMmwDssMCB.dataPathObj.EDMA_errorInfo = *errorInfo;
    Ranging_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void Ranging_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    gMmwDssMCB.dataPathObj.EDMA_transferControllerErrorInfo = *errorInfo;
    Ranging_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_edmaOpen(Ranging_DataPathObj *obj, uint8_t instance)
{
    int32_t              errCode;
    EDMA_instanceInfo_t  edmaInstanceInfo;
    EDMA_errorConfig_t   errorConfig;

    obj->edmaHandle = EDMA_open(
        instance,
        &errCode, 
        &edmaInstanceInfo);
    if (obj->edmaHandle == NULL)
    {
        Ranging_debugAssert (0);
        return;
    }

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = Ranging_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = Ranging_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        Ranging_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close EDMA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_edmaClose(Ranging_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void Ranging_sensorStopEpilog(void)
{
    Hwi_StackInfo   stackInfo;
    Task_Stat       stat;
    bool            hwiStackOverflow;
/*
    System_printf("Data Path Stopped (last frame processing done)\n");
    // Print DSS task statistics
    System_printf("DSS Task Stack Usage (Note: Task Stack Usage) ==========\n");

    Task_stat(gMmwDssMCB.initTaskHandle, &stat);
    System_printf("%20s %12d %12d %12d\n", "initTask",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);

    Task_stat(gMmwDssMCB.objDetDpmTaskHandle, &stat);
    System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");
    System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);

    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    hwiStackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (hwiStackOverflow == TRUE)
    {
        System_printf("DSS HWI Stack overflowed\n");
        Ranging_debugAssert(0);
    }
    else
    {
        System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize,
                      stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
*/
}

/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_DPC_Ranging_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{

    /* Only errors are logged on the console: */
    if (errCode != 0)
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("DSSApp: DPM Report IOCTL, command = %d\n", arg0);
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("DSSApp: DPM Report start\n");
            gMmwDssMCB.dpmStartEvents++;
            /* every sensor start should cause 2 DPM start events due to distributed domain */
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * DPC Results have been passed:
             * - This implies that we have valid profile results which have
             *   been received from the profile.
             *****************************************************************/

            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /*****************************************************************
             * DPC Results have been acked:
             * - This implies that MSS received the results.
             *****************************************************************/

            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            System_printf ("DSS Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("DSSApp: DPM Report stop\n");
            gMmwDssMCB.dpmStopEvents++;
            Ranging_sensorStopEpilog();
            break;
        }
        case DPM_Report_DPC_INFO:
        {
            /* Currently chain does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing.
 *      Note: In this demo objdetdsp DPC only have inter-frame processing, hence this 
 *      callback function won't be called.
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void Ranging_DPC_Ranging_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    /* Empty function */
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of inter-frame/inter-sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during active frame (chirping)
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void Ranging_DPC_Ranging_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwDssMCB.dataPathObj.subFrameStats[subFrameIndx].interFrameCPULoad = Load_getCPULoad();
}


/**
 *  @b Description
 *  @n
 *      Update stats based on the stats from DPC
 *
 *  @param[in]  currDpcStats        Pointer to DPC status
 *  @param[in]  outputMsgStats      Pointer to Output message stats 
 *
 *  @retval
 *      Not Applicable.
 */
 void Ranging_updateObjectDetStats
(
    DPC_Ranging_Stats       *currDpcStats,
    Ranging_output_message_stats    *outputMsgStats
)
{
    static uint32_t prevInterFrameEndTimeStamp = 0U;

    /* Calculate interframe proc time */
    outputMsgStats->interFrameProcessingTime =
            (currDpcStats->interFrameEndTimeStamp - currDpcStats->interFrameStartTimeStamp)/DSP_CLOCK_MHZ; /* In micro seconds */

    outputMsgStats->interChirpProcessingMargin = currDpcStats->interChirpProcessingMargin/DSP_CLOCK_MHZ;

    /* Calculate interFrame processing Margin for previous frame, but saved to current frame */
    outputMsgStats->interFrameProcessingMargin =
        (currDpcStats->frameStartTimeStamp - prevInterFrameEndTimeStamp - currDpcStats->subFramePreparationCycles)/DSP_CLOCK_MHZ;

    prevInterFrameEndTimeStamp = currDpcStats->interFrameEndTimeStamp;
}


/**
 *  @b Description
 *  @n
 *      Copy DPC results and output stats to HSRAM to share with MSS
 *
 *  @param[in]  ptrHsramBuffer      Pointer to HSRAM buffer memory
 *  @param[in]  result              Pointer to DPC results
 *  @param[in]  outStats            Pointer to Output message stats
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Ranging_copyResultToHSRAM
(
    Ranging_HSRAM           *ptrHsramBuffer,
    DPC_Ranging_ExecuteResult *result,
    Ranging_output_message_stats *outStats
)
{
    uint8_t             *ptrCurrBuffer;
    uint32_t            totalHsramSize;
    uint32_t            itemPayloadLen;

    /* Save result in HSRAM */
    if(ptrHsramBuffer == NULL)
    {
        return -1;
    }

    /* Save result in HSRAM */
    if(result != NULL)
    {
        itemPayloadLen = sizeof(DPC_Ranging_ExecuteResult);
        memcpy((void *)&ptrHsramBuffer->result, (void *)result, itemPayloadLen);
    }
    else
    {
        return -1;
    }

    /* Save output Stats in HSRAM */
    if(outStats != NULL)
    {
        itemPayloadLen = sizeof(Ranging_output_message_stats);
        memcpy((void *)&ptrHsramBuffer->outStats, (void *)outStats, itemPayloadLen);
    }

    /* Set payload pointer to HSM buffer */
    ptrCurrBuffer = &ptrHsramBuffer->payload[0];
    totalHsramSize = RANGING_HSRAM_PAYLOAD_SIZE;

    /* Save DPC_Ranging_Stats in HSRAM */
    if(result->stats != NULL)
    {
        itemPayloadLen = sizeof(DPC_Ranging_Stats);
        if((totalHsramSize- itemPayloadLen) > 0)
        {
            memcpy(ptrCurrBuffer, (void *)result->stats, itemPayloadLen);
            ptrHsramBuffer->result.stats = (DPC_Ranging_Stats *)ptrCurrBuffer;
            ptrCurrBuffer+= itemPayloadLen;
            totalHsramSize -=itemPayloadLen;
        }
        else
        {
            return -1;
        }
    }

    /* Save DPC_Ranging_Data in HSRAM */
    if(result->rangingData != NULL)
    {
        itemPayloadLen = sizeof(DPC_Ranging_Data);
        if((totalHsramSize- itemPayloadLen) > 0)
        {
            memcpy(ptrCurrBuffer, (void *)result->rangingData, itemPayloadLen);
            ptrHsramBuffer->result.rangingData = (DPC_Ranging_Data *)ptrCurrBuffer;
            ptrCurrBuffer+= itemPayloadLen;
            totalHsramSize -=itemPayloadLen;
        }
        else
        {
            return -1;
        }
    }

    return totalHsramSize;
}

/**
 *  @b Description
 *  @n
 *      DPM Execution Task. DPM execute results are processed here:
 *      a) Update states based on timestamp from DPC.
 *      b) Copy results to shared memory to be shared with MSS.
 *      c) Send Results to MSS by calling DPM_sendResult()
 *
 *  @retval
 *      Not Applicable.
 */
#include <ti/alg/mmwavelib/src/fft/mmwavelib_fft.h>
static void Ranging_DPC_Ranging_dpmTask(UArg arg0, UArg arg1)
{
    int32_t     retVal;
    DPC_Ranging_ExecuteResult *result;
    volatile uint32_t              startTime;

    while (1)
    {
        /* Execute the DPM module: */
        retVal = DPM_execute (gMmwDssMCB.dataPathObj.objDetDpmHandle, &resultBuffer);
        if (retVal < 0)
        {
            System_printf ("Error: DPM execution failed [Error code %d]\n", retVal);
            Ranging_debugAssert (0);
        }
        else
        {
            if ((resultBuffer.size[0] == sizeof(DPC_Ranging_ExecuteResult)))
            {
                result = (DPC_Ranging_ExecuteResult *)resultBuffer.ptrBuffer[0];

                /* Get the time stamp before copy data to HSRAM */
                startTime = Cycleprofiler_getTimeStamp();

                /* Update processing stats and added it to buffer 1*/
                Ranging_updateObjectDetStats(result->stats,
                                                &gMmwDssMCB.dataPathObj.subFrameStats[result->subFrameIdx]);

                /* Copy result data to HSRAM */
                if ((retVal = Ranging_copyResultToHSRAM(&gHSRAM, result, &gMmwDssMCB.dataPathObj.subFrameStats[result->subFrameIdx])) >= 0)
                {
                    /* Update interframe margin with HSRAM copy time */
                    gHSRAM.outStats.interFrameProcessingMargin -= ((Cycleprofiler_getTimeStamp() - startTime)/DSP_CLOCK_MHZ);

                    /* Update DPM buffer */
                    resultBuffer.ptrBuffer[0] = (uint8_t *)&gHSRAM.result;
                    resultBuffer.ptrBuffer[1] = (uint8_t *)&gHSRAM.outStats;
                    resultBuffer.size[1] = sizeof(Ranging_output_message_stats);

                    /* YES: Results are available send them. */
                    retVal = DPM_sendResult (gMmwDssMCB.dataPathObj.objDetDpmHandle, true, &resultBuffer);
                    if (retVal < 0)
                    {
                        System_printf ("Error: Failed to send results [Error: %d] to remote\n", retVal);
                    }
                }
                else
                {
                    System_printf ("Error: Failed to copy processing results to HSRAM, error=%d\n", retVal);
                    Ranging_debugAssert (0);
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    Task_Params         taskParams;
    DPM_InitCfg         dpmInitCfg;
    DPC_Ranging_InitParams      objDetInitParams;

    /*****************************************************************************
     * Driver Init:
     *****************************************************************************/

    // Initialize the Mailbox
    Mailbox_init(MAILBOX_TYPE_DSS);

    //*****************************************************************************
    //* Driver Open/Configuraiton:
    //*****************************************************************************
    // Initialize EDMA
    Ranging_edmaInit(&gMmwDssMCB.dataPathObj, DPC_RANGING_DSP_EDMA_INSTANCE);

    // Use instance 1 on DSS
    Ranging_edmaOpen(&gMmwDssMCB.dataPathObj, DPC_RANGING_DSP_EDMA_INSTANCE);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset ((void *)&objDetInitParams, 0, sizeof(DPC_Ranging_InitParams));

    objDetInitParams.L3ramCfg.addr = (void *)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreL2RamCfg.addr = &gDPC_ObjDetL2Heap[0];
    objDetInitParams.CoreL2RamCfg.size = sizeof(gDPC_ObjDetL2Heap);
    objDetInitParams.CoreL1RamCfg.addr = &gDPC_ObjDetL1Heap[0];
    objDetInitParams.CoreL1RamCfg.size = sizeof(gDPC_ObjDetL1Heap);

    /* initialize edma handles, unused handles will remain NULL to to memset above */
    objDetInitParams.edmaHandle[DPC_RANGING_DSP_EDMA_INSTANCE] = gMmwDssMCB.dataPathObj.edmaHandle;

    /* DPC Call-back config */
    objDetInitParams.processCallBackCfg.processFrameBeginCallBackFxn =
        Ranging_DPC_Ranging_processFrameBeginCallBackFxn;
    objDetInitParams.processCallBackCfg.processInterFrameBeginCallBackFxn =
        Ranging_DPC_Ranging_processInterFrameBeginCallBackFxn;

    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    /* Setup the configuration: */
    dpmInitCfg.socHandle        = gMmwDssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = &gDPC_RangingCfg;
    dpmInitCfg.instanceId       = DPC_OBJDET_DSP_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_REMOTE;
    dpmInitCfg.reportFxn        = Ranging_DPC_Ranging_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_Ranging_InitParams);

    /* Initialize the DPM Module: */
    gMmwDssMCB.dataPathObj.objDetDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gMmwDssMCB.dataPathObj.objDetDpmHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        Ranging_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gMmwDssMCB.dataPathObj.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            Ranging_debugAssert (0);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    System_printf ("Debug: DPM Module Sync is done\n");

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority = RANGING_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 4*1024;
    gMmwDssMCB.objDetDpmTaskHandle = Task_create(Ranging_DPC_Ranging_dpmTask, &taskParams, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction.
 *     When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function.
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" IDLE ");
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwDssMCB, 0, sizeof(Ranging_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        Ranging_debugAssert (0);
        return -1;
    }

    gMmwDssMCB.socHandle = socHandle;

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    gMmwDssMCB.initTaskHandle = Task_create(Ranging_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
