 /* 
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
/**
 *   @file  rangingdsp.c
 *
 *   @brief
 *      Implements ranging functionality on DSP.
 */

/////////////////////////////////////////////////////////////////////////
//                   Include Files
/////////////////////////////////////////////////////////////////////////

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmwave SDK include files */
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <ti/alg/mmwavelib/mmwavelib.h>

#include <xdc/runtime/System.h>

/* C64P dsplib (fixed point part for C674X) */
#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/DSP_fft16x16_imre.h>
#include <ti/dsplib/src/DSP_ifft16x32/c64P/DSP_ifft16x32.h>
//#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/gen_twiddle_fft16x16_imre.h>

// Copied over along with the C source file from DSPLIB
#include <inc/gen_twiddle_fft16x32.h>

/* Internal include Files */
#include <inc/rangingdsp_internal.h>
#include <inc/gold_code.h>
#include <inc/line_fit.h>
#include <inc/computed_twiddle_factor.h>
#include <shared/ranging_rfConfig.h>

/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>

/////////////////////////////////////////////////////////////////////////
//                  Defines
/////////////////////////////////////////////////////////////////////////

#define     DEBUG_CHECK_PARAMS      1
#define     DETECTION_THRESHOLD     1e13f

/* Macros to determine pingpong index */
#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x) == 1U)
#define BYTES_PER_SAMP_1D   sizeof(cmplx16ImRe_t)


#pragma SET_CODE_SECTION(".l1pcode")

/////////////////////////////////////////////////////////////////////////
//                  Internal Function prototype
/////////////////////////////////////////////////////////////////////////
static void rangingDSP_WaitEDMAComplete
(
    EDMA_Handle         edmaHandle,
    uint8_t             chId
);

static int32_t rangingDSP_ConfigDataInEDMA
(
    rangingDSPObj          *rangingObj,
    DPU_RangingDSP_HW_Resources  *hwRes
);

static int32_t rangingDSP_ConfigDataOutEDMA
(
    rangingDSPObj          *rangingObj,
    DPU_RangingDSP_HW_Resources  *hwRes
);

/**
 *  @b Description
 *  @n
 *      Polling function to wait for EDMA transper complete.
 *
 *  @param[in]  edmaHandle              EDMA Handle
 *  @param[in]  chId                    EDMA channel id
 *
 *  \ingroup    DPU_ranging_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 * 
 */
static void rangingDSP_WaitEDMAComplete
(
    EDMA_Handle         edmaHandle,
    uint8_t             chId
)
{
    volatile bool isTransferDone = false;
    do {
        if (EDMA_isTransferComplete(edmaHandle,
                                    (uint8_t) chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
        }
    } while (isTransferDone == false);
}

int32_t DPEDMA_setup_shadow_link_nonlib
(
    EDMA_Handle     handle,
    uint8_t         chId,
    uint16_t        shadowParamId,
    EDMA_paramSetConfig_t *config,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t       transferCompletionCallbackFxnArg
)
{
    EDMA_paramConfig_t paramConfig;
    int32_t errorCode = EDMA_NO_ERROR;

    paramConfig.paramSetConfig = *config; //this will copy the entire param set config
    paramConfig.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    paramConfig.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;

    errorCode = EDMA_configParamSet(handle, shadowParamId, &paramConfig);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configParamSet() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_linkParamSets(handle, (uint16_t) chId, shadowParamId);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_linkParamSets() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_linkParamSets(handle, shadowParamId, shadowParamId);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_linkParamSets() failed with error code = %d\n", errorCode);
        goto exit;
    }

exit:
    return(errorCode);
}

int32_t DPEDMA_configSyncA_singleFrame_nonlib
(
    EDMA_Handle             handle,
    DPEDMA_ChanCfg          *chanCfg,
    DPEDMA_ChainingCfg      *chainingCfg,
    DPEDMA_syncACfg         *syncACfg,
    bool                    isEventTriggered,
    bool                    isIntermediateTransferInterruptEnabled,
    bool                    isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t               transferCompletionCallbackFxnArg
)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    if ((chanCfg == NULL) || (syncACfg == NULL))
    {
        errorCode = DPEDMA_EINVAL;
        goto exit;
    }

    config.channelId    = chanCfg->channel;
    config.channelType  = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId      = chanCfg->channel;
    config.eventQueueId = chanCfg->eventQueue;

    config.paramSetConfig.sourceAddress = SOC_translateAddress(
                                            syncACfg->srcAddress,
                                            SOC_TranslateAddr_Dir_TO_EDMA,
                                            NULL);

    config.paramSetConfig.destinationAddress = SOC_translateAddress(
                                            syncACfg->destAddress,
                                            SOC_TranslateAddr_Dir_TO_EDMA,
                                            NULL);

    config.paramSetConfig.aCount = syncACfg->aCount;
    config.paramSetConfig.bCount = syncACfg->bCount;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = config.paramSetConfig.bCount;

    config.paramSetConfig.sourceBindex = syncACfg->srcBIdx;
    config.paramSetConfig.destinationBindex = syncACfg->dstBIdx;

    config.paramSetConfig.sourceCindex = 0;
    config.paramSetConfig.destinationCindex = 0;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled =
        isTransferCompletionEnabled;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled =
        isIntermediateTransferInterruptEnabled;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;

    if(chainingCfg == NULL)
    {
        config.paramSetConfig.transferCompletionCode = chanCfg->channel;
        config.paramSetConfig.isFinalChainingEnabled = false;
        config.paramSetConfig.isIntermediateChainingEnabled = false;
    }
    else
    {
        config.paramSetConfig.transferCompletionCode = chainingCfg->chainingChan;
        config.paramSetConfig.isFinalChainingEnabled =
            chainingCfg->isFinalChainingEnabled;
        config.paramSetConfig.isIntermediateChainingEnabled =
            chainingCfg->isIntermediateChainingEnabled;
    }

    if (transferCompletionCallbackFxn != NULL) {
        config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;
    }

    errorCode = EDMA_configChannel(handle, &config, isEventTriggered);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = DPEDMA_setup_shadow_link_nonlib(handle, chanCfg->channel, chanCfg->channelShadow,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Helper function to configuration data in EDMA
 *
 *  @param[in]  rangingObj             Pointer to ranging object
 *  @param[in]  hwRes                    Pointer to hard resource configuration
 *
 *  \ingroup    DPU_ranging_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangingDSP_ConfigDataInEDMA
(
    rangingDSPObj          *rangingObj,
    DPU_RangingDSP_HW_Resources  *hwRes
)
{
    int32_t retVal;
    ranging_dpParams   *dpParams;
    DPEDMA_syncACfg         syncACfg;

    dpParams = &rangingObj->DPParams;


    /*
     * Copy an antennas worth
     * of contiguous cmplx16ImRe_t samples into the data in buffer
     ***/

    // aCount: One antenna's worth
    syncACfg.aCount = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);

    // bCount is 1: We get the only antenna's worth of data in a single copy operation
    syncACfg.bCount = 1;

    // srcBIdx don't care
    syncACfg.srcBIdx = 0U ;

    // dstBidx don't care
    syncACfg.dstBIdx = 0U;

    // Only PING configuration
    syncACfg.srcAddress = (uint32_t)rangingObj->ADCdataBuf;
    syncACfg.destAddress = (uint32_t)rangingObj->adcDataInL1_16kB;

    retVal = DPEDMA_configSyncA_singleFrame_nonlib(
                hwRes->edmaCfg.edmaHandle,
                &hwRes->edmaCfg.dataInPing,
                NULL,  // no Chaining
                &syncACfg,
                false,
                true,
                true,
                NULL,
                NULL);
    if (retVal < 0)
    {
        goto exit;
    }
exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Helper function to configuration data out EDMA
 *
 *  @param[in]  rangingObj             Pointer to ranging object
 *  @param[in]  hwRes                    Pointer to hard resource configuration
 *
 *  \ingroup    DPU_ranging_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangingDSP_ConfigDataOutEDMA
(
    rangingDSPObj          *rangingObj,
    DPU_RangingDSP_HW_Resources  *hwRes
)
{
    int32_t     retVal;
    DPEDMA_syncACfg         syncACfg;
    ranging_dpParams      *dpParams;
    dpParams = &rangingObj->DPParams;

    /*****************************************************
     * EDMA configuration for storing 1d fft output to L3.
     * It copies all Rx antennas of the chirp per trigger event.
     * The EDMA transfer is called per chirp.
     *****************************************************/
//    samplesPerChirp = dpParams->numAdcSamples * dpParams->numRxAntennas;


    // Here is the destination address that is set when the EDMA transfer is triggered:
    // radarCubeAddr = (uint32_t)(rangingObj->radarCubebuf + rangingObj->chirpCount * rangingObj->numSamplePerChirp);



    // aCount: One antenna's worth
    syncACfg.aCount = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);

    // bCount is 1: We get the only antenna's worth of data in a single copy operation
    syncACfg.bCount = 1;

    // srcBIdx don't care
    syncACfg.srcBIdx = 0U ;

    // dstBidx don't care
    syncACfg.dstBIdx = 0U;

    // scratchBuffer is in L2. radarCubebuf is in L3
    syncACfg.srcAddress = (uint32_t)rangingObj->scratchBufferTwoL2_32kB;
    syncACfg.destAddress= (uint32_t)rangingObj->radarCubebuf;

    retVal = DPEDMA_configSyncA_singleFrame_nonlib(
                hwRes->edmaCfg.edmaHandle,
                &hwRes->edmaCfg.dataOutPing,
                NULL,  // no Chaining
                &syncACfg,
                false,
                true,
                true,
                NULL,
                NULL);
    if (retVal < 0)
    {
        goto exit;
    }

    exit:
        return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to parse ranging configuration and save in internal ranging object
 *
 *  @param[in]  rangingObj              Pointer to ranging object
 *  @param[in]  pConfigIn                 Pointer to rangingDSP configuration structure
 *
 *  \ingroup    DPU_ranging_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
#pragma FUNCTION_OPTIONS(rangingDSP_ParseConfig, "--opt_for_speed")
#pragma CODE_SECTION(rangingDSP_ParseConfig, ".l1pcode")
static int32_t rangingDSP_ParseConfig
(
    rangingDSPObj          *rangingObj,
    DPU_RangingDSP_Config  *pConfigIn
)
{
    int32_t                 retVal = 0;
    ranging_dpParams  *params;
    DPU_RangingDSP_StaticConfig      *pStaticCfg;
    DPU_RangingDSP_HW_Resources      *pHwRes;

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    pHwRes = &pConfigIn->hwRes;
    params    = &rangingObj->DPParams;

    /* Save datapath parameters */
    params->numTxAntennas           = pStaticCfg->numTxAntennas;
    params->numRxAntennas           = pStaticCfg->ADCBufData.dataProperty.numRxAntennas;
    params->numVirtualAntennas      = pStaticCfg->numVirtualAntennas;
    params->numChirpsPerChirpEvent  = pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent;
    params->numAdcSamples           = pStaticCfg->ADCBufData.dataProperty.numAdcSamples;
    params->numChirpsPerFrame       = pStaticCfg->numChirpsPerFrame;
    params->adcSampleRate           = pStaticCfg->adcSampleRate;

    /* Save EDMA Handle */
    rangingObj->edmaHandle = pHwRes->edmaCfg.edmaHandle;

    // Save interface buffers
    rangingObj->ADCdataBuf                  = (cmplx16ImRe_t *)pStaticCfg->ADCBufData.data;
    rangingObj->radarCubebuf                = (cmplx16ImRe_t *)pHwRes->radarCube.data;
    rangingObj->magnitudeDataL3             = (cmplx16ImRe_t *)(((char *)rangingObj->radarCubebuf)          + pStaticCfg->ADCBufData.dataSize);     // Each value 2 bytes
    rangingObj->fftOfMagnitudeL3            = (cmplx16ImRe_t *)(((char *)rangingObj->magnitudeDataL3)       + pStaticCfg->ADCBufData.dataSize);     // Each value 2 bytes
    rangingObj->vectorMultiplyOfFFtedDataL3 = (cmplx32ImRe_t *)(((char *)rangingObj->fftOfMagnitudeL3)      + pStaticCfg->ADCBufData.dataSize);     // Each value 4 bytes, takes up twice as much room
    rangingObj->iFftDataL3                  = (cmplx32ImRe_t *)(((char *)rangingObj->vectorMultiplyOfFFtedDataL3)    + 2 * pStaticCfg->ADCBufData.dataSize); // Each value 4 bytes
    rangingObj->magIfftDataL3               = (uint32_t      *)(((char *)rangingObj->iFftDataL3)            + 2 * pStaticCfg->ADCBufData.dataSize);     // Each value 4 bytes, but only 1 value per sample
    rangingObj->fftGoldCodeL3_16kB          = (cmplx16ImRe_t *)(((char *)rangingObj->magIfftDataL3)         + pStaticCfg->ADCBufData.dataSize); // Each value 2 bytes
    rangingObj->ifftTwiddle16x16L3_16kB     = (cmplx16ImRe_t *)(((char *)rangingObj->fftGoldCodeL3_16kB)    + pStaticCfg->ADCBufData.dataSize); // Each value 2 bytes
    rangingObj->fftTwiddle16x16L3_16kB      = (cmplx16ImRe_t *)twiddle_factors;     // Each value 2 bytes

    /* Save Scratch buffers */
    rangingObj->scratchBufferOneL2_32kB     = pHwRes->fftTwiddle16x16L2_16kB;   // 32kB contiguous RAM from fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB
    rangingObj->scratchBufferTwoL2_32kB     = pHwRes->scratchBufferL2_32kB;
    rangingObj->adcDataInL1_16kB            = pHwRes->adcDataInL1_16kB;

    /* Scratch windowing & twiddle buffers */
    rangingObj->fftTwiddle16x16L2_16kB      = pHwRes->fftTwiddle16x16L2_16kB;
    rangingObj->ifftTwiddle16x32L2_16kB     = pHwRes->ifftTwiddle16x16L2_16kB;

    rangingObj->fftGoldCodeL2_16kB          = pHwRes->localGoldCodeFFTBufferL2_16kB;

    if(params->numRxAntennas > 1)
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }

    /* Prepare Ping/Pong EDMA data in/out channels */
    rangingObj->dataInChan[0] = pHwRes->edmaCfg.dataInPing.channel;
    rangingObj->dataInChan[1] = pHwRes->edmaCfg.dataInPong.channel;
    rangingObj->dataOutChan[0] = pHwRes->edmaCfg.dataOutPing.channel;
    rangingObj->dataOutChan[1] = pHwRes->edmaCfg.dataOutPong.channel;

    /* Calculation used at runtime */
    rangingObj->numSamplePerChirp       = params->numAdcSamples * params->numRxAntennas ;
    rangingObj->numSamplePerTx          = params->numChirpsPerFrame *rangingObj->numSamplePerChirp;
    rangingObj->DPParams.adcSampleRate  = pConfigIn->staticCfg.adcSampleRate;

exit:
    return(retVal);
}



/**
 *  @b Description
 *  @n
 *      The function is ranging DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_ranging_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid ranging handle
 *  @retval
 *      Error       - NULL
 */
DPU_RangingDSP_Handle DPU_RangingDSP_init
(
    int32_t*    errCode
)
{
    rangingDSPObj *rangingObj;

    rangingObj = MemoryP_ctrlAlloc(sizeof(rangingDSPObj), 0);
    if(rangingObj == NULL)
    {
        *errCode = DPU_RANGINGDSP_ENOMEM;
        return NULL;
    }

    /* Initialize memory */
    memset((void *)rangingObj, 0, sizeof(rangingDSPObj));

    return ((DPU_RangingDSP_Handle)rangingObj);
}

/**
 *  @b Description
 *  @n
 *      The function is ranging DPU config function. It saves buffer pointer and configurations
 *  including system resources and configures EDMA for runtime range processing.
 *  
 *  @pre    DPU_rangingDSP_init() has been called
 *
 *  @param[in]  handle                  ranging DPU handle
 *  @param[in]  pConfig                 Pointer to ranging configuration data structure
 *
 *  \ingroup    DPU_ranging_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
#pragma FUNCTION_OPTIONS(DPU_RangingDSP_config, "--opt_for_speed")
#pragma CODE_SECTION(DPU_RangingDSP_config, ".l1pcode")
int32_t DPU_RangingDSP_config
(
    DPU_RangingDSP_Handle     handle,
    DPU_RangingDSP_Config*    pConfig
)
{
    rangingDSPObj               *rangingObj;
    DPU_RangingDSP_StaticConfig *pStaticCfg;
    DPU_RangingDSP_HW_Resources *pHwRes;
    int32_t                     retVal = 0;
    gold_code_struct_t          gold_code;
    gold_code_struct_t          sampled_gold_code;
    double                      sample_rate      = RX_SAMPLE_RATE_KSPS * 1000;
    double                      chip_duration    = ((double)TX_RAMP_DURATION_US)/((double)1000000.0);
    double                      zeros_duration   = ((double)TX_IDLE_TIME_US)/((double)1000000.0);
    int16_t                     index;
    int16_t                     first_non_zero_index = -1;
    int16_t                     last_non_zero_index = -1;

    rangingObj = (rangingDSPObj *)handle;
    if(rangingObj == NULL)
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }

    pStaticCfg = &pConfig->staticCfg;
    pHwRes = &pConfig->hwRes;

#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!pConfig->hwRes.edmaCfg.edmaHandle ||
      !pHwRes->radarCube.data ||
      !pStaticCfg->ADCBufData.data||
      !pHwRes->adcDataInL1_16kB ||
      !pHwRes->scratchBufferL2_32kB ||
      !pHwRes->fftTwiddle16x16L2_16kB
      )
    {
        retVal = DPU_RANGINGDSP_EBUFFER_POINTER;
        goto exit;
    }

    /* Validate buffer size */
    if(
      (pHwRes->radarCube.dataSize < pStaticCfg->ADCBufData.dataProperty.numAdcSamples * pStaticCfg->numChirpsPerFrame * sizeof(cmplx16ImRe_t) *
                                  pStaticCfg->ADCBufData.dataProperty.numRxAntennas) ||
      (pHwRes->adcDataInSize < sizeof(cmplx16ImRe_t) * pStaticCfg->ADCBufData.dataProperty.numAdcSamples ) ||
      (pHwRes->scratchBufferSize < sizeof(cmplx16ImRe_t) * pStaticCfg->ADCBufData.dataProperty.numAdcSamples * pStaticCfg->ADCBufData.dataProperty.numRxAntennas * 2U) ||
      (pHwRes->fftTwiddleSize < sizeof(cmplx16ImRe_t) * pStaticCfg->ADCBufData.dataProperty.numAdcSamples))
    {
        retVal = DPU_RANGINGDSP_EBUFFER_SIZE;
        goto exit;
    }

    if(rangingObj->inProgress == true)
    {
        retVal = DPU_RANGINGDSP_EINPROGRESS;
        goto exit;
    }

    /* Parameter check: validate Adc data interface configuration
        Support:
            - Complex 16bit ADC data in IMRE format
            - Non-interleaved mode
     */
    if( (pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_COMPLEX16_IMRE) ||
       (pStaticCfg->ADCBufData.dataProperty.interleave != DPIF_RXCHAN_NON_INTERLEAVE_MODE) )
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }

    /* Validate dp radarCube interface */
    if (pConfig->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_1)
    {
        /* Only one format is supported */
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }
#endif
    /* Save hardware resources */

    retVal = rangingDSP_ParseConfig(rangingObj, pConfig);
    if(retVal < 0)
    {
        goto exit;
    }

    memcpy(rangingObj->fftTwiddle16x16L2_16kB, twiddle_factors, rangingObj->DPParams.numAdcSamples * sizeof(cmplx16ImRe_t) );

    // Generate twiddle factors for the IFFT. This is one time
    gen_twiddle_fft16x32((short *)rangingObj->ifftTwiddle16x32L2_16kB, rangingObj->DPParams.numAdcSamples);

    ////////////////////////////////////////////////////////////////////////////////
    // Gold code, length 2^N - 1
    rangingObj->gold_code_n = 6;
    rangingObj->rxPrn = pStaticCfg->rxPrn;
    gold_code.data = NULL;
    if (generate_one_gold_sequence(
        rangingObj->gold_code_n,
        &gold_code,
        rangingObj->rxPrn))
    {
        if(gold_code.data != NULL)
        {
            free(gold_code.data);
        }
        goto exit;
    }

    /////////////////////////////////////////////////////////////////////////////////
    // Upsample the gold code to our sample rate
    // Include idle time periods that occur between chirps
    sampled_gold_code.data = NULL;
    if (sample_gold_code_with_idle_time(
        &sampled_gold_code,
        &gold_code,
        sample_rate,
        chip_duration,
        zeros_duration))
    {
        if(gold_code.data != NULL)
        {
            free(gold_code.data);
        }
        if(sampled_gold_code.data != NULL)
        {
            free(sampled_gold_code.data);
        }
        goto exit;
    }
    //memcpy(rangingObj->radarCubebuf, sampled_gold_code.data, sampled_gold_code.length * sizeof(int16_t));

    //////////////////////////////////////////////////////////
    // Transform to complex
    // Pad with zeros out to the length of the number of samples
    // Scale by 100 to compensate for the scaling that occurs in the FFT function
    memset(rangingObj->scratchBufferTwoL2_32kB, 0, rangingObj->DPParams.numAdcSamples*sizeof(cmplx16ImRe_t));
    for(index = 0; index < sampled_gold_code.length; index++)
    {
        rangingObj->scratchBufferTwoL2_32kB[index].real = sampled_gold_code.data[index]*100;

        // Compute the index of the first and last non-zero indices
        // These will be used for signal and noise calculations later
        if(first_non_zero_index == -1 && sampled_gold_code.data[index] > 0)
        {
            first_non_zero_index = index;
        }
        if(last_non_zero_index == -1 && sampled_gold_code.data[sampled_gold_code.length - index - 1] > 0)
        {
            last_non_zero_index = sampled_gold_code.length - index - 1;
        }
    }
    rangingObj->firstGoldCodeNonZeroIndex = first_non_zero_index;
    rangingObj->lastGoldCodeNonZeroIndex = last_non_zero_index;
    //memcpy(rangingObj->magnitudeData, rangingObj->scratchBuffer, rangingObj->DPParams.numAdcSamples * sizeof(cmplx16ImRe_t));

    /////////////////////////////////////////////////////////
    // FFT of the gold code
    DSP_fft16x16_imre(
            (int16_t *) rangingObj->fftTwiddle16x16L2_16kB,
            rangingObj->DPParams.numAdcSamples,
            (int16_t *) rangingObj->scratchBufferTwoL2_32kB,  // Source Address
            (int16_t *) rangingObj->fftGoldCodeL2_16kB );  // Destination Address
    //memcpy(rangingObj->fftOfMagnitude, rangingObj->fftGoldCode, rangingObj->DPParams.numAdcSamples * sizeof(cmplx16ImRe_t));

    /////////////////////////////////////////////////////////
    // Complex Conjugate
    for(index = 0; index < rangingObj->DPParams.numAdcSamples; index++)
    {
        rangingObj->fftGoldCodeL2_16kB[index].imag = -1*rangingObj->fftGoldCodeL2_16kB[index].imag;
    }
    memcpy(rangingObj->fftGoldCodeL3_16kB, rangingObj->fftGoldCodeL2_16kB, rangingObj->DPParams.numAdcSamples * sizeof(cmplx16ImRe_t) );

    /////////////////////////////////////////////////////////
    // Cleanup temporary memory used for gold code generation
    if(gold_code.data != NULL)
    {
        free(gold_code.data);
    }
    if(sampled_gold_code.data != NULL)
    {
        free(sampled_gold_code.data);
    }

    ////////////////////////////////////////////////////////////////
    // Configure EDMA
    retVal = rangingDSP_ConfigDataInEDMA(rangingObj, &pConfig->hwRes);
    if(retVal < 0)
    {
        goto exit;
    }
    retVal = rangingDSP_ConfigDataOutEDMA(rangingObj, &pConfig->hwRes);
    if(retVal < 0)
    {
        goto exit;
    }

    rangingObj->chirpCount = 0;
    rangingObj->inProgress = false;
exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      In place calculation of magnitude of a complex array.
 *
 *  @param[in]  numSamples          size of array
 *  @param[in]  input               array of cmplx16ImRe_t
 *
 *  @retval
 *      None
 */
void calcMag16ImRe
(
    uint16_t        numSamples,
    cmplx16ImRe_t   *input
)
{
    uint16_t index;

    // Using floats to prevent overflow
    float temp_i;
    float temp_q;

    for(index = 0; index < numSamples; index++)
    {
        // Square
        // I sample
        temp_i = (float)( (int16_t *) input)[2*index] * (float)( (int16_t *) input)[2*index];
        //imagSrcAddr[2*index] = (float)imagSrcAddr[2*index] * (float)imagSrcAddr[2*index];

        // Q sample
        temp_q = (float)( (int16_t *) input)[2*index + 1] * (float)( (int16_t *) input)[2*index + 1];
        //imagSrcAddr[2*index+1] = imagSrcAddr[2*index+1] * imagSrcAddr[2*index+1];

        // Sum of I^2 + Q^2
        temp_q = temp_i + temp_q;
        //imagSrcAddr[2*index+1] = imagSrcAddr[2*index] * imagSrcAddr[2*index+1];

        // Sqrt
        ( (int16_t *) input)[2*index + 1] = (uint16_t)sqrt(temp_q);

        // Zero out the imaginary channel
        ( (int16_t *) input)[2*index] = 0;
    }
}


/**
 *  @b Description
 *  @n
 *      In place calculation of magnitude of a complex array.
 *
 *  @param[in]  numSamples          size of array
 *  @param[in]  input               array of cmplx32ImRe_t
 *
 *  @retval
 *      None
 */
void calcMag32ImRe
(
    uint16_t        numSamples,
    cmplx32ImRe_t   *input
)
{
    uint16_t index;

    // Using floats to prevent overflow
    int64_t temp_i;
    int64_t temp_q;

    for(index = 0; index < numSamples; index++)
    {
        // Square
        // I sample
        temp_i = ((int64_t)( input[index].real)) * ((int64_t)( input[index].real));

        // Q sample
        temp_q = ((int64_t)( input[index].imag)) * ((int64_t)( input[index].imag));

        // sqrt(sum of I^2 + Q^2)
        input[index].real = (int32_t)sqrt(temp_i + temp_q);

        // Zero out the imaginary channel
        input[index].imag = 0;
    }
}

void calcMagInt16
(
    uint16_t numSamples,
    int16_t     *input

)
{
    uint16_t index;

    // Using floats to prevent overflow
    float temp_i;
    float temp_q;

    for(index = 0; index < numSamples; index++)
    {
        // Square
        // I sample
        temp_i = (float)input[2*index] * (float)input[2*index];
        //imagSrcAddr[2*index] = (float)imagSrcAddr[2*index] * (float)imagSrcAddr[2*index];

        // Q sample
        temp_q = (float)input[2*index + 1] * (float)input[2*index + 1];
        //imagSrcAddr[2*index+1] = imagSrcAddr[2*index+1] * imagSrcAddr[2*index+1];

        // Sum of I^2 + Q^2
        temp_q = temp_i + temp_q;
        //imagSrcAddr[2*index+1] = imagSrcAddr[2*index] * imagSrcAddr[2*index+1];

        // Sqrt
        input[2*index + 1] = (uint16_t)sqrt(temp_q);

        // Zero out the imaginary channel
        input[2*index] = 0;
    }
}

// The CODE_SECTION pragma places the code into the fastest memory cache, defined in mmw_dss_linker.cmd
// The FUNCTION_OPTIONS "--opt_for_speed" means that the compiler places all of the functions
// that are called from DPU_RangingDSP_process in that section as well
#pragma FUNCTION_OPTIONS(DPU_RangingDSP_process, "--opt_for_speed")
#pragma CODE_SECTION(DPU_RangingDSP_process, ".l1pcode")
/**
 *  @b Description
 *  @n
 *      The function is ranging DPU process function. It executes FFT operation.
 *  It can be called multiple times in a frame until all chirps are handled in the frame.
 *
 *  @pre    DPU_rangingDSP_init() has been called
 *
 *  @param[in]  handle                  ranging DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_ranging_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangingDSP_process
(
    DPU_RangingDSP_Handle     handle,
    DPU_RangingDSP_OutParams     *outParams
)
{
    ranging_dpParams  *DPParams;
    rangingDSPObj     *rangingObj;
    EDMA_Handle         edmaHandle;
    volatile uint32_t   startTime;
    volatile uint32_t   startTime1;
    volatile uint32_t   stopTime;
    uint32_t            waitingTime;
    uint32_t            outChannel;
    int32_t             retVal = 0;
    uint16_t            index;
    Ranging_PRN_Detection_Stats * detectionStats = &outParams->stats.detectionStats;

    rangingObj = (rangingDSPObj *)handle;
    if(rangingObj == NULL)
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }

    if(rangingObj->inProgress == true)
    {
        retVal = DPU_RANGINGDSP_EINPROGRESS;
        goto exit;
    }
    else
    {
        rangingObj->inProgress = true;
    }

    DPParams = &rangingObj->DPParams;

    detectionStats->wasCodeDetected = 0;

    // If we are transmitting, the number of chirps per frame is > 1
    // We only need to process data if we are receiving.
    if(DPParams->numChirpsPerFrame > 1)
    {
        detectionStats->isTxComplete = 1;
    }
    else
    {
        uint32_t            dataInAddr;
        int16_t             *L1Buffer16kB;
        //uint16_t            index_of_max            = 0;
        //uint32_t            max_value               = 0;
        uint16_t            num_line_fit_points     = 11;
        cmplx16ImRe_t *     scratchPageTwo_L2_16kB  = rangingObj->scratchBufferTwoL2_32kB + DPParams->numAdcSamples;

        // fftTwiddle16x16L2_16kB            - 16kB - scratchBufferOneL2_32kB first 16kB
        // localGoldCodeFFTBufferL2_16kB     - 16kB - scratchBufferOneL2_32kB last  16kB
        // ifftTwiddle16x16L2_16kB           - 16kB
        // scratchBufferL2_32kB              - 32kB
        // scratchBufferOneL2_32kB is 32kB contiguous RAM from fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB
        cmplx32ImRe_t *     ifftBuffer              = (cmplx32ImRe_t *) rangingObj->scratchBufferOneL2_32kB;
        float         *     ifftMagnitudeBuffer     = (float *)      rangingObj->scratchBufferTwoL2_32kB;
        cmplx32ImRe_t *     vectorMultiplyBuffer    = (cmplx32ImRe_t *) rangingObj->scratchBufferTwoL2_32kB;
        float temp_i;
        float temp_q;
        int32_t int_index_of_max;
        float32_t maxpow = 0;

        edmaHandle = rangingObj->edmaHandle;
        waitingTime = 0;

        outParams->endOfChirp = false;

        detectionStats->rxPrn = rangingObj->rxPrn;
        detectionStats->isTxComplete = 0;

        dataInAddr = (uint32_t)&rangingObj->ADCdataBuf[0];

        // Set EDMA input source Address
        retVal = EDMA_setSourceAddress(
                edmaHandle,
                rangingObj->dataInChan[0],
            (uint32_t) SOC_translateAddress(dataInAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
        if (retVal != 0)
        {
            goto exit;
        }

        // Transfer all of the samples for this chirp
        // It sends them into &rangingObj->adcDataIn[0]
        EDMA_startDmaTransfer(edmaHandle, rangingObj->dataInChan[0]);

        // Verify if DMA has completed for current antenna
        rangingDSP_WaitEDMAComplete (  edmaHandle, rangingObj->dataInChan[0]);
        waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);

        ////////////////////////////////////
        // Data Processing
        ////////////////////////////////////

        startTime = Cycleprofiler_getTimeStamp();

        // Get the ADC data
        L1Buffer16kB = (int16_t *)&rangingObj->adcDataInL1_16kB[0];
        memcpy(rangingObj->radarCubebuf, &rangingObj->adcDataInL1_16kB[0], DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 1. Magnitude Calculation
        //      Magnitude is stored in real channel (odd indices)
        //      Calculation performed in place
        //      Very costly (1700us), can be optimized - look at mmwavelib_log2Abs16 for inspiration
        //      Also see mmwavelib_power
        ////////////////////////////////////
        startTime1 = Cycleprofiler_getTimeStamp();
        calcMagInt16(DPParams->numAdcSamples, L1Buffer16kB);
        stopTime = Cycleprofiler_getTimeStamp();
        outParams->stats.magAdcTime = stopTime - startTime1;
        memcpy(rangingObj->magnitudeDataL3, L1Buffer16kB, DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 2. FFT
        //      Convert to frequency domain
        //      Inefficient - rakes 45 us - only need half as many computations once optimized
        ////////////////////////////////////

        ////////////////////////////////////
        // 16bit FFT in imre format
        // Write to the second 16kB of the scratchbuffer so we can use it for the magnitude calculation
        startTime1 = Cycleprofiler_getTimeStamp();
        DSP_fft16x16_imre(
                (int16_t *) rangingObj->fftTwiddle16x16L2_16kB,     // Twiddle factors
                DPParams->numAdcSamples,                            // number of complex samples
                (int16_t *) L1Buffer16kB,                           // Input
                (int16_t *) scratchPageTwo_L2_16kB );               // Output
        stopTime = Cycleprofiler_getTimeStamp();
        outParams->stats.fftTime = stopTime - startTime1;
        memcpy(rangingObj->fftOfMagnitudeL3, scratchPageTwo_L2_16kB, DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 3. Vector Multiply
        //      Multiplication in the frequency domain is convolution in the time domain, but much more efficient
        //      Inefficient - takes 200us - look at mmwavelib_vecmul16x16 for inspiration (should take ~10us)
        ////////////////////////////////////
        int32_t temp_one;
        int32_t temp_two;
        int32_t temp_three;
        startTime1 = Cycleprofiler_getTimeStamp();
        for(index = 0; index < DPParams->numAdcSamples; index++)
        {
            // Real portion
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].real);
            temp_three = (temp_one * temp_two);
            vectorMultiplyBuffer[index].real = temp_three;

            // Real times imaginary
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].imag);
            temp_three = (temp_one * temp_two);
            vectorMultiplyBuffer[index].imag = temp_three;

            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].real);
            temp_three = (temp_one * temp_two);
            vectorMultiplyBuffer[index].imag += temp_three;

            // Imaginary portion
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].imag);
            temp_three = (temp_one * temp_two);
            vectorMultiplyBuffer[index].real -= temp_three;
        }
        stopTime = Cycleprofiler_getTimeStamp();
        outParams->stats.vecmulTime = stopTime - startTime1;
        memcpy(rangingObj->vectorMultiplyOfFFtedDataL3, vectorMultiplyBuffer, DPParams->numAdcSamples * sizeof(cmplx32ImRe_t));

        ///////////////////////////////////////////////////////////////////
        // 4. IFFT
        //      Convert result back to time domain for interpretation
        //      DSP_ifft16x32 assumes input data in format ReIm - flipped from what we have
        //      This has no effect on magnitude which is the next step
        //      This overwrites the Complex Conjugate(FFT(Gold Code)) and FFT twiddle factors
        //      they are: fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB
        //      They need to be copied back from L3 later
        //      Inefficient - takes 241 us - only need half as many computations once optimized
        //      This array needs to be reversed
        ///////////////////////////////////////////////////////////////////
        startTime1 = Cycleprofiler_getTimeStamp();
        DSP_ifft16x32(
                (int16_t *) rangingObj->ifftTwiddle16x32L2_16kB,    // Twiddle factors (int16_t)
                DPParams->numAdcSamples,                            // number of complex samples
                (int32_t *) vectorMultiplyBuffer,                   // Input  (int32_t), scratch buffer two
                (int32_t *) ifftBuffer );                           // Output (int32_t), scratch buffer one
        stopTime = Cycleprofiler_getTimeStamp();
        outParams->stats.ifftTime = stopTime - startTime1;
        memcpy(rangingObj->iFftDataL3, ifftBuffer, DPParams->numAdcSamples * sizeof(cmplx32ImRe_t) );

        // Reverse the ifft Buffer
        for(index = 0; index < DPParams->numAdcSamples; index++)
        {
            vectorMultiplyBuffer[index] = ifftBuffer[DPParams->numAdcSamples - 1 - index];
        }


        /////////////////////////////////////////////////////////////////////
        // 5.  Calculate magnitude and find the max
        //      The maximum value of the correlation peak shows where the code started
        //      The IFFT buffer is reversed, so we reverse it here
        /////////////////////////////////////////////////////////////////////
        startTime1 = Cycleprofiler_getTimeStamp();
        int_index_of_max = mmwavelib_powerAndMax((int32_t *) vectorMultiplyBuffer,
                                                 DPParams->numAdcSamples,           // number of complex samples
                                                 ifftMagnitudeBuffer,
                                                 &maxpow);
        stopTime = Cycleprofiler_getTimeStamp();
        outParams->stats.magIfftTime = stopTime - startTime1;
        memcpy(rangingObj->magIfftDataL3, ifftMagnitudeBuffer, DPParams->numAdcSamples * sizeof(float) );
        detectionStats->promptValue = maxpow;
        detectionStats->promptIndex = (uint32_t) int_index_of_max;

        //////////////////////////////////////////////////////////////////////
        // 6. Threshold check
        //      Determine if the peak corresponds to an actual code match by using a couple of threshold checks
        /////////////////////////////////////////////////////////////////////

        /*

        // Find the signal power and noise floor
        // There are chip_duration * sampling_rate samples per chip
        float   chip_duration           = 0.000006;
        float   zeros_duration          = 0.000003;
        float   sample_rate             = 4e6;
        uint16_t num_signal_samples     = floor(chip_duration*sample_rate) - 1;
        uint16_t num_zeros_samples      = floor(zeros_duration*sample_rate) - 1;
        uint32_t    avg_signal_power    = 0;
        uint32_t    avg_noise_power     = 0;
        uint16_t power_index;
        // COMPUTE SNR
        if( int_index_of_max < DPParams->numAdcSamples/2 )
        {
            for(index = 0; index < num_signal_samples; index++)
            {
                // L1Buffer16kB holds the magnitude of the ADC data.
                // Real channel (odd indices) has a value, imaginary channel (even indices) has zeros.
                // The int_index_of_max is the rising edge of the first bit
                power_index = 2*int_index_of_max;  // index_of_max is in a float array of magnitudes.
                                                   // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                power_index += 3;                  // Add three to ensure we are at the top of the signal chip, not on the rising edge
                power_index += 2*index;            // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                power_index += 2*rangingObj->firstGoldCodeNonZeroIndex;  // Just in case the first non zero bit is not at the start
                avg_signal_power += L1Buffer16kB[power_index];
            }
            avg_signal_power /= num_signal_samples;

            if(int_index_of_max - 3 - 2*num_zeros_samples >= 0)
            {
                for(index = 0; index < num_zeros_samples; index++)
                {
                    // L1Buffer16kB holds the magnitude of the ADC data.
                    // Real channel (odd indices) has a value, imaginary channel (even indices) has zeros.
                    power_index = 2*int_index_of_max;  // index_of_max is in a float array of magnitudes.
                                                       // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                    power_index -= 3;                  // Subtract three to ensure we are off the signal chip, not on the rising edge
                    power_index -= 2*index;            // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                    power_index += 2*rangingObj->firstGoldCodeNonZeroIndex;  // Just in case the first non zero bit is not at the start
                    avg_noise_power += L1Buffer16kB[power_index];
                }
                avg_noise_power /= num_zeros_samples;
            }
            else
            {
                for(index = 0; index < num_zeros_samples; index++)
                {
                    // L1Buffer16kB holds the magnitude of the ADC data.
                    // Real channel (odd indices) has a value, imaginary channel (even indices) has zeros.
                    power_index = 2*int_index_of_max;       // index_of_max is in a float array of magnitudes.
                                                            // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                    power_index += 2*num_signal_samples;    // Move to the other side of the chip
                    power_index += 7;                       // Add seven to ensure we are off the signal chip, not on the rising edge
                    power_index += 2*index;                 // Multiply it by two to adjust it to a cmplx16ImRe_t array.
                    power_index += 2*rangingObj->firstGoldCodeNonZeroIndex;  // Just in case the first non zero bit is not at the start
                    avg_noise_power += L1Buffer16kB[2*int_index_of_max + 7 + 2*num_signal_samples + 2*index];
                }
                avg_noise_power /= num_zeros_samples;
            }
        }
        else
        {
            // Perform a similar calculation using rangingObj->lastGoldCodeNonZeroIndex
        }
        */

        detectionStats->eplOffset  = 7;
        detectionStats->earlyValue = ifftMagnitudeBuffer[int_index_of_max - detectionStats->eplOffset]; // left side of the correlation peak, over halfway down
        detectionStats->lateValue  = ifftMagnitudeBuffer[int_index_of_max + detectionStats->eplOffset]; // right side of the correlation peak, over halfway down

        // COMPUTE THRESHOLD
        if(detectionStats->promptValue > DETECTION_THRESHOLD)
        {
            if(     detectionStats->earlyValue > detectionStats->lateValue - detectionStats->lateValue/5 &&
                    detectionStats->earlyValue < detectionStats->lateValue + detectionStats->lateValue/5)
            {
                if(     maxpow > 1.25*detectionStats->earlyValue &&
                        maxpow > 1.25*detectionStats->lateValue )
                {
                    // Potential peak detected

                    //////////////////////////////////////////
                    // 6a. Coarse peak time
                    //      Offset with respect to the first ADC sample in nanoseconds
                    //////////////////////////////////////////
                    float f_index_of_max = (float)(int_index_of_max);                       // range of zero to 4095
                    float f_adcSampleRate = (float)rangingObj->DPParams.adcSampleRate;  // 4000000
                    float f_secondsOffset = f_index_of_max/f_adcSampleRate;             // range of zero to 0.00102375 in steps of 1/adcSampleRate

                    // Convert to DSP CPU cycles (600000000 cycles per second (600 MHz))
                    detectionStats->coarsePeakTimeOffsetCycles = ((uint32_t)(f_secondsOffset*DSP_CLOCK_MHZ*1e6)); // range of zero to 1023750

                    //////////////////////////////////////////
                    // 6b. Fine peak detection
                    //      Offset with respect to the coarse peak time
                    //////////////////////////////////////////

                    detectionStats->leftSlope      = ( sqrt(ifftMagnitudeBuffer[int_index_of_max - detectionStats->eplOffset + 2]) -
                                                        sqrt(ifftMagnitudeBuffer[int_index_of_max - detectionStats->eplOffset - 2]) ) / (11.0f);

                    detectionStats->leftIntercept  = ( sqrt(ifftMagnitudeBuffer[int_index_of_max])) - detectionStats->leftSlope*f_index_of_max;


                    detectionStats->rightSlope     = ( sqrt(ifftMagnitudeBuffer[int_index_of_max + detectionStats->eplOffset + 2]) -
                                                        sqrt(ifftMagnitudeBuffer[int_index_of_max + detectionStats->eplOffset - 2]) ) / (11.0f);

                    detectionStats->rightIntercept = ( sqrt(ifftMagnitudeBuffer[int_index_of_max])) - detectionStats->rightSlope*f_index_of_max;

                    /*
                    temp_one = outParams->stats.eplOffset + num_line_fit_points/2;
                    temp_two = outParams->stats.eplOffset - num_line_fit_points/2;
                    for(index = 0; index < num_line_fit_points; index++)
                    {
                        ((uint32_t *)rangingObj->scratchBufferTwoL2_32kB)[index] = (uint32_t)(index_of_max - temp_one + index);
                        ((uint32_t *)rangingObj->scratchBufferTwoL2_32kB)[num_line_fit_points + index] = (uint32_t)(index_of_max + temp_two + index);
                    }

                    line_fit(
                            num_line_fit_points,
                            ((uint32_t *)rangingObj->scratchBufferTwoL2_32kB),
                            &bufferTwo[index_of_max - temp_one],
                            &left_slope,
                            &left_intercept);

                    line_fit(
                            num_line_fit_points,
                            ((uint32_t *)rangingObj->scratchBufferTwoL2_32kB) + num_line_fit_points,
                            &bufferTwo[index_of_max + temp_two],
                            &right_slope,
                            &right_intercept);
                    */

                    if( detectionStats->leftSlope > -1*detectionStats->rightSlope - detectionStats->leftSlope/5 &&
                            detectionStats->leftSlope < -1*detectionStats->rightSlope + detectionStats->leftSlope/5 )
                    {
                        detectionStats->wasCodeDetected = 1;
                        float peak_index_fine = ((float) DPParams->numAdcSamples) - \
                                (detectionStats->rightIntercept - detectionStats->leftIntercept)/(detectionStats->leftSlope - detectionStats->rightSlope);
                        float f_secondsOffsetFine = peak_index_fine/f_adcSampleRate;
                        detectionStats->RefinedPeakTimePicoseconds = (int32_t)((f_secondsOffsetFine - f_secondsOffset)*1e12);
                    }
                }
            }
        }

        stopTime = Cycleprofiler_getTimeStamp();

        // Reset the gold code and FFT twiddle - in the future this could be triggered by a framestart interrupt
        memcpy(rangingObj->fftGoldCodeL2_16kB,
               rangingObj->fftGoldCodeL3_16kB,
               DPParams->numAdcSamples * sizeof(cmplx16ImRe_t) );

        memcpy(rangingObj->fftTwiddle16x16L2_16kB,
               rangingObj->fftTwiddle16x16L3_16kB,
               DPParams->numAdcSamples * sizeof(cmplx16ImRe_t) );


        ///////////////////////////////////////
        // Data Output
        ///////////////////////////////////////
        outChannel = rangingObj->dataOutChan[0];

        uint32_t    radarCubeAddr;

        radarCubeAddr = (uint32_t)(rangingObj->radarCubebuf);
        EDMA_setDestinationAddress(edmaHandle, outChannel,
            (uint32_t)SOC_translateAddress((radarCubeAddr), SOC_TranslateAddr_Dir_TO_EDMA, NULL));

        if(rangingObj->chirpCount > 1U)
        {
            startTime1 = Cycleprofiler_getTimeStamp();
            rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
            waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);
        }

        //EDMA_startDmaTransfer(edmaHandle, outChannel);

        // Increment chirp count
        rangingObj->chirpCount++;

        // Last chirp , wait until EDMA is completed
        if(rangingObj->chirpCount == DPParams->numChirpsPerFrame)
        {
            // Wait until last transfer is done
            //rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
            rangingObj->chirpCount = 0;
            outParams->endOfChirp = true;
        }

        rangingObj->numProcess++;
    }

    /* Update outParams */
    outParams->stats.processingTime = stopTime - startTime;
    outParams->stats.waitTime = waitingTime;

    rangingObj->inProgress = false;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is ranging DPU control function.
 *
 *  @pre    DPU_rangingDSP_init() has been called
 *
 *  @param[in]  handle           ranging DPU handle
 *  @param[in]  cmd              ranging DPU control command
 *  @param[in]  arg              ranging DPU control argument pointer
 *  @param[in]  argSize          ranging DPU control argument size
 *
 *  \ingroup    DPU_ranging_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangingDSP_control
(
    DPU_RangingDSP_Handle     handle,
    DPU_RangingDSP_Cmd        cmd,
    void*                       arg,
    uint32_t                    argSize
)
{
    int32_t             retVal = 0;
    rangingDSPObj     *rangingObj;

    /* Get ranging data object */
    rangingObj = (rangingDSPObj *)handle;

    /* Sanity check */
    if (rangingObj == NULL)
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }

    /* Check if control() is called during processing time */
    if(rangingObj->inProgress == true)
    {
        retVal = DPU_RANGINGDSP_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        case DPU_RangingDSP_Cmd_dcRangeCfg:
            retVal = DPU_RANGINGDSP_ECMD;
            break;

        default:
            retVal = DPU_RANGINGDSP_ECMD;
            break;
    }
exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is ranging DPU deinitl function. It frees up the
 *   resources allocated during init.
 *
 *  @pre    DPU_rangingDSP_init() has been called
 *
 *  @param[in]  handle           ranging DPU handle
 *
 *  \ingroup    DPU_ranging_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangingDSP_deinit(DPU_RangingDSP_Handle handle)
{
    rangingDSPObj     *rangingObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    rangingObj = (rangingDSPObj *)handle;
    if(rangingObj == NULL)
    {
        retVal = DPU_RANGINGDSP_EINVAL;
        goto exit;
    }
    else
    {
        /* Free memory */
        MemoryP_ctrlFree(handle, sizeof(rangingDSPObj));
    }
exit:
    return (retVal);
}
