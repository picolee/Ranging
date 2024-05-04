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

/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>

#define  DEBUG_CHECK_PARAMS 1

/* Macros to determine pingpong index */
#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x) == 1U)
#define BYTES_PER_SAMP_1D   sizeof(cmplx16ImRe_t)

/******************************************************************
 *                      Internal Function prototype
 ******************************************************************/
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

    /* Copy data from ADCbuffer to internal adcbufIn scratch buffer 
      Data is in non-interleaved mode:
      All of the samples from one antenna are sequential in a block
      Then the next antenna has its block, etc

      The data is IMRE - two imaginary bytes first, then 2 real bytes

      We want to compute the magnitude:
      we will copy the imaginary bytes to one vector, square each element
      copy real bytes to another vector, square each element
      add the two vectors
      square root them

      We need to do an AB Sync EDMA transfer.
    samplesPerChirp = dpParams->numAdcSamples * dpParams->numRxAntennas;

    // Ping/Pong common configuration
    syncABCfg.aCount = 2;                           // real or imaginary 16 bit value
    syncABCfg.bCount = dpParams->numAdcSamples;
    syncABCfg.cCount = dpParams->numRxAntennas;     // For our use case, should be 1
    syncABCfg.srcBIdx = sizeof(cmplx16ImRe_t);      // Size of the data we're interested in and the two bytes we need to skip
    syncABCfg.srcCIdx = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);
    syncABCfg.dstBIdx = 2;                          // We're making the imaginary bytes contiguous and the real bytes contiguous
    syncABCfg.dstCIdx = 0;                          // Don't care

    // PING configuration - writing imaginary to L1
    syncABCfg.srcAddress = (uint32_t)rangingObj->ADCdataBuf; // First imaginary sample
    syncABCfg.destAddress = (uint32_t)rangingObj->adcDataIn;

    retVal = DPEDMA_configSyncAB (
                            hwRes->edmaCfg.edmaHandle,
                            &hwRes->edmaCfg.dataInPing,
                            NULL,  // no Chaining
                            &syncABCfg,
                            false,
                            true,
                            true,
                            NULL,
                            NULL
                            );
    if (retVal < 0)
    {
        goto exit;
    }

    // PONG configuration - writing real to L1
    syncABCfg.srcAddress = (uint32_t)(rangingObj->ADCdataBuf + 2); // First real sample
    syncABCfg.destAddress = (uint32_t)rangingObj->adcDataIn + dpParams->numAdcSamples*2;

    retVal = DPEDMA_configSyncAB (
                            hwRes->edmaCfg.edmaHandle,
                            &hwRes->edmaCfg.dataInPong,
                            NULL,  // no Chaining
                            &syncABCfg,
                            false,
                            true,
                            true,
                            NULL,
                            NULL
                            );
    if (retVal < 0)
    {
        goto exit;
    }
    */

    /*
     * Here is the old code which copies an antennas worth
     * of continguous cmplx16ImRe_t samples into the data in buffer,
     * rather than copying imaginary to a contiguous block and real to a
     * contiguous block

    // aCount: One antenna's worth
    syncACfg.aCount = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);

    // bCount: num antennas divided by 2 - half in ping, half in pong
    syncACfg.bCount = MAX(dpParams->numRxAntennas / 2U, 1U) * dpParams->numChirpsPerChirpEvent;

    // B index is 2x channel offset, so we get antenna zero and antenna 2 in ping, and antenna 1 and 3 in pong (for 4 antennas)
    syncACfg.srcBIdx = rangingObj->rxChanOffset * 2U ;

    // dstBidx is zero: the destination array is not changed between consecutive Sync A's
    syncACfg.dstBIdx = 0U;

    // PING configuration
    syncACfg.srcAddress = (uint32_t)rangingObj->ADCdataBuf;
    syncACfg.destAddress = (uint32_t)rangingObj->adcDataIn;
    
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

    // PONG src/dest address
    syncACfg.srcAddress = (uint32_t)rangingObj->ADCdataBuf + rangingObj->rxChanOffset;
    syncACfg.destAddress = (uint32_t)&rangingObj->adcDataIn[dpParams->numAdcSamples];

    retVal = DPEDMA_configSyncA_singleFrame_nonlib(
                hwRes->edmaCfg.edmaHandle,
                &hwRes->edmaCfg.dataInPong,
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
    */
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
//    uint16_t    samplesPerChirp;
//    uint16_t     oneD_destinationCindex;
//    uint8_t     *oneD_destinationPongAddress;
//    DPEDMA_syncABCfg         syncABCfg;
    DPEDMA_syncACfg         syncACfg;
    ranging_dpParams      *dpParams;
//    DPU_RangingDSP_EDMAConfig *edmaCfg;

//    edmaCfg = &hwRes->edmaCfg;
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
    /*

    oneD_destinationCindex = samplesPerChirp * sizeof(cmplx16ImRe_t);;
    oneD_destinationPongAddress = (uint8_t *)(&rangingObj->radarCubebuf[samplesPerChirp]);

    // Ping/Pong common configuration
    syncABCfg.aCount = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);
    syncABCfg.bCount = dpParams->numRxAntennas;
    syncABCfg.cCount = dpParams->numChirpsPerFrame / 2U; //bCount
    syncABCfg.srcBIdx = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);
    syncABCfg.srcCIdx = 0;
    syncABCfg.dstBIdx = dpParams->numAdcSamples * sizeof(cmplx16ImRe_t);
    syncABCfg.dstCIdx = oneD_destinationCindex;

    // scratchBuffer is in L2. radarCubebuf is in L3
    syncABCfg.srcAddress = (uint32_t)rangingObj->scratchBuffer;
    syncABCfg.destAddress= (uint32_t)rangingObj->radarCubebuf;

    // Ping - Copies from ping FFT output (even chirp indices)  to L3
    retVal = DPEDMA_configSyncAB (edmaCfg->edmaHandle,
                                 &edmaCfg->dataOutPing,
                                 NULL,
                                 &syncABCfg,
                                 false,
                                 true,
                                 true,
                                 NULL,
                                 NULL
                                 );
    if (retVal < 0)
    {
        goto exit;
    }

    // Pong - copies from pong FFT output (odd chirp indices)  to L3
    syncABCfg.srcAddress = (uint32_t)&rangingObj->scratchBuffer[samplesPerChirp];
    syncABCfg.destAddress= (uint32_t)oneD_destinationPongAddress;

    retVal = DPEDMA_configSyncAB (edmaCfg->edmaHandle,
                                 &edmaCfg->dataOutPong,
                                 NULL,
                                 &syncABCfg,
                                 false,
                                 true,
                                 true,
                                 NULL,
                                 NULL
                                 );

exit:
    return(retVal);
    */
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
    rangingObj->magIfftDataL3               = (cmplx32ImRe_t *)(((char *)rangingObj->iFftDataL3)            + 2 * pStaticCfg->ADCBufData.dataSize); // Each value 4 bytes
    rangingObj->fftGoldCodeL3_16kB          = (cmplx16ImRe_t *)(((char *)rangingObj->magIfftDataL3)         + 2 * pStaticCfg->ADCBufData.dataSize); // Each value 2 bytes
    rangingObj->fftTwiddle16x16L3_16kB      = (cmplx16ImRe_t *)(((char *)rangingObj->fftGoldCodeL3_16kB)    + pStaticCfg->ADCBufData.dataSize);     // Each value 2 bytes

    /* Save Scratch buffers */
    rangingObj->scratchBufferOneL2_32kB     = pHwRes->fftTwiddle16x16L2_16kB;
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
    double                      sample_rate      = 4000000;
    double                      chip_duration    = 0.000006;
    double                      zeros_duration   = 0.000003;
    int16_t                     index;

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

    // Generate twiddle factors for 1D FFT. This is one time
    mmwavelib_gen_twiddle_fft16x16_imre_sa((short *)rangingObj->fftTwiddle16x16L2_16kB, rangingObj->DPParams.numAdcSamples);
    memcpy(rangingObj->fftTwiddle16x16L3_16kB, rangingObj->fftTwiddle16x16L2_16kB, rangingObj->DPParams.numAdcSamples * sizeof(cmplx16ImRe_t) );

    // Generate twiddle factors for the IFFT. This is one time
    gen_twiddle_fft16x32((short *)rangingObj->ifftTwiddle16x32L2_16kB, rangingObj->DPParams.numAdcSamples);

    ////////////////////////////////////////////////////////////////////////////////
    // Gold code, length 2^N - 1
    rangingObj->gold_code_n = 6;
    rangingObj->gold_code_prn = 3;
    gold_code.data = NULL;
    if (generate_one_gold_sequence(
        rangingObj->gold_code_n,
        &gold_code,
        rangingObj->gold_code_prn))
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
    }
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
    uint32_t            waitingTime;
    uint32_t            outChannel;
    int32_t             retVal = 0;
    uint16_t            index;

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

    // If we are transmitting, the number of chirps per frame is > 1
    // We only need to process data if we are receiving.
    if(DPParams->numChirpsPerFrame > 1)
    {

    }
    else
    {
        uint32_t            dataInAddr;
        int16_t             *L1Buffer16kB;
        int16_t             index_of_max    = -1;
        uint32_t            max_value       = 0;
        cmplx16ImRe_t *     scratchPageTwo_L2_16kB = rangingObj->scratchBufferTwoL2_32kB + DPParams->numAdcSamples;

        edmaHandle = rangingObj->edmaHandle;

        startTime = Cycleprofiler_getTimeStamp();
        waitingTime = 0;

        outParams->endOfChirp = false;

        outParams->stats.PRN = rangingObj->gold_code_prn;
        outParams->stats.wasCodeDetected = 0;

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
        startTime1 = Cycleprofiler_getTimeStamp();
        rangingDSP_WaitEDMAComplete (  edmaHandle, rangingObj->dataInChan[0]);
        waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);

        ////////////////////////////////////
        // Data Processing
        ////////////////////////////////////

        // Get the ADC data
        L1Buffer16kB = (int16_t *)&rangingObj->adcDataInL1_16kB[0];
        memcpy(rangingObj->radarCubebuf, &rangingObj->adcDataInL1_16kB[0], DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 1. Magnitude Calculation
        //      Magnitude is stored in real channel (odd indices)
        //      Calculation performed in place
        ////////////////////////////////////
        calcMagInt16(DPParams->numAdcSamples, L1Buffer16kB);
        memcpy(rangingObj->magnitudeDataL3, L1Buffer16kB, DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 2. FFT
        //      Convert to frequency domain
        ////////////////////////////////////

        ////////////////////////////////////
        // 16bit FFT in imre format
        // Write to the second 16kB of the scratchbuffer so we can use it for the magnitude calculation
        DSP_fft16x16_imre(
                (int16_t *) rangingObj->fftTwiddle16x16L2_16kB,     // Twiddle factors
                DPParams->numAdcSamples,                            // number of complex samples
                (int16_t *) L1Buffer16kB,                           // Input
                (int16_t *) scratchPageTwo_L2_16kB );               // Output
        memcpy(rangingObj->fftOfMagnitudeL3, scratchPageTwo_L2_16kB, DPParams->numAdcSamples * sizeof(cmplx16ImRe_t));

        ////////////////////////////////////
        // 3. Vector Multiply
        //      Multiplication in the frequency domain is convolution in the time domain, but much more efficient
        ////////////////////////////////////
        int32_t temp_one;
        int32_t temp_two;
        int32_t temp_three;
        for(index = 0; index < DPParams->numAdcSamples; index++)
        {
            // Real portion
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].real);
            temp_three = (temp_one * temp_two);
            ((cmplx32ImRe_t *)rangingObj->scratchBufferTwoL2_32kB)[index].real = temp_three;

            // Real times imaginary
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].imag);
            temp_three = (temp_one * temp_two);
            ((cmplx32ImRe_t *)rangingObj->scratchBufferTwoL2_32kB)[index].imag = temp_three;

            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].real);
            temp_three = (temp_one * temp_two);
            ((cmplx32ImRe_t *)rangingObj->scratchBufferTwoL2_32kB)[index].imag += temp_three;

            // Imaginary portion
            temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
            temp_two = ((int32_t) rangingObj->fftGoldCodeL2_16kB[index].imag);
            temp_three = (temp_one * temp_two);
            ((cmplx32ImRe_t *)rangingObj->scratchBufferTwoL2_32kB)[index].real -= temp_three;
        }
        memcpy(rangingObj->vectorMultiplyOfFFtedDataL3, rangingObj->scratchBufferTwoL2_32kB, DPParams->numAdcSamples * sizeof(cmplx32ImRe_t));

        ///////////////////////////////////////////////////////////////////
        // 4. IFFT
        //      Convert result back to time domain for interpretation
        //      DSP_ifft16x32 assumes input data in format ReIm - flipped from what we have
        //      This has no effect on magnitude which is the next step
        //      This overwrites the Complex Conjugate(FFT(Gold Code)) and FFT twiddle factors
        //      They need to be copied back from L3 later
        ///////////////////////////////////////////////////////////////////
        DSP_ifft16x32(
                (int16_t *) rangingObj->ifftTwiddle16x32L2_16kB,    // Twiddle factors (int16_t)
                DPParams->numAdcSamples,                            // number of complex samples
                (int32_t *) rangingObj->scratchBufferTwoL2_32kB,    // Input  (int32_t)
                (int32_t *) rangingObj->scratchBufferOneL2_32kB );  // Output (int32_t)
        memcpy(rangingObj->iFftDataL3, rangingObj->scratchBufferOneL2_32kB, DPParams->numAdcSamples * sizeof(cmplx32ImRe_t) );


        /////////////////////////////////////////////////////////////////////
        // 5.  Calculate magnitude and find the max
        //      The maximum value of the correlation peak shows where the code started
        /////////////////////////////////////////////////////////////////////
        float * temp_i = &((float *)rangingObj->scratchBufferTwoL2_32kB)[0];
        float * temp_q = &((float *)rangingObj->scratchBufferTwoL2_32kB)[1];
        index_of_max = -1;
        max_value = 0;

        cmplx32ImRe_t *bufferOne = (cmplx32ImRe_t *)rangingObj->scratchBufferOneL2_32kB;
        for(index = 0; index < DPParams->numAdcSamples; index++)
        {
            /////////////////////////////////////////////////////
            // Calculate magnitude
            /////////////////////////////////////////////////////
            // Square
            // I sample
            *temp_i = ((float)bufferOne[index].real) * ((float)bufferOne[index].real);

            // Q sample
            *temp_q = ((float)bufferOne[index].imag) * ((float)bufferOne[index].imag);

            // sqrt(sum of I^2 + Q^2)
            bufferOne[index].real = (int32_t)sqrt(*temp_i + *temp_q);

            // Zero out the imaginary channel
            bufferOne[index].imag = 0;

            /////////////////////////////////////////////////////
            // Find the max
            /////////////////////////////////////////////////////
            if( max_value < bufferOne[index].real)
            {
                max_value = bufferOne[index].real;
                index_of_max = index;
            }

            if(index == 100)
            {
                max_value = bufferOne[index].real;
            }
        }
        memcpy(rangingObj->magIfftDataL3, rangingObj->scratchBufferOneL2_32kB, DPParams->numAdcSamples * sizeof(cmplx32ImRe_t) );
        outParams->stats.promptValue = max_value;
        outParams->stats.promptIndex = index_of_max;

        //////////////////////////////////////////////////////////////////////
        // 6. Threshold check
        //      Determine if the peak corresponds to an actual code match by using a couple of threshold checks
        /////////////////////////////////////////////////////////////////////
        outParams->stats.eplOffset  = 14;
        outParams->stats.earlyValue = ((cmplx32ImRe_t*)rangingObj->scratchBufferOneL2_32kB)[index_of_max - outParams->stats.eplOffset].real; // left side of the correlation peak, over halfway down
        outParams->stats.lateValue  = ((cmplx32ImRe_t*)rangingObj->scratchBufferOneL2_32kB)[index_of_max + outParams->stats.eplOffset].real; // right side of the correlation peak, over halfway down

        if(     outParams->stats.earlyValue > outParams->stats.lateValue - outParams->stats.lateValue/10 &&
                outParams->stats.earlyValue < outParams->stats.lateValue + outParams->stats.lateValue/10)
        {
            if(     max_value > 2*outParams->stats.earlyValue &&
                    max_value > 2*outParams->stats.lateValue )
            {
                // Peak detected!
                outParams->stats.wasCodeDetected = 1;

                //////////////////////////////////////////
                // 6a. Coarse peak time
                //      Offset with respect to the first ADC sample in nanoseconds
                //////////////////////////////////////////
                float f_index_of_max = (float)index_of_max;                             // range of zero to 4095
                float f_adcSampleRate = (float)rangingObj->DPParams.adcSampleRate;      // 4000000
                float f_secondsOffset = f_index_of_max/f_adcSampleRate;                 // range of zero to 0.00102375 in steps of 1/adcSampleRate

                // Convert to DSP CPU cycles (600000000 cycles per second (600 MHz))
                outParams->stats.coarsePeakTimeOffsetCycles = ((uint32_t)(f_secondsOffset*600000000)); // range of zero to 1023750


                //////////////////////////////////////////
                // 6b. Fine peak detection
                //      Offset with respect to the coarse peak time
                //////////////////////////////////////////
            }
        }

        // Reset the gold code and FFT twiddle - in the future this could be triggered by a framestart interrupt
        memcpy(rangingObj->fftGoldCodeL2_16kB,
               rangingObj->fftGoldCodeL3_16kB,
               DPParams->numAdcSamples * sizeof(cmplx16ImRe_t) );

        memcpy(rangingObj->fftTwiddle16x16L2_16kB,
               rangingObj->fftTwiddle16x16L3_16kB,
               DPParams->numAdcSamples * sizeof(cmplx16ImRe_t) );


        /*********************************
         * Data Output
         *********************************/
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

        /* Increment chirp count */
        rangingObj->chirpCount++;

        /* Last chirp , wait until EDMA is completed */
        if(rangingObj->chirpCount == DPParams->numChirpsPerFrame)
        {
            /* Wait until last transfer is done */
            //rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
            rangingObj->chirpCount = 0;
            outParams->endOfChirp = true;
        }

        rangingObj->numProcess++;
    }

    /* Update outParams */
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitingTime;
    outParams->stats.waitTime = waitingTime;

    rangingObj->inProgress = false;

exit:
    return retVal;

    /* 
    // Process chirp data per loop, loops for numChirpsPerChirpEvent
    // for (chirpIndex = 0; chirpIndex < DPParams->numChirpsPerChirpEvent; chirpIndex++)
    {
        uint32_t    dataInAddr[2];
        uint32_t    numAdcSampleAligned;
        cmplx16ImRe_t *adcBufPongOffset;

        // *********************************
        // * Prepare for the FFT
        // ********************************
        numAdcSampleAligned = (DPParams->numAdcSamples + 3U)/4U * 4U;
        adcBufPongOffset = (cmplx16ImRe_t *)((uint32_t)rangingObj->ADCdataBuf + rangingObj->rxChanOffset);

        dataInAddr[0] = (uint32_t)&rangingObj->ADCdataBuf[chirpIndex * numAdcSampleAligned];
        dataInAddr[1] = (uint32_t)&adcBufPongOffset[chirpIndex * numAdcSampleAligned];

        // Set Ping source Address
        // rangingObj->dataInChan maps to either:
        // DPC_RANGING_DSP_DPU_EDMAIN_PING_CH  for rangingObj->dataInChan[0]
        // DPC_RANGING_DSP_DPU_EDMAIN_PONG_CH  for rangingObj->dataInChan[1]
        // rangingObj->dataInChan is used in this function to tell if we're doing something with PING or PONG
        retVal = EDMA_setSourceAddress(
                edmaHandle,
                rangingObj->dataInChan[0],
            (uint32_t) SOC_translateAddress(dataInAddr[0], SOC_TranslateAddr_Dir_TO_EDMA, NULL));
        if (retVal != 0)
        {
            goto exit;
        }

        // Set Pong source Address
        retVal = EDMA_setSourceAddress(
                edmaHandle,
                rangingObj->dataInChan[1],
            (uint32_t) SOC_translateAddress(dataInAddr[1], SOC_TranslateAddr_Dir_TO_EDMA, NULL));
        if (retVal != 0)
        {
            goto exit;
        }

        // Kick off DMA to fetch data from ADC buffer for first channel
        // This is PING. This is the first A Sync transfer so it starts at the start of the
        // ADC memory buffer - where antenna zero records.
        // It transfers numAdcSamples to the destination buffer
        // Successive A Sync events on the same channel increment the source address by srcBIdx = 2 * numAdcSamples
        // So the first PING transfer is antenna zero from location     ADCBUF + 0                  (set in rangingDSP_ConfigDataInEDMA)
        // The first PONG transfer is antenna one from location         ADCBUF + numAdcSamples      (set in rangingDSP_ConfigDataInEDMA)
        // The second  PING transfer is antenna two from location       ADCBUF + 2*numAdcSamples    (set in rangingDSP_ConfigDataInEDMA (base address + srcBIdx))
        // The second  PONG transfer is antenna three from location     ADCBUF + 3*numAdcSamples    (set in rangingDSP_ConfigDataInEDMA (base address + srcBIdx))
        EDMA_startDmaTransfer(edmaHandle, rangingObj->dataInChan[0]);

        // If x is even pingPongId(x) returns 0.
        // If x is odd pingPongId(x) returns 1.
        chirpPingPongId = pingPongId(rangingObj->chirpCount);

        // 1d fft for first antenna, followed by kicking off the DMA of fft output
        for (rxChanId = 0; rxChanId < DPParams->numRxAntennas; rxChanId++)
        {
            int16_t     *imagSrcAddr;
            int16_t     *realSrcAddr;
            int16_t     *fftDestAddr;
            uint8_t     inChannel;

            // ********************************
             //  Data Input
             // ********************************
            // Either DPC_RANGING_DSP_DPU_EDMAIN_PING_CH or DPC_RANGING_DSP_DPU_EDMAIN_PONG_CH
            inChannel = rangingObj->dataInChan[pingPongId(rxChanId)];

            if(rxChanId < DPParams->numRxAntennas - 1U)
            {
                // Kick off DMA to fetch data from ADC buffer for the next channel
                EDMA_startDmaTransfer(edmaHandle, rangingObj->dataInChan[pingPongId(rxChanId + 1)]);
            }

            // Verify if DMA has completed for current antenna
            startTime1 = Cycleprofiler_getTimeStamp();
            rangingDSP_WaitEDMAComplete (  edmaHandle, inChannel);
            waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);

            // ********************************
            // Data Processing
            // ********************************

            // Get the src/dest Address for FFT operation
            imagSrcAddr = (int16_t*)&rangingObj->adcDataIn[0];
            realSrcAddr = (int16_t*)&rangingObj->adcDataIn[0];
           fftDestAddr = (int16_t*)&rangingObj->fftOut1D[chirpPingPongId * DPParams->numAdcSamples * DPParams->numRxAntennas +
                                                      (DPParams->numAdcSamples * rxChanId)];

            ////////////////////////////////////
            // 1. Magnitude Calculation
            ////////////////////////////////////

            ////////////////////////////////////
            // 2. FFT
            ////////////////////////////////////

            // 16bit FFT in imre format
            //DSP_fft16x16
            DSP_fft16x16_imre(
                    (int16_t *) rangingObj->twiddle16x16,
                    DPParams->numAdcSamples,
                    (int16_t *)fftSrcAddr,
                    (int16_t *) fftDestAddr);


            ////////////////////////////////////
            // 3. Vector Multiply
            ////////////////////////////////////

            ////////////////////////////////////
            // 3. IFFT
            ////////////////////////////////////

            ////////////////////////////////////
            // 5.  Find Max
            ////////////////////////////////////

            // DSPF_sp_maxidx

        }

        // ********************************
        // * Data Output
        //  ********************************
        outChannel = rangingObj->dataOutChan[chirpPingPongId];

        uint32_t    radarCubeAddr;

        radarCubeAddr = (uint32_t)(rangingObj->radarCubebuf + rangingObj->chirpCount * rangingObj->numSamplePerChirp);
        EDMA_setDestinationAddress(edmaHandle, outChannel,
            (uint32_t)SOC_translateAddress((radarCubeAddr), SOC_TranslateAddr_Dir_TO_EDMA, NULL));

        if(rangingObj->chirpCount > 1U)
        {
            startTime1 = Cycleprofiler_getTimeStamp();
            rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
            waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);
        }

        EDMA_startDmaTransfer(edmaHandle, outChannel);

        // Increment chirp count
        rangingObj->chirpCount++;

        // Last chirp , wait until EDMA is completed
        if(rangingObj->chirpCount == DPParams->numChirpsPerFrame)
        {
            // Wait until last tansfer is done
            rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
            rangingObj->chirpCount = 0;
            outParams->endOfChirp = true;
        }

        rangingObj->numProcess++;
    }

    // Update outParams
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitingTime;
    outParams->stats.waitTime = waitingTime;

    rangingObj->inProgress = false;

exit:
    return retVal;
    */
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
