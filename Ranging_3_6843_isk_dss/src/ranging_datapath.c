/*
 * ranging_datapath.c
 *
 *  Created on: Jun 3, 2024
 *      Author: LeeLemay
 */

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/DSP_fft16x16_imre.h>
#include <inc/ranging_dss.h>
#include <inc/ranging_datapath.h>
#include <inc/ranging_res.h>
#include <inc/computed_twiddle_factor.h>
#include <shared/ranging_rfConfig.h>
#include <shared/ranging_mailbox.h>
#include <shared/gold_code.h>

// Copied over along with the C source file from DSPLIB
#include <inc/gen_twiddle_fft16x32.h>

#define Ranging_Assert(expression) {                \
              _ranging_Assert(expression,  __FILE__, __LINE__ );          \
               DebugP_assert(expression);                      \
                                      }

/*! L3 RAM buffer for results */
// A chunk is reserved for the twiddle factor, precalculated in "computed_twiddle_factor.h"
#define RANGING_OBJDET_L3RAM_SIZE (SOC_L3RAM_SIZE - 16U * 1024U)
uint8_t g_rangingL3Heap[RANGING_OBJDET_L3RAM_SIZE];
#pragma DATA_SECTION(g_rangingL3Heap, ".l3ram");

#define RANGING_OBJDET_L2RAM_SIZE (80U * 1024U)
uint8_t g_rangingL2Heap[RANGING_OBJDET_L2RAM_SIZE];
#pragma DATA_SECTION(g_rangingL2Heap, ".dpc_l2Heap");

 /*! L1DSRAM RAM buffer - used to hold ADC IN*/
// L1 hwRes->adcDataInL1_16kB - 16kB
#define RANGING_OBJDET_L1RAM_SIZE (16U * 1024U)
uint8_t g_rangingL1Heap[RANGING_OBJDET_L1RAM_SIZE];


#pragma DATA_SECTION(g_rangingL1Heap, ".dpc_l1Heap");


#pragma SET_CODE_SECTION(".l1pcode")

extern Ranging_DSS_MCB gMmwDssMCB;

int32_t ranging_setupGoldCode();
int32_t ranging_dssProcessGoldCode (  );
int32_t Ranging_mmWaveCtrlStop (void);

/**
 *  @b Description
 *  @n
 *      Sends Assert
 *
 *  @retval
 *      Not Applicable.
 */
void _ranging_Assert(int32_t expression, const char *file, int32_t line)
{
    if (!expression)
    {
        System_printf("Assert: %s %i\n", file, line);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
static void ranging_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
static void *ranging_MemPoolAlloc(MemPoolObj *pool,
                                  uint32_t size,
                                  uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
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
static int32_t ranging_configDataInEDMA ( )
{
    int32_t retVal;
    DPEDMA_syncACfg         syncACfg;

    //////////////////////////////////////////////////////////////////
    // Copy an antennas worth of contiguous cmplx16ImRe_t
    // samples into the data in buffer
    //////////////////////////////////////////////////////////////////

    // aCount: One antenna's worth
    syncACfg.aCount = RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t);

    // bCount is 1: We get the only antenna's worth of data in a single copy operation
    // and there is only one chirp per chirp event
    syncACfg.bCount = 1;

    // srcBIdx don't care
    syncACfg.srcBIdx = 0U ;

    // dstBidx don't care
    syncACfg.dstBIdx = 0U;

    // Only PING configuration
    gMmwDssMCB.dataPathObject.workingVariables.ADCdataBuf = (void *)SOC_XWR68XX_DSS_ADCBUF_BASE_ADDRESS;
    syncACfg.srcAddress = (uint32_t)gMmwDssMCB.dataPathObject.workingVariables.ADCdataBuf;
    syncACfg.destAddress = (uint32_t)gMmwDssMCB.dataPathObject.workingVariables.adcDataInL1_16kB;

    retVal = DPEDMA_configSyncA_singleFrame(
            gMmwDssMCB.edmaContainer.edmaCfg.edmaHandle,
            &gMmwDssMCB.edmaContainer.edmaCfg.dataInPing,
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
    do
    {
        if (EDMA_isTransferComplete(edmaHandle,
                                    (uint8_t) chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
        }
    } while (isTransferDone == false);
}

static int32_t ranging_getADCSamples ( )
{
    int32_t         retVal              = 0;
    EDMA_Handle     edmaHandle          = gMmwDssMCB.edmaContainer.edmaCfg.edmaHandle;
    rangingDSPObj_t * workingVariables  = &gMmwDssMCB.dataPathObject.workingVariables;
    uint32_t        dataInAddr          = (uint32_t)&workingVariables->ADCdataBuf[0];
    uint8_t         channel             = gMmwDssMCB.edmaContainer.edmaCfg.dataInPing.channel;
    ////////////////////////////////////////////////////
    // ADC SAMPLES
    // Set EDMA input source Address
    retVal = EDMA_setSourceAddress(
            edmaHandle,
            channel,
        (uint32_t) SOC_translateAddress(dataInAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal)
    {
        return retVal;
    }

    // Transfer all of the samples for this chirp
    // It sends them into &rangingObj->adcDataIn[0]
    retVal = EDMA_startDmaTransfer(edmaHandle, channel);
    if (retVal)
    {
        return retVal;
    }

    // Verify if DMA has completed for current antenna
    rangingDSP_WaitEDMAComplete ( edmaHandle, channel );
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for chirp available. It runs in the ISR context.
 *
 *  @retval
 *      Not Applicable.
 */
static void ranging_dssChirpIntHandler(uintptr_t arg)
{
    gMmwDssMCB.dataPathObject.rangingData.chirpStartTimeLow = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.chirpStartTimeHigh = TSCH;

    /* Increment interrupt counter for debugging purpose */
    gMmwDssMCB.stats.chirpIntCounter++;

    /* Post event to notify chirp available interrupt */
    Event_post(gMmwDssMCB.eventHandle, RANGING_CHIRP_EVT);
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for frame start ISR.
 *
 *  @retval
 *      Not Applicable.
 */
static void ranging_dssFrameStartIntHandler(uintptr_t arg)
{
    gMmwDssMCB.dataPathObject.rangingData.frameStartTimeLow     = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.frameStartTimeHigh    = TSCH;

    /* Increment interrupt counter for debugging purpose */
    gMmwDssMCB.stats.frameStartIntCounter++;

    /* Post event to notify frame start interrupt */
    Event_post(gMmwDssMCB.eventHandle, RANGING_FRAMESTART_EVT);
}


/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t ranging_dssDataPathOneTimeConfig(void)
{
    int32_t                 errCode;
    SOC_SysIntListenerCfg   socIntCfg;
    rangingDSPObj_t*        workingVariables;
    uint16_t                numSamples = RX_NUM_SAMPLES;

    //////////////////////////////////////////////////////////////////////////
    // 1. REGISTER INTERRUPTS
    //////////////////////////////////////////////////////////////////////////

    // Register chirp interrupt listener //
    socIntCfg.systemInterrupt = SOC_XWR68XX_DSS_INTC_EVENT_CHIRP_AVAIL;
    socIntCfg.listenerFxn     = ranging_dssChirpIntHandler;
    socIntCfg.arg             = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register chirp interrupt listener , error = %d\n", errCode);
        return -1;
    }

    // Register frame start interrupt listener //
    socIntCfg.systemInterrupt = SOC_XWR68XX_DSS_INTC_EVENT_FRAME_START;
    socIntCfg.listenerFxn     = ranging_dssFrameStartIntHandler;
    socIntCfg.arg             = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
        return -1;
    }

    //////////////////////////////////////////////////////////////////////////
    // 2. ALLOCATE MEMORY
    //////////////////////////////////////////////////////////////////////////
    gMmwDssMCB.dataPathObject.CoreL1RamObj.cfg.addr = &g_rangingL1Heap[0];
    gMmwDssMCB.dataPathObject.CoreL1RamObj.cfg.size = sizeof(g_rangingL1Heap);
    gMmwDssMCB.dataPathObject.CoreL2RamObj.cfg.addr = &g_rangingL2Heap[0];
    gMmwDssMCB.dataPathObject.CoreL2RamObj.cfg.size = sizeof(g_rangingL2Heap);
    gMmwDssMCB.dataPathObject.L3RamObj.cfg.addr = (void *)&g_rangingL3Heap[0];
    gMmwDssMCB.dataPathObject.L3RamObj.cfg.size = sizeof(g_rangingL3Heap);

    ranging_MemPoolReset(&gMmwDssMCB.dataPathObject.L3RamObj);
    ranging_MemPoolReset(&gMmwDssMCB.dataPathObject.CoreL2RamObj);
    ranging_MemPoolReset(&gMmwDssMCB.dataPathObject.CoreL1RamObj);

    // L1 adcDataInSize
    workingVariables = &gMmwDssMCB.dataPathObject.workingVariables;
    workingVariables->adcDataInL1_16kB   = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.CoreL1RamObj,
                                                                numSamples * sizeof(cmplx16ImRe_t),
                                                                DPU_RANGINGDSP_ADCDATAIN_BYTE_ALIGNMENT_DSP);
    Ranging_Assert((int)workingVariables->adcDataInL1_16kB);

    // L2 FFT twiddle
    workingVariables->fftTwiddle16x16L2_16kB    = (cmplx16ImRe_t *)ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.CoreL2RamObj,
                                                                                              numSamples * sizeof(cmplx16ImRe_t),
                                                                                              DPU_RANGINGDSP_TWIDDLEBUF_BYTE_ALIGNMENT_DSP);
    DebugP_assert(workingVariables->fftTwiddle16x16L2_16kB != NULL);

    // L2 Gold Code FFT
    workingVariables->fftGoldCodeL2_16kB    = (cmplx16ImRe_t *)ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.CoreL2RamObj,
                                                                                                     numSamples * sizeof(cmplx16ImRe_t),
                                                                                                     DPU_RANGINGDSP_FFTOUT_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)workingVariables->fftGoldCodeL2_16kB);

    // Ensure that fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB are contiguous
    // They need to be combined to form a 32 kB buffer
    DebugP_assert(workingVariables->fftTwiddle16x16L2_16kB + numSamples == workingVariables->fftGoldCodeL2_16kB);
    workingVariables->scratchBufferOneL2_32kB     = workingVariables->fftTwiddle16x16L2_16kB;   // 32kB contiguous RAM from fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB

    // L2 IFFT twiddle
    workingVariables->ifftTwiddle16x32L2_16kB   = (cmplx16ImRe_t *)ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.CoreL2RamObj,
                                                                                              numSamples * sizeof(cmplx16ImRe_t),
                                                                                              DPU_RANGINGDSP_TWIDDLEBUF_BYTE_ALIGNMENT_DSP);
    DebugP_assert(workingVariables->ifftTwiddle16x32L2_16kB != NULL);

    // L2 Scratch buffer
    workingVariables->scratchBufferTwoL2_32kB   = (cmplx16ImRe_t *)ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.CoreL2RamObj,
                                                                                      2 * numSamples * sizeof(cmplx16ImRe_t),
                                                                                      DPU_RANGINGDSP_FFTOUT_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)workingVariables->scratchBufferTwoL2_32kB);

    // L3 allocations
    // ADC Data - complex array of 2 byte values
    workingVariables->ADCDataL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx16ImRe_t),
                                                       sizeof(int16_t));

    // Magnitude - complex array of 2 byte values - magnitudeDataL3
    workingVariables->magnitudeDataL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx16ImRe_t),
                                                       sizeof(int16_t));

    // FFT data - complex array of 2 byte values - fftOfMagnitudeL3
    workingVariables->fftOfMagnitudeL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx16ImRe_t),
                                                       sizeof(int16_t));

    // Vector Multiply - complex array of 4 byte values - twice as large as others - vectorMultiplyOfFFtedDataL3
    workingVariables->vectorMultiplyOfFFtedDataL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx32ImRe_t),
                                                       sizeof(int16_t));

    // IFFT - complex array of 4 byte values - twice as large as others - iFftDataL3
    workingVariables->iFftDataL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx32ImRe_t),
                                                       sizeof(int16_t));

    // MAG IFFT - complex array of 4 byte values - twice as large as others - magIfftDataL3
    workingVariables->magIfftDataL3 = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx32ImRe_t),
                                                       sizeof(int16_t));

    // Complex Conjugate of Gold Code - complex array of 2 byte values - fftGoldCodeL3_16kB
    workingVariables->fftGoldCodeL3_16kB = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx16ImRe_t),
                                                       sizeof(int16_t));

    // IFFT twiddle factors - complex array of 2 byte values - ifftTwiddle16x16L3_16kB
    workingVariables->ifftTwiddle16x16L3_16kB = ranging_MemPoolAlloc(&gMmwDssMCB.dataPathObject.L3RamObj,
                                                       numSamples * sizeof(cmplx16ImRe_t),
                                                       sizeof(int16_t));

    workingVariables->fftTwiddle16x16L3_16kB      = (cmplx16ImRe_t *)twiddle_factors;     // Each value 2 bytes
    memcpy(workingVariables->fftTwiddle16x16L2_16kB, twiddle_factors, RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t) );

    //////////////////////////////////////////////////////////////////////////
    // 3. EDMA
    //////////////////////////////////////////////////////////////////////////
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPing.channel        = DPC_RANGING_DSP_DPU_EDMAIN_PING_CH;
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPing.channelShadow  = DPC_RANGING_DSP_DPU_EDMAIN_PING_SHADOW;
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPing.eventQueue     = DPC_RANGING_DSP_DPU_EDMAIN_PING_EVENT_QUE;
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPong.channel        = DPC_RANGING_DSP_DPU_EDMAIN_PONG_CH;
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPong.channelShadow  = DPC_RANGING_DSP_DPU_EDMAIN_PONG_SHADOW;
    gMmwDssMCB.edmaContainer.edmaCfg.dataInPong.eventQueue     = DPC_RANGING_DSP_DPU_EDMAIN_PONG_EVENT_QUE;

    // Ping - used for RX of 1 chirp of gold codes
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPing.channel       = DPC_RANGING_DSP_DPU_EDMAOUT_PING_CH;
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPing.channelShadow = DPC_RANGING_DSP_DPU_EDMAOUT_PING_SHADOW;
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPing.eventQueue    = DPC_RANGING_DSP_DPU_EDMAOUT_PING_EVENT_QUE;

    // Pong  - currently unused
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPong.channel       = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_CH;
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPong.channelShadow = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_SHADOW;
    gMmwDssMCB.edmaContainer.edmaCfg.dataOutPong.eventQueue    = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_EVENT_QUE;

    if(ranging_configDataInEDMA ( ))
    {
        System_printf("Error configuring EDMA in.\n");
        Ranging_Assert(0);
    }

    //////////////////////////////////////////////////////////////////////////
    // 4. VARIABLE INITIALIZATION
    //////////////////////////////////////////////////////////////////////////
    // Generate twiddle factors for the IFFT. This is one time
    gen_twiddle_fft16x32((short *)workingVariables->ifftTwiddle16x32L2_16kB, RX_NUM_SAMPLES);
    workingVariables->rxPrn             = DEFAULT_PRN;
    workingVariables->goldCodeNumBits   = GOLD_CODE_NUM_BITS;
    workingVariables->inProgress        = false;
    if(ranging_setupGoldCode())
    {
        System_printf("Error setting up gold codes.\n");
        Ranging_Assert(0);
    }




//    // DPC running on remote core, address need to be converted
//    objDetPreStartDspCfg.staticCfg.ADCBufData.data = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.staticCfg.ADCBufData.data,
//                                         SOC_TranslateAddr_Dir_TO_OTHER_CPU,
//                                         &errCode);
//    DebugP_assert ((uint32_t)objDetPreStartDspCfg.staticCfg.ADCBufData.data != SOC_TRANSLATEADDR_INVALID);
//    objDetPreStartDspCfg.staticCfg.ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.adcBits = 2; /* 16-bit */
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.dataFmt                  = DPIF_DATAFORMAT_COMPLEX16_IMRE;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.interleave               = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numAdcSamples            = RFparserOutParams.numAdcSamples;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent   = RFparserOutParams.numChirpsPerChirpEvent;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numRxAntennas            = RX_NUM_ANTENNAS;
//    objDetPreStartDspCfg.staticCfg.ADCBufData.dataSize                              = RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);

    return 0;
}

void itoa(uint32_t value, char* str, int base)
{
    char* ptr = str, *ptr1 = str, tmp_char;
    int tmp_value;

    do
    {
        tmp_value = value;
        value /= base;
        *ptr++ = "0123456789abcdef"[tmp_value - (value * base)];
    } while (value);

    // Apply null terminator
    *ptr-- = '\0';

    // Reverse the string
    while (ptr1 < ptr)
    {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
}

void formatString(uint32_t TSCHigh, uint32_t TSCLow, const char *inputString, char *outputString, size_t outputStringSize)
{
    // Initialize the stringData buffer
    memset(outputString, 0, outputStringSize);

    // Format the uint32_t values and the short string manually
    char buffer[20]; // Temporary buffer for integer to string conversion

    // Convert the first uint32_t to a string and concatenate
    itoa(TSCHigh, buffer, 10);
    strncat(outputString, buffer, outputStringSize - strlen(outputString) - 1);
    strncat(outputString, ".", outputStringSize - strlen(outputString) - 1);

    // Convert the second uint32_t to a string and concatenate
    itoa(TSCLow, buffer, 10);
    strncat(outputString, buffer, outputStringSize - strlen(outputString) - 1);
    strncat(outputString, ":", outputStringSize - strlen(outputString) - 1);

    // Concatenate the short string
    strncat(outputString, inputString, outputStringSize - strlen(outputString) - 1);

    // Ensure null termination
    outputString[outputStringSize - 1] = '\0';
}


/**
 *  @b Description
 *  @n
 *      Data Path main task that handles events from remote and do dataPath processing.
 *
 *  @retval
 *      Not Applicable.
 */
void ranging_dssDataPathTask(UArg arg0, UArg arg1)
{
    int32_t                 retVal = 0;
    UInt                    event;
    rangingDSPObj_t*        workingVariables;
    uint32_t                TSCLow;
    uint32_t                TSCHigh;
    char                    statusString[128];

    //////////////////////////////////////////////////////////////////////////
    // Data Path :: One time config
    //////////////////////////////////////////////////////////////////////////

    if ((retVal = ranging_dssDataPathOneTimeConfig()) < 0)
    {
        System_printf("Debug: DSS Data Path config failed with Error[%d]\n", retVal);
        goto exit;
    }

    gMmwDssMCB.sensorState = Ranging_SensorState_STARTED;
    workingVariables = &gMmwDssMCB.dataPathObject.workingVariables;

    //////////////////////////////////////////////////////////////////////////
    // Data Path :: Main loop
    //////////////////////////////////////////////////////////////////////////
    while (1)
    {
        event = Event_pend(gMmwDssMCB.eventHandle,
                           Event_Id_NONE,
                           RANGING_FRAMESTART_EVT | RANGING_CHIRP_EVT |
                           RANGING_STOP_EVT | RANGING_CONFIG_EVT |
                           RANGING_NEXT_TIMESLOT_STARTED_EVT,
                           BIOS_WAIT_FOREVER);

        TSCLow  = TSCL;
        TSCHigh = TSCH;

        if (event & RANGING_STOP_EVT)
        {
            // Output to the MSS
            formatString(TSCHigh, TSCLow, ": RANGING_STOP_EVT\r\n", &statusString[0], sizeof(statusString));
            dssSendStringToMss(&statusString[0]);
            memset(statusString, 0, sizeof(statusString));
        }

        //////////////////////////////////////////////////////////////////////////
        // The next timeslot started
        //////////////////////////////////////////////////////////////////////////
        if (event & RANGING_NEXT_TIMESLOT_STARTED_EVT)
        {
            // Can be posted from:
            // - sensor start countdown timer, clockISRSensorStart
            // - next timeslot started countdown timer, clockISRMsgMSS
            // - CMD_DSS_TO_START_SENSOR_NOW message, from dss_ipc_mailbox_task.c
            gMmwDssMCB.currentTimeslot = gMmwDssMCB.nextTimeslot;

            // Send an event to the MSS that a new timeslot has started
            dssReportsTimeslotStart();

            // Output to the MSS
            formatString(TSCHigh, TSCLow, ": RANGING_NEXT_TIMESLOT_STARTED_EVT\r\n", &statusString[0], sizeof(statusString));
            dssSendStringToMss(&statusString[0]);
            memset(statusString, 0, sizeof(statusString));
        }

        //////////////////////////////////////////////////////////////////////////
        // Data Path process frame start event
        //////////////////////////////////////////////////////////////////////////
        if (event & RANGING_FRAMESTART_EVT)
        {
            // Make sure we're not still processing
            Ranging_Assert(!workingVariables->inProgress);
            workingVariables->areAllChirpsCompletedForThisFrame = false;
            workingVariables->inProgress = true;

            // Output to the MSS
            formatString(TSCHigh, TSCLow, ": RANGING_FRAMESTART_EVT\r\n", &statusString[0], sizeof(statusString));
            dssSendStringToMss(&statusString[0]);
            memset(statusString, 0, sizeof(statusString));
        }

        //////////////////////////////////////////////////////////////////////////
        // Data Path process chirp event
        //////////////////////////////////////////////////////////////////////////
        if (event & RANGING_CHIRP_EVT)
        {
            workingVariables->chirpCount++;

            // Check for last chirp
            if(workingVariables->chirpCount == workingVariables->DPParams.numChirpsPerFrame)
            {
                // Stop the sensor
                if(Ranging_mmWaveCtrlStop())
                {
                    System_printf("Error stopping sensor.\n");
                    Ranging_debugAssert(0);
                }

                workingVariables->chirpCount = 0;
                workingVariables->areAllChirpsCompletedForThisFrame = true;

                // If we are receiving, we need to process the result
                if( gMmwDssMCB.currentTimeslot.slotType == SLOT_TYPE_SYNCHRONIZATION_RX ||
                    gMmwDssMCB.currentTimeslot.slotType == SLOT_TYPE_RANGING_START_CODE_RX ||
                    gMmwDssMCB.currentTimeslot.slotType == SLOT_TYPE_RANGING_RESPONSE_CODE_RX)
                {
                    ranging_getADCSamples();
                    ranging_dssProcessGoldCode();
                }
                dssReportsResult(&gMmwDssMCB.dataPathObject.rangingData);
                workingVariables->inProgress = false;

                // Output to the MSS
                formatString(TSCHigh, TSCLow, ": Last RANGING_CHIRP_EVT\r\n", &statusString[0], sizeof(statusString));
                dssSendStringToMss(&statusString[0]);
                memset(statusString, 0, sizeof(statusString));
            }
            formatString((uint32_t)gMmwDssMCB.stats.chirpIntCounter, gMmwDssMCB.dataPathObject.rangingData.chirpStartTimeLow, "CHIRP\r\n", &statusString[0], sizeof(statusString));
            //dssSendStringToMss(&statusString[0]);
            memset(statusString, 0, sizeof(statusString));
        }

        //////////////////////////////////////////////////////////////////////////////
        // Data Path re-config
        //////////////////////////////////////////////////////////////////////////////
        if (event & RANGING_CONFIG_EVT)
        {
            //////////////////////////////////////////////////////////////////////////
            // SETUP GOLD CODES
            //////////////////////////////////////////////////////////////////////////
            if( gMmwDssMCB.nextTimeslot.slotType == SLOT_TYPE_SYNCHRONIZATION_TX ||
                gMmwDssMCB.nextTimeslot.slotType == SLOT_TYPE_RANGING_START_CODE_TX ||
                gMmwDssMCB.nextTimeslot.slotType == SLOT_TYPE_RANGING_RESPONSE_CODE_TX)
            {
                // TX
                workingVariables->DPParams.numChirpsPerFrame    = pow(2,gMmwDssMCB.nextTimeslot.goldCodeNumBits) - 1;
                // Output to the MSS
                formatString(TSCHigh, TSCLow, ": CFG TX\r\n", &statusString[0], sizeof(statusString));
            }
            else
            {
                // RX
                workingVariables->DPParams.numChirpsPerFrame    = 1;

                // Do we need to compute the Gold Code and the complex conjugate of its FFT?
                if( workingVariables->rxPrn != gMmwDssMCB.nextTimeslot.prn ||
                    workingVariables->goldCodeNumBits != gMmwDssMCB.nextTimeslot.goldCodeNumBits)
                {
                    workingVariables->rxPrn         = gMmwDssMCB.nextTimeslot.prn;
                    workingVariables->goldCodeNumBits   = gMmwDssMCB.nextTimeslot.goldCodeNumBits;

                    if(ranging_setupGoldCode())
                    {
                        System_printf("Error setting up gold codes.\n");
                        Ranging_Assert(0);
                    }
                }
                // Output to the MSS
                formatString(TSCHigh, TSCLow, ": CFG RX\r\n", &statusString[0], sizeof(statusString));
            }
            dssSendStringToMss(&statusString[0]);
            memset(statusString, 0, sizeof(statusString));
        }
    }

exit:
    System_printf("Debug: DSS Data path exit\n");
}
