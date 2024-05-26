/*
 *   @file  objectdetection.c
 *
 *   @brief
 *      Ranging DPC implementation using DSP.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* mmWave SDK Include Files: */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/board/antenna_geometry.h>

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this object detection DPC are prefixed by DPC_RANGING_ */
#include <inc/ranging_res.h>
#include <inc/rangingdsp_internal.h>

/* Ranging instance etc */
#include <inc/ranging_dpc_internal.h>
#include <inc/ranging_dpc.h>
#include <inc/ranging_dss.h>

#include <ti/alg/mmwavelib/mmwavelib.h>

#define DBG_DPC_OBJDET

#ifdef DBG_DPC_OBJDET
ObjDetObj     *gObjDetObj;
#endif

#pragma SET_CODE_SECTION(".l1pcode")

/**************************************************************************
 ************************** Local Definitions **********************************
 **************************************************************************/

 /** @addtogroup DPC_OBJDET_IOCTL__INTERNAL_DEFINITIONS
  @{ */

/*! Radar cube data buffer alignment in bytes. No DPU module specifying alignment
 *  need (through a \#define) implies that no alignment is needed i.e 1 byte alignment.
 *  But we do the natural data type alignment which is 2 bytes (as radar cube is complex16-bit type)
 *  because radar cube is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing.
 */
#define DPC_RANGING_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT       (sizeof(int16_t))

/*! Detection matrix alignment is declared by CFAR dpu, we size to
 *  the max of this and CPU alignment for accessing detection matrix
 *  it is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing. Note currently the CFAR alignment is the same as
 *  CPU alignment so this max is redundant but it is more to illustrate the
 *  generality of alignments should be done.
 */
#define DPC_RANGING_DET_MATRIX_DATABUF_BYTE_ALIGNMENT       (MAX(sizeof(uint16_t), \
                                                                DPU_CFARCAPROCDSP_DET_MATRIX_BYTE_ALIGNMENT))

/*! cfar list alignment is declared by cfar and AoA dpu, for debug purposes we size to
 *  the max of these and CPU alignment for accessing cfar list for debug purposes.
 *  Note currently the dpu alignments are the same as CPU alignment so these max are
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_RANGING_CFAR_DET_LIST_BYTE_ALIGNMENT            (MAX(MAX(DPU_CFARCAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT,  \
                                                                    DPU_AOAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT),\
                                                                DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT))


/*! Point cloud cartesian alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#ifdef SOC_XWR68XX
#define DPC_RANGING_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT))
#else
/* Speculative workaround for an issue where EDMA is not completing transfer. */
#define DPC_RANGING_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(MAX(DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                       DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT), \
                                                                   64U))
#endif

/*! Point cloud side info alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#ifdef SOC_XWR68XX
#define DPC_RANGING_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT))
#else
/* Speculative workaround for an issue where EDMA is not completing transfer. */
#define DPC_RANGING_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(MAX(DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                       DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT), \
                                                                   64U))
#endif

/*! Azimuth static heat map alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_RANGING_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT, \
                                                                   sizeof(int16_t)))

/*! Elevation angle alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_RANGING_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT, \
                                                                   sizeof(float)))

/**
@}
*/
/*! Maximum Number of objects that can be detected in a frame */
#define DPC_RANGING_MAX_NUM_OBJECTS                       500U

/* Window definition for range and doppler */
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE            MMWAVELIB_WIN_BLACKMAN
#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE          MMWAVELIB_WIN_HANNING

/* QFORMAT for Range and Doppler Windowing */
#define DPC_RANGING_DSP_QFORMAT_RANGE_FFT 15
#define DPC_RANGING_DSP_QFORMAT_DOPPLER_FFT 19


////////////////////////////////////////////////////////////////////////////
////////////////////////// Globals /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern Ranging_DSS_MCB    gMmwDssMCB;

/**************************************************************************
 ************************** Local Functions Prototype **********************
 **************************************************************************/

static DPM_DPCHandle DPC_Ranging_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
);

static int32_t DPC_Ranging_execute
(
    DPM_DPCHandle handle,
    DPM_Buffer*       ptrResult
);

static int32_t DPC_Ranging_ioctl
(
    DPM_DPCHandle   handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
);

static int32_t DPC_Ranging_start  (DPM_DPCHandle handle);
static int32_t DPC_Ranging_stop   (DPM_DPCHandle handle);
static int32_t DPC_Ranging_deinit (DPM_DPCHandle handle);
static void DPC_Ranging_frameStart (DPM_DPCHandle handle);
void DPC_Ranging_chirpEvent (DPM_DPCHandle handle);

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/
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
static void DPC_ObjDetDSP_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}

/**
 *  @b Description
 *  @n
 *      Utility function for setting memory pool to desired address in the pool.
 *      Helps to rewind for example.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  addr Address to assign to the pool's current address.
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
/*
static void DPC_ObjDetDSP_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr = (uintptr_t)addr;
    pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
}
*/

/**
 *  @b Description
 *  @n
 *      Utility function for getting memory pool current address.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to current address of the pool (from which next allocation will
 *      allocate to the desired alignment).
 */
/*
static void *DPC_ObjDetDSP_MemPoolGet(MemPoolObj *pool)
{
    return((void *)pool->currAddr);
}
*/

/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
static uint32_t DPC_ObjDetDSP_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
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
static void *DPC_ObjDetDSP_MemPoolAlloc(MemPoolObj *pool,
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
 *      Sends Assert
 *
 *  @retval
 *      Not Applicable.
 */
void _DPC_Objdet_Assert(DPM_Handle handle, int32_t expression,
                        const char *file, int32_t line)
{
    DPM_DPCAssert       fault;

    if (!expression)
    {
        fault.lineNum = (uint32_t)line;
        fault.arg0    = 0U;
        fault.arg1    = 0U;
        strncpy (fault.fileName, file, (DPM_MAX_FILE_NAME_LEN-1));

        /* Report the fault to the DPM entities */
        DPM_ioctl (handle,
                   DPM_CMD_DPC_ASSERT,
                   (void*)&fault,
                   sizeof(DPM_DPCAssert));
    }
}

/**
 *  @b Description
 *  @n
 *      DPC chirp event function registered with DPM. This is invoked on reception
 *      of the chirp data available ISR from the RF front-end. This API is also invoked
 *      when application issues @ref DPC_RANGING_IOCTL__TRIGGER_CHIRP to simulate
 *      a chirp event trigger (e.g for unit testing purpose).
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void DPC_Ranging_chirpEvent (DPM_DPCHandle handle)
{
    uint32_t        chirpTimeLow = Cycleprofiler_getTimeStamp();
    uint32_t        chirpTimeHigh = TSCH;
    ObjDetObj       *objDetObj = (ObjDetObj *) handle;
    uint32_t        margin;

    objDetObj->rangingData.chirpStartTimeLow = chirpTimeLow;
    objDetObj->rangingData.chirpStartTimeHigh = chirpTimeHigh;
    memset(&objDetObj->rangingData.detectionStats, 0, sizeof(Ranging_PRN_Detection_Stats));
    objDetObj->rangingData.responseStartTimeLow = 0;
    objDetObj->rangingData.responseStartTimeHigh = 0;

    // Make sure we're not still processing
//    if( !objDetObj->interSubFrameProcToken )
//    {
        if(objDetObj->chirpIndex != 0)
        {
            margin = Cycleprofiler_getTimeStamp() - objDetObj->chirpEndTime;
            if(margin < objDetObj->chirpMargin)
            {
                /* Find the smallest margin to report as chirp margin */
                objDetObj->chirpMargin = margin;
            }
        }
        else
        {
            /* Reset margin min */
            objDetObj->chirpMargin = 0xffffffff;
        }

        /* Notify the DPM Module that the DPC is ready for execution */
        DebugP_assert (DPM_notifyExecute (objDetObj->dpmHandle, handle, true) == 0);
    //}

    return;
}

/**
 *  @b Description
 *  @n
 *     Configure range DPU.
 *
 *  @param[in]  dpuHandle       Handle to Range DPU
 *  @param[in]  staticCfg       Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg          Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle      Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube       Pointer to DPIF radar cube, which is output of range
 *                              processing.
 *  @param[in]  CoreL2RamObj    Pointer to core local L2 RAM object to allocate local memory
 *                              for the DPU, only for scratch purposes
 *  @param[in]  CoreL1RamObj    Pointer to core local L1 RAM object to allocate local memory
 *                              for the DPU, only for scratch purposes
 *  @param[in]  twiddle         Pointer to range FFT twiddle buffer
 *  @param[in]  twiddleSize     Range FFT twiddle buffer size in bytes. See Range DPU
 *                              configuration for more information.
 *  @param[out] cfgSave         Configuration that is built in local
 *                              (stack) variable is saved here. This is for facilitating
 *                              quick reconfiguration later without having to go through
 *                              the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 */
#pragma FUNCTION_OPTIONS(DPC_ObjDetDSP_rangeConfig, "--opt_for_speed")
#pragma CODE_SECTION(DPC_ObjDetDSP_rangeConfig, ".l1pcode")
static int32_t DPC_ObjDetDSP_rangeConfig
(
    DPU_RangingDSP_Handle dpuHandle,
    DPC_Ranging_StaticCfg *staticCfg,
    EDMA_Handle                   edmaHandle,
    DPIF_RadarCube                *radarCube,
    MemPoolObj                    *CoreL2RamObj,
    MemPoolObj                    *CoreL1RamObj,
    DPU_RangingDSP_Config       *cfgSave
)
{
    int32_t retVal = 0;
    DPU_RangingDSP_Config rangeCfg;
    DPU_RangingDSP_HW_Resources *hwRes = &rangeCfg.hwRes;
    DPU_RangingDSP_EDMAConfig *edmaCfg = &hwRes->edmaCfg;
    uint32_t numRxAntennas;
    uint16_t numSamples = staticCfg->ADCBufData.dataProperty.numAdcSamples;

    memset(&rangeCfg, 0, sizeof(rangeCfg));

    numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;

    /* rangeProcDSP only supports non-interleaved at present */
    DebugP_assert(staticCfg->ADCBufData.dataProperty.interleave == DPIF_RXCHAN_NON_INTERLEAVE_MODE);

    /* static configuration */
    rangeCfg.staticCfg.ADCBufData           = staticCfg->ADCBufData;
    rangeCfg.staticCfg.numChirpsPerFrame    = staticCfg->numChirpsPerFrame;
    rangeCfg.staticCfg.numTxAntennas        = staticCfg->numTxAntennas;
    rangeCfg.staticCfg.numVirtualAntennas   = staticCfg->numVirtualAntennas;
    rangeCfg.staticCfg.adcSampleRate        = staticCfg->adcSampleRate;
    rangeCfg.staticCfg.rxPrn                = staticCfg->rxPrn;

    /* radarCube */
    hwRes->radarCube = *radarCube;

    ///////////////////////////////////////////////////////
    // MEMORY ALLOCATIONS
    ///////////////////////////////////////////////////////

    // L1 adcDataInSize
    if(numRxAntennas > 1)
    {
        // For more than one antenna, we use a ping/pong approach
        hwRes->adcDataInSize = 2U * numSamples * sizeof(cmplx16ImRe_t);
        DebugP_assert(0);
    }
    else
    {
        hwRes->adcDataInSize = numSamples * sizeof(cmplx16ImRe_t);
    }
    hwRes->adcDataInL1_16kB                 = DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  hwRes->adcDataInSize,
                                                  DPU_RANGINGDSP_ADCDATAIN_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)hwRes->adcDataInL1_16kB);

    // L2 FFT twiddle
    hwRes->fftTwiddleSize                   = numSamples * sizeof(cmplx16ImRe_t);
    hwRes->fftTwiddle16x16L2_16kB           = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                                      hwRes->fftTwiddleSize,
                                              DPU_RANGINGDSP_TWIDDLEBUF_BYTE_ALIGNMENT_DSP);
    DebugP_assert(hwRes->fftTwiddle16x16L2_16kB != NULL);

    // L2 Gold Code FFT
    hwRes->localGoldCodeFFTBufferSize       = numSamples * sizeof(cmplx16ImRe_t);
    hwRes->localGoldCodeFFTBufferL2_16kB    = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  hwRes->localGoldCodeFFTBufferSize,
                                                  DPU_RANGINGDSP_FFTOUT_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)hwRes->localGoldCodeFFTBufferL2_16kB);

    // Ensure that fftTwiddle16x16L2_16kB and localGoldCodeFFTBufferL2_16kB are contiguous
    // They need to be combined to form a 32 kB buffer
    DebugP_assert(hwRes->fftTwiddle16x16L2_16kB + numSamples == hwRes->localGoldCodeFFTBufferL2_16kB);

    // L2 IFFT twiddle
    hwRes->ifftTwiddleSize                  = numSamples * sizeof(cmplx16ImRe_t);
    hwRes->ifftTwiddle16x16L2_16kB          = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                                      hwRes->ifftTwiddleSize,
                                              DPU_RANGINGDSP_TWIDDLEBUF_BYTE_ALIGNMENT_DSP);
    DebugP_assert(hwRes->ifftTwiddle16x16L2_16kB != NULL);

    // L2 Scratch buffer
    hwRes->scratchBufferSize                = 2* numRxAntennas * numSamples * sizeof(cmplx16ImRe_t);
    hwRes->scratchBufferL2_32kB             = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  hwRes->scratchBufferSize,
                                                  DPU_RANGINGDSP_FFTOUT_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)hwRes->scratchBufferL2_32kB);

    // hwres - edma
    edmaCfg->edmaHandle = edmaHandle;
    edmaCfg->dataInPing.channel             = DPC_RANGING_DSP_DPU_EDMAIN_PING_CH;
    edmaCfg->dataInPing.channelShadow       = DPC_RANGING_DSP_DPU_EDMAIN_PING_SHADOW;
    edmaCfg->dataInPing.eventQueue          = DPC_RANGING_DSP_DPU_EDMAIN_PING_EVENT_QUE;
    edmaCfg->dataInPong.channel             = DPC_RANGING_DSP_DPU_EDMAIN_PONG_CH;
    edmaCfg->dataInPong.channelShadow       = DPC_RANGING_DSP_DPU_EDMAIN_PONG_SHADOW;
    edmaCfg->dataInPong.eventQueue          = DPC_RANGING_DSP_DPU_EDMAIN_PONG_EVENT_QUE;

    /* Ping */
    edmaCfg->dataOutPing.channel            = DPC_RANGING_DSP_DPU_EDMAOUT_PING_CH;
    edmaCfg->dataOutPing.channelShadow      = DPC_RANGING_DSP_DPU_EDMAOUT_PING_SHADOW;
    edmaCfg->dataOutPing.eventQueue         = DPC_RANGING_DSP_DPU_EDMAOUT_PING_EVENT_QUE;

    /* Pong */
    edmaCfg->dataOutPong.channel            = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_CH;
    edmaCfg->dataOutPong.channelShadow      = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_SHADOW;
    edmaCfg->dataOutPong.eventQueue         = DPC_RANGING_DSP_DPU_EDMAOUT_PONG_EVENT_QUE;

    retVal = DPU_RangingDSP_config(dpuHandle, &rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated and
     * dc range sig should not be reset. */
    *cfgSave = rangeCfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Utility function to do a parabolic/quadratic fit on 3 input points
 *      and return the coordinates of the peak. This is used to accurately estimate
 *      range bias.
 *
 *  @param[in]  x Pointer to array of 3 elements representing the x-coordinate
 *              of the points to fit
 *  @param[in]  y Pointer to array of 3 elements representing the y-coordinate
 *              of the points to fit
 *  @param[out] xv Pointer to output x-coordinate of the peak value
 *  @param[out] yv Pointer to output y-coordinate of the peak value
 *
 *  @retval   None
 *
 * \ingroup DPC_RANGING__INTERNAL_FUNCTION
 */
/*
static void DPC_ObjDetDSP_quadFit(float *x, float*y, float *xv, float *yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];

    denom = (x0 - x1)*(x0 - x2)*(x1 - x2);
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
    b = (x2*x2 * (y0 - y1) + x1*x1 * (y2 - y0) + x0*x0 * (y1 - y2)) / denom;
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;

    *xv = -b/(2 * a);
    *yv = c - b*b/(4 * a);
}
*/
/**
 *  @b Description
 *  @n
 *      Sub-frame reconfiguration, used when switching sub-frames. Invokes the
 *      DPU configuration using the configuration that was stored during the
 *      pre-start configuration so reconstruction time is saved  because this will
 *      happen in real-time.
 *  @param[in]  objDetObj Pointer to DPC object
 *  @param[in]  subFrameIndx Sub-frame index.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 * \ingroup DPC_RANGING__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_reconfigSubFrame(ObjDetObj *objDetObj, uint8_t subFrameIndx)
{
    int32_t retVal = 0;
    SubFrameObj *subFrmObj;

    subFrmObj = &objDetObj->subFrameObj[subFrameIndx];
    retVal = DPU_RangingDSP_config(subFrmObj->dpuRangeObj, &subFrmObj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Function to initialize all DPUs used in the DPC chain
 *
 *  @param[in] objDetObj        Pointer to sub-frame object
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t DPC_ObjDetDSP_initDPU
(
    ObjDetObj     *objDetObj,
    uint8_t       numSubFrames
)
{
    SubFrameObj     *subFrmObj;
    int32_t         retVal = 0;
    int32_t         idx;

    for(idx = 0; idx < numSubFrames; idx++)
    {
        subFrmObj = &objDetObj->subFrameObj[idx];

        subFrmObj->dpuRangeObj = DPU_RangingDSP_init(&retVal);
        if (retVal != 0)
        {
            goto exit;
        }
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Function to de-initialize all DPUs used in the DPC chain
 *
 *  @param[in] objDetObj        Pointer to sub-frame object
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t DPC_ObjDetDSP_deinitDPU
(
    ObjDetObj     *objDetObj,
    uint8_t       numSubFrames
)
{
    SubFrameObj     *subFrmObj;
    int32_t         retVal = 0;
    int32_t         idx;

    for(idx = 0; idx < numSubFrames; idx++)
    {
        subFrmObj = &objDetObj->subFrameObj[idx];

        retVal = DPU_RangingDSP_deinit(subFrmObj->dpuRangeObj);

        if (retVal != 0)
        {
            goto exit;
        }
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *     Performs processing related to pre-start configuration, which is per sub-frame,
 *     by configuring each of the DPUs involved in the processing chain.
 *  Memory management notes:
 *  1. Core Local Memory that needs to be preserved across sub-frames (such as range DPU's calib DC buffer)
 *     will be allocated using MemoryP_alloc.
 *  2. Core Local Memory that needs to be preserved within a sub-frame across DPU calls
 *     (the DPIF * type memory) or for intermediate private scratch memory for
 *     DPU (i.e no preservation is required from process call to process call of the DPUs
 *     within the sub-frame) will be allocated from the Core Local RAM configuration supplied in
 *     @ref DPC_Ranging_InitParams given to @ref DPC_Ranging_init API
 *  3. L3 memory will only be allocated from the L3 RAM configuration supplied in
 *     @ref DPC_Ranging_InitParams given to @ref DPC_Ranging_init API
 *     No L3 buffers are presently required that need to be preserved across sub-frames
 *     (type described in #1 above), neither are L3 scratch buffers required for
 *     intermediate processing within DPU process call.
 *
 *  @param[in]  subFrameObj     Pointer to sub-frame object
 *  @param[in]  commonCfg       Pointer to pre-start common configuration
 *  @param[in]  preStartCfg     Pointer to pre-start configuration of the sub-frame
 *  @param[in]  edmaHandle      Pointer to array of EDMA handles for the device, this
 *                              can be distributed among the DPUs, the actual EDMA handle used
 *                              in DPC is determined by definition in application resource file
 *  @param[in]  L3ramObj        Pointer to L3 RAM memory pool object
 *  @param[in]  CoreL2RamObj    Pointer to Core Local L2 memory pool object
 *  @param[in]  CoreL1RamObj    Pointer to Core Local L1 memory pool object
 *  @param[out] L3RamUsage      Net L3 RAM memory usage in bytes as a result of allocation
 *                              by the DPUs.
 *  @param[out] CoreL2RamUsage  Net Local L2 RAM memory usage in bytes
 *  @param[out] CoreL1RamUsage  Net Core L1 RAM memory usage in bytes
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 */
#pragma FUNCTION_OPTIONS(DPC_ObjDetDSP_preStartConfig, "--opt_for_speed")
#pragma CODE_SECTION(DPC_ObjDetDSP_preStartConfig, ".l1pcode")
static int32_t DPC_ObjDetDSP_preStartConfig
(
    SubFrameObj                     *subFrameObj,
    DPC_Ranging_PreStartCommonCfg   *commonCfg,
    DPC_Ranging_PreStartCfg         *preStartCfg,
    EDMA_Handle                     edmaHandle[EDMA_NUM_CC],
    MemPoolObj                      *L3ramObj,
    MemPoolObj                      *CoreL2RamObj,
    MemPoolObj                      *CoreL1RamObj,
    uint32_t                        *L3RamUsage,
    uint32_t                        *CoreL2RamUsage,
    uint32_t                        *CoreL1RamUsage
)
{
    int32_t retVal = 0;
    DPC_Ranging_StaticCfg  *staticCfg;
    DPIF_RadarCube radarCube;
    uint32_t dataSize = 0;

    staticCfg = &preStartCfg->staticCfg;

    /* Save configs to object. We need to pass this stored config (instead of
       the input arguments to this function which will be in stack) to
       the DPU config functions inside of this function because the DPUs
       have pointers to dynamic configurations which are later going to be
       reused during re-configuration (intra sub-frame or inter sub-frame)
     */
    subFrameObj->staticCfg = *staticCfg;

    /* Run-time L3RAM memory configuration
     * By default, L3RAM memory configuration is initialized during DPC init time.
     * L3 RAM and radar cube memory configuration can also be overwritten at run time
     * in case the range DPU is not part of the DPC chain.
     */
    if(preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        /* L3 RAM configuration */
        if(preStartCfg->shareMemCfg.L3Ram.addr != NULL)
        {
            L3ramObj->cfg.addr = preStartCfg->shareMemCfg.L3Ram.addr;
            L3ramObj->cfg.size = preStartCfg->shareMemCfg.L3Ram.size;
        }
        else
        {
            retVal = DPC_RANGING_EINVAL__COMMAND;
            goto exit;
        }
    }

    /* Derived config */

    DPC_ObjDetDSP_MemPoolReset(L3ramObj);
    DPC_ObjDetDSP_MemPoolReset(CoreL2RamObj);
    DPC_ObjDetDSP_MemPoolReset(CoreL1RamObj);

    // L3 allocations
    // ADC Data - complex array of 2 byte values
    dataSize = staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx16ImRe_t);

    // Magnitude - complex array of 2 byte values
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx16ImRe_t);

    // FFT data - complex array of 2 byte values
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx16ImRe_t);

    // Vector Multiply - complex array of 4 byte values - twice as large as others
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx32ImRe_t);

    // IFFT - complex array of 4 byte values - twice as large as others
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx32ImRe_t);

    // MAG IFFT - complex array of 4 byte values - twice as large as others
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * staticCfg->numChirpsPerFrame *
            staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx32ImRe_t);

    // Complex Conjugate of Gold Code - complex array of 2 byte values
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(cmplx16ImRe_t);

    // FFT twiddle factors - complex array of 2 byte values
    dataSize = dataSize + staticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(cmplx16ImRe_t);

    // Add some space for headers
    dataSize = dataSize + 16; // Four 4 byte headers
    radarCube.dataSize = dataSize;
    if(preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        if((preStartCfg->shareMemCfg.radarCubeMem.addr != NULL) &&
          (preStartCfg->shareMemCfg.radarCubeMem.size == radarCube.dataSize))
        {
            /* Use assigned radar cube address */
            radarCube.data = preStartCfg->shareMemCfg.radarCubeMem.addr;
        }
        else
        {
            retVal = DPC_RANGING_EINVAL__COMMAND;
            goto exit;
        }
    }
    else
    {
        /* Allocate from memory */
        radarCube.data = DPC_ObjDetDSP_MemPoolAlloc(L3ramObj, radarCube.dataSize,
                                                 DPC_RANGING_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);

        if (radarCube.data == NULL)
        {
            retVal = DPC_RANGING_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }
    }

    /* Only supported radar Cube format in this DPC */
    radarCube.datafmt = DPIF_RADARCUBE_FORMAT_1;

    /* L1 or L2 Local memory allocaiton that are not shared between DPUs.
      It includes windowing coeffecients buffer, twiddle buffer, sin/cos table and CFAR detection list etc.
     */

    DebugP_assert(edmaHandle[DPC_RANGING_DSP_DPU_EDMA_INST_ID] != NULL);
    retVal = DPC_ObjDetDSP_rangeConfig(
            subFrameObj->dpuRangeObj,
            &subFrameObj->staticCfg,
            edmaHandle[DPC_RANGING_DSP_DPU_EDMA_INST_ID],
            &radarCube,
            CoreL2RamObj,
            CoreL1RamObj,
            &subFrameObj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Report RAM usage */
    *CoreL2RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(CoreL2RamObj);
    *CoreL1RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(CoreL1RamObj);
    *L3RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(L3ramObj);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC frame start function registered with DPM. This is invoked on reception
 *      of the frame start ISR from the RF front-end. This API is also invoked
 *      when application issues @ref DPC_RANGING_IOCTL__TRIGGER_FRAME to simulate
 *      a frame trigger (e.g for unit testing purpose).
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void DPC_Ranging_frameStart (DPM_DPCHandle handle)
{
    ObjDetObj     *objDetObj = (ObjDetObj *) handle;

    objDetObj->stats.frameStartTimeStamp = Cycleprofiler_getTimeStamp();

    DebugP_log2("ObjDet DPC: Frame Start, frameIndx = %d, subFrameIndx = %d\n",
                objDetObj->stats.frameStartIntCounter, objDetObj->subFrameIndx);

    /* Check if previous frame (sub-frame) processing has completed */
    // interSubFrameProcToken is decremented when the DSS receives message
    // DPC_RANGING_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED
    // which is sent from the MSS function Ranging_handleObjectDetResult
    // Ranging_handleObjectDetResult is called from MSS Ranging_DPC_reportFxn,
    // after the MSS receives the DPM_Report_NOTIFY_DPC_RESULT function.
    DPC_Objdet_Assert(objDetObj->dpmHandle, (objDetObj->interSubFrameProcToken == 0));
    objDetObj->interSubFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */
    if (objDetObj->subFrameIndx == 0)
    {
        objDetObj->stats.frameStartIntCounter++;
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) start function which is invoked by the
 *      application using DPM_start API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_Ranging_start (DPM_DPCHandle handle)
{
    ObjDetObj   *objDetObj;
    int32_t retVal = 0;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);

    objDetObj->stats.frameStartIntCounter = 0;

    /* Start marks consumption of all pre-start configs, reset the flag to check
     * if pre-starts were issued only after common config was issued for the next
     * time full configuration happens between stop and start */
    objDetObj->isCommonCfgReceived = false;

    /* App must issue export of last frame after stop which will switch to sub-frame 0,
     * so start should always see sub-frame indx of 0, check */
    DebugP_assert(objDetObj->subFrameIndx == 0);

    if(objDetObj->commonCfg.numSubFrames > 1U)
    {
        /* Pre-start cfgs for sub-frames may have come in any order, so need
         * to ensure we reconfig for the current (0) sub-frame before starting */
        DPC_ObjDetDSP_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
    }
    DebugP_log0("ObjDet DPC: Start done\n");
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) stop function which is invoked by the
 *      application using DPM_stop API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_Ranging_stop (DPM_DPCHandle handle)
{
    ObjDetObj   *objDetObj;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);

    /* We can be here only after complete frame processing is done, which means
     * processing token must be 0 and subFrameIndx also 0  */
    DebugP_assert((objDetObj->interSubFrameProcToken == 0) && (objDetObj->subFrameIndx == 0));

    DebugP_log0("ObjDet DPC: Stop done\n");
    return(0);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) execute function which is invoked by the application
 *      in the DPM's execute context when the DPC issues DPM_notifyExecute API from
 *      its registered @ref DPC_Ranging_frameStart API that is invoked every
 *      frame interrupt.
 *
 *  @param[in]  handle       DPM's DPC handle
 *  @param[out]  ptrResult   Pointer to the result
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
#pragma FUNCTION_OPTIONS(DPC_Ranging_execute, "--opt_for_speed")
#pragma CODE_SECTION(DPC_Ranging_execute, ".l1pcode")
int32_t DPC_Ranging_execute
(
    DPM_DPCHandle   handle,
    DPM_Buffer*     ptrResult
)
{
    ObjDetObj   *objDetObj;
    SubFrameObj *subFrmObj;
    rangingDSPObj *rangingObj;
    DPU_RangingDSP_OutParams outRanging;
    DPC_Ranging_ProcessCallBackCfg *processCallBack;
    DPC_Ranging_ExecuteResult *result;
    int32_t retVal;
    uint8_t numChirpsPerChirpEvent;
    int32_t i;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);
    DebugP_assert (ptrResult != NULL);

    DebugP_log1("ObjDet DPC: Processing sub-frame %d\n", objDetObj->subFrameIndx);

    processCallBack = &objDetObj->processCallBackCfg;
    result = &objDetObj->executeResult;

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];
    rangingObj = (rangingDSPObj *)subFrmObj->dpuRangeObj;
    numChirpsPerChirpEvent = subFrmObj->staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent;

    /*==============================================
                   Chirp Processing
     ===============================================*/
    if (((objDetObj->chirpIndex % subFrmObj->staticCfg.numChirpsPerFrame)== 0) &&
       (processCallBack->processFrameBeginCallBackFxn != NULL))
    {
        (*processCallBack->processFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
    }
    retVal = DPU_RangingDSP_process(subFrmObj->dpuRangeObj, &outRanging);
    if (retVal != 0)
    {
        goto exit;
    }

    objDetObj->chirpEndTime = Cycleprofiler_getTimeStamp();

    /* Chirp is processed, increase chirpIndex */
    objDetObj->chirpIndex += numChirpsPerChirpEvent;

    if (outRanging.endOfChirp == true)
    {
        if (processCallBack->processInterFrameBeginCallBackFxn != NULL)
        {
            (*processCallBack->processInterFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
        }
        objDetObj->stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();

        DebugP_log0("ObjDet DPC: Range Proc Done\n");

        /* Set DPM result with measure (bias, phase) and detection info */
        result->subFrameIdx = objDetObj->subFrameIndx;

        /* interChirpProcessingMargin is the smallest margin in the frame */
        objDetObj->stats.interChirpProcessingMargin = objDetObj->chirpMargin / numChirpsPerChirpEvent;

        objDetObj->stats.interFrameEndTimeStamp = Cycleprofiler_getTimeStamp();
        result->stats = &objDetObj->stats;

        result->rangingData = &objDetObj->rangingData;

        // Populate the ranging detection result
        memcpy(&result->rangingData->detectionStats, &outRanging.stats.detectionStats, sizeof(Ranging_PRN_Detection_Stats));
        result->rangingData->processingTime             = outRanging.stats.processingTime;
        result->rangingData->magAdcTime                 = outRanging.stats.magAdcTime;
        result->rangingData->fftTime                    = outRanging.stats.fftTime;
        result->rangingData->vecmulTime                 = outRanging.stats.vecmulTime;
        result->rangingData->ifftTime                   = outRanging.stats.ifftTime;
        result->rangingData->magIfftTime                = outRanging.stats.magIfftTime;

        // Check for rollover


        /* populate DPM_resultBuf - first pointer and size are for results of the processing */
        result->radarCube.data = rangingObj->radarCubebuf;
        result->radarCube.dataSize = rangingObj->DPParams.numAdcSamples * \
                rangingObj->DPParams.numChirpsPerFrame * \
                rangingObj->DPParams.numRxAntennas * sizeof(cmplx16ReIm_t);
        result->radarCube.datafmt = DPIF_RADARCUBE_FORMAT_1;

        ptrResult->ptrBuffer[0] = (uint8_t *)result;
        ptrResult->size[0] = sizeof(DPC_Ranging_ExecuteResult);

        /* clear rest of the result */
        for (i = 1; i < DPM_MAX_BUFFER; i++)
        {
            ptrResult->ptrBuffer[i] = NULL;
            ptrResult->size[i] = 0;
        }

        /* Frame is done, reset chirp index */
        objDetObj->chirpIndex = 0;
    }

exit:
    return retVal;
}



/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *  @pre
 *      MMWave_config (Only in full configuration mode)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

#include <ti/control/mmwave/include/mmwave_internal.h>
#pragma FUNCTION_OPTIONS(MMWave_start_internal, "--opt_for_speed")
#pragma CODE_SECTION(MMWave_start_internal, ".l1pcode")
int32_t MMWave_start_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrCalibrationCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if (ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL)
    {
        /* Full Configuration Mode: Ensure that the application has configured the mmWave module
         * Only then can we start the module. */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_CONFIGURED)   == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Sanity Check: Validate the DFE output mode. This should always match in the FULL configuration mode. */
        if (ptrMMWaveMCB->dfeDataOutputMode != ptrCalibrationCfg->dfeDataOutputMode)
        {
            /* Error: Invalid argument. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }
    }
    else
    {
        /* Minimal Configuration Mode: Application should have opened and synchronized the mmWave module
         * Configuration of the mmWave link is the responsibility of the application using the link API */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U) ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Initialize the DFE Output mode: */
        ptrMMWaveMCB->dfeDataOutputMode = ptrCalibrationCfg->dfeDataOutputMode;
    }

    /* Sanity Check: Ensure that the module has not already been started */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == MMWAVE_STATUS_STARTED)
    {
        /* Error: Invalid usage the module should be stopped before it can be started again. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Copy over the calibration configuration: */
    memcpy ((void*)&ptrMMWaveMCB->calibrationCfg, (const void*)ptrCalibrationCfg, sizeof(MMWave_CalibrationCfg));

    /* SOC specific start: We need to notify the peer domain before the real time starts. */
    retVal = MMWave_deviceStartFxn (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: SOC Start failed; error code is already setup */
        goto exit;
    }

    /* Start the mmWave link: */
    retVal = MMWave_startLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to start the link; error code is already setup */
        goto exit;
    }

    /* The module has been started successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_STARTED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC IOCTL commands configuration API which will be invoked by the
 *      application using DPM_ioctl API
 *
 *  @param[in]  handle   DPM's DPC handle
 *  @param[in]  cmd      Capture DPC specific commands
 *  @param[in]  arg      Command specific arguments
 *  @param[in]  argLen   Length of the arguments which is also command specific
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
#pragma FUNCTION_OPTIONS(DPC_Ranging_ioctl, "--opt_for_speed")
#pragma CODE_SECTION(DPC_Ranging_ioctl, ".l1pcode")
static int32_t DPC_Ranging_ioctl
(
    DPM_DPCHandle       handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
)
{
    ObjDetObj   *objDetObj;
    SubFrameObj *subFrmObj;
    int32_t      retVal = 0;
    int32_t             errCode;

    /* Get the DSS MCB: */
    objDetObj = (ObjDetObj *) handle;
    DebugP_assert(objDetObj != NULL);

    /* Process the commands. Process non sub-frame specific ones first
     * so the sub-frame specific ones can share some code. */
    if((cmd < DPC_RANGING_IOCTL__STATIC_PRE_START_CFG) || (cmd > DPC_RANGING_IOCTL__MAX))
    {
        retVal = DPM_EINVCMD;
    }
    else if (cmd == DPC_RANGING_IOCTL__TRIGGER_FRAME)
    {
        DPC_Ranging_frameStart(handle);
    }
    else if(cmd == DPC_RANGING_IOCTL__TRIGGER_CHIRP)
    {
        DPC_Ranging_chirpEvent(handle);
    }
    else if (cmd == DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
        DPC_Ranging_PreStartCommonCfg *cfg;

        DebugP_assert(argLen == sizeof(DPC_Ranging_PreStartCommonCfg));

        cfg = (DPC_Ranging_PreStartCommonCfg*)arg;

        int32_t indx;

        /* Free all buffers that were allocated from system (MemoryP) heap.
         * Note we cannot free buffers during allocation time
         * for new config during the pre-start config processing because the heap is not capable
         * of defragmentation. This means pre-start common config must precede
         * all pre-start configs. */
        for(indx = 0; indx < objDetObj->commonCfg.numSubFrames; indx++)
        {
            subFrmObj = &objDetObj->subFrameObj[indx];
        }

        objDetObj->commonCfg = *cfg;
        objDetObj->isCommonCfgReceived = true;

        DebugP_log0("ObjDet DPC: Pre-start Common Config IOCTL processed\n");
    }
    else if (cmd == DPC_RANGING_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED)
    {
        DPC_Ranging_ExecuteResultExportedInfo *inp;
        volatile uint32_t startTime;

        startTime = Cycleprofiler_getTimeStamp();

        DebugP_assert(argLen == sizeof(DPC_Ranging_ExecuteResultExportedInfo));

        inp = (DPC_Ranging_ExecuteResultExportedInfo *)arg;

        /* input sub-frame index must match current sub-frame index */
        DebugP_assert(inp->subFrameIdx == objDetObj->subFrameIndx);

        /* Reconfigure all DPUs resources for next sub-frame as EDMA and scrach buffer
         * resources overlap across sub-frames */
        if (objDetObj->commonCfg.numSubFrames > 1)
        {
            /* Next sub-frame */
            objDetObj->subFrameIndx++;
            if (objDetObj->subFrameIndx == objDetObj->commonCfg.numSubFrames)
            {
                objDetObj->subFrameIndx = 0;
            }

            DPC_ObjDetDSP_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
        }
        DebugP_log0("ObjDet DPC: Range Proc Triggered in export IOCTL\n");

        objDetObj->stats.subFramePreparationCycles =
            Cycleprofiler_getTimeStamp() - startTime;

        /* mark end of processing of the frame/sub-frame by the DPC and the app */
        objDetObj->interSubFrameProcToken--;
    }
    else
    {
        DebugP_assert(arg != NULL);
        switch (cmd)
        {
            /* Related to pre-start configuration */
            case DPC_RANGING_IOCTL__STATIC_PRE_START_CFG:
            {
                DPC_Ranging_PreStartCfg *cfg;
                DPC_Ranging_DPC_IOCTL_preStartCfg_memUsage *memUsage;
                MemoryP_Stats statsStart;
                MemoryP_Stats statsEnd;
                uint8_t subFrameNum;

                /* Pre-start common config must be received before pre-start configs
                 * are received. */
                if (objDetObj->isCommonCfgReceived == false)
                {
                    retVal = DPC_RANGING_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;
                    goto exit;
                }

                DebugP_assert(argLen == sizeof(DPC_Ranging_PreStartCfg));

                /* Get system heap size before preStart configuration */
                MemoryP_getStats(&statsStart);

                cfg = (DPC_Ranging_PreStartCfg*)arg;
                subFrameNum = cfg->subFrameNum;
                subFrmObj = &objDetObj->subFrameObj[subFrameNum];

                memUsage = &cfg->memUsage;
                memUsage->L3RamTotal = objDetObj->L3RamObj.cfg.size;
                memUsage->CoreL2RamTotal = objDetObj->CoreL2RamObj.cfg.size;
                memUsage->CoreL1RamTotal = objDetObj->CoreL1RamObj.cfg.size;

                // This series of calls needs to be flattened.
                // Functions need to be renamed to make sense.
                // DPC_ObjDetDSP_preStartConfig --> DPC_ObjDetDSP_rangeConfig -->
                // --> DPU_RangingDSP_config --> rangingDSP_ParseConfig

                // DPC_ObjDetDSP_preStartConfig resets L1, L2, L3 memory pools
                // it then allocates memory for the radar cube

                // DPC_ObjDetDSP_preStartConfig calls DPC_ObjDetDSP_rangeConfig

                // DPC_ObjDetDSP_rangeConfig extracts some parameters from the staticCfg passed by MSS
                // Then DPC_ObjDetDSP_rangeConfig sets up the L1, L2, and L3 pointers and the EDMA
                // Then DPC_ObjDetDSP_rangeConfig calls DPU_RangingDSP_config

                // DPU_RangingDSP_config performs some validation, then calls rangingDSP_ParseConfig
                // rangingDSP_ParseConfig extracts more parameters and fills out rangingDSPObj

                retVal = DPC_ObjDetDSP_preStartConfig(subFrmObj,
                             &objDetObj->commonCfg, 
                             cfg,
                             &objDetObj->edmaHandle[0],
                             &objDetObj->L3RamObj,
                             &objDetObj->CoreL2RamObj,
                             &objDetObj->CoreL1RamObj,
                             &memUsage->L3RamUsage,
                             &memUsage->CoreL2RamUsage,
                             &memUsage->CoreL1RamUsage);
                if (retVal != 0)
                {
                    goto exit;
                }

                /* Get system heap size after preStart configuration */
                MemoryP_getStats(&statsEnd);

                /* Populate system heap usage */
                memUsage->SystemHeapTotal = statsEnd.totalSize;
                memUsage->SystemHeapUsed = statsEnd.totalSize -statsEnd.totalFreeSize;
                memUsage->SystemHeapDPCUsed = statsStart.totalFreeSize - statsEnd.totalFreeSize;

                DebugP_log1("ObjDet DPC: Pre-start Config IOCTL processed (subFrameIndx = %d)\n", subFrameNum);
                break;
            }

            default:
            {
                /* Error: This is an unsupported command */
                retVal = DPM_EINVCMD;
                break;
            }
        }
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) initialization function which is invoked by the
 *      application using DPM_init API. Among other things, this API allocates DPC instance
 *      and DPU instances (by calling DPU's init APIs) from the MemoryP osal
 *      heap. If this API returns an error of any type, the heap is not guaranteed
 *      to be in the same state as before calling the API (i.e any allocations
 *      from the heap while executing the API are not guaranteed to be deallocated
 *      in case of error), so any error from this API should be considered fatal and
 *      if the error is of _ENOMEM type, the application will
 *      have to be built again with a bigger heap size to address the problem.
 *
 *  @param[in]  dpmHandle   DPM's DPC handle
 *  @param[in]  ptrInitCfg  Handle to the framework semaphore
 *  @param[out] errCode     Error code populated on error
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
#pragma FUNCTION_OPTIONS(DPC_Ranging_init, "--opt_for_speed")
#pragma CODE_SECTION(DPC_Ranging_init, ".l1pcode")
static DPM_DPCHandle DPC_Ranging_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
)
{
    ObjDetObj     *objDetObj = NULL;
    DPC_Ranging_InitParams *dpcInitParams;
    int32_t i;

    *errCode = 0;

    if ((ptrInitCfg == NULL) || (ptrInitCfg->arg == NULL))
    {
        *errCode = DPC_RANGING_EINVAL;
        goto exit;
    }

    if (ptrInitCfg->argSize != sizeof(DPC_Ranging_InitParams))
    {
        *errCode = DPC_RANGING_EINVAL__INIT_CFG_ARGSIZE;
        goto exit;
    }

    dpcInitParams = (DPC_Ranging_InitParams *) ptrInitCfg->arg;

    objDetObj = MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);

#ifdef DBG_DPC_OBJDET
    gObjDetObj = objDetObj;
#endif

    DebugP_log1("ObjDet DPC: objDetObj address = %d\n", (uint32_t) objDetObj);

    if(objDetObj == NULL)
    {
        *errCode = DPC_RANGING_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)objDetObj, 0, sizeof(ObjDetObj));

    /* Copy over the DPM configuration: */
    memcpy ((void*)&objDetObj->dpmInitCfg, (void*)ptrInitCfg, sizeof(DPM_InitCfg));

    objDetObj->dpmHandle = dpmHandle;
    objDetObj->socHandle = ptrInitCfg->socHandle;
    objDetObj->L3RamObj.cfg = dpcInitParams->L3ramCfg;
    objDetObj->CoreL2RamObj.cfg = dpcInitParams->CoreL2RamCfg;
    objDetObj->CoreL1RamObj.cfg = dpcInitParams->CoreL1RamCfg;

    for(i = 0; i < EDMA_NUM_CC; i++)
    {
        objDetObj->edmaHandle[i] = dpcInitParams->edmaHandle[i];
    }
    objDetObj->processCallBackCfg = dpcInitParams->processCallBackCfg;

    *errCode = DPC_ObjDetDSP_initDPU(objDetObj, RL_MAX_SUBFRAMES);
exit:
    if(*errCode != 0)
    {
        if(objDetObj != NULL)
        {
            MemoryP_ctrlFree(objDetObj, sizeof(ObjDetObj));
            objDetObj = NULL;
        }
    }

    return ((DPM_DPCHandle)objDetObj);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) de-initialization function which is invoked by the
 *      application using DPM_deinit API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_RANGING__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_Ranging_deinit (DPM_DPCHandle handle)
{
    ObjDetObj *objDetObj = (ObjDetObj *) handle;
    int32_t retVal = 0;

    if (handle == NULL)
    {
        retVal = DPC_RANGING_EINVAL;
        goto exit;
    }

    retVal = DPC_ObjDetDSP_deinitDPU(objDetObj, RL_MAX_SUBFRAMES);

    MemoryP_ctrlFree(handle, sizeof(ObjDetObj));

exit:
    return (retVal);
}

/**************************************************************************
 ************************* Global Declarations ****************************
 **************************************************************************/

/** @addtogroup DPC_RANGING__GLOBAL
 @{ */

/**
 * @brief   Global used to register Object Detection DPC in DPM
 */
DPM_ProcChainCfg gDPC_RangingCfg =
{
    DPC_Ranging_init,            /* Initialization Function:         */
    DPC_Ranging_start,           /* Start Function:                  */
    DPC_Ranging_execute,         /* Execute Function:                */
    DPC_Ranging_ioctl,           /* Configuration Function:          */
    DPC_Ranging_stop,            /* Stop Function:                   */
    DPC_Ranging_deinit,          /* Deinitialization Function:       */
    NULL,                        /* Inject Data Function:            */
    DPC_Ranging_chirpEvent,      /* Chirp Available Function:        */
    DPC_Ranging_frameStart       /* Frame Start Function:            */
};

/* @} */

