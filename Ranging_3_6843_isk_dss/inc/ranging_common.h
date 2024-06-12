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
 *   @file  ranging_common.h
 *
 *   @brief
 *      Implements Common definition across rangeProc DPU.
 */

/**
 * @mainpage Ranging DPU
 * [TOC]
 * This DPU implements range processing using DSP.
 *
 *  @section dpu_range_intro RangeProc DPU
 *
 *  Description
 *  ----------------
 *
 *  Ranging Unit takes RF data in ADC buffer, computes 1D FFT and saves results in
 * radarCube in the requested format.
 *
 *  Range Processing includes two DPU implementations:
 *
 *   DPU         |  runs on cores
 *  :------------|:----------------
 *  rangingDSP|     DSP
 *
 *
 *  Data Interface
 *  ----------------
 *
 *  Range processing only supports 16bits complex data in ImRe format (@ref DPIF_DATAFORMAT_COMPLEX16_IMRE).\n
 *  Range processing data Input interface is defined by @ref DPIF_ADCBufData_t.\n
 *  Range processing data Output interface is define by @ref DPIF_RadarCube_t.\n
 *  The parameters for radarCube are defined in@ref DPU_RangingDSP_StaticConfig_t.
 *
 *
 *  Ranging DPUs
 *  ---------------------
 *  - Ranging using DSP \ref dpu_rangedsp
 *
 */
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROC_COMMON_H
#define RANGEPROC_COMMON_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
@defgroup DPU_RANGEPROC_EXTERNAL_FUNCTION            ranging DPU External Functions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the exported API which the applications need to
*   invoke in order to use the ranging DPU
*/
/**
@defgroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE      ranging DPU External Data Structures
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the data structures which are exposed to the application
*/
/**
@defgroup DPU_RANGEPROC_ERROR_CODE                   ranging DPU Error Codes
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the error codes which are generated by the sampleProc DPU
*/
/**
@defgroup DPU_RANGEPROC_INTERNAL_FUNCTION            ranging DPU Internal Functions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE      ranging DPU Internal Data Structures
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal data structures which are used internally
*   by the ranging DPU module.
*/
/**
@defgroup DPU_RANGEPROC_INTERNAL_DEFINITION          ranging DPU Internal Definitions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal definitions which are used internally
*   by the ranging DPU.
*/

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of 1D FFT bins in DC range antenna signature compensation */
#define DPU_RANGING_SIGNATURE_COMP_MAX_BIN_SIZE               32
 typedef struct Ranging_PRN_Detection_Stats_t
 {
     /*! @brief  Index of prompt value */
     uint16_t    promptIndex;

     /*! @brief  prompt value */
     float       promptValue;

     /*! @brief  early value */
     float       earlyValue;

     /*! @brief  late value */
     float       lateValue;

     /*! @brief  late value */
     float       leftSlope;

     /*! @brief  late value */
     float       leftIntercept;

     /*! @brief  late value */
     float       rightSlope;

     /*! @brief  late value */
     float       rightIntercept;

     /*! @brief  amount that the early and late indices are offset from the prompt index */
     uint8_t     eplOffset;

     /*! @brief  Time in cycles of the peak time since the first ADC sample  */
     uint32_t    coarsePeakTimeOffsetCycles;

     /*! @brief  plus/minus correction applied to the coarse peak time
      *          Units are picoseconds  */
     int32_t     refinedPeakTimePicoseconds;

     /*! @brief  plus/minus correction applied to the coarse peak time
      *          Units are DSP cycles  */
     int32_t     refinedPeakTimeDSPCycles;

     /*! @brief  boolean, true if code was detected */
     uint8_t     wasCodeDetected;

     /*! @brief  PRN of the code */
     uint16_t    rxPrn;
 } Ranging_PRN_Detection_Stats;

/**
 * @brief
 *  Data processing Unit statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Ranging_stats_t
{
    /*! @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t    processingTime;

    /*! @brief processing time to compute the magnitude of the ADC samples*/
    uint32_t    magAdcTime;

    /*! @brief processing time to compute the fft of the magnitude of the ADC samples*/
    uint32_t    fftTime;

    /*! @brief processing time to multiply the fft with the cmplx cnj of the gold code*/
    uint32_t    vecmulTime;

    /*! @brief processing time to compute the ifft of the vecmul*/
    uint32_t    ifftTime;

    /*! @brief processing time to compute the magnitude of the ifft*/
    uint32_t    magIfftTime;

    // Contains early, prompt, late, offsets, PRN, etc
    Ranging_PRN_Detection_Stats detectionStats;
}DPU_Ranging_stats;

#ifdef __cplusplus
}
#endif

#endif
