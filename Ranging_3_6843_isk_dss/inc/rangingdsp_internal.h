/*
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
 *   @file  rangeprocdsp_internal.h
 *
 *   @brief
 *      Implements Data path processing functionality.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROCDSP_INTERNAL_H
#define RANGEPROCDSP_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK include files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>

//#include <ti/datapath/dpu/rangeproc/rangeprocdsp.h>
//#include <ti/datapath/dpu/rangeproc/include/rangeproc_internal.h>
#include <inc/rangingdsp.h>
#include <inc/ranging_internal.h>

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  RangeProcDSP DPU Object
 *
 * @details
 *  The structure is used to hold RangeProcDSP DPU object
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct rangingDSPObj
{
    /*! @brief     Data path common parameters */
    ranging_dpParams        DPParams;

    /*! @brief      The Gold code number of bits */
    uint16_t                goldCodeNumBits;

    /*! @brief      The Gold code sequence number used for receive. There are 2^N+1 Gold codes */
    uint16_t                rxPrn;

    /*! @brief      The Gold code sequence number used for receive. There are 2^N+1 Gold codes */
    uint16_t                txPrn;

    /*! @brief      The index of the first non-zero bit in the gold code. Used for signal and noise calculations */
    uint16_t                firstGoldCodeNonZeroIndex;

    /*! @brief      The index of the last non-zero bit in the gold code. Used for signal and noise calculations */
    uint16_t                lastGoldCodeNonZeroIndex;

    /*! @brief      Chirp counter modulo number of chirps per frame */
    uint16_t                chirpCount;

    /*! @brief      Chirp counter modulo number of chirps per frame */
    uint8_t                 areAllChirpsCompletedForThisFrame;

    /*! @brief     ADC data buffer RX channel offset - fixed for all channels */
    uint16_t                rxChanOffset;

    /*! @brief     Pointer to ADC buffer - this is the only format supported */
    cmplx16ImRe_t           *ADCdataBuf;

    /*! @brief      Points to fftGoldCodeL2_16kB. Forms 32 kB contiguous L2RAM with fftTwiddle16x16L2_16kB and fftGoldCodeL2_16kB
     *              Used to calculate the IFFT of FFT*(Complex Conjugate(FFT(Gold Code)))*/
    cmplx16ImRe_t           *scratchBufferOneL2_32kB;

    /*! @brief      Twiddle table for 1D FFT. Forms 32 kB contiguous L2RAM with the fftGoldCodeL2_16kB */
    cmplx16ImRe_t           *fftTwiddle16x16L2_16kB;

    /*! @brief      FFT of the Gold Code. Forms 32 kB contiguous L2RAM with the fftTwiddle16x16L2_16kB */
    cmplx16ImRe_t           *fftGoldCodeL2_16kB;

    /*! @brief      Twiddle table for 1D IFFT */
    cmplx16ImRe_t           *ifftTwiddle16x32L2_16kB;

    /*! @brief      Used for FFT magnitude calculation, and FFT(magnitude data)*(Complex Conjugate(FFT(Gold Code))) */
    cmplx16ImRe_t           *scratchBufferTwoL2_32kB;

    /*! @brief      ADCBUF input samples in scratch memory */
    cmplx16ImRe_t           *adcDataInL1_16kB;

    /*! @brief      ADC data copied to L3 */
    cmplx16ImRe_t           *ADCDataL3;

    /*! @brief      Magnitude of input samples */
    cmplx16ImRe_t           *magnitudeDataL3;

    /*! @brief      FFT(magnitude data) */
    cmplx16ImRe_t           *fftOfMagnitudeL3;

    /*! @brief      FFT(magnitude data)*(Complex Conjugate(FFT(Gold Code))) */
    cmplx32ImRe_t           *vectorMultiplyOfFFtedDataL3;

    /*! @brief      IFFT of FFT(magnitude data)*(Complex Conjugate(FFT(Gold Code)))*/
    cmplx32ImRe_t           *iFftDataL3;

    /*! @brief      Mag( IFFT ( FFT(magnitude data) * ( Complex Conjugate( FFT(Gold Code))))) */
    uint32_t                *magIfftDataL3;

    /*! @brief      L3 storage for the complex conjugate of the FFT of the gold code */
    cmplx16ImRe_t           *fftGoldCodeL3_16kB;

    /*! @brief      L3 storage for the IFFT twiddle factors - currently unused */
    cmplx16ImRe_t           *ifftTwiddle16x16L3_16kB;

    /*! @brief      L3 storage for the FFT twiddle factors - defined in gen_twiddle_fft16x32.h */
    cmplx16ImRe_t           *fftTwiddle16x16L3_16kB;

    /*! @brief     ranging DPU is in processing state */
    bool                    inProgress;
}rangingDSPObj_t;

#ifdef __cplusplus
}
#endif

#endif
