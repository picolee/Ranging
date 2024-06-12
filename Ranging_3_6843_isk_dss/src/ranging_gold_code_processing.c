/*
 * ranging_gold_code_processing.c
 *
 *  Created on: Jun 6, 2024
 *      Author: LeeLemay
 */

#include <inc/ranging_dss.h>
#include <shared/ranging_rfConfig.h>
#include <shared/gold_code.h>

/* C64P dsplib (fixed point part for C674X) */
#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/DSP_fft16x16_imre.h>
#include <ti/dsplib/src/DSP_ifft16x32/c64P/DSP_ifft16x32.h>
#include <ti/alg/mmwavelib/mmwavelib.h>     // for mmwavelib_powerAndMax

#pragma SET_CODE_SECTION(".l1pcode")


Ranging_DSS_MCB    gMmwDssMCB;



/**
 *  @b Description
 *  @n
 *      Initializes the gold code and computes the complex conjugate of the FFT
 *
 *  @retval
 *      Not Applicable.
 */
int32_t ranging_setupGoldCode()
{
    gold_code_struct_t  gold_code;
    gold_code_struct_t  sampled_gold_code;
    double              sample_rate      = RX_SAMPLE_RATE_KSPS * 1000;
    double              chip_duration    = ((double)TX_RAMP_DURATION_US)/((double)1000000.0);
    double              zeros_duration   = ((double)TX_IDLE_TIME_US)/((double)1000000.0);
    int16_t             index;
    int16_t             first_non_zero_index = -1;
    int16_t             last_non_zero_index = -1;
    rangingDSPObj_t*    workingVariables = &gMmwDssMCB.dataPathObject.workingVariables;

    ////////////////////////////////////////////////////////////////////////////////
    // Gold code, length 2^N - 1
    gold_code.data = NULL;
    if (generate_one_gold_sequence(
        workingVariables->goldCodeNumBits,
        &gold_code,
        workingVariables->rxPrn))
    {
        if(gold_code.data != NULL)
        {
            free(gold_code.data);
        }
        return -1;
    }

    /////////////////////////////////////////////////////////////////////////////////
    // Upsample the gold code to our sample rate
    // Include idle time periods that occur between chirps
    sampled_gold_code.data = NULL;
    if ( sample_gold_code_with_idle_time_preallocated_memory(
            &sampled_gold_code,
            &gold_code,
            sample_rate,
            chip_duration,
            zeros_duration,
            workingVariables->scratchBufferTwoL2_32kB,
            32*1024))
    {
        if(gold_code.data != NULL)
        {
            free(gold_code.data);
        }
        return -1;
    }

    //////////////////////////////////////////////////////////
    // Transform to complex
    // Pad with zeros out to the length of the number of samples
    // Scale by 100 to compensate for the scaling that occurs in the FFT function
    memset(workingVariables->scratchBufferTwoL2_32kB, 0, RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t));
    for(index = 0; index < sampled_gold_code.length; index++)
    {
        workingVariables->scratchBufferTwoL2_32kB[index].real = sampled_gold_code.data[index]*100;

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
    workingVariables->firstGoldCodeNonZeroIndex = first_non_zero_index;
    workingVariables->lastGoldCodeNonZeroIndex = last_non_zero_index;

    /////////////////////////////////////////////////////////
    // FFT of the gold code
    DSP_fft16x16_imre(
            (int16_t *) workingVariables->fftTwiddle16x16L2_16kB,
            RX_NUM_SAMPLES,
            (int16_t *) workingVariables->scratchBufferTwoL2_32kB,  // Source Address
            (int16_t *) workingVariables->fftGoldCodeL2_16kB );  // Destination Address

    /////////////////////////////////////////////////////////
    // Complex Conjugate
    for(index = 0; index < RX_NUM_SAMPLES; index++)
    {
        workingVariables->fftGoldCodeL2_16kB[index].imag = -1*workingVariables->fftGoldCodeL2_16kB[index].imag;
    }
    memcpy(workingVariables->fftGoldCodeL3_16kB, workingVariables->fftGoldCodeL2_16kB, RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t) );

    /////////////////////////////////////////////////////////
    // Cleanup temporary memory used for gold code generation
    if(gold_code.data != NULL)
    {
        free(gold_code.data);
    }
    return 0;
}


void ranging_calcMagInt16
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
void ranging_dssProcessGoldCode (  )
{
    volatile uint32_t           startTime;
    volatile uint32_t           startTime1;
    volatile uint32_t           stopTime;
    uint16_t                    index;
    int32_t                     int_index_of_max;
    float32_t                   maxpow                      = 0;
    Ranging_PRN_Detection_Stats * detectionStats            = &gMmwDssMCB.dataPathObject.rangingData.detectionStats;
    rangingDSPObj_t             * workingVariables          = &gMmwDssMCB.dataPathObject.workingVariables;
    int16_t                     * L1Buffer16kB              = (int16_t *)&workingVariables->adcDataInL1_16kB[0];
    cmplx16ImRe_t               * scratchPageTwo_L2_16kB    = workingVariables->scratchBufferTwoL2_32kB + RX_NUM_SAMPLES;
    cmplx32ImRe_t               * ifftBuffer                = (cmplx32ImRe_t *) workingVariables->scratchBufferOneL2_32kB;
    float                       * ifftMagnitudeBuffer       = (float *)         workingVariables->scratchBufferTwoL2_32kB;
    cmplx32ImRe_t               * vectorMultiplyBuffer      = (cmplx32ImRe_t *) workingVariables->scratchBufferTwoL2_32kB;

    detectionStats->wasCodeDetected = 0;
    detectionStats->rxPrn = workingVariables->rxPrn;

    ////////////////////////////////////
    // DATA PROCESSING
    ////////////////////////////////////

    startTime = Cycleprofiler_getTimeStamp();

    ////////////////////////////////////
    // 1. Magnitude Calculation
    //      Magnitude is stored in real channel (odd indices)
    //      Calculation performed in place
    //      Very costly (1700us), can be optimized - look at mmwavelib_log2Abs16 for inspiration
    //      Also see mmwavelib_power
    ////////////////////////////////////
    startTime1 = Cycleprofiler_getTimeStamp();
    ranging_calcMagInt16(RX_NUM_SAMPLES, L1Buffer16kB);
    stopTime = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.magAdcTime = stopTime - startTime1;
    memcpy(workingVariables->magnitudeDataL3, L1Buffer16kB, RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t));

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
            (int16_t *) workingVariables->fftTwiddle16x16L2_16kB,     // Twiddle factors
            RX_NUM_SAMPLES,                            // number of complex samples
            (int16_t *) L1Buffer16kB,                           // Input
            (int16_t *) scratchPageTwo_L2_16kB );               // Output
    stopTime = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.fftTime = stopTime - startTime1;
    memcpy(workingVariables->fftOfMagnitudeL3, scratchPageTwo_L2_16kB, RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t));

    ////////////////////////////////////
    // 3. Vector Multiply
    //      Multiplication in the frequency domain is convolution in the time domain, but much more efficient
    //      Inefficient - takes 200us - look at mmwavelib_vecmul16x16 for inspiration (should take ~10us)
    ////////////////////////////////////
    int32_t temp_one;
    int32_t temp_two;
    int32_t temp_three;
    startTime1 = Cycleprofiler_getTimeStamp();
    for(index = 0; index < RX_NUM_SAMPLES; index++)
    {
        // Real portion
        temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
        temp_two = ((int32_t) workingVariables->fftGoldCodeL2_16kB[index].real);
        temp_three = (temp_one * temp_two);
        vectorMultiplyBuffer[index].real = temp_three;

        // Real times imaginary
        temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].real);
        temp_two = ((int32_t) workingVariables->fftGoldCodeL2_16kB[index].imag);
        temp_three = (temp_one * temp_two);
        vectorMultiplyBuffer[index].imag = temp_three;

        temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
        temp_two = ((int32_t) workingVariables->fftGoldCodeL2_16kB[index].real);
        temp_three = (temp_one * temp_two);
        vectorMultiplyBuffer[index].imag += temp_three;

        // Imaginary portion
        temp_one = ((int32_t) scratchPageTwo_L2_16kB[index].imag);
        temp_two = ((int32_t) workingVariables->fftGoldCodeL2_16kB[index].imag);
        temp_three = (temp_one * temp_two);
        vectorMultiplyBuffer[index].real -= temp_three;
    }
    stopTime = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.vecmulTime = stopTime - startTime1;
    memcpy(workingVariables->vectorMultiplyOfFFtedDataL3, vectorMultiplyBuffer, RX_NUM_SAMPLES * sizeof(cmplx32ImRe_t));

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
            (int16_t *) workingVariables->ifftTwiddle16x32L2_16kB,    // Twiddle factors (int16_t)
            RX_NUM_SAMPLES,                            // number of complex samples
            (int32_t *) vectorMultiplyBuffer,                   // Input  (int32_t), scratch buffer two
            (int32_t *) ifftBuffer );                           // Output (int32_t), scratch buffer one
    stopTime = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.ifftTime = stopTime - startTime1;
    memcpy(workingVariables->iFftDataL3, ifftBuffer, RX_NUM_SAMPLES * sizeof(cmplx32ImRe_t) );

    //  The IFFT buffer comes back reversed, so we reverse it here
    for(index = 0; index < RX_NUM_SAMPLES; index++)
    {
        vectorMultiplyBuffer[index] = ifftBuffer[RX_NUM_SAMPLES - 1 - index];
    }


    /////////////////////////////////////////////////////////////////////
    // 5.  Calculate magnitude and find the max
    //      The maximum value of the correlation peak shows where the code started
    /////////////////////////////////////////////////////////////////////
    startTime1 = Cycleprofiler_getTimeStamp();
    int_index_of_max = mmwavelib_powerAndMax((int32_t *) vectorMultiplyBuffer,
                                             RX_NUM_SAMPLES,           // number of complex samples
                                             ifftMagnitudeBuffer,
                                             &maxpow);
    stopTime = Cycleprofiler_getTimeStamp();
    gMmwDssMCB.dataPathObject.rangingData.magIfftTime = stopTime - startTime1;
    memcpy(workingVariables->magIfftDataL3, ifftMagnitudeBuffer, RX_NUM_SAMPLES * sizeof(float) );
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
                float f_index_of_max = (float)(int_index_of_max);                   // range of zero to 4095
                float f_adcSampleRate = (float)(RX_SAMPLE_RATE_KSPS * 1000);        // 4000000
                float f_secondsOffset = f_index_of_max/f_adcSampleRate;             // range of zero to 0.00102375 in steps of 1/adcSampleRate

                // Convert to DSP CPU cycles (600000000 cycles per second (600 MHz))
                detectionStats->coarsePeakTimeOffsetCycles = ((uint32_t)(f_secondsOffset*( (float) DSP_CLOCK_MHZ )*1e6)); // range of zero to 1023750

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
                    float peak_index_fine = ((float) RX_NUM_SAMPLES) - \
                            (detectionStats->rightIntercept - detectionStats->leftIntercept)/(detectionStats->leftSlope - detectionStats->rightSlope);
                    float f_secondsOffsetFine = peak_index_fine/f_adcSampleRate;
                    detectionStats->refinedPeakTimePicoseconds  = (int32_t)((f_secondsOffsetFine - f_secondsOffset)*1e12);
                    float cycles_per_sample = (((float) DSP_CLOCK_MHZ ) * 1e6f ) / f_adcSampleRate;
                    detectionStats->refinedPeakTimeDSPCycles    = (f_index_of_max - peak_index_fine) * cycles_per_sample;
                }
            }
        }
    }

    stopTime = Cycleprofiler_getTimeStamp();

    // Reset the gold code and FFT twiddle - in the future this could be triggered by a framestart interrupt
    memcpy(workingVariables->fftGoldCodeL2_16kB,
           workingVariables->fftGoldCodeL3_16kB,
           RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t) );

    memcpy(workingVariables->fftTwiddle16x16L2_16kB,
           workingVariables->fftTwiddle16x16L3_16kB,
           RX_NUM_SAMPLES * sizeof(cmplx16ImRe_t) );

    ///////////////////////////////////////
    // Data Output
    ///////////////////////////////////////
//        outChannel = rangingObj->dataOutChan[0];
//
//        uint32_t    radarCubeAddr;
//
//        radarCubeAddr = (uint32_t)(rangingObj->radarCubebuf);
//        EDMA_setDestinationAddress(edmaHandle, outChannel,
//            (uint32_t)SOC_translateAddress((radarCubeAddr), SOC_TranslateAddr_Dir_TO_EDMA, NULL));
//
//        if(rangingObj->chirpCount > 1U)
//        {
//            startTime1 = Cycleprofiler_getTimeStamp();
//            rangingDSP_WaitEDMAComplete (  edmaHandle, outChannel);
//            waitingTime += (Cycleprofiler_getTimeStamp() - startTime1);
//        }

    //EDMA_startDmaTransfer(edmaHandle, outChannel);

    /* Update outParams */
    gMmwDssMCB.dataPathObject.rangingData.processingTime = stopTime - startTime;
}
