/*
 * ranging_rfConfig.h
 *
 *  Created on: May 26, 2024
 *      Author: LeeLemay
 */

#ifndef SUBSYS_DSS
#ifndef DSP_CLOCK_MHZ
#define DSP_CLOCK_MHZ 600U
#endif
#endif

#ifndef SHARED_RANGING_RFCONFIG_H_
#define SHARED_RANGING_RFCONFIG_H_

#define  RX_FREQUENCY_GHZ   63.95
#define  TX_FREQUENCY_GHZ   63.9494
#define  SYNC_PRN           0
#define  DEFAULT_PRN        3
#define  GOLD_CODE_NUM_BITS 6

#define RX_SAMPLE_RATE_KSPS     4000
#define RX_NUM_SAMPLES          4096
#define RX_IDLE_TIME_US         3
#define RX_ADC_START_TIME_US    5
// Ramp duration must be greater than (1/(1000*RX_SAMPLE_RATE_KSPS))*RX_NUM_SAMPLES + RX_ADC_START_TIME_US
// (4096 / 4e6) = 0.001024
#define RX_RAMP_DURATION_BUFFER_US  10
#define RX_RAMP_DURATION_US     ( ( 1000.0 * (float)RX_NUM_SAMPLES) / ((float)RX_SAMPLE_RATE_KSPS) + RX_ADC_START_TIME_US + RX_RAMP_DURATION_BUFFER_US )

// These must be one
#define RX_NUM_ANTENNAS         1
#define TX_NUM_ANTENNAS         1


#define TX_SAMPLE_RATE_KSPS     12500
#define TX_NUM_SAMPLES          64
#define TX_IDLE_TIME_US         3
#define TX_ADC_START_TIME_US    0
#define TX_RAMP_DURATION_US     6

// Tests show DSP_PROCESSING_TIME_US of 13500 is necessary - this can certainly be optimized
// The
#define DSP_PROCESSING_TIME_US  13000
#define RX_INTERFRAME_BLANK_TIME_US 450
#define RX_FRAME_PERIOD_MS         ( ( (float) (RX_INTERFRAME_BLANK_TIME_US + RX_RAMP_DURATION_US + RX_IDLE_TIME_US ) ) / 1000.0 )
//#define TX_FRAME_PERIOD_MS         ( ( pow(2,GOLD_CODE_NUM_BITS) - 1 ) * ( ( (float) ( TX_IDLE_TIME_US + TX_RAMP_DURATION_US ) ) / 1000.0) + ( (float) FRAME_BUFFER_TIME_US) / 1000.0 )
#define TX_FRAME_PERIOD_MS 2
#define MICROSECONDS_TO_DSP_CYCLES DSP_CLOCK_MHZ

#define TIME_SLOT_DURATION_DSP_CYCLES   (RX_RAMP_DURATION_US + DSP_PROCESSING_TIME_US) * MICROSECONDS_TO_DSP_CYCLES
#define TX_DELAY_START_DSP_CYCLES       (0 * MICROSECONDS_TO_DSP_CYCLES)

#define RESPONSE_CODE_DELAY_DSP_CYCLES       (TIME_SLOT_DURATION_DSP_CYCLES + 100 * MICROSECONDS_TO_DSP_CYCLES)

// Threshold applied to maximum correlator value to determine if this is a hit
#define     DETECTION_THRESHOLD     1e13f


#endif /* SHARED_RANGING_RFCONFIG_H_ */
