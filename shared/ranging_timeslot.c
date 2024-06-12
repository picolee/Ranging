/*
 * ranging_timslot.c
 *
 *  Created on: May 27, 2024
 *      Author: LeeLemay
 */

#include <shared/ranging_timeslot.h>
#include <shared/ranging_rfConfig.h>
#include <inc/ranging_dpc.h>  // for DPC_Ranging_Data

// Slot type names table
const char* slotTypeNames[NUMBER_OF_SLOT_TYPES] = {
    "NO_OP",
    "SYNC_RX",
    "START_CODE_RX",
    "RESPONSE_CODE_RX",
    "SYNC_TX",
    "START_CODE_TX",
    "RESPONSE_CODE_TX"
};

// Function to initialize a time slot with specified parameters
int32_t initializeTimeSlot(
    rangingTimeSlot_t *slot,
    rangingTimeSlotType_t slotType,
    uint32_t slotStartTSCL,
    uint32_t slotStartTSCH,
    uint32_t slotDurationDSPCycles,
    uint32_t transmitDelayAfterSlotStartsDSPCycles,
    uint32_t responseTransmitDelayAfterSlotStartsDSPCycles,
    float    frequencyInGHz,
    uint16_t prn,
    uint8_t goldCodeNumBits
)
{
    if (slot == NULL)
    {
        return -1;
    }

    slot->slotType                              = slotType;
    slot->slotStartTSCL                         = slotStartTSCL;
    slot->slotStartTSCH                         = slotStartTSCH;
    slot->slotDurationDSPCycles                 = slotDurationDSPCycles;
    slot->transmitDelayAfterSlotStartsDSPCycles = transmitDelayAfterSlotStartsDSPCycles;
    slot->responseTransmitDelayAfterRxDSPCycles = responseTransmitDelayAfterSlotStartsDSPCycles;
    slot->frequencyInGHz                        = frequencyInGHz;
    slot->prn                                   = prn;
    slot->goldCodeNumBits                       = goldCodeNumBits;

    return 0;
}

// Function to initialize a timeslot with default parameters
int32_t initializeDefaultTimeSlot(rangingTimeSlot_t *slot,
                                  rangingTimeSlotType_t slotType,
                                  float frequencyInGHz,
                                  uint16_t prn,
                                  uint8_t goldCodeNumBits)
{
    if (slot == NULL)
    {
        return -1;
    }

    // Defines in ranging_rfConfig.h
    slot->slotType                                      = slotType;
    slot->slotStartTSCL                                 = 0;
    slot->slotStartTSCH                                 = 0;
    slot->slotDurationDSPCycles                         = TIME_SLOT_DURATION_DSP_CYCLES;
    slot->transmitDelayAfterSlotStartsDSPCycles         = TX_DELAY_START_DSP_CYCLES;
    slot->responseTransmitDelayAfterRxDSPCycles         = RESPONSE_CODE_DELAY_DSP_CYCLES;
    slot->frequencyInGHz                                = frequencyInGHz;
    slot->prn                                           = prn;
    slot->goldCodeNumBits                               = goldCodeNumBits;

    return 0;
}

static void computeRxTime(rangingTimeSlot_t *timeSlot, DPC_Ranging_Data_t    *rangingData, uint32_t* startLow, uint32_t* startHigh)
{
    uint32_t    prnStart;
    int32_t     refinedCyclesOffset;

    refinedCyclesOffset = rangingData->detectionStats.refinedPeakTimeDSPCycles;
    prnStart = rangingData->frameStartTimeLow + rangingData->detectionStats.coarsePeakTimeOffsetCycles;

    // Check for roll over cases
    // 1. Adding coarsePeakTimeOffsetCycles to chirpStartTimeLow causes roll over
    if(prnStart < rangingData->frameStartTimeLow)
    {
        // Does the refined peak cause it to roll back?
        if(refinedCyclesOffset < 0 && prnStart + refinedCyclesOffset > prnStart)
        {
            // Yes, the refined peak time causes it to roll back
            prnStart += refinedCyclesOffset;
            *startLow = prnStart;
            *startHigh = rangingData->chirpStartTimeHigh;
        }
        else
        {
            // No, the refined peak time does not cause it to roll back
            prnStart += refinedCyclesOffset;
            *startLow = prnStart;
            *startHigh = rangingData->chirpStartTimeHigh + 1;
        }
    }

    // 2. Adding RefinedPeakTimePicoseconds to prnStart causes roll over
    else if(refinedCyclesOffset > 0 && prnStart + refinedCyclesOffset < prnStart)
    {
        // Yes, the refined peak time causes it to roll over
        prnStart += refinedCyclesOffset;
        *startLow = prnStart;
        *startHigh = rangingData->chirpStartTimeHigh + 1;
    }

    // 3. No roll over
    else
    {
        prnStart += refinedCyclesOffset;
        *startLow = prnStart;
        *startHigh = rangingData->chirpStartTimeHigh;
    }
}

void computeFirstStartTime(rangingTimeSlot_t *timeSlot, DPC_Ranging_Data_t    *rangingData)
{
    // Case 1: RX Synchronization
    if(timeSlot->slotType == SLOT_TYPE_SYNCHRONIZATION_RX)
    {
        computeRxTime(timeSlot, rangingData, &timeSlot->slotStartTSCL, &timeSlot->slotStartTSCH);

        // Adjust for the TX delay
        // Check for roll over
        if( timeSlot->slotStartTSCL - timeSlot->transmitDelayAfterSlotStartsDSPCycles >  rangingData->frameStartTimeLow )
        {
            timeSlot->slotStartTSCH -= 1;
        }
        timeSlot->slotStartTSCL -= timeSlot->transmitDelayAfterSlotStartsDSPCycles;
    }

    // Case 2: TX Synchronization - initialize the time base
    else if(timeSlot->slotType == SLOT_TYPE_SYNCHRONIZATION_TX)
    {
        // Low register
        // TX began at the frame start.
        // However, TX is delayed from the time slot start, so compensate for that.
        timeSlot->slotStartTSCL  =  rangingData->frameStartTimeLow;
        timeSlot->slotStartTSCL -=  timeSlot->transmitDelayAfterSlotStartsDSPCycles;

        // High register
        timeSlot->slotStartTSCH =     rangingData->frameStartTimeHigh;

        // Check for roll over
        if( timeSlot->slotStartTSCL >  rangingData->frameStartTimeLow )
        {
            timeSlot->slotStartTSCH -= 1;
        }
    }
}

void computeNextStartTime(rangingTimeSlot_t *currentTimeSlot, rangingTimeSlot_t *nextTimeSlot)
{
    // Calculate the start time for the next time slot
    // Low register
    nextTimeSlot->slotStartTSCL =     currentTimeSlot->slotStartTSCL;
    nextTimeSlot->slotStartTSCL +=    currentTimeSlot->slotDurationDSPCycles;

    // High register
    nextTimeSlot->slotStartTSCH =     currentTimeSlot->slotStartTSCH;

    // Check for rollover
    if( nextTimeSlot->slotStartTSCL <  currentTimeSlot->slotStartTSCL )
    {
        nextTimeSlot->slotStartTSCH += 1;
    }
}

void computeTxResponseTime(rangingTimeSlot_t *timeSlot, DPC_Ranging_Data_t    *rangingData)
{
    uint32_t    rxStartLow;
    uint32_t    rxStartHigh;

    ////////////////////////////////////////////////////////////////////////////////////
    // 1. Find RX time of the Start Code
    computeRxTime(timeSlot, rangingData, &rxStartLow, &rxStartHigh);

    //////////////////////////////////////////////////////////////////////////////////////
    // 2. Add the delay duration to it
    timeSlot->txResponseStartTSCL = rxStartLow + timeSlot->responseTransmitDelayAfterRxDSPCycles;
    timeSlot->txResponseStartTSCH = rxStartHigh;

    // Check for roll over
    if(timeSlot->txResponseStartTSCL < rxStartLow)
    {
        timeSlot->txResponseStartTSCH += 1;
    }
}
