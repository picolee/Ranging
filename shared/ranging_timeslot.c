/*
 * ranging_timslot.c
 *
 *  Created on: May 27, 2024
 *      Author: LeeLemay
 */

#include <shared/ranging_timeslot.h>
#include <shared/ranging_rfConfig.h>
#include <inc/ranging_dpc.h>  // for DPC_Ranging_Data

#ifndef SUBSYS_DSS
#ifndef DSP_CLOCK_MHZ
#define DSP_CLOCK_MHZ 600
#endif
#endif

// Slot type names table
const char* slotTypeNames[NUMBER_OF_SLOT_TYPES] = {
    "NO_OP",
    "SYNC_RX",
    "SYNC_TX",
    "START_CODE_RX",
    "START_CODE_TX",
    "RESPONSE_CODE_RX",
    "RESPONSE_CODE_TX"
};

// Function to initialize a time slot with specified parameters
int32_t initializeTimeSlot(
    rangingTimeSlot_t *slot,
    rangingTimeSlotType_t slotType,
    uint32_t slotStartTSCL,
    uint32_t slotStartTSCH,
    uint32_t slotDurationDSPCycles,
    uint32_t slotRxStartEarlyByDSPCycles,
    uint32_t slotGuardBeforeDurationDSPCycles,
    uint32_t slotGuardAfterDurationDSPCycles,
    uint16_t frequencyInGHz,
    float prn,
    uint8_t goldCodeNumBits
)
{
    if (slot == NULL)
    {
        return -1;
    }

    slot->slotType = slotType;
    slot->slotStartTSCL = slotStartTSCL;
    slot->slotStartTSCH = slotStartTSCH;
    slot->slotDurationDSPCycles = slotDurationDSPCycles;
    slot->slotRxStartEarlyByDSPCycles = slotRxStartEarlyByDSPCycles;
    slot->slotGuardBeforeDurationDSPCycles = slotGuardBeforeDurationDSPCycles;
    slot->slotGuardAfterDurationDSPCycles = slotGuardAfterDurationDSPCycles;
    slot->frequencyInGHz = frequencyInGHz;
    slot->prn = prn;
    slot->goldCodeNumBits                   = goldCodeNumBits;

    return 0;
}

// Function to initialize a timeslot with default parameters
int32_t initializeDefaultTimeSlot(rangingTimeSlot_t *slot,
                                  rangingTimeSlotType_t slotType,
                                  uint16_t frequencyInGHz,
                                  float prn,
                                  uint8_t goldCodeNumBits)
{
    if (slot == NULL)
    {
        return -1;
    }

    // Defines in ranging_rfConfig.h
    slot->slotType                          = slotType;
    slot->slotStartTSCL                     = 0;
    slot->slotStartTSCH                     = 0;
    slot->slotDurationDSPCycles             = TIME_SLOT_DURATION_DSP_CYCLES;
    slot->slotRxStartEarlyByDSPCycles       = RX_START_EARLY_DSP_CYCLES;
    slot->slotGuardBeforeDurationDSPCycles  = PRE_GUARD_DURATION_DSP_CYCLES;
    slot->slotGuardAfterDurationDSPCycles   = POST_GUARD_DURATION_DSP_CYCLES;
    slot->frequencyInGHz                    = frequencyInGHz;
    slot->prn                               = prn;
    slot->goldCodeNumBits                   = goldCodeNumBits;

    return 0;
}


static void computeFirstTimeSlot(rangingTimeSlot_t currentTimeslot, rangingTimeSlot_t *nextTimeslot, DPC_Ranging_Data    *rangingData)
{
    uint32_t    prnStart;
    int32_t     refinedCyclesOffset;
    prnStart = rangingData->chirpStartTimeLow + rangingData->detectionStats.coarsePeakTimeOffsetCycles;
    refinedCyclesOffset = rangingData->detectionStats.RefinedPeakTimePicoseconds * DSP_CLOCK_MHZ / 1000000;

    // Check for roll over cases
    // 1. Adding coarsePeakTimeOffsetCycles to chirpStartTimeLow causes roll over
    if(prnStart < rangingData->chirpStartTimeLow)
    {
        // Does the refined peak cause it to roll back?
        if(refinedCyclesOffset < 0 && prnStart + refinedCyclesOffset > prnStart)
        {
            // Yes, the refined peak time causes it to roll back
            prnStart += refinedCyclesOffset;
            nextTimeslot->slotStartTSCL = prnStart;
            nextTimeslot->slotStartTSCH = rangingData->chirpStartTimeHigh;
        }
        else
        {
            // No, the refined peak time does not cause it to roll back
            // Yes, the refined peak time causes it to roll back
            prnStart += refinedCyclesOffset;
            nextTimeslot->slotStartTSCL = prnStart;
            nextTimeslot->slotStartTSCH = rangingData->chirpStartTimeHigh + 1;
        }
    }
    // 2. Adding RefinedPeakTimePicoseconds to prnStart causes roll over
    else if(refinedCyclesOffset > 0 && prnStart + refinedCyclesOffset > prnStart)
    {
        // No, the refined peak time does not cause it to roll back
        // Yes, the refined peak time causes it to roll back
        prnStart += refinedCyclesOffset;
        nextTimeslot->slotStartTSCL = prnStart;
        nextTimeslot->slotStartTSCH = rangingData->chirpStartTimeHigh + 1;
    }
    // 3. No roll over
    else
    {
        prnStart += refinedCyclesOffset;
        nextTimeslot->slotStartTSCL = prnStart;
        nextTimeslot->slotStartTSCH = rangingData->chirpStartTimeHigh;
    }
}

void updateTimeSlotTime(rangingTimeSlot_t currentTimeslot, rangingTimeSlot_t *nextTimeslot, DPC_Ranging_Data    *rangingData)
{

    // Case 1: RX Synchronization
    if(currentTimeslot.slotType == SLOT_TYPE_SYNCHRONIZATION_RX)
    {
        if(rangingData->detectionStats.wasCodeDetected)
        {
            computeFirstTimeSlot(currentTimeslot, nextTimeslot, rangingData);
        }
    }

    // Case 2: TX Synchronization - initialize the time base
    else if(currentTimeslot.slotType == SLOT_TYPE_SYNCHRONIZATION_TX)
    {
        nextTimeslot->slotStartTSCL =     rangingData->frameStartTimeLow;
        nextTimeslot->slotStartTSCL +=    currentTimeslot.slotDurationDSPCycles;
        nextTimeslot->slotStartTSCL +=    currentTimeslot.slotGuardAfterDurationDSPCycles;
        nextTimeslot->slotStartTSCL +=    nextTimeslot->slotGuardBeforeDurationDSPCycles;

        // High register
        nextTimeslot->slotStartTSCH =     rangingData->frameStartTimeHigh;

        // Check for rollover
        if( nextTimeslot->slotStartTSCL <  rangingData->frameStartTimeLow)
        {
            nextTimeslot->slotStartTSCH += 1;
        }
    }

    // Case 3: Already synchronized, just propagate
    else
    {
        // Low register
        nextTimeslot->slotStartTSCL =     currentTimeslot.slotStartTSCL;
        nextTimeslot->slotStartTSCL +=    currentTimeslot.slotDurationDSPCycles;
        nextTimeslot->slotStartTSCL +=    currentTimeslot.slotGuardAfterDurationDSPCycles;
        nextTimeslot->slotStartTSCL +=    nextTimeslot->slotGuardBeforeDurationDSPCycles;

        // High register
        nextTimeslot->slotStartTSCH =     currentTimeslot.slotStartTSCH;

        // Check for rollover
        if( nextTimeslot->slotStartTSCL <  currentTimeslot.slotStartTSCL)
        {
            nextTimeslot->slotStartTSCH += 1;
        }
    }
}


