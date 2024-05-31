/*
 * timeslot.h
 *
 *  Created on: May 26, 2024
 *      Author: LeeLemay
 */

#ifndef SHARED_RANGING_TIMESLOT_H_
#define SHARED_RANGING_TIMESLOT_H_

#include <stdint.h>

typedef enum rangingTimeSlotType
{
    SLOT_TYPE_NO_OP = 0,
    SLOT_TYPE_SYNCHRONIZATION_RX,
    SLOT_TYPE_RANGING_START_CODE_RX,
    SLOT_TYPE_RANGING_RESPONSE_CODE_RX,
    SLOT_TYPE_SYNCHRONIZATION_TX,
    SLOT_TYPE_RANGING_START_CODE_TX,
    SLOT_TYPE_RANGING_RESPONSE_CODE_TX,
    NUMBER_OF_SLOT_TYPES
} rangingTimeSlotType_t;

// Slot type names table
extern const char* slotTypeNames[NUMBER_OF_SLOT_TYPES];

typedef struct rangingTimeSlot
{
    // Start times in units of the DSP high precision timer, low and high register
    uint32_t                slotStartTSCL;
    uint32_t                slotStartTSCH;

    // Slot duration in DSP high precision timer cycles
    uint32_t                slotDurationDSPCycles;

    // The amount of DSP cycles that we want to start RX early
    uint32_t                slotRxStartEarlyByDSPCycles;

    // This slot guard is unused buffer time before the slot
    uint32_t                slotGuardBeforeDurationDSPCycles;

    // This slot guard is unused buffer time after the slot
    uint32_t                slotGuardAfterDurationDSPCycles;

    uint8_t                 goldCodeNumBits;            // 2^N + 1 possible PRNs, each of length 2^N-1
    float                   frequencyInGHz;
    uint16_t                prn;                        // 2^N + 1 possibilities
    rangingTimeSlotType_t   slotType;

} rangingTimeSlot_t;

// Define a type alias for a pointer to the struct
typedef struct rangingTimeSlot* rangingTimeSlot_Ptr_t;

// Factory function prototypes
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
);

int32_t initializeDefaultTimeSlot(rangingTimeSlot_t *slot,
                                  rangingTimeSlotType_t slotType,
                                  uint16_t frequencyInGHz,
                                  float prn,
                                  uint8_t goldCodeNumBits);


#endif /* SHARED_RANGING_TIMESLOT_H_ */
