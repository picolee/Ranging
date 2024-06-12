/*
 * timeslot.h
 *
 *  Created on: May 26, 2024
 *      Author: LeeLemay
 */

#ifndef SHARED_RANGING_TIMESLOT_H_
#define SHARED_RANGING_TIMESLOT_H_

#include <stdint.h>
#include <inc/ranging_dpc.h>

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
    // Time slot start time in units of the DSP high precision timer, low and high register
    uint32_t                slotStartTSCL;
    uint32_t                slotStartTSCH;

    // Slot duration in DSP high precision timer cycles
    uint32_t                slotDurationDSPCycles;

    // The number of cycles the transmitter delays after the start of the cycle
    uint32_t                transmitDelayAfterSlotStartsDSPCycles;

    // TX start times in units of the DSP high precision timer, low and high register
    uint32_t                txResponseStartTSCL;
    uint32_t                txResponseStartTSCH;

    // The number of cycles the response transmit delays after the start of the cycle
    uint32_t                responseTransmitDelayAfterRxDSPCycles;

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
    uint32_t transmitDelayAfterSlotStartsDSPCycles,
    uint32_t responseTransmitDelayAfterRxDSPCycles,
    float    frequencyInGHz,
    uint16_t prn,
    uint8_t  goldCodeNumBits
);

int32_t initializeDefaultTimeSlot(rangingTimeSlot_t *slot,
                                  rangingTimeSlotType_t slotType,
                                  float frequencyInGHz,
                                  uint16_t prn,
                                  uint8_t goldCodeNumBits);

void computeFirstStartTime( rangingTimeSlot_t *timeSlot,        DPC_Ranging_Data_t  *rangingData );
void computeNextStartTime ( rangingTimeSlot_t *currentTimeSlot, rangingTimeSlot_t   *nextTimeSlot );
void computeTxResponseTime( rangingTimeSlot_t *timeSlot,        DPC_Ranging_Data_t  *rangingData );

#endif /* SHARED_RANGING_TIMESLOT_H_ */
