/*
 * timeslot_list.h
 *
 *  Created on: May 29, 2024
 *      Author: LeeLemay
 */

#ifndef INC_TIMESLOT_LIST_H_
#define INC_TIMESLOT_LIST_H_

#include <shared/ranging_timeslot.h>

typedef struct timeslotListNode
{
    rangingTimeSlot_t timeslot;
    struct timeslotListNode* next;
} timeslotListNode_t;

typedef struct {
    timeslotListNode_t* head;
    timeslotListNode_t* tail;
    timeslotListNode_t* current; // Pointer to track the current timeslot
    int size;
} circularLinkedTimeSlotList_t;

void initTimeSlotList(circularLinkedTimeSlotList_t* list);
int16_t addTimeSlotToEnd(circularLinkedTimeSlotList_t* list, rangingTimeSlot_t timeslot);
int16_t insertTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, rangingTimeSlot_t timeslot, uint16_t index);
rangingTimeSlot_t* getTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, uint16_t index);
rangingTimeSlot_t* getCurrentTimeSlot(circularLinkedTimeSlotList_t* list); // New function to get the current timeslot
rangingTimeSlot_t* getNextTimeSlot(circularLinkedTimeSlotList_t* list); // New function to get the next timeslot
int16_t incrementCurrentTimeSlot(circularLinkedTimeSlotList_t* list); // New function to increment the current timeslot
int16_t resetCurrentTimeSlot(circularLinkedTimeSlotList_t* list); // New function to reset the current timeslot
int16_t removeTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, uint16_t index);
int16_t removeLastTimeSlot(circularLinkedTimeSlotList_t* list);
int16_t clearTimeSlotList(circularLinkedTimeSlotList_t* list);

#endif /* INC_TIMESLOT_LIST_H_ */
