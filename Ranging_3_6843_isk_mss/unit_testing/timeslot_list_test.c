/*
 * timeslot_listTests.c
 *
 *  Created on: May 29, 2024
 *      Author: LeeLemay
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inc/timeslot_list.h>

#define MAX_TEST_SLOTS 10

void printResult(const char* testName, int result) {
    if (result == 1) {
        printf("%s: Passed\n", testName);
    } else {
        printf("%s: Failed\n", testName);
    }
}

void testInitList() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    printResult("testInitList", list.size == 0 && list.head == NULL && list.tail == NULL && list.current == NULL);
}

void testAddTimeSlotToEnd() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};

    addTimeSlotToEnd(&list, slot1);
    int result = addTimeSlotToEnd(&list, slot2);

    printResult("testAddTimeSlotToEnd", result == 0 && list.size == 2 && list.head != NULL && list.tail != NULL && list.current == list.head);
}

void testInsertTimeSlotAtIndex() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};

    addTimeSlotToEnd(&list, slot1);
    int result = insertTimeSlotAtIndex(&list, slot2, 0);

    rangingTimeSlot_t* firstSlot = getTimeSlotAtIndex(&list, 0);
    rangingTimeSlot_t* secondSlot = getTimeSlotAtIndex(&list, 1);

    printResult("testInsertTimeSlotAtIndex", result == 0 && list.size == 2 && firstSlot->slotDurationDSPCycles == 200 && secondSlot->slotDurationDSPCycles == 100);
}

void testGetTimeSlotAtIndex() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    rangingTimeSlot_t slot3 = {0, 0, 300, 10, 5, 5, SLOT_TYPE_NO_OP};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);
    addTimeSlotToEnd(&list, slot3);

    rangingTimeSlot_t* retrievedSlot = getTimeSlotAtIndex(&list, 1);

    printResult("testGetTimeSlotAtIndex", retrievedSlot != NULL && retrievedSlot->slotDurationDSPCycles == 200);
}

void testRemoveTimeSlotAtIndex() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    rangingTimeSlot_t slot3 = {0, 0, 300, 10, 5, 5, SLOT_TYPE_NO_OP};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);
    addTimeSlotToEnd(&list, slot3);

    int result = removeTimeSlotAtIndex(&list, 1);
    
    rangingTimeSlot_t* retrievedSlot0 = getTimeSlotAtIndex(&list, 0);
    rangingTimeSlot_t* retrievedSlot1 = getTimeSlotAtIndex(&list, 1);

    printResult("testRemoveTimeSlotAtIndex", result == 0 &&
        retrievedSlot0->slotType == SLOT_TYPE_SYNCHRONIZATION_RX && \
        retrievedSlot1->slotType == SLOT_TYPE_NO_OP && \
        list.size == 2);
}

void testRemoveLastTimeSlot() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);

    int result = removeLastTimeSlot(&list);

    printResult("testRemoveLastTimeSlot", result == 0 && list.size == 1 && list.tail->timeslot.slotDurationDSPCycles == 100);
}

void testClearTimeSlotList() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    for (int i = 0; i < MAX_TEST_SLOTS; i++)
    {
        rangingTimeSlot_t slot = {0, 0, 100 + i, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
        addTimeSlotToEnd(&list, slot);
    }

    int result = clearTimeSlotList(&list);

    printResult("testClearTimeSlotList", result == 0 && list.size == 0 && list.head == NULL && list.tail == NULL && list.current == NULL);
}

void testGetCurrentTimeSlot() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    rangingTimeSlot_t slot3 = {0, 0, 300, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);
    addTimeSlotToEnd(&list, slot3);

    rangingTimeSlot_t* currentSlot = getCurrentTimeSlot(&list);

    printResult("testGetCurrentTimeSlot", currentSlot != NULL && currentSlot->slotDurationDSPCycles == 100);
}

void testGetNextTimeSlot() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    rangingTimeSlot_t slot3 = {0, 0, 300, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);
    addTimeSlotToEnd(&list, slot3);

    rangingTimeSlot_t* nextSlot = getNextTimeSlot(&list);

    printResult("testGetNextTimeSlot", nextSlot != NULL && nextSlot->slotDurationDSPCycles == 200);

    incrementCurrentTimeSlot(&list);
    nextSlot = getNextTimeSlot(&list);

    printResult("testGetNextTimeSlot2", nextSlot != NULL && nextSlot->slotDurationDSPCycles == 300);

    incrementCurrentTimeSlot(&list);
    nextSlot = getNextTimeSlot(&list);

    printResult("testGetNextTimeSlot3", nextSlot != NULL && nextSlot->slotDurationDSPCycles == 100);
}

void testIncrementCurrentTimeSlot() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);

    incrementCurrentTimeSlot(&list);
    rangingTimeSlot_t* currentSlot = getCurrentTimeSlot(&list);

    printResult("testIncrementCurrentTimeSlot", currentSlot != NULL && currentSlot->slotDurationDSPCycles == 200);
}

void testResetCurrentTimeSlot() {
    circularLinkedTimeSlotList_t list;
    initTimeSlotList(&list);

    rangingTimeSlot_t slot1 = {0, 0, 100, 10, 5, 5, SLOT_TYPE_SYNCHRONIZATION_RX};
    rangingTimeSlot_t slot2 = {0, 0, 200, 10, 5, 5, SLOT_TYPE_RANGING_START_CODE_TX};
    addTimeSlotToEnd(&list, slot1);
    addTimeSlotToEnd(&list, slot2);

    incrementCurrentTimeSlot(&list);
    resetCurrentTimeSlot(&list);
    rangingTimeSlot_t* currentSlot = getCurrentTimeSlot(&list);

    printResult("testResetCurrentTimeSlot", currentSlot != NULL && currentSlot->slotDurationDSPCycles == 100);
}

int main() {
    testInitList();
    testAddTimeSlotToEnd();
    testInsertTimeSlotAtIndex();
    testGetTimeSlotAtIndex();
    testRemoveTimeSlotAtIndex();
    testRemoveLastTimeSlot();
    testClearTimeSlotList();
    testGetCurrentTimeSlot();
    testGetNextTimeSlot();
    testIncrementCurrentTimeSlot();
    testResetCurrentTimeSlot();

    return 0;
}
