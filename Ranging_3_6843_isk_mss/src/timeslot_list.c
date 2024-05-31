/*
 * timeslot_list.c
 * Circular linked list to hold the time slot schedule
 *
 *  Created on: May 29, 2024
 *      Author: LeeLemay
 */

#include <stdio.h>
#include <stdlib.h>
#include <inc/timeslot_list.h>

void initTimeSlotList(circularLinkedTimeSlotList_t* list)
{
    list->head = NULL;
    list->tail = NULL;
    list->current = NULL;
    list->size = 0;
}

int16_t addTimeSlotToEnd(circularLinkedTimeSlotList_t* list, rangingTimeSlot_t timeslot)
{
    timeslotListNode_t* newNode = (timeslotListNode_t*)malloc(sizeof(timeslotListNode_t));
    if (!newNode)
    {
        return -1;
    }
    newNode->timeslot = timeslot;
    if (list->size == 0)
    {
        list->head = newNode;
        list->tail = newNode;
        list->current = newNode;
        newNode->next = newNode;
    }
    else
    {
        newNode->next = list->head;
        list->tail->next = newNode;
        list->tail = newNode;
    }
    list->size++;

    return 0;
}

int16_t insertTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, rangingTimeSlot_t timeslot, uint16_t index)
{
    uint16_t i;
    if (index > list->size)
    {
        return -1;
    }

    timeslotListNode_t* newNode = (timeslotListNode_t*)malloc(sizeof(timeslotListNode_t));
    if (!newNode)
    {
        return -2;
    }
    newNode->timeslot = timeslot;

    if (index == 0)
    {
        if (list->size == 0)
        {
            list->head = newNode;
            list->tail = newNode;
            list->current = newNode;
            newNode->next = newNode;
        }
        else
        {
            newNode->next = list->head;
            list->head = newNode;
            list->tail->next = newNode;
        }
    }
    else
    {
        timeslotListNode_t* current = list->head;
        for (i = 0; i < index - 1; i++)
        {
            current = current->next;
        }
        newNode->next = current->next;
        current->next = newNode;
        if (index == list->size)
        {
            list->tail = newNode;
        }
    }
    list->size++;

    return 0;
}

rangingTimeSlot_t* getTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, uint16_t index)
{
    uint16_t i;
    if (index >= list->size)
    {
        return NULL;
    }

    timeslotListNode_t* current = list->head;
    for (i = 0; i < index; i++)
    {
        current = current->next;
    }
    return &current->timeslot;
}

rangingTimeSlot_t* getCurrentTimeSlot(circularLinkedTimeSlotList_t* list)
{
    if (list->current == NULL)
    {
        return NULL;
    }
    return &list->current->timeslot;
}

rangingTimeSlot_t* getNextTimeSlot(circularLinkedTimeSlotList_t* list)
{
    if (list->current == NULL)
    {
        return NULL;
    }
    return &list->current->next->timeslot;
}

int16_t incrementCurrentTimeSlot(circularLinkedTimeSlotList_t* list)
{
    if (list->current == NULL)
    {
        return -1;
    }
    list->current = list->current->next;
    return 0;
}

int16_t resetCurrentTimeSlot(circularLinkedTimeSlotList_t* list)
{
    if (list->head == NULL)
    {
        return -1;
    }
    list->current = list->head;
    return 0;
}

int16_t removeTimeSlotAtIndex(circularLinkedTimeSlotList_t* list, uint16_t index)
{
    uint16_t i;
    if (index >= list->size)
    {
        return -1;
    }

    timeslotListNode_t* toDelete;
    if (index == 0)
    {
        toDelete = list->head;
        list->tail->next = list->head->next;
        list->head = list->head->next;
    }
    else
    {
        timeslotListNode_t* current = list->head;
        for (i = 0; i < index - 1; i++)
        {
            current = current->next;
        }
        toDelete = current->next;
        current->next = toDelete->next;
        if (index == list->size - 1)
        {
            list->tail = current;
        }
    }
    free(toDelete);
    list->size--;

    if (list->size == 0)
    {
        list->head = NULL;
        list->tail = NULL;
        list->current = NULL;
    }

    return 0;
}

int16_t removeLastTimeSlot(circularLinkedTimeSlotList_t* list)
{
    return removeTimeSlotAtIndex(list, list->size - 1);
}

int16_t clearTimeSlotList(circularLinkedTimeSlotList_t* list)
{
    while (list->size > 0)
    {
        if (removeTimeSlotAtIndex(list, 0))
        {
            return -1;
        }
    }
    return 0;
}
