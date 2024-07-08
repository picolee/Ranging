/*
 * rangingQueue.c
 *
 *  Created on: Jun 11, 2024
 *      Author: LeeLemay
 */


#include <shared/ranging_queue.h>
#include <stdlib.h>
#include <string.h>

void rangingQueueInit(rangingQueue_t* queue, int max_size)
{
    uint16_t i;
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    queue->max_size = max_size;

    // Initialize the free list
    queue->freeList = &queue->nodes[0];
    for (i = 0; i < MAX_NODES - 1; i++)
    {
        queue->nodes[i].next = &queue->nodes[i + 1];
    }
    queue->nodes[MAX_NODES - 1].next = NULL;
}

bool rangingQueueIsEmpty(rangingQueue_t* queue) {
    return queue->size == 0;
}

bool rangingQueueIsFull(rangingQueue_t* queue) {
    return queue->size >= queue->max_size;
}

bool rangingQueueEnqueue(rangingQueue_t* queue, Ranging_MSS_DSS_Message_t* message) {
    if (rangingQueueIsFull(queue) || queue->freeList == NULL)
    {
        return false;
    }

    // Allocate a node from the free list
    rangingQueueNode_t* newNode = queue->freeList;
    queue->freeList = queue->freeList->next;

    newNode->next = NULL;
    memcpy(&newNode->message, message, sizeof(Ranging_MSS_DSS_Message_t));

    if (rangingQueueIsEmpty(queue))
    {
        queue->head = newNode;
    }
    else
    {
        queue->tail->next = newNode;
    }
    queue->tail = newNode;
    queue->size++;
    return true;
}

bool rangingQueueDequeue(rangingQueue_t* queue, Ranging_MSS_DSS_Message_t* message)
{
    if (rangingQueueIsEmpty(queue))
    {
        return false;
    }

    rangingQueueNode_t* tempNode = queue->head;
    memcpy(message, &tempNode->message, sizeof(Ranging_MSS_DSS_Message_t));
    queue->head = queue->head->next;

    // Return the node to the free list
    tempNode->next = queue->freeList;
    queue->freeList = tempNode;

    queue->size--;

    if (queue->size == 0)
    {
        queue->tail = NULL;
    }

    return true;
}
