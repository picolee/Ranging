/*
 * rangingQueue.c
 *
 *  Created on: Jun 11, 2024
 *      Author: LeeLemay
 */


#include <shared/ranging_queue.h>
#include <stdlib.h>
#include <string.h>

void rangingQueueInit(rangingQueue_t* queue, int max_size) {
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    queue->max_size = max_size;
}

bool rangingQueueIsEmpty(rangingQueue_t* queue) {
    return queue->size == 0;
}

bool rangingQueueIsFull(rangingQueue_t* queue) {
    return queue->size >= queue->max_size;
}

bool rangingQueueEnqueue(rangingQueue_t* queue, Ranging_MSS_DSS_Message* message) {
    if (rangingQueueIsFull(queue))
    {
        return false;
    }

    rangingQueueNode_t* newNode = (rangingQueueNode_t*)malloc(sizeof(rangingQueueNode_t));
    if (!newNode)
    {
        return false;
    }

    newNode->next = NULL;
    memcpy(&newNode->message, message, sizeof(Ranging_MSS_DSS_Message));

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

bool rangingQueueDequeue(rangingQueue_t* queue, Ranging_MSS_DSS_Message* message)
{
    if (rangingQueueIsEmpty(queue))
    {
        return false;
    }

    rangingQueueNode_t* tempNode = queue->head;
    memcpy(message, &tempNode->message, sizeof(Ranging_MSS_DSS_Message));
    queue->head = queue->head->next;
    free(tempNode);
    queue->size--;

    if (queue->size == 0)
    {
        queue->tail = NULL;
    }

    return true;
}
