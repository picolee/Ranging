/*
 * rangingQueue.h
 *
 *  Created on: Jun 11, 2024
 *      Author: LeeLemay
 */

#ifndef SHARED_RANGINGQUEUE_H_
#define SHARED_RANGINGQUEUE_H_
#include <stdint.h>
#include <stdbool.h>
#include <shared/ranging_mailbox.h>

typedef struct rangingQueueNode {
    struct rangingQueueNode* next;
    Ranging_MSS_DSS_Message message;
} rangingQueueNode_t;

typedef struct rangingQueue
{
    rangingQueueNode_t* head;
    rangingQueueNode_t* tail;
    int size;
    int max_size;
} rangingQueue_t;

void rangingQueueInit(rangingQueue_t* queue, int max_size);
bool rangingQueueIsEmpty(rangingQueue_t* queue);
bool rangingQueueIsFull(rangingQueue_t* queue);
bool rangingQueueEnqueue(rangingQueue_t* queue, Ranging_MSS_DSS_Message* message);
bool rangingQueueDequeue(rangingQueue_t* queue, Ranging_MSS_DSS_Message* message);


#endif /* SHARED_RANGINGQUEUE_H_ */
