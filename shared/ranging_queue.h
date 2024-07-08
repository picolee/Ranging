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

#define MAX_NODES 10  // Define the maximum number of nodes in the queue

typedef struct rangingQueueNode {
    struct rangingQueueNode* next;
    Ranging_MSS_DSS_Message_t message;
} rangingQueueNode_t;


typedef struct rangingQueue
{
    rangingQueueNode_t* head;
    rangingQueueNode_t* tail;
    int size;
    int max_size;
    rangingQueueNode_t nodes[MAX_NODES]; // Preallocated nodes
    rangingQueueNode_t* freeList;        // List of available nodes
} rangingQueue_t;

void rangingQueueInit(rangingQueue_t* queue, int max_size);
bool rangingQueueIsEmpty(rangingQueue_t* queue);
bool rangingQueueIsFull(rangingQueue_t* queue);
bool rangingQueueEnqueue(rangingQueue_t* queue, Ranging_MSS_DSS_Message_t* message);
bool rangingQueueDequeue(rangingQueue_t* queue, Ranging_MSS_DSS_Message_t* message);


#endif /* SHARED_RANGINGQUEUE_H_ */
