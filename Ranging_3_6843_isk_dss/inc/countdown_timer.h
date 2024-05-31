/*
 * countdown_timer.h
 *
 *  Created on: May 14, 2024
 *      Author: LeeLemay
 */

#ifndef INC_COUNTDOWN_TIMER_H_
#define INC_COUNTDOWN_TIMER_H_

#include <ti/sysbios/knl/Semaphore.h>

// Declare the semaphore handle
extern Semaphore_Handle timerExecutedSemaphore;

// ASM function in precise_countdown.asm
extern void precision_countdown(uint16_t cycles);
// ASM function in precise_count_until.asm
extern void precision_count_until(uint32_t desiredTSCL);

void timerInitialization();
void getTargetTime(uint32_t *outTargetL, uint32_t *outTargetH);
void getExecutionTime(uint32_t *outExecutionL, uint32_t *outExecutionH);
void getTimerIsrTime(uint32_t *outTimerIsrL, uint32_t *outTimerIsrH);
void setTargetTime(uint32_t tscl, uint32_t tsch);
void launchSensorAtTargetTime(uint32_t targetTSCL, uint32_t targetTSCH);
void computeTargetTime(uint32_t startTSCL, uint32_t startTSCH, float deltaTimeSec, uint32_t *endTSCL, uint32_t *endTSCH);

#endif /* INC_COUNTDOWN_TIMER_H_ */
