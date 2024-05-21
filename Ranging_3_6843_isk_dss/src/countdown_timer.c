/*
 * line_fit.c
 *
 *  Created on: May 14, 2024
 *      Author: LeeLemay
 *
 * Example usage:
            deltaTimeSec = ((float)cycles_to_wait)*0.00000001;
            startTSCL = TSCL;
            startTSCH = TSCH;
            computeTargetTime( startTSCL, startTSCH, deltaTimeSec, &objectiveTSCL, &objectiveTSCH );
            launchTimerForTargetTime( objectiveTSCL, objectiveTSCH );
 */
#include <stdint.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/timers/rti/Timer.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/common/sys_common_xwr68xx.h>
#include <inc/countdown_timer.h>
#include <xdc/runtime/Error.h>

#define TIMER_MARGIN 1000
#define CYCLES_PER_MICROSECOND 600

// Variables to hold the target timestamp values
volatile uint32_t targetTSCL = 0;
volatile uint32_t targetTSCH = 0;
volatile uint32_t executionTSCL = 0;
volatile uint32_t executionTSCH = 0;
volatile uint32_t timerIsrTSCL = 0;
volatile uint32_t timerIsrTSCH = 0;

//static Clock_Handle precisionClock;
static Timer_Handle precisionTimer;
Semaphore_Handle timerExecutedSemaphore;
static uint32_t timerFreqMHz = 0;


// Function to calculate the delay required to reach the target time
static uint32_t calculateDelayCycles(uint32_t currentTSCL, uint32_t currentTSCH, uint32_t targetTSCL, uint32_t targetTSCH)
{
    uint64_t currentCycles, targetCycles;
    currentCycles = ((uint64_t)currentTSCH << 32) | currentTSCL;
    targetCycles = ((uint64_t)targetTSCH << 32) | targetTSCL;
    return targetCycles > currentCycles ? targetCycles - currentCycles : 0;
}

// ISR to handle the timer interrupt
void clockISR(UArg arg)
{
    uint32_t startTSCL, startTSCH;
    uint32_t remainingCycles;


    ////////////////////////////////////////////////
    // 1. Perform coarse delay

    // Read the current time stamp counter values
    timerIsrTSCL = TSCL;
    timerIsrTSCH = TSCH;

    // Calculate the remaining cycles until the target time
    remainingCycles = calculateDelayCycles(timerIsrTSCL, timerIsrTSCH, targetTSCL, targetTSCH);
    while(remainingCycles > 65535)
    {
        startTSCL = TSCL;
        startTSCH = TSCH;
        remainingCycles = calculateDelayCycles(startTSCL, startTSCH, targetTSCL, targetTSCH);
    }

    ////////////////////////////////////////////////
    // 2. Perform precise delay

    // This ASM function precisely counts until the target TSCL
    // It handles TSCL rollover as well
    precision_count_until(targetTSCL);

    ////////////////////////////////////////////////
    // 3. Execute time critical function

    // Record the execution time
    executionTSCL = TSCL;
    executionTSCH = TSCH;

    ////////////////////////////////////////////////
    // 4. Stop the clock
    if (precisionTimer != NULL)
    {
        //Clock_stop(precisionClock);
        Timer_stop(precisionTimer);
    }

    ////////////////////////////////////////////////
    // 5. Post semaphore
    Semaphore_post(timerExecutedSemaphore);
}

// Function to configure and start precisionTimer
static void configureAndStartTimer()
{
    uint32_t startTSCL, startTSCH;
    uint32_t delayCycles;
    Error_Block eb;
    xdc_runtime_Types_FreqHz fqHz;

    // Initialize the error block
    Error_init(&eb);

    ////////////////////////////////////////////////
    // 1. Calculate the delay

    // Read the current time stamp counter values
    startTSCL = TSCL;
    startTSCH = TSCH;

    // Set the timer to go off roughly 1000 cycles before the desired time.
    // From there the ISR will enter assembly language module precise_count_until to count down
    delayCycles = calculateDelayCycles(startTSCL, startTSCH, targetTSCL, targetTSCH);
    delayCycles = delayCycles - TIMER_MARGIN;

    if(precisionTimer == NULL)
    {
        // Configuration code for Timer 0 to trigger an interrupt after 'delayCycles'
        Timer_Params params;
        Timer_Params_init(&params);
        params.period = delayCycles;  // Period in clock cycles
        params.periodType = Timer_PeriodType_COUNTS;
        params.arg = 0;

        precisionTimer = Timer_create(Timer_ANY, clockISR, &params, &eb);
        Timer_getFreq(precisionTimer, &fqHz);
        timerFreqMHz = fqHz.lo/1000000;
        delayCycles = delayCycles/DSP_CLOCK_MHZ;
        delayCycles = delayCycles*timerFreqMHz;
        Timer_setPeriod(precisionTimer, delayCycles);
        System_printf("Freq Hz Hi: %u Lo: %u Total: %u.\n", fqHz.hi, fqHz.lo, ((uint64_t)fqHz.hi << 32) | fqHz.lo);
        if (precisionTimer == NULL)
        {
            System_printf("Failed to create precision timer %s.\n", Error_getMsg(&eb));
        }
    }
    else
    {
        delayCycles = delayCycles/DSP_CLOCK_MHZ;
        delayCycles = delayCycles*timerFreqMHz;
        Timer_setPeriod(precisionTimer, delayCycles);
    }


    Timer_start(precisionTimer);

    // Convert delayCycles to microseconds
//    delayMicroseconds = delayCycles*CYCLES_PER_MICROSECOND;
//
//    ///////////////////////////////////////////////
//    // 2. Configure clock to start immediately and trigger after delay
//    Clock_Params params;
//    Clock_Params_init(&params);
//    params.period = 0;              // This is a one shot clock - period is zero
//    params.arg = 0;
//    params.startFlag = TRUE;        // Start immediately
//
//    // delayCycles sets the one shot trigger time
//    precisionClock = Clock_create(clockISR, delayMicroseconds, &params, NULL);
//    if (precisionClock == NULL)
//    {
//        System_printf("Failed to create precision clock.\n");
//    }
}

// Function to set the target time
void setTargetTime(uint32_t inputTscl, uint32_t inputTsch)
{
    targetTSCL = inputTscl;
    targetTSCH = inputTsch;
}

// Function to get the target time
void getTargetTime(uint32_t *outTargetL, uint32_t *outTargetH)
{
    *outTargetL = targetTSCL;
    *outTargetH = targetTSCH;
}

// Function to get the time that was recorded when the timer ISR finishes its precision delay
void getExecutionTime(uint32_t *outExecutionL, uint32_t *outExecutionH)
{
    *outExecutionL = executionTSCL;
    *outExecutionH = executionTSCH;
}

// Function to get the time that was recorded when the timer ISR finishes its precision delay
void getTimerIsrTime(uint32_t *outTimerIsrL, uint32_t *outTimerIsrH)
{
    *outTimerIsrL = timerIsrTSCL;
    *outTimerIsrH = timerIsrTSCH;
}

// Function to compute the desired TSCH and TSCL values
void computeTargetTime(uint32_t startTSCL, uint32_t startTSCH, float deltaTimeSec, uint32_t *endTSCL, uint32_t *endTSCH)
{
    uint64_t deltaCycles;
    uint64_t currentCycles;
    uint64_t targetCycles;

    // Calculate the number of cycles for the given delta time
    deltaCycles = (uint64_t)(deltaTimeSec * DSP_CLOCK_MHZ * 1e6);  // deltaTimeSec * (DSP_CLOCK_MHZ * 1e6)

    // Combine the startTSCH and startTSCL into a single 64-bit value representing the current cycles
    currentCycles = ((uint64_t)startTSCH << 32) | startTSCL;

    // Calculate the target cycles
    targetCycles = currentCycles + deltaCycles;

    // Split the target cycles into targetTSCH and targetTSCL
    *endTSCH = (uint32_t)(targetCycles >> 32);
    *endTSCL = (uint32_t)(targetCycles & 0xFFFFFFFF);
}

// Function to configure the timer to trigger at a specific TSCL and TSCH
void launchTimerForTargetTime(uint32_t targetTSCL, uint32_t targetTSCH)
{
    // Record the target time into variables that are global for this file
    setTargetTime(targetTSCL, targetTSCH);

    // Launch the timer with the calculated delay
    configureAndStartTimer();
}

// Initialize the semaphore that is used to signal when the timer is completed
void timerInitialization()
{
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    timerExecutedSemaphore = Semaphore_create(0, &semParams, NULL);
    if (timerExecutedSemaphore == NULL)
    {
        // Handle error
        System_printf("Failed to create timerExecutedSemaphore.\n");
    }
}

// We define a custom timer in the mmw_dss.cfg file.
// This is its callback.
//void timerTick(UArg arg)
//{
//    Clock_tick();
//}
