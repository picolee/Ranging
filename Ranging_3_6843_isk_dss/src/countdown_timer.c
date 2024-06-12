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
#include <xdc/runtime/Error.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/timers/rti/Timer.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/common/sys_common_xwr68xx.h>

#include <inc/countdown_timer.h>
#include <inc/dss_mmwave_sensor_interface.h>
#include <inc/ranging_datapath.h>
#include <inc/ranging_dss.h>
#include <shared/ranging_mailbox.h>

#define TIMER_MARGIN 1000
#define CYCLES_PER_MICROSECOND 600

#ifdef SUBSYS_DSS
#pragma SET_CODE_SECTION(".l1pcode")
#endif

extern Ranging_DSS_MCB gMmwDssMCB;

typedef void (*clockISRFunc)(UArg arg);

// Functions from dss_mmwave_functions.c
int16_t startSensorPartOne();
int16_t startSensorPartTwo();

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
void clockISRMsgMSS(UArg arg)
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
    // We post an event at the end of the function.
    // The datapath thread will send an alert to the MSS.

    // Record the execution time
    executionTSCL = TSCL;
    executionTSCH = TSCH;

    ////////////////////////////////////////////////
    // 4. Stop the timer
    if (precisionTimer != NULL)
    {
        //Clock_stop(precisionClock);
        Timer_stop(precisionTimer);
    }

    ////////////////////////////////////////////////
    // 5. Post event
    Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
}

// ISR to handle the timer interrupt
void clockISRSensorStart(UArg arg)
{
    ////////////////////////////////////////////////
    // 1. Wake up the high priority startSensorTask to perform precise delay
    Semaphore_post(timerExecutedSemaphore);

    ////////////////////////////////////////////////
    // 2. Stop the timer
    if (precisionTimer != NULL)
    {
        //Clock_stop(precisionClock);
        Timer_stop(precisionTimer);
    }
}

void startSensorTask(UArg arg0, UArg arg1)
{
    uintptr_t           key;
    uint32_t startTSCL, startTSCH;
    uint32_t remainingCycles;

    while(1)
    {
        Semaphore_pend(timerExecutedSemaphore, BIOS_WAIT_FOREVER);


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

        // Disable interrupts
        //key = HwiP_disable();

        ////////////////////////////////////////////////////////
        // 2. Fine delay
        // This ASM function precisely counts until the target TSCL
        // It handles TSCL rollover as well
        precision_count_until(targetTSCL);

        ////////////////////////////////////////////////
        // 3. Execute time critical function
        //startSensorPartTwo();
        if(startSensor())
        {
            dssReportsFailure();
        }
        else
        {
            dssReportsSensorStart();
        }

        // Record the execution time
        executionTSCL = TSCL;
        executionTSCH = TSCH;

        // Enable interrupts
        //HwiP_restore(key);

        /////////////////////////////////////////////////////////
        // 4. Post Event - The data path thread is waiting for it
        Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
    }
}

// Function to configure and start precisionTimer
static void configureAndStartTimer(clockISRFunc func)
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

        precisionTimer = Timer_create(Timer_ANY, func, &params, &eb);
        Timer_getFreq(precisionTimer, &fqHz);
        timerFreqMHz = fqHz.lo/1000000;
        delayCycles = delayCycles/DSP_CLOCK_MHZ;
        delayCycles = delayCycles*timerFreqMHz;
        Timer_setPeriod(precisionTimer, delayCycles);
        if (precisionTimer == NULL)
        {
            System_printf("Failed to create precision timer %s.\n", Error_getMsg(&eb));
        }
    }
    else
    {
        Timer_setFunc(precisionTimer, func, 0);
        delayCycles = delayCycles/DSP_CLOCK_MHZ;
        delayCycles = delayCycles*timerFreqMHz;
        Timer_setPeriod(precisionTimer, delayCycles);
    }

    Timer_start(precisionTimer);
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
void launchSensorAtTargetTime(uint32_t targetTSCL, uint32_t targetTSCH)
{
    // Perform all of the parts of starting the sensor except the actual starting
    //startSensorPartOne();

    // Record the target time into variables that are global for this file
    setTargetTime(targetTSCL, targetTSCH);

    // Launch the timer with the calculated delay
    configureAndStartTimer(clockISRSensorStart);
}

// Function to configure the timer to trigger at a specific TSCL and TSCH
void msgMssAtTargetTime(uint32_t targetTSCL, uint32_t targetTSCH)
{
    // Record the target time into variables that are global for this file
    setTargetTime(targetTSCL, targetTSCH);

    // Launch the timer with the calculated delay
    configureAndStartTimer(clockISRMsgMSS);
}

// Initialize the semaphore that is used to signal when the timer is completed
void timerInitialization( Task_Handle* task )
{
    Task_Params         taskParams;
    Semaphore_Params semParams;

    Semaphore_Params_init(&semParams);
    timerExecutedSemaphore = Semaphore_create(0, &semParams, NULL);
    if (timerExecutedSemaphore == NULL)
    {
        // Handle error
        System_printf("Failed to create timerExecutedSemaphore.\n");
    }

    Task_Params_init(&taskParams);
    taskParams.priority             = 5;
    taskParams.stackSize            = 4 * 1024;
    (*task)                         = Task_create(startSensorTask, &taskParams, NULL);
}

// We define a custom timer in the mmw_dss.cfg file.
// This is its callback.
//void timerTick(UArg arg)
//{
//    Clock_tick();
//}
