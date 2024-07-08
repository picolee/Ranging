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
volatile uint32_t timerTaskTSCL = 0;
volatile uint32_t timerTaskTSCH = 0;
volatile uint32_t entryPointTSCL = 0;
volatile uint32_t entryPointTSCH = 0;

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
    // 1. Get the time
    timerIsrTSCL = TSCL;
    timerIsrTSCH = TSCH;

    ////////////////////////////////////////////////
    // 2. Stop the timer
    if (precisionTimer != NULL)
    {
        //Clock_stop(precisionClock);
        Timer_stop(precisionTimer);
    }

    ////////////////////////////////////////////////
    // 3. Wake up the high priority startSensorTask to perform precise delay
    Semaphore_post(timerExecutedSemaphore);
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
        timerTaskTSCL = TSCL;
        timerTaskTSCH = TSCH;

        // Calculate the remaining cycles until the target time
        remainingCycles = calculateDelayCycles(timerTaskTSCL, timerTaskTSCH, targetTSCL, targetTSCH);
        while(remainingCycles > 65535)
        {
            startTSCL = TSCL;
            startTSCH = TSCH;
            remainingCycles = calculateDelayCycles(startTSCL, startTSCH, targetTSCL, targetTSCH);
        }

        // Disable interrupts
        // key = HwiP_disable();

        ////////////////////////////////////////////////////////
        // 2. Fine delay
        // This ASM function precisely counts until the target TSCL
        // It handles TSCL rollover as well
        precision_count_until(targetTSCL);

        ////////////////////////////////////////////////
        // 3. Execute time critical function
        /*
         * We will use this when we have SYNC_IN working.
         * Until then, the datapath task will alert the MSS to start the sensor
         * We send the RANGING_NEXT_TIMESLOT_STARTED_EVT to the Task
         * it sends dssReportsTimeslotStart()
         *
        if(startSensor())
        {
            dssReportsFailure();
        }
        else
        {
            dssReportsSensorStart();
        }
        */

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
    uint32_t delayCycles, delayMicroseconds, delayCounts;
    Error_Block eb;
    Timer_Params params;
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


    delayMicroseconds   = delayCycles/DSP_CLOCK_MHZ;

    // Configuration code for Timer 0 to trigger an interrupt after 'delayCycles'

    if (precisionTimer != NULL)
    {
        Timer_delete(&precisionTimer);
    }

    Timer_Params_init(&params);
    params.period = delayMicroseconds;  // Period in clock cycles
    //params.periodType = Timer_PeriodType_COUNTS;
    params.periodType = Timer_PeriodType_MICROSECS;
    params.arg = 0;
    params.runMode = ti_sysbios_interfaces_ITimer_RunMode_ONESHOT;
    params.startMode = ti_sysbios_interfaces_ITimer_StartMode_USER;

    precisionTimer = Timer_create(Timer_ANY, func, &params, &eb);
    if (precisionTimer == NULL)
    {
        System_printf("Failed to create precision timer %s.\n", Error_getMsg(&eb));
        dssReportsFailure();
    }

    /*

    if(precisionTimer == NULL)
    {

        delayMicroseconds   = delayCycles/DSP_CLOCK_MHZ;

        // Configuration code for Timer 0 to trigger an interrupt after 'delayCycles'
        Timer_Params params;
        Timer_Params_init(&params);
        params.period = delayMicroseconds;  // Period in clock cycles
        //params.periodType = Timer_PeriodType_COUNTS;
        params.periodType = Timer_PeriodType_MICROSECS;
        params.arg = 0;
        params.runMode = ti_sysbios_interfaces_ITimer_RunMode_ONESHOT;
        params.startMode = ti_sysbios_interfaces_ITimer_StartMode_USER;

        precisionTimer = Timer_create(Timer_ANY, func, &params, &eb);
        if (precisionTimer == NULL)
        {
            System_printf("Failed to create precision timer %s.\n", Error_getMsg(&eb));
            dssReportsFailure();
        }
//        else
//        {
//            Timer_getFreq(precisionTimer, &fqHz);
//            timerFreqMHz        = fqHz.lo/1000000;
//            delayCounts         = delayMicroseconds*timerFreqMHz;
//            Timer_setPeriod(precisionTimer, delayMicroseconds);
//        }
    }
    else
    {
        Timer_setFunc(precisionTimer, func, 0);
        delayMicroseconds   = delayCycles/DSP_CLOCK_MHZ;
        delayCounts         = delayCycles*timerFreqMHz;
        Timer_setPeriod(precisionTimer, delayMicroseconds);
    }
    */

    Timer_start(precisionTimer);
}

// Function to set the target time
uint16_t setTargetTime(uint32_t inputTSCL, uint32_t inputTSCH)
{
    uint64_t currentCycles, targetCycles;
    uint32_t currentTSCL, currentTSCH;
    currentTSCL = TSCL;
    currentTSCH = TSCH;
    currentCycles = ((uint64_t)currentTSCH << 32) | currentTSCL;
    targetCycles = ((uint64_t)inputTSCH << 32) | inputTSCL;

    if(targetCycles < currentCycles)
    {
        return 1;
    }
    targetTSCL = inputTSCL;
    targetTSCH = inputTSCH;

    return 0;
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
void launchSensorAtTargetTime(uint32_t desiredTSCL, uint32_t desiredTSCH)
{
    entryPointTSCL = TSCL;
    entryPointTSCH = TSCH;
    // Record the target time into variables that are global for this file
    if(setTargetTime(desiredTSCL, desiredTSCH))
    {
        System_printf("Missed target time %u.%u, currently %u.%u.\n",
                      desiredTSCH, desiredTSCL,
                      entryPointTSCH, entryPointTSCL);
        System_printf("Missed target time.\n");
        System_printf("Slot type: %s.\n", slotTypeNames[gMmwDssMCB.nextTimeslot.slotType]);
        System_printf("Times entered: %u.\n", gMmwDssMCB.nextTimeslot.timesEntered);
        dssReportsFailure();
        return;
    }

    // Launch the timer with the calculated delay
    configureAndStartTimer(clockISRSensorStart);


//    Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
}

// Function to configure the timer to trigger at a specific TSCL and TSCH
void msgMssAtTargetTime(uint32_t desiredTSCL, uint32_t desiredTSCH)
{
    entryPointTSCL = TSCL;
    entryPointTSCH = TSCH;
    // Record the target time into variables that are global for this file
    if(setTargetTime(desiredTSCL, desiredTSCH))
    {
        System_printf("Missed target time.\n");
        System_printf("Slot type: %s.\n", slotTypeNames[gMmwDssMCB.nextTimeslot.slotType]);
        System_printf("Times entered: %u.\n", gMmwDssMCB.nextTimeslot.timesEntered);
        dssReportsFailure();
        return;
    }

    // Launch the timer with the calculated delay
    configureAndStartTimer(clockISRMsgMSS);


//    Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
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
        dssReportsFailure();
    }

    Task_Params_init(&taskParams);
    taskParams.priority             = 5;
    taskParams.stackSize            = 4 * 1024;
    (*task)                         = Task_create(startSensorTask, &taskParams, NULL);
}
