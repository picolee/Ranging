/*
 * ranging_datapath.h
 *
 *  Created on: Jun 3, 2024
 *      Author: LeeLemay
 */

#ifndef INC_RANGING_DATAPATH_H_
#define INC_RANGING_DATAPATH_H_


/*! @brief   mmwave Config event triggered from mmwave config call back function */
#define RANGING_CONFIG_EVT Event_Id_00

/*! @brief   mmwave Start event triggered from mmwave start call back function */
#define RANGING_START_EVT Event_Id_01

/*! @brief   mmwave Stop event triggered from mmwave stop call back function */
#define RANGING_STOP_EVT Event_Id_02

/*! @brief   Frame start interupt triggered event */
#define RANGING_FRAMESTART_EVT Event_Id_03

/*! @brief   Chirp available interrupt triggered event */
#define RANGING_CHIRP_EVT Event_Id_04

/*! @brief   BSS frame trigger ready event */
#define RANGING_BSS_FRAME_TRIGGER_READY_EVT Event_Id_05

/*! @brief   Next timeslot started event */
#define RANGING_NEXT_TIMESLOT_STARTED_EVT Event_Id_06



#endif /* INC_RANGING_DATAPATH_H_ */
