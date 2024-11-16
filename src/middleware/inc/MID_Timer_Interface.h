/*
 *  Filename: MID_Timer_Interface.h
 *
 *  Created on: 11-05-2024
 *      Author: Ndhieu131020@gmail.com
*/

#ifndef MID_TIMER_INTERFACE_H_
#define MID_TIMER_INTERFACE_H_

/*******************************************************************************
 * Include
 ******************************************************************************/

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define LPIT_INSTANCE     0u

/*******************************************************************************
 * API
 ******************************************************************************/

void MID_Timer_Init(void);

void MID_Timer_RegisterNotificationCallback(void (*cb_ptr)(void));

void MID_Timer_StartTimer(void);

void MID_Timer_StopTimer(void);

#endif /* MID_TIMER_INTERFACE_H_ */
