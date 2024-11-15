/*
 *  Filename: application.c
 *
 *  Created on: 11-04-2024
 *      Author: Ndhieu131020@gmail.com
*/

#include <stdint.h>
#include "MID_Sensor_Interface.h"
#include "MID_Clock_Interface.h"
#include "MID_Notification_Manager.h"
#include "MID_Timer_Interface.h"
#include "MID_CAN_Interface.h"
#include "MID_GPIO_Interface.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define ABS(a,b)    ((a < b) ? (b - a) : (a - b))

#define CHANGE_THRESHOLD    (20U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void App_ReceiveMessageNotification(void);
static void App_BusOffNotification(void);
static void App_TriggerSensor_Notification(void);
static void App_Sensor_Notification(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint16_t Cur_Sensor_Value = 0U;
uint16_t Pre_Sensor_Value = 0U;
uint16_t Delta            = 0U;

Data_Typedef Data_Receive;

/*******************************************************************************
 * Code
 ******************************************************************************/
int main(void)
{
    MID_Clock_Init();
    MID_CAN_Init();
    MID_Sensor_Init();
    MID_Timer_Init();

    MID_Timer_RegisterNotificationCallback(&App_TriggerSensor_Notification);
    MID_ADC_RegisterNotificationCallback(&App_Sensor_Notification);
    MID_CAN_RegisterRxNotificationCallback(&App_ReceiveMessageNotification);
    MID_CAN_RegisterBusOffNotificationCallback(&App_BusOffNotification);

    MID_EnableNotification();

    MID_Timer_StartTimer();

    while(1)
    {
        if(MID_Get_DataSensorState() == READY_TO_READ)
        {
            Cur_Sensor_Value = MID_Read_DistanceValue();
        }

        Delta = ABS(Cur_Sensor_Value, Pre_Sensor_Value);

        if(Delta > CHANGE_THRESHOLD)
        {
            Pre_Sensor_Value = Cur_Sensor_Value;
            MID_CAN_SendCANMessage(TX_DISTANCE_DATA_MB, Cur_Sensor_Value);
        }
    }

    return 0;
}

static void App_ReceiveMessageNotification(void)
{
   if(MID_CheckCommingMessageEvent(RX_CONNECTION_MB) == CAN_MSG_RECEIVED)
   {
        MID_CAN_ReceiveMessage(RX_CONNECTION_MB, &Data_Receive);
        MID_CAN_SendCANMessage(TX_CONFIRM_CONNECTION_MB, TX_MSG_CONFIRM_CONNECTION_DATA);
        MID_ClearMessageCommingEvent(RX_CONNECTION_MB);
        MID_TurnOffLed(LED_RED);
   }

   if(MID_CheckCommingMessageEvent(RX_STOPOPR_MB) == CAN_MSG_RECEIVED)
   {
        MID_CAN_ReceiveMessage(RX_STOPOPR_MB, &Data_Receive);
        MID_ClearMessageCommingEvent(RX_STOPOPR_MB);
   }
}

static void App_BusOffNotification(void)
{
    MID_TurnOnLed(LED_RED);
}

static void App_TriggerSensor_Notification(void)
{
    MID_Set_DataSensorState(BUSY);
    MID_Trigger_ReadProcess();
}

static void App_Sensor_Notification(void)
{
    MID_Set_DataSensorState(READY_TO_READ);
}