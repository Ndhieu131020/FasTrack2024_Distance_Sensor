/*
 *  Filename: application.c
 *
 *  Created on: 11-04-2024
 *      Author: Ndhieu131020@gmail.com
*/

#include <stdint.h>
#include <stdbool.h>
#include "MID_Sensor_Interface.h"
#include "MID_Clock_Interface.h"
#include "MID_Notification_Manager.h"
#include "MID_Timer_Interface.h"
#include "MID_CAN_Interface.h"
#include "MID_GPIO_Interface.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

/* Macro to calculate the absolute difference between two values */
#define ABS(a,b)    ((a < b) ? (b - a) : (a - b))

/* Threshold for triggering a CAN message when sensor value changes */
#define CHANGE_THRESHOLD    (20U)

typedef enum
{
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_STOP
} State_Type;

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

/* Structure to save received CAN data */
static Data_Typedef Data_Receive;

/* The current state of the application. */
static volatile State_Type g_current_state = STATE_IDLE;

/* Flag indicating whether this is the first entry into the active state */
static bool g_isActiveEntry = true;

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
  * @brief  Main function: Initializes peripherals, registers callbacks, and processes sensor data
  *                        to send CAN messages when significant changes are detected.
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Initialize system peripherals */
    MID_Clock_Init();
    MID_CAN_Init();
    MID_Sensor_Init();
    MID_Timer_Init();
    MID_Led_Init();

    /* Register notification callbacks */
    MID_Timer_RegisterNotificationCallback(&App_TriggerSensor_Notification);
    MID_ADC_RegisterNotificationCallback(&App_Sensor_Notification);
    MID_CAN_RegisterRxNotificationCallback(&App_ReceiveMessageNotification);

    /* Enable notifications and start periodic timer */
    MID_EnableNotification();

    while(1)
    {
        switch (g_current_state)
        {
            case STATE_ACTIVE:
                if (g_isActiveEntry == true)
                {
                    MID_Timer_StartTimer();
                    MID_TurnOffLed(LED_RED);
                    g_isActiveEntry = false;
                }

                if(MID_Get_DataSensorState() == READY_TO_READ)
                {
                    Cur_Sensor_Value = MID_Read_DistanceValue();
                    MID_Set_DataSensorState(IDLE);
                }

                Delta = ABS(Cur_Sensor_Value, Pre_Sensor_Value);

                if(Delta > CHANGE_THRESHOLD)
                {
                    Pre_Sensor_Value = Cur_Sensor_Value;
                    MID_CAN_SendCANMessage(TX_DISTANCE_DATA_MB, Cur_Sensor_Value);
                }
                break;

            case STATE_STOP:
                MID_Timer_StopTimer();
                MID_TurnOnLed(LED_RED);
                g_isActiveEntry = true;
                break;

            default:
                break;
        }
    }

    return 0;
}

/**
  * @brief Callback for handling received CAN messages.
  *        Processes messages from specific mailboxes and sends acknowledgment.
  * @param  None
  * @retval None
  */
static void App_ReceiveMessageNotification(void)
{
    if (MID_CheckCommingMessageEvent(RX_CONNECTION_MB) == CAN_MSG_RECEIVED)
    {
          MID_CAN_ReceiveMessage(RX_CONNECTION_MB, &Data_Receive);
          MID_CAN_SendCANMessage(TX_CONFIRM_CONNECTION_MB, TX_MSG_CONFIRM_CONNECTION_DATA);
          MID_ClearMessageCommingEvent(RX_CONNECTION_MB);
          g_current_state = STATE_ACTIVE;
    }

    if (MID_CheckCommingMessageEvent(RX_STOPOPR_MB) == CAN_MSG_RECEIVED)
    {
          MID_CAN_ReceiveMessage(RX_STOPOPR_MB, &Data_Receive);
          MID_CAN_SendCANMessage(TX_CONFIRM_STOPOPR_MB, TX_MSG_CONFIRM_CONNECTION_DATA);

          if (Data_Receive.Data == RX_MSG_STOP_OPR_DATA)
          {
              g_current_state = STATE_STOP;
          }
          else if (Data_Receive.Data == RX_MSG_RESUME_OPR_DATA)
          {
              g_current_state = STATE_ACTIVE;
          }
          else
          {
              /* Do nothing */
          }

          MID_ClearMessageCommingEvent(RX_STOPOPR_MB);
    }

    if (MID_CheckCommingMessageEvent(RX_CONFIRM_DATA_MB) == CAN_MSG_RECEIVED)
    {
          MID_ClearMessageCommingEvent(RX_CONFIRM_DATA_MB);
    }

    if (MID_CheckCommingMessageEvent(RX_PING_MSG_MB) == CAN_MSG_RECEIVED)
    {
          MID_CAN_ReceiveMessage(RX_PING_MSG_MB, &Data_Receive);
          MID_CAN_SendCANMessage(TX_CONFIRM_PING_MB, TX_MSG_CONFIRM_CONNECTION_DATA);
          MID_ClearMessageCommingEvent(RX_PING_MSG_MB);
    }
}

/**
  * @brief Callback triggered by the timer to initiate sensor read process.
  *        Sets the sensor state to BUSY and starts the read process.
  * @param  None
  * @retval None
  */
static void App_TriggerSensor_Notification(void)
{
    MID_Set_DataSensorState(BUSY);
    MID_Trigger_ReadProcess();
}

/**
  * @brief Callback to signal that sensor data acquisition is complete.
  *        Sets the sensor state to READY_TO_READ.
  * @param  None
  * @retval None
  */
static void App_Sensor_Notification(void)
{
    MID_Set_DataSensorState(READY_TO_READ);
}
