/*
 *  Filename: MID_ADC_Interface.h
 *
 *  Created on: 11-04-2024
 *      Author: Ndhieu131020@gmail.com
*/

#ifndef MID_SENSOR_INTERFACE_H_
#define MID_SENSOR_INTERFACE_H_

/*******************************************************************************
 * Include
 ******************************************************************************/

/*******************************************************************************
 * Definition
 ******************************************************************************/

/** @defgroup Sensor ADC
  * @{
  */
#define SENSOR_ADC_CHANNEL    ADC_Channel_12
#define SENSOR_ADC            0u

/** @defgroup Sensor Pin
  * @{
  */
#define PORT_SENSOR       IP_PORTC
#define PORT_SENSOR_PIN   PORT_PIN_14

typedef enum
{
    IDLE,
    READY_TO_READ,
    BUSY
}ReadFlag_Typedef;

/*******************************************************************************
 * API
 ******************************************************************************/

/**
  * @brief  this function initialize ADC Peripheral to read the Sensor
  * @param  None
  * @retval None
  */
void MID_Sensor_Init(void);

uint16_t MID_Read_DistanceValue(void);

void MID_Set_DataSensorState(ReadFlag_Typedef State);

uint8_t MID_Get_DataSensorState(void);

void MID_Trigger_ReadProcess(void);

void MID_ADC_RegisterNotificationCallback(void (*cb_ptr)(void));

#endif /* MID_ADC_INTERFACE_H_ */
