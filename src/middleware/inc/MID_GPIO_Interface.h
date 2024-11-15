/*
 *  Filename: MID_GPIO_Interface.h
 *
 *  Created on: 11-15-2024
 *      Author: luanduong1209@gmail.com
*/

#ifndef MID_GPIO_INTERFACE_H_
#define MID_GPIO_INTERFACE_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    LED_RED,
    LED_GREEN,
    LED_BLUE,
} led_color_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

void MID_Led_Init(void);

void MID_TurnOnLed(led_color_t ledtype);

void MID_TurnOffLed(led_color_t ledtype);

void MID_ToggleLed(led_color_t ledtype);

#endif /* MID_GPIO_INTERFACE_H_ */
