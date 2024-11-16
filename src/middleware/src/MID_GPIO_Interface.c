/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "DRV_S32K144_PORT.h"
#include "DRV_S32K144_GPIO.h"
#include "MID_GPIO_Interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RED_LED_PIN    (PTD15)
#define GREEN_LED_PIN  (PTD16)
#define BLUE_LED_PIN   (PTD0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/

/* Initializes the specified pin as a LED */
void MID_Led_Init(void)
{
    const PortConfig_t portConfigLed =
    {
        .Mux       = PORT_MUX_AS_GPIO,
        .Interrupt = PORT_INT_DISABLED,
        .Pull      = PORT_INTERNAL_PULL_NOT_ENABLED
    };

    /* Configure LEDs */
    DRV_PORT_Init(GET_PORT((uint8_t)RED_LED_PIN), GET_PIN_NUM((uint8_t)RED_LED_PIN), &portConfigLed);
    DRV_PORT_Init(GET_PORT((uint8_t)GREEN_LED_PIN), GET_PIN_NUM((uint8_t)GREEN_LED_PIN), &portConfigLed);
    DRV_PORT_Init(GET_PORT((uint8_t)BLUE_LED_PIN), GET_PIN_NUM((uint8_t)BLUE_LED_PIN), &portConfigLed);

    /* Set LEDs as GPIO output and turn them off */
    DRV_GPIO_SetPinDirection(GET_PORT((uint8_t)RED_LED_PIN), GET_PIN_NUM((uint8_t)RED_LED_PIN), Output_Direction);
    DRV_GPIO_SetPinDirection(GET_PORT((uint8_t)GREEN_LED_PIN), GET_PIN_NUM((uint8_t)GREEN_LED_PIN), Output_Direction);
    DRV_GPIO_SetPinDirection(GET_PORT((uint8_t)BLUE_LED_PIN), GET_PIN_NUM((uint8_t)BLUE_LED_PIN), Output_Direction);

    DRV_GPIO_SetPins(GET_PORT((uint8_t)RED_LED_PIN), GET_PIN_NUM((uint8_t)RED_LED_PIN));
    DRV_GPIO_SetPins(GET_PORT((uint8_t)GREEN_LED_PIN), GET_PIN_NUM((uint8_t)GREEN_LED_PIN));
    DRV_GPIO_SetPins(GET_PORT((uint8_t)BLUE_LED_PIN), GET_PIN_NUM((uint8_t)BLUE_LED_PIN));
}

/* Turn on the LED pin */
void MID_TurnOnLed(led_color_t ledtype)
{
    virtual_pin_id_t PinID     = RED_LED_PIN;
    uint8_t          PortType  = 0U;
    uint8_t          PinNumber = 0U;

    switch (ledtype)
    {
        case LED_RED:
            PinID = RED_LED_PIN;
            break;
        case LED_GREEN:
            PinID = GREEN_LED_PIN;
            break;
        case LED_BLUE:
            PinID = BLUE_LED_PIN;
            break;
        default:
            break;
    }

    PortType  = GET_PORT((uint8_t)PinID);
    PinNumber = GET_PIN_NUM((uint8_t)PinID);

    DRV_GPIO_ClearPins(PortType, PinNumber);
}

/* Turn off the LED pin */
void MID_TurnOffLed(led_color_t ledtype)
{
    virtual_pin_id_t PinID     = RED_LED_PIN;
    uint8_t          PortType  = 0U;
    uint8_t          PinNumber = 0U;

    switch (ledtype)
    {
        case LED_RED:
            PinID = RED_LED_PIN;
            break;
        case LED_GREEN:
            PinID = GREEN_LED_PIN;
            break;
        case LED_BLUE:
            PinID = BLUE_LED_PIN;
            break;
        default:
            break;
    }

    PortType  = GET_PORT((uint8_t)PinID);
    PinNumber = GET_PIN_NUM((uint8_t)PinID);

    DRV_GPIO_SetPins(PortType, PinNumber);
}

/* Toggle the LED pin */
void MID_ToggleLed(led_color_t ledtype)
{
    virtual_pin_id_t PinID     = RED_LED_PIN;
    uint8_t          PortType  = 0U;
    uint8_t          PinNumber = 0U;

    switch (ledtype)
    {
        case LED_RED:
            PinID = RED_LED_PIN;
            break;
        case LED_GREEN:
            PinID = GREEN_LED_PIN;
            break;
        case LED_BLUE:
            PinID = BLUE_LED_PIN;
            break;
        default:
            break;
    }

    PortType  = GET_PORT((uint8_t)PinID);
    PinNumber = GET_PIN_NUM((uint8_t)PinID);

    DRV_GPIO_TogglePins(PortType, PinNumber);
}

/*******************************************************************************
 * End Of File
 ******************************************************************************/
