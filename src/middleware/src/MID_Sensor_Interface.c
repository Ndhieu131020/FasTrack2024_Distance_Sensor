/*
 *  Filename: MID_Sensor_Interface.c
 *
 *  Created on: 11-04-2024
 *      Author: Ndhieu131020@gmail.com
*/

#include "DRV_S32K144_ADC.h"
#include "DRV_S32K144_PORT.h"
#include "MID_Sensor_Interface.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

#define SENSOR_ADC_SAMPLING_TIME    (10U)
#define DISTANCE_SCALE_FACTOR       (400U)
#define ADC_RESOLUTION              (4095U)
#define ADC_CHANNEL_IDX_0           (0U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void Pin_Init(void);
static void ADC_Init(void);
static uint16_t ConvertToDistance(uint16_t input);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t  Sensor_State = IDLE;
static uint16_t ADC_Value    = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void Pin_Init(void)
{
    virtual_pin_id_t Sensor_Pin = PTC14;

    /* Initialize Sensor pin as Analog Input mode */
    const PortConfig_t  PortConfigADCPin =
    {
        .Mux         =  PORT_PIN_DISABLED,                /* Configure the pin mux mode as analog mode  */
        .Interrupt   =  PORT_INT_DISABLED,                /* Disable interrupt for the pin              */
        .Pull        =  PORT_INTERNAL_PULL_NOT_ENABLED    /* Disable internal pull resistor for the pin */
    };

    DRV_PORT_Init(GET_PORT((uint8_t)Sensor_Pin), GET_PIN_NUM((uint8_t)Sensor_Pin), &PortConfigADCPin);
}

static void ADC_Init(void)
{
    ADC_InitTypedef ADC_InitStructure;

    /* Initialize ADC parameter */
    ADC_InitStructure.ADC_ClockSrc = ADC_CLKSRC_ALT1;
    ADC_InitStructure.ADC_Prescaler = ADC_CLK_DIV1;
    ADC_InitStructure.ADC_DMAAccessMode = DISABLE;
    ADC_InitStructure.ADC_ConversionMode = ADC_CONV_MODE_12;
    ADC_InitStructure.ADC_SamplingTime = SENSOR_ADC_SAMPLING_TIME;
    ADC_InitStructure.ADC_TriggMode = ADC_SOFTWARE_TRIGG;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

    DRV_ADC_Init(SENSOR_ADC, &ADC_InitStructure);

    DRV_ADC_SetSoftTrigChannel(SENSOR_ADC, SENSOR_ADC_CHANNEL, ENABLE);
}

/**
  * @brief  this function initialize ADC Peripheral to read the Sensor
  * @param  None
  * @retval None
  */
void MID_Sensor_Init(void)
{
    Pin_Init();
    ADC_Init();
}

uint8_t MID_Get_DataSensorState(void)
{
    return Sensor_State;
}

void MID_Set_DataSensorState(ReadFlag_Typedef State)
{
    Sensor_State = State;

    if(State == READY_TO_READ)
    {
        DRV_ADC_ClearITFlag(SENSOR_ADC, ADC_CHANNEL_IDX_0);
    }
}

static uint16_t ConvertToDistance(uint16_t input)
{
    return (input * DISTANCE_SCALE_FACTOR) / ADC_RESOLUTION;
}

uint16_t MID_Read_DistanceValue(void)
{
    uint16_t Sensor_Value = 0U;

    ADC_Value = DRV_ADC_GetSoftTriggChannelResult(SENSOR_ADC);

    Sensor_Value = ConvertToDistance(ADC_Value);

    return Sensor_Value;
}

void MID_Trigger_ReadProcess(void)
{
    DRV_ADC_SoftwareTriggerConversion(SENSOR_ADC, SENSOR_ADC_CHANNEL);
}

void MID_ADC_RegisterNotificationCallback(void (*cb_ptr)(void))
{
    ADC_RegisterIRQHandlerCallback(SENSOR_ADC, cb_ptr);
}
