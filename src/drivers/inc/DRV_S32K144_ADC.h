/*
 * DRV_S32K144_ADC.h
 *
 *  Created on: April 25, 2024
 *      Author: ndhieu131020
 */

#ifndef DRV_S32K144_ADC_H_
#define DRV_S32K144_ADC_H_

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "S32K144.h"
#include "common_typedef.h"
#include <stddef.h>

/*******************************************************************************
 * Definition
 ******************************************************************************/

/* ADC init struct */
typedef struct
{
    uint32_t    ADC_ClockSrc;                   /* Clock source selection, refer to @ADC_ClockSrc */
    uint32_t    ADC_Prescaler;                  /* Select the freq of the clock, refer to @ADC_Prescaler */
    Functional_State   ADC_DMAAccessMode;        /* Configure DMA Access mode */
    uint32_t    ADC_ConversionMode;             /* Select the ADC resolution, refer to @ADC_ConversionMode */
    uint8_t     ADC_SamplingTime;               /* Select the sampling time */
    uint32_t    ADC_TriggMode;                  /* Select the trigger mode, refer to @ADC_TriggMode*/
    Functional_State   ADC_ContinuousConvMode;   /* Configure Continuous conversion mode */
}ADC_InitTypedef;

/** @defgroup ADC_ClockSrc
  * @{
  */
#define ADC_CLKSRC_ALT1    ((uint32_t)0x00000000)
#define ADC_CLKSRC_ALT2    ((uint32_t)0x00000001)
#define ADC_CLKSRC_ALT3    ((uint32_t)0x00000002)
#define ADC_CLKSRC_ALT4    ((uint32_t)0x00000003)

/** @defgroup ADC_Prescaler
  * @{
  */
#define ADC_CLK_DIV1    ((uint32_t)0x00000000)
#define ADC_CLK_DIV2    ((uint32_t)0x00000020)
#define ADC_CLK_DIV4    ((uint32_t)0x00000040)
#define ADC_CLK_DIV8    ((uint32_t)0x00000060)

/** @defgroup ADC_ConversionMode
  * @{
  */
#define ADC_CONV_MODE_8     ((uint32_t)0x00000000)
#define ADC_CONV_MODE_12    ((uint32_t)0x00000004)
#define ADC_CONV_MODE_10    ((uint32_t)0x00000008)

/** @defgroup ADC_TriggMode
  * @{
  */
#define ADC_SOFTWARE_TRIGG    ((uint32_t)0x00000000)
#define ADC_HARDWARE_TRIGG    ((uint32_t)0x00000040)

/** @defgroup ADC_Channel
  * @{
  */
#define ADC_Channel_0     0x00
#define ADC_Channel_1     0x01
#define ADC_Channel_2     0x02
#define ADC_Channel_3     0x03
#define ADC_Channel_4     0x04
#define ADC_Channel_5     0x05
#define ADC_Channel_6     0x06
#define ADC_Channel_7     0x07
#define ADC_Channel_8     0x08
#define ADC_Channel_9     0x09
#define ADC_Channel_10    0x0A
#define ADC_Channel_11    0x0B
#define ADC_Channel_12    0x0C
#define ADC_Channel_13    0x0D
#define ADC_Channel_14    0x0E
#define ADC_Channel_15    0x0F

/** @defgroup ADC conversion complete state
  * @{
  */
#define CONV_COMPLETE     1u
#define CONV_INCOMPLETE   0u

/*******************************************************************************
 * API
 ******************************************************************************/
/**
  * @brief  Initialize ADC Peripheral
  * @param[in]  instance: ADC instance number
  * @param[in]  ADC_InitStructure: pointer to an ADC_InitTypeDef structure that
  *             contains the configuration information for the specified ADC peripheral.
  * @retval None
  */
void DRV_ADC_Init(uint8_t instance, ADC_InitTypedef *ADC_InitStructure);

/**
  * @brief  Sets the input channel and Channel Interrupt Enable as configuration for the measurement channel
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @param[in]  Input_Channel: the Input Channel selected for the Measurement Channel
  * @param[in]  En_Interrupt: the new Channel Interrupt Enable Flag state
  * @retval None
  */
void DRV_ADC_SetInputChannel(uint8_t instance, uint8_t Channel_idx, \
                             uint8_t Input_Channel, \
                             Functional_State En_Interrupt);

/**
  * @brief  This function sets a software trigger channel configuration.
  *         When Software Trigger mode is enabled, configuring control channel index 0,
  *         implicitly triggers a new conversion on the selected ADC input channel.
  * @param[in]  instance: ADC instance number
  * @param[in]  Input_Channel: the Input Channel selected for the Measurement Channel
  * @param[in]  En_Interrupt: the new Channel Interrupt Enable Flag state
  * @retval None
  */
void DRV_ADC_SetSoftTrigChannel(uint8_t instance, uint8_t Input_Channel, Functional_State En_Interrupt);

/**
  * @brief  This function clear conversion flag of the channel
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval ADC value
  */
void DRV_ADC_ClearITFlag(uint8_t instance, uint8_t Channel_idx);

 /**
  * @brief  This function returns the state of the Conversion Complete
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval State of converson (this can be a value of @defgroup ADC conversion complete state)
  */
uint8_t DRV_ADC_GetConvCompleteFlag(uint8_t instance, uint8_t Channel_idx);

/**
  * @brief  This function return the value of Channel result
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval ADC value
  */
uint16_t DRV_ADC_GetChannelResult(uint8_t instance, uint8_t Channel_idx);

/**
  * @brief  This function return the value of Software trigger channel result
  * @param[in]  instance: ADC instance number
  * @retval ADC value
  */
uint16_t DRV_ADC_GetSoftTriggChannelResult(uint8_t instance);

/**
  * @brief  This function sets a software trigger conversion for a channel.
  * @param[in]  ADCx: refer to @ADC_Type
  * @param[in]  Input_Channel: the Input Channel selected for the Measurement Channel
  * @retval None
  */
void DRV_ADC_SoftwareTriggerConversion(uint8_t instance, uint8_t Input_Channel);

/**
  * @brief  This function allow the uper layer to call ISR function.
  * @param[in]  instance: ADC instance number
  * @param[in]  fp: pointer to handler function
  * @retval None
  */
void ADC_RegisterIRQHandlerCallback(uint8_t instance, IRQ_FuncCallback fp);

#endif /* DRV_S32K144_ADC_H_ */
