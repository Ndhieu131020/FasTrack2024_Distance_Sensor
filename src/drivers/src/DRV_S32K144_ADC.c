/*
 * HAL_S32K144_ADC.c
 *
 *  Created on: April 25, 2024
 *      Author: ndhieu131020
 */
#include "DRV_S32K144_ADC.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CAN instances. */
ADC_Type *g_AdcBase[ADC_INSTANCE_COUNT] = IP_ADC_BASE_PTRS;

/* Array of function pointers for ADC interrupt handlers */
static IRQ_FuncCallback ADC_IRQ_Fp[ADC_INSTANCE_COUNT] = {NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
  * @brief  This function allow the uper layer to call ISR function.
  * @param[in]  instance: ADC instance number
  * @param[in]  fp: pointer to handler function
  * @retval None
  */
void ADC_RegisterIRQHandlerCallback(uint8_t instance, IRQ_FuncCallback fp)
{
    if (instance < ADC_INSTANCE_COUNT)
    {
        ADC_IRQ_Fp[instance] = fp;
    }
    else
    {
        /* Do nothing */
    }
}

/**
  * @brief  ADC0 interrupt request handler function
  * @param  None
  * @retval None
  */
void ADC0_IRQHandler(void)
{
    ADC_IRQ_Fp[0]();
}

/**
  * @brief  ADC1 interrupt request handler function
  * @param  None
  * @retval None
  */
void ADC1_IRQHandler(void)
{
    ADC_IRQ_Fp[1]();
}

/**
  * @brief  Initialize ADC Peripheral
  * @param[in]  instance: ADC instance number
  * @param[in]  ADC_InitStructure: pointer to an ADC_InitTypeDef structure that
  *             contains the configuration information for the specified ADC peripheral.
  * @retval None
  */
void DRV_ADC_Init(uint8_t instance, ADC_InitTypedef *ADC_InitStructure)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    /* Configure ADC Clock source */
    ADCx->CFG1 &= ~ADC_CFG1_ADICLK_MASK; /* Clear ADICLK field */
    ADCx->CFG1 |= ADC_InitStructure->ADC_ClockSrc;

    /* Configure ADC Prescaler */
    ADCx->CFG1 &= ~ADC_CFG1_ADIV_MASK; /* Clear ADIV field */
    ADCx->CFG1 |= ADC_InitStructure->ADC_Prescaler;

    /* Configure ADC DMA */
    if(ADC_InitStructure->ADC_DMAAccessMode == ENABLE)
    {
        ADCx->SC2 |= (1 << ADC_SC2_DMAEN_SHIFT);
    }
    else
    {
        ADCx->SC2 &= ~(1 << ADC_SC2_DMAEN_SHIFT);
    }

    /* Configure ADC Conversion mode */
    ADCx->CFG1 &= ~ADC_CFG1_MODE_MASK; /* Clear MODE field */
    ADCx->CFG1 |= ADC_InitStructure->ADC_ConversionMode;

    /* Configure ADC Sampling time */
    ADCx->CFG2 &= ~ADC_CFG2_SMPLTS_MASK; /* Clear SMPLTS field */
    ADCx->CFG2 |= ADC_CFG2_SMPLTS(ADC_InitStructure->ADC_SamplingTime);

    /* Configure ADC Trigger mode */
    ADCx->SC2 &= ~ADC_SC2_ADTRG_MASK; /* Clear ADTRG field */
    ADCx->SC2 |= ADC_InitStructure->ADC_TriggMode;

    /* Configure ADC continuous conversion mode */
    if(ADC_InitStructure->ADC_ContinuousConvMode == ENABLE)
    {
        ADCx->SC3 |= (1 << ADC_SC3_ADCO_SHIFT);
    }
    else
    {
        ADCx->SC3 &= ~(1 << ADC_SC3_ADCO_SHIFT);
    }
}

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
                             Functional_State En_Interrupt)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    uint32_t regVal = ADCx->SC1[Channel_idx];

    regVal &= ~ADC_SC1_ADCH_MASK;
    regVal |= ADC_SC1_ADCH(Input_Channel);

    if(En_Interrupt == ENABLE)
    {
        regVal |= ADC_SC1_AIEN_MASK;
    }
    else
    {
        regVal &= ~ADC_SC1_AIEN_MASK;
    }

    ADCx->SC1[Channel_idx] = regVal;
}

/**
  * @brief  This function sets a software trigger channel configuration.
  *         When Software Trigger mode is enabled, configuring control channel index 0,
  *         implicitly triggers a new conversion on the selected ADC input channel.
  * @param[in]  instance: ADC instance number
  * @param[in]  Input_Channel: the Input Channel selected for the Measurement Channel
  * @param[in]  En_Interrupt: the new Channel Interrupt Enable Flag state
  * @retval None
  */
void DRV_ADC_SetSoftTrigChannel(uint8_t instance, uint8_t Input_Channel, Functional_State En_Interrupt)
{
//    ADC_Type *ADCx = g_AdcBase[instance];

    DRV_ADC_SetInputChannel(instance, 0, Input_Channel, En_Interrupt);
}

/**
  * @brief  This function clear conversion flag of the channel
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval ADC value
  */
void DRV_ADC_ClearITFlag(uint8_t instance, uint8_t Channel_idx)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    (void)ADCx->R[Channel_idx];
}

/**
  * @brief  This function returns the state of the Conversion Complete
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval State of converson (this can be a value of @defgroup ADC conversion complete state)
  */
uint8_t DRV_ADC_GetConvCompleteFlag(uint8_t instance, uint8_t Channel_idx)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    uint8_t retVal  = 0u;
    uint32_t regVal = ADCx->SC1[Channel_idx];

    retVal = (uint8_t)((regVal & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT);

    return retVal;
}

/**
  * @brief  This function return the value of Channel result
  * @param[in]  instance: ADC instance number
  * @param[in]  Channel_idx: the adc measurement channel index
  * @retval ADC value
  */
uint16_t DRV_ADC_GetChannelResult(uint8_t instance, uint8_t Channel_idx)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    /* Return ADC value */
    return ADCx->R[Channel_idx];
}

/**
  * @brief  This function return the value of Software trigger channel result
  * @param[in]  instance: ADC instance number
  * @retval ADC value
  */
uint16_t DRV_ADC_GetSoftTriggChannelResult(uint8_t instance)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    /* Return ADC value */
    return ADCx->R[0];;
}

/**
  * @brief  This function sets a software trigger conversion for a channel.
  * @param[in]  instance: ADC instance number
  * @param[in]  Input_Channel: the Input Channel selected for the Measurement Channel
  * @retval None
  */
void DRV_ADC_SoftwareTriggerConversion(uint8_t instance, uint8_t Input_Channel)
{
    ADC_Type *ADCx = g_AdcBase[instance];

    uint32_t regVal = ADCx->SC1[0];

    /* Write ADCH field to trigger an ADC Conversion*/
    regVal &= ~ADC_SC1_ADCH_MASK;
    regVal |= ADC_SC1_ADCH(Input_Channel);

    ADCx->SC1[0] = regVal;
}
