/*
 * DRV_S32K144_LPIT.h
 *
 *  Created on: May 6, 2024
 *      Author: ndhieu131020
 */

#ifndef DRV_S32K144_LPIT_H_
#define DRV_S32K144_LPIT_H_

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "S32K144.h"
#include "common_typedef.h"
#include <stddef.h>

/*******************************************************************************
 * Definition
 ******************************************************************************/

/* LPIT mode typedef */
typedef enum
{
    Periodic_Cnt_32b     = 0u,
    Periodic_Cnt_16bDual = 1u,
    Trig_Accumulator_32b = 2u,
    Trig_IC_32b          = 3u
}LPIT_Mode;


/* LPIT channel typedef */
typedef enum
{
    LPIT_CH0 = 0u,
    LPIT_CH1 = 1u,
    LPIT_CH2 = 2u,
    LPIT_CH3 = 3u
}LPIT_ChannelTypedef;

typedef struct
{
    LPIT_Mode LPIT_OperationMode;       /* LPIT mode, refer to @LPIT_ChannelTypedef */
    Functional_State LPIT_ChainChannel; /* LPIT chain channel */
    Functional_State LPIT_Interupt;     /* LPIT interrupt */
}LPIT_InitTypedef;

/*******************************************************************************
 * API
 ******************************************************************************/

/**
  * @brief  Enables the peripheral clock to the module timers.
  * @param[in]  instance: LPIT instance number.
  * @retval None
  */
void DRV_LPIT_EnableModule(uint8_t instance);

/**
  * @brief  Disable the peripheral clock to the module timers.
  * @param[in]  instance: LPIT instance number.
  * @retval None
  */
void DRV_LPIT_DisableModule(uint8_t instance);

/**
  * @brief  Initialize LPIT Peripheral
  * @param[in]  instance: LPIT instance number.
  * @param[in]  CHx: refer to @LPIT_ChannelTypedef
  * @param[in]  LPIT_InitStructure: pointer to an LPIT_InitTypedef structure that
  *             contains the configuration information for the specified LPIT peripheral.
  * @retval None
  */
void DRV_LPIT_Init(uint8_t instance, LPIT_ChannelTypedef CHx, LPIT_InitTypedef *LPIT_InitStructure);

/**
  * @brief  Clears the interrupt flag for a specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_ClearInterruptFlagTimerChannels(uint8_t instance, LPIT_ChannelTypedef CHx);

/**
  * @brief  Retrieves the current timer count value for a specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
uint32_t DRV_LPIT_GetCurrentTimerCount(uint8_t instance, LPIT_ChannelTypedef CHx);

/**
  * @brief  Retrieves the current timer count value for a specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_SetReloadValue(uint8_t instance, LPIT_ChannelTypedef CHx, uint32_t Val);

/**
  * @brief  Starts the specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_StartTimerChannel(uint8_t instance, LPIT_ChannelTypedef CHx);

/**
  * @brief  Stops the specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_StopTimerChannel(uint8_t instance, LPIT_ChannelTypedef CHx);

/**
  * @brief  Registers an interrupt callback function for the specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to register the callback (LPIT0_Channel_0, LPIT0_Channel_1, etc.).
  * @param[in]  fp: The function pointer to the callback function that will handle the interrupt.
  * @retval None
  */
void DRV_LPIT0_RegisterIntCallback(LPIT_ChannelTypedef CHx, IRQ_FuncCallback fp);

#endif /* DRV_S32K144_LPIT_H_ */
