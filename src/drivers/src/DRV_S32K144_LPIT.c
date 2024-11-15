/*
 * DRV_S32K144_LPIT.c
 *
 *  Created on: May 6, 2024
 *      Author: ndhieu131020
 */
#include "DRV_S32K144_LPIT.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

#define LPIT_CHANNEL_COUNT  (4U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for CAN instances. */
LPIT_Type *g_LpitBase[LPIT_INSTANCE_COUNT] = IP_LPIT_BASE_PTRS;

/* Array of function pointers for LPIT interrupt handlers */
static IRQ_FuncCallback LPIT_IRQ_fp[LPIT_CHANNEL_COUNT] = {NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
  * @brief  Enables the peripheral clock to the module timers.
  * @param[in]  instance: LPIT instance number.
  * @retval None
  */
void DRV_LPIT_EnableModule(uint8_t instance)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    LPITx->MCR |= LPIT_MCR_M_CEN_MASK;
}

/**
  * @brief  Disable the peripheral clock to the module timers.
  * @param[in]  instance: LPIT instance number.
  * @retval None
  */
void DRV_LPIT_DisableModule(uint8_t instance)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    LPITx->MCR &= ~LPIT_MCR_M_CEN_MASK;
}

/**
  * @brief  Initialize LPIT Peripheral
  * @param[in]  instance: LPIT instance number.
  * @param[in]  CHx: refer to @LPIT_ChannelTypedef
  * @param[in]  LPIT_InitStructure: pointer to an LPIT_InitTypedef structure that
  *             contains the configuration information for the specified LPIT peripheral.
  * @retval None
  */
void DRV_LPIT_Init(uint8_t instance, LPIT_ChannelTypedef CHx, LPIT_InitTypedef *LPIT_InitStructure)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    /* Clear MODE field */
    LPITx->TMR[CHx].TCTRL &= ~LPIT_TMR_TCTRL_MODE_MASK;
    /* Select MODE */
    LPITx->TMR[CHx].TCTRL |= LPIT_TMR_TCTRL_MODE(LPIT_InitStructure->LPIT_OperationMode);

    /* Configure chain channel */
    if(LPIT_InitStructure->LPIT_ChainChannel == ENABLE)
    {
        LPITx->TMR[CHx].TCTRL |= LPIT_TMR_TCTRL_CHAIN_MASK;
    }
    else
    {
        LPITx->TMR[CHx].TCTRL &= ~LPIT_TMR_TCTRL_CHAIN_MASK;
    }

    /* Configure interrupt */
    if(LPIT_InitStructure->LPIT_Interupt == ENABLE)
    {
        LPITx->MIER |= (1<<CHx);
    }
    else
    {
        LPITx->MIER &= ~(1<<CHx);
    }
}

/**
  * @brief  Clears the interrupt flag for a specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_ClearInterruptFlagTimerChannels(uint8_t instance, LPIT_ChannelTypedef CHx)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    /* Clear interrupt flag */
    LPITx->MSR |= (LPIT_MSR_TIF0_MASK << CHx);
}

/**
  * @brief  Retrieves the current timer count value for a specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
uint32_t DRV_LPIT_GetCurrentTimerCount(uint8_t instance, LPIT_ChannelTypedef CHx)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    return LPITx->TMR[CHx].CVAL;
}

/**
  * @brief  Retrieves the current timer count value for a specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_SetReloadValue(uint8_t instance, LPIT_ChannelTypedef CHx, uint32_t Val)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    /* Configure timer value */
    LPITx->TMR[CHx].TVAL = Val - 1;
}

/**
  * @brief  Starts the specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_StartTimerChannel(uint8_t instance, LPIT_ChannelTypedef CHx)
{
    LPIT_Type *LPITx = g_LpitBase[instance];
    LPITx->SETTEN |= (1<<CHx);
}

/**
  * @brief  Stops the specified LPIT timer channel.
  * @param[in]  CHx: The LPIT channel for which to clear the interrupt flag.
  * @retval None
  */
void DRV_LPIT_StopTimerChannel(uint8_t instance, LPIT_ChannelTypedef CHx)
{
    LPIT_Type *LPITx = g_LpitBase[instance];

    LPITx->CLRTEN |= (1<<CHx);
}

/**
  * @brief  Registers an interrupt callback function for the specified LPIT channel.
  * @param[in]  CHx: The LPIT channel for which to register the callback (LPIT0_Channel_0, LPIT0_Channel_1, etc.).
  * @param[in]  fp: The function pointer to the callback function that will handle the interrupt.
  * @retval None
  */
void DRV_LPIT0_RegisterIntCallback(LPIT_ChannelTypedef CHx, IRQ_FuncCallback fp)
{
    if (CHx < LPIT_CHANNEL_COUNT)
    {
        LPIT_IRQ_fp[CHx] = fp;
    }
    else
    {
        /* Do nothing */
    }
}

/**
  * @brief  LPIT0 Channel 0 interrupt handler function
  * @param  None
  * @retval None
  */
void LPIT0_Ch0_IRQHandler(void)
{
    /* Clear interrupt flag */
    IP_LPIT0->MSR |= (LPIT_MSR_TIF0_MASK << LPIT_CH0);

    LPIT_IRQ_fp[(uint8_t)(LPIT_CH0)]();
}

/**
  * @brief  LPIT0 Channel 1 interrupt handler function
  * @param  None
  * @retval None
  */
void LPIT0_Ch1_IRQHandler(void)
{
    /* Clear interrupt flag */
    IP_LPIT0->MSR |= (LPIT_MSR_TIF0_MASK << LPIT_CH1);

    LPIT_IRQ_fp[(uint8_t)(LPIT_CH1)]();
}

/**
  * @brief  LPIT0 Channel 2 interrupt handler function
  * @param  None
  * @retval None
  */
void LPIT0_Ch2_IRQHandler(void)
{
    /* Clear interrupt flag */
    IP_LPIT0->MSR |= (LPIT_MSR_TIF0_MASK << LPIT_CH2);

    LPIT_IRQ_fp[(uint8_t)(LPIT_CH2)]();
}

/**
  * @brief  LPIT0 Channel 3 interrupt handler function
  * @param  None
  * @retval None
  */
void LPIT0_Ch3_IRQHandler(void)
{
    /* Clear interrupt flag */
    IP_LPIT0->MSR |= (LPIT_MSR_TIF0_MASK << LPIT_CH3);
    
    LPIT_IRQ_fp[(uint8_t)(LPIT_CH3)]();
}
