/*
 * DRV_S32K144_NVIC.h
 *
 *  Created on: April 15, 2024
 *      Author: ndhieu131020
 */

#ifndef DRV_S32K144_NVIC_H_
#define DRV_S32K144_NVIC_H_

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "S32K144.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define NVIC_ISER_COUNT    8u
#define NVIC_ICER_COUNT    8u
#define NVIC_ISPR_COUNT    8u
#define NVIC_ICPR_COUNT    8u
#define NVIC_IABR_COUNT    8u
#define NVIC_IP_COUNT    240u

typedef struct {
    __IO uint32_t ISER[NVIC_ISER_COUNT];    /* Interrupt Set Enable Register n */
    uint8_t RESERVED_0[96];
    __IO uint32_t ICER[NVIC_ICER_COUNT];    /* Interrupt Clear Enable Register n */
    uint8_t RESERVED_1[96];
    __IO uint32_t ISPR[NVIC_ISPR_COUNT];    /* Interrupt Set Pending Register n */
    uint8_t RESERVED_2[96];
    __IO uint32_t ICPR[NVIC_ICPR_COUNT];    /* Interrupt Clear Pending Register n */
    uint8_t RESERVED_3[96];
    __IO uint32_t IABR[NVIC_IABR_COUNT];    /* Interrupt Active bit Register n */
    uint8_t RESERVED_4[224];
    __IO uint8_t IP[NVIC_IP_COUNT];         /* Interrupt Priority Register n */
    uint8_t RESERVED_5[2576];
    __IO  uint32_t STIR;                    /* Software Trigger Interrupt Register */
} NVIC_Typedef;

#define NVIC_BASE    ((uint32_t)0xE000E100)
#define NVIC         ((NVIC_Typedef *)NVIC_BASE)

/*******************************************************************************
 * API
 ******************************************************************************/

/**
  * @brief  Enable interrupt request
  * @param  IRQn: IRQ type
  * @retval None
  */
void NVIC_EnableIRQ(IRQn_Type IRQn);

/**
  * @brief  Disable interrupt request
  * @param  IRQn: IRQ type
  * @retval None
  */
void NVIC_DisbleIRQ(IRQn_Type IRQn);

/**
  * @brief  Set the pending status of interrupt
  * @param  IRQn: IRQ type
  * @retval None
  */
void NVIC_SetPendingIRQ(IRQn_Type IRQn);

/**
  * @brief  Clear the pending status of interrupt
  * @param  IRQn: IRQ type
  * @retval None
  */
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);

/**
  * @brief  Get the pending status of interrupt
  * @param  IRQn: IRQ type
  * @retval Set(1) or Clear(0)
  */
uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn);

/**
  * @brief  Set the priority of interrupt
  * @param  IRQn: IRQ type
  * @param  priority: IRQ priority number
  * @retval None
  */
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);

/**
  * @brief  Get the priority of interrupt
  * @param  IRQn: IRQ type
  * @retval Priority number
  */
uint32_t NVIC_GetPriority(IRQn_Type IRQn);

#endif /* DRV_S32K144_NVIC_H_ */
