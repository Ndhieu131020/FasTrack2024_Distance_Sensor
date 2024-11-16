/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "S32K144.h"
#include "DRV_S32K144_GPIO.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Array address GPIO */
static GPIO_Type *g_GPIO_Base_Addr[] = IP_GPIO_BASE_PTRS;

/*******************************************************************************
 * Codes
 ******************************************************************************/

/* Set Pins Direction (Input or Output) */
void DRV_GPIO_SetPinDirection(const uint8_t               Port_Ins,
                              const uint8_t               Pin,
                              const GPIO_Data_Direction_t mode)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    if (mode == Input_Direction)
    {
        base->PDDR &= (uint32_t)(~(1 << Pin));
    }
    else
    {
        base->PDDR |= (uint32_t)(1 << Pin);
    }
}

/* Read Input Pins */
uint32_t DRV_GPIO_ReadPins(const uint8_t Port_Ins, const uint8_t Pin)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    return ((base->PDIR) & (uint32_t)(1 << Pin));
}

/* Write Output Pins */
void DRV_GPIO_WritePins(const uint8_t            Port_Ins,
                        const uint8_t            Pin,
                        const GPIO_Data_Output_t mode)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    if (mode == GPIO_PIN_0_LOGIC)
    {
        base->PDOR &= (uint32_t)(~(1 << Pin));
    }
    else
    {
        base->PDOR |= (uint32_t)(1 << Pin);
    }
}

/* Set Output Pins */
void DRV_GPIO_SetPins(const uint8_t Port_Ins, const uint8_t Pin)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    base->PSOR |= (uint32_t)(1 << Pin);
}

/* Clear Output Pins */
void DRV_GPIO_ClearPins(const uint8_t Port_Ins, const uint8_t Pin)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    base->PCOR |= (uint32_t)(1 << Pin);
}

/* Toggle Output Pins */
void DRV_GPIO_TogglePins(const uint8_t Port_Ins, const uint8_t Pin)
{
    GPIO_Type *base = g_GPIO_Base_Addr[Port_Ins];

    base->PTOR |= (uint32_t)(1 << Pin);
}

/*******************************************************************************
 * End Of File
 ******************************************************************************/

