#ifndef DRV_S32K144_PORT_H_
#define DRV_S32K144_PORT_H_

#include "S32K144.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Enum: Port MUX type */
typedef enum
{
    PORT_PIN_DISABLED    = 0U,  /* Corresponding pin is disabled, but is used as an analog pin */
    PORT_MUX_AS_GPIO     = 1U,  /* Corresponding pin is configured as GPIO                     */
    PORT_MUX_ALT2        = 2U,  /* Chip-specific                                               */
    PORT_MUX_ALT3        = 3U,  /* Chip-specific                                               */
    PORT_MUX_ALT4        = 4U,  /* Chip-specific                                               */
    PORT_MUX_ALT5        = 5U,  /* Chip-specific                                               */
    PORT_MUX_ALT6        = 6U,  /* Chip-specific                                               */
    PORT_MUX_ALT7        = 7U   /* Chip-specific                                               */
} port_mux_t;

/* Enum: Port interrupt config type */
typedef enum
{
    PORT_INT_DISABLED      = 0x0U,  /* Interrupt request is disabled */
    PORT_INT_LOGIC_ZERO    = 0x8U,  /* Interrupt when logic 0        */
    PORT_INT_RISING_EDGE   = 0x9U,  /* Interrupt on rising edge      */
    PORT_INT_FALLING_EDGE  = 0xAU,  /* Interrupt on falling edge     */
    PORT_INT_EITHER_EDGE   = 0xBU,  /* Interrupt on either edge      */
    PORT_INT_LOGIC_ONE     = 0xCU   /* Interrupt when logic 1        */
} port_interrupt_config_t;

/* Enum: Port pull config type */
typedef enum
{
    PORT_INTERNAL_PULL_NOT_ENABLED   = 0U,  /* Internal pull-down or pull-up resistor is not enabled */
    PORT_INTERNAL_PULL_DOWN_ENABLED  = 1U,  /* Internal pull-down resistor is enabled                */
    PORT_INTERNAL_PULL_UP_ENABLED    = 2U   /* Internal pull-up resistor is enabled                  */
} port_pull_config_t;

/* Struct: Port config type */
typedef struct
{
    port_mux_t                 Mux;            /* Config MUX       */
    port_interrupt_config_t    Interrupt;      /* Config Interrupt */
    port_pull_config_t         Pull;           /* Config Pull      */
} PortConfig_t;

#define GET_PORT(pin)      ((pin) / 32U)
#define GET_PIN_NUM(pin)   ((pin) % 32U)

/* Enum: Virtual Pin ID */
typedef enum
{
    /* Pin in Port A */
    PTA0  = 0U,     PTA1  = 1U,     PTA2  = 2U,     PTA3  = 3U,     PTA4  = 4U,     PTA5  = 5U,
    PTA6  = 6U,     PTA7  = 7U,     PTA8  = 8U,     PTA9  = 9U,     PTA10 = 10U,    PTA11 = 11U,
    PTA12 = 12U,    PTA13 = 13U,    PTA14 = 14U,    PTA15 = 15U,    PTA16 = 16U,    PTA17 = 17U,

    /* Pin in Port B */
    PTB0  = 32U,    PTB1  = 33U,    PTB2  = 34U,    PTB3  = 35U,    PTB4  = 36U,    PTB5  = 37U,
    PTB6  = 38U,    PTB7  = 39U,    PTB8  = 40U,    PTB9  = 41U,    PTB10 = 42U,    PTB11 = 43U,
    PTB12 = 44U,    PTB13 = 45U,    PTB14 = 46U,    PTB15 = 47U,    PTB16 = 48U,    PTB17 = 49U,

    /* Pin in Port C */
    PTC0  = 64U,    PTC1  = 65U,    PTC2  = 66U,    PTC3  = 67U,    PTC4  = 68U,    PTC5  = 69U,
    PTC6  = 70U,    PTC7  = 71U,    PTC8  = 72U,    PTC9  = 73U,    PTC10 = 74U,    PTC11 = 75U,
    PTC12 = 76U,    PTC13 = 77U,    PTC14 = 78U,    PTC15 = 79U,    PTC16 = 80U,    PTC17 = 81U,

    /* Pin in Port D */
    PTD0  = 96U,    PTD1  = 97U,    PTD2  = 98U,    PTD3  = 99U,    PTD4  = 100U,   PTD5  = 101U,
    PTD6  = 102U,   PTD7  = 103U,   PTD8  = 104U,   PTD9  = 105U,   PTD10 = 106U,   PTD11 = 107U,
    PTD12 = 108U,   PTD13 = 109U,   PTD14 = 110U,   PTD15 = 111U,   PTD16 = 112U,   PTD17 = 113U,

    /* Pin in Port E */
    PTE0  = 128U,   PTE1  = 129U,   PTE2  = 130U,   PTE3  = 131U,   PTE4  = 132U,   PTE5  = 133U,
    PTE6  = 134U,   PTE7  = 135U,   PTE8  = 136U,   PTE9  = 137U,   PTE10 = 138U,   PTE11 = 139U,
    PTE12 = 140U,   PTE13 = 141U,   PTE14 = 142U,   PTE15 = 143U,   PTE16 = 144U
} virtual_pin_id_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*##############################################################################
**
* @brief: Configures the port
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] pConfig: The port configuration
* @return
*
*#############################################################################*/
void DRV_PORT_Init(const uint8_t        Port_Ins,
                   const uint8_t        Pin,
                   const PortConfig_t * pConfig);

#endif /* DRV_S32K144_PORT_H_ */

/*******************************************************************************
 * End Of File
 ******************************************************************************/
