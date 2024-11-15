/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "DRV_S32K144_PORT.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*##############################################################################
**
* @brief: Configures the port MUX
*
* @param [in] base: Port base pointer (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] PortMuxType: The MUX configuration
* @return
*
*#############################################################################*/
static void DRV_PORT_SetMux(      PORT_Type * base,
                            const uint8_t     Pin,
                            const port_mux_t  PortMuxType);

/*##############################################################################
**
* @brief: Configures the port interrupt
*
* @param [in] base: Port base pointer (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] intConfig: Interrupt configuration
* @return
*
*#############################################################################*/
static void DRV_PORT_SetInterrupt(      PORT_Type *             base,
                                  const uint8_t                 Port_Ins,
                                  const uint8_t                 Pin,
                                  const port_interrupt_config_t intConfig);

/*##############################################################################
**
* @brief: Configures the port pull
*
* @param [in] base: Port base pointer (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] pullConfig: The pull configuration
* @return
*
*#############################################################################*/
static void DRV_PORT_SetPull(      PORT_Type *        base,
                             const uint8_t            Pin,
                             const port_pull_config_t pullConfig);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Array address port */
static PORT_Type *g_PORT_Base_Addr[] = IP_PORT_BASE_PTRS;

/*******************************************************************************
 * Codes
 ******************************************************************************/

/* Configures the port */
void DRV_PORT_Init(const uint8_t        Port_Ins,
                   const uint8_t        Pin,
                   const PortConfig_t * pConfig)
{
    /* Get port base address */
    PORT_Type *base = g_PORT_Base_Addr[Port_Ins];

    /* Set MUX */
    DRV_PORT_SetMux(base, Pin, pConfig->Mux);

    /* Set interrupt */
    DRV_PORT_SetInterrupt(base, Port_Ins, Pin, pConfig->Interrupt);

    /* Set pull */
    DRV_PORT_SetPull(base, Pin, pConfig->Pull);
}

/* Configures the port MUX */
static void DRV_PORT_SetMux(      PORT_Type * base,
                            const uint8_t     Pin,
                            const port_mux_t  PortMuxType)
{
    base->PCR[Pin] = (base->PCR[Pin] & (~PORT_PCR_MUX_MASK))
                                     |   PORT_PCR_MUX(PortMuxType);
}

/* Configures the port interrupt */
static void DRV_PORT_SetInterrupt(      PORT_Type *             base,
                                  const uint8_t                 Port_Ins,
                                  const uint8_t                 Pin,
                                  const port_interrupt_config_t intConfig)
{
    /* Modify IRQC */
    base->PCR[Pin] = (base->PCR[Pin] & (~PORT_PCR_IRQC_MASK))
                                     |   PORT_PCR_IRQC(intConfig);
}

/* Configures the port pull */
static void DRV_PORT_SetPull(      PORT_Type *        base,
                             const uint8_t            Pin,
                             const port_pull_config_t pullConfig)
{
    switch (pullConfig)
    {
        case PORT_INTERNAL_PULL_NOT_ENABLED:
            {
                base->PCR[Pin] &= ~(PORT_PCR_PE_MASK);
            }
            break;
        case PORT_INTERNAL_PULL_DOWN_ENABLED:
            {
                base->PCR[Pin] |= PORT_PCR_PE_MASK;     /* Pull enabled */
                base->PCR[Pin] &= ~(PORT_PCR_PS_MASK);  /* Pulldown */
            }
            break;
        case PORT_INTERNAL_PULL_UP_ENABLED:
            {
                base->PCR[Pin] |= PORT_PCR_PE_MASK;     /* Pull enabled */
                base->PCR[Pin] |= PORT_PCR_PS_MASK;     /* Pullup */
            }
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * End Of File
 ******************************************************************************/

