/*
 *  Filename: MID_Clock_Interface.c
 *
 *  Created on: 11-04-2024
 *      Author: Ndhieu131020@gmail.com
*/

#include "DRV_S32K144_MCU.h"
#include "MID_Clock_Interface.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

#define NUM_OF_PERIPHERAL_CLOCKS_0     (6U)
#define CLOCK_SOURCE_NONE              (0U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Initializes the clock module */
void MID_Clock_Init(void)
{
    peripheral_clk_config_t peripheralClockConfig0[NUM_OF_PERIPHERAL_CLOCKS_0] =
    {
        {
            .clockName   = PORTC_CLK,
            .enableClock = true,
            .clkSrc      = CLOCK_SOURCE_NONE
        }
        ,
        {
            .clockName   = PORTD_CLK,
            .enableClock = true,
            .clkSrc      = CLOCK_SOURCE_NONE
        }
        ,
        {
            .clockName   = PORTE_CLK,
            .enableClock = true,
            .clkSrc      = CLOCK_SOURCE_NONE
        }
        ,
        {
            .clockName   = FlexCAN0_CLK,
            .enableClock = true,
            .clkSrc      = CLOCK_SOURCE_NONE
        }
        ,
        {
            .clockName   = ADC0_CLK,
            .enableClock = true,
            .clkSrc      = (uint8_t)SCG_SYSTEM_CLOCK_SRC_SPLL
        }
        ,
        {
            .clockName   = LPIT0_CLK,
            .enableClock = true,
            .clkSrc      = (uint8_t)SCG_SYSTEM_CLOCK_SRC_SPLL
        }
    };

    const clock_manager_config_t clock_InitConfig0 =
    {
        .scgConfig =
        {
            .sircConfig =
            {
                .enableModule = true,
                .locked       = false,                          /* unlocked                               */
                .range        = SCG_SIRC_RANGE_HIGH,            /* Slow IRC high range clock (8 MHz)      */
                .div1         = SCG_ASYNC_CLOCK_DIV_BY_1,       /* Slow IRC Clock Divider 1: divided by 1 */
                .div2         = SCG_ASYNC_CLOCK_DIV_BY_1,       /* Slow IRC Clock Divider 2: divided by 1 */
            },
            .fircConfig =
            {
                .regulator    = true,                        /* FIRC regulator is enabled              */
                .locked       = false,                       /* unlocked                               */
                .div1         = SCG_ASYNC_CLOCK_DIV_BY_1,    /* Fast IRC Clock Divider 1: divided by 1 */
                .div2         = SCG_ASYNC_CLOCK_DIV_BY_1,    /* Fast IRC Clock Divider 2: divided by 1 */
            },
            .soscConfig =
            {
                .enableModule = true,
                .monitorMode  = SCG_SOSC_MONITOR_DISABLE,    /* Monitor disabled                                                            */
                .locked       = false,                       /* Slow IRC disabled                                                           */
                .extRef       = SCG_SOSC_REF_OSC,            /* Internal oscillator of OSC requested                                        */
                .gain         = SCG_SOSC_GAIN_LOW,           /* Configure crystal oscillator for low-gain operation                         */
                .range        = SCG_SOSC_RANGE_HIGH,         /* High frequency range selected for the crystal oscillator of 8 MHz to 40 MHz */
                .div1         = SCG_ASYNC_CLOCK_DIV_BY_1,    /* System OSC Clock Divider 1: divided by 1                                    */
                .div2         = SCG_ASYNC_CLOCK_DIV_BY_1,    /* System OSC Clock Divider 2: divided by 1                                    */
            },
            .spllConfig =
            {
                .enableModule = true,
                .monitorMode  = SCG_SPLL_MONITOR_DISABLE,    /* Monitor disabled                         */
                .locked       = false,                       /* unlocked                                 */
                .prediv       = SCG_SPLL_PREDIV1,            /* Divided by 1                             */
                .mult         = SCG_SPLL_VCO_MULT_40,        /* Multiply Factor is 40                    */
                .div1         = SCG_ASYNC_CLOCK_DIV_BY_2,    /* System PLL Clock Divider 1: divided by 2 */
                .div2         = SCG_ASYNC_CLOCK_DIV_BY_4,    /* System PLL Clock Divider 2: divided by 4 */
            },
            .clkModeConfig =
            {
                .initialize = true,
                .rccrConfig =
                {
                    .src     = SCG_SYSTEM_CLOCK_SRC_SPLL,    /* System PLL                       */
                    .divCore = SCG_SYSTEM_CLOCK_DIV_BY_2,    /* Core Clock Divider: divided by 2 */
                    .divBus  = SCG_SYSTEM_CLOCK_DIV_BY_2,    /* Bus Clock Divider: divided by 2  */
                    .divSlow = SCG_SYSTEM_CLOCK_DIV_BY_3,    /* Slow Clock Divider: divided by 3 */
                }
            }
        },
        .pccConfig =
        {
            .peripheralClockConfigs = peripheralClockConfig0,      /* Peripheral clock control configurations                */
            .numPeripherals         = NUM_OF_PERIPHERAL_CLOCKS_0,  /* Number of the peripheral clock control configurations  */
        }
    };

    DRV_Clock_Init(&clock_InitConfig0);
}
