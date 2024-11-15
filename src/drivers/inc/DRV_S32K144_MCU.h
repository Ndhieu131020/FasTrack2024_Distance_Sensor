#ifndef HAL_INCLUDE_CLOCK_H_
#define HAL_INCLUDE_CLOCK_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "S32K144.h"
#include "S32K144_features.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    CLOCK_STATUS_SUCCESS,
    CLOCK_STATUS_ERROR,
    CLOCK_STATUS_UNSUPPORTED,
    CLOCK_STATUS_MCU_GATED_OFF
} clock_status_t;

/* Enum: SCG asynchronous clock divider value */
typedef enum
{
    SCG_ASYNC_CLOCK_DISABLE   = 0U,    /* Clock output is disabled */
    SCG_ASYNC_CLOCK_DIV_BY_1  = 1U,    /* Divided by 1             */
    SCG_ASYNC_CLOCK_DIV_BY_2  = 2U,    /* Divided by 2             */
    SCG_ASYNC_CLOCK_DIV_BY_4  = 3U,    /* Divided by 4             */
    SCG_ASYNC_CLOCK_DIV_BY_8  = 4U,    /* Divided by 8             */
    SCG_ASYNC_CLOCK_DIV_BY_16 = 5U,    /* Divided by 16            */
    SCG_ASYNC_CLOCK_DIV_BY_32 = 6U,    /* Divided by 32            */
    SCG_ASYNC_CLOCK_DIV_BY_64 = 7U     /* Divided by 64            */
} scg_async_clock_div_t;

/* Enum: SCG slow IRC clock frequency range */
typedef enum
{
    SCG_SIRC_RANGE_HIGH = 1U     /* Slow IRC high range clock (8 MHz) */
} scg_sirc_range_t;

/* Struct: SCG slow IRC clock config */
typedef struct
{
    scg_sirc_range_t        range;               /* Slow IRC frequency range                */
    scg_async_clock_div_t   div1;                /* Asynchronous peripheral source          */
    scg_async_clock_div_t   div2;                /* Asynchronous peripheral source          */
    bool                    locked;              /* SIRC Control Register can be written    */
    bool                    enableModule;        /* Enable or disable the SIRC module       */
} scg_sirc_config_t;

/* Struct: SCG fast IRC clock config */
typedef struct
{
    scg_async_clock_div_t   div1;            /* Asynchronous peripheral source       */
    scg_async_clock_div_t   div2;            /* Asynchronous peripheral source       */
    bool                    regulator;       /* FIRC regulator is enable or not      */
    bool                    locked;          /* FIRC Control Register can be written */
} scg_firc_config_t;

/* Enum: SCG system OSC monitor mode */
typedef enum
{
    SCG_SOSC_MONITOR_DISABLE = 0U,    /* Monitor disable                          */
    SCG_SOSC_MONITOR_INT     = 1U,    /* Interrupt when system OSC error detected */
    SCG_SOSC_MONITOR_RESET   = 2U     /* Reset when system OSC error detected     */
} scg_sosc_monitor_mode_t;

/* Enum: SCG OSC external reference clock select */
typedef enum
{
    SCG_SOSC_REF_EXT   = 0x0,    /* External reference clock requested   */
    SCG_SOSC_REF_OSC   = 0x1     /* Internal oscillator of OSC requested */
} scg_sosc_ext_ref_t;

/* Enum: SCG OSC gain oscillator select */
typedef enum
{
    SCG_SOSC_GAIN_LOW    = 0x0,    /* Configure crystal oscillator for low-power operation */
    SCG_SOSC_GAIN_HIGH   = 0x1     /* Configure crystal oscillator for high-gain operation */
} scg_sosc_gain_t;

/* Enum: SCG OSC frequency range select */
typedef enum
{
    SCG_SOSC_RANGE_MID    = 2U,    /* Medium frequency range selected for the crystal OSC (4 Mhz to 8 Mhz) */
    SCG_SOSC_RANGE_HIGH   = 3U     /* High frequency range selected for the crystal OSC (8 Mhz to 40 Mhz)  */
} scg_sosc_range_t;

/* Struct: SCG SOSC clock config */
typedef struct
{
    scg_sosc_monitor_mode_t   monitorMode;     /* System OSC Clock monitor mode              */
    scg_sosc_ext_ref_t        extRef;          /* System OSC External Reference Select       */
    scg_sosc_gain_t           gain;            /* System OSC high-gain operation             */
    scg_sosc_range_t          range;           /* System OSC frequency range                 */
    scg_async_clock_div_t     div1;            /* Asynchronous peripheral source             */
    scg_async_clock_div_t     div2;            /* Asynchronous peripheral source             */
    bool                      locked;          /* System OSC Control Register can be written */
    bool                      enableModule;    /* Enable or disable the System OSC module    */
} scg_sosc_config_t;

/* Enum: SCG system PLL monitor mode */
typedef enum
{
    SCG_SPLL_MONITOR_DISABLE = 0U,    /* Monitor disable                          */
    SCG_SPLL_MONITOR_INT     = 1U,    /* Interrupt when system PLL error detected */
    SCG_SPLL_MONITOR_RESET   = 2U     /* Reset when system PLL error detected     */
} scg_spll_monitor_mode_t;

/* Enum: SCG system PLL VCO Multiply Factor */
typedef enum
{
    SCG_SPLL_VCO_MULT_16 = 0U,     /* Multiply Factor 16 */
    SCG_SPLL_VCO_MULT_17 = 1U,     /* Multiply Factor 17 */
    SCG_SPLL_VCO_MULT_18 = 2U,     /* Multiply Factor 18 */
    SCG_SPLL_VCO_MULT_19 = 3U,     /* Multiply Factor 19 */
    SCG_SPLL_VCO_MULT_20 = 4U,     /* Multiply Factor 20 */
    SCG_SPLL_VCO_MULT_21 = 5U,     /* Multiply Factor 21 */
    SCG_SPLL_VCO_MULT_22 = 6U,     /* Multiply Factor 22 */
    SCG_SPLL_VCO_MULT_23 = 7U,     /* Multiply Factor 23 */
    SCG_SPLL_VCO_MULT_24 = 8U,     /* Multiply Factor 24 */
    SCG_SPLL_VCO_MULT_25 = 9U,     /* Multiply Factor 25 */
    SCG_SPLL_VCO_MULT_26 = 10U,    /* Multiply Factor 26 */
    SCG_SPLL_VCO_MULT_27 = 11U,    /* Multiply Factor 27 */
    SCG_SPLL_VCO_MULT_28 = 12U,    /* Multiply Factor 28 */
    SCG_SPLL_VCO_MULT_29 = 13U,    /* Multiply Factor 29 */
    SCG_SPLL_VCO_MULT_30 = 14U,    /* Multiply Factor 30 */
    SCG_SPLL_VCO_MULT_31 = 15U,    /* Multiply Factor 31 */
    SCG_SPLL_VCO_MULT_32 = 16U,    /* Multiply Factor 32 */
    SCG_SPLL_VCO_MULT_33 = 17U,    /* Multiply Factor 33 */
    SCG_SPLL_VCO_MULT_34 = 18U,    /* Multiply Factor 34 */
    SCG_SPLL_VCO_MULT_35 = 19U,    /* Multiply Factor 35 */
    SCG_SPLL_VCO_MULT_36 = 20U,    /* Multiply Factor 36 */
    SCG_SPLL_VCO_MULT_37 = 21U,    /* Multiply Factor 37 */
    SCG_SPLL_VCO_MULT_38 = 22U,    /* Multiply Factor 38 */
    SCG_SPLL_VCO_MULT_39 = 23U,    /* Multiply Factor 39 */
    SCG_SPLL_VCO_MULT_40 = 24U,    /* Multiply Factor 40 */
    SCG_SPLL_VCO_MULT_41 = 25U,    /* Multiply Factor 41 */
    SCG_SPLL_VCO_MULT_42 = 26U,    /* Multiply Factor 42 */
    SCG_SPLL_VCO_MULT_43 = 27U,    /* Multiply Factor 43 */
    SCG_SPLL_VCO_MULT_44 = 28U,    /* Multiply Factor 44 */
    SCG_SPLL_VCO_MULT_45 = 29U,    /* Multiply Factor 45 */
    SCG_SPLL_VCO_MULT_46 = 30U,    /* Multiply Factor 46 */
    SCG_SPLL_VCO_MULT_47 = 31U     /* Multiply Factor 47 */
} scg_spll_vco_mult_t;

/* Enum: SCG system PLL Reference Divide Factor */
typedef enum
{
    SCG_SPLL_PREDIV1 = 0U,    /* Divide Factor 1 */
    SCG_SPLL_PREDIV2 = 1U,    /* Divide Factor 2 */
    SCG_SPLL_PREDIV3 = 2U,    /* Divide Factor 3 */
    SCG_SPLL_PREDIV4 = 3U,    /* Divide Factor 4 */
    SCG_SPLL_PREDIV5 = 4U,    /* Divide Factor 5 */
    SCG_SPLL_PREDIV6 = 5U,    /* Divide Factor 6 */
    SCG_SPLL_PREDIV7 = 6U,    /* Divide Factor 7 */
    SCG_SPLL_PREDIV8 = 7U     /* Divide Factor 8 */
} scg_spll_prediv_t;

/* Struct: SCG system PLL config */
typedef struct
{
    scg_spll_monitor_mode_t   monitorMode;     /* Clock monitor mode selected                */
    scg_spll_prediv_t         prediv;          /* PLL reference clock divider                */
    scg_spll_vco_mult_t       mult;            /* System PLL multiplier                      */
    scg_async_clock_div_t     div1;            /* Asynchronous peripheral source             */
    scg_async_clock_div_t     div2;            /* Asynchronous peripheral source             */
    bool                      locked;          /* System PLL Control Register can be written */
    bool                      enableModule;    /* Enable or disable the System PLL module    */
} scg_spll_config_t;

typedef enum
{
    SCG_SYSTEM_CLOCK_DIV_BY_1   = 0U,     /* Divided by 1  */
    SCG_SYSTEM_CLOCK_DIV_BY_2   = 1U,     /* Divided by 2  */
    SCG_SYSTEM_CLOCK_DIV_BY_3   = 2U,     /* Divided by 3  */
    SCG_SYSTEM_CLOCK_DIV_BY_4   = 3U,     /* Divided by 4  */
    SCG_SYSTEM_CLOCK_DIV_BY_5   = 4U,     /* Divided by 5  */
    SCG_SYSTEM_CLOCK_DIV_BY_6   = 5U,     /* Divided by 6  */
    SCG_SYSTEM_CLOCK_DIV_BY_7   = 6U,     /* Divided by 7  */
    SCG_SYSTEM_CLOCK_DIV_BY_8   = 7U,     /* Divided by 8  */
    SCG_SYSTEM_CLOCK_DIV_BY_9   = 8U,     /* Divided by 9  */
    SCG_SYSTEM_CLOCK_DIV_BY_10  = 9U,     /* Divided by 10 */
    SCG_SYSTEM_CLOCK_DIV_BY_11  = 10U,    /* Divided by 11 */
    SCG_SYSTEM_CLOCK_DIV_BY_12  = 11U,    /* Divided by 12 */
    SCG_SYSTEM_CLOCK_DIV_BY_13  = 12U,    /* Divided by 13 */
    SCG_SYSTEM_CLOCK_DIV_BY_14  = 13U,    /* Divided by 14 */
    SCG_SYSTEM_CLOCK_DIV_BY_15  = 14U,    /* Divided by 15 */
    SCG_SYSTEM_CLOCK_DIV_BY_16  = 15U,    /* Divided by 16 */
} scg_system_clock_div_t;

typedef enum
{
    SCG_SYSTEM_CLOCK_SRC_SOSC = 1U,       /* System OSC */
    SCG_SYSTEM_CLOCK_SRC_SIRC = 2U,       /* Slow IRC   */
    SCG_SYSTEM_CLOCK_SRC_FIRC = 3U,       /* Fast IRC   */
    SCG_SYSTEM_CLOCK_SRC_SPLL = 6U        /* System PLL */
} scg_system_clock_src_t;

typedef struct
{
    scg_system_clock_div_t divSlow;  /* Slow clock divider  */
    scg_system_clock_div_t divBus;   /* BUS clock divider   */
    scg_system_clock_div_t divCore;  /* Core clock divider  */
    scg_system_clock_src_t src;      /* System clock source */
} scg_sys_clock_config_t;

/* Struct: SCG clock mode config */
typedef struct
{
    scg_sys_clock_config_t  rccrConfig;   /* Run Clock Control configuration                */
    bool                    initialize;   /* Initialize or not the Clock Mode Configuration */
} scg_clock_mode_config_t;

/* Struct: SCG config */
typedef struct
{
    scg_sirc_config_t         sircConfig;       /* Slow internal reference clock configuration */
    scg_firc_config_t         fircConfig;       /* Fast internal reference clock configuration */
    scg_sosc_config_t         soscConfig;       /* System oscillator configuration             */
    scg_spll_config_t         spllConfig;       /* System Phase locked loop configuration      */
    scg_clock_mode_config_t   clkModeConfig;    /* Clock mode configuration                    */
} scg_config_t;

/* Struct: PCC config */
typedef struct
{
    clock_names_t    clockName;
    uint8_t          clkSrc;       /* Peripheral clock source     */
    bool             enableClock;  /* Enable or disable the clock */
} peripheral_clk_config_t;

typedef struct
{
    uint32_t                  numPeripherals;           /* Number of peripheral configurations           */
    peripheral_clk_config_t * peripheralClockConfigs;   /* Array of clock configurations for peripherals */
} pcc_config_t;

/* Struct: Clock configuration */
typedef struct
{
    scg_config_t    scgConfig;     /* SCG Clock configuration */
    pcc_config_t    pccConfig;     /* PCC Clock configuration */
} clock_manager_config_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

clock_status_t DRV_Clock_Init(const clock_manager_config_t * clkConfig);

clock_status_t DRV_Clock_GetFrequency(clock_names_t clockName, uint32_t * frequency);

#endif /* HAL_INCLUDE_CLOCK_H_ */

/*******************************************************************************
 * End Of File
 ******************************************************************************/

