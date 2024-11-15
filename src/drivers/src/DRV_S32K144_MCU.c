/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "DRV_S32K144_MCU.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    SCG_SYSTEM_CLOCK_CORE,  /* Core clock        */
    SCG_SYSTEM_CLOCK_BUS,   /* BUS clock         */
    SCG_SYSTEM_CLOCK_SLOW   /* System slow clock */
} scg_system_clock_type;

typedef enum
{
    SCG_ASYNC_CLOCK_DIV1   = 0U,   /* Clock divider 1  */
    SCG_ASYNC_CLOCK_DIV2   = 1U    /* Clock divider 2  */
} scg_async_clock_type;

/* default frequency clock modules */
#define FEATURE_SCG_SOSC_FREQ                 (8000000U)

/* Slow IRC LOW range clock frequency */
#define FEATURE_SCG_SIRC_LOW_RANGE_FREQ       (2000000U)

/* System PLL base multiplier value */
#define SCG_SPLL_MULT_BASE                    (16U)

/* System PLL base divider value */
#define SCG_SPLL_PREDIV_BASE                  (1U)

/* VCO divider shift amount */
#define VCO_DIVIDER                           (1U)

/* Number of available run modes for the system */
#define RUN_MODE_COUNT                        (7U)

/* Number of different clock types: Core, Bus, and Slow */
#define CLOCK_TYPE_COUNT                      (3U)

/* Array index for the Core clock in frequency tables */
#define CORE_CLK_INDEX                        (0U)

/* Array index for the Bus clock in frequency tables */
#define BUS_CLK_INDEX                         (1U)

/* Array index for the Slow clock in frequency tables */
#define SLOW_CLK_INDEX                        (2U)

/* Shift value for system clock frequency scaling */
#define SYSTEM_CLOCK_SHIFT                    (4U)

#define SYSTEM_CLK_MAX_FREQUENCIES_RUN_MODE                                                           \
{/* SYS_CLK    BUS_CLK     SLOW_CLK */                                                                \
{       0U,        0U,         0U},              /* Invalid entry */                                  \
{80000000U, 48000000U,  26670000U},              /* Maximum frequencies when system clock is SOSC */  \
{80000000U, 48000000U,  26670000U},              /* Maximum frequencies when system clock is SIRC */  \
{80000000U, 48000000U,  26670000U},              /* Maximum frequencies when system clock is FIRC */  \
{       0U,        0U,         0U},              /* Invalid entry */                                  \
{       0U,        0U,         0U},              /* Invalid entry */                                  \
{80000000U, 40000000U,  26670000U},              /* Maximum frequencies when system clock is SPLL */  \
}

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static clock_status_t DRV_SCG_Configuration(const scg_config_t *scgConfig);

static void DRV_SCG_SIRC_Config(const scg_sirc_config_t *sircConfig);

static void DRV_SCG_SIRC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2);


static void DRV_SCG_SIRC_SetConfiguration(const scg_sirc_range_t range);


static void DRV_SCG_SIRC_Control(const bool enableSIRC, const bool lockMode);

static void DRV_SCG_FIRC_Config(const scg_firc_config_t *fircConfig);

static void DRV_SCG_FIRC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2);


static void DRV_SCG_FIRC_Control(const bool regulator, const bool lockMode);

static void DRV_SCG_SOSC_Config(const scg_sosc_config_t *soscConfig);

static void DRV_SCG_SOSC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2);


static void DRV_SCG_SOSC_SetConfiguration(const scg_sosc_range_t range,
                                          const scg_sosc_gain_t gain,
                                          const scg_sosc_ext_ref_t extRef);


static void DRV_SCG_SOSC_Control(const bool enableSOSC,
                                 const scg_sosc_monitor_mode_t monitorMode,
                                 const bool lockMode);

static void DRV_SCG_SPLL_Config(const scg_spll_config_t *spllConfig);

static void DRV_SCG_SPLL_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2);


static void DRV_SCG_SPLL_SetConfiguration(const scg_spll_prediv_t   divider,
                                          const scg_spll_vco_mult_t multiplier);


static void DRV_SCG_SPLL_Control(const bool enableSPLL,
                                 const scg_spll_monitor_mode_t monitorMode,
                                 const bool lockMode);

static clock_status_t DRV_SCG_SystemClockConfig(const scg_sys_clock_config_t * sysClkConfig);

static void DRV_SCG_SetRunClockControl(uint32_t source, uint32_t coreDivider, uint32_t busDivider, uint32_t slowDivider);

static void DRV_PCC_Configuration(const pcc_config_t *peripheralClockConfig);

static void DRV_PCC_SetPeripheralClockControl(const clock_names_t clockName,
                                              const uint8_t       clkSrc,
                                              const bool          isClockEnabled);

static clock_status_t DRV_SCG_GetClockFrequency(clock_names_t clockName, uint32_t * frequency);

static uint32_t DRV_Clock_GetSourceFrequency(scg_system_clock_src_t source);

static uint32_t DRV_Clock_GetSysClockFrequency(scg_system_clock_type clkType);

static uint32_t DRV_Clock_GetSOSCFrequency(void);

static uint32_t DRV_Clock_GetSIRCFrequency(void);

static uint32_t DRV_Clock_GetFIRCFrequency(void);

static uint32_t DRV_Clock_GetSPLLFrequency(void);

static uint32_t DRV_Clock_GetSysAsyncFrequency(clock_names_t clockSrc, scg_async_clock_type type);

static clock_status_t DRV_PCC_GetClockFrequency(clock_names_t clockName, uint32_t * frequency);

static clock_status_t DRV_PCC_CheckClock(clock_names_t clockName);

static uint32_t DRV_PCC_GetPeripheralClock(clock_names_t        clockName,
                                           scg_async_clock_type divider);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* The array storing the mappings between clock names and PCC indexes */
static uint8_t  g_PCC_ClockNameMappings[]  = PCC_CLOCK_NAME_MAPPINGS;
static uint32_t g_PeripheralFeaturesList[] = PERIPHERAL_FEATURES;

/*******************************************************************************
 * Codes
 ******************************************************************************/

clock_status_t DRV_Clock_Init(const clock_manager_config_t * clkConfig)
{
    clock_status_t retVal = CLOCK_STATUS_SUCCESS;

    if (clkConfig == NULL)
    {
        retVal = CLOCK_STATUS_ERROR;
    }
    else
    {
        retVal = DRV_SCG_Configuration(&(clkConfig->scgConfig));

        if (retVal == CLOCK_STATUS_SUCCESS)
        {
            DRV_PCC_Configuration(&(clkConfig->pccConfig));
        }
        else
        {
            /* Do nothing */
        }
    }

    return retVal;
}

/* Configure the SCG block */
static clock_status_t DRV_SCG_Configuration(const scg_config_t *scgConfig)
{
    clock_status_t retVal = CLOCK_STATUS_SUCCESS;

    if (scgConfig == NULL)
    {
        retVal = CLOCK_STATUS_ERROR;
    }
    else
    {
        /* Configure SIRC module */
        DRV_SCG_SIRC_Config(&(scgConfig->sircConfig));

        /* Configure FIRC module */
        DRV_SCG_FIRC_Config(&(scgConfig->fircConfig));

        /* Configure SOSC module */
        DRV_SCG_SOSC_Config(&(scgConfig->soscConfig));

        /* Configure SPLL module */
        DRV_SCG_SPLL_Config(&(scgConfig->spllConfig));

        /* Configure SCG clock module */
        if (scgConfig->clkModeConfig.initialize)
        {
            retVal = DRV_SCG_SystemClockConfig(&(scgConfig->clkModeConfig.rccrConfig));
        }
        else
        {
            /* Do nothing */
        }
    }

    return retVal;
}

/* Configure SIRC module */
static void DRV_SCG_SIRC_Config(const scg_sirc_config_t *sircConfig)
{
    if (sircConfig != NULL)
    {
        /* Disable SIRC to configure SIRC */
        IP_SCG->SIRCCSR &= ~(SCG_SIRCCSR_SIRCEN_MASK);

        /* Set SCG asynchronous dividers for SIRC */
        DRV_SCG_SIRC_AsyncConfig(sircConfig->div1, sircConfig->div2);

        /* Set SIRC frequency range */
        DRV_SCG_SIRC_SetConfiguration(sircConfig->range);

        /* Set SIRC control register */
        DRV_SCG_SIRC_Control(sircConfig->enableModule, sircConfig->locked);
    }
    else
    {
        /* Do nothing */
    }
}

/* Set SCG asynchronous dividers for SIRC */
static void DRV_SCG_SIRC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2)
{
    IP_SCG->SIRCDIV = (IP_SCG->SIRCDIV & (~(SCG_SIRCDIV_SIRCDIV1_MASK  | SCG_SIRCDIV_SIRCDIV2_MASK)))
                                       |    SCG_SIRCDIV_SIRCDIV1(div1) | SCG_SIRCDIV_SIRCDIV2(div2);
}

/* Set SIRC frequency range */
static void DRV_SCG_SIRC_SetConfiguration(const scg_sirc_range_t range)
{
    IP_SCG->SIRCCFG = (IP_SCG->SIRCCFG & (~SCG_SIRCCFG_RANGE_MASK))
                                       |   SCG_SIRCCFG_RANGE(range);
}

/* Set SIRC control register */
static void DRV_SCG_SIRC_Control(const bool enableSIRC, const bool lockMode)
{
    IP_SCG->SIRCCSR = (IP_SCG->SIRCCSR & (~(SCG_SIRCCSR_LK_MASK      | SCG_SIRCCSR_SIRCEN_MASK )))
                                       |    SCG_SIRCCSR_LK(lockMode) | SCG_SIRCCSR_SIRCEN(enableSIRC);

    if (enableSIRC)
    {
        /* Wait for SIRC clk valid */
        while(!(IP_SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK));
    }
    else
    {
        /* Do nothing */
    }
}

/* Configure FIRC module */
static void DRV_SCG_FIRC_Config(const scg_firc_config_t *fircConfig)
{
    if (fircConfig != NULL)
    {
        /* Set SCG asynchronous dividers for FIRC */
        DRV_SCG_FIRC_AsyncConfig(fircConfig->div1, fircConfig->div2);

        /* Set FIRC control register */
        DRV_SCG_FIRC_Control(fircConfig->regulator, fircConfig->locked);
    }
    else
    {
        /* Do nothing */
    }
}

/* Set SCG asynchronous dividers for FIRC */
static void DRV_SCG_FIRC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2)
{
    IP_SCG->FIRCDIV = (IP_SCG->FIRCDIV & (~(SCG_FIRCDIV_FIRCDIV1_MASK  | SCG_FIRCDIV_FIRCDIV2_MASK)))
                                       |    SCG_FIRCDIV_FIRCDIV1(div1) | SCG_FIRCDIV_FIRCDIV2(div2);
}

/* Set FIRC control register */
static void DRV_SCG_FIRC_Control(const bool regulator, const bool lockMode)
{
    IP_SCG->FIRCCSR = (IP_SCG->FIRCCSR & (~(SCG_FIRCCSR_FIRCREGOFF_MASK        | SCG_FIRCCSR_LK_MASK)))
                                       |    SCG_FIRCCSR_FIRCREGOFF(!regulator) | SCG_FIRCCSR_LK(lockMode);
}

/* Configure SOSC module */
static void DRV_SCG_SOSC_Config(const scg_sosc_config_t *soscConfig)
{
    if (soscConfig != NULL)
    {
        /* Disable SOSC to configure SOSC */
        IP_SCG->SOSCCSR &= ~(SCG_SOSCCSR_SOSCEN_MASK);

        /* Set SCG asynchronous dividers for SOSC */
        DRV_SCG_SOSC_AsyncConfig(soscConfig->div1, soscConfig->div2);

        /* Set SOSC configuration register */
        DRV_SCG_SOSC_SetConfiguration(soscConfig->range,
                                      soscConfig->gain,
                                      soscConfig->extRef);

        /* Set SOSC control register */
        DRV_SCG_SOSC_Control(soscConfig->enableModule,
                             soscConfig->monitorMode,
                             soscConfig->locked);
    }
    else
    {
        /* Do nothing */
    }
}

/* Set SCG asynchronous dividers for SOSC */
static void DRV_SCG_SOSC_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2)
{
    IP_SCG->SOSCDIV = (IP_SCG->SOSCDIV & (~(SCG_SOSCDIV_SOSCDIV1_MASK  | SCG_SOSCDIV_SOSCDIV2_MASK)))
                                       |    SCG_SOSCDIV_SOSCDIV1(div1) | SCG_SOSCDIV_SOSCDIV2(div2);
}

/* Set SOSC configuration register */
static void DRV_SCG_SOSC_SetConfiguration(const scg_sosc_range_t range,
                                          const scg_sosc_gain_t gain,
                                          const scg_sosc_ext_ref_t extRef)
{
    IP_SCG->SOSCCFG = (IP_SCG->SOSCCFG & (~(SCG_SOSCCFG_RANGE_MASK   | SCG_SOSCCFG_HGO_MASK  | SCG_SOSCCFG_EREFS_MASK)))
                                       |    SCG_SOSCCFG_RANGE(range) | SCG_SOSCCFG_HGO(gain) | SCG_SOSCCFG_EREFS(extRef);
}

/* Set SOSC control register */
static void DRV_SCG_SOSC_Control(const bool enableSOSC,
                                 const scg_sosc_monitor_mode_t monitorMode,
                                 const bool lockMode)
{
    bool isMonitorEnabled = false;
    bool resetMode        = false;

    switch (monitorMode)
    {
        case SCG_SOSC_MONITOR_DISABLE:
            {
                isMonitorEnabled = false;
                resetMode        = false;
            }
            break;
        case SCG_SOSC_MONITOR_INT:
            {
                isMonitorEnabled = true;
                resetMode        = false;
            }
            break;
        case SCG_SOSC_MONITOR_RESET:
            {
                isMonitorEnabled = true;
                resetMode        = true;
            }
            break;
        default:
            break;
    }

    IP_SCG->SOSCCSR = (IP_SCG->SOSCCSR & (~(SCG_SOSCCSR_LK_MASK      | SCG_SOSCCSR_SOSCCM_MASK              | SCG_SOSCCSR_SOSCCMRE_MASK       | SCG_SOSCCSR_SOSCEN_MASK)))
                                       |    SCG_SOSCCSR_LK(lockMode) | SCG_SOSCCSR_SOSCCM(isMonitorEnabled) | SCG_SOSCCSR_SOSCCMRE(resetMode) | SCG_SOSCCSR_SOSCEN(enableSOSC);

    if (enableSOSC)
    {
        /* Wait for SOSC clk valid */
        while(!(IP_SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));
    }
    else
    {
        /* Do nothing */
    }
}

/* Configure SPLL module */
static void DRV_SCG_SPLL_Config(const scg_spll_config_t *spllConfig)
{
    if (spllConfig != NULL)
    {
        /* Disable SPLL to configure SPLL */
        IP_SCG->SPLLCSR &= ~(SCG_SPLLCSR_SPLLEN_MASK);

        /* Set SCG asynchronous dividers for SPLL */
        DRV_SCG_SPLL_AsyncConfig(spllConfig->div1, spllConfig->div2);

        /* Set SPLL configuration register */
        DRV_SCG_SPLL_SetConfiguration(spllConfig->prediv, spllConfig->mult);

        /* Set SPLL control register */
        DRV_SCG_SPLL_Control(spllConfig->enableModule,
                             spllConfig->monitorMode,
                             spllConfig->locked);
    }
    else
    {
        /* Do nothing */
    }
}

/* Set SCG asynchronous dividers for SPLL */
static void DRV_SCG_SPLL_AsyncConfig(const scg_async_clock_div_t div1,
                                     const scg_async_clock_div_t div2)
{

    IP_SCG->SPLLDIV = (IP_SCG->SPLLDIV & (~(SCG_SPLLDIV_SPLLDIV1_MASK  | SCG_SPLLDIV_SPLLDIV2_MASK)))
                                       |    SCG_SPLLDIV_SPLLDIV1(div1) | SCG_SPLLDIV_SPLLDIV2(div2);
}

/* Set SPLL configuration register */
static void DRV_SCG_SPLL_SetConfiguration(const scg_spll_prediv_t   divider,
                                          const scg_spll_vco_mult_t multiplier)
{
    IP_SCG->SPLLCFG = (IP_SCG->SPLLCFG & (~(SCG_SPLLCFG_PREDIV_MASK     | SCG_SPLLCFG_MULT_MASK)))
                                       |    SCG_SPLLCFG_PREDIV(divider) | SCG_SPLLCFG_MULT(multiplier);
}

/* Set SPLL control register */
static void DRV_SCG_SPLL_Control(const bool enableSPLL,
                                 const scg_spll_monitor_mode_t monitorMode,
                                 const bool lockMode)
{
    bool isMonitorEnabled = false;
    bool resetMode        = false;

    switch (monitorMode)
    {
        case SCG_SOSC_MONITOR_DISABLE:
            {
                isMonitorEnabled = false;
                resetMode        = false;
            }
            break;
        case SCG_SOSC_MONITOR_INT:
            {
                isMonitorEnabled = true;
                resetMode        = false;
            }
            break;
        case SCG_SOSC_MONITOR_RESET:
            {
                isMonitorEnabled = true;
                resetMode        = true;
            }
            break;
        default:
            break;
    }

    IP_SCG->SPLLCSR = (IP_SCG->SPLLCSR & (~(SCG_SPLLCSR_LK_MASK      | SCG_SPLLCSR_SPLLCM_MASK              | SCG_SPLLCSR_SPLLCMRE_MASK       | SCG_SPLLCSR_SPLLEN_MASK)))
                                       |    SCG_SPLLCSR_LK(lockMode) | SCG_SPLLCSR_SPLLCM(isMonitorEnabled) | SCG_SPLLCSR_SPLLCMRE(resetMode) | SCG_SPLLCSR_SPLLEN(enableSPLL);

    if (enableSPLL)
    {
        /* Wait for SPLL clk valid */
        while(!(IP_SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK));
    }
    else
    {
        /* Do nothing */
    }
}

static clock_status_t DRV_SCG_SystemClockConfig(const scg_sys_clock_config_t * sysClkConfig)
{
          clock_status_t retVal                                             = CLOCK_STATUS_SUCCESS;
          uint32_t       sourceFreq                                         = DRV_Clock_GetSourceFrequency(sysClkConfig->src) >> SYSTEM_CLOCK_SHIFT;
    const uint32_t       maxRunModeClocks[RUN_MODE_COUNT][CLOCK_TYPE_COUNT] = SYSTEM_CLK_MAX_FREQUENCIES_RUN_MODE;
    const uint32_t       coreClockDivider                                   = ((uint32_t)sysClkConfig->divCore) + 1U;
    const uint32_t       busClockDivider                                    = coreClockDivider * (((uint32_t)sysClkConfig->divBus) + 1U);
    const uint32_t       slowClockDivider                                   = coreClockDivider * (((uint32_t)sysClkConfig->divSlow) + 1U);

    /* Checks if the scaled frequency of the source clock exceeds the maximum allowable frequency for each clock type in Run mode */
    if ((sourceFreq > (coreClockDivider * (maxRunModeClocks[(uint32_t)sysClkConfig->src][CORE_CLK_INDEX] >> SYSTEM_CLOCK_SHIFT)))  ||    /* Sys(core) clock */
        (sourceFreq > (busClockDivider  * (maxRunModeClocks[(uint32_t)sysClkConfig->src][BUS_CLK_INDEX]  >> SYSTEM_CLOCK_SHIFT)))  ||    /* Bus clock       */
        (sourceFreq > (slowClockDivider * (maxRunModeClocks[(uint32_t)sysClkConfig->src][SLOW_CLK_INDEX] >> SYSTEM_CLOCK_SHIFT))))       /* Slow clock      */
    {
        /* Configuration for the next system clock source is not valid. */
        retVal = CLOCK_STATUS_ERROR;
    }
    else
    {
        DRV_SCG_SetRunClockControl((uint32_t)sysClkConfig->src, (uint32_t)sysClkConfig->divCore, (uint32_t)sysClkConfig->divBus, (uint32_t)sysClkConfig->divSlow);
    }

    return retVal;
}

static void DRV_SCG_SetRunClockControl(uint32_t source, uint32_t coreDivider, uint32_t busDivider, uint32_t slowDivider)
{
    IP_SCG->RCCR = (IP_SCG->RCCR & (~(SCG_RCCR_SCS_MASK | SCG_RCCR_DIVCORE_MASK         | SCG_RCCR_DIVBUS_MASK        | SCG_RCCR_DIVSLOW_MASK)))
                                 | SCG_RCCR_SCS(source) | SCG_RCCR_DIVCORE(coreDivider) | SCG_RCCR_DIVBUS(busDivider) | SCG_RCCR_DIVSLOW(slowDivider);
}

/* Configure the PCC block */
static void DRV_PCC_Configuration(const pcc_config_t *peripheralClockConfig)
{
    uint32_t index     = 0U;
    uint8_t  PCC_Index = 0U;

    if ((peripheralClockConfig != NULL) && (peripheralClockConfig->peripheralClockConfigs != NULL))
    {
        for (index = 0U; index < peripheralClockConfig->numPeripherals; index++)
        {
            PCC_Index = g_PCC_ClockNameMappings[peripheralClockConfig->peripheralClockConfigs[index].clockName];

            /* Disable PCC to configure PCC */
            IP_PCC->PCCn[PCC_Index] &= ~(PCC_PCCn_CGC_MASK);

            /* Configure the peripheral clock source and clock gate */
            DRV_PCC_SetPeripheralClockControl(peripheralClockConfig->peripheralClockConfigs[index].clockName,
                                              peripheralClockConfig->peripheralClockConfigs[index].clkSrc,
                                              peripheralClockConfig->peripheralClockConfigs[index].enableClock);
        }
    }
    else
    {
        /* Do nothing */
    }
}

/* Set the peripheral clock source */
static void DRV_PCC_SetPeripheralClockControl(const clock_names_t clockName,
                                              const uint8_t       clkSrc,
                                              const bool          isClockEnabled)
{
    /* Get PCC Index */
    uint8_t   PCC_Index        = g_PCC_ClockNameMappings[clockName];
    uint32_t  PCC_Register_tmp = IP_PCC->PCCn[PCC_Index];

    IP_PCC->PCCn[PCC_Index] = (PCC_Register_tmp & (~(PCC_PCCn_PCS_MASK    | PCC_PCCn_CGC_MASK)))
                                                |   (PCC_PCCn_PCS(clkSrc) | PCC_PCCn_CGC(isClockEnabled));
}








clock_status_t DRV_Clock_GetFrequency(clock_names_t clockName, uint32_t * frequency)
{
    clock_status_t retVal = CLOCK_STATUS_SUCCESS;

    /* Frequency of the clock name from SCG */
     if (clockName < SCG_END_OF_CLOCKS)
    {
        retVal = DRV_SCG_GetClockFrequency(clockName, frequency);
    }
    /* Frequency of the clock name from PCC */
    else if ((SIM_END_OF_CLOCKS < clockName) && (clockName < PCC_END_OF_CLOCKS))
    {
        retVal = DRV_PCC_GetClockFrequency(clockName, frequency);
    }
    /* Invalid clock name */
    else
    {
        /* Do nothing */
    }

    return retVal;
}

/* Get the frequency of a SCG clock modules */
static clock_status_t DRV_SCG_GetClockFrequency(clock_names_t clockName, uint32_t * frequency)
{
    clock_status_t retVal = CLOCK_STATUS_SUCCESS;
    uint32_t       Freq   = 0U;

    switch (clockName)
    {
        /* Main clocks */
        case CORE_CLK:
            Freq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_CORE);
            break;
        case BUS_CLK:
            Freq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_BUS);
            break;
        case SLOW_CLK:
            Freq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_SLOW);
            break;

        /* Other internal clocks used by peripherals. */
        case SIRC_CLK:
            Freq = DRV_Clock_GetSIRCFrequency();
            break;
        case FIRC_CLK:
            Freq = DRV_Clock_GetFIRCFrequency();
            break;
        case SOSC_CLK:
            Freq = DRV_Clock_GetSOSCFrequency();
            break;
        case SPLL_CLK:
            Freq = DRV_Clock_GetSPLLFrequency();
            break;
        case SIRCDIV1_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SIRC_CLK, SCG_ASYNC_CLOCK_DIV1);
            break;
        case SIRCDIV2_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SIRC_CLK, SCG_ASYNC_CLOCK_DIV2);
            break;
        case FIRCDIV1_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(FIRC_CLK, SCG_ASYNC_CLOCK_DIV1);
            break;
        case FIRCDIV2_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(FIRC_CLK, SCG_ASYNC_CLOCK_DIV2);
            break;
        case SOSCDIV1_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SOSC_CLK, SCG_ASYNC_CLOCK_DIV1);
            break;
        case SOSCDIV2_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SOSC_CLK, SCG_ASYNC_CLOCK_DIV2);
            break;
        case SPLLDIV1_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SPLL_CLK, SCG_ASYNC_CLOCK_DIV1);
            break;
        case SPLLDIV2_CLK:
            Freq = DRV_Clock_GetSysAsyncFrequency(SPLL_CLK, SCG_ASYNC_CLOCK_DIV2);
            break;
        default:
            retVal = CLOCK_STATUS_UNSUPPORTED;
            break;
    }

    if (frequency != NULL)
    {
        *frequency = Freq;
    }
    else
    {
        /* Do nothing */
    }

    return retVal;
}

static uint32_t DRV_Clock_GetSourceFrequency(scg_system_clock_src_t source)
{
    uint32_t sourceFreq = 0U;

    switch (source)
    {
        case SCG_SYSTEM_CLOCK_SRC_SOSC:
            sourceFreq = DRV_Clock_GetSOSCFrequency();
            break;
        case SCG_SYSTEM_CLOCK_SRC_SIRC:
            sourceFreq = DRV_Clock_GetSIRCFrequency();
            break;
        case SCG_SYSTEM_CLOCK_SRC_FIRC:
            sourceFreq = DRV_Clock_GetFIRCFrequency();
            break;
        case SCG_SYSTEM_CLOCK_SRC_SPLL:
            sourceFreq = DRV_Clock_GetSPLLFrequency();
            break;
        default:
            sourceFreq = 0U;
            break;
    }

    return sourceFreq;
}

static uint32_t DRV_Clock_GetSysClockFrequency(scg_system_clock_type clkType)
{
    uint32_t Freq = 0U;

    /* Get current system clock source */
    switch ((IP_SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT)
    {
        case ((uint32_t)SCG_SYSTEM_CLOCK_SRC_SOSC):
            Freq = DRV_Clock_GetSOSCFrequency();
            break;
        case ((uint32_t)SCG_SYSTEM_CLOCK_SRC_SIRC):
            Freq = DRV_Clock_GetSIRCFrequency();
            break;
        case ((uint32_t)SCG_SYSTEM_CLOCK_SRC_FIRC):
            Freq = DRV_Clock_GetFIRCFrequency();
            break;
        case ((uint32_t)SCG_SYSTEM_CLOCK_SRC_SPLL):
            Freq = DRV_Clock_GetSPLLFrequency();
            break;
        default:
            Freq = 0U;
            break;
    }

    /* Divide frequency by core clock divider (DIVCORE) */
    Freq /= (((IP_SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1U);

    switch (clkType)
    {
        case SCG_SYSTEM_CLOCK_CORE:
            /* Core clock, no further division needed */
            break;
        case SCG_SYSTEM_CLOCK_BUS:
            /* Divide by bus clock divider (DIVBUS) */
            Freq /= (((IP_SCG->CSR & SCG_CSR_DIVBUS_MASK) >> SCG_CSR_DIVBUS_SHIFT) + 1U);
            break;
        case SCG_SYSTEM_CLOCK_SLOW:
            /* Divide by slow clock divider (DIVSLOW) */
            Freq /= (((IP_SCG->CSR & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT) + 1U);
            break;
        default:
            Freq = 0U;
            break;
    }

    return Freq;
}

static uint32_t DRV_Clock_GetSOSCFrequency(void)
{
    uint32_t Freq = 0U;

    if (((IP_SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) >> SCG_SOSCCSR_SOSCVLD_SHIFT) != 0U) /* System OSC clock is valid */
    {
        Freq = FEATURE_SCG_SOSC_FREQ;
    }
    else
    {
        Freq = 0U;
    }

    return Freq;
}

static uint32_t DRV_Clock_GetSIRCFrequency(void)
{
    uint32_t Freq = 0U;

    if (((IP_SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT) != 0U) /* SIRC is valid */
    {
        if (((IP_SCG->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) >> SCG_SIRCCFG_RANGE_SHIFT) != 0U) /* Get SIRC Range */
        {
            Freq = FEATURE_SCG_SIRC_HIGH_RANGE_FREQ;
        }
        else
        {
            Freq = FEATURE_SCG_SIRC_LOW_RANGE_FREQ;
        }
    }
    else
    {
        Freq = 0U;
    }

    return Freq;
}

static uint32_t DRV_Clock_GetFIRCFrequency(void)
{
    uint32_t Freq = 0U;

    if (((IP_SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT) != 0U) /* FIRC is valid */
    {
        Freq = FEATURE_SCG_FIRC_FREQ0;
    }
    else
    {
        Freq = 0U;
    }

    return Freq;
}

static uint32_t DRV_Clock_GetSPLLFrequency(void)
{
    uint32_t Freq = 0U;

    if (((IP_SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK) >> SCG_SPLLCSR_SPLLVLD_SHIFT) != 0U) /* System PLL is valid */
    {

        Freq = DRV_Clock_GetSOSCFrequency();

        if (Freq != 0U) /* If source is valid. */
        {
            Freq /= (((IP_SCG->SPLLCFG & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT) + SCG_SPLL_PREDIV_BASE);    /* Pre-divider */
            Freq *= (((IP_SCG->SPLLCFG & SCG_SPLLCFG_MULT_MASK)   >> SCG_SPLLCFG_MULT_SHIFT)   + SCG_SPLL_MULT_BASE);      /* Multiplier */
            Freq  = Freq >> VCO_DIVIDER;                                                                                   /* Divide VCO by 2 */
        }
        else
        {
            /* Do nothing */
        }
    }
    else
    {
        Freq = 0U;
    }

    return Freq;
}

static uint32_t DRV_Clock_GetSysAsyncFrequency(clock_names_t clockSrc, scg_async_clock_type type)
{
    uint32_t Freq = 0U;
    uint32_t div  = 0U;

    switch (type)
    {
        case SCG_ASYNC_CLOCK_DIV1:
        {
            switch (clockSrc)
            {
                case FIRC_CLK:
                {
                    Freq = DRV_Clock_GetFIRCFrequency();
                    div  = (IP_SCG->FIRCDIV & SCG_FIRCDIV_FIRCDIV1_MASK) >> SCG_FIRCDIV_FIRCDIV1_SHIFT; /* Get the first asynchronous divider for FIRC */
                }
                break;
                case SIRC_CLK:
                {
                    Freq = DRV_Clock_GetSIRCFrequency();
                    div  = (IP_SCG->SIRCDIV & SCG_SIRCDIV_SIRCDIV1_MASK) >> SCG_SIRCDIV_SIRCDIV1_SHIFT; /* Get the first asynchronous divider for SIRC */
                }
                break;
                case SOSC_CLK:
                {
                    Freq = DRV_Clock_GetSOSCFrequency();
                    div  = (IP_SCG->SOSCDIV & SCG_SOSCDIV_SOSCDIV1_MASK) >> SCG_SOSCDIV_SOSCDIV1_SHIFT; /* Get the first asynchronous divider for SOSC */
                }
                break;
                case SPLL_CLK:
                {
                    Freq = DRV_Clock_GetSPLLFrequency();
                    div  = (IP_SCG->SPLLDIV & SCG_SPLLDIV_SPLLDIV1_MASK) >> SCG_SPLLDIV_SPLLDIV1_SHIFT; /* Get the first asynchronous divider for SPLL */
                }
                break;
                default:
                {
                    /* Invalid clock source type */
                    Freq = 0U;
                }
                break;
            }
        }
        break;
        case SCG_ASYNC_CLOCK_DIV2:
        {
            switch (clockSrc)
            {
                case FIRC_CLK:
                {
                    Freq = DRV_Clock_GetFIRCFrequency();
                    div  = (IP_SCG->FIRCDIV & SCG_FIRCDIV_FIRCDIV2_MASK) >> SCG_FIRCDIV_FIRCDIV2_SHIFT; /* Get the second asynchronous divider for FIRC */
                }
                break;
                case SIRC_CLK:
                {
                    Freq = DRV_Clock_GetSIRCFrequency();
                    div  = (IP_SCG->SOSCDIV & SCG_SOSCDIV_SOSCDIV2_MASK) >> SCG_SOSCDIV_SOSCDIV2_SHIFT; /* Get the second asynchronous divider for SIRC */
                }
                break;
                case SOSC_CLK:
                {
                    Freq = DRV_Clock_GetSOSCFrequency();
                    div  = (IP_SCG->SOSCDIV & SCG_SOSCDIV_SOSCDIV2_MASK) >> SCG_SOSCDIV_SOSCDIV2_SHIFT; /* Get the second asynchronous divider for SOSC */
                }
                break;
                case SPLL_CLK:
                {
                    Freq = DRV_Clock_GetSPLLFrequency();
                    div  = (IP_SCG->SPLLDIV & SCG_SPLLDIV_SPLLDIV2_MASK) >> SCG_SPLLDIV_SPLLDIV2_SHIFT; /* Get the second asynchronous divider for SPLL */
                }
                break;
                default:
                {
                    /* Invalid clock source type */
                    Freq = 0U;
                }
                break;
            }
        }
        break;
        default:
            /* Invalid async clock source */
            Freq = 0U;
            break;
    }


    if (div != 0U)
    {
        Freq = (Freq >> (div - 1U));
    }
    else  /* Output disabled */
    {
        Freq = 0U;
    }

    return Freq;
}

static clock_status_t DRV_PCC_GetClockFrequency(clock_names_t clockName, uint32_t * frequency)
{
    clock_status_t retVal          = CLOCK_STATUS_SUCCESS;
    uint32_t       Freq            = 0U;
    uint32_t       interfaceFreq   = 0U;

    retVal = DRV_PCC_CheckClock(clockName);

    if (CLOCK_STATUS_SUCCESS == retVal)
    {
        if ((g_PeripheralFeaturesList[clockName] & HAS_INT_CLOCK_FROM_BUS_CLOCK) != 0U)
        {
            /* Check whether BUS CLOCK is clocked */
            interfaceFreq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_BUS);
            retVal        = (clock_status_t)((interfaceFreq == 0U) ? CLOCK_STATUS_MCU_GATED_OFF : CLOCK_STATUS_SUCCESS);
        }
        else if ((g_PeripheralFeaturesList[clockName] & HAS_INT_CLOCK_FROM_SYS_CLOCK) != 0U)
        {
            /* Check whether SYS CLOCK is clocked. */
            interfaceFreq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_CORE);
            retVal        = (clock_status_t)((interfaceFreq == 0U) ? CLOCK_STATUS_MCU_GATED_OFF : CLOCK_STATUS_SUCCESS);
        }
        else if ((g_PeripheralFeaturesList[clockName] & HAS_INT_CLOCK_FROM_SLOW_CLOCK) != 0U)
        {
            /* Check whether SLOW CLOCK is clocked. */
            interfaceFreq = DRV_Clock_GetSysClockFrequency(SCG_SYSTEM_CLOCK_SLOW);
            retVal        = (clock_status_t)((interfaceFreq == 0U) ? CLOCK_STATUS_MCU_GATED_OFF : CLOCK_STATUS_SUCCESS);
        }
        else
        {
            /* It's an issue in peripheral features list, each peripheral must have one interface clock. */
        }

        if (retVal == CLOCK_STATUS_SUCCESS)
        {
            /* Check whether peripheral has protocol clock (functional clock) */
            if ((g_PeripheralFeaturesList[clockName] & (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_PROTOCOL_CLOCK_FROM_ASYNC2)) != 0U)
            {
                if ((g_PeripheralFeaturesList[clockName] & HAS_PROTOCOL_CLOCK_FROM_ASYNC1) != 0U)
                {
                    /* Check whether the functional clock is clocked */
                    Freq = DRV_PCC_GetPeripheralClock(clockName, SCG_ASYNC_CLOCK_DIV1);
                }
                else
                {
                    /* Do nothing */
                }

                if ((g_PeripheralFeaturesList[clockName] & HAS_PROTOCOL_CLOCK_FROM_ASYNC2) != 0U)
                {
                    /* Check whether the functional clock is clocked */
                    Freq = DRV_PCC_GetPeripheralClock(clockName, SCG_ASYNC_CLOCK_DIV2);
                }
                else
                {
                    /* Do nothing */
                }

                if (Freq == 0U)
                {
                    retVal = CLOCK_STATUS_MCU_GATED_OFF;
                }
                else
                {
                    /* Do nothing */
                }
            }
            else
            {
                Freq = interfaceFreq;
            }
        }
        else
        {
            /* Do nothing */
        }
    }
    else
    {
        /* Do nothing */
    }

    if (frequency != NULL)
    {
        *frequency = Freq;
    }
    else
    {
        /* Do nothing */
    }

    return retVal;
}

static clock_status_t DRV_PCC_CheckClock(clock_names_t clockName)
{
    clock_status_t retVal = CLOCK_STATUS_SUCCESS;

    /* Invalid PCC clock names. */
    if ((clockName <= SIM_END_OF_CLOCKS)             ||
        (clockName == PCC_END_OF_BUS_CLOCKS)         ||
        (clockName == PCC_END_OF_SYS_CLOCKS)         ||
        (clockName == PCC_END_OF_SLOW_CLOCKS)        ||
        (clockName == PCC_END_OF_ASYNCH_DIV1_CLOCKS) ||
        (clockName == PCC_END_OF_ASYNCH_DIV2_CLOCKS))
    {
        retVal = CLOCK_STATUS_UNSUPPORTED;
    }
    else if (((IP_PCC->PCCn[g_PCC_ClockNameMappings[clockName]] & PCC_PCCn_CGC_MASK) >> PCC_PCCn_CGC_SHIFT) == 0U) /* Get the clock gate mode */
    {
        retVal = CLOCK_STATUS_MCU_GATED_OFF;
    }
    else
    {
        retVal = CLOCK_STATUS_SUCCESS;
    }

    return retVal;
}

static uint32_t DRV_PCC_GetPeripheralClock(clock_names_t        clockName,
                                           scg_async_clock_type divider)
{
    uint32_t Freq = 0U;

    if ((IP_PCC->PCCn[g_PCC_ClockNameMappings[clockName]] & PCC_PCCn_CGC_MASK) >> PCC_PCCn_CGC_SHIFT) /* Get the clock gate mode */
    {
        /* Get the selection of a clock source for a specific peripheral */
        switch ((IP_PCC->PCCn[g_PCC_ClockNameMappings[clockName]] & PCC_PCCn_PCS_MASK) >> PCC_PCCn_PCS_SHIFT)
        {
            case (uint32_t)SCG_SYSTEM_CLOCK_SRC_SOSC:
                Freq = DRV_Clock_GetSysAsyncFrequency(SOSC_CLK, divider);
                break;
            case (uint32_t)SCG_SYSTEM_CLOCK_SRC_SIRC:
                Freq = DRV_Clock_GetSysAsyncFrequency(SIRC_CLK, divider);
                break;
            case (uint32_t)SCG_SYSTEM_CLOCK_SRC_FIRC:
                Freq = DRV_Clock_GetSysAsyncFrequency(FIRC_CLK, divider);
                break;
            case (uint32_t)SCG_SYSTEM_CLOCK_SRC_SPLL:
                Freq = DRV_Clock_GetSysAsyncFrequency(SPLL_CLK, divider);
                break;
            default:
                break;
        }
    }

    return Freq;
}

/*******************************************************************************
 * End Of File
 ******************************************************************************/
