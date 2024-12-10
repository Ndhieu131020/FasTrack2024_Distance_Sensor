#include "DRV_S32K144_FLEXCAN.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define MIN_BITRATE    1000000U
#define MIN_SAMPLE     100U
#define SAMPLE_POINT   80U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void FLEXCAN_BitrateToTimeSeg(uint32_t bitrate, uint32_t clkFreq, flexcan_time_segment_t *timeSeg);
static flexcan_freeze_mode_status_t FLEXCAN_GetFreezeMode(uint8_t instance);
static void FLEXCAN_EnterFreezeMode(uint8_t instance);
static void FLEXCAN_ExitFreezeMode(uint8_t instance);
static void FLEXCAN_SetBitrate(uint8_t instance, flexcan_time_segment_t *timeSeg);
static void FLEXCAN_EnableModule(uint8_t instance);
static void FLEXCAN_DisableModule(uint8_t instance);
static void FLEXCAN_SelectClockSource(uint8_t instance, flexcan_clock_source_t clkSrc);
static uint8_t FLEXCAN_GetMaxMbNum(uint8_t instance);
static void FLEXCAN_ClearRAM(uint8_t instance);
static void FLEXCAN_InitMb(uint8_t instance);
static void FLEXCAN_ConfigRxMaskType(uint8_t instance, flexcan_rx_mask_type_t rxMaskType);
static void FLEXCAN_SetOperationModes(uint8_t instance, flexcan_operation_modes_t flexcanMode);
static void FLEXCAN_Mb_IRQHandler(uint8_t instance);
static void FLEXCAN_BusOff_IRQHandler(uint8_t instance);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for CAN instances. */
FLEXCAN_Type *g_flexcanBase[FLEXCAN_INSTANCE_COUNT] = IP_FLEXCAN_BASE_PTRS;

/* Pointer to runtime handle structure.*/
flexcan_handle_t *g_flexcanHandle[FLEXCAN_INSTANCE_COUNT] = {NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/

/* INIT */

/**
  * @brief      Calculate the CAN bit timing segments based on bitrate and clock frequency
  * @param[in]  bitrate: Desired bitrate for CAN communication
  * @param[in]  clkFreq: The clock frequency provided to the FlexCAN module
  * @param[out] timeSeg: Pointer to a structure that will hold the computed time segment values.
  * @retval     None
  */
static void FLEXCAN_BitrateToTimeSeg(uint32_t bitrate, uint32_t clkFreq, flexcan_time_segment_t *timeSeg)
{
    uint32_t tmpBitrate = 0U, dBitrate = 0U, tmpSample = 0U, dSample = 0U, tmpPhaseSeg1 = 0U, tmpPhaseSeg2 = 0U, tmpPropSeg = 0U, tmpPresdiv = 0U, tSeg1 = 0U, tSeg2 = 0U, numTq = 0U;
    uint32_t dBitrateMin = MIN_BITRATE, dSampleMin = MIN_SAMPLE, samplePoint = SAMPLE_POINT; /* Desired sample point percentage */
    uint32_t presDiv = 0U, propSeg = 0U, phaseSeg1 = 0U, phaseSeg2 = 0U;
    uint8_t proceedFlag = 1U; /* Flag to determine if the current configuration is valid */
    uint8_t exitFlag = 1U;    /* Flag to exit the loop when the best configuration is found */
    /* Iterate through all possible prescaler divider values */
    for (tmpPresdiv = 0U; tmpPresdiv <= FLEXCAN_PRESDIV_MAX && exitFlag; tmpPresdiv++)
    {
        proceedFlag = 1U;
        /* Calculate the number of time quanta (Tq) for the given prescaler value */
        numTq = (clkFreq) / ((bitrate) * (tmpPresdiv + 1U));
        /* Compute temporary bitrate based on the current prescaler and numTq */
        tmpBitrate = (clkFreq) / ((numTq) * (tmpPresdiv + 1U));
        /* Check if the number of Tq is within the allowed range */
        if (numTq >= FLEXCAN_NUM_TQ_MIN && numTq <= FLEXCAN_NUM_TQ_MAX)
        {
            tSeg1 = (numTq * samplePoint / 100U) - 1U;
            tSeg2 = numTq - tSeg1 - 1U;
            /* Adjust TSEG1 and TSEG2 to ensure they are within the valid range */
            while (tSeg1 > FLEXCAN_TSEG1_MAX || tSeg2 < FLEXCAN_TSEG2_MIN)
            {
                tSeg1 = tSeg1 - 1U;
                tSeg2 = tSeg2 + 1U;
            }
            tmpPhaseSeg2 = tSeg2 - 1U;
            tmpPhaseSeg1 = tmpPhaseSeg2;
            tmpPropSeg = tSeg1 - tmpPhaseSeg1 - 2U;
            /* Ensure propagation segment is within valid range */
            while (tmpPropSeg <= 0U)
            {
                tmpPropSeg = tmpPropSeg + 1U;
                tmpPhaseSeg1 = tmpPhaseSeg1 - 1U;
            }
            while (tmpPropSeg > FLEXCAN_PROPSEG_MAX)
            {
                tmpPropSeg = tmpPropSeg - 1U;
                tmpPhaseSeg1 = tmpPhaseSeg1 + 1U;
            }
            if ((tSeg1 > FLEXCAN_TSEG1_MAX) || (tSeg1 < FLEXCAN_TSEG1_MIN) || (tSeg2 > FLEXCAN_TSEG2_MAX) || (tSeg2 < FLEXCAN_TSEG2_MIN) || (tmpPropSeg > FLEXCAN_PROPSEG_MAX) || (tmpPhaseSeg1 > FLEXCAN_PSEG1_MAX) || (tmpPhaseSeg2 > FLEXCAN_PSEG2_MAX) || (tmpPhaseSeg2 < FLEXCAN_PSEG2_MIN))
            {
                proceedFlag = 0U; /* Invalid configuration */
            }
            /*  If the configuration is valid, calculate difference and select the best match */
            if (proceedFlag == 1U)
            {
                tmpSample = ((tSeg1 + 1U) * 100U) / numTq;
                dSample = tmpSample > samplePoint ? (tmpSample - samplePoint) : (samplePoint - tmpSample);
                dBitrate = tmpBitrate > bitrate ? (tmpBitrate - bitrate) : (bitrate - tmpBitrate);
                if ((dBitrate < dBitrateMin) || ((dBitrate == dBitrateMin) && (dSample < dSampleMin)))
                {
                    dBitrateMin = dBitrate;
                    dSampleMin = dSample;
                    presDiv = tmpPresdiv;
                    propSeg = tmpPropSeg;
                    phaseSeg1 = tmpPhaseSeg1;
                    phaseSeg2 = tmpPhaseSeg2;
                    if ((dBitrate == 0U) && (dSample <= 1U))
                    {
                        exitFlag = 0U;
                    }
                }
            }
        }
    }
    /* Update the time segment structure with the best configuration found */
    timeSeg->presDiv = presDiv;
    timeSeg->propSeg = propSeg;
    timeSeg->phaseSeg1 = phaseSeg1;
    timeSeg->phaseSeg2 = phaseSeg2;
    if (phaseSeg1 < FLEXCAN_RJW_MAX)
    {
        timeSeg->rJumpWidth = phaseSeg1;
    }
    else
    {
        timeSeg->rJumpWidth = FLEXCAN_RJW_MAX;
    }
}

/**
  * @brief      Check the current state whether module is in Freezw Mode or not.
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     FLEXCAN_IN_FREEZE_MODE: The FlexCAN module is in freeze mode.
  * @retval     FLEXCAN_OUT_FREEZE_MODE: The FlexCAN module is not in freeze mode.
  */
static flexcan_freeze_mode_status_t FLEXCAN_GetFreezeMode(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    if (((base->MCR & FLEXCAN_MCR_FRZACK_MASK) >> (FLEXCAN_MCR_FRZACK_SHIFT)) == 1U)
    {
        return FLEXCAN_IN_FREEZE_MODE;
    }
    return FLEXCAN_OUT_FREEZE_MODE;
}

/**
  * @brief      Allow module go to freeze mode for safe configuration
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_EnterFreezeMode(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    base->MCR = (base->MCR & ~FLEXCAN_MCR_FRZACK_MASK) | FLEXCAN_MCR_FRZ(1U);
    base->MCR = (base->MCR & ~FLEXCAN_MCR_HALT_MASK) | FLEXCAN_MCR_HALT(1U);
    while (((base->MCR & FLEXCAN_MCR_FRZACK_MASK) >> FLEXCAN_MCR_FRZACK_SHIFT) != 1U)
    {
    }
}

/**
  * @brief      Allow module exit freeze mode for normal operation
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_ExitFreezeMode(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    base->MCR = (base->MCR & ~FLEXCAN_MCR_HALT_MASK) | FLEXCAN_MCR_HALT(0U);
    base->MCR = (base->MCR & ~FLEXCAN_MCR_FRZACK_MASK) | FLEXCAN_MCR_FRZ(0U);
    while (((base->MCR & FLEXCAN_MCR_FRZACK_MASK) >> FLEXCAN_MCR_FRZACK_SHIFT) != 0U)
    {
    }
}

/**
  * @brief      Configures the CAN bitrate
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  timeSeg: Pointer to a structure containing bit timing configurations
  * @retval     None
  */
static void FLEXCAN_SetBitrate(uint8_t instance, flexcan_time_segment_t *timeSeg)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    base->CTRL1 = ((base->CTRL1) & ~((FLEXCAN_CTRL1_PRESDIV_MASK) | (FLEXCAN_CTRL1_PROPSEG_MASK) | (FLEXCAN_CTRL1_PSEG1_MASK) | (FLEXCAN_CTRL1_PSEG2_MASK) | (FLEXCAN_CTRL1_RJW_MASK)));
    base->CTRL1 = ((base->CTRL1) | (FLEXCAN_CTRL1_PRESDIV(timeSeg->presDiv) | FLEXCAN_CTRL1_PROPSEG(timeSeg->propSeg) | FLEXCAN_CTRL1_PSEG1(timeSeg->phaseSeg1) | FLEXCAN_CTRL1_PSEG2(timeSeg->phaseSeg2) | FLEXCAN_CTRL1_RJW(timeSeg->rJumpWidth)));
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }
}

/**
  * @brief      Enable the FLEXCAN module
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_EnableModule(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    base->MCR = ((base->MCR) & ~(FLEXCAN_MCR_MDIS_MASK)) | (FLEXCAN_MCR_MDIS(0U));
}

/**
  * @brief      Disable the FLEXCAN module
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_DisableModule(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    base->MCR = ((base->MCR) & ~(FLEXCAN_MCR_MDIS_MASK)) | (FLEXCAN_MCR_MDIS(1U));
}

/**
  * @brief      Configures the clock source for the FLEXCAN module
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  clkSrc: Clock source to be configured (Oscillator or Peripheral Clock)
  * @retval     None
  */
static void FLEXCAN_SelectClockSource(uint8_t instance, flexcan_clock_source_t clkSrc)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    if (clkSrc == FLEXCAN_CLK_SRC_OSC)
    {
        base->CTRL1 = (base->CTRL1 & ~(FLEXCAN_CTRL1_CLKSRC_MASK)) | (FLEXCAN_CTRL1_CLKSRC(0U));
    }
    else if (clkSrc == FLEXCAN_CLK_SRC_PERIPH)
    {
        base->CTRL1 = (base->CTRL1 & ~(FLEXCAN_CTRL1_CLKSRC_MASK)) | (FLEXCAN_CTRL1_CLKSRC(1U));
    }
    else
    {
    }
}

/**
  * @brief      determines the maximum number of message buffers (MBs) available based on the specific FLEXCAN instance
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     Number of message buffers supported by the specified module
  */
static uint8_t FLEXCAN_GetMaxMbNum(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint8_t ret = 0U;
    if (base == IP_FLEXCAN0)
    {
        ret = 32U;
    }
    else if (base == IP_FLEXCAN1)
    {
        ret = 16U;
    }
    else if (base == IP_FLEXCAN2)
    {
        ret = 16U;
    }
    else
    {
    }
    return ret;
}

/**
 * @brief       This function clears the message buffer (MB) and RXIMR (Receive Individual Mask Registers)
 *              regions in RAM by setting all words to zero.
 * @param[in]   instance: Identifies which FlexCAN module
 * @retval      None
 */
static void FLEXCAN_ClearRAM(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint8_t RAM_size = FLEXCAN_GetMaxMbNum(instance) * 4U; /* numMB * 4 = total words */
    uint8_t RXIMR_size = FLEXCAN_GetMaxMbNum(instance);
    uint8_t i = 0U;
    /* Clear MB region */
    for (i = 0; i < RAM_size; i++)
    {
        base->RAMn[i] = 0x0;
    }
    /* Clear RXIMR region */
    for (i = 0; i < RXIMR_size; i++)
    {
        base->RXIMR[i] = 0x0;
    }
}

/**
 * @brief       Initializes all message buffers (MBs), ready to config for transfer (transmit/ receive)
 * @param[in]   instance: Identifies which FlexCAN module
 * @retval      None
 */
static void FLEXCAN_InitMb(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint8_t maxMB = FLEXCAN_GetMaxMbNum(instance);
    uint8_t i = 0U;
    for (i = 0U; i < maxMB; i++)
    {
        /* Mailbox = 128 words, each MB is 4 words */
        base->RAMn[i * MESSAGE_BUFFER_SIZE + 0U] = 0x0; /* Control and Status word*/
        base->RAMn[i * MESSAGE_BUFFER_SIZE + 1U] = 0x0; /* ID word */
        base->RAMn[i * MESSAGE_BUFFER_SIZE + 2U] = 0x0; /* Data word 1 */
        base->RAMn[i * MESSAGE_BUFFER_SIZE + 3U] = 0x0; /* Data word 2 */
    }
}

/**
 * @brief       Configures the type of receive mask used by the FLEXCAN module.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   rxMaskType: Receive mask type (individual or global).
 * @retval      None
 */
static void FLEXCAN_ConfigRxMaskType(uint8_t instance, flexcan_rx_mask_type_t rxMaskType)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    if (rxMaskType == FLEXCAN_RX_MASK_INDIVIDUAL)
    {
        base->MCR = (base->MCR & ~(FLEXCAN_MCR_IRMQ_MASK)) | FLEXCAN_MCR_IRMQ(1U);
    }
    else if (rxMaskType == FLEXCAN_RX_MASK_GLOBAL)
    {
        base->MCR = (base->MCR & ~(FLEXCAN_MCR_IRMQ_MASK)) | FLEXCAN_MCR_IRMQ(0U);
    }
    else
    {
    }
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }
}

/**
 * @brief       Configures the operation mode of the FLEXCAN module
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   flexcanMode:  Desired operation mode.
 * @retval      None
 */
static void FLEXCAN_SetOperationModes(uint8_t instance, flexcan_operation_modes_t flexcanMode)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    switch (flexcanMode)
    {
    case FLEXCAN_FREEZE_MODE:
        FLEXCAN_EnterFreezeMode(instance);
        break;
    case FLEXCAN_DISABLE_MODE:
        base->MCR = (base->MCR & ~(FLEXCAN_MCR_MDIS_MASK)) | FLEXCAN_MCR_MDIS(1U);
        break;
    case FLEXCAN_NORMAL_MODE:
        base->CTRL1 = (base->CTRL1 & ~FLEXCAN_CTRL1_LOM_MASK) | FLEXCAN_CTRL1_LOM(0U);
        base->CTRL1 = (base->CTRL1 & ~FLEXCAN_CTRL1_LPB_MASK) | FLEXCAN_CTRL1_LPB(0U);
        break;
    case FLEXCAN_LOOPBACK_MODE:
        base->CTRL1 = (base->CTRL1 & ~(FLEXCAN_CTRL1_LPB_MASK)) | FLEXCAN_CTRL1_LPB(1U);
        base->MCR = (base->MCR & ~(FLEXCAN_MCR_SRXDIS_MASK)) | FLEXCAN_MCR_SRXDIS(0U);
        break;
    case FLEXCAN_LISTEN_ONLY_MODE:
        base->CTRL1 = (base->CTRL1 & ~(FLEXCAN_CTRL1_LOM_MASK)) | FLEXCAN_CTRL1_LOM(1U);
        break;
    default:
        break;
    }
}

/**
 * @brief       Enable the interrupt for a specific message buffer.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   mbIdx:    Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_EnableMbInt(uint8_t instance, uint8_t mbIdx)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint32_t tmp = 1U << (uint32_t)mbIdx;
    base->IMASK1 = ((base->IMASK1) | (tmp));
}

/**
 * @brief       Disable the interrupt for a specific message buffer.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   mbIdx:    Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_DisableMbInt(uint8_t instance, uint8_t mbIdx)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint32_t tmp = 1U << (uint32_t)mbIdx;
    base->IMASK1 = ((base->IMASK1) & (~tmp));
}

/**
 * @brief       Initializes the FLEXCAN module with the specified configuration.
 *              This function performs the following initialization steps:
 *              - Disables the module to configure clock source.
 *              - Sets the clock source and enables the module.
 *              - Configures the RX mask type.
 *              - Sets the CAN bitrate using the specified configuration.
 *              - Clears and initializes the RAM regions and message buffers.
 *              - Sets the operation mode.
 *              - Prepares the handle for callback functionality.
 *
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   config:   Pointer to the FLEXCAN configuration structure.
 * @param[out]  handle:   Pointer to the FLEXCAN handle structure.
 * @retval      None
 */
void DRV_FLEXCAN_Init(uint8_t instance, flexcan_module_config_t *config, flexcan_handle_t *handle)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    flexcan_time_segment_t timeSeg;
    uint8_t i = 0U;
    uint8_t flexcanMaxMBNum = FLEXCAN_GetMaxMbNum(instance);
    /* Disable module to config clock source*/
    FLEXCAN_DisableModule(instance);
    /* Select clock source */
    FLEXCAN_SelectClockSource(instance, config->clkSrc);
    /* Enable module */
    FLEXCAN_EnableModule(instance);
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    FLEXCAN_ConfigRxMaskType(instance, config->rxMaskType);
    if (config->flexcanMode != FLEXCAN_LOOPBACK_MODE)
    {
        base->MCR = (base->MCR & ~(FLEXCAN_MCR_SRXDIS_MASK)) | FLEXCAN_MCR_SRXDIS(1U);
    }
    /*Set bitrate*/
    FLEXCAN_BitrateToTimeSeg(config->bitrate, config->flexcanClkFreq, &timeSeg);
    FLEXCAN_SetBitrate(instance, &timeSeg);

    /* Initialize MBs to inactive */
    FLEXCAN_ClearRAM(instance);
    FLEXCAN_InitMb(instance);

    /*Set operation mode*/
    FLEXCAN_SetOperationModes(instance, config->flexcanMode);

    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }

    for (i = 0U; i < flexcanMaxMBNum; i++)
    {
        handle->mbs[i] = NULL;
    }
    /* Prepare for callback */
    handle->mb_callback = NULL;
    handle->bus_off_callback = NULL;
    g_flexcanHandle[instance] = handle;
}

/* RECEIVE */
/**
 * @brief       Configures the global mask for RX message buffers.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   idType:    ID type (standard or extended).
 * @param[in]   mask:      Global mask value.
 * @retval      None
 */
void DRV_FLEXCAN_SetRxMbGlobalMask(uint8_t instance, flexcan_mb_id_type_t idType, uint32_t mask)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    if (idType == FLEXCAN_MB_ID_STD)
    {
        base->RXMGMASK = ((uint32_t)((uint32_t)(mask << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK));
    }
    else if (idType == FLEXCAN_MB_ID_EXT)
    {
        base->RXMGMASK = ((uint32_t)((uint32_t)(mask << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK | FLEXCAN_MB_ID_EXT_MASK));
    }
    else
    {
    }
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }
}

/**
 * @brief       Configures an individual mask for a specific RX message buffer.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   idType:    ID type (standard or extended)
 * @param[in]   mbIdx:     Message buffer index.
 * @param[in]   mask:      Individual mask value.
 * @retval      None
 */
void DRV_FLEXCAN_SetRxMbIndividualMask(uint8_t instance, flexcan_mb_id_type_t idType, uint8_t mbIdx, uint32_t mask)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    if (idType == FLEXCAN_MB_ID_STD)
    {
        base->RXIMR[mbIdx] = ((uint32_t)((uint32_t)(mask << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK));
    }
    else if (idType == FLEXCAN_MB_ID_EXT)
    {
        base->RXIMR[mbIdx] = ((uint32_t)((uint32_t)(mask << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK | FLEXCAN_MB_ID_EXT_MASK));
    }
    else
    {
    }
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }
}

/**
 * @brief       Checks the interrupt flag for the specified message buffer index and returns its status.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   mbIdx:     Message buffer index.
 * @retval      Interrupt flag status (1 if set, 0 otherwise).
 */
uint8_t DRV_FLEXCAN_GetMbIntFlag(uint8_t instance, uint8_t mbIdx)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    uint8_t flag = 0;
    uint32_t mask = base->IMASK1;
    flag = (uint8_t)((((base->IFLAG1) & mask) >> mbIdx) & 1U);
    return flag;
}

/**
 * @brief       Clears the interrupt flag for a specific message buffer
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   mbIdx:     Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_ClearMbIntFlag(uint8_t instance, uint8_t mbIdx)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    base->IFLAG1 &= (uint32_t)((uint32_t)(1U) << mbIdx);
}

/**
  * @brief      Configure a Receive Message Buffer
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[in]  rx_mb: Pointer to message buffer configuration structure
  * @param[in]  mb_id: Message buffer ID
  * @retval     None
  */
void DRV_FLEXCAN_ConfigRxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *rx_mb, uint32_t mb_id)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    DRV_FLEXCAN_ClearMbIntFlag(instance, mbIdx);
    /* Congif IDE bit */
    if (rx_mb->idType == FLEXCAN_MB_ID_STD)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = ((uint32_t)((uint32_t)(0U << FLEXCAN_MB_IDE_SHIFT)) & (FLEXCAN_MB_IDE_MASK));
    }
    else if (rx_mb->idType == FLEXCAN_MB_ID_EXT)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = ((uint32_t)((uint32_t)(1U << FLEXCAN_MB_IDE_SHIFT)) & (FLEXCAN_MB_IDE_MASK));
    }
    else
    {
    }
    /* Config data length */
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] & ~(FLEXCAN_MB_DLC_MASK)) | FLEXCAN_MB_DLC(rx_mb->dataLength);
    /* Config ID */
    if (rx_mb->idType == FLEXCAN_MB_ID_STD)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] = ((uint32_t)((uint32_t)(mb_id << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK));
    }
    else if (rx_mb->idType == FLEXCAN_MB_ID_EXT)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] = ((uint32_t)((uint32_t)(mb_id << FLEXCAN_MB_ID_EXT_SHIFT)) & (FLEXCAN_MB_ID_EXT_MASK));
    }
    else
    {
    }
    /* Write EMPTY code to active mailbox */
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] & ~(FLEXCAN_MB_CODE_MASK)) | FLEXCAN_MB_CODE(FLEXCAN_RX_EMPTY);
}

/**
  * @brief      Receive a CAN message in blocking manner
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[out] data: Pointer to received message structure
  * @retval     None
  */
void DRV_FLEXCAN_Receive(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    /* Check BUSY by checking CODE field. if EMPTY == BUSY deasserted */
    data->cs = base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0];
    data->code = ((data->cs & FLEXCAN_MB_CODE_MASK) >> FLEXCAN_MB_CODE_SHIFT);
    while (data->code == FLEXCAN_RX_BUSY)
    {
        data->cs = base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0];
        data->code = ((data->cs & FLEXCAN_MB_CODE_MASK) >> FLEXCAN_MB_CODE_SHIFT);
    }
    /*Read content of the mail box*/
    data->msgId = ((base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] & FLEXCAN_MB_ID_STD_MASK) >> FLEXCAN_MB_ID_STD_SHIFT);
    data->dataLength = ((data->cs & FLEXCAN_MB_DLC_MASK) >> FLEXCAN_MB_DLC_SHIFT);
    data->data[0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 2U]);
    data->data[1U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 3U]);
    /*Clear flag*/
    DRV_FLEXCAN_ClearMbIntFlag(instance, mbIdx);
    /*Read the free running timer to unlock MB*/
    (void)base->TIMER;
}

/**
  * @brief      Receive a CAN message using interrupt
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[out] data: Pointer to received message structure
  * @retval     None
  */
void DRV_FLEXCAN_ReceiveInt(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_handle_t *handle = g_flexcanHandle[instance];
    handle->mbs[mbIdx] = data;

    /* Lock MB by reading it */
    volatile uint32_t *flexcan_mb = &(base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE]);
    (void)*flexcan_mb;
    /*Read content of the mail box*/
    handle->mbs[mbIdx]->cs = base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U];
    handle->mbs[mbIdx]->msgId = ((base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] & FLEXCAN_MB_ID_STD_MASK) >> FLEXCAN_MB_ID_STD_SHIFT);
    handle->mbs[mbIdx]->dataLength = ((handle->mbs[mbIdx]->cs & FLEXCAN_MB_DLC_MASK) >> FLEXCAN_MB_DLC_SHIFT);
    handle->mbs[mbIdx]->data[0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 2U]);
    handle->mbs[mbIdx]->data[1U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 3U]);
    /* Unlock MB by reading Free Running Timer*/
    (void)base->TIMER;
}

/**
  * @brief      Message Buffer Interrupt Handler
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_Mb_IRQHandler(uint8_t instance)
{
    flexcan_handle_t *handle = g_flexcanHandle[instance];
    /* Invoke callback if registered */
    if (handle->mb_callback != NULL)
    {
        handle->mb_callback();
    }
}

/* SEND */
/**
  * @brief      Configure a Transmit Message Buffer
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[in]  tx_mb: Pointer to message buffer configuration structure
  * @param[in]  mb_id: Message buffer ID
  * @retval     None
  */
void DRV_FLEXCAN_ConfigTxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *tx_mb, uint32_t mb_id)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    DRV_FLEXCAN_ClearMbIntFlag(instance, mbIdx);
    /* Congif IDE bit */
    if (tx_mb->idType == FLEXCAN_MB_ID_STD)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = ((uint32_t)((uint32_t)(0U << FLEXCAN_MB_IDE_SHIFT)) & (FLEXCAN_MB_IDE_MASK));
    }
    else if (tx_mb->idType == FLEXCAN_MB_ID_EXT)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = ((uint32_t)((uint32_t)(1U << FLEXCAN_MB_IDE_SHIFT)) & (FLEXCAN_MB_IDE_MASK));
    }
    else
    {
    }
    /* Config data length */
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] & ~(FLEXCAN_MB_DLC_MASK)) | FLEXCAN_MB_DLC(tx_mb->dataLength);
    /* Config ID */
    if (tx_mb->idType == FLEXCAN_MB_ID_STD)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] = ((uint32_t)((uint32_t)(mb_id << FLEXCAN_MB_ID_STD_SHIFT)) & (FLEXCAN_MB_ID_STD_MASK));
    }
    else if (tx_mb->idType == FLEXCAN_MB_ID_EXT)
    {
        base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 1U] = ((uint32_t)((uint32_t)(mb_id << FLEXCAN_MB_ID_EXT_SHIFT)) & (FLEXCAN_MB_ID_EXT_MASK));
    }
    else
    {
    }
    /* Write EMPTY code to active mailbox */
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] & ~(FLEXCAN_MB_CODE_MASK)) | FLEXCAN_MB_CODE(FLEXCAN_TX_INACTIVE);
}

/**
  * @brief      Transmit a CAN message
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx: Mailbox index
  * @param[in]  data: Pointer to message structure to transmit
  * @retval     None
  */
void DRV_FLEXCAN_Transmit(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    /*Clear flag*/
    DRV_FLEXCAN_ClearMbIntFlag(instance, mbIdx);
    /*Prepare content of the mail box*/
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 2U] = data->data[0];
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 3U] = data->data[1];
    /* Write TX_DATA code to transmit */
    base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] = (base->RAMn[mbIdx * MESSAGE_BUFFER_SIZE + 0U] & ~(FLEXCAN_MB_CODE_MASK)) | FLEXCAN_MB_CODE(FLEXCAN_TX_DATA);
}

/*BUSOFF*/
/**
  * @brief      Clear Bus-Off Interrupt Flag
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_ClearBusOffIntFlag(uint8_t instance)
{
    FLEXCAN_Type *base = g_flexcanBase[instance];
    if ((base->ESR1 & FLEXCAN_ESR1_BOFFINT_MASK) != 0U)
    {
        base->ESR1 = (uint32_t)(1U << FLEXCAN_ESR1_BOFFINT_SHIFT);
    }
}

/**
  * @brief      Bus-Off Interrupt Handler
  * @param[in]  instance: Identifies which FlexCAN module
  * @retval     None
  */
static void FLEXCAN_BusOff_IRQHandler(uint8_t instance)
{
    flexcan_handle_t *handle = g_flexcanHandle[instance];
    if (handle->bus_off_callback != NULL)
    {
        handle->bus_off_callback();
    }
    FLEXCAN_ClearBusOffIntFlag(instance);
}

/* REGISTER CALL BACK FUNCTION */
/**
  * @brief      Register Message Buffer Callback Function
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  cb_ptr Pointer to callback function
  * @retval     None
  */
void DRV_FLEXCAN_RegisterMbCallback(uint8_t instance, void (*cb_ptr)(void))
{
    flexcan_handle_t *handle = g_flexcanHandle[instance];
    handle->mb_callback = cb_ptr;
}

/**
  * @brief      Register Bus off Callback Function
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  cb_ptr: Pointer to callback function
  * @retval     None
  */
void DRV_FLEXCAN_RegisterBusOffCallback(uint8_t instance, void (*cb_ptr)(void))
{
    flexcan_freeze_mode_status_t freeze = FLEXCAN_GetFreezeMode(instance);
    FLEXCAN_Type *base = g_flexcanBase[instance];
    flexcan_handle_t *handle = g_flexcanHandle[instance];
    handle->bus_off_callback = cb_ptr;
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_EnterFreezeMode(instance);
    }
    /* enable busoff interrupt if busoff callback function exists */
    if (cb_ptr != NULL)
    {
        base->CTRL1 = (base->CTRL1) | (FLEXCAN_CTRL1_BOFFMSK_MASK);
    }
    else
    {
        base->CTRL1 = (base->CTRL1) & ~(FLEXCAN_CTRL1_BOFFMSK_MASK);
    }
    if (freeze == FLEXCAN_OUT_FREEZE_MODE)
    {
        FLEXCAN_ExitFreezeMode(instance);
    }
}

/* REAL HANDLER */
void CAN0_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_Mb_IRQHandler(0U);
}

void CAN0_ORed_16_31_MB_IRQHandler(void)
{
    FLEXCAN_Mb_IRQHandler(0U);
}

void CAN0_ORed_IRQHandler(void)
{
    FLEXCAN_BusOff_IRQHandler(0U);
}

void CAN1_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_Mb_IRQHandler(1U);
}

void CAN1_ORed_IRQHandler(void)
{
    FLEXCAN_BusOff_IRQHandler(1U);
}

void CAN2_ORed_0_15_MB_IRQHandler(void)
{
    FLEXCAN_Mb_IRQHandler(2U);
}

void CAN2_ORed_IRQHandler(void)
{
    FLEXCAN_BusOff_IRQHandler(2U);
}
