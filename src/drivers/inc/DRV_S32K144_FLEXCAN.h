/*
 *  Filename: DRV_S32K144_FLEXCAN.h
 *
 *  Created on: 11-08-2024
 *      Author: Ndhieu131020@gmail.com
*/

#ifndef DRV_S32K144_FLEXCAN_H_
#define DRV_S32K144_FLEXCAN_H_

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "S32K144.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

/* Define a NULL pointer */
#define NULL ((void *)0)

/* Boundary value of time segment to config baudrate */
#define FLEXCAN_PRESDIV_MAX 255U /* maximum value for prescaler */
#define FLEXCAN_NUM_TQ_MIN 8U    /* minimum numbers of time quanta per bit time*/
#define FLEXCAN_NUM_TQ_MAX 25U   /* maximum numbers of time quanta per bit time*/
#define FLEXCAN_TSEG1_MIN 2U     /* minimum numbers of time quanta for time segment 1*/
#define FLEXCAN_TSEG1_MAX 16U    /* maximum numbers of time quanta for time segment 1*/
#define FLEXCAN_TSEG2_MIN 2U     /* minimum numbers of time quanta for time segment 2*/
#define FLEXCAN_TSEG2_MAX 8U     /* maximum numbers of time quanta for time segment 2*/
#define FLEXCAN_PROPSEG_MAX 7U   /* maximum numbers of time quanta for propagation segment*/
#define FLEXCAN_PSEG1_MAX 7U     /* maximum numbers of time quanta for phase segment 1*/
#define FLEXCAN_PSEG2_MIN 1U     /* minimum numbers of time quanta for phase segment 2*/
#define FLEXCAN_PSEG2_MAX 7U     /* maximum numbers of time quanta for phase segment 2*/
#define FLEXCAN_RJW_MAX 3U       /* maximum numbers of time quanta for RJW*/

/* FLEXCAN_Mb_Masks FLEXCAN Message Buffer Masks */
#define FLEXCAN_MB_ID_STD_MASK (0x1FFC0000U)
#define FLEXCAN_MB_ID_STD_SHIFT 18U
#define FLEXCAN_MB_ID_STD_WIDTH 11U

#define FLEXCAN_MB_ID_EXT_MASK (0x3FFFFU)
#define FLEXCAN_MB_ID_EXT_SHIFT 0U
#define FLEXCAN_MB_ID_EXT_WIDTH 18U

#define FLEXCAN_MB_IDE_MASK (0x200000U)
#define FLEXCAN_MB_IDE_SHIFT 21U
#define FLEXCAN_MB_IDE_WIDTH 1U

#define FLEXCAN_MB_DLC_MASK (0xF0000U)
#define FLEXCAN_MB_DLC_SHIFT 16U
#define FLEXCAN_MB_DLC_WIDTH 4U
#define FLEXCAN_MB_DLC(x) (((uint32_t)((uint32_t)(x) << FLEXCAN_MB_DLC_SHIFT)) & (FLEXCAN_MB_DLC_MASK))

#define FLEXCAN_MB_CODE_MASK (0xF000000U)
#define FLEXCAN_MB_CODE_SHIFT 24U
#define FLEXCAN_MB_CODE_WIDTH 4U
#define FLEXCAN_MB_CODE(x) (((uint32_t)((uint32_t)(x) << FLEXCAN_MB_CODE_SHIFT)) & (FLEXCAN_MB_CODE_MASK))

#define FLEXCAN_MAX_MB_NUM 32U /* Maximum number of Mbs in FLEXCAN module of S32K144 */

/* FlexCAN message buffer CODE FOR Tx buffers */
enum
{
    FLEXCAN_TX_INACTIVE = 0x08, /* MB is not active */
    FLEXCAN_TX_ABORT = 0x09,    /* MB is aborted */
    FLEXCAN_TX_DATA = 0x0C,     /* MB is a TX Data Frame(MB RTR must be 0) */
    FLEXCAN_TX_REMOTE = 0x1C,   /* MB is a TX Remote Request Frame (MB RTR must be 1) */
    FLEXCAN_TX_TANSWER = 0x0E,  /* MB is a TX Response Request Frame from an incoming Remote Request Frame*/
    FLEXCAN_TX_NOT_USED = 0xF   /* Not used */
};

/* FLEXCAN message buffer CODE for Rx buffers */
enum
{
    FLEXCAN_RX_INACTIVE = 0x0, /* MB is not active */
    FLEXCAN_RX_BUSY = 0x1,     /* FlexCAN is updating the contents of the MB */
                               /* The CPU must not access the MB */
    FLEXCAN_RX_FULL = 0x2,     /* MB is full */
    FLEXCAN_RX_EMPTY = 0x4,    /* MB is active and empty */
    FLEXCAN_RX_OVERRUN = 0x6,  /* MB is overwritten into a full buffer */
    FLEXCAN_RX_RANSWER = 0xA,  /* A frame was configured to recognize a Remote Request Frame */
                               /* and transmit a Response Frame in return */
    FLEXCAN_RX_NOT_USED = 0xF  /* Not used */
};

/* CAN standard: 4 words = 1 CS + 1 ID + 2 data (=8 bytes)*/
#define MESSAGE_BUFFER_SIZE 4U

/* FlexCAN operation modes */
typedef enum
{
    FLEXCAN_NORMAL_MODE,
    FLEXCAN_LISTEN_ONLY_MODE,
    FLEXCAN_LOOPBACK_MODE,
    FLEXCAN_FREEZE_MODE,
    FLEXCAN_DISABLE_MODE
} flexcan_operation_modes_t;

/* FlexCAN in/out of Freeze Mode */
typedef enum
{
    FLEXCAN_OUT_FREEZE_MODE,
    FLEXCAN_IN_FREEZE_MODE
} flexcan_freeze_mode_status_t;

/* FlexCAN bitrate related structure */
typedef struct
{
    uint32_t presDiv;
    uint32_t rJumpWidth;
    uint32_t propSeg;
    uint32_t phaseSeg1;
    uint32_t phaseSeg2;
} flexcan_time_segment_t;

/*FLEXCAN clock source*/
typedef enum
{
    FLEXCAN_CLK_SRC_OSC,
    FLEXCAN_CLK_SRC_PERIPH
} flexcan_clock_source_t;

/*FLEXCAN Rx mask type*/
typedef enum
{
    FLEXCAN_RX_MASK_GLOBAL,
    FLEXCAN_RX_MASK_INDIVIDUAL
} flexcan_rx_mask_type_t;

/* FLEXCAN message buffer id type */
typedef enum
{
    FLEXCAN_MB_ID_STD,
    FLEXCAN_MB_ID_EXT
} flexcan_mb_id_type_t;

/* FLEXCAN message buffer */
typedef struct
{
    flexcan_mb_id_type_t idType;
    uint32_t dataLength;
} flexcan_mb_config_t;

/* structure to store MBs */
typedef struct
{
    uint32_t cs;
    uint32_t code;
    uint32_t msgId;
    uint32_t data[2];
    uint32_t dataLength;
} flexcan_mb_t;

/*Information needed for internal handling of a given MB*/
// typedef struct
// {
//     flexcan_mb_t *data; /* The FlexCAN MB structure */
// } flexcan_mb_handle_t;

/* FLEXCAN Interrupt Enable/ Disable*/
typedef enum
{
    FLEXCAN_INTERRUPT_DISABLE,
    FLEXCAN_INTERRUPT_ENABLE
} flexcan_interrupt_t;

/* FLEXCAN module config structure */
typedef struct
{
    flexcan_clock_source_t clkSrc;
    uint32_t flexcanClkFreq;
    uint32_t bitrate;
    flexcan_operation_modes_t flexcanMode;
    flexcan_rx_mask_type_t rxMaskType;
} flexcan_module_config_t;

/* FLEXCAN handle structure */
typedef struct
{
    flexcan_mb_t * mbs[FLEXCAN_MAX_MB_NUM];
    void (*mb_callback)(void);
    void (*bus_off_callback)(void);
} flexcan_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

extern FLEXCAN_Type *g_flexcanBase[FLEXCAN_INSTANCE_COUNT];

void DRV_FLEXCAN_EnableMbInt(uint8_t instance, uint8_t mbIdx);

void DRV_FLEXCAN_DisableMbInt(uint8_t instance, uint8_t mbIdx);

uint8_t DRV_FLEXCAN_GetMbIntFlag(uint8_t instance, uint8_t mbIdx);

void DRV_FLEXCAN_ClearMbIntFlag(uint8_t instance, uint8_t mbIdx);

void DRV_FLEXCAN_Init(uint8_t instance, flexcan_module_config_t *config, flexcan_handle_t *handle);

void DRV_FLEXCAN_SetRxMbGlobalMask(uint8_t instance, flexcan_mb_id_type_t idType, uint32_t mask);

void DRV_FLEXCAN_SetRxMbIndividualMask(uint8_t instance, flexcan_mb_id_type_t idType, uint8_t mbIdx, uint32_t mask);

void DRV_FLEXCAN_ConfigRxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *rx_mb, uint32_t mb_id);

void DRV_FLEXCAN_Receive(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

void DRV_FLEXCAN_ReceiveInt(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

void DRV_FLEXCAN_ConfigTxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *tx_mb, uint32_t mb_id);

void DRV_FLEXCAN_Transmit(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

void DRV_FLEXCAN_RegisterMbCallback(uint8_t instance, void (*cb_ptr)(void));

void DRV_FLEXCAN_RegisterBusOffCallback(uint8_t instance, void (*cb_ptr)(void));

#endif /* DRV_S32K144_FLEXCAN_H_ */
