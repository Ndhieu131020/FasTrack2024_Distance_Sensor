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
#define FLEXCAN_PRESDIV_MAX (255U)  /* maximum value for prescaler */
#define FLEXCAN_NUM_TQ_MIN  (8U)    /* minimum numbers of time quanta per bit time*/
#define FLEXCAN_NUM_TQ_MAX  (25U)   /* maximum numbers of time quanta per bit time*/
#define FLEXCAN_TSEG1_MIN   (2U)    /* minimum numbers of time quanta for time segment 1*/
#define FLEXCAN_TSEG1_MAX   (16U)   /* maximum numbers of time quanta for time segment 1*/
#define FLEXCAN_TSEG2_MIN   (2U)    /* minimum numbers of time quanta for time segment 2*/
#define FLEXCAN_TSEG2_MAX   (8U)    /* maximum numbers of time quanta for time segment 2*/
#define FLEXCAN_PROPSEG_MAX (7U)    /* maximum numbers of time quanta for propagation segment*/
#define FLEXCAN_PSEG1_MAX   (7U)    /* maximum numbers of time quanta for phase segment 1*/
#define FLEXCAN_PSEG2_MIN   (1U)    /* minimum numbers of time quanta for phase segment 2*/
#define FLEXCAN_PSEG2_MAX   (7U)    /* maximum numbers of time quanta for phase segment 2*/
#define FLEXCAN_RJW_MAX     (3U)    /* maximum numbers of time quanta for RJW*/

/* FLEXCAN_Mb_Masks FLEXCAN Message Buffer Masks */
#define FLEXCAN_MB_ID_STD_MASK  (0x1FFC0000U)
#define FLEXCAN_MB_ID_STD_SHIFT (18U)
#define FLEXCAN_MB_ID_STD_WIDTH (11U)

#define FLEXCAN_MB_ID_EXT_MASK  (0x3FFFFU)
#define FLEXCAN_MB_ID_EXT_SHIFT (0U)
#define FLEXCAN_MB_ID_EXT_WIDTH (18U)

#define FLEXCAN_MB_IDE_MASK     (0x200000U)
#define FLEXCAN_MB_IDE_SHIFT    (21U)
#define FLEXCAN_MB_IDE_WIDTH    (1U)

#define FLEXCAN_MB_DLC_MASK     (0xF0000U)
#define FLEXCAN_MB_DLC_SHIFT    (16U)
#define FLEXCAN_MB_DLC_WIDTH    (4U)
#define FLEXCAN_MB_DLC(x)       (((uint32_t)((uint32_t)(x) << FLEXCAN_MB_DLC_SHIFT)) & (FLEXCAN_MB_DLC_MASK))

#define FLEXCAN_MB_CODE_MASK    (0xF000000U)
#define FLEXCAN_MB_CODE_SHIFT   (24U)
#define FLEXCAN_MB_CODE_WIDTH   (4U)
#define FLEXCAN_MB_CODE(x)      (((uint32_t)((uint32_t)(x) << FLEXCAN_MB_CODE_SHIFT)) & (FLEXCAN_MB_CODE_MASK))

#define FLEXCAN_MAX_MB_NUM 32U  /* Maximum number of Mbs in FLEXCAN module of S32K144 */

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

/* FlexCAN message buffer CODE for Rx buffers */
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

/* CAN standard: Message buffer contains 4 words = 1 CS + 1 ID + 2 data (= 8 bytes) */
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

/*FlexCAN Clock Source*/
typedef enum
{
    FLEXCAN_CLK_SRC_OSC,
    FLEXCAN_CLK_SRC_PERIPH
} flexcan_clock_source_t;

/*FlexCAN Rx mask type*/
typedef enum
{
    FLEXCAN_RX_MASK_GLOBAL,
    FLEXCAN_RX_MASK_INDIVIDUAL
} flexcan_rx_mask_type_t;

/* FlexCAN Message Buffer ID type */
typedef enum
{
    FLEXCAN_MB_ID_STD, /* standard ID */
    FLEXCAN_MB_ID_EXT  /* extended ID */
} flexcan_mb_id_type_t;

/* FlexCAN Message Buffer Configuration Structure */
typedef struct
{
    flexcan_mb_id_type_t idType;
    uint32_t dataLength;
} flexcan_mb_config_t;

/* FlexCAN structure to store MBs */
typedef struct
{
    uint32_t cs;          /* Control and Status Word */
    uint32_t code;        /* CODE field of message buffer */
    uint32_t msgId;       /* ID of message */
    uint32_t data[2];     /* Data */
    uint32_t dataLength;  /* Data length */
} flexcan_mb_t;

/* FlexCAN Interrupt Enable/ Disable*/
typedef enum
{
    FLEXCAN_INTERRUPT_DISABLE,
    FLEXCAN_INTERRUPT_ENABLE
} flexcan_interrupt_t;

/* FlexCAN Module Configuration Structure */
typedef struct
{
    flexcan_clock_source_t clkSrc;
    uint32_t flexcanClkFreq;
    uint32_t bitrate;
    flexcan_operation_modes_t flexcanMode;
    flexcan_rx_mask_type_t rxMaskType;
} flexcan_module_config_t;

/* FlexCAN Handle Structure */
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

/**
 * @brief       Enable the interrupt for a specific message buffer.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   mbIdx:    Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_EnableMbInt(uint8_t instance, uint8_t mbIdx);

/**
 * @brief       Disable the interrupt for a specific message buffer.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   mbIdx:    Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_DisableMbInt(uint8_t instance, uint8_t mbIdx);

/**
 * @brief       Checks the interrupt flag for the specified message buffer index and returns its status.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   mbIdx:     Message buffer index.
 * @retval      Interrupt flag status (1 if set, 0 otherwise).
 */
uint8_t DRV_FLEXCAN_GetMbIntFlag(uint8_t instance, uint8_t mbIdx);

/**
 * @brief       Clears the interrupt flag for a specific message buffer
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   mbIdx:     Message buffer index.
 * @retval      None
 */
void DRV_FLEXCAN_ClearMbIntFlag(uint8_t instance, uint8_t mbIdx);

/**
 * @brief       Initializes the FLEXCAN module with the specified configuration.
 * @param[in]   instance: Identifies which FlexCAN module
 * @param[in]   config:   Pointer to the FLEXCAN configuration structure.
 * @param[out]  handle:   Pointer to the FLEXCAN handle structure.
 * @retval      None
 */
void DRV_FLEXCAN_Init(uint8_t instance, flexcan_module_config_t *config, flexcan_handle_t *handle);

/**
 * @brief       Configures the global mask for RX message buffers.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   idType:    ID type (standard or extended).
 * @param[in]   mask:      Global mask value.
 * @retval      None
 */
void DRV_FLEXCAN_SetRxMbGlobalMask(uint8_t instance, flexcan_mb_id_type_t idType, uint32_t mask);

/**
 * @brief       Configures an individual mask for a specific RX message buffer.
 * @param[in]   instance:  Identifies which FlexCAN module
 * @param[in]   idType:    ID type (standard or extended)
 * @param[in]   mbIdx:     Message buffer index.
 * @param[in]   mask:      Individual mask value.
 * @retval      None
 */
void DRV_FLEXCAN_SetRxMbIndividualMask(uint8_t instance, flexcan_mb_id_type_t idType, uint8_t mbIdx, uint32_t mask);

/**
  * @brief      Configure a Receive Message Buffer
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[in]  rx_mb: Pointer to message buffer configuration structure
  * @param[in]  mb_id: Message buffer ID
  * @retval     None
  */
void DRV_FLEXCAN_ConfigRxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *rx_mb, uint32_t mb_id);

/**
  * @brief      Receive a CAN message in blocking manner
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[out] data: Pointer to received message structure
  * @retval     None
  */
void DRV_FLEXCAN_Receive(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

/**
  * @brief      Receive a CAN message using interrupt
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[out] data: Pointer to received message structure
  * @retval     None
  */
void DRV_FLEXCAN_ReceiveInt(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

/**
  * @brief      Configure a Transmit Message Buffer
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx:  Message buffer index
  * @param[in]  tx_mb: Pointer to message buffer configuration structure
  * @param[in]  mb_id: Message buffer ID
  * @retval     None
  */
void DRV_FLEXCAN_ConfigTxMb(uint8_t instance, uint8_t mbIdx, flexcan_mb_config_t *tx_mb, uint32_t mb_id);

/**
  * @brief      Transmit a CAN message
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  mbIdx: Mailbox index
  * @param[in]  data: Pointer to message structure to transmit
  * @retval     None
  */
void DRV_FLEXCAN_Transmit(uint8_t instance, uint8_t mbIdx, flexcan_mb_t *data);

/**
  * @brief      Register Message Buffer Callback Function
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  cb_ptr Pointer to callback function
  * @retval     None
  */
void DRV_FLEXCAN_RegisterMbCallback(uint8_t instance, void (*cb_ptr)(void));

/**
  * @brief      Register Bus off Callback Function
  * @param[in]  instance: Identifies which FlexCAN module
  * @param[in]  cb_ptr: Pointer to callback function
  * @retval     None
  */
void DRV_FLEXCAN_RegisterBusOffCallback(uint8_t instance, void (*cb_ptr)(void));

#endif /* DRV_S32K144_FLEXCAN_H_ */
