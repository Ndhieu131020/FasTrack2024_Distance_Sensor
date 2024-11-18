/*
 *  Filename: MID_CAN_Interface.c
 *
 *  Created on: 11-08-2024
 *      Author: Ndhieu131020@gmail.com
*/


#include "DRV_S32K144_PORT.h"
#include "DRV_S32K144_FLEXCAN.h"
#include "DRV_S32K144_MCU.h"
#include "MID_CAN_Interface.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/

#define FLEXCAN_BITRATE               (500000u)
#define FLEXCAN_DATA_LENGTH           (8U)             /* Message buffer data length (8 bytes) */
#define FLEXCAN_GLOBAL_MASK           (0x1FFFFFFF)     /* Global mask (29-bit mask) */
#define FLEXCAN_INDIVIDUAL_MASK       (0xFFFFFFFF)     /* Individual mask (all bits set) */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void FLEXCAN_Pin_Init(void);
static void FLEXCAN_ParamConfig(void);
static void FLEXCAN_Tx_Mb_Init(void);
static void FLEXCAN_Rx_Mb_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static flexcan_handle_t handle;

static flexcan_mb_t Transmit_Message;

static flexcan_mb_t Receive_Message;

flexcan_mb_config_t mbCfg =
{
    .idType = FLEXCAN_MB_ID_STD,
    .dataLength = 4U
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void FLEXCAN_Pin_Init(void)
{
    virtual_pin_id_t Flexcan_Tx_Pin = PTE5;
    virtual_pin_id_t Flexcan_Rx_Pin = PTE4;

    const PortConfig_t  PortConfigCAN =
    {
        .Mux         =  PORT_MUX_ALT5,                    /* Configure the pin mux mode as CAN          */
        .Interrupt   =  PORT_INT_DISABLED,                /* Disable interrupt for the pin              */
        .Pull        =  PORT_INTERNAL_PULL_NOT_ENABLED    /* Disable internal pull resistor for the pin */
    };
    DRV_PORT_Init(GET_PORT((uint8_t)Flexcan_Tx_Pin), GET_PIN_NUM((uint8_t)Flexcan_Tx_Pin), &PortConfigCAN);
    DRV_PORT_Init(GET_PORT((uint8_t)Flexcan_Rx_Pin), GET_PIN_NUM((uint8_t)Flexcan_Rx_Pin), &PortConfigCAN);
}

static void FLEXCAN_ParamConfig(void)
{
    uint32_t CAN_ClkFreq = 0u;

    flexcan_module_config_t moduleCfg =
    {
        .clkSrc = FLEXCAN_CLK_SRC_PERIPH,
        .flexcanClkFreq = CAN_ClkFreq,
        .bitrate = FLEXCAN_BITRATE,
        .rxMaskType = FLEXCAN_RX_MASK_INDIVIDUAL,
        .flexcanMode = FLEXCAN_NORMAL_MODE
    };

    if(moduleCfg.clkSrc == FLEXCAN_CLK_SRC_PERIPH)
    {
        DRV_Clock_GetFrequency(FlexCAN0_CLK, &CAN_ClkFreq);
    }
    else
    {
        DRV_Clock_GetFrequency(SOSCDIV2_CLK, &CAN_ClkFreq);
    }

    if(CAN_ClkFreq != 0u)
    {
        moduleCfg.flexcanClkFreq = CAN_ClkFreq;
    }

    DRV_FLEXCAN_Init(FLEXCAN_INSTANCE, &moduleCfg, &handle);
}

static void FLEXCAN_Tx_Mb_Init(void)
{
    /* Initialize Tx Message buffer to send sensor data to CAN Bus */
    DRV_FLEXCAN_ConfigTxMb(FLEXCAN_INSTANCE, TX_DISTANCE_DATA_MB, &mbCfg, TX_MSG_DISTANCE_DATA_ID);

    /* Initialize Tx Message buffer to send confirm connection from Forwarder to CAN Bus */
    DRV_FLEXCAN_ConfigTxMb(FLEXCAN_INSTANCE, TX_CONFIRM_CONNECTION_MB, &mbCfg, TX_MSG_CONFIRM_CONNECTION_ID);
}

static void FLEXCAN_Rx_Mb_Init(void)
{
    DRV_FLEXCAN_SetRxMbGlobalMask(FLEXCAN_INSTANCE, FLEXCAN_MB_ID_STD, FLEXCAN_GLOBAL_MASK);

    DRV_FLEXCAN_SetRxMbIndividualMask(FLEXCAN_INSTANCE, FLEXCAN_MB_ID_STD, RX_STOPOPR_MB, FLEXCAN_INDIVIDUAL_MASK);
    DRV_FLEXCAN_SetRxMbIndividualMask(FLEXCAN_INSTANCE, FLEXCAN_MB_ID_STD, RX_CONNECTION_MB, FLEXCAN_INDIVIDUAL_MASK);
    DRV_FLEXCAN_SetRxMbIndividualMask(FLEXCAN_INSTANCE, FLEXCAN_MB_ID_STD, RX_CONFIRM_DATA_MB, FLEXCAN_INDIVIDUAL_MASK);

    DRV_FLEXCAN_ConfigRxMb(FLEXCAN_INSTANCE, RX_STOPOPR_MB, &mbCfg, RX_MSG_STOPOPR_ID);
    DRV_FLEXCAN_ConfigRxMb(FLEXCAN_INSTANCE, RX_CONNECTION_MB, &mbCfg, RX_MSG_CONNECTION_ID);
    DRV_FLEXCAN_ConfigRxMb(FLEXCAN_INSTANCE, RX_CONFIRM_DATA_MB, &mbCfg, RX_MSG_CONFIRM_DATA_ID);

    DRV_FLEXCAN_EnableMbInt(FLEXCAN_INSTANCE, RX_STOPOPR_MB);
    DRV_FLEXCAN_EnableMbInt(FLEXCAN_INSTANCE, RX_CONNECTION_MB);
    DRV_FLEXCAN_EnableMbInt(FLEXCAN_INSTANCE, RX_CONFIRM_DATA_MB);
}

void MID_CAN_Init(void)
{
    FLEXCAN_Pin_Init();
    FLEXCAN_ParamConfig();
    FLEXCAN_Tx_Mb_Init();
    FLEXCAN_Rx_Mb_Init();
}

void MID_CAN_MailboxInit(void)
{
    FLEXCAN_Tx_Mb_Init();
    FLEXCAN_Rx_Mb_Init();
}

void MID_CAN_RegisterRxNotificationCallback(void (*cb_ptr)(void))
{
    DRV_FLEXCAN_RegisterMbCallback(FLEXCAN_INSTANCE, cb_ptr);
}

void MID_CAN_RegisterBusOffNotificationCallback(void (*cb_ptr)(void))
{
    DRV_FLEXCAN_RegisterBusOffCallback(FLEXCAN_INSTANCE, cb_ptr);
}

void MID_CAN_ReceiveMessage(uint8_t mbIdx, Data_Typedef *data)
{
    DRV_FLEXCAN_ReceiveInt(FLEXCAN_INSTANCE, mbIdx, &Receive_Message);

    data->ID = Receive_Message.msgId;
    data->Data = Receive_Message.data[0];
}

void MID_CAN_SendCANMessage(uint8_t Tx_Mb, uint16_t Data)
{
    Transmit_Message.data[0] = Data;

    DRV_FLEXCAN_Transmit(FLEXCAN_INSTANCE, Tx_Mb, &Transmit_Message);
}

void MID_ClearMessageCommingEvent(uint8_t Mailbox)
{
    DRV_FLEXCAN_ClearMbIntFlag(FLEXCAN_INSTANCE, Mailbox);
}

uint8_t MID_CheckCommingMessageEvent(uint8_t Mailbox)
{
    return DRV_FLEXCAN_GetMbIntFlag(FLEXCAN_INSTANCE, Mailbox);
}
