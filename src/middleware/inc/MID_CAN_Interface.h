/*
 *  Filename: MID_CAN_Interface.h
 *
 *  Created on: 11-08-2024
 *      Author: Ndhieu131020@gmail.com
*/

#ifndef MID_CAN_INTERFACE_H_
#define MID_CAN_INTERFACE_H_

/*******************************************************************************
 * Include
 ******************************************************************************/

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define FLEXCAN_INSTANCE        0u

#define RX_MSG_CONNECTION_ID    0xE0
#define RX_MSG_STOPOPR_ID       0x30

#define TX_MSG_CONFIRM_CONNECTION_ID    0xE1
#define TX_MSG_CONFIRM_CONNECTION_DATA  0xFF

#define TX_MSG_DISTANCE_DATA_ID       0x20

#define TX_DISTANCE_DATA_MB        0u
#define TX_CONFIRM_CONNECTION_MB   1u

#define RX_STOPOPR_MB      4u
#define RX_CONNECTION_MB   5u

#define CAN_MSG_RECEIVED      1u
#define CAN_MSG_NO_RECEIVED   0u

typedef struct MID_CAN_Interface
{
    uint32_t ID;
    uint32_t Data;
}Data_Typedef;

/*******************************************************************************
 * API
 ******************************************************************************/

void MID_CAN_Init(void);

void MID_CAN_MailboxInit(void);

void MID_CAN_RegisterRxNotificationCallback(void (*cb_ptr)(void));

void MID_CAN_RegisterBusOffNotificationCallback(void (*cb_ptr)(void));

void MID_CAN_ReceiveMessage(uint8_t mbIdx, Data_Typedef *data);

void MID_CAN_SendCANMessage(uint8_t Tx_Mb, int16_t Data);

void MID_ClearMessageCommingEvent(uint8_t Mailbox);

uint8_t MID_CheckCommingMessageEvent(uint8_t Mailbox);

#endif /* MID_CAN_INTERFACE_H_ */