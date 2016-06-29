//****************************************************************************
//*                                                                          *
//*                   _____   _____   _   _   ___                            *
//*                  |  ___| |  ___| | | | | |   `                           *
//*                  | |_    | |_    | |_| | | [] |                          *
//*                  |  _|   |  _|   |  _  | |  _ <                          *
//*                  | |___  | |___  | | | | | |_| |                         *
//*                  |_____| |_____| |_| |_| |____/                          *
//*                                                                          *
//*                                                                          *
//*  Project:          BIOSControl uPD70Fxxxx                                *
//*  (c) copyright     Harman-Becker Automotive Systems                      *
//****************************************************************************


#ifndef _CANMSG_H
#define _CANMSG_H


#define CANMSG_LAENGE                  8  // Anzahl aller Bytes je CAN-Message

//*--------------------------------------------------------------------------*
//* CAN-Message Receiver possible States                                     *
//*--------------------------------------------------------------------------*

#define CANMSG_RX_DISABLED      		   0
#define CANMSG_RX_ENABLED   	 		   1

//*--------------------------------------------------------------------------*
//* CAN-Message Transceiver possible States                                  *
//*--------------------------------------------------------------------------*

#define CANMSG_TX_DISABLED      		   0
#define CANMSG_TX_ENABLED   	 		   1

//*--------------------------------------------------------------------------*
//* all CAN Errors                                                           *
//*--------------------------------------------------------------------------*

// da Fehler kombiniert vorkommen können, ist das Ver-Odern verschiedener Fehler erlaubt

#define CANMSG_NO_ERROR      					   0x00000000

//* D0 bis D3 für TX-Fehler reserviert
#define CANMSG_TX_CAN_ERROR                  0x16000001
#define CANMSG_TX_PACKET_FORMAT_ERROR    	   0x16000002
#define CANMSG_TX_TIMEOUT_ERROR 				   0x16000004
#define CANMSG_TX_ACCESS_ERROR               0x16000008

//* D4 bis D7 für RX-Fehler reserviert
#define CANMSG_RX_CAN_ERROR                  0x16000010
#define CANMSG_RX_PACKET_FORMAT_ERROR		   0x16000020
#define CANMSG_RX_UNKNOWNINT_ERROR           0x16000030
#define CANMSG_RX_BUFFER_FULL_ERROR          0x16000040

// andere Fehler
#define CANMSG_POWERON_ERROR					   0x16000400		// der Chip generiert keinen Interrupt


//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*


extern UINT32	CANMSG_Init		      (UINT8 Channel);
extern void 	CANMSG_MarkColdstart (void);


//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

// Fehlervariable nur für die Interruptroutine
extern UINT32  CANMSG_INT_Error;
// Steuervariable für Interruptweiterleitung aus CAN
extern UINT8   CANMSG_INT_Enable;

extern UINT8   CAN_Selected_Channel;
extern UINT32   Gateway_CAN_CommIndex[16];

#endif // _CANMSG_H

