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

#ifndef _IICMSG_H
#define _IICMSG_H


//*--------------------------------------------------------------------------*
//* all IIC Errors                                                           *
//*--------------------------------------------------------------------------*

// da Fehler kombiniert vorkommen können, ist das Ver-Odern verschiedener Fehler erlaubt

#define IICMSG_NO_ERROR      					   0x0000

//* D8 bis D11 für TX-Fehler reserviert
#define IICMSG_TX_CAN_ERROR                  0x0100
#define IICMSG_TX_PACKET_FORMAT_ERROR    	   0x0200
#define IICMSG_TX_TIMEOUT_ERROR 				   0x0400
#define IICMSG_TX_ACCESS_ERROR               0x0800

//* D12 bis D15 für RX-Fehler reserviert
#define IICMSG_RX_CAN_ERROR                  0x1000
#define IICMSG_RX_PACKET_FORMAT_ERROR		   0x2000
#define IICMSG_RX_UNKNOWNINT_ERROR           0x3000
#define IICMSG_RX_BUFFER_FULL_ERROR          0x4000

//* D0 bis D7 für Treiber-Fehler reserviert
#define IICMSG_POWERON_ERROR					   0x0006		// der Chip generiert keinen Interrupt

// aus IIC-Treibern hochgereichte Fehler
#define IICMSG_I2C_BUS_BUSY                  0x0001U
#define IICMSG_I2C_NO_DEVICE                 0x0002U
#define IICMSG_I2C_NO_ACK                    0x0004U
#define IICMSG_I2C_WRONG_CH                  0x0005U


//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*

extern UINT16  IICMSG_Init		   (void);
extern UINT16	IICMSG_Write  	   (UINT8 *Buffer);
extern BOOL    IICMSG_Read 		(UINT8 *Buffer);

#if (CP_DEBUG == 13)
   extern void       Debug_Info  		(void);  // zum Debuggen
#endif

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

#endif // _IICMSG_H

