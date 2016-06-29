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


#ifndef _MOSTMSG_H
#define _MOSTMSG_H


//*--------------------------------------------------------------------------*
//* physikalischer Mostring                                                  *
//*--------------------------------------------------------------------------*

#define  MOST_NOT_INITIALIZED          0
#define  MOST_MASTER_OK                1
#define  MOST_MASTER_UNLOCK_DETECTED   2
#define  MOST_MASTER_WAIT_RESTART      3
#define  MOST_SLAVE_OK                 4
#define  MOST_SLAVE_UNLOCK_DETECTED    5
#define  MOST_SLAVE_NO_NETWORKACTIVITY 6
#define  MOST_SLAVE_BYPASS_CLOSED      7
#define  MOST_LIGHT_OFF                8

//*--------------------------------------------------------------------------*
//* MOST-Message Receiver possible States                                    *
//*--------------------------------------------------------------------------*

#define MOSTMSG_RX_DISABLED      		0
#define MOSTMSG_RX_ENABLED   	 		   1
#define MOSTMSG_RX_RECEIVED     	      2

//*--------------------------------------------------------------------------*
//* MOST-Message Transceiver possible States                                 *
//*--------------------------------------------------------------------------*

#define MOSTMSG_TX_DISABLED      		0
#define MOSTMSG_TX_ENABLED   	 		   1
#define MOSTMSG_TX_BUSY     	         2

//*--------------------------------------------------------------------------*
//* MOST-Message Controls                                                    *
//*--------------------------------------------------------------------------*

#define  POLL_CREDITS                  10
#define  POLL_TIMEOUT                  25
#define  REQ_READ_TIMEOUT              200

//*--------------------------------------------------------------------------*
//* all MOST Errors                                                          *
//*--------------------------------------------------------------------------*

// da Fehler kombiniert vorkommen können, ist das Ver-Odern verschiedener Fehler erlaubt

#define MOSTMSG_NO_ERROR      					0x00000000

//* D0 bis D3 für TX-Fehler reserviert
#define MOSTMSG_TX_TARGET_RESPONSE_ERROR     0x02000001
#define MOSTMSG_TX_BAD_CRC_ERROR             0x02000002
#define MOSTMSG_TX_XMIT_TYPE_ERROR           0x02000003
#define MOSTMSG_TX_REC_BUFFER_FULL_ERROR     0x02000004
#define MOSTMSG_TX_PACKET_FORMAT_ERROR    	0x02000005
#define MOSTMSG_TX_TIMEOUT_ERROR 				0x02000006
#define MOSTMSG_TX_ACCESS_ERROR              0x02000007

//* D4 bis D7 für RX-Fehler reserviert
#define MOSTMSG_RX_WRONG_SOURCE_ADDR_ERROR   0x02000010
#define MOSTMSG_RX_PACKET_FORMAT_ERROR		   0x02000020
#define MOSTMSG_RX_UNKNOWNINT_ERROR          0x02000030
#define MOSTMSG_RX_BUFFER_FULL_ERROR         0x02000040
#define MOSTMSG_RX_BUFFERALLOCATE_ERROR      0x02000050
#define MOSTMSG_RX_ACCESS_ERROR              0x02000060
#define MOSTMSG_RX_CRC_ERROR                 0x02000070

//* D8 bis D11 für allgemeine MOST-Fehler reserviert
// IIC-Bus Fehler siehe File iic.h

// andere Fehler
#define MOSTMSG_POWERON_ERROR					   0x02000400		// der Chip generiert keinen Interrupt
#define MOSTMSG_NODENUMBER_ERROR             0x02000500     // Die Nodenummer ist nicht frei
#define MOSTMSG_DRIVER_ERROR					   0x02000600		// gewählter Treiber nicht verfügbar
#define MOSTMSG_FBLOCK_ERROR					   0x02000700		// falscher FBlock (nur bei MOSTFORMAT_STANDARD)

#define MOSTMSG_RESPONSE_TIMEOUT             0x02090000  // Wait for response or not response timeout

//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    HW Control Section  (nicht vollständig)    *
//*--------------------------------------------------------------------------*

#define OS8104_bCM4                 0x38
#define OS8104_bXCR					   0x80
#define OS8104_bXSR                 0x81
#define OS8104_bSDC1       			0x82
#define OS8104_bSDC2       			0x8C
#define OS8104_bSDC3       			0x8D
#define OS8104_bCM1						0x83
#define OS8104_bMSGC                0x85
#define OS8104_bMSGS                0x86
#define OS8104_bNPR                 0x87
#define OS8104_bIE                  0x88
#define OS8104_bNAH                 0x8A
#define OS8104_bNAL                 0x8B
#define OS8104_bCM2						0x8E
#define OS8104_bSBC                 0x96

// fuer Packet Daten
#define OS8104_bPCTC                0xE2
#define OS8104_bPCTS                0xE3
#define OS8104_bPCMA                0xE6
#define OS8104_bAPAH                0xE8
#define OS8104_bAPAL                0xE9
#define OS8104_bPSTX                0xEA
#define OS8104_bPLDT                0xEC
#define OS8104_bPPI                 0xF2

#define OS8104_bPAGE                0xFF

//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    Rec Ctrl Msg Buffer (nicht vollständig)    *
//*--------------------------------------------------------------------------*

#define OS8104_bRTYP   				   0xA0
#define OS8104_bRSAH                0xA1
#define OS8104_bRSAL                0xA2

#define OS8104_bRCD0                0xA3
#define OS8104_bRCD1                0xA4
#define OS8104_bRCD2                0xA5
#define OS8104_bRCD3                0xA6
#define OS8104_bRCD4                0xA7
#define OS8104_bRCD5                0xA8
#define OS8104_bRCD6                0xA9
#define OS8104_bRCD7                0xAA
#define OS8104_bRCD8                0xAB
#define OS8104_bRCD9                0xAC
#define OS8104_bRCD10               0xAD
#define OS8104_bRCD11               0xAE
#define OS8104_bRCD12               0xAF
#define OS8104_bRCD13               0xB0
#define OS8104_bRCD14               0xB1
#define OS8104_bRCD15               0xB2
#define OS8104_bRCD16               0xB3

#define OS8104_bXTIM                0xBE
#define OS8104_bXRTY                0xBF


//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    Xmit Ctrl Msg Buffer (nicht vollständig)   *
//*--------------------------------------------------------------------------*

#define OS8104_bXPRI                0xC0
#define OS8104_bXTYP                0xC1
#define OS8104_bXTAH                0xC2
#define OS8104_bXTAL                0xC3

#define OS8104_bXCD0                0xC4
#define OS8104_bXCD1                0xC5
#define OS8104_bXCD2                0xC6
#define OS8104_bXCD3                0xC7
#define OS8104_bXCD4                0xC8
#define OS8104_bXCD5                0xC9
#define OS8104_bXCD6                0xCA
#define OS8104_bXCD7                0xCB
#define OS8104_bXCD8                0xCC
#define OS8104_bXCD9                0xCD
#define OS8104_bXCD10               0xCE
#define OS8104_bXCD11               0xCF
#define OS8104_bXCD12               0xD0
#define OS8104_bXCD13               0xD1
#define OS8104_bXCD14               0xD2
#define OS8104_bXCD15               0xD3
#define OS8104_bXCD16               0xD4

#define OS8104_bXTS                 0xD5

//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    Packet Receive Buffer (nicht vollständig)  *
//*--------------------------------------------------------------------------*

#define OS8104_bARTH                0x180
#define OS8104_bARTL                0x181

#define OS8104_bASAH                0x182
#define OS8104_bASAL                0x183

#define OS8104_bARD0                0x184
#define OS8104_bARD1                0x185
#define OS8104_bARD2                0x186
#define OS8104_bARD47               0x1B3

//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    Packet Transmit Buffer (nicht vollständig) *
//*--------------------------------------------------------------------------*

#define OS8104_bATAH                0x1C0
#define OS8104_bATAL                0x1C1

#define OS8104_bAXD0                0x1C2
#define OS8104_bAXD1                0x1C3
#define OS8104_bAXD2                0x1C4
#define OS8104_bAXD47               0x1F1

//*--------------------------------------------------------------------------*
//* Defines of OS8104 Register    Routing table (nicht vollständig)          *
//*--------------------------------------------------------------------------*

#define OS8104_MRT0x00               0x00
#define OS8104_MRT0x01               0x01
#define OS8104_MRT0x02               0x02
#define OS8104_MRT0x03               0x03

#define OS8104_MRT0x44               0x44
#define OS8104_MRT0x54               0x54
#define OS8104_MRT0x64               0x64
#define OS8104_MRT0x74               0x74

#define OS8104_MRT0x47               0x47
#define OS8104_MRT0x4F               0x4F
#define OS8104_MRT0x57               0x57
#define OS8104_MRT0x5F               0x5F
#define OS8104_MRT0x67               0x67
#define OS8104_MRT0x6F               0x6F
#define OS8104_MRT0x77               0x77
#define OS8104_MRT0x7F               0x7F

#define OS8104_MRT0x4C               0x4C
#define OS8104_MRT0x5C               0x5C
#define OS8104_MRT0x6C               0x6C
#define OS8104_MRT0x7C               0x7C

// Zugriffsmakros zum Extrahieren der Bytes aus den Registeradressen
#define LOW_ADD(A)                  ((UINT8)(A & 0xFF))
#define HIGH_ADD(A)                 ((UINT8)(A >> 8))


//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*

#if (CP_DEBUG == 1)
   extern void Debug_Info  		(void);  // zum Debuggen
#endif

extern UINT32  MOSTMSG_Init		(UINT8 Channel);

extern void    MOSTMSG_WR        (void);  // fuer Handsteuerung ueber Z-Befehl
extern void    MOSTMSG_RD        (void);  // fuer Handsteuerung ueber z-Befehl

#if CP_INIC
   extern BOOL MostMsgRunning(void);
#endif

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*


#endif // _MOSTMSG_H

