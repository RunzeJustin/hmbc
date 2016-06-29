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


#ifndef _SER_H
#define _SER_H

//*--------------------------------------------------------------------------*
//* Funktionen                                                               *
//*--------------------------------------------------------------------------*

extern void    SerielUart_CMD (void);
extern BOOL    RS232_Init  (UINT8 Channel, UINT32 Baudrate);

extern BOOL    SerWriteChar (UINT8 Channel, UINT8 Data);

//*-----------------------------------------------------------------------*
//* Constants                                                             *
//*-----------------------------------------------------------------------*

// definitions for the serial data format
#define SER_8N1      0x0   // CHR = 0, PE = 0, O/E = 0, STOP = 0
#define SER_8N2      0x1   // CHR = 0, PE = 0, O/E = 0, STOP = 1
#define SER_8E1      0x4   // CHR = 0, PE = 1, O/E = 0, STOP = 0
#define SER_8E2      0x5   // CHR = 0, PE = 1, O/E = 0, STOP = 1
#define SER_8O1      0x6   // CHR = 0, PE = 1, O/E = 1, STOP = 0
#define SER_8O2      0x7   // CHR = 0, PE = 1, O/E = 1, STOP = 1
#define SER_7N1      0x8   // CHR = 1, PE = 0, O/E = 0, STOP = 0
#define SER_7N2      0x9   // CHR = 1, PE = 0, O/E = 0, STOP = 1
#define SER_7E1      0xC   // CHR = 1, PE = 1, O/E = 0, STOP = 0
#define SER_7E2      0xD   // CHR = 1, PE = 1, O/E = 0, STOP = 1
#define SER_7O1      0xE   // CHR = 1, PE = 1, O/E = 1, STOP = 0
#define SER_7O2      0xF   // CHR = 1, PE = 1, O/E = 1, STOP = 1

#define SER_ENABLE_RTSCTS  0x10
#define SER_RTS_POLARITY   0x20

#define SER_BUS_ACTIVE     0x80

#define SER_CMD_BIOS       0xC0  // Bioscontroll Kommandos
#define SER_LIN_20   0xB0  // Lin 2.0 Unterstuetzung
#define SER_FRT_AUDI       0xF0  // Frontkontroller Protokoll Audi Unterstuetzung

//*--------------------------------------------------------------------------*
//*	error codes                                                            *
//*--------------------------------------------------------------------------*

#define SER_ERR_CHANNEL          0x00000004U
#define SER_ERR_PROTOCOL         0x00000008U
#define SER_ERR_TIMEOUT          0x00000020U
#define SER_ERR_OPEN             0x00000040U
#define SER_ERR_CLOSE            0x00000080U

#define SER_ERR_LIN_RESPONSETIME 0x0000B001U
#define SER_ERR_LIN_CHECKSUM     0x0000B002U
#define SER_ERR_LIN_BIT1         0x0000B010U
#define SER_ERR_LIN_BIT2         0x0000B020U
#define SER_ERR_LIN_PHYSICAL_BUS 0x0000B040U

#define SER_ERR_HANDSHAKE_0      0x0000E200U
#define SER_ERR_HANDSHAKE_1      0x0000E201U

#define SER_ERR_FRT_ACKTIMEOUT   0x0000F001U

#define SER_ERR_FRT_MISSINGSTX   0x0000F002U
#define SER_ERR_FRT_MAXLEN       0x0000F003U
#define SER_ERR_FRT_WRONGDLE     0x0000F004U
#define SER_ERR_FRT_CHKSUM       0x0000F005U

#if (CP_DEBUG == 18)
   extern void Debug_Info (void);
#endif


#endif // _SER_H


