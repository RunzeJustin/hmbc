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


#ifndef _SPI_H
#define _SPI_H

//*--------------------------------------------------------------------------*
//* Defines                                                                  *
//*--------------------------------------------------------------------------*

#define SPI_CH_OFFSET   0x30     // kleinste SPI Kanalnummer
#define CSIE_CH_OFFSET  0x40     // kleinste CSIE Kanalnummer (erweiterte SPI)
#define CSIG_CH_OFFSET  0x50     // SPI port in derivative 3558
#define CSIH_CH_OFFSET  0x60     // SPI Port im Derivat 3558

//*--------------------------------------------------------------------------*
//* all SPI Errors                                                           *
//*--------------------------------------------------------------------------*

// da Fehler kombiniert vorkommen können, ist das Ver-Odern verschiedener Fehler erlaubt

#define SPIMSG_NO_ERROR      					   0x00000000

// allgemeine Fehler
#define SPIMSG_INIT_ERROR      			      0x00000100

//* D8 bis D11 für TX-Fehler reserviert

//* D12 bis D15 für RX-Fehler reserviert

//* D0 bis D7 für Treiber-Fehler reserviert

//*--------------------------------------------------------------------------*
//* Funktionen                                                               *
//*--------------------------------------------------------------------------*

extern void    SerielSpi_CMD        (void);
extern void    SerielCSIE_CMD       (void);

extern UINT32	SPIMSG_Init		      (UINT8 Channel, UINT32 Baudrate);
extern UINT32	SPIMSG_Close		   (UINT8 Channel);


extern BOOL    SerOpen              (UINT8 Channel, UINT32 Baudrate, UINT8 Protocol, BOOL Commandmode, UINT32 Wait_Interbyte, UINT32 Wait_After_CS0, UINT32 Wait_Before_CS1);
extern BOOL    SerClose             (UINT8 Channel);

extern BOOL    SpiWrite_1Word       (UINT8 Channel, UINT16 Data);
extern BOOL    SpiReadMaster_1Char  (UINT8 Channel, UINT8 *Data);
extern BOOL    Activate_CS          (UINT8 Channel, BOOL Set);

#if (CP_DEBUG == 19)
   extern void Debug_Info (void);
#endif

#endif // _SPI_H


