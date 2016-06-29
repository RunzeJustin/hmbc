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

#ifndef _ADC_H
#define _ADC_H

//*--------------------------------------------------------------------------*
//* ADC Error Messages                                                       *
//*--------------------------------------------------------------------------*

#define ERROR_ADC_TIMEOUT                       0x00000001U
#define ERROR_WRONG_CHANNEL                     0x00000002U
#define ERROR_VIRTUELL_CHANNEL_UNCONFIGED       0x00000003U
#define ERROR_VIRTUELL_CHANNEL_INVALID_CONFIG   0x00000004U
#define ERROR_VIRTUELL_CHANNEL_BUF_OVERFLOW     0x00000005U

//*--------------------------------------------------------------------------*
//* Definitionen für den A/D-Wandler                                         *
//*--------------------------------------------------------------------------*

extern void    ADC_RD      (void);
extern void    ADC_Init    (void);

//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*

#if (CP_DEBUG == 11)
   extern void       Debug_Info  		(void);  // zum Debuggen
#endif


#endif // _ADC_H

