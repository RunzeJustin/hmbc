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

#ifndef _CPU_DEF_H
#define _CPU_DEF_H

#include "hbbios_define.h"

//*--------------------------------------------------------------------------*
//* Definitionen Mehr-Byte-Variable                                          *
//*--------------------------------------------------------------------------*

#define FALSE              (UINT8)0       // Boolean-Typ false
#define TRUE               (UINT8)1       // Boolean-Typ true

#define false              FALSE
#define true               TRUE

#define UINT8              unsigned char
#define INT8               char
#define UINT16             unsigned short
#define INT16              signed short
#define UINT32             unsigned int
#define INT32              signed int
#define BOOL               unsigned char

//****************************************************************************
//* Inhalt       : SFR Adressen und Bits entsprechend dem Datenblatt des     *
//*                gewählten Prozessors                                      *
//****************************************************************************

#if (CPU_TYPE==CPU_NEC_V850E2_DF3554)
	#include "df3554_0.h"
   #include "df3554_irq.h"
   #include "df3554_defs.h"
#endif

#endif // _CPU_DEF_H


