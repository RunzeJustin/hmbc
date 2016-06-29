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

//*-----------------------------------------------------------------------*
//* Includes                                                              *
//*-----------------------------------------------------------------------*

#include "config.h"
#include "define.h"
#include <stdint.h>

//*-----------------------------------------------------------------------*
//* Variables                                                             *
//*-----------------------------------------------------------------------*

UINT8  CPU_Running   = 0x00;              // Status nach Reset


INT8 verDate[]       = Date;              // für die Begrüssungsmeldung
INT8 verTime[]       = Time;
INT8 verStr[]        = Version;

//---- sprintf ----
INT8                 str[256];

BOOL debug_msg_on    = FALSE;
BOOL intern_cmd      = FALSE;

UINT32*              modePtr;
UINT32*              errorPtr;

UINT32               modePtrAdd;
UINT32               errorPtrAdd;

uint8_t * volatile resultPtr;
uint8_t * volatile commandPtr;
