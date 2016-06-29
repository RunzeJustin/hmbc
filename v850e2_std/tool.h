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


#ifndef _TOOL_H
#define _TOOL_H

#include "buffer.h"


//*--------------------------------------------------------------------------*
//* Funktionen                                                               *
//*--------------------------------------------------------------------------*

extern BOOL          Insert_Fifo_16 (UINT16 Wert, TBuffer *Buffer);
extern BOOL          Insert_Fifo_32 (UINT32 Wert, TBuffer *Buffer);
extern UINT16        Get_Fifo_16    (TBuffer *Buffer);
extern UINT32        Get_Fifo_32    (TBuffer *Buffer);

extern void          Statusabfrage  (void);
extern void          DelayLoop      (UINT32);
extern BOOL          Test_Ram       (UINT8 *Buffer);

#if CP_DEBUG_TOOL
extern void Debug_Format_Print      (UINT8 Channel, const INT8 *Format, ...);
extern void Debug_ASCII_Print       (UINT8 *Buffer);
extern void Debug_Textmessage_Print (UINT8 *Buffer);
extern void Debug_CMD_Print         (UINT8 *Buffer);
#endif

#endif // _TOOL_H


