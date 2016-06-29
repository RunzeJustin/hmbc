//****************************************************************************
//*                                                                          *
//*              _____   ____    _____         ___      ____                 *
//*             |  _  | |  _ `  |  ___|       |   `    / ___|                *
//*             | |_| | | | ` | | |_    ____  | [] |  | / __                 *
//*             |  ___| | | | | |  _|  |____| |  _ <  | ||_ |                *
//*             | |     | |_/ | | |___        | |_| | | |_| |                *
//*             |_|     |____/  |_____|       |____/   `___/                 *
//*                                                                          *
//*                                                                          *
//*  Project:          BIOSControl V850E2x                                   *
//*  (c) copyright     Harman International LIFESTYLE HW                     *
//****************************************************************************


#ifndef _IIC_1_H
#define _IIC_1_H


//*--------------------------------------------------------------------------*
//* Contains routines for controlling the 2nd IIC interface                  *
//*--------------------------------------------------------------------------*


//*--------------------------------------------------------------------------*
//* Prototypen for xxx - Routine                                             *
//*--------------------------------------------------------------------------*

extern UINT32 IIC_Write_1_Str   (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length, UINT8 Mode);
extern UINT32 IIC_Read_1_Str    (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length,
                                 UINT8 *RD_Buf, UINT8 RD_Length, UINT8 Mode);
extern void   IIC_Init_1        (void);


#endif // _IIC_1_H

