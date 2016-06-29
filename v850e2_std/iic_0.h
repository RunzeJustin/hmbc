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
//*  Project:          BIOSControl ST10F269                                  *
//*  (c) copyright     Harman-Becker Automotive Systems                      *
//****************************************************************************


#ifndef _IIC_0_H
#define _IIC_0_H


//*--------------------------------------------------------------------------*
//* Contains routines for controlling the 2nd IIC interface                  *
//*--------------------------------------------------------------------------*


//*--------------------------------------------------------------------------*
//* Prototypen fuer xxx - Routinen                                           *
//*--------------------------------------------------------------------------*

extern UINT32 IIC_Write_0_Str   (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length, UINT8 Mode);
extern UINT32 IIC_Read_0_Str    (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length,
                                 UINT8 *RD_Buf, UINT8 RD_Length, UINT8 Mode);
extern void   IIC_Init_0        (void);


#endif // _IIC_0_H

