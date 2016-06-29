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


//*--------------------------------------------------------------------------*
//* Include files                                                            *
//*--------------------------------------------------------------------------*

#include "config.h"
#include "define.h"

#include "iic_0.h"            		// für I2C-Busses
#include "tool.h"             		// für DelayLoop
#include "iic.h"
#include "iicmsg.h"

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if CP_IIC0

//*--------------------------------------------------------------------------*
//* Defines                                                                  *
//*--------------------------------------------------------------------------*

// Define the macros for the Port Mapping
#define  SDA_IN            IIC0_SDA_IN
#define  SCK_IN            IIC0_SCK_IN
#define  SDA_HIGH          IIC0_SDA_HIGH
#define  SDA_LOW           IIC0_SDA_LOW
#define  SCK_HIGH          IIC0_SCK_HIGH
#define  SCK_LOW           IIC0_SCK_LOW

// Remapping of the global functions
#define  IIC_InitX         IIC_Init_0
#define  IIC_WriteX_Str    IIC_Write_0_Str
#define  IIC_ReadX_Str     IIC_Read_0_Str

// Remapping of the global functions
#define  ACKxTRIALS        IIC0_ACK_TRIALS
#define  IICxCS            IIC0_CS
#define  IICxIGNORE_ACK    IIC0_IGNORE_ACK
#define  IICxINIT          IIC0_INIT
#define  IICxKBPS          IIC0_KBPS


// Embed Generic C drivers here
#include "iic_x.c"


#endif // #if CP_IIC0
