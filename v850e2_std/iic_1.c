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
//*  (c) copyright     Harman Internation LIFESTYLE HW                       *
//****************************************************************************


//*--------------------------------------------------------------------------*
//* Include files                                                            *
//*--------------------------------------------------------------------------*

#include "config.h"
#include "define.h"

#include "iic_1.h"            		// for HW I2C
#include "tool.h"             		// for DelayLoop
#include "iic.h"
#include "iicmsg.h"

//*--------------------------------------------------------------------------*
//*	Master switch for the whole file from here                              *
//*--------------------------------------------------------------------------*

#if CP_IIC1

//*--------------------------------------------------------------------------*
//* Defines                                                                  *
//*--------------------------------------------------------------------------*

// Define the macros for the Port Mapping
#define  SDA_IN            IIC1_SDA_IN
#define  SCK_IN            IIC1_SCK_IN
#define  SDA_HIGH          IIC1_SDA_HIGH
#define  SDA_LOW           IIC1_SDA_LOW
#define  SCK_HIGH          IIC1_SCK_HIGH
#define  SCK_LOW           IIC1_SCK_LOW

// Remapping of the global functions
#define  IIC_InitX         IIC_Init_1
#define  IIC_WriteX_Str    IIC_Write_1_Str
#define  IIC_ReadX_Str     IIC_Read_1_Str

// Remapping of the global functions
#define  ACKxTRIALS        IIC1_ACK_TRIALS
#define  IICxCS            IIC1_CS
#define  IICxIGNORE_ACK    IIC1_IGNORE_ACK
#define  IICxINIT          IIC1_INIT
#define  IICxKBPS          IIC1_KBPS


// Embed Generic C drivers here
#include "iic_x.c"


#endif // #if CP_IIC1
