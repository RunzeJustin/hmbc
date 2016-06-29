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

#ifndef _DEFINE_H
#define _DEFINE_H

//****************************************************************************
//*  Befehle fuer Mikrocontroller                                            *
//****************************************************************************

#include "hbbios_command.h"
#include "v850_command.h"
#include "hbbios_define.h"

//*--------------------------------------------------------------------------*
//* Clockdefines                                                             *
//*--------------------------------------------------------------------------*

typedef enum
{
   CLK_DOMAIN_000 =  1,
   CLK_DOMAIN_006 =  2,
   CLK_DOMAIN_011 =  3,
   CLK_DOMAIN_012 =  4,
   CLK_DOMAIN_108 =  5,
   CLK_DOMAIN_112 =  6
} ClockDomains;

//*--------------------------------------------------------------------------*
//* Portzugriff                                                              *
//*--------------------------------------------------------------------------*

// by set / reset registers without cross effects interrupt
#define SETPORT_1(PORTNAME,BIT)     {PSR##PORTNAME   = ((UINT32)0x00010001 << BIT);}
#define SETPORT_0(PORTNAME,BIT)     {PSR##PORTNAME   = ((UINT32)0x00010000 << BIT);}
#define SETPORT_OUT(PORTNAME,BIT)   {PMSR##PORTNAME  = ((UINT32)0x00010000 << BIT);}
#define SETPORT_IN(PORTNAME,BIT)    {PMSR##PORTNAME  = ((UINT32)0x00010001 << BIT);}
#define SETPORT_PORT(PORTNAME,BIT)  {PMCSR##PORTNAME = ((UINT32)0x00010000 << BIT);}
#define SETPORT_ALT(PORTNAME,BIT)   {PMCSR##PORTNAME = ((UINT32)0x00010001 << BIT);}

#define SETJPORT_1(PORTNAME,BIT)    {JPSR##PORTNAME  = ((UINT32)0x00010001 << BIT);}
#define SETJPORT_0(PORTNAME,BIT)    {JPSR##PORTNAME  = ((UINT32)0x00010000 << BIT);}
#define SETJPORT_OUT(PORTNAME,BIT)  {JPMSR##PORTNAME = ((UINT32)0x00010000 << BIT);}
#define SETJPORT_IN(PORTNAME,BIT)   {JPMSR##PORTNAME = ((UINT32)0x00010001 << BIT);}
#define SETJPORT_PORT(PORTNAME,BIT) {JPMSR##PORTNAME = ((UINT32)0x00010000 << BIT);}
#define SETJPORT_ALT(PORTNAME,BIT)  {JPMSR##PORTNAME = ((UINT32)0x00010001 << BIT);}


// Port Bit zwar ausmaskiert aber kein echtes Bool !
#define INPORT(PORTNAME,BIT)        (PPR##PORTNAME   & ((UINT16)0x0001 << BIT))
#define INJPORT(PORTNAME,BIT)       (JPPR##PORTNAME  & ((UINT16)0x0001 << BIT))

// Port Bit als echtes Bool (TRUE oder FALSE)
#define INPORT_BOOL(PORTNAME,BIT)   ((PPR##PORTNAME   & ((UINT16)0x0001 << BIT))!=0)
#define INJPORT_BOOL(PORTNAME,BIT)  ((JPPR##PORTNAME  & ((UINT16)0x0001 << BIT))!=0)




//*--------------------------------------------------------------------------*
//* Definitions for the ports depends on the CPU                             *
//*--------------------------------------------------------------------------*
#if (CPU_TYPE == CPU_NEC_V850E2_DF3554)
   #define MAX_ADC_CHANNEL    23          // 0 to 23 , 12Bit Resolution

   #define PORT0        0
   #define PORT0IN      0xFFFF            // OK
	#define PORT0PP      0xFFFF
	#define PORT0OD      0xFFFF

	#define PORT1        1
   #define PORT1IN      0xFFFE            // OK (Bit 0 nicht vorhanden)
	#define PORT1PP      0xFFFE
	#define PORT1OD      0xFFFE

	#define PORT2        2
	#define PORT2IN      0x0001            // OK (only bit 0 available)
	#define PORT2PP      0x0001
	#define PORT2OD      0x0001

	#define PORT3        3
	#define PORT3IN      0x00FC            // only bits 2 to 7 present
	#define PORT3PP      0x00FC
	#define PORT3OD      0x00FC

	#define PORT4        4                 // OK (Bits 0 bis 11)
	#define PORT4IN      0x0FFF
	#define PORT4PP      0x0FFF
	#define PORT4OD      0x0FFF

	#define PORT10       10                // OK (Bits 6 bis 15)
	#define PORT10IN     0xFFC0
	#define PORT10PP     0xFFC0
	#define PORT10OD     0xFFC0

	#define PORT11       11                // OK (Bits 0 bis 7)
	#define PORT11IN     0x00FF
	#define PORT11PP     0x00FF
	#define PORT11OD     0x00FF

	#define PORT21       21                // OK (Bits 2 bis 11)
	#define PORT21IN     0x0FFC
	#define PORT21PP     0x0FFC
	#define PORT21OD     0x0FFC

	#define PORT25       25                // OK (Bits 0 bis 15)
	#define PORT25IN     0xFFFF
	#define PORT25PP     0xFFFF
	#define PORT25OD     0xFFFF

	#define PORT27       27                // OK (Bits 0 bis 2)
	#define PORT27IN     0x0007
	#define PORT27PP     0x0007
	#define PORT27OD     0x0007

 #define PORTJ0       0                 // OK (Bits 0 bis 5)
 #define PORTJ0IN     0x003F
 #define PORTJ0PP     0x003F
 #define PORTJ0OD     0x003F
#endif

#endif // _DEFINE_H
