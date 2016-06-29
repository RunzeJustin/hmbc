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



#ifndef _HW_INIT_H
#define _HW_INIT_H

//*--------------------------------------------------------------------------*
//* Initialisieren der Ports usw.                                            *
//*--------------------------------------------------------------------------*

#define LOW     0                    		// Port: Level: Signal Low
#define HIGH    1                       	// Port: Level: Signal High

#define IN      1                       	// Port: I/O:   Input
#define PP      0                       	// Port: I/O:   Output -> type Push-Pull
#define OD      2                       	// Port: I/O:   Output -> type Open Drain
#define IGNORE  0xFF                    	// Port: I/O:   alten Wert lassen

#define ANALOG IN                         // wie Input

//*--------------------------------------------------------------------------*
//* internal functions                                                       *
//*--------------------------------------------------------------------------*


void Port_Init    (void);
void Treiber_Init (void);
void CPU_Init     (void);
void Businterface_Init (void);

UINT32 GetDomainClock(UINT32 Domain);

//*--------------------------------------------------------------------------*
//* Businterface                                                             *
//*--------------------------------------------------------------------------*

#if (BUSINTERFACE == BUSINTERFACE_MISSING)
   #if (CPU_TYPE==CPU_NEC_V850E2_DF3558)
      #error BUSINTERFACE-Definitions CPU_NEC_V850E2_DF3558 missing
   #endif
#endif


//*--------------------------------------------------------------------------*
//* Businterface                                                             *
//*--------------------------------------------------------------------------*

#if (BUSINTERFACE == NO_BUS)
   #error BUSINTERFACE-Definitions CPU_NEC_V850E2_DF3558 missing
#endif

//*--------------------------------------------------------------------------*

#if (BUSINTERFACE == CS2_RAM_4M_16BIT_MPX)
   #if (CPU_TYPE==CPU_NEC_V850E2_DF3558)
   // CS2 : RAM

   // set busmode  (CS2 : 16 bit) ; multiplexed Addresses
   #define SET_BSC         { BSC &= ~BS21; BSC |= BS20;}

   // set little endian
   #define SET_DEC         { DEC &= ~DE2;}

   // set bus cycle type
   #define SET_BCT         { BCT0 &= ~(BCT20 | BCT21); BCT0 |= ME2;}

   // set data wait 15 clocks
   #define SET_DWC         { DWC0 |= (DW20 | DW21 | DW22 | DW23);}

   // set data hold 3 clocks
   #define SET_DHC         { DHC |= (DH20 | DH21);}

   // set data setup 3 clocks
   #define SET_DSC         { DSC |= (DS20 | DS21);}

   // set Adress setup wait 3 clocks, hold wait 3 clocks
   #define SET_AWC         { AWC0 = (ASW20 | ASW21 | AHW20 | AHW21);}

   // set Idle cycle after read 3 clocks, after write 3 clocks
   #define SET_ICC         { ICC0 = (RIC20 | RIC21 | WIC20 | WIC21);}

   // disable external wait error
   #define SET_EWC         { EWC &= ~EW2;}

   // 16 Bit Datenbus +16 Bit Addressen gemultiplexed aktiv
   #define SET_DATABUS     { PFC25 = 0x0000; PFCE25 = 0x0000; PM25 = 0x0000; PMC25 = 0xFFFF; PIPC25 = 0xFFFF;}

   // Adressbus A16-A18, ASTB aktiv
   #define SET_ADRESSBUS   { PFC27 &= ~0x07; PFC27 |= 0x20; PFCE27 &= ~0x27; PM27 &= ~0x27; PMC27 |=0x27;}

   // RW-Steuersignale 16 Bit RD, WR
   #define SET_CONTROL     { PFC21 &= ~0x023C; PFCE21 &= ~0x023C; PM21 &= ~0x023C; PMC21 |=0x023C;}

   #define SET_SPECIAL     {;}

   #endif  // CPU_TYPE != DF3558

//*--------------------------------------------------------------------------*

   #else
      #if (CPU_TYPE==CPU_NEC_V850E2_DF3558)
         #error BUSINTERFACE-Definitions CS2_RAM_4M_16BIT_MPX missing
      #endif
#endif


#endif // _HW_INIT_H

