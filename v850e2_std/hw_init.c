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
#include "hw_init.h"
#include "global.h"

#if (CP_ADC)                     // ADC Modul
   #include "adc.h"
#endif
   #include "ints.h"
#if (CP_CAN0)                    // CAN-Modul
   #include "can.h"              // für das CAN - Modul
#endif
#if (CP_RDS1||CP_RDS2||CP_RDS3)  // RDS-Daten Kanal 1 oder 2
   #include "rds.h"
#endif
#if CP_CMDDEV_IIC
   #include "iicmsg.h"
#endif

#include "gateway.h"

#define  BIT8     (1<<8)
#define  BIT9     (1<<9)
#define  BIT12    (1<<12)




// ---------------------------------------------------------------------------
//****************************************************************************
//* Function     : HW_Init()/HW_DeInit()                                     *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Function   In r_port.h sind alle Port Pin mit den Parametern             *
//*            Portname,Port,Bit,InitDirection,InitPegel,                    *
//*            PowerDownDirection und PowerDownPegel beschrieben.            *
//*            Eingestellt werden diese Paramater ueber die Portregister     *
//*            PX, das Port Direction Register DPX und ggf. Open Drain       *
//*            Register OPDX (nicht bei allen Ports)                         *
//*            In HW_INIT wird r_port.h 3x included, wobei die MACROS        *
//*            jedesmal umdefiniert werden:                                  *
//*            Per Bit shiften werden fuer jedes Bit im Port die             *
//*            entsprechenden Werte an die richtige Position im              *
//*            16-Bit-Wort des Portregisters gesetzt                         *
//*                                                                          *
//*            1.Include: Festlegung der Portpegel          PX^X  =0/1       *
//*            2.Include: Festlegung des OpenDrain Register ODPX^X=OD/PP     *
//*            3.Include: Festlegung der Port Direction     DPX^X =IN/OUT    *
//*                                                                          *
//*            Bei HW_Init() werden die Ports fuer den Betriebsfall          *
//*            gesetzt (Init-Parameter INITLVL, INITDIR).                    *
//*            Bei HW_DeInit() werden die Ports so gesetzt, dass             *
//*            Ruhestrom moeglichst minimal wird, aber benoetigte WakeUp-    *
//*            Ports funktionieren.                                          *
//*                                                                          *
//*            !!! Hinweis zum Debuggen:                                     *
//*            Uebergibt man dem Compiler das CFLAG -P erzeugt er ein File   *
//*            *.I, in dem man die Aufloesung der MACROS durch den           *
//*            Praeprozessor sieht, also letzlich der Code den der           *
//*            C-Compiler uebersetzt                                         *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************
void Port_Init ()    // DF3554
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

   UINT32 PODC_VAL0,
          PODC_VAL1,
          PODC_VAL2,
          PODC_VAL3,
          PODC_VAL4,
          PODC_VAL10,
          PODC_VAL11,
          //PODC_VAL12,    // IHE
          PODC_VAL21,
          PODC_VAL25,
          PODC_VAL27,
          JPODC_VAL0;

//*--------------------------------------------------------------------------*
//* Defines vorbereiten                                                      *
//*--------------------------------------------------------------------------*

   // als erstes eventuelle alte Definitionen löschen
   #undef  JOD_PORTDEF
   #undef  JOD_ENDPORTDEF
   #undef  JOD_IOPORT
   #undef  OD_PORTDEF
   #undef  OD_ENDPORTDEF
   #undef  OD_IOPORT

   #define JOD_ENDPORTDEF(PORTNAME) ;      // Zeilenende für C-Code ";"
   #define OD_ENDPORTDEF(PORTNAME) ;       // Zeilenende für C-Code ";"

//*--------------------------------------------------------------------------*
//* Ausgangsregister aller Ports setzen                                      *
//*--------------------------------------------------------------------------*

   #define JOD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL) |((INITLVL&0x01)<<BIT)
   #define OD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)  |((INITLVL&0x01)<<BIT)

   #define JOD_PORTDEF(PORTNAME)    JP##PORTNAME=0x00
   #define OD_PORTDEF(PORTNAME)     P##PORTNAME=0x00

   #include "portlist.h"            // hier Importierung Portlisten-File

   #undef  JOD_IOPORT
   #undef  JOD_PORTDEF
   #undef  OD_IOPORT
   #undef  OD_PORTDEF

//*--------------------------------------------------------------------------*
//* Open Drain Konfigurationsregister aller Ports setzen                     *
//*--------------------------------------------------------------------------*

#define JOD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL) |(((INITDIR&0x02)/2)<<BIT)
#define OD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)  |(((INITDIR&0x02)/2)<<BIT)

#define JOD_PORTDEF(PORTNAME)    JPODC_VAL##PORTNAME=0x00
#define OD_PORTDEF(PORTNAME)     PODC_VAL##PORTNAME=0x00

#include "portlist.h"            // hier Importierung Portlisten-File

#undef  JOD_IOPORT
#undef  JOD_PORTDEF
#undef  OD_IOPORT
#undef  OD_PORTDEF

//*--------------------------------------------------------------------------*
//* Direction Konfigurationsregister aller Ports setzen                      *
//*--------------------------------------------------------------------------*

   #define JOD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL) |((INITDIR&0x01)<<BIT)
   #define OD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)  |((INITDIR&0x01)<<BIT)

   #define JOD_PORTDEF(PORTNAME)    JPM##PORTNAME=0x00
   #define OD_PORTDEF(PORTNAME)     PM##PORTNAME=0x00

   #include "portlist.h" // hier Importierung Portlisten-File

//*--------------------------------------------------------------------------*
//* ODC-Register hier physikalisch setzen                                    *
//*--------------------------------------------------------------------------*

   // protected write         // Port 0 
   PPCMD0=0xA5;
   PODC0 = PODC_VAL0;        
   PODC0 = ~PODC_VAL0;
   PODC0 = PODC_VAL0;

   // protected write         // Port 1 
   PPCMD1=0xA5;
   PODC1 = PODC_VAL1;
   PODC1 = ~PODC_VAL1;
   PODC1 = PODC_VAL1;

   // protected write
   PPCMD2=0xA5;
   PODC2 = PODC_VAL2;
   PODC2 = ~PODC_VAL2;
   PODC2 = PODC_VAL2;
   
   // protected write         // Port 3
   PPCMD3=0xA5;
   PODC3 = PODC_VAL3;
   PODC3 = ~PODC_VAL3;
   PODC3 = PODC_VAL3;

   // protected write         // Port 4
   PPCMD4=0xA5;
   PODC4 = PODC_VAL4;
   PODC4 = ~PODC_VAL4;
   PODC4 = PODC_VAL4;

   // protected write         // Port 10
   PPCMD10=0xA5;
   PODC10 = PODC_VAL10;
   PODC10 = ~PODC_VAL10;
   PODC10 = PODC_VAL10;

   // protected write         // Port 11 
   PPCMD11=0xA5;
   PODC11 = PODC_VAL11;
   PODC11 = ~PODC_VAL11;
   PODC11 = PODC_VAL11;

   // protected write
   PPCMD21=0xA5;
   PODC21 = PODC_VAL21;
   PODC21 = ~PODC_VAL21;
   PODC21 = PODC_VAL21;

   // protected write
   PPCMD25=0xA5;
   PODC25 = PODC_VAL25;
   PODC25 = ~PODC_VAL25;
   PODC25 = PODC_VAL25;

   // protected write
   PPCMD27=0xA5;
   PODC27 = PODC_VAL27;
   PODC27 = ~PODC_VAL27;
   PODC27 = PODC_VAL27;
   
   // protected write
   JPPCMD0=0xA5;
   JPODC0 = JPODC_VAL0;
   JPODC0 = ~JPODC_VAL0;
   JPODC0 = JPODC_VAL0;

//*--------------------------------------------------------------------------*
//* sonstige Port Defaults                                                   *
//*--------------------------------------------------------------------------*

   // Port Input Buffer aller Ports setzen, sonst kann kein Pin gelesen werden
   PIBC0  =  PORT0IN;       // IHE
   PIBC1  =  PORT1IN;
   PIBC2  =  PORT2IN;
   PIBC3  =  PORT3IN;
   PIBC4  =  PORT4IN;
   PIBC10 =  PORT10IN;
   PIBC11 =  PORT11IN;
   PIBC21 =  PORT21IN;
   PIBC25 =  PORT25IN;
   PIBC27 =  PORT27IN;
   JPIBC0 =  PORTJ0IN;

} // end of function Port_Init

// -------------------------------------------------------------------------------------------------
//****************************************************************************
//* Function     :  Treiber_Init                                             *
//****************************************************************************
//* Description  :  komfortable Initialisierung mit Textausgaben             *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      : kommt erst nach Ausgabe des Bios-Header zum Laufen        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************
void Treiber_Init (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*


} // end of function Treiber_Init


// -------------------------------------------------------------------------------------------------
#if (CPU_TYPE == CPU_NEC_V850E2_DF3554)   // L122
void PowerDomainsInit(void)               // DF3554
{
    UINT32 value;
    UINT32 value_low;  // low speed

value     = (0x1C << 1);      // 0x1C = Clock Source ID for PLL1 / 1  = 48MHz/1 = 48MHz
value_low = (0x01 << 1);      // 0x01 = Low Speed IntOsc [240kHz] / 1 = 240kHz


#if 0
//--------------------------------
// configure CPUCLK
//--------------------------------
PROTCMD0 = 0xA5;
CKSC_000 = (value<<1);                    //CPUCLK selected PLL0, since PLL0 has been locked
CKSC_000 = ~(value<<1);
CKSC_000 = (value<<1);
#endif

//--------------------------------
// configure PCLK for WDTA0,CLMA0,CLMA2
//--------------------------------
 PROTCMD2 = 0xA5;
 CKSC_A02 = value;        // 0x1C => PLL1 / 1 = 48MHz
 CKSC_A02 = ~value;
 CKSC_A02 = value;
 while (CSCSTAT_A02 != (value+1)) {;}      // wait for clock stable

//--------------------------------
// configure PCLK for TAUA0 Used for Timer Input Check
//--------------------------------
 PROTCMD0 = 0xA5;
 CKSC_006 = value;        // 0x1C => PLL1 / 1 = 48MHz
 CKSC_006 = ~value;
 CKSC_006 = value;
 while (CSCSTAT_006 != (value+1)) {;}      // wait for clock stable

//--------------------------------
// configure PCLK for IICB0 and CSIG0 
//--------------------------------
 PROTCMD1 = 0xA5;
 CKSC_108 = value;      // 0x1C => PLL1 / 1 = 48MHz
 CKSC_108 = ~value;
 CKSC_108 = value;
 while (CSCSTAT_108 != (value+1)) {;}      // wait for clock stable

//--------------------------------
// configure PCLK for URTE11 and CSIG4 
//--------------------------------
 PROTCMD0 = 0xA5;
 CKSC_011 = value;         // 0x1C => PLL1 / 1 = 48MHz
 CKSC_011 = ~value;
 CKSC_011 = value;
 while (CSCSTAT_011 != (value+1)) {;}      // wait for clock stable

//--------------------------------
// configure PCLK for ADAA0 
//--------------------------------
 PROTCMD0 = 0xA5;
 CKSC_012 = value;      // 0x1C => PLL1 / 1 = 48MHz
 CKSC_012 = ~value;
 CKSC_012 = value;
 while (CSCSTAT_012 != (value+1)) {;}      // wait for clock stable

//--------------------------------
// configure PCLK for OSTM0 
//--------------------------------
 PROTCMD1 = 0xA5;
 CKSC_112 = value;     // 0x0C => MainOsc / 1 = 12MHz
 CKSC_112 = ~value;   
 CKSC_112 = value;
 while (CSCSTAT_112 != (value+1)) {;}      // wait for clock stable

 // domain:  AWO_7 (only low speed) WDTA0: WDTACKI()??? set to 240 khz
 PROTCMD2 =  0xA5;
 CKSC_A07 =  value_low;
 CKSC_A07 = ~value_low;
 CKSC_A07 =  value_low;
 while (CSCSTAT_A07 != (value_low+1)) {;}  // wait for clock stable

// domain:  ISO0_7 (max. 240kHz), Clock domain for WDTA1:WDTATCKI(??????)
PROTCMD0 =  0xA5;
CKSC_007 =  value_low;
CKSC_007 = ~value_low;
CKSC_007 =  value_low;
 while (CSCSTAT_007 != (value_low+1)) {;}   // wait for clock stable

}

#else
   #error CPU Type not supported
#endif


// -------------------------------------------------------------------------------------------------
void DisableAllInts(void)
{
   volatile UINT16   *pointer;
   UINT8    cnt, max;

   #if (CPU_TYPE == CPU_NEC_V850E2_DF3554)    // L122 Interrupt Register Initialize
   UINT16   IntArray[] =
   {
     0x000, 0x002, 0x004,        0x008, 0x00a, 0x00c, 0x00e, 0x010, 0x012, 0x014, 0x016, 0x018, 0x01a, 0x01c, 0x01e,
     0x020, 0x022, 0x024, 0x026, 0x028, 0x02a, 0x02c, 0x02e, 0x030, 0x032, 0x034, 0x036, 0x038, 0x03a, 0x03c, 0x03e,
     0x040, 0x042, 0x044, 0x046, 0x048, 0x04a, 0x04c, 0x04e, 0x050, 0x052, 0x054, 0x056, 0x058, 0x05a, 0x05c, 0x05e,
     0x060, 0x062, 0x064, 0x066, 0x068, 0x06a, 0x06c, 0x06e, 0x070, 0x072, 0x074, 0x076, 0x078, 0x07a, 0x07c, 0x07e,
     0x080, 0x082, 0x084, 0x086, 0x088, 0x08a, 0x08c, 0x08e, 0x090, 0x092, 0x094, 0x096, 0x098, 0x09a, 0x09c, 0x09e,
     0x0a0, 0x0a2, 0x0a4, 0x0a6, 0x0a8, 0x0aa, 0x0ac, 0x0ae, 0x0b0, 0x0b2, 0x0b4, 0x0b6, 0x0b8, 0x0ba, 0x0bc, 0x0be,
     0x0c0, 0x0c2, 0x0c4, 0x0c6, 0x0c8, 0x0ca, 0x0cc, 0x0ce, 0x0d0, 0x0d2, 0x0d4, 0x0d6, 0x0d8, 0x0da, 0x0dc, 0x0de,
                                                      0x0ee, 0x0f0, 0x0f2, 0x0f4, 0x0f6, 0x0f8, 0x0fa, 0x0fc,
     0x100, 0x102,               0x108, 0x10a, 0x10c, 0x10e, 0x110, 0x112, 0x114, 0x116, 0x118, 0x11a, 0x11c,
                   0x124, 0x126,
     0x140, 0x142, 0x144, 0x146, 0x148,                      0x150, 0x152, 0x154, 0x156, 0x158,
     0x160, 0x162, 0x164, 0x166,                                                  0x176, 0x178, 0x17a, 0x17c, 0x17e,
            0x182, 0x184, 0x186, 0x188, 0x18a, 0x18c, 0x18e, 0x190, 0x192, 0x194, 0x196, 0x198, 0x19a,
     0x1a0, 0x1a2, 0x1a4, 0x1a6,                                                         0x1b8,
            0x1c2, 0x1c4, 0x1c6,
                                                             0x1f0, 0x1f2, 0x1f4, 0x1f6, 0x1f8, 0x1fa
   };   
   #else
      #error CPU Type not supported
   #endif

   max = (sizeof(IntArray) / sizeof(IntArray[0]));   // DIM

   for (cnt = 0; cnt < max; cnt++)
   {
      pointer  = (volatile UINT16*)(IntArray[cnt]+0xFFFF6000);
      *pointer = 0x008F; 
   }
}


// -------------------------------------------------------------------------------------------------
//****************************************************************************
//* Function     :  CPU_Init                                                 *
//****************************************************************************
//* Description  :  absolute Grundinitialisierung der CPU                    *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************
void CPU_Init (void)
{
#if (CPU_TYPE == CPU_NEC_V850E2_DF3554) // L122
UINT32  value, value1;

// clear I/O Buffer Hold (required after deepstop) -------------------------
PROTCMD2=0xA5;
PSC0 = 0x08;
PSC0 = ~0x08;
PSC0 = 0x08;

PROTCMD2=0xA5;
PSC1 = 0x08;
PSC1 = ~0x08;
PSC1 = 0x08;

// Enable MainOSC --------------------------------------------------------
MOSCST = 0x0F;          // main osz stabilisation time = 2^17/8MHz = 16.38ms

PROTCMD2 = 0xA5;         // protection access
MOSCE = 0x01;
MOSCE = ~0x01;
MOSCE = 0x01;
while (MOSCS != 0x07) {}

//--------------------------------
// configure CPUCLK to use MainOsc
//--------------------------------
PROTCMD0 = 0xA5;
CKSC_000 = (0x0C<<1);
CKSC_000 = ~(0x0C<<1);
CKSC_000 = (0x0C<<1);

//Disable PLL0 and PLL1
 PROTCMD2 = 0xA5;
 PLLE0 = 0x02;
 PLLE0 = ~0x02;
 PLLE0 = 0x02;

 PROTCMD2 = 0xA5;
 PLLE1 = 0x02;
 PLLE1 = ~0x02;
 PLLE1 = 0x02;

//Configure PLL0 to no function
PROTCMD2 =   0xA5;
PLLE0    =  0x00;
PLLE0    = ~0x00;
PLLE0    =  0x00;
PLLST0   =   0x04;

switch (MAIN_OSZ)    // hier eventuell ergaenzen wenn anderer QuarzOszillator
{
   case  4:        // 4 MHz
      value  = 0x00800227;    // PLL0 --> (39+1) / 2 *  4 = 80MHz CPU Clock
      value1 = 0x00800327;    // PLL1 --> (39+1) / 4 *  4 = 40MHz Peripheral Clock
   break;

   case 12:       // 12 MHz
      value  = 0x0080020A;    // Pll0 --> (10 +1) / 2 * 12 = 66MHz CPU Clock
      value1  = 0x0080030F;    // Pll0 --> (15 +1) / 4 * 12 = 48MHz CPU Clock
   break;

   case 16:       // 16 MHz
      value  = 0x00800209;    // Pll0 --> (9 +1) / 2 * 16 = 80MHz CPU Clock
      value1 = 0x0080030B;    // PLL1 --> (11+1) / 4 * 16 = 48MHz Peripheral Clock
   break;

   case 20:       // 20 MHz
      value  = 0x00800207;    // Pll0 --> (7 +1) / 2 * 20 = 80MHz CPU Clock
      value1 = 0x00800307;    // PLL1 --> (7 +1) / 4 * 20 = 40MHz Peripheral Clock
   break;
}

// set PLL0 to 66MHz -------------------------------------------------------
PLLC0 = value;

// Configure PLL1 to no function
PROTCMD2 = 0xA5; 
PLLE1    =  0x00;
PLLE1    = ~0x00;
PLLE1    =  0x00;
PLLST1   = 0x04;

PLLC1    = value1;

// set enable trigger for PLL0 and PLL1
PROTCMD2 = 0xA5;
PLLE0    =  0x01;
PLLE0    = ~0x01;
PLLE0    =  0x01;
    
PROTCMD2 = 0xA5;
PLLE1    =  0x01;
PLLE1    = ~0x01;
PLLE1    =  0x01;

// wait for stable PLL0 & PLL1
while ( (PLLS0 != 0x07) || (PLLS1 != 0x07) )  {;}

//--------------------------------
// configure CPUCLK
//--------------------------------
PROTCMD0 = 0xA5;
CKSC_000 = (0x14<<1);                    //CPUCLK selected PLL0, since PLL0 has been locked
CKSC_000 = ~(0x14<<1);
CKSC_000 = (0x14<<1);

#else
   #error CPU Type not supported
#endif

   // gemeinsamer Teil
   DisableAllInts();
   PowerDomainsInit();     //Powe Domain Init, setup different clock domain

   // IIC Hardware init, Setup Driver
   #if CP_IIC0
      IIC0_HWINIT;
   #endif
   
   // IIC Software init, Setup Driver
#if CP_IIC1
      IIC1_HWINIT;
#endif

//*--------------------------------------------------------------------------*

   #if CP_ADC
   ADC_Init();
   #endif

   // define for externen Clock in config.h, But not used here, it's empty function
   SET_CLOCKOUT;


   // set global interrupt enable flag
   __EI();

   // Systemtimer für Delays und Timeouts initialisieren
   dlyini();

   // Adress Assignment?????
   modePtr  = &modePtrAdd;
   errorPtr = &errorPtrAdd;

   // Mode to set start value
   setModeReg(MODE_STARTVALUE);

}


// -------------------------------------------------------------------------------------------------
UINT32 GetPLL_Clock(UINT8 PLL_Number)
{
   volatile UINT32   RegValue = 0;
   UINT8 Teiler, Multi;


   switch (PLL_Number)
   {
      case 0: RegValue = PLLC0; break;
      #if (CPU_TYPE == CPU_NEC_V850E2_DF3558)   //  wie im B140
      case 1: RegValue = PLLC1; break;
      case 2: RegValue = *(volatile UINT32*)(0xff425028); break;
      #elif (CPU_TYPE == CPU_NEC_V850E2_DF3554) // L122
      case 1:RegValue = PLLC1; break;
      #endif
   }

   Multi  = (RegValue & 0x7F)+1;

   switch ((RegValue & 0x300) >> 8)
   {
      case 1: Teiler = 0x01; break;
      case 2: Teiler = 0x02; break;
      case 3: Teiler = 0x04; break;
      default:Teiler = 0xFF; break;
   }
   return(MAIN_OSZ * Multi / Teiler);
}


// -------------------------------------------------------------------------------------------------
UINT32 GetDomainClock(UINT32 Domain)
{
   UINT32 value;
   UINT32 Pll0_Clock = 0, Pll1_Clock = 0, Pll2_Clock = 0;
   volatile UINT32   regValue;

   Pll0_Clock = GetPLL_Clock(0) * 1000000;    //  PLL0_Clock in MHz = 66,000,000
   Pll1_Clock = GetPLL_Clock(1) * 1000000;    //  PLL1_Clock in MHz = 48,000,000

   #if (CPU_TYPE == CPU_NEC_V850E2_DF3554)     // L122
   switch (Domain)
   {
      case (CLK_DOMAIN_000):  regValue = CKSC_000; break;   // stellt CPU Clock bereit
      case (CLK_DOMAIN_006):  regValue = CKSC_006; break;
      case (CLK_DOMAIN_011):  regValue = CKSC_011; break;
      case (CLK_DOMAIN_012):  regValue = CKSC_012; break;
      case (CLK_DOMAIN_108):  regValue = CKSC_108; break;
      case (CLK_DOMAIN_112):  regValue = CKSC_112; break;
   }
   #else
      #error CPU Type not supported
   #endif

   regValue = (regValue >> 1) & 0x0000003F;
   switch (regValue)
   {
      case 0x01: value =  240000;            break;
      case 0x03: value =   60000;            break;
      case 0x05: value =     469;            break;   // 240000 / 512 = 468,75
      case 0x07: value = 8000000;            break;
      case 0x08: value = 4000000;            break;
      case 0x09: value = 2000000;            break;
      case 0x0A: value = 1000000;            break;
      case 0x0B: value =  250000;            break;
      case 0x0C: value = MAIN_OSZ * 1000000; break;   // MAIN_OSZ ist in MHz z.B. 12
      case 0x0D: value = MAIN_OSZ *  500000; break;   // MAIN_OSZ / 2
      case 0x0E: value = MAIN_OSZ *  250000; break;   // MAIN_OSZ / 4
      case 0x0F: value = MAIN_OSZ *  125000; break;   // MAIN_OSZ / 8

      case 0x12: value = 32000;              break;   // 32kHz SubOsc

      case 0x14: value = (Pll0_Clock   );    break;
      case 0x15: value = (Pll0_Clock>>1);    break;
      case 0x17: value = (Pll0_Clock>>2);    break;
      case 0x1A: value = (Pll0_Clock>>3);    break;

      case 0x1C: value = (Pll1_Clock   );    break;
      case 0x1D: value = (Pll1_Clock>>1);    break;
      case 0x1F: value = (Pll1_Clock>>2);    break;
      case 0x22: value = (Pll1_Clock>>8);    break;
      case 0x23: value = (Pll1_Clock/10);    break;

      case 0x24: value = (Pll2_Clock   );    break;
      case 0x25: value = (Pll2_Clock>>1);    break;
      case 0x27: value = (Pll2_Clock>>2);    break;
      case 0x2A: value = (Pll2_Clock>>3);    break;

      case 0x3A: value = 8000000;            break;     // nicht ganz richtig, unter bestimmten umstaenden nur 240kHz

      default  : value = 0; break;           // nicht zulaessige Werte
   }

   return(value);
}


// -------------------------------------------------------------------------------------------------
//****************************************************************************
//* Function     :  Businterface_Init                                        *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************
void Businterface_Init (void)    // no Bus used
{
#if (CPU_TYPE == CPU_NEC_V850E2_DF3558)   // B140
   #if (BUSINTERFACE != BUSINTERFACE_MISSING)
   // set busmode
   SET_BSC;

   // set little endian
   SET_DEC;

   // set bus cycle type
   SET_BCT;

   // set data wait
   SET_DWC;

   // set data hold
   SET_DHC;

   // set data setup
   SET_DSC;

   // set Adress setup wait, hold wait
   SET_AWC;

   // set Idle cycle after read, after write
   SET_ICC;

   // set external wait error
   SET_EWC;

   // set Databus
   SET_DATABUS;

   // set Adressbus
   SET_ADRESSBUS;

   // RW-Steuersignale
   SET_CONTROL;

   // CPU-typische Besonderheiten
   SET_SPECIAL;
   #endif
#endif
} // end of function Businterface_Init

