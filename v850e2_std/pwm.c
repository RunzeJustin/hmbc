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
#include "hbbios.h"
#include "global.h"                 // globale Datenstrukturen
#include "tool.h"                 	// für hex2str usw.
#include "pwm.h"
#include "ints.h"
#include "hw_init.h"

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if CP_PWM

#define BIT0     0x0001
#define BIT1     0x0002
#define BIT2     0x0004
#define BIT3     0x0008
#define BIT4     0x0010
#define BIT5     0x0020
#define BIT6     0x0040
#define BIT7     0x0080
#define BIT8     0x0100
#define BIT9     0x0200
#define BIT10    0x0400
#define BIT11    0x0800
#define BIT12    0x1000
#define BIT13    0x2000
#define BIT14    0x4000
#define BIT15    0x8000



//*--------------------------------------------------------------------------*
//* Definitionen                                                             *
//*--------------------------------------------------------------------------*

#define STOP_ALL        0x00        // Stop-Commando
#define RUN_PWM         0x70        // Run-Commando für PWM
#define RUN_SQW         0x60        // Generate Sqare Wave (50% Dutycyle)

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*


//*--------------------------------------------------------------------------*
//* Interrupt handling macros                                                *
//*--------------------------------------------------------------------------*

// Hier alternative Portbelegungen verschaltern
#if ((CPU_TYPE==CPU_NEC_V850E2_DF3554) && (CP_PWM == 1))    // check it IHE
   #define  TAUA0O2_PORT_UNCONFIG   {SETPORT_OUT(3,2); SETPORT_0(3,2); SETPORT_PORT(3,2);}
   #define  TAUA0O2_PORT_CONFIG     {PFC3 |= 0x0004; PFCE3 &= ~0x0004; SETPORT_OUT(3,2); SETPORT_ALT(3,2);}

   #define  TAUA0O3_PORT_UNCONFIG   {SETPORT_OUT(3,3);SETPORT_0(3,3);SETPORT_PORT(3,3);}
   #define  TAUA0O3_PORT_CONFIG     {PFC3 |= 0x0008; PFCE3 &= ~0x0008; SETPORT_OUT(3,3); SETPORT_ALT(3,3);}

   #define  TAUA0O4_PORT_UNCONFIG   {SETPORT_OUT(3,4); SETPORT_0(3,4); SETPORT_PORT(3,4);}
   #define  TAUA0O4_PORT_CONFIG     {PFC3 |= 0x0010;PFCE3 &= ~0x0010; SETPORT_OUT(3,4); SETPORT_ALT(3,4);}

   #define  TAUA0O5_PORT_UNCONFIG   {SETPORT_OUT(3,5);SETPORT_0(3,5); SETPORT_PORT(3,5);}
   #define  TAUA0O5_PORT_CONFIG     {PFC3 |= 0x0020;PFCE3 &= ~0x0020; SETPORT_OUT(3,5); SETPORT_ALT(3,5);}

   #define  TAUA0O6_PORT_UNCONFIG   {SETPORT_OUT(3,6); SETPORT_0(3,6); SETPORT_PORT(3,6);}
   #define  TAUA0O6_PORT_CONFIG     {PFC3 |= 0x0040; PFCE3 &= ~0x0040; SETPORT_OUT(3,6); SETPORT_ALT(3,6);}

   #define  TAUA0O7_PORT_UNCONFIG   {SETPORT_OUT(3,7); SETPORT_0(3,7); SETPORT_PORT(3,7);}
   #define  TAUA0O7_PORT_CONFIG     {PFC3 |= 0x0080; PFCE3 &= ~0x0080; SETPORT_OUT(3,7); SETPORT_ALT(3,7);}



   #define  TAUA0O10_PORT_UNCONFIG  {SETPORT_OUT(1,10); SETPORT_0(1,10); SETPORT_PORT(1,10);}
   #define  TAUA0O10_PORT_CONFIG    {PFC1 &= ~0x0400;PFCE1 &= ~0x0400; SETPORT_OUT(1,10); SETPORT_ALT(1,10);}

   #define  TAUA0O11_PORT_UNCONFIG  {SETPORT_OUT(1,11); SETPORT_0(1,11); SETPORT_PORT(1,11);}
   #define  TAUA0O11_PORT_CONFIG    {PFC1 &= ~0x0800; PFCE1 &= ~0x0800; SETPORT_OUT(1,11); SETPORT_ALT(1,11);}

   #define  TAUA0O12_PORT_UNCONFIG  {SETPORT_OUT(1,12); SETPORT_0(1,12); SETPORT_PORT(1,12);}
   #define  TAUA0O12_PORT_CONFIG    {PFC1 &= ~0x1000; PFCE1 &= ~0x1000; SETPORT_OUT(1,12); SETPORT_ALT(1,12);}




   #define  TAUA0O13_PORT_UNCONFIG  {SETPORT_OUT(4,0); SETPORT_0(4,0); SETPORT_PORT(4,0);}
   #define  TAUA0O13_PORT_CONFIG    {PFC4 |= 0x0001; PFCE4 &= ~0x0001; SETPORT_OUT(4,0); SETPORT_ALT(4,0);}

   #define  TAUA0O14_PORT_UNCONFIG  {SETPORT_OUT(4,1); SETPORT_0(4,1); SETPORT_PORT(4,1);}
   #define  TAUA0O14_PORT_CONFIG    {PFC4 |= 0x0002; PFCE4 &= ~0x0002; SETPORT_OUT(4,1); SETPORT_ALT(4,1);}

   #define  TAUA0O15_PORT_UNCONFIG  {SETPORT_OUT(4,2); SETPORT_0(4,2); SETPORT_PORT(4,2);}
   #define  TAUA0O15_PORT_CONFIG    {PFC4 |= 0x0004; PFCE4 &= ~0x0004; SETPORT_OUT(4,2); SETPORT_ALT(4,2);}
#endif

//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

void TAUA0_n_Stop                (UINT8 Channel1, UINT8 Channel2);
void TAUA0_n_Start               (UINT8 Channel1, UINT8 Channel2);
void TAUA0_n_Config_Output_SQR   (UINT8 Channel);
void TAUA0_n_Config_Output_PWM   (UINT8 Channel);
void TAUA0_n_Disable_Output      (UINT8 Channel);

//****************************************************************************
//* Function     :  PWM                                                      *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*                                                                          *
//*	  Das Tastverhältnis kann für jedes Modul separat eingestellt          *
//*     werden                                                               *
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
extern volatile UINT8  PWM_Duty;
void PWM (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Freq_Soll,     // Frequenz des PWM-Signals
         Freq_Ist = 0,
         Duty_Soll,     // Dutyverhältnis des PWM-Signals
         Duty_Ist = 0,
         Error;

UINT8    PWM_TimerNr,
         PWM_Mode;

UINT32   Puls_Cycle,   // Registerwerte fuer Timer
         Puls_Width;

UINT8    Clk_Select;
UINT32   Clk_Selected;
UINT32   P_Clk;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // PWM Auswerten
   PWM_Mode    = (commandPtr[3] & 0xF0);           // Mode
   PWM_TimerNr = (commandPtr[3] & 0x0F);           // Timernummer
   Freq_Soll   = readLong (commandPtr+4);          // Frequence
   Duty_Soll   = readWord (commandPtr+8);          // Duty Cycle 0,00 .. 99,99


   // Clamp Range Setup
   if (Freq_Soll > 40000000)
      Freq_Soll = 40000000;

   if (Freq_Soll == 0)
      Freq_Soll = 1;

   if (Duty_Soll > 10000)
      Duty_Soll = 10000;

// --------------------------------------------------------------------------

//Clock Initialize for TAUA Timer
   P_Clk = GetDomainClock (CLK_DOMAIN_006);           // Clock Domain Setup For TAUA Timer, 48 MHz

   // Prescaler in 16-Schritten einstellen
   //TAUA0BRS = 0;
   //Since all PRS set to A(10d), CK1~3 been set to PCLK/2E10, PCLK is clock domain 06, 48 MHz/1024=46875 hz
   TAUA0TPS = PRS0(0) | PRS1(4) | PRS2(8) | PRS3(12);

// --------------------------------------------------------------------------

   // Search for the optimize clock
   if (Freq_Soll > (P_Clk / 0x10000))
   {
      Clk_Select = 0;
   }
   else
   {
      if (Freq_Soll > (P_Clk / 0x100000))
      {
         Clk_Select = 1;
      }
      else
      {
         if (Freq_Soll > (P_Clk / 0x1000000))
         {
            Clk_Select = 2;
         }
         else
         {
            Clk_Select = 3;
         }
      }
   }

   // Slected Clock calculation
   Clk_Selected =  P_Clk / (1 << (Clk_Select * 4));

// --------------------------------------------------------------------------

   if (PWM_Mode == RUN_PWM)
   {
      // Calculate the Period
      Puls_Cycle = (UINT32)(Clk_Selected / Freq_Soll) -1;

       //Calculate the DC 
      Puls_Width = ((Puls_Cycle +1) * Duty_Soll) / 10000;

      // For DC Safe, Clamp it
      if (Puls_Cycle > 0xFFFF)
         Puls_Cycle = 0xFFFF;

      if (Puls_Cycle == 0)
         Puls_Cycle = 1;

      if (Puls_Width >= Puls_Cycle)
         Puls_Width = Puls_Cycle -1 ;

      // Actually Calculation Value
      Freq_Ist  = Clk_Selected / (UINT32)(Puls_Cycle +1);
      Duty_Ist  = (Puls_Width * 10000) / (Puls_Cycle +1);
   }

// --------------------------------------------------------------------------

   if (PWM_Mode == RUN_SQW)
   {
       // Calculate the Period
      Puls_Cycle = (UINT32)((Clk_Selected / Freq_Soll) / 2) -1;

      // For DC Safe, Clamp it
      if (Puls_Cycle > 0xFFFF)
         Puls_Cycle = 0xFFFF;

      // Actually Calculation Value
      Freq_Ist  = Clk_Selected / (UINT32)((Puls_Cycle +1) *2);
      Duty_Ist = 5000;
   }

   Error = NO_ERROR;
   resultPtr[0] = 15;
   writeLong(resultPtr+8, Freq_Ist);
   writeWord(resultPtr+12, Duty_Ist);
   resultPtr[14] = commandPtr[10];


   // PWM stop
   if (PWM_Mode == STOP_ALL)
   {
      switch (PWM_TimerNr)
      {
         case 11:     // Stop Timer
                     TAUA0_n_Stop (PWM_TimerNr, PWM_TimerNr);               
                     TAUA0O11_PORT_UNCONFIG;
                     break;

         default:    resultPtr[0] = 8;
                     Error = ERROR_PWM | ERROR_PWM_BIT;
                     break;

      }
   } // if PWM_Mode

   // PWM start
   else if (PWM_Mode == RUN_PWM)
   {
      if (!(PWM_TimerNr & 1))
      {
         resultPtr[0] = 8;
         Error = ERROR_PWM | ERROR_PWM_BIT;
      }
      else
      switch (PWM_TimerNr)
      {
         case 11:
                  // Set Masterchannel 10
                  TAUA0_n_Stop (PWM_TimerNr, PWM_TimerNr-1);         // stop both timer
                  ICTAUA0I10 |= INT_DISABLE;                          // Disable interrupts
                  TAUA0CMOR10 = CKS (Clk_Select) | MAS | MD(1);      // Intervaltimer Mode 

                  TAUA0_n_Disable_Output(PWM_TimerNr-1);             // Output off
                  TAUA0CDR10 = Puls_Cycle;                           // Period setup

                  // Set Slavechannel 11
                  ICTAUA0I11 |= INT_DISABLE;                         // Disableinterrupts
                  TAUA0CMOR11 = CKS (Clk_Select) | STS(4) | MD(9);   // One Count Mode

                  TAUA0_n_Config_Output_PWM(PWM_TimerNr);
                  TAUA0CDR11 = Puls_Width;                           // Duty

                  TAUA0O11_PORT_CONFIG;                              // Turn off PWM output

                  TAUA0_n_Start(PWM_TimerNr, PWM_TimerNr-1);         // start timer
                  break;

         default:    resultPtr[0] = 8;
                     Error = ERROR_PWM | ERROR_PWM_BIT;
                     break;

      }
   } // if PWM_Mode

   // Start SquareWave
   else if (PWM_Mode == RUN_SQW)
   {
      switch (PWM_TimerNr)
      {

         default:    resultPtr[0] = 8;
                     Error = ERROR_PWM | ERROR_PWM_BIT;
                     break;
      }
   } // if PWM_Mode
   else
   {
      resultPtr[0] = 8;
      Error = ERROR_PWM | ERROR_PWM_MODE;
   }

   writeLong(resultPtr+4, Error);

} // end of function


//****************************************************************************
//* Function     :  SQW                                                      *
//****************************************************************************
//* Description  :  Hilfsfunktionen                                          *
//*                                                                          *
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

void TAUA0_n_Stop (UINT8 Channel1, UINT8 Channel2)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // stop timer for initialize
   TAUA0TT = (0x0001 << Channel1) | (0x0001 << Channel2);
}  // end of function


void TAUA0_n_Start (UINT8 Channel1, UINT8 Channel2)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // starts timer
   TAUA0TS = (0x0001 << Channel1) | (0x0001 << Channel2);
}  // end of function


void TAUA0_n_Disable_Output (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Output Togglemode on
   TAUA0TOE &= ~(0x0001 << Channel);
}  // end of function


void TAUA0_n_Config_Output_SQR (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Output Togglemode on
   TAUA0TOE |=  (0x0001 << Channel);
   TAUA0TOM &= ~(0x0001 << Channel);
   TAUA0TOC &= ~(0x0001 << Channel);
   TAUA0TDE &= ~(0x0001 << Channel);
   //TAUA0TRE &= ~(0x0001 << Channel);
   //TAUA0TME &= ~(0x0001 << Channel);
   //TAUA0TDM &= ~(0x0001 << Channel);
   TAUA0TOL &= ~(0x0001 << Channel);
   TAUA0TDL &= ~(0x0001 << Channel);

} // end of function


void TAUA0_n_Config_Output_PWM (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Output synchron PWM on
   TAUA0TOE |=  (0x0001 << Channel);
   TAUA0TOM |=  (0x0001 << Channel);
   TAUA0TOC &= ~(0x0001 << Channel);
   TAUA0TDE &= ~(0x0001 << Channel);
   //TAUA0TRE &= ~(0x0001 << Channel);
   //TAUA0TME &= ~(0x0001 << Channel);
   //TAUA0TDM &= ~(0x0001 << Channel);
   TAUA0TOL &= ~(0x0001 << Channel);
   TAUA0TDL &= ~(0x0001 << Channel);

} // end of function

#endif // #if CP_PWM


