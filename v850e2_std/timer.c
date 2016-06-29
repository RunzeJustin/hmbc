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
#include "ints.h"
#include "timer.h"
#include "hw_init.h"


//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

static UINT32  timMrk[TIM_MAX_MRK];          // time marker
static BOOL    timMrkEnable[TIM_MAX_MRK];    // time marker enable

static volatile UINT32  GlobalTime;

// Hilfsfunktion nur zum Debuggen (nur fuer 8Mht int. Oszillator)
void Delay_us (UINT32 Wert)
{
   UINT32 timer;

   timer = Wert*2;

   while (1)
   {
      if (timer == 0)
         break;

      timer--;
   }
}

//****************************************************************************
//* Function     :  dlyini                                                   *
//****************************************************************************
//* Description  :  Initialisiert Timer M fuer die Delays                    *
//*                                                                          *
//*                                                                          *
//* Parameter    :  UINT16 Zeit in ms                                        *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

void dlyini(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    i;
UINT32   P_Clk;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Get Clock for the Module, CKSC_112 is for OSTM0, is 48 MHz
   P_Clk =  GetDomainClock(CLK_DOMAIN_012);       //Supplied OS Timer (OSTM0) Clock48,000,000, should be  CLK_DOMAIN_032, not 12

   // stop timer and clear the bit for initialization
   OSTM0TT = 0x01;

   // Compare value in free-run compare mode, Start value of the down-counter in Interval timer mode.
   OSTM0CMP = (UINT32)(P_Clk/1000.0) -1;

   // Interval timermode and Enable interrupt at counter start
   OSTM0CTL = 0x01;

   // Interrupt Enable ?????
   ICOSTM0 = INIT_INTOSTM0;

   // start timer
   OSTM0TS = 0x01;

   // ????
   for (i = 0; i < TIM_MAX_MRK; i++)
   {
      timSetMarker(i);
   }

} // end of function


//****************************************************************************
//* Function     :  dlyoff                                                   *
//****************************************************************************
//* Description  :  Schaltet die Delays aus                                  *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

void dlyoff(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // stop timer n
   OSTM0TT = 0x01;

} // end of function


//****************************************************************************
//* Function     :  INTTM0EQ0                                                *
//****************************************************************************
//* Description  :  Interrupt Service Routine für die Delays (Timer0)        *
//*                 Aufruf bei Timer0-Interrupt                              *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************
volatile UINT8  PWM_Duty;
#pragma ghs interrupt
void INTOSTM0(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // serve external Watchdog (optional)
   SERVE_WATCHDOG;

   // optional Userfunktion
   #ifdef USERFUNCTION_TIMER
   USERFUNCTION_TIMER;
   #endif

   //if (PWM_Duty & (0x01 << (GlobalTime & 0x07))) {SETPORT_1(1, 11);}
   //else                                          {SETPORT_0(1, 11);}

   ++GlobalTime;
} // end of function


//****************************************************************************
//* Function     :  timWait                                                  *
//****************************************************************************
//* Description  :  wartet die übergebene Anzahl in Millisekunden            *
//*                                                                          *
//*                                                                          *
//* Parameter    :  x 	: Dauer des Delays in ms                             *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

void timWait(UINT32 x)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // x ms warten
   timSetMarker(TIMER_WAIT_MS);
   while(!timTimeout(TIMER_WAIT_MS,x))
   {}

} // end of function


//-------------------------------------------------------------------------
//  Function     : timSetMarker
//-------------------------------------------------------------------------
//  Description  : Sets a time marker.
//
//  Parameter    : marker     - number of marker to be used
//                              (0...TIM_MAX_MRK - 1)
//
//  Returnvalue  : true, if successful, false, if marker no. out of range
//
//  Changed Var. : -none-
//
//  Comment      :
//-------------------------------------------------------------------------

BOOL timSetMarker(UINT8 marker)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // test for correct marker range
   if(marker >= TIM_MAX_MRK)
      return(FALSE);

   // copy time meter into marker
   timMrk[marker] = GlobalTime;

   // Enable Timer
   timMrkEnable[marker] = TRUE;

   return(TRUE);
}


//-------------------------------------------------------------------------
//  Function     : timStopTimer
//-------------------------------------------------------------------------
//  Description  : Sets a time marker invalid
//
//  Parameter    : marker     - number of marker to be used
//                              (0...TIM_MAX_MRK - 1)
//
//  Returnvalue  : -none-
//
//  Changed Var. : -none-
//
//  Comment      :
//-------------------------------------------------------------------------

BOOL timStopTimer (UINT8 marker)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // test for correct marker range
   if(marker >= TIM_MAX_MRK)
      return(FALSE);

   // Disable Timer
   timMrkEnable[marker] = FALSE;

   return(TRUE);
}


//-------------------------------------------------------------------------
//  Function     : timTimeout
//-------------------------------------------------------------------------
//  Description  : Checks for timeout.
//
//  Parameter    : marker     - number of marker to be used
//                              (0...TIM_MAX_MRK - 1)
//                 timeout    - timeout in ms
//
//  Returnvalue  : true, if timeout occurred, false if not
//
//  Changed Var. : -none-
//
//  Comment      :
//-------------------------------------------------------------------------
//  Quality      : ( ) not tested  ( ) partly tested  (x) fully tested
//-------------------------------------------------------------------------

BOOL timTimeout(UINT8 marker, UINT32 timeout)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // test for correct marker range
   if(marker >= TIM_MAX_MRK)
   {
      return(FALSE);
   }

   // test for timer enabled
   if(!timMrkEnable[marker])
   {
      return(FALSE);
   }

   if((GlobalTime - timMrk[marker]) >= timeout)
   {
      return(TRUE);
   }
   else
   {
      return(FALSE);
   }
}


//****************************************************************************
//* Function     :  get_ms                                                   *
//****************************************************************************
//* Description  :  Gibt die aktuelle Zeit in ms seit Systemstart aus
//*
//*
//* Parameter    :  x 	: Dauer des Delays in ms
//*
//* Returnvalue  :
//*
//* Changed Var. :
//*
//* Comment      :
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

UINT32   get_ms (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   return(GlobalTime);
}


UINT32   timGetTime (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   return(GlobalTime);
}


UINT32 timDifference(UINT32 time)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   return(GlobalTime - time);
}


#if (CP_DEBUG == 17)
#include "global.h"
//*--------------------------------------------------------------------------*
//* Available Debuf Commands                                                 *
//*                                                                          *
//*--------------------------------------------------------------------------*
void Debug_Info (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT32 Error;
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

  // Fallunterscheidung für verschiedene Testbetriebsarten
  Error = 0;
  switch ( commandPtr[2])

    {

         case 00:       test = readWord (commandPtr+3);
                        dlyini();
                        P5 = 0x00;
                        break;

         case 0x01 :    dlyoff();
                        break;

    } // end of case

   // Error auswerten und rückmelden
   writeLong(resultPtr+4, Error);

} // end of Debug_Info
#endif // #if CP_DEBUG

