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


//****************************************************************************
//* Modul        :  Main                                                     *
//****************************************************************************
//* Overview     : Main-Loop und Initialisierung der Inbetriebnahmesoftware  *
//*                                                                          *
//*                                                                          *
//*                                                                          *
//****************************************************************************


//*--------------------------------------------------------------------------*
//* Include files                                                            *
//*--------------------------------------------------------------------------*

#include "config.h"
#include "define.h"
#include "global.h"           // globale Datenstrukturen
#include "hbbios.h"
#include "gateway.h"
#include "hw_init.h"          // für die Hardware-Initialisierung
#include "tool.h"             // für Interpreter-Funktion
#include "timer.h"		  		// für Timerfunktionen
#include "interprt.h"

   #include "ints.h"
#if (CP_CAN0 == 1)               // CAN-Modul
      #include "can.h"           // für das CAN - Modul
   #endif
#if CP_CMDDEV_MOST
   #include "mostmsg.h"          // MOST-Modul
#endif
#if CP_CMDDEV_IIC
   #include "iicmsg.h"           // IIC-Modul
#endif
#if CP_CMDDEV_CAN
   #include "canmsg.h"           // CAN-Modul
#endif
#if CP_CMDDEV_SPI
   #include "spi.h"              // SPI-Modul
#endif
#if CP_CMDDEV_RS232
   #include "ser.h"
#endif

#include "stdlib.h"
//#include "gateway.h"
#if CP_INIC
   #include "inic.h"
#endif


#include "comm.h"


UINT8 Present_HW_Detection(void)
{
   static UINT8 DetectedBoard = 0;
   UINT8 SPIData[] = {0xdd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   static int i=0;
   
   if (DetectedBoard == 0)    // Only been called once
   {
      #if (CP_CSIG4)
      CsigOpen(0x0C, 0x0007A120, 0x21);   // 500k, auto-CS; CISG SPI 12-8 = 4
      timWait(5);
      //                   Channel   number
      SpiWrite_String_CSIG(0x0C, 8,     SPIData, SPIData);
      
      CsigClose(0x0C);

#if 0
      for(i=0;i<8;i++)
       {
         printf("Current SPIData[%d] value is 0x%X \n", i, SPIData[i] );
       }
#endif

      if (SPIData[4] == 'M')
      {
         if (SPIData[5] == '0')
         {
            if (SPIData[6] == '3')
            {
               if (SPIData[7] == '3')
               {
                  DetectedBoard = 0x33;
                  SETPORT_OUT( 0, 13); SETPORT_0( 0, 13);  // P0_13
                  printf("HW M033 detected\n");
               }
               else
               {
                  if (SPIData[7] == '4')
                  {
                     verStr[11] = '4'; // konfiguriert Startup-Ausgabe um, ersetzt '3' durch '4'
                     verStr[14] = '2'; // ersetzt B1   '1' durch '2' fuer B2 beim M034-B2
                     DetectedBoard = 0x34;
                     // Port-Konfigurations-Unterschiede der HW M034-B2
                     // verglichen mit HW-M033-B1
                     SETPORT_OUT( 3, 3); SETPORT_0( 3, 3);  // P3_3
                     SETPORT_OUT( 3, 4); SETPORT_0( 3, 4);  // P3_4
                     SETPORT_OUT(10, 4); SETPORT_0(10, 4);  // P10_4
                     SETPORT_OUT(11, 1); SETPORT_0(11, 1);  // P11_1
                     printf("HW M034 detected\n");
                  }
               }
            }
         }
      }
      #else
      DetectedBoard = 0x33;
      #endif
   }
   return(DetectedBoard);
}


//****************************************************************************
//* Function     : Main                                                      *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Main-Loop und Initialisierung der Inbetriebnahmesoftware                 *
//*--------------------------------------------------------------------------*
//* Function  main                                                           *
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
extern void PROGRAM_START(void);

void main (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   CPU_Init();                            // CPU Initialize

   Port_Init();                           // initialisiere Ports nach Portliste


   CPU_Running = 0x00;                    // CPU noch im Reset

   msgInit(255);                          // 254: use highest defined interface as default interface (255 = ifNone would broadcast)

//*--------------------------------------------------------------------------*
//* set gateway parameters and initialize communication                      *
//*--------------------------------------------------------------------------*

   // Start setting for all other applications
   Init_Gateway_Standard();

   Treiber_Init();                  // Initialize High Level Driver
                                    // Error messages possible already been printf

   Present_HW_Detection();          // HW Detection
   
   #if (SW_SCRIPT == 1)
   msgScript(AUTORUNSCRIPT, 0, 200);

   // run msgPoll till both autorun are executed; force broadcast to all known interfaces
   while (msgPoll(true))
   {};
   #endif

//*--------------------------------------------------------------------------*
//*	Initialisierung der Hardware Teil 2                                    *
//*--------------------------------------------------------------------------*
   // Begrüssungsmeldung
   printf("\n");
   printf("PLL0 Clock(fPL0): %dMHz\n", GetPLL_Clock(0));
   printf("ProgramStart : 0x%08X\n", (UINT32)&PROGRAM_START);

   printf("-------- BIOSControl V850 --------\n");
   printf("     ____   ____           ____   _    _  \n");
   printf("   /  ___| |  __ \\       /  ___| | |  | | \n");
   printf("   | \\___  | |  | |  __  | \\___  | |__| | \n");
   printf("   \\___  \\ | |  | | |__| \\___  \\ |  __  | \n");
   printf("    ___/ / | |__| |       ___/ / | |  | | \n");
   printf("   |____/  |_____/       |____/  |_|  |_| \n");
   printf(" \n");

   printf("   %s\n", verStr);
   printf("   Built on %s at %s.\n", verDate, verTime);
   printf("\n");

   if (RESF & 0x10)
      printf("   Resetsource : Watchdog\n");

   #if CP_STARTMELDUNG
      Statusabfrage();   // Controller Status ohne Aufforderung senden
   #endif


   msgBroadCast(resultPtr, true);   // stop startup logging

   // reset Text Output
   setMode(MODE_SA, 1);

   CPU_Running = 0x01;                 // CPU läuft jetzt

//*--------------------------------------------------------------------------*
//*  Mainloop                                                                *
//*--------------------------------------------------------------------------*

	while (1)
  	{
      #if CP_INIC
      // test and read out inic message
      inicProceedNewMsg();
      #endif
//*--------------------------------------------------------------------------*
//*	Standard-Gateway Modus eingeschaltet ?                                 *
//*--------------------------------------------------------------------------*

      msgPoll(false);

//*--------------------------------------------------------------------------*
//*	Mostring ueberwachen                                                   *
//*--------------------------------------------------------------------------*

#if INIC_SPI_CONFIF_MBC
      #if CP_CMDDEV_MOST
      // MOST Fehleranzeige
      MOST_TestRing ();
      #endif
#endif


//*--------------------------------------------------------------------------*

   } // end of while (1)

} // end of function
