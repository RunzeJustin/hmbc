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
                                    // ===============================================
#include "global.h"                 // globale Datenstrukturen
#include "port.h"
#include "tool.h"
#include "timer.h"

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if CP_PORT_RW


#if (CP_FPGA_INFO == 1)
//*----------------------------------------------------------------*
//* function p FA x5 for reading FPGA-Info of FPGA_x implemented   *
//* structure for fpga-info-database                               *
//*----------------------------------------------------------------*
typedef struct fpga_glob_info_s
{  UINT32            BaseAddr;
   UINT32            VerOffsset;
} fpga_glob_info_t;

static fpga_glob_info_t  fpga_info[FPGA_NO] =  def_FPGA_INFO;

static UINT8             i;
static UINT8             fpga_num;
static UINT8             rdata[16];

UINT32 fpga_ReadVersion(UINT8 fpga_num, UINT8* data);    // Versionsabfrage fuer FPGAs
#endif

//****************************************************************************
//* Function     :  Port_RD                                                  *
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

void Port_RD (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 Error,
       EdgeCounter,
       EdgeMeasuretime;

UINT16 EdgeMask,
       EdgeState;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Rough test on command syntax
   if (   ((commandPtr[0] == 4)  && (commandPtr[3] == 0x20))
       || ((commandPtr[0] == 10) && (commandPtr[3] == 0x40))
       || ((commandPtr[0] == 4)  && (commandPtr[3] == 0x80))
   #if (CP_FPGA_INFO == 1)
       || ((commandPtr[0] == 4)  && (commandPtr[2] == 0xFA))
   #endif
       )
   {}
   else
   {  // Send interpreter Command error by default
      return;
   }

   switch (commandPtr[3])
   {
//*--------------------------------------------------------------------------*
//* Read Port 16 Bit                                                         *
//*--------------------------------------------------------------------------*

      case 0x20 :
         // ever enter the length
         resultPtr[0] = 10;
         // differ by port number
         switch (commandPtr[2])
         {  // Port 0
            case 0x00:
               writeWord(resultPtr+8, (UINT16)PPR0);
               break;

            // Port 1
            case 0x01:
               writeWord(resultPtr+8, (UINT16)PPR1);
               break;
               
            // Port 2
            case 0x02:
               writeWord(resultPtr+8, (UINT16)PPR2);
               break;

            // Port 3
            case 0x03:
               writeWord(resultPtr+8, (UINT16)PPR3);
               break;

            // Port 4
            case 0x04:
               writeWord(resultPtr+8, (UINT16)PPR4);
               break;

            // Port 10
            case 0x10:
               writeWord(resultPtr+8, (UINT16)PPR10);
               break;

            // Port 11
            case 0x11:
               writeWord(resultPtr+8, (UINT16)PPR11);
               break;

            // Port 21
            case 0x21:
               writeWord(resultPtr+8, (UINT16)PPR21);
               break;

            // Port 25
            case 0x25:
               writeWord(resultPtr+8, (UINT16)PPR25);
               break;

            // Port 27
            case 0x27:
               writeWord(resultPtr+8, (UINT16)PPR27);
               break;

            // Port JP0
            case 0xA0:
               writeWord(resultPtr+8, (UINT16)JPPR0);
               break;

            default:
               resultPtr[0] = 8;
               Error = ERROR_PORT | ERROR_PORT_NUMBER;
               break;
         } // switch (commandPtr[2])

         break; // case 0x20

//*--------------------------------------------------------------------------*
//*       count port changes                                                 *
//*--------------------------------------------------------------------------*

      case 0x40 :
         // ever enter the length
         resultPtr[0] = 12;

         // Preparing numerator and mask
         EdgeCounter = 0;
         EdgeMask = readWord (commandPtr+4);
         EdgeMeasuretime = readLong (commandPtr+6);

         // set measurement time
         timSetMarker(TIMER_WAIT_SIGNAL);

         // differ by port number
         switch (commandPtr[2])
         {  // Port 0
            case 0x00:
               // starting state
               EdgeState = (UINT16)PPR0 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR0 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR0 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 1
            case 0x01:
               // starting state
               EdgeState = (UINT16)PPR1 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR1 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR1 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 2
            case 0x02:
               // starting state
               EdgeState = (UINT16)PPR2 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR2 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR2 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;

            // Port 3
            case 0x03:
               // starting state
               EdgeState = (UINT16)PPR3 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR3 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR3 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 4
            case 0x04:
               // starting state
               EdgeState = (UINT16)PPR4 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR4 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR4 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 10
            case 0x10:
               // starting state
               EdgeState = (UINT16)PPR10 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR10 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR10 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 11
            case 0x11:
               // starting state
               EdgeState = (UINT16)PPR11 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR11 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR11 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;

            // Port 21
            case 0x21:
               // starting state
               EdgeState = (UINT16)PPR21 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR21 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR21 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 25
            case 0x25:
               // starting state
               EdgeState = (UINT16)PPR25 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR25 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR25 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            // Port 27
            case 0x27:
               // starting state
               EdgeState = (UINT16)PPR27 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)PPR27 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)PPR27 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;

            // Port JP0
            case 0xA0:
               // starting state
               EdgeState = (UINT16)JPPR0 & EdgeMask;
               do
               {  // long measuring time runs
                  if (((UINT16)JPPR0 & EdgeMask) != EdgeState)
                  {  // count each change
                     EdgeState = (UINT16)JPPR0 & EdgeMask;
                     EdgeCounter++;
                  }
               }
               while (!timTimeout(TIMER_WAIT_SIGNAL, EdgeMeasuretime));
               break;


            default:
               resultPtr[0] = 8;
               Error = ERROR_PORT | ERROR_PORT_NUMBER;
               break;
         } // switch (commandPtr[2])

         // gezaehlte Impulse eintragen
         writeLong(resultPtr+8, EdgeCounter);
         break; // case 0x40

//*--------------------------------------------------------------------------*
//* Read Port Configuration                                                  *
//*--------------------------------------------------------------------------*

      // function ports are masked
      case 0x80:
         // ever enter the length
         resultPtr[0] = 20;
         // differ by port number
         switch (commandPtr[2])
         {
            // Port 0
            case 0x00:
               writeWord(resultPtr+8 , PORT0IN & ~(UINT16)PMC0);
               writeWord(resultPtr+10, PORT0PP & ~(UINT16)PMC0);
               writeWord(resultPtr+12, PORT0OD & ~(UINT16)PMC0);
               writeWord(resultPtr+14, (UINT16)PPR0);
               writeWord(resultPtr+16, (UINT16)~PM0 & PORT0PP);
               writeWord(resultPtr+18, (UINT16)PODC0 & (UINT16)(~PM0 & PORT0PP));
               break;


            // Port 1
            case 0x01:
               writeWord(resultPtr+8 , PORT1IN & ~(UINT16)PMC1);
               writeWord(resultPtr+10, PORT1PP & ~(UINT16)PMC1);
               writeWord(resultPtr+12, PORT1OD & ~(UINT16)PMC1);
               writeWord(resultPtr+14, (UINT16)PPR1);
               writeWord(resultPtr+16, (UINT16)~PM1 & PORT1PP);
               writeWord(resultPtr+18, (UINT16)PODC1 & (UINT16)(~PM1 & PORT1PP));
               break;

            // Port 2
            case 0x02:
               writeWord(resultPtr+8 , PORT2IN & ~(UINT16)PMC2);
               writeWord(resultPtr+10, PORT2PP & ~(UINT16)PMC2);
               writeWord(resultPtr+12, PORT2OD & ~(UINT16)PMC2);
               writeWord(resultPtr+14, (UINT16)PPR2);
               writeWord(resultPtr+16, (UINT16)~PM2 & PORT2PP);
               writeWord(resultPtr+18, (UINT16)PODC2 & (UINT16)(~PM2 & PORT2PP));
               break;

            // Port 3
            case 0x03:
               writeWord(resultPtr+8 , PORT3IN & ~(UINT16)PMC3);
               writeWord(resultPtr+10, PORT3PP & ~(UINT16)PMC3);
               writeWord(resultPtr+12, PORT3OD & ~(UINT16)PMC3);
               writeWord(resultPtr+14, (UINT16)PPR3);
               writeWord(resultPtr+16, (UINT16)~PM3 & PORT3PP);
               writeWord(resultPtr+18, (UINT16)PODC3 & (UINT16)(~PM3 & PORT3PP));
               break;


            // Port 4
            case 0x04:
               writeWord(resultPtr+8 , PORT4IN & ~(UINT16)PMC4);
               writeWord(resultPtr+10, PORT4PP & ~(UINT16)PMC4);
               writeWord(resultPtr+12, PORT4OD & ~(UINT16)PMC4);
               writeWord(resultPtr+14, (UINT16)PPR4);
               writeWord(resultPtr+16, (UINT16)~PM4 & PORT4PP);
               writeWord(resultPtr+18, (UINT16)PODC4 & (UINT16)(~PM4 & PORT4PP));
               break;


            // Port 10
            case 0x10:
               writeWord(resultPtr+8 , PORT10IN & ~(UINT16)PMC10);
               writeWord(resultPtr+10, PORT10PP & ~(UINT16)PMC10);
               writeWord(resultPtr+12, PORT10OD & ~(UINT16)PMC10);
               writeWord(resultPtr+14, (UINT16)PPR10);
               writeWord(resultPtr+16, (UINT16)~PM10 & PORT10PP);
               writeWord(resultPtr+18, (UINT16)PODC10 & (UINT16)(~PM10 & PORT10PP));
               break;


            // Port 11
            // kein PMC-Register
            case 0x11:
               writeWord(resultPtr+8 , PORT11IN);
               writeWord(resultPtr+10, PORT11PP);
               writeWord(resultPtr+12, PORT11OD);
               writeWord(resultPtr+14, (UINT16)PPR11);
               writeWord(resultPtr+16, (UINT16)~PM11 & PORT11PP);
               writeWord(resultPtr+18, (UINT16)PODC11 & (UINT16)(~PM11 & PORT11PP));
               break;


            // Port 21
            case 0x021:
               writeWord(resultPtr+8 , PORT21IN & ~(UINT16)PMC21);
               writeWord(resultPtr+10, PORT21PP & ~(UINT16)PMC21);
               writeWord(resultPtr+12, PORT21OD & ~(UINT16)PMC21);
               writeWord(resultPtr+14, (UINT16)PPR21);
               writeWord(resultPtr+16, (UINT16)~PM21 & PORT21PP);
               writeWord(resultPtr+18, (UINT16)PODC21 & (UINT16)(~PM21 & PORT21PP));
               break;


            // Port 25
            case 0x025:
               writeWord(resultPtr+8 , PORT25IN & ~(UINT16)PMC25);
               writeWord(resultPtr+10, PORT25PP & ~(UINT16)PMC25);
               writeWord(resultPtr+12, PORT25OD & ~(UINT16)PMC25);
               writeWord(resultPtr+14, (UINT16)PPR25);
               writeWord(resultPtr+16, (UINT16)~PM25 & PORT25PP);
               writeWord(resultPtr+18, (UINT16)PODC25 & (UINT16)(~PM25 & PORT25PP));
               break;


            // Port 27
            case 0x027:
               writeWord(resultPtr+8 , PORT27IN & ~(UINT16)PMC27);
               writeWord(resultPtr+10, PORT27PP & ~(UINT16)PMC27);
               writeWord(resultPtr+12, PORT27OD & ~(UINT16)PMC27);
               writeWord(resultPtr+14, (UINT16)PPR27);
               writeWord(resultPtr+16, (UINT16)~PM27 & PORT27PP);
               writeWord(resultPtr+18, (UINT16)PODC27 & (UINT16)(~PM27 & PORT27PP));
               break;

            // Port JP0
            case 0xA0:
               writeWord(resultPtr+8 , PORTJ0IN & ~(UINT16)JPMC0);
               writeWord(resultPtr+10, PORTJ0PP & ~(UINT16)JPMC0);
               writeWord(resultPtr+12, PORTJ0OD & ~(UINT16)JPMC0);
               writeWord(resultPtr+14, (UINT16)JPPR0);
               writeWord(resultPtr+16, (UINT16)~JPM0 & PORTJ0PP);
               writeWord(resultPtr+18, (UINT16)JPODC0 & (UINT16)(~JPM0 & PORTJ0PP));
               break;


            default:
               resultPtr[0] = 8;
               Error = ERROR_PORT | ERROR_PORT_NUMBER;
               break;
         } // switch (commandPtr[2])
         break; // case 0x80

      default:
         #if (CP_FPGA_INFO == 1)
         // *----------------------------------------------------*
         // * FPGA_VERSION auslesen
         // *----------------------------------------------------*
         if (commandPtr[2] == 0xFA)
         {
            // test for valid command
            if ((commandPtr[3] & 0x0F) != 0x05)
            {
               // invalid command
               resultPtr[0] = 8;
               Error = ERROR_PORT | ERROR_PORT_NUMBER;
            }
            else
            {
               // test for valid FPGA_NO
               fpga_num = (commandPtr[3] & 0xF0) >> 4;
               if  (fpga_num >= FPGA_NO)
               {
                  // invalid FPGA_NO
                  resultPtr[0] = 8;
                  Error = ERROR_PORT | ERROR_PORT_NUMBER;
               }
               else
               {
                   Error = fpga_ReadVersion(fpga_num, rdata);

                   printf("** FPGA Version Info:\n");
                   printf("     FPGA Number:   %d\n", (UINT32)fpga_num);
                   printf("     Register Addr: %08X\n",(UINT32)(fpga_info[fpga_num].BaseAddr + fpga_info[fpga_num].VerOffsset));
                   printf("     Project:       %02X%02X\n", (UINT32)rdata[0], (UINT32)rdata[1]);
                   printf("     HW-Revision:   %02X\n", (UINT32)rdata[2]);
                   printf("     FPGA-Revision: %02X\n", (UINT32)rdata[4]);
                   switch(rdata[3])
                   {
                      case 0:  printf("     Typ:           TPORT\n");
                               break;
                      case 1:  printf("     Typ:           STANDARD\n");
                               break;
                      default: printf("     Typ:           UnKnown\n");
                               break;
                   }
                   printf("     RevisionCode:  %02X%02X%02X\n", (UINT32)rdata[8], (UINT32)rdata[9], (UINT32)rdata[10]);
                   printf("     Checksum:      %02X%02X%02X%02X\n", (UINT32)rdata[12], (UINT32)rdata[13], (UINT32)rdata[14], (UINT32)rdata[15]);

                   for(i=0; i<16; i++)
                      resultPtr[8+i] = rdata[i];
                   resultPtr[0]=8+16;
               }
            }
         }
         break;
       #endif

   } // switch (commandPtr[3])

   // Ergebnis senden
   writeLong(resultPtr+4, Error);
} // end of function Port_RD


//****************************************************************************
//* Function     :   Set_Bits                                                *
//****************************************************************************
//* Description  :   maskierte Bits setzen oder loeschen                     *
//*                                                                          *
//* Parameter    :   Port, pMask, value                                      *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

UINT32  Set_Bits (UINT8 *pPort, UINT8 *pMask, BOOL value)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // nach Portnummer unterscheiden
   switch (*pPort)
   {
      // Port 0
      case 0x00:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT0PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC0)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR0 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 1
      case 0x01:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT1PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC1)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR1 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;

      // Port 2
      case 0x02:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT2PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC2)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR2 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;

      // Port 3
      case 0x03:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT3PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC3)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR3 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 4
      case 0x04:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT4PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC4)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR4 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 10
      case 0x10:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT10PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC10)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR10 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 11
      // kein PMC-Register
      case 0x11:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT11PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR11 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 21
      case 0x21:
         // test mask on validity
         if (readWord(pMask) & ~PORT21PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC21)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR21 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 25
      case 0x25:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT25PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC25)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR25 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      // Port 27
      case 0x27:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORT27PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)PMC27)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         PSR27 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;

      // Port JP0
      case 0xA0:
         // Maske auf Gültigkeit testen
         if (readWord(pMask) & ~PORTJ0PP)
         {  Error = ERROR_PORT | ERROR_PORT_MASKE;
            break;
         }
         // Port vielleicht als Alternate Function belegt ?
         if (readWord(pMask) & (UINT16)JPMC0)
         {  Error = ERROR_PORT | ERROR_PORT_ALTERNATEFUNCTION;
            break;
         }

         // bitmaessiger Port Set Reset Zugriff
         JPSR0 = ((UINT32)readWord(pMask)<<16) + (value ? 0xFFFF : 0x0000);
         break;


      default:
         Error = ERROR_PORT | ERROR_PORT_NUMBER;
         break;
   } // switch (*pPort)

   return (Error);
} // end of function Set_Bits


//****************************************************************************
//* Function     :  Port_WR                                                  *
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

void Port_WR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Error;
UINT16   i;
UINT32   ODC_value;
INT32    Delaytime;
UINT8*   pWork;

#define LAUFZEITKORREKTUR   10     // [us] Rechenzeit fürs Portsetzen beim Gruppenbefehl
                                   // wird abgezogen
                                   // Wert aus Messung  120us bei t>0    @5Mhz

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Grobtest auf Kommandosyntax
   if (((commandPtr[0] == 8) && (commandPtr[3] == 0x80)) ||
       ((commandPtr[0] == 8) && (commandPtr[3] == 0x81)) ||
       ((commandPtr[0] == 8) && (commandPtr[3] == 0x82)) ||
       ((commandPtr[0] == 6) && (commandPtr[3] == 0x10)) ||
       ((commandPtr[0] == 6) && (commandPtr[3] == 0x20)) ||
       // Grober Test für den Portgruppenbefehl
       ((commandPtr[2] == 0xE0)
       && (commandPtr[0] <= 100)
       && (commandPtr[0] >= 10)
       && (((commandPtr[0]-4)%6) == 0)
       && (commandPtr[3] >= 1)
       && (commandPtr[3] <= 16)))
   {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

//*--------------------------------------------------------------------------*

   // Testen, ob Portgruppenbefehl vorliegt
   if (commandPtr[2] == 0xE0)
   {
      // Arbeitszeiger setzen
      pWork = commandPtr + 4;

      for (i=0; i< commandPtr[3]; i++)
      {  // 1 Gruppe bearbeiten

         #ifdef PORT_DEBUG
         printf ("Nr.  %d Portnummer %2.2x Befehl %2.2x Maske %4.4x Zeit %4.4x\n", i,
         (UINT8)*(pWork),
         (UINT8)*(pWork+1),
         readWord(pWork+2),
         readWord(pWork+4));
         printf ("Delaycount : %8.8X\n",(UINT32)(Delaytime*(FRQ/1e6)*0.987));
         #endif

         // Befehl untersuchen
         switch (*(pWork+1))
         {  // maskierte Bits löschen
            case 0x10 :
               Error = Set_Bits (pWork, pWork+2, FALSE);
               break; // case 0x10

            // maskierte Bits setzen
            case 0x20 :
               Error = Set_Bits (pWork, pWork+2, TRUE);
               break; // case 0x20

            default :
               Error = (ERROR_INTERPRT | ERROR_COMMAND);
               break;
         }

         if (Error != NO_ERROR)
            break;

         // reine CPU-Laufzeitschleife fuer bis zu 65000 us
         Delaytime = (signed long)readWord(pWork+4) - LAUFZEITKORREKTUR;

         if (Delaytime > 0)
            DelayLoop (us(Delaytime));

         // nächster Befehl
         pWork += 6;
      } // of for
   } // of if
//*--------------------------------------------------------------------------*
   // only if no port group command
   else
   {
      switch (commandPtr[3])
      {
         // Port 16 Bit Portkonfiguration schreiben
         case 0x80 : // direkt schreiben
         case 0x81 : // Maske fuer Bits loeschen
         case 0x82 : // Maske fuer Bits setzen
            // nach Portnummer unterscheiden
            switch (commandPtr[2])
            {
               // Port 0
               case 0x00:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT0PP) ||
                      (readWord(commandPtr+6) & ~PORT0OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM0 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM0 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM0 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC0 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC0 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD0=0xA5;
                  PODC0 = ODC_value;
                  PODC0 = ~ODC_value;
                  PODC0 = ODC_value;
                  break;


               // Port 1
               case 0x01:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT1PP) ||
                      (readWord(commandPtr+6) & ~PORT1OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM1 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM1 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM1 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC1 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC1 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD1=0xA5;
                  PODC1 = ODC_value;
                  PODC1 = ~ODC_value;
                  PODC1 = ODC_value;
                  break;

               // Port 2
               case 0x02:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT2PP) ||
                      (readWord(commandPtr+6) & ~PORT2OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM2 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM2 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM2 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC2 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC2 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD2=0xA5;
                  PODC2 = ODC_value;
                  PODC2 = ~ODC_value;
                  PODC2 = ODC_value;
                  break;

               // Port 3
               case 0x03:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT3PP) ||
                      (readWord(commandPtr+6) & ~PORT3OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM3 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM3 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM3 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC3 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC3 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD3=0xA5;
                  PODC3 = ODC_value;
                  PODC3 = ~ODC_value;
                  PODC3 = ODC_value;
                  break;


               // Port 4
               case 0x04:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT4PP) ||
                      (readWord(commandPtr+6) & ~PORT4OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM4 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM4 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM4 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC4 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC4 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD4=0xA5;
                  PODC4 = ODC_value;
                  PODC4 = ~ODC_value;
                  PODC4 = ODC_value;
                  break;


               // Port 10
               case 0x10:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT10PP) ||
                      (readWord(commandPtr+6) & ~PORT10OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM10 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM10 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM10 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC10 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC10 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD10=0xA5;
                  PODC10 = ODC_value;
                  PODC10 = ~ODC_value;
                  PODC10 = ODC_value;
                  break;


               // Port 11
               case 0x11:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT11PP) ||
                      (readWord(commandPtr+6) & ~PORT11OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM11 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM11 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM11 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC11 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC11 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD11=0xA5;
                  PODC11 = ODC_value;
                  PODC11 = ~ODC_value;
                  PODC11 = ODC_value;
                  break;



               // Port 21
               case 0x21:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT21PP) ||
                      (readWord(commandPtr+6) & ~PORT21OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM21 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM21 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM21 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC21 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC21 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD21=0xA5;
                  PODC21 = ODC_value;
                  PODC21 = ~ODC_value;
                  PODC21 = ODC_value;
                  break;


               // Port 25
               case 0x25:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT25PP) ||
                      (readWord(commandPtr+6) & ~PORT25OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM25 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM25 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM25 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC25 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC25 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD25=0xA5;
                  PODC25 = ODC_value;
                  PODC25 = ~ODC_value;
                  PODC25 = ODC_value;
                  break;


               // Port 27
               case 0x27:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORT27PP) ||
                      (readWord(commandPtr+6) & ~PORT27OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     PM27 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     PM27 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     PM27 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = PODC27 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = PODC27 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  PPCMD27=0xA5;
                  PODC27 = ODC_value;
                  PODC27 = ~ODC_value;
                  PODC27 = ODC_value;
                  break;

               // Port JP0
               case 0xA0:
                  // Masken auf Gültigkeit testen
                  if ((readWord(commandPtr+4) & ~PORTJ0PP) ||
                      (readWord(commandPtr+6) & ~PORTJ0OD))
                  {  Error = ERROR_PORT | ERROR_PORT_MASKE;
                     break;
                  }

                  if (commandPtr[3] == 0x81)
                     JPM0 |= readWord(commandPtr+4);
                  if (commandPtr[3] == 0x82)
                     JPM0 &= ~readWord(commandPtr+4);
                  if (commandPtr[3] == 0x80)
                     JPM0 = ~readWord(commandPtr+4);

                  if (commandPtr[3] == 0x81)
                     ODC_value = JPODC0 & ~(UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x82)
                     ODC_value = JPODC0 | (UINT32)readWord(commandPtr+6);
                  if (commandPtr[3] == 0x80)
                     ODC_value = (UINT32)readWord(commandPtr+6);

                  // protected write
                  JPPCMD0=0xA5;
                  JPODC0 = ODC_value;
                  JPODC0 = ~ODC_value;
                  JPODC0 = ODC_value;
                  break;


               default:
                  Error = ERROR_PORT | ERROR_PORT_NUMBER;
                  break;

            } // of switch (commandPtr[2])
            break; // case 0x80; 0x81; 0x82

         // maskierte Bits löschen
         case 0x10 :
            Error = Set_Bits (commandPtr+2, commandPtr+4, FALSE);
            break; // case 0x10

         // maskierte Bits setzen
         case 0x20 :
            Error = Set_Bits (commandPtr+2, commandPtr+4, TRUE);
            break; // case 0x20

      } // switch (commandPtr[3])
   } // of else (commandPtr[2] == 0xE0)


   // Ergebnis senden
   writeLong(resultPtr+4, Error);
} // end of function  Port_WR


// special portCommands with FPGA available


#if (CP_FPGA_INFO == 1)

//-------------------------------------------------------------------------
// fpga_ReadVersion                                     G. Kern, 2001-03-02
//-------------------------------------------------------------------------
// Function
//           read Version from FPGA and return result in buffer
//-------------------------------------------------------------------------
UINT32 fpga_ReadVersion(UINT8 fpga_num, UINT8* data)
{
UINT8 i;
volatile UINT8 *versadr;

   UINT32 error = NO_ERROR;
   versadr = (UINT8 *)(fpga_info[fpga_num].BaseAddr + fpga_info[fpga_num].VerOffsset) ;

   //   if(debug_msg_on) printf("     Read Version:");
   for(i=0; i<16; i++)
   {
      *versadr = i;
      data[i] = *versadr;
   }
   /*
   if(debug_msg_on)
   {
      printf("       Addr: %08X\n", (UINT32) versadr);
      printf("       data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
      printf("             %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
   }
   */
   return(error);
}

#endif


#endif // #if CP_PORT_RW

