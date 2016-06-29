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
#include "global.h"           // globale data structure

#include "tool.h"
#include "adc.h"
#include "gateway.h"
#include <string.h>           // for memset
#include "timer.h"
#include "ints.h"
#include "interprt.h"


//*--------------------------------------------------------------------------*
//*	Master switch for the whole file from here                               *
//*--------------------------------------------------------------------------*

#if CP_ADC

//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

UINT16 Get_virtual_ADC  (UINT8 *CommandBuf);
UINT16 adcConvert       (UINT8 Kanal);

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

#define MAX_COMMANDLENGHT 20        // Bytes per command
#define MAX_COMMANDNUMBER 16        // Number of Commands

#if (CP_VIRTUELL_ADC)
UINT8  VirtuellCommandArray  [MAX_COMMANDNUMBER][MAX_COMMANDLENGHT];
#endif

static UINT32 Error;

volatile BOOL ConversionReady;

//****************************************************************************
//* Function     : ADC_Init                                                  *
//****************************************************************************
//* Description  : Only for ADCA0                                            *
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

void ADC_Init(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   P_Clk;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // eingestellten Clock erfragen
   P_Clk = GetDomainClock (CLK_DOMAIN_012);   //Return 48,000,000

   if (P_Clk != 40000000)
      printf ("   ADC-Error : Clock Domain 012 not set to 40 Mhz\n");

   // startup time after power on 255 * PCLK
   ADCA0CNT = 0xFF;

   // Switch the power on
   ADCA0CTL1 |= ADCAnGPS;

   // Disable ADC
   ADCA0CTL0 &= ~ADCAnCE;

   // 1 repetition a time
   ADCA0CTL0 = 0x0000;

   // Set format : right-aligned, software trigger, disables the discharge, results overwritten,
   // 12-bit resolution, 5 Mhz (PCLK / 8) clock, buffer amplifier disable, one-shot conversion mode
   ADCA0CTL1 &= ~(ADCAnCRAC | ADCAnMD1 | ADCAnMD0 | ADCAnDISC | ADCAnRCL | ADCAnCTYP | ADCAnBPC | ADCAnTiETS(0));
   //Start Conversion respectively for CG0,CG1,CG2, Totally 77 clock conversion time
   ADCA0CTL1 |= ADCAnTRM0 | ADCAnTRM1 | ADCAnTRM2 | ADCAnFR(6);

   // Clear A/D converter channel group register
   ADCA0CG0 = ADCA0CG1 = ADCA0CG2 = 0;

   // Clear A/D converter interrupt control register
   ADCA0IOC0 = ADCA0IOC1 = ADCA0IOC2 = 0;

   // Disable A/D converter channel S&H control
   // ADCA0SHCTL = 0;                                       // IHE                                       ganzes File muss noch angepasst werden

   // Disable hardwaretrigger
   ADCA0TSEL0 = ADCA0TSEL1 = ADCA0TSEL2 = 0;

   // Disable A/D converter result check
   ADCA0CTL2 = 0;

   // Enable ADC
   ADCA0CTL0 |= ADCAnCE;

   // Set and enable Interrupt
   ICADCA0LLT = INIT_ADCA0LLT;

#if (CP_VIRTUELL_ADC)
   //Set VirtuellCommandArray value to 0x00
   memset (VirtuellCommandArray, 0x00, sizeof (VirtuellCommandArray));
#endif

} // end of function


//****************************************************************************
//* Function     : ADC_RD                                                    *
//****************************************************************************
//* Description  : Here, the ADC starts , converted values thru RS232        *
//*                then sent to the PC                                       *
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

void ADC_RD(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT16  AnalogValue;
UINT16  i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Rough test on command syntax,Read A/D Channel
   if (((commandPtr[0] == 4) && (commandPtr[3] == 0x00)) ||
       ((commandPtr[0] == 4) && (commandPtr[3] == 0x01))
     #if (CP_VIRTUELL_ADC)
       // virtual Command Check
       || ((commandPtr[2] > MAX_ADC_CHANNEL) && (commandPtr[3] == 0x80))
     #endif
     )
   {
      //Do nothing, Means 3 Mode
      //1. Read AD Channel 1 conversion
      //2. Read AD Channel 1 average
      //3. Read Vitrual AD Channel 
   }
   else
   {  // Return interpreter Command error by default
      return;
   }

//*--------------------------------------------------------------------------*
   // Standard conversion
   if (commandPtr[2] <= MAX_ADC_CHANNEL)   //23 = 17Hex
   {
      switch (commandPtr[3])
      {  case 0 :    AnalogValue = adcConvert (commandPtr[2]);
                     if (Error == NO_ERROR)
                     {
                        resultPtr[0] = 10;
                        writeWord(resultPtr+8, AnalogValue);
                     }
                     break;

         case 1:     AnalogValue = 0;
                     for ( i=0; i< ADC_AV_CONV_NUMBER; i++)
                        AnalogValue += adcConvert (commandPtr[2]);

                     AnalogValue = AnalogValue / ADC_AV_CONV_NUMBER;
                     if (Error == NO_ERROR)
                     {
                        resultPtr[0] = 10;
                        writeWord(resultPtr+8, AnalogValue);
                     }
                     break;

      }  // switch
      // Output Result
      writeLong (resultPtr+4, Error);
   }
   else
//*--------------------------------------------------------------------------*
   // valid virtual AD channel
   #if (CP_VIRTUELL_ADC)
      if (commandPtr[2] <= (MAX_ADC_CHANNEL + MAX_COMMANDNUMBER))
      {
         switch (commandPtr[3])
         {  case 0   :
            case 1   :  AnalogValue = Get_virtual_ADC (&VirtuellCommandArray [commandPtr[2]-MAX_ADC_CHANNEL-1][0]);
                        // Output Result
                        if (Error == NO_ERROR)
                           resultPtr[0] = 10;
                        writeLong (resultPtr+4, Error);
                        writeWord(resultPtr+8, AnalogValue);
                        break;

            case 0x80:  // test length whether reinpasst the command
                        if ((commandPtr[0] <= MAX_COMMANDLENGHT) &&
                           (commandPtr[0] >= 7))
                           // Insert command complete with length byte
                           memcpy (&VirtuellCommandArray[commandPtr[2]-MAX_ADC_CHANNEL-1][0],
                                    commandPtr,MAX_COMMANDLENGHT);
                        else
                           Error = ERROR_ADC | ERROR_VIRTUELL_CHANNEL_INVALID_CONFIG;
                        writeLong (resultPtr+4, Error);
                        break;
         }  // switch
      }
      else
   #endif
//*--------------------------------------------------------------------------*
      // otherwise always wrong channel
      writeLong (resultPtr+4, ERROR_ADC | ERROR_WRONG_CHANNEL);

} // end of function


//****************************************************************************
//* Function     :  INTADCA0LLT                                              *
//****************************************************************************
//* Description :   Interrupt Service Routine für Conversion Ready           *
//*                                                                          *                                                                          *
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

#pragma ghs interrupt
void INTADCA0LLT (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   ConversionReady = TRUE;
}


//****************************************************************************
//* Function     : adcConvert                                                *
//****************************************************************************
//* Description  : Hier wird der ADC gestartet, gewandelte Werte werden per  *
//*                Rueckgabeparameter an das aufrufende Modul zurueckge-     *
//*                geben.                                                    *
//* Parameter    : Kanal-Nr.                                                 *
//*                                                                          *
//* Returnvalue  : gewandelter ADC-Wert                                      *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

UINT16 adcConvert(UINT8 Kanal)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT16   AnalogValue;
UINT32   Result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Channel greater than max value
   if  (Kanal > MAX_ADC_CHANNEL)  //23 = 17Hex
   {
      Error = ERROR_ADC | ERROR_WRONG_CHANNEL;
      return(0);
   }

   // Specifies analog channel to be convert
   ADCA0CG0 = (1 << Kanal) ;

   // Start the conversion
   ConversionReady = FALSE;
   ADCA0TRG0 = ADCAnSTTi;

   // wait for result
   while (!ConversionReady)
   {}

   // collect 12-bit result and engaging to Right Right
   Result = ADCA0LCR;

   // Test whether channel number really about matches, Above Part will dispaly Channel Number, Last part is the current channel number
   // Pls refert to P2562
   if ((Result & ADCAnLCN) != (Kanal << 16))
   {
      Error = ERROR_ADC | ERROR_ADC_TIMEOUT;
      return(0);
   }

   // extract result
   AnalogValue = (UINT16)(Result & 0x0000FFFF);

   // Reture the reading value
   return (AnalogValue);
} // end of function


//****************************************************************************
//* Function     :   Get_virtual_ADC                                         *
//****************************************************************************
//* Description  :   zuvor gespeichertes Kommando an den Interpreter schicken*
//*                  und ein bestimmtes Byte oder wort aus der Antwort als   *
//*                  AD-Wert in die virtuelle Kanalabfrage  einsetzen        *                                                *
//*                                                                          *
//* Parameter    :   Buffer                                                  *
//*                                                                          *
//* Returnvalue  :   AD-Wert                                                 *
//*                                                                          *
//* Changed Var. :   Error                                                   *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

UINT16 Get_virtual_ADC (UINT8 *CommandBuf)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

   UINT8 Prefix[4];
   UINT16  Wert;

#ifdef test
   UINT16 i;
#endif

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Test whether any of this channel configured ( length byte ! = 0 )
   if (*CommandBuf == 0x00)
   {
      Error = ERROR_ADC | ERROR_VIRTUELL_CHANNEL_UNCONFIGED;
      return (0);
   }

   // cancel first 4 bytes from the original command
   memcpy (Prefix, resultPtr, 4);

   // Insert Virtual command in Command Buffer
   memcpy (commandPtr+1, CommandBuf+6, *CommandBuf -6);

   // Length byte to calculate and use
   commandPtr[0] = *CommandBuf -5;

   #ifdef test
   printf("----------   inserted command\n");
   printf("Length byte: %2.2x\n", commandPtr[0]);
   printf("Command   : %c\n",    commandPtr[1]);
   for (i=2; i< commandPtr[0]; i++)
      printf("Data %d     : %2.2x\n",i, commandPtr[i]);
   #endif

   // prevent recursive calls to this function , possibly are crashes
   if (intern_cmd)
   {
      Error = ERROR_ADC | ERROR_VIRTUELL_CHANNEL_BUF_OVERFLOW;
      return (0);
   }

   // suppress output of the response of the virtual commands
   // by switching to internal command processing
   setMode(MODE_IC, 1);

   // Command for processing notification ; there is no feedback to the PC
   interpreter();

   // Open commands again outwards
   setMode(MODE_IC, 0);

   #ifdef test
   printf("----------   Answer on starting command\n");
   printf("Length byte : %2.2x\n", resultPtr[0]);
   printf("Command   : %c\n",    resultPtr[1]);
   for (i=2; i< resultPtr[0]; i++)
      printf("Data %d     : %2.2x\n",i, resultPtr[i]);
   #endif

   // Insert first 4 bytes from the original command again
   memcpy (resultPtr, Prefix, 4);

   // pull out and check Error
   Error =  readLong (resultPtr+4);
   if (Error != NO_ERROR)
      return (0);

   // extract measured value from the response string
   switch (*(CommandBuf+4))
   {
      case 0x00 :    Wert = resultPtr [*(CommandBuf+5)];
                     break;

      case 0x01 :    Wert = readWord (resultPtr + (*(CommandBuf+5)));
                     break;

                     // 11 read bit leftadjusted and adjust
      case 0x02 :    Wert = readWord (resultPtr + (*(CommandBuf+5))) >>5;
                     break;

      case 0x20:
      case 0x21:
      case 0x22:
      case 0x23:
      case 0x24:
      case 0x25:
      case 0x26:
      case 0x27:     // Read 2 bytes and shift your order (low nibble D4 ) positions to the right
                     Wert = readWord (resultPtr + (*(CommandBuf+5))) >>(*(CommandBuf+4) & 0x0F);

                     // A / D component in the Bioscontrol can only D0..D9
                     Wert &= 0x3FF;
                     break;

      case 0x30:
      case 0x31:
      case 0x32:
      case 0x33:
      case 0x34:
      case 0x35:
      case 0x36:
      case 0x37:     // Read 2 bytes and shift your order (low nibble D4 ) positions to the right
                     Wert = ((((UINT16)resultPtr[CommandBuf[5]+1])<<8) + resultPtr[CommandBuf[5]]) >>(*(CommandBuf+4) & 0x0F);

                     // add offset
                     Wert += 0x200;

                     // A / D component in the Bioscontrol can only D0..D9
                     Wert &= 0x3FF;
                     break;

      case 0x40:     // for 16-bit numbers in two
                     Wert = readWord(resultPtr + (*(CommandBuf+5)));
                     //jprintf("Wert (unveraendert 16 Bit): 0x%X", Wert);
                     //                            if negative numerical value from 0 to 0x7FFF
                     if  (Wert & 0x8000)
                     {
                        Wert = (Wert & 0x7FFF);
                     }
                     else Wert = Wert + 0x8000; // if positive value = 0x8000 to 0xFFFF
                     //jprintf("Wert vor  Shift: 0x%X", Wert);
                     Wert = (Wert >> 6);  // verkuerze auf 10 Bit
                     //jprintf("Wert nach Shift: 0x%X", Wert);
                     break;

      default :      Error = ERROR_ADC | ERROR_VIRTUELL_CHANNEL_INVALID_CONFIG;
                     return(0);

   }

   return (Wert);                   // Return Reading Value
} // end of function


//****************************************************************************
//* Function     :	Debug_Info                                              *
//****************************************************************************
//* Description  :                                                           *
//* Available Debuf Commands                                                 *
//*                                                                          *
//* y 00 Kanal       - virtuellen Befehlsstring abfragen                     *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_DEBUG == 11)
void Debug_Info (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

  // Case analysis for different test modes
  switch ( commandPtr[2])
   {
      case 0x00:  ADC_Init();

                  writeLong (resultPtr+4, NO_ERROR);
                  break;
   } // end of case
} // end of Debug_Info
#endif // #if CP_DEBUG



#endif // #if CP_ADC
