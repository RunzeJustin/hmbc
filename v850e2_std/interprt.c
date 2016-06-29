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
#include "global.h"           // globale Datenstrukturen
#include "ints.h"

#if CP_ADC                       // ADC Modul
   #include "adc.h"              // für das ADC - Modul
#endif
#if (CP_CAN0 || CP_CAN1 || CP_CAN2 || CP_CAN3)         // CAN-Modul
   #include "can.h"           // für das CAN - Modul
#endif
   #include "timer.h"
#if (CP_RDS1||CP_RDS2||CP_RDS3)  // RDS-Daten Kanal 1 oder 2
   #include "rds.h"				   // RDS-Kanaele
#endif
#if CP_OS8104_PAR                // Most parallel einschalten
   #include "os8104.h"
#endif
#if ((CP_CMDDEV_MOST || CP_CMDDEV_IIC || CP_CMDDEV_CAN || CP_CMDDEV_RS232) != 0)            // Interpreter für Gateway-Befehle "@..."  einschalten
   #include "gateway.h"          // fuer die Gateway-Befehle
#endif
   #include "global.h"
#if CP_IIC5
   #include "iic_5.h"
#endif
#if CP_CMDDEV_IIC
   #include "iicmsg.h"
#endif
#if CP_CMDDEV_CAN
   #include "canmsg.h"
#endif
#if CP_CMDDEV_MOST
   #include "mostmsg.h"
#endif
#include "spi.h"
#if (SW_SCRIPT == 1)
   #include "script.h"
#endif
#if CP_INIC
   #include "inic.h"
#endif
#if CP_DSPBOOT
   #include "dspboot.h"
#endif
#if CP_ECL
   #include "ecl.h"
#endif
#include "tool.h"
#include "ser.h"

#include "comm.h"

//*--------------------------------------------------------------------------*
//* Type Definitions                                                         *
//*--------------------------------------------------------------------------*

typedef unsigned int TCallFct(unsigned char *Buffer);

//*--------------------------------------------------------------------------*
//* global variables                                                         *
//*--------------------------------------------------------------------------*

#if HOSTTIMEOUT
   BOOL          Interpreter_CommandReceived;
#endif

//****************************************************************************
//* Function     :  Wait                                                     *
//****************************************************************************
//* Description  :  unterbricht den Programmfluss für eine bestimmte Zeit.   *
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

void Wait (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Grobtest auf Kommandosyntax
   if (commandPtr[0] == 8)
      {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

   timWait (readLong (commandPtr+4));
   writeLong(resultPtr+4, NO_ERROR);
} // end of function


//****************************************************************************
//* Function     :  Jump_To_Addr                                             *
//****************************************************************************
//* Description  :  springt auf die mitgegebene Addresse zw. Start eines     *
//*                 nachgeladenen Programmteils.                             *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (x) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void Jump_To_Addr(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
   static TCallFct *Addr;
   static UINT32   Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Grobtest auf Kommandosyntax
   if ((commandPtr[0] >= 8) && !(commandPtr[7] & 0x01))
      {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

//*--------------------------------------------------------------------------*

   if (commandPtr[0] == 8)
   {
      // Startaddresse in D4 .. D7
      Addr     = (TCallFct*)(readLong(commandPtr+4));
      // ab D8 liegen die Parameter, die
      // dem aufgerufenen Programm uebergeben werden
      // falls das aufgerufene Programm eine
      // Rueckkehr vorsieht
      Error = (UINT32) Addr((UINT8*) commandPtr+8);
      writeLong(resultPtr+4, Error);
   }
   else

//*--------------------------------------------------------------------------*

   {
      if (commandPtr[0] == 9)
      {
         switch (commandPtr[8])
         {
            case 0x00 : // erst quittieren
                        writeLong(resultPtr+4, NO_ERROR);
                        // Watchdog nicht mehr bedienen und so Hardwarereset erzwingen
                        // clear global interrupt enable flag
                        __DI();
                        // warten auf Ausloesung des Watchdogs
                        while (1)
                        {  };

            case 0x01 : // erst quittieren
                        writeLong(resultPtr+4, NO_ERROR);

                        // clear global interrupt enable flag
                        __DI();
                        // Sprung nach Adresse 0
                        #pragma asm
                        jmp   [r0]
                        #pragma endasm
                        break;

            default:    // Interpreter Commandfehler defaultmässig schicken
                        break;
         }  // of switch
      }
   } // of else (commandPtr[0] == 8)

} // end of function


/* ****************************************************************************** */

#ifdef BoardVersion
void SendProjectAndBoardVersion(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
   char     CharArray[]  = BoardVersion;
   char     CharArray2[] = BoardVersion2;
   UINT8    cnt;
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   resultPtr [1] = CONTROLLERTYP;
   resultPtr [2] = 0x01;                   // Sub-Command
   resultPtr [3] = 0xFF;                   // Fill-byte
   writeLong(resultPtr+4, NO_ERROR);
   resultPtr [8] = (sizeof(CharArray) - 1);

   if (Present_HW_Detection() == 0x34)
   {
      for (cnt = 0; cnt < resultPtr[8]; cnt++)
         resultPtr[cnt+9] = CharArray2[cnt];
   }
   else
   {
      for (cnt = 0; cnt < resultPtr[8]; cnt++)
         resultPtr[cnt+9] = CharArray[cnt];
   }

   resultPtr [0] = resultPtr[8] + 9;     // Gesamtlänge der Antwort
}
#endif


//****************************************************************************
//* Function     :  SubInterpreter_Diagnostic                                *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "D"                    *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_Diagnostic (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
      case SUBCMD_ASCIIMESSAGE:     if (commandPtr[3] == 0x00)
                                    {  printf("** ASCII Output Off\n");
                                       setMode(MODE_SA, 0);
                                    }
                                    else
                                       if (commandPtr[3] == 0x01)
                                       {  printf("** ASCII Output On\n");
                                          setMode(MODE_SA, 1);
                                       }
                                    break;


      case SUBCMD_DEBUGMESSAGE:     if (commandPtr[3] == 0x00)
                                    {  printf("** DEBUG Output Off\n");
                                       setMode(MODE_SD, 0);
                                    }
                                    else
                                       if (commandPtr[3] == 0x01)
                                       {  printf("** DEBUG Output On\n");
                                          setMode(MODE_SD, 1);
                                       }
                                    break;


      default:                      break;
   } // of switch

   Statusabfrage();
} // end of function SubInterpreter_Diagnostic


//****************************************************************************
//* Function     :  SubInterpreter_Status                                    *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "s"                    *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_Status (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (commandPtr[0] == 2)
   {
      Statusabfrage();
      return;
   }

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
      case SUBCMD_ASCIIMESSAGE:
      case SUBCMD_DEBUGMESSAGE:     SubInterpreter_Diagnostic();
                                    return;

      #ifdef BoardVersion
      case SUBCMD_BOARD_VERSION:
      {
         SendProjectAndBoardVersion();
         return;
      }
      #endif

      default:                      break;
   } // of switch
} // end of function SubInterpreter_Status


//****************************************************************************
//* Function     :  SubInterpreter_xtd_Init                                  *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "H"                    *
//*                 Entsprechend dem im Instring_Zeichen - Array vorhandenen *
//*                 Unterbefehl werden unterschiedliche Funktionen aufge-    *
//*                 rufen.                                                   *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_xtd_Init(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
#if CP_OS8104_PAR
      case HWINIT_MOST:             writeLong(resultPtr+4, MOST_Reset ());
                                    resultPtr[3] = 0xDD;
                                    break;
#endif
#if CP_INIC
      case HWINIT_INIC:             writeLong(resultPtr+4, inicHwInit (INIC_ACCESS_I2C));
                                    break;
#endif
      default:                      break; // "Kommando unbekannt" melden
   } // of switch
} // end of function


//****************************************************************************
//* Function     :  SubInterpreter_xtd_Cmd                                   *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "X"                    *
//*                 Entsprechend dem im Instring_Zeichen - Array vorhandenen *
//*                 Unterbefehl werden unterschiedliche Funktionen aufge-    *
//*                 rufen.                                                   *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_xtd_CMD(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
      case SUBCMD_WARTEN:           Wait();
                                    break;
#if CP_SPARMODUS
      case SUBCMD_STROMSPARMODUS:   Pwr_Save();
                                    break;
#endif
#if CP_PWM
      case SUBCMD_PWM_ERZEUGUNG:    PWM();
                                    break;
#endif
      case SUBCMD_JUMPCALLADDR:     Jump_To_Addr();
                                    break;
#if CP_INIC
      case SUBCMD_INIC:             inicCommand();
                                    break;
#endif
#if CP_DSPBOOT
   case SUBCMD_DSPBOOT:             tida6xxCmd();
                                    break;
#endif
#if CP_ECL
      case CMD_X_ECL:               ECL_Interpreter();
                                    break;
#endif
      default:                      break; // "Kommando unbekannt" melden
   } // of switch
} // end of function


//****************************************************************************
//* Function     :  SubInterpreter_ext_Write                                 *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "Z"                    *
//*                 Entsprechend dem im Instring_Zeichen - Array vorhandenen *
//*                 Unterbefehl werden unterschiedliche Funktionen aufge-    *
//*                 rufen.                                                   *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_xtd_Write(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
#if (CP_CAN0 || CP_CAN1 || CP_CAN2 || CP_CAN3)
      case SUBCMD_CAN_SCHREIBEN:    CAN_WR ();
                                    break;
#endif
#if CP_CMDDEV_MOST
   #if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
      case SUBCMD_MOSTMSG_WRITE:    MOSTMSG_WR();
                                    break;
   #endif
#endif
      default:                      break; // "Kommando unbekannt" melden
   } // of switch
} // end of function


//****************************************************************************
//* Function     :  SubInterpreter_ext_Read                                  *
//****************************************************************************
//* Description  :  SubInterpreter fuer Befehlsgruppe "z"                    *
//*                 Entsprechend dem im Instring_Zeichen - Array vorhandenen *
//*                 Unterbefehl werden unterschiedliche Funktionen aufge-    *
//*                 rufen.                                                   *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

void SubInterpreter_xtd_Read(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (commandPtr[2])            // SubKommando-Byte auswerten
   {
#if (CP_CAN0 || CP_CAN1 || CP_CAN2 || CP_CAN3)
      case SUBCMD_CAN_LESEN:        CAN_RD();
                                    break;
#endif
#if CP_CMDDEV_MOST
   #if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
      case SUBCMD_MOSTMSG_READ:     MOSTMSG_RD();
                                    break;
   #endif
#endif
case SUBCMD_AUTORUNINFO_READ :      writeLong(resultPtr+4, NO_ERROR);
                                    writeLong(resultPtr+8, (UINT32)AUTORUNSCRIPT);
                                    resultPtr[0]  = 12;
                                    break;

      default:                      break; // "Kommando unbekannt" melden
   } // of switch
} // end of function


//****************************************************************************
//* Function     :  Seriel_CMD                                               *
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
//* Quality      :     (X) not tested  ( ) partly tested  ( ) fully tested   *
//****************************************************************************

#if (CP_SER11||CP_SPI0||CP_SPI1||CP_SPI2||CP_SPI3||CP_SPI4||CP_SPI5||CP_CSIE0||CP_CSIE1||CP_CSIG0||CP_CSIG4||CP_CSIG7||CP_CSIH0||CP_CSIH1||CP_CSIH2)
void Seriel_CMD(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (commandPtr[3] < SPI_CH_OFFSET)
   {
      #if (CP_SER11)
      SerielUart_CMD();
      #endif
      return;
   }

   if (commandPtr[3] < CSIE_CH_OFFSET)          // < 0x40 normale (alte) SPI wird verwendet
   {
      #if (CP_SPI0||CP_SPI1||CP_SPI2||CP_SPI3||CP_SPI4||CP_SPI5)
      SerielSpi_CMD();
      #endif
      return;
   }

   if (commandPtr[3] < CSIG_CH_OFFSET)          // < 0x50 erste SPI Erweiterung (CSIE) wird verwendet
   {
   #if (CP_CSIE0||CP_CSIE1)
   SerielCSIE_CMD();
   #endif
   }

   if (commandPtr[3] < CSIH_CH_OFFSET)          // < 60 zweite SPI Erweiterung CSIG wird verwendet
   {
      #if (CP_CSIG0 || CP_CSIG4 || CP_CISG7)
         SerielCSIG_CMD();
      #endif
      return;
   }

   //if (commandPtr[3] < CSIH_CH_OFFSET)          // >= 60 dritte SPI Erweiterung CSIH wird verwendet
   {
      #if (CP_CSIH0||CP_CSIH1||CP_CSIH2||CP_CSIH3)
      SerielCSIH_CMD();
      #endif
      return;
   }
} // end of function
#endif


//****************************************************************************
//* Function     :  Interpreter                                              *
//****************************************************************************
//* Description  :  Entsprechend dem im Instring_Zeichen - Array vorhandenen *
//*                 Befehl werden unterschiedliche Funktionen aufgerufen.    *
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

void interpreter(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   //standard command ever copy ( length 8 bytes )
   resultPtr[0]  = 8;
   resultPtr[1]  = commandPtr[1];
   resultPtr[2]  = commandPtr[2];
   resultPtr[3]  = commandPtr[3];

   // use and standard error
   writeLong(resultPtr+4, ERROR_INTERPRT | ERROR_COMMAND);

//*--------------------------------------------------------------------------*
//* parse commands                                                           *
//*--------------------------------------------------------------------------*
   switch (commandPtr[1])
   {      
      case TEXTAUSGABE:             break; // Special case: text output from other CPU

      case ACK:				         break;

#if ((CP_CMDDEV_MOST || CP_CMDDEV_IIC || CP_CMDDEV_CAN || CP_CMDDEV_RS232))
      case GATEWAY_BEFEHL:			   if (commandPtr[2] == 0)
                                    {
                                       // Edit only gateway number 0
                                    Gateway_Interpreter();
                                    printf("GATWAY INTERPRETER\n");
                                    }
                                    else
                                    {  // Insert Error value
                                       writeLong(resultPtr+4, ERROR_GATEWAY | GATEWAY_CMD_ERROR);
							                                // and report back
                                    }
                                    break;
#endif
#if CP_ADC
      case ADC_LESEN:               ADC_RD();
                                    break;
#endif
#if CP_PORT_RW
      case PORT_SCHREIBEN:          Port_WR();
                                    break;
      case PORT_LESEN:              Port_RD();
                                    break;
#endif
#if (CP_IIC0 || CP_IIC1 || CP_IIC2 || CP_IIC3 || CP_IIC4 || CP_IIC5 || CP_IIC6 || CP_IIC7 || CP_IIC10 || CP_IIC11)
      case IIC_LESEN:               IIC_RD();
                                    break;

      case IIC_SCHREIBEN:           IIC_WR();
                                    break;
#endif
#if CP_RAM
      case RAM_OPERATION:           RAM_RW();
                                    break;
#endif
#if CP_FPGA
      case FPGA_PROGRAMMIEREN:      FPGA_Prog();
                                    break;
#endif
#if CP_OS8104_PAR
      case MOST_SCHREIBEN:          MOST_WR();
                                    break;

      case MOST_LESEN:              MOST_RD();
                                    break;
#endif
#if (CP_RDS1 || CP_RDS2 || CP_RDS3)
      case RDS_LESEN :				   RDS_RD();
                                    break;
#endif
#if CP_DEBUG
	   case DEBUG_BEFEHL:		  	   Debug_Info();
	                                 break;
#endif
#if (CP_SER11 || CP_SPI0 || CP_SPI1 || CP_SPI2 || CP_SPI3 || CP_SPI4 || CP_SPI5)
      case SER_OPERATION:           Seriel_CMD();
                                    break;
#endif

#if (SW_SCRIPT == 1)
      case SCRIPT_CONTROLL :		   scriptCmd();
                                    break;
#endif

      case CONTROLLERTYP:           SubInterpreter_Status();
                                    break;

      case CMD_DIAGNOSTIC:          SubInterpreter_Diagnostic();
                                    break;

      case XTD_COMMAND:             SubInterpreter_xtd_CMD();
                                    break;

      case XTD_WRITE:		         SubInterpreter_xtd_Write();
                                    break;

      case XTD_READ:		            SubInterpreter_xtd_Read();
                                    break;

      case XTD_INIT :		         SubInterpreter_xtd_Init();
                                    break;
#if (CP_SELFFLASH)
      case CMD_FLASH_OP:
      {
         extern void NecFlash_RD (void);
         NecFlash_RD();
         break;
      }
#endif


        default:                    break; // "Kommando unbekannt" melden

   } // of switch

} // end of function




