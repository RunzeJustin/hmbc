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
#include "tool.h"

// Leerdefinition fuer Makros in portlist.h
#define JOD_ENDPORTDEF(PORTNAME)
#define OD_ENDPORTDEF(PORTNAME)
#define JOD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)
#define OD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)
#define JOD_PORTDEF(PORTNAME)
#define OD_PORTDEF(PORTNAME)


#include "portlist.h"

#include "comm.h"



//****************************************************************************
//* Function     :	ledON/ledOFF                                            *
//****************************************************************************
//* Description  : 	                                                        *
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

#if SW_LED
void ledON (unsigned channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   LED_MESSAGE_ACTIVATE;
} // end of ledON

void ledOFF (unsigned channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   LED_MESSAGE_DEACTIVATE;
} // end of ledOFF
#endif

//****************************************************************************
//* Function     :	Insert_Fifo_32                                          *
//****************************************************************************
//* Description  :   32 Bit Parameter in den Buffer schreiben                *
//*                  in big endian                                           *
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

#if CP_BUFFER
BOOL Insert_Fifo_32 (UINT32 Wert, TBuffer *Buffer)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (bufSpace(Buffer) >= 4)
   {
      bufPutFiFo(Buffer, (UINT8)((0xff000000 & Wert) >> 24));
      bufPutFiFo(Buffer, (UINT8)((0x00ff0000 & Wert) >> 16));
      bufPutFiFo(Buffer, (UINT8)((0x0000ff00 & Wert) >> 8));
      bufPutFiFo(Buffer, (UINT8) (0x000000ff & Wert));
      return(TRUE);
   }
   else
      return(FALSE);
} // end of Insert_Fifo_32


//****************************************************************************
//* Function     :	Insert_Fifo_16                                          *
//****************************************************************************
//* Description  : 	16 Bit Parameter in den Buffer schreiben                *
//*                  in big endian                                           *
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

BOOL Insert_Fifo_16 (UINT16 Wert, TBuffer *Buffer)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (bufSpace(Buffer) >= 2)
   {
      bufPutFiFo(Buffer, (UINT8)((0xff00 & Wert) >> 8));
      bufPutFiFo(Buffer, (UINT8) (0x00ff & Wert));
      return(TRUE);
   }
   else
      return(FALSE);
} // end of Insert_Fifo_16


//****************************************************************************
//* Function     :	Get_Fifo_16                                             *
//****************************************************************************
//* Description  : 	16 Bit Parameter aus dem Buffer lesen                   *
//*                  in big endian                                           *
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

UINT16 Get_Fifo_16 (TBuffer *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Bytes[2];

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (bufItems(Buffer) >= 2)
   {
      bufGet (Buffer, &Bytes[1]);
      bufGet (Buffer, &Bytes[0]);

      return ((UINT16)Bytes[0] |((UINT16)Bytes[1] << 8));
   }
   else
      return(0xFFFF);
} // end of Get_Fifo_16


//****************************************************************************
//* Function     :	Get_Fifo_32                                             *
//****************************************************************************
//* Description  : 	32 Bit Parameter aus dem Buffer lesen                   *
//*                  in big endian                                           *
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

UINT32 Get_Fifo_32 (TBuffer *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Bytes[4];

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (bufItems(Buffer) >= 4)
   {
      bufGet (Buffer, &Bytes[3]);
      bufGet (Buffer, &Bytes[2]);
      bufGet (Buffer, &Bytes[1]);
      bufGet (Buffer, &Bytes[0]);

      return ((UINT32)Bytes[0] | ((UINT32)Bytes[1] << 8)  |
             ((UINT32)Bytes[2] << 16) | ((UINT32)Bytes[3] << 24));
   }
   else
      return(0xFFFFFFFF);
} // end of Get_Fifo_32
#endif

//****************************************************************************
//* Function     :  Status Check                                             *
//****************************************************************************
//* Description  :  Diese Funktion sendet Informationsbytes an den PC,       *
//*                 welche den Mikrocontroller identifiziert.                *
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

void Statusabfrage(void)
{
   // Gesamtlänge des Strings
   resultPtr [0] = 0x18;
   resultPtr [1] = CONTROLLERTYP;

   // Prozessortyp einsetzen
   resultPtr [2] = CPU_TYPE;
   // Systemzustand
   resultPtr [3] = CPU_Running;

   // Fehlereintrag ins Antwortprotokoll
   writeLong(resultPtr+4,NO_ERROR);

   // Datum
   resultPtr [8]  = ((verDate[8]-0x30)<<4) + (verDate[9]-0x30);  // Tag
   resultPtr [9]  = ((verDate[5]-0x30)<<4) + (verDate[6]-0x30);  // Monat
   resultPtr [10] = ((verDate[2]-0x30)<<4) + (verDate[3]-0x30);  // Jahr

    // Zeit
   resultPtr [11] = ((verTime[0]-0x30)<<4) + (verTime[1]-0x30);  // Stunde
   resultPtr [12] = ((verTime[3]-0x30)<<4) + (verTime[4]-0x30);  // Minute

   // Beckernummer
   #ifdef BoardVersion
      resultPtr [13] = 0xFF;
      resultPtr [14] = 0xFF;
   #else
      resultPtr [13] = ((verStr[ 8]-0x30) <<4) + (verStr[ 9]-0x30);
      resultPtr [14] = ((verStr[10]-0x30) <<4) + (verStr[11]-0x30);
   #endif

   // Musterstand : 1 Character + 1 Ziffer
   resultPtr [15] = ((verStr[13]-0x37)<<4) + (verStr[14]-0x30);

   // Revisionsnummer
   resultPtr [16] = ((verStr[26]-0x30)<<4) + (verStr[27]-0x30);

   // Dummybytes einsetzen
   resultPtr [17] = 0xDD;
   resultPtr [18] = 0xDD;
   resultPtr [19] = 0xDD;

   // Modeeintrag
   writeLong(resultPtr+20, getModeReg());

} // end of function


//****************************************************************************
//* Function     :  DelayLoop                                                *
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

void DelayLoop (UINT32 n)
{
   if (!n)
   {
      #pragma asm
      nop
      nop
      #pragma endasm
   }
   for (; n > 0; n--)
   {}

} // end of function


//****************************************************************************
//* Function     :	Test_Ext_Ram                                            *
//****************************************************************************
//* Description  : 	test Ram with a little pattern                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   True : RAM is operational                               *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :  old contents is restored                                 *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL Test_Ram (UINT8 *Ram)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Bytes[8];
UINT8    MagicBytes[8] = { 'M','a','g','i','c',0x00,0xFF,0xAA };
UINT32   i;
BOOL result = TRUE;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // save 8 Bytes
   memcpy (Bytes, Ram, 8);

   // write magic
   memcpy (Ram, MagicBytes, 8);

   // compare
   for (i=0 ; i<sizeof(MagicBytes); i++)
   {
      if (Ram[i] !=  MagicBytes[i])
         result = FALSE;
   }

   // restore 8 Bytes
   memcpy (Ram, Bytes, 8);

   return(result);
} // end of Test_Ram


//****************************************************************************
//* Function     :	Debugxxxx                                               *
//****************************************************************************
//* Description  :  Sammlung verschiedener Print-Routinen zu Debugzwecken    *
//*                                                                          *
//* Parameter    :  aehnlich wie  printf                                     *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if CP_DEBUG_TOOL

void Debug_Format_Print (UINT8 Channel, const INT8 *Format, ...)
{
   va_list        arg_ptr;
   INT8 Mybuf [120];
   UINT8  Anzahl;
   UINT16 i;

   // Argumentenliste erzeugen
   va_start (arg_ptr, Format);

   // Textstring generieren
   Anzahl = vsprintf (Mybuf,Format,arg_ptr);
   for (i = 0; i < Anzahl ; i++)
      SerWriteChar (Channel, Mybuf[i]);
   return;
} // end of function

//*--------------------------------------------------------------------------*

void Debug_ASCII_Print (UINT8 *Buffer)
{
   UINT16 i;

   for (i = 1; i < Buffer[0] ; i++)
      SerWriteChar (0, Buffer[i]);
   return;
} // end of function

//*--------------------------------------------------------------------------*

void Debug_Textmessage_Print (UINT8 *Buffer)
{
   UINT16 i;

   for (i = 2; i < Buffer[0] ; i++)
      SerWriteChar (0, Buffer[i]);
   return;
} // end of function


//*--------------------------------------------------------------------------*

void Debug_CMD_Print (UINT8 *Buffer)
{
   UINT16 i;

   for (i = 0; i < Buffer[0] ; i++)
      	Debug_Format_Print (0, "%02X ",Buffer[i]);
   return;
} // end of function
#endif


