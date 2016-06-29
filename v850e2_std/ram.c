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
#include <stdio.h>
#include "ram.h"
#include "tool.h"
#include <string.h>
#if CP_FPGA
   #include "fpga.h"
#endif

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if CP_RAM


#if (CP_SELFFLASH)
   extern void Flmd0_on();
   extern void Flmd0_off();
#endif

//****************************************************************************
//* Function     :  RAM_RW                                                   *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************
void RAM_RW(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
   UINT8  i;
   UINT32 addr;
   UINT8  checksum;
   UINT32 Error;

   UINT32 addr2,
          size,
          errorcount,
          j;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for correct command
   if ((commandPtr[1U] != 't') ||
      ((commandPtr[2U] == 0x00) && (commandPtr[3U] > 244U)) ||
      ((commandPtr[2U] == 0x01) && (commandPtr[3U] > 244U)) ||
      ((commandPtr[2U] == 0x02) && (commandPtr[3U] > 122U)) ||
      ((commandPtr[2U] == 0x03) && (commandPtr[3U] > 122U)) ||
      ((commandPtr[2U] == 0x04) && (commandPtr[3U] >  61U)) ||
      ((commandPtr[2U] == 0x05) && (commandPtr[3U] >  61U)) ||
      ((commandPtr[2U] == 0x06) && (commandPtr[3U] > 244U)) ||
      ((commandPtr[2U] == 0x07) && (commandPtr[3U] > 244U)) ||
      ((commandPtr[2U]  > 0x07) && (commandPtr[2U] < 0x10)) ||
      ((commandPtr[2U]  > 0x10) && (commandPtr[2U] < 0x15)) ||

#if CP_FPGA
      ((commandPtr[2U]  > 0x15) && (commandPtr[2U] < 0x80)) ||
       (commandPtr[2U]  > 0x83))
#else
      ((commandPtr[2U]  > 0x15) && (commandPtr[2U] != 0xF7)) )  // abgewandelt fuer SelfFlashBios
#endif
   {
      // send the answer and exit
      writeLong(resultPtr+4, ERROR_MEM | ERROR_MEM_COMMAND);
      return;
   }

   addr = readLong(&commandPtr[4U]);

   // Defaultfehler schon mal einsetzen
   Error = NO_ERROR;

   switch(commandPtr[2U])
   {
//*--------------------------------------------------------------------------*
//*   UINT8                                                                  *
//*--------------------------------------------------------------------------*

         case 0x00U: //  8bit read
            for(i = 0U; i < commandPtr[3U]; i++)
            {
               *(UINT8*)&resultPtr[8U + i] = *(UINT8*)addr;
               addr += 1U;
            }
            resultPtr[0] = 8 + commandPtr[3U];
            break;

      case 0x01U: //  8bit write
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            *(UINT8*)addr = *(UINT8*)&commandPtr[8U + i];
            addr += 1U;
         }
         break;

//*--------------------------------------------------------------------------*
//*   UINT16                                                                 *
//*--------------------------------------------------------------------------*

      case 0x02U: // 16bit read
         if(addr % 2U != 0U) // check for word boundary
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
            break;
         }
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            writeWord(resultPtr+8U + 2U * i, *(UINT16*)addr);
            addr += 2U;
         }
         resultPtr[0] =  8U  + (commandPtr[3U] * 2U);
         break;

      case 0x03U: // 16bit write
         if(addr % 2U != 0U) // check for word boundary
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
            break;
         }
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            *(UINT16*)addr = readWord (&commandPtr[8U + 2U * i]);
            addr += 2U;
         }
         break;

//*--------------------------------------------------------------------------*
//*   UINT32                                                                 *
//*--------------------------------------------------------------------------*

         case 0x04U: // 32bit read
            if(addr % 2U != 0U) // check for long boundary (=word boundary)
            {
               Error = ERROR_MEM | ERROR_MEM_COMMAND;
               break;
            }
            for(i = 0U; i < commandPtr[3U]; i++)
            {
               writeLong(resultPtr+8U + 4U * i, *(UINT32*)addr);
               addr += 4U;
            }
            resultPtr[0] =  8U  + (commandPtr[3U] * 4U);
            break;

      case 0x05U: // 32bit write
         if(addr % 2U != 0U) // check for long boundary (=word boundary)
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
            break;
         }
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            *(UINT32*)addr = readLong (&commandPtr[8U + 4U * i]);
            addr += 4U;
         }
         break;

//*--------------------------------------------------------------------------*
//*   UINT8 mit CRC                                                          *
//*--------------------------------------------------------------------------*

      case 0x06U: //  8bit read with checksum
         checksum = 0;
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            *(UINT8*)&resultPtr[11+i] = *(UINT8*)addr;
            checksum += *(UINT8*)addr;
            addr += 1U;
         }
         // calculate CRC in BIOSControl:  checksum = resultPtr[10] + ~checksum +1U;
         // checksum is 0 when data were successfully received
         resultPtr[10] = checksum;         // CRC
         resultPtr[9]  = commandPtr[9];     // Blocknumber (Hi)
         resultPtr[8]  = commandPtr[8];     // Blocknumber (Lo)
         resultPtr[0] = 11U + commandPtr[3U];
         break;

      case 0x07U: //  8bit write with checksum
         checksum = 0;
         for(i = 0U; i < commandPtr[3U]; i++)
         {
            *(UINT8*)addr = *(UINT8*)&commandPtr[11U + i];
            checksum += *(UINT8*)addr;
            addr += 1U;
         }
         // calculate CRC
         checksum = commandPtr[10] + ~checksum +1U;

         if (checksum != 0)
         {
            Error = ERROR_MEM | ERROR_MEM_CRC;
         }

         resultPtr[10] = checksum;         // Result CRC
         resultPtr[9]  = commandPtr[9];     // Blocknumber (Hi)
         resultPtr[8]  = commandPtr[8];     // Blocknumber (Lo)
         resultPtr[0]  = 11U;
         break;

//*--------------------------------------------------------------------------*
//*   Copy & Compare                                                         *
//*--------------------------------------------------------------------------*

      case 0x10U: // copy memory area
         memmove((void*) readLong(&commandPtr[8U]),
                 (void*) readLong(&commandPtr[4U]),
                 (UINT32)readLong(&commandPtr[12U]));
         break;


      case 0x15U: // compare bits
         errorcount = 0;
         addr  = readLong(&(commandPtr[4U]));
         addr2 = readLong(&(commandPtr[8U]));
         size  = readLong(&(commandPtr[12U]));

         for(j = 0U; j < size; j++)
         {
            if (*(UINT8*)(addr + j) != *(UINT8*)(addr2 + j))
            {
               errorcount++;
            }
         }

         resultPtr[0] = 12U;
         writeLong(resultPtr+8, errorcount);
         break;

//*--------------------------------------------------------------------------*
//*   FPGA                                                                   *
//*--------------------------------------------------------------------------*

      #if CP_FPGA
      case 0x80U: // 8-Bit FPGA Lesen
         if (commandPtr[3] >= 244U)
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
         }
         else
         {
            for(i=0;i< commandPtr[3] ;i++)
            {
               // Wert im 2 Bit Adressraum lesen
               resultPtr[i+8] = FPGA_RD ((UINT8)(addr & 0x00000003));
            }
            resultPtr[0] = commandPtr[3] +8U;
         }
         break;


      case 0x81U: // 8-Bit FPGA Schreiben
         if (commandPtr[3] >= 244U)
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
         }
         else
         {
            for(i=0;i< commandPtr[3] ;i++)
            {
               // Wert im 2 Bit Adressraum schreiben
               FPGA_WR ((UINT8)(addr & 0x00000003), commandPtr[8+i]);
            }
         }
         break;

      case 0x82U: // 8-Bit FPGA registermapped lesen
         if (commandPtr[3] >= 244U)
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
         }
         else
         {  // Leseadresse im FPGA setzen
            FPGA_WR (0, (UINT8)(addr & 0x000000FF));

            for(i=0;i< commandPtr[3] ;i++)
            {
               // Werte aus Registeradresse lesen
               resultPtr[i+8] = FPGA_RD (1);
            }
            resultPtr[0] = commandPtr[3] +8U;
         }
         break;


      case 0x83U: // 8-Bit FPGA registermapped schreiben
         if (commandPtr[3] >= 244U)
         {
            Error = ERROR_MEM | ERROR_MEM_COMMAND;
         }
         else
         {  // Schreibadresse im FPGA setzen
            FPGA_WR (0, (UINT8)(addr & 0x000000FF));

            for(i=0;i< commandPtr[3] ;i++)
            {
               // Wert im 2 Bit Adressraum schreiben
               FPGA_WR (1, commandPtr[8+i]);
            }
         }
         break;
      #endif // if CP_FPGA

      #if (CP_SELFFLASH)
      case 0xF7: //  8 bit write to flash  (8bit write with checksum)
      {
         UINT8 FillBytes;

         #if (FLASH_LIB_COPY)
            SelfLib_Init((void*)START_INTERNAL_RAM); // SelfFlash-Lib wird kopiert
         #else
            SelfLib_Init();                          // es handelt sich um das RAM-Bios
         #endif                                      // SelfLib wird nicht kopiert

         checksum = 0;
         // bilde CRC durch Summation der Uebertragenen Daten
         for(i = 0; i < commandPtr[3]; i++) checksum += *(UINT8*)&commandPtr[11 + i];

         if ( (checksum - commandPtr[10]) !=0)   Error = (ERROR_MEM | ERROR_MEM_CRC);
         else                             // Uebertragung ist OK
         {
            if (commandPtr[3] < 129 )      // es sind max. 128 Byte zulaessig
            {
               FillBytes = (128 - commandPtr[3]);

               // fuellt den Rest des Empgfangbuffers mit 0xFF auf
               // [ 11] --> erste DatenPosition  im Buffer
               // [138] --> letzte DatenPosition im Buffer
               for (i = 0; i < FillBytes; i++) commandPtr[138 - i] = 0xFF;

               Flmd0_on();
               INT_OFF;
               // Laengenangbe in DWORD's (4 Byte) --->    32 DWORDs = 128 byte
               Error = SelfLib_Write((void*)(commandPtr + 11), (void*)addr, 32);
               INT_ON;
               Flmd0_off();         // Error = 0, wenn OK
            }
            else
            {
               Error = (ERROR_MEM | ERROR_DATA_SIZE);  // Anzahl DatenBytes ist zu gross
            }
         }

         resultPtr[10] = checksum;         // Result CRC
         resultPtr[9]  = commandPtr[9];     // Blocknumber (Hi)
         resultPtr[8]  = commandPtr[8];     // Blocknumber (Lo)
         resultPtr[0]  = 11U;

         break;
      }
      #endif

//*--------------------------------------------------------------------------*
//*   Ende                                                                   *
//*--------------------------------------------------------------------------*

      default :  	// Errorwert fuer unbekanntes Kommando einsetzen
                  Error = ERROR_MEM | ERROR_COMMAND;
      				break;

   }

   writeLong(resultPtr+4, Error);

} // of function RAM_RW

#endif // #if CP_RAM
