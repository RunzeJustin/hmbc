//*******************************************************************************
//*                   _____   _____   _   _   ___                               *
//*                  |  ___| |  ___| | | | | |   `                              *
//*                  | |_    | |_    | |_| | | [] |                             *
//*                  |  _|   |  _|   |  _  | |  _ <                             *
//*                  | |___  | |___  | | | | | |_| |                            *
//*                  |_____| |_____| |_| |_| |____/                             *
//*                                                                             *
//*  Project:          BIOSControl uPD70Fxxxx                                   *
//*  (c) copyright     Harman-Becker Automotive Systems                         *
//*******************************************************************************

// ------------------------------------------------------------------------------
//  Include files
// ------------------------------------------------------------------------------
#include <string.h>          		   // für memcpy
#include <stdint.h>
#include <stdarg.h>
                                    // voneinander abhaengige includes
                                    // daher Reihenfolge wichtig
#include "cpu_def.h"                // Registerdefinitionen des gewaehlten up
#include "config.h"                 // Schalter fuer Projektspezifische Einstellungen
#include "define.h"                 // global definitions
#include "global.h"                 // globale Datenstrukturen
#include "comm.h"
#include "ints.h"						   // Interruptverwaltung
#include "tool.h"
#include "spi.h"
#include "buffer.h"
#include "timer.h"
#include "ser.h"



// ------------------------------------------------------------------------------
// 	Hauptschalter für das ganze File ab hier
// ------------------------------------------------------------------------------
#if (CP_CSIH0 || CP_CSIH1 || CP_CSIH2)


#define  BIT0     (1<<0)
#define  BIT1     (1<<1)
#define  BIT2     (1<<2)
#define  BIT3     (1<<3)
#define  BIT4     (1<<4)
#define  BIT5     (1<<5)
#define  BIT6     (1<<6)
#define  BIT7     (1<<7)
#define  BIT8     (1<<8)
#define  BIT9     (1<<9)
#define  BIT10    (1<<10)
#define  BIT11    (1<<11)
#define  BIT12    (1<<12)
#define  BIT13    (1<<13)
#define  BIT14    (1<<14)
#define  BIT15    (1<<15)
#define  BIT16    (1<<16)
#define  BIT17    (1<<17)
#define  BIT18    (1<<18)

#define  ALL_CS_INAKTIV    BIT7
#define  CS_LEVEL_INAKTIV  BIT6
#define  CS_LEVEL_AKTIV    0


#define SPI1_PORT_UNCONFIG    PMC1 &= ~(BIT9 | BIT8 | BIT7);
#define SPI2_PORT_UNCONFIG    PMC1 &= ~(BIT4 | BIT3 | BIT2);


/*
struct SPI_CSIH            // angepasst an CSIH
{
   UINT8    ctl0;          // 0x00
   UINT8    dummy1;        // 0x01
   UINT8    dummy2;        // 0x02
   UINT8    dummy3;        // 0x03

   UINT32   str0;          // 0x04

   UINT16   stcr0;         // 0x08
   UINT16   dummy4;        // 0x0A

   UINT32   dummy5;        // 0x0C

   UINT32   ctl1;          // 0x10

   UINT16   ctl2;          // 0x14
   UINT16   dummy6;        // 0x16

   UINT8    emu;           // 0x18
   UINT8    dummy7;        // 0x19
   UINT8    dummy8;        // 0x1A
   UINT8    dummy9;        // 0x1B

   UINT32   dummy10;       // 0x1C bis 0x1F

   // ----------------------------------------------
   UINT8    dummy11[0xFE0];// fuellt auf bis 0x1000-1
   // -----------------------------------------------

   UINT32   mctl1;         // 0x1000

   UINT32   mctl2;         // 0x1004

   UINT32   tx0w;          // 0x1008

   UINT16   tx0h;          // 0x100C
   UINT16   dummy12;       // 0x100E


   UINT32   rx0w;          // 0x1010

   UINT16   rx0h;          // 0x1014
   UINT16   dummy13;       // 0x1016

   UINT32   mrwp0;         // 0x1018

   UINT32   dummy14[10];   // 0x101C

   UINT32   cfg0;          // 0x1044
   UINT32   cfg1;          // 0x1048
   UINT32   cfg2;          // 0x104C
   UINT32   cfg3;          // 0x1050
   UINT32   cfg4;          // 0x1054
   UINT32   cfg5;          // 0x1058
   UINT32   cfg6;          // 0x105C
   UINT32   cfg7;          // 0x1060
};
*/


//*--------------------------------------------------------------------------*
//* SPI-Message Transceiver possible States                                  *
//*--------------------------------------------------------------------------*

#define SPIMSG_MASTER_DISABLED            0
#define SPIMSG_MASTER_ENABLED   	 		   1
#define SPIMSG_MASTER_TRANSMITTING        2
#define SPIMSG_MASTER_RECEIVED            3
#define SPIMSG_MASTER_ERRORSTATE          4


// ------------------------------------------------------------------------------
//  Local defines
// ------------------------------------------------------------------------------
#define SER_CH_NUM      2        // höchste Kanalnummer (Kanaele 0,1,2 CSIH)
#define SPI_BUF_SIZE    256      // Buffergroesse


// Prototypes
BOOL     SpiWrite_String_CSIH(UINT8 Channel, UINT8 Number, UINT8* SendData, UINT8* ReceiveData);
UINT32   CalculateSpiSettings_CSIH(UINT8 Channel, UINT32 Baudrate, UINT8 Mode);
UINT32   GyroTest(void);


// globale Varibeln

// -------------------------------------------
// uebertrage maximal 16 Daten-Bytes // momentan nur CSIH 1 Byte zum Interrupt-Anstossen
volatile UINT8    CSIH_TransmitCounter_Ch1 = 0;
volatile UINT8    CSIH_TransmitCounter_Ch2 = 0;
volatile UINT8    CSIH_TransmitIndex_Ch1   = 0;
volatile UINT8    CSIH_TransmitIndex_Ch2   = 0;
volatile UINT8    CSIH_TransmitBuffer_Ch1[16];
volatile UINT8    CSIH_TransmitBuffer_Ch2[16];
// -------------------------------------------
volatile UINT8    CSIH_ReceiveCounter_Ch1 = 0;
volatile UINT8    CSIH_ReceiveCounter_Ch2 = 0;
// -------------------------------------------

volatile UINT8    CSIH_CS_Ch_ON  = 0;           // hier wird vermerkt wenn CS-Leitung des Channels
                                                // benutzt wird



static   BOOL  SpiOpen[16];      // Schnittstelle 0 bis 15
//                                  CSIH Schnittstelle enthaelt nur die Channel 0 + 4
UINT32         csih_base[3] = {0xFF6C0000, 0xFF6D0000, 0xFF6E0000};


/* *************************************************************************** */
void SetChipSelect_CSIH(UINT8 data)   // arbeitet nur fuer Channel 0 bis 7 korrekt !
{
   UINT8    Channel;


   Channel = (data & 0x3F);      // nur unteren 6 Bits

   if ( (data & ALL_CS_INAKTIV) == ALL_CS_INAKTIV)
   {
      SETPORT_1(1,  6);     // eeprom
      SETPORT_1(1, 11);     // CRM120
      SETPORT_1(1, 12);     // AIS328DQ
      return;
   }

   // wenn keine weitere CS Steuerung eingeschaltet
   if ((CSIH_CS_Ch_ON & (1<<Channel)) == 0)
      return;

   if ( (data & CS_LEVEL_INAKTIV) == CS_LEVEL_INAKTIV)
   {
      if (Channel == 1) SETPORT_1(1, 12);           // Ch1 : Port 1.12   AIS328DQ
      if (Channel == 2) SETPORT_1(1,  6);           // Ch2 : --> eeprom
      if (Channel == 4) SETPORT_1(1, 11);           // VCh4: Port 1.11   CRM120
   }
   else                                             // CS aktiv/low
   {
      if (Channel == 1) SETPORT_0(1, 12);           // Ch1 : Port 1.12   AIS328DQ
      if (Channel == 2) SETPORT_0(1,  6);           // Ch2 : --> eeprom
      if (Channel == 4)                             // VCh4: Port 1.11   CRM120
      {
         SETPORT_0(1, 11);
         DelayLoop(2000);     // benoetigt Vorlaufzeit von 25µs (25µs / 80MHz = 2000)
      }
   }

   DelayLoop(200);

   return;
}


//*******************************************************************************
//* Function     :  SerielSpi_CMD                                               *
//*-----------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested      *
//*******************************************************************************
void SerielCSIH_CMD (void)
{
   UINT32   Error;
   UINT32   BaudRate;
   UINT8    Channel;                // physisch vorhandener Kanal
   UINT8    Proto;
   UINT8    cmd = commandPtr[2];


   // Grobtest auf Kommandosyntax
   //                   Size                    Kommando
   if (((commandPtr[0] ==  9) && (cmd == 0x00)) ||     // Open  ist mit  9 zulaessig
       ((commandPtr[0] ==  4) && (cmd == 0x01)) ||     // Close ist mit  4 zulaessig
       ((commandPtr[0]  >  4) && (cmd == 0x02)) ||     // write ist mit >4 zulaessig
       ((commandPtr[0]  >  5) && (cmd == 0x03)) ||     // read  ist mit >5 zulaessig
       ((commandPtr[0]  >  4) && (cmd == 0xE0)) ||     // extended open
       ((commandPtr[0] == 19) && (cmd == 0xF0))  )     // Gyro-Test
   {}
   else
   {
      writeLong(resultPtr+4,  ERROR_SER | ERROR_COMMAND);
      return;
   }

   Error = NO_ERROR;

   // Channel errechnen
   Channel    = commandPtr[3] - CSIH_CH_OFFSET;       // -0x60
   BaudRate   = readLong(commandPtr+4);
   Proto      = commandPtr[8];

   // bislang sind nur Channel 1 + 2 erlaubt  +    Virtual Channel 4 fuer CRM120
   if ( (Channel != 0x01) && (Channel != 0x02) && (Channel != 0x04))
   {
      // Interpreter Commandfehler defaultmässig schicken
      writeLong(resultPtr+4, ERROR_SER | SER_ERR_CHANNEL);
      return;
   }

   switch (cmd)
   {
      //*--------------------------------------------------------------------------*
      //*	Open Serial Interface                                                  *
      //*--------------------------------------------------------------------------*
      case 0x00:
      {
         if (SpiOpen[Channel] == FALSE)
         {
            Error = CalculateSpiSettings_CSIH(Channel, BaudRate, Proto);

            if (Error == 0) SpiOpen[Channel] = TRUE;
         }
         else  Error = (ERROR_SER | SER_ERR_OPEN);

         break;
      }

      //*-----------------------------------------------------------------------*
      //*	Close Serial Interface                                              *
      //*-----------------------------------------------------------------------*
      case 0x01:
      {
         if ( (Channel == 0x01) || (Channel == 0x04) )   // Virtual Channel 4
         {
            jprintf("Transmit1-Int-Cnt: %d", CSIH_TransmitIndex_Ch1);
            jprintf("Receive1--Int-Cnt: %d", CSIH_ReceiveCounter_Ch1);
            SPI1_PORT_UNCONFIG
            CSIH1CTL0 = 0;                // disable TX+RX+Power
         }

         if (Channel == 0x02)
         {
            jprintf("Transmit2-Int-Cnt: %d", CSIH_TransmitIndex_Ch2);
            jprintf("Receive2--Int-Cnt: %d", CSIH_ReceiveCounter_Ch2);
            SPI2_PORT_UNCONFIG
            CSIH2CTL0 = 0;                // disable TX+RX+Power
         }

         SpiOpen[Channel] = FALSE;

         break;
      }

      //*-----------------------------------------------------------------------*
      //*	Write to Serial Interface                                           *
      //*-----------------------------------------------------------------------*
      case 0x02:
      {
         UINT8 Number = (commandPtr[0] - 4);     // Anzahl an SPI Daten


         if (Number < 17)                       // max. 16 Byte
         {
            if (SpiOpen[Channel] == TRUE)
            {
               SpiWrite_String_CSIH(Channel, Number, (commandPtr+4), (resultPtr+8));
               resultPtr[0] = (Number + 8);
            }
            else
            {
               jprintf("error: Channel %d wurde zuvor nicht initialisiert", Channel);
               Error = (ERROR_SER | SER_ERR_OPEN);
            }
         }
         else
         {
            jprintf("error: Anzahl Transferbytes zu gross (max 16)");
            Error = (ERROR_SER | ERROR_COMMAND);
         }
         break;
      }

      //*--------------------------------------------------------------------------*
      //*	Read from Serial Interface                                             *
      //*--------------------------------------------------------------------------*
      case 0x03:
      {
         UINT8 Number = commandPtr[4];     // Anzahl an SPI Daten die zu lesen sind


         if (Number < 17)
         {
            if (SpiOpen[Channel] == TRUE)
            {  //                                    dummy buffer
               SpiWrite_String_CSIH(Channel, Number, (commandPtr+5), (resultPtr+8));
               resultPtr[0] = (Number + 8);
            }
            else
            {
               jprintf("error: Channel %d wurde zuvor nicht initialisiert", Channel);
               Error = (ERROR_SER | SER_ERR_OPEN);
            }
         }
         else
         {
            jprintf("error: Anzahl Transferbytes zu gross");
            Error = (ERROR_SER | ERROR_COMMAND);
         }
         break;
      }

      /*
      case 0xE0:
      {
         Error = spi_extopen ((commandPtr[3]-CSIH_CH_OFFSET),
                               commandPtr[0]-4, &commandPtr[4]);
         break;
      }
      */

      case 0xF0:
      {
         GyroTest();
         break;
      }
   }  // of switch(commandPtr[2])

   //*--------------------------------------------------------------------------*
   //* zum Schluss Ergebnisstring ausgeben                                      *
   //*--------------------------------------------------------------------------*

   // Ergebnis senden
   writeLong(resultPtr+4, Error);
}


// ------------------------------------------------------------------------------
// called by CMD-open
// calculate settings + CSIH-Init
UINT32 CalculateSpiSettings_CSIH(UINT8 Channel, UINT32 Baudrate, UINT8 Mode)
{
   UINT32            TestClock, Differenz, ModulClock;
   UINT32            OldDifferenz;
   UINT16            cnt;
   UINT16            RegValue;
   UINT32            CSIHnCKR;
   UINT32            CSIHnDAP;


   ModulClock   = GetDomainClock(CLK_DOMAIN_108);        // for CSIH0,1,2
   OldDifferenz = ModulClock;
   jprintf("ModulClock: %d", ModulClock);

   // fange ungeueltige/nicht implementierte Protokolleinstellungen ab
   if ( ((Mode & BIT6) == BIT6) ||     // momentan nur 3-Line oder 4-Line mit CS zulaessig
        ((Mode & BIT4) == BIT4) ||     // momentan nur Master-Mode zulaessig
        ((Mode & BIT2) == BIT2)  )     // momentan nur 8-Bit-Mode zulaessig
   {
      return(ERROR_SER | SER_ERR_PROTOCOL);
   }

   if ((Mode & BIT5) == BIT5)          // verwende zusaetzlich CS-Leitung
   {
      CSIH_CS_Ch_ON |=  (1<<Channel);  // setze entsprechendes Bit
   }
   else
   {
      CSIH_CS_Ch_ON &= ~(1<<Channel);  // loesche entsprechendes Bit
   }

   // in portlist all CS-lines are low (aktiv)   !!!!!!!!
   SetChipSelect_CSIH(ALL_CS_INAKTIV);         // all CS lines inaktiv (high)

   // alternative port-configuration for SPI-suuport
   switch (Channel)
   {
      case 1:  // CSIH 1      Master               // Sensoren
      case 4:  // auch fuer den virtual Channel 4 gueltig
      {
         // port-pin-configuration

         // alternative  ------- Funktionswahl -------   -- In/Out --         Steuerung durch Modul
         /* CSIHHSC @ P1_9  */                      //   0 -> Out, 1 -> In
         PMC1 |= BIT9;   PFCE1 |= BIT9; PFC1 &= ~BIT9;   PM4 &= ~BIT9;        PIPC1 |= BIT9;

         /* CSIH4S0 @ P1_8  */
         PMC1 |= BIT8;   PFCE1 |= BIT8; PFC1 &= ~BIT8;   PM1 &= ~BIT8;        PIPC1 |= BIT8;

         /* CSIH4SI @ P1_7  */
         PMC1 |= BIT7;   PFCE1 |= BIT7; PFC1 &= ~BIT7;   PM1 |= BIT7;         FCLA22CTL6=0x80;

         break;
      }

      case 2:  // CSIH 2      Master               // Flash
      {
         // port-pin-configuration

         // alternative  ------- Funktionswahl -------   -- In/Out --         Steuerung durch Modul


         /* CSIHHSC @ P1_4  */                      //   0 -> Out, 1 -> In
         PMC1 |= BIT4;   PFCE1 |= BIT4;  PFC1 |= BIT4;   PM1 &= ~BIT4;        PIPC1 |= BIT4;

         /* CSIH4S0 @ P1_3  */
         PMC1 |= BIT3;   PFCE1 |= BIT3;  PFC1 |= BIT3;   PM1 &= ~BIT3;        PIPC1 |= BIT3;

         /* CSIH4SI @ P1_2  */
         PMC1 |= BIT2;   PFCE1 |= BIT2;  PFC1 |= BIT2;   PM1 |= BIT2;         FCLA23CTL2=0x80;

         break;
      }

      default:
      {
         jprintf("CSIH Channel nicht vorhanden");
            return(ERROR_SER | SER_ERR_CHANNEL);
      }
   }

   // kalkuliere Settings fuer communication clock
   for (cnt = 1; cnt <= 4095; cnt++)
   {
      TestClock  = ModulClock / (cnt<<1);

      Differenz  = abs(Baudrate-TestClock);

      if (Differenz < OldDifferenz)
      {
         OldDifferenz = Differenz;
         RegValue = cnt;
      }
      else break;
   }
   RegValue &= 0xFFF;


   // create register-settings for SPI mode
   switch(Mode & 0x03)
   {
      //   Doku          Register
      case 0x00:
      {
         CSIHnDAP = 0;
         CSIHnCKR = BIT17;
         break;
      }

      case 0x01:
      {
         CSIHnDAP = BIT16;
         CSIHnCKR = BIT17;
         break;
      }

      case 0x02:
      {
         CSIHnDAP = 0;           // CSIHnCFG0 register
         CSIHnCKR = 0;           // CSIHnCTL1 register
         break;
      }

      case 0x03:
      {
         CSIHnDAP = BIT16;
         CSIHnCKR = 0;
         break;
      }
   }

   if ( (Channel != 1) &&  (Channel != 2) &&  (Channel != 4))
   {
      return(ERROR_SER|SER_ERR_CHANNEL);
   }

   if ( (Channel == 1) || (Channel == 4) )      // virtual channel 4
   {
      CSIH1CTL0   = 0x01;        // must be
      CSIH1CTL0  |= 0x80;
      CSIH1CTL1  = (0x00010000 | CSIHnCKR); // interrupt generation when free for next data | SCK initial level selection
      CSIH1CTL2  = RegValue;                // master-mode
      CSIH1CFG0  = (0x08000000 | CSIHnDAP); // 8 bit data length | Data-Phase-Selection
      CSIH1CTL0  = 0xE1;                    // enable CSIG, TX+RX+PWR + Bypass-Mode

      // enable Transmit and Receive Interrupt
      ICCSIH1IR   =  0x0000;           // enable receive interrupt
      ICCSIH1IC   =  0x0000;           // enable tranmit interrupt
   }

   if (Channel == 2)
   {
      CSIH2CTL0   = 0x01;        // must be
      CSIH2CTL0  |= 0x80;
      CSIH2CTL1  = (0x00010000 | CSIHnCKR); // interrupt generation when free for next data | SCK initial level selection
      CSIH2CTL2  = RegValue;                // master-mode
      CSIH2CFG0  = (0x08000000 | CSIHnDAP); // 8 bit data length | Data-Phase-Selection
      CSIH2CTL0  = 0xE1;                    // enable CSIG, TX+RX+PWR + Bypass-Mode

      // enable Transmit and Receive Interrupt
      ICCSIH2IR   =  0x0000;           // enable receive interrupt
      ICCSIH2IC   =  0x0000;           // enable tranmit interrupt
   }

   jprintf("CSIH Channel %d tuned baudrate: %d", Channel, ModulClock / (2*RegValue) );
   return(0);
}


// ------------------------------------------------------------------------------
BOOL SpiWrite_String_CSIH(UINT8 Channel, UINT8 Number, UINT8* SendData, UINT8* ReceiveData)
{
   UINT8    *pData;
   UINT8    cnt;
   UINT32   time1;


   pData = SendData;


   // ---------------------------------------------------------------------------
   if ( (Channel == 1) || (Channel == 4) )
   {
      CSIH_TransmitCounter_Ch1 = Number;
      CSIH_ReceiveCounter_Ch1  = 0;


      // kopiert Daten (max. 16) in den Interrupt-Transmitbuffer um
      for (cnt = 0; cnt < Number; cnt++)
      {
         CSIH_TransmitBuffer_Ch1[cnt] = *pData;
         //jprintf("Data: 0x%2X", CSIH_TransmitBuffer_Ch0[cnt]);
         pData++;  // erhoehe DataPointer
      }

      SetChipSelect_CSIH(Channel | CS_LEVEL_AKTIV);   // Leitung 1 oder 4, aktiv low

      CSIH_TransmitCounter_Ch1--;                     // 1 Byte wird ohne Interrupt uebertragen
      CSIH_TransmitIndex_Ch1 = 1;                     // Buffer-Index, Startwert fuer ISR

      CSIH1TX0H = CSIH_TransmitBuffer_Ch1[0];         // stosse interruptgetriebenes Senden an

      time1 = timGetTime();
      do
      {
         if (timGetTime() > (time1+2000))             // 2000ms Zeit
         {
            jprintf("Ch1 TimeOut  erwartet: %d     empfangen: %d", Number, CSIH_ReceiveCounter_Ch1);

            return(FALSE);
         }
      }
      while (CSIH_ReceiveCounter_Ch1 != Number);      // warte bis alle Daten empfangen wurden
      SetChipSelect_CSIH(Channel | CS_LEVEL_INAKTIV); // Leitung 1 oder 4 inaktiv, high
   }

   // ---------------------------------------------------------------------------
   if (Channel == 2)
   {
      CSIH_TransmitCounter_Ch2 = Number;
      CSIH_ReceiveCounter_Ch2  = 0;


      // kopiert Daten (max. 16) in den Interrupt-Transmitbuffer um
      for (cnt = 0; cnt < Number; cnt++)
      {
         CSIH_TransmitBuffer_Ch2[cnt] = *pData;
         //jprintf("Data: 0x%2X", CSIH_TransmitBuffer_Ch0[cnt]);
         pData++;  // erhoehe DataPointer
      }

      SetChipSelect_CSIH(Channel | CS_LEVEL_AKTIV);      // Leitung 2, aktiv low

      CSIH_TransmitCounter_Ch2--;                     // 1 Byte wird ohne Interrupt uebertragen
      CSIH_TransmitIndex_Ch2 = 1;                     // Buffer-Index, Startwert fuer ISR

      CSIH2TX0H = CSIH_TransmitBuffer_Ch2[0];         // stosse interruptgetriebenes Senden an

      time1 = timGetTime();
      do
      {
         if (timGetTime() > (time1+2000))             // 2000ms Zeit
         {
            jprintf("Ch1 TimeOut  erwartet: %d     empfangen: %d", Number, CSIH_ReceiveCounter_Ch2);

            return(FALSE);
         }
      }
      while (CSIH_ReceiveCounter_Ch2 != Number);      // warte bis alle Daten empfangen wurden
      SetChipSelect_CSIH(Channel | CS_LEVEL_INAKTIV);    // Leitung 2 inaktiv, high
   }

   return(TRUE);
}


// start ISR Routinen ------------------------------------------------------------------------------
// Channel (1) transmit interrupt ISR --------------------------------------------------------------
__interrupt void INTCSIH1IC()
{
   if (CSIH_TransmitCounter_Ch1 > 0)
   {
      CSIH1TX0H = CSIH_TransmitBuffer_Ch1[CSIH_TransmitIndex_Ch1];
      CSIH_TransmitCounter_Ch1--;
      CSIH_TransmitIndex_Ch1++;
   }
}


// Channel (1) receive interrupt ISR --------------------------------------------
__interrupt void INTCSIH1IR()
{
   UINT8 data;

   data = CSIH1RX0H;

   // wird momentan nicht als IPC benutzt
   //if ( (CSIH_IPC_Ch_ON & BIT0) == BIT0)    // IPC Channel 0 ist ON
   //{
   //   msgReceive(msgRegisterHandle_Ch1, &data, 1);
   //}
   //else
   {
      // maximal 248 (256-8) Byte in einem Transfer
      if (CSIH_ReceiveCounter_Ch1 < 248)
         resultPtr[8+CSIH_ReceiveCounter_Ch1] = data;
   }

   CSIH_ReceiveCounter_Ch1++;
}

// ------------------------------------------------------------------------------
// Channel (2) transmit interrupt ISR -------------------------------------------
__interrupt void INTCSIH2IC()
{
   if (CSIH_TransmitCounter_Ch2 > 0)
   {
      CSIH2TX0H = CSIH_TransmitBuffer_Ch2[CSIH_TransmitIndex_Ch2];
      CSIH_TransmitCounter_Ch2--;
      CSIH_TransmitIndex_Ch2++;
   }
}


// Channel (1) receive interrupt ISR --------------------------------------------
__interrupt void INTCSIH2IR()
{
   UINT8 data;

   data = CSIH2RX0H;

   // wird momentan nicht als IPC benutzt
   //if ( (CSIH_IPC_Ch_ON & BIT0) == BIT0)    // IPC Channel 0 ist ON
   //{
   //   msgReceive(msgRegisterHandle_Ch1, &data, 1);
   //}
   //else
   {
      // maximal 248 (256-8) Byte in einem Transfer
      if (CSIH_ReceiveCounter_Ch2 < 248)
         resultPtr[8+CSIH_ReceiveCounter_Ch2] = data;
   }

   CSIH_ReceiveCounter_Ch2++;
}

// -------------------------------------------------------------------------------------------------
void Ram_Print (UINT32* Buffer, const INT8 *Format, ...)
{
   //*--------------------------------------------------------------------------*
   //* Local variables                                                          *
   //*--------------------------------------------------------------------------*

   va_list        arg_ptr;
   UINT8  Anzahl;

   //*--------------------------------------------------------------------------*
   //* Start of function                                                        *
   //*--------------------------------------------------------------------------*

   // Argumentenliste erzeugen
   va_start (arg_ptr, Format);

   // Textstring generieren
   Anzahl = vsprintf ((UINT8*)*Buffer,Format,arg_ptr);

   *Buffer += Anzahl;
} // of function Ram_Print


UINT8    GYRO_TX_Data[] = {0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
UINT8    GYRO_RX_Data[sizeof(GYRO_TX_Data)];

UINT8    CRM100_TX_Data[] = {0x38, 0x00, 0x00, 0x00, 0x00, 0xC7};
UINT8    CRM100_RX_Data[sizeof(CRM100_TX_Data)];


// ------------------------------------------------------------------------------
UINT32 GyroTest(void)
{
   #define  eRAM_Start           0x04000000        // externes RAM
   #define  RAM_SIZE             0x80000           // 512kByte
   UINT16   i;
   INT16    GYRO_Out_X, GYRO_Out_Y, GYRO_Out_Z;
   UINT8    Channel              = (commandPtr[3] & 0x0F);
   UINT8    GYRO_DataFormat      = commandPtr[5];
   UINT8    GYRO_TextFormat      = commandPtr[6];
   UINT8    *pGYRO_Data          = (UINT8*)readLong(commandPtr + 7);
   UINT32   GYRO_MaxCount        = readLong(commandPtr + 11);
   UINT32   GYRO_MeasureInterval = readLong(commandPtr + 15);
   UINT32   GYRO_Count           = 0;
   UINT32   Error;
   UINT32   time;

   // Speicher komplett loeschen
   memset ((UINT8*)eRAM_Start, 0x0A, RAM_SIZE);

   // max. Messzeit 1 Sekunde
   //if (GYRO_MeasureInterval > 1000)   GYRO_MeasureInterval = 1000;
   // Wertebereich Zeiger auf ext. Ram Untergrenze eingrenzen

   if (pGYRO_Data < ((UINT8*)eRAM_Start))         pGYRO_Data = (UINT8*)eRAM_Start;

   // Wertebereich Zeiger auf ext. Ram Obergrenze eingrenzen
   if (pGYRO_Data > (UINT8*)(eRAM_Start+(RAM_SIZE-0x20))) pGYRO_Data = (UINT8*)(eRAM_Start+(RAM_SIZE-0x20));

   // ------------------------------------------------------------

   jprintf("%d datapoints with %d ms interval sampled",
   GYRO_MaxCount,GYRO_MeasureInterval);

   if (GYRO_DataFormat == 1)
   {  // Header schreiben
      Ram_Print(&(UINT32)pGYRO_Data,"%d datapoints with %d ms interval sampled\n",
      GYRO_MaxCount,GYRO_MeasureInterval);
   }

   // ------------------------------------------------------------------------------
   // aktuelle Zeit in ms festhalten
   time = get_ms();

   jprintf("used SPI-Port = %d", Channel);

   // Messschleife
   while (GYRO_Count <= GYRO_MaxCount)
   {
      while ( get_ms() < (time+GYRO_MeasureInterval) )
      {
         ;
      }


      // aktuelle Zeit erneut festhalten
      time = get_ms();

      // auf Data ready Int vom Gyro warten
      while (SPI0_EN_WAIT)
      {
         // sicherheitshalber Timeout
         if ( getms() >= (time+GYRO_MeasureInterval) )
         {
            jprintf ("Error : GYRO doesn´t respond within measurement interval");
            break;
         }
      }

      // Messung durchfuehren  --> ueber SPI schreiben/lesen

      // ---------------------------------------------------------
      if (Channel == 0x01)  // AIS328DQ
      {
         for (i = 0; i < sizeof(GYRO_TX_Data); i++)
         {
            if (!SpiWrite_String_CSIH(Channel, sizeof(GYRO_TX_Data), GYRO_TX_Data, GYRO_RX_Data))
               {Error =  ERROR_SER | SER_ERR_TIMEOUT; break;}

            resultPtr[0] += sizeof(GYRO_TX_Data);
         }

         // Werte mit Vorzeichen abholen
         GYRO_Out_X = ((INT16)GYRO_RX_Data[3] << 8) + (INT16)GYRO_RX_Data[2];
         GYRO_Out_Y = ((INT16)GYRO_RX_Data[5] << 8) + (INT16)GYRO_RX_Data[4];
         GYRO_Out_Z = ((INT16)GYRO_RX_Data[7] << 8) + (INT16)GYRO_RX_Data[6];

         // auf 12 Bit reduzieren
         GYRO_Out_X = GYRO_Out_X / 16;
         GYRO_Out_Y = GYRO_Out_Y / 16;
         GYRO_Out_Z = GYRO_Out_Z / 16;
      }

      // ---------------------------------------------------------
      if (Channel == 0x04)  // CRM100/CRM120
      {

         if (!SpiWrite_String_CSIH(Channel, sizeof(GYRO_TX_Data), CRM100_TX_Data, CRM100_RX_Data))
            {Error =  ERROR_SER | SER_ERR_TIMEOUT; break;}

         resultPtr[0] += sizeof(GYRO_TX_Data);

         // Werte mit Vorzeichen abholen
         GYRO_Out_Z = (INT16)CRM100_RX_Data[0];    // Status
         GYRO_Out_X = ((INT16)CRM100_RX_Data[1] << 8) + (INT16)CRM100_RX_Data[2];
         GYRO_Out_Y = ((INT16)CRM100_RX_Data[3] << 8) + (INT16)CRM100_RX_Data[4];
      }
      // ---------------------------------------------------------

      if (GYRO_TextFormat == 1)
      {
         jprintf ("measurement # %5d data 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
            GYRO_Count, GYRO_RX_Data[1], GYRO_RX_Data[2], GYRO_RX_Data[3], GYRO_RX_Data[4],
            GYRO_RX_Data[5],GYRO_RX_Data[6],GYRO_RX_Data[7]);
      }

      if (GYRO_TextFormat == 2)
      {
         if (Channel == 0x08)  // EWTS8N
         {
            jprintf ("measurement # %5d X : %5d Y : %5d Z : %5d",
            GYRO_Count, (unsigned short)GYRO_Out_X, (unsigned short)GYRO_Out_Y, (unsigned short)GYRO_Out_Z);
         }
         else
         {
            jprintf ("measurement # %5d X : %5d Y : %5d Z : %5d",
                     GYRO_Count, GYRO_Out_X, GYRO_Out_Y, GYRO_Out_Z);
         }
      }

      // Werte binaer ins Ram schreiben
      if (GYRO_DataFormat == 0)
      {  // Status zuerst schreiben, Werte als 16 Bit Int-Zahlen in big endian
         *pGYRO_Data++ =  GYRO_RX_Data[1];
         *pGYRO_Data++ =  (UINT8)((GYRO_Out_X >>8) & 0x00FF);
         *pGYRO_Data++ =  (UINT8)(GYRO_Out_X & 0x00FF);
         *pGYRO_Data++ =  (UINT8)((GYRO_Out_Y >>8) & 0x00FF);
         *pGYRO_Data++ =  (UINT8)(GYRO_Out_Y & 0x00FF);
         *pGYRO_Data++ =  (UINT8)((GYRO_Out_Z >>8) & 0x00FF);
         *pGYRO_Data++ =  (UINT8)(GYRO_Out_Z & 0x00FF);
      }

      // Werte als Ascii ins Ram schreiben
      if (GYRO_DataFormat == 1)
      {
         if (Channel == 0x08)  // EWTS8N
         {
            // Status zuerst schreiben Werte als Ascii
            Ram_Print(&(UINT32)pGYRO_Data,"Status 0x%02X,", GYRO_RX_Data[1]);
            Ram_Print(&(UINT32)pGYRO_Data,"%d,", (unsigned short)GYRO_Out_X);
            Ram_Print(&(UINT32)pGYRO_Data,"%d,", (unsigned short)GYRO_Out_Y);
            Ram_Print(&(UINT32)pGYRO_Data,"%d\n",(unsigned short)GYRO_Out_Z);
         }
         else
         {
            // Status zuerst schreiben Werte als Ascii
            Ram_Print(&(UINT32)pGYRO_Data,"Status 0x%02X,", GYRO_RX_Data[1]);
            Ram_Print(&(UINT32)pGYRO_Data,"%d,",GYRO_Out_X);
            Ram_Print(&(UINT32)pGYRO_Data,"%d,",GYRO_Out_Y);
            Ram_Print(&(UINT32)pGYRO_Data,"%d\n",GYRO_Out_Z);
         }
      }

      GYRO_Count++;

      // Wertebereich Zeiger auf ext. Ram Obergrenze eingrenzen
      if ( pGYRO_Data > (UINT8*)(eRAM_Start+(RAM_SIZE-0x20)) )
      {
         jprintf ("Error : Ram overflow, measurement aborted");
         break;
      }
   }  // Ende Messschleife

   // Messung durchfuehren
   jprintf ("measurement finished");
   return(Error);
}




#endif // (CP_CSIH0 || CP_CSIH4)






