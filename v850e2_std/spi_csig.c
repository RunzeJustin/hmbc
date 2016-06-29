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
#if (CP_CSIG0 || CP_CSIG4)


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


//PMCn is port mode when equal to 0, the data of Pn_m is output to pin Pn_m
#define SPI0_PORT_UNCONFIG    PMC3 &= ~(BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
#define SPI4_PORT_UNCONFIG    PMC25 &= ~(BIT3 | BIT4 | BIT5 | BIT6 | BIT7);


struct SPI_CSIG            // adapted to CISG
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
   // -------------------------------------
   UINT8    dummy11[0xFE0];// make up to 0x1000-1

   UINT8    bctl0;         // 0x1000
   UINT8    dummy12;       // 0x1001
   UINT8    dummy13;       // 0x1002
   UINT8    dummy14;       // 0x1003

   UINT32   tx0w;          // 0x1004

   UINT16   tx0h;          // 0x1008
   UINT16   dummy15;       // 0x100A

   UINT16   rx0;           // 0x100C
   UINT16   dummy16;       // 0x100E

   UINT32   cfg0;          // 0x1010
};



//*--------------------------------------------------------------------------*
//* SPI-Message Transceiver possible States                                  *
//*--------------------------------------------------------------------------*

#define SPIMSG_MASTER_DISABLED             0
#define SPIMSG_MASTER_ENABLED   	 		 1
#define SPIMSG_MASTER_TRANSMITTING        2
#define SPIMSG_MASTER_RECEIVED             3
#define SPIMSG_MASTER_ERRORSTATE          4


// ------------------------------------------------------------------------------
//  Local defines
// ------------------------------------------------------------------------------
//#define SER_CH_NUM      15     // höchste Kanalnummer (Kanaele 0 + 4 + 7 CSIG)    // 4 nicht verwendet beim M033
#define MAX_BURST_TRANSFER_SIZE  256


// Prototypes
BOOL     SpiWrite_String_CSIG(UINT8 Channel, UINT8 Number, UINT8* SendData, UINT8* ReceiveData);
static   UINT32 SPI_CSIG_MSG_Master_Init (UINT8 Channel);
UINT32   CalculateSpiSettings(UINT8 Channel, UINT32 Baudrate, UINT8 Mode);
UINT32   GyroTest(void);
static   uint32_t spi_extopen (uint8_t channel, uint8_t len, uint8_t * cmd);

// globale Varibeln
static   UINT32   msgRegisterHandle_Ch0;     // um Schnittstelle beim MBC Handler registrieren Ch0
static   UINT32   msgRegisterHandle_Ch4;     // um Schnittstelle beim MBC Handler registrieren Ch7

// -------------------------------------------
// uebertrage maximal 16 Daten-Bytes // momentan nur CSIG 4 1 Byte zum Interrupt-Anstossen
volatile UINT8    CSIG_TransmitCounter_Ch0 = 0;
volatile UINT8    CSIG_TransmitCounter_Ch4 = 0;
volatile UINT8    CSIG_TransmitIndex_Ch0 = 0;
volatile UINT8    CSIG_TransmitIndex_Ch4 = 0;
// CSIG-Transmit Interrupt Buffer
volatile UINT8    CSIG_TransmitBuffer_Ch0[MAX_BURST_TRANSFER_SIZE];
volatile UINT8    CSIG_TransmitBuffer_Ch4[MAX_BURST_TRANSFER_SIZE];
// CSIG-Receive Interrupt Buffer
UINT8    CSIG_ReceiveBuffer_Ch0[MAX_BURST_TRANSFER_SIZE];
UINT8    CSIG_ReceiveBuffer_Ch4[MAX_BURST_TRANSFER_SIZE];
// -------------------------------------------
volatile UINT8    CSIG_ReceiveCounter_Ch0 = 0;
volatile UINT8    CSIG_ReceiveCounter_Ch4 = 0;



// -------------------------------------------
volatile UINT16    CSIG_IPC_Ch_ON = 0;          // if IPC for Channel must actively bits are set to 1

volatile UINT16    CSIG_CS_Ch_ON  = 0;          // Here is noted when CS line of Channels is Used



static   BOOL  CsigSpiOpen[16];     // Interface from 0 to 15

//CSIG interface only contains  0 + 4
UINT32         csig_base[5] = {0xFF700000, 0x00000000, 0x00000000, 0x00000000, 0xFF740000};


/* *************************************************************************** */
void SetChipSelect(UINT8 data)   // Channel as a number not as a bit mask
{                                // only works for Channel 0 ... 15
   UINT8    Channel;


   //jprintf("CS-Data: 0x%X    ON: 0x%X", data, CSIG_CS_Ch_ON);


   Channel = (data & 0x0F);      // nur unteren 4 Bits   0 bis 15 (0x0F)

   // Set all chip select lines on inactive
   if ( (data & ALL_CS_INAKTIV) == ALL_CS_INAKTIV)
   {
      // zwei Leitungen fuer Channel 0
      SETPORT_1(3,  4);     // inaktiv high Port 3.2     MCU_INIC_SPI_CS_N    (M033)
      SETPORT_1(3,  3);     // inaktiv high Port 0.15    MCU_E2P_SPI_CS_N     (M033)

      // zwei Leitungen fuer Channel 4
      SETPORT_1(25,  6);    // inaktiv high Port 11.3    MCU_DSP_SPI_CS_N     (M033)
      SETPORT_1(25,  7);    // inaktiv high Port  1.1    MCU_FPGA_SPI_CS_N    (M033)

//      jprintf("CS channel: %d; data is %d", Channel, data);   

      return;
   }

   // If there's no CS switch operation
   if ((CSIG_CS_Ch_ON & (1<<Channel)) == 0)
      return;

   if ( (data & CS_LEVEL_INAKTIV) == CS_LEVEL_INAKTIV)
   {
      // HW Channel 0 (Channel 0 + virtual Channel 8)
      if (Channel ==  0) SETPORT_1(3, 4);           //   Ch0: Port  3.2 --> high/inaktiv MCU_INIC_SPI_CS_N
      if (Channel ==  8) SETPORT_1(3, 3);           //  vCh8: Port 0.15 --> high/inkativ MCU_E2P_SPI_CS_N

      // HW Channel 4 (Channel 4 + virtual Channel 12)
      if (Channel ==  4) SETPORT_1(25,6);           //   Ch7: Port 11.3 --> high/inaktiv MCU_DSP_SPI_CS_N
      if (Channel == 12) SETPORT_1(25,7);           // vCh15: Port  1.1 --> high/inaktiv MCU_FPGA_SPI_CS_N

//      jprintf("IAC channel is: %d;", Channel);

   }
   else  //   CS active/low
   {
      //jprintf("setze Channel: %d aktiv/low", Channel);

      // HW Channel 0 (Channel 0 + virtual Channel 8)
      if (Channel == 0)  SETPORT_0(3, 4);           //   Ch0: Port  3.2 --> low/aktiv  MCU_INIC_SPI_CS_N
      if (Channel == 8)  SETPORT_0(3, 3);           //  vCh8: Port 0.15 --> low/aktiv  MCU_E2P_SPI_CS_N

      // HW Channel 4 (Channel 4 + virtual Channel 12)
      if (Channel ==  4) SETPORT_0(25,6);           //   Ch4: Port 25.6 --> low/aktiv  MCU_DSP_SPI_CS_N
      if (Channel == 12) SETPORT_0(25,7);           // vCh12: Port 25.7 --> low/aktiv  MCU_FPGA_SPI_CS_N

//      jprintf("AC channel is: %d;", Channel);

   }

   return;
}
// ------------------------------------------------------------------------------------------------
BOOL CsigClose(UINT8 Channel)
{
   switch (Channel)
   {
      case  0:
      //case  8:     // wuerde bei einem close auch die SPI-Verbindung zum INIC trennen, da selber HW-SPI-Channel
      {
         //jprintf("Transmit0-Int-Cnt: %d", CSIG_TransmitIndex_Ch0);
         //jprintf("Receive0--Int-Cnt: %d", CSIG_ReceiveCounter_Ch0);
         SPI0_PORT_UNCONFIG
         CSIG0CTL0 = 0;                      // disable TX+RX+Power OFF
         break;
      }

      case  4:
      case  12:
      {
         //jprintf("Transmit7-Int-Cnt: %d", CSIG_TransmitIndex_Ch7);
         //jprintf("Receive7--Int-Cnt: %d", CSIG_ReceiveCounter_Ch7);
         SPI4_PORT_UNCONFIG
         CSIG4CTL0 = 0;                   // disable TX+RX+Power OFF
         break;
      }
   }
   CSIG_IPC_Ch_ON &= ~(1<<Channel);    // deletes corresponding channel bit
   CsigSpiOpen[Channel] = FALSE;
   return(TRUE);
}


BOOL CsigOpen(UINT8 Channel, UINT32 BaudRate, UINT8 Proto)
{
   UINT32   Error;

   // If already open
   if (CsigSpiOpen[Channel] == TRUE) return(FALSE);


   Error = CalculateSpiSettings(Channel, BaudRate, Proto);

   if (Error == 0)
   {
      CsigSpiOpen[Channel] = TRUE;
      return(TRUE);
   }
   else            
      return(FALSE);
}



//*******************************************************************************
//* Function     :  SerielSpi_CMD                                               *
//*-----------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested      *
//*******************************************************************************
extern volatile UINT32 unused_isr_cnt;
void SerielCSIG_CMD (void)
{
   UINT32   Error;
   UINT32   BaudRate;
   UINT8    Channel;                // physisch vorhandener Kanal
   UINT8    Proto;
   UINT8    cmd = commandPtr[2];


   // Grobtest auf Kommandosyntax
   //                   Size                    Kommando
   if (((commandPtr[0] ==  9) && (cmd == 0x00)) ||     // Open is permissible with 9
       ((commandPtr[0] ==  4) && (cmd == 0x01)) ||     // Close ist mit  4 zulaessig
       ((commandPtr[0]  >  4) && (cmd == 0x02)) ||     // write is permissible with > 4
       ((commandPtr[0]  >  5) && (cmd == 0x03)) ||     // read  ist mit >5 zulaessig
       ((commandPtr[0]  >  4) && (cmd == 0xE0)) ||     // extended open
       ((commandPtr[0]  >  8) && (cmd == 0xE2)) ||
       ((commandPtr[0]  >  2) && (cmd == 0xFF))  )
   {}
   else
   {
      writeLong(resultPtr+4,  ERROR_SER | ERROR_COMMAND);
      return;
   }

   Error = NO_ERROR;

   // channel calculate
   Channel    = commandPtr[3] - CSIG_CH_OFFSET;
   BaudRate   = readLong(commandPtr+4);
   Proto      = commandPtr[8];

   // checke Channel-Number
   if ((Channel != 0) && (Channel != 4)  && (Channel != 8)  && (Channel != 12))
   {
      //Send interpreter Command error by default      
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
         if (CsigOpen(Channel, BaudRate, Proto) != TRUE)
            Error = (ERROR_SER | SER_ERR_OPEN);
         else Error = NO_ERROR;
         break;
      }

      //*-----------------------------------------------------------------------*
      //*	Close Serial Interface                                              *
      //*-----------------------------------------------------------------------*
      case 0x01:
      {
         CsigClose(Channel);
         break;
      }

      //*-----------------------------------------------------------------------*
      //*	Write to Serial Interface                                           *
      //*-----------------------------------------------------------------------*
      case 0x02:
      {
         UINT8 Number = (commandPtr[0] - 4);     // Number of SPI data


         if (Number < MAX_BURST_TRANSFER_SIZE)
         {
            if (CsigSpiOpen[Channel] == TRUE)
            {
               SpiWrite_String_CSIG(Channel, Number, (commandPtr+4), (resultPtr+8));
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


         if (Number < MAX_BURST_TRANSFER_SIZE)
         {
            if (CsigSpiOpen[Channel] == TRUE)
            {  //                                    dummy buffer
               SpiWrite_String_CSIG(Channel, Number, (commandPtr+5), (resultPtr+8));
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

      case 0xE0:
      {
         Error = spi_extopen ((commandPtr[3]-CSIG_CH_OFFSET),
                               commandPtr[0]-4, &commandPtr[4]);
         break;
      }
      
      case 0xFF:
      {
         resultPtr[0] = 12;
         writeLong(resultPtr+4, NO_ERROR);
         writeLong(resultPtr+8, unused_isr_cnt);
         jprintf("Unused ISR Counter: 0x%08X", unused_isr_cnt);

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
// calculate settings + CSIG-Init Mode is 100001(0x21 = bit0 & bit5 been set) for HW Detection
UINT32 CalculateSpiSettings(UINT8 Channel, UINT32 Baudrate, UINT8 Mode)
{
   volatile struct   SPI_CSIG   *csig;
   UINT32            TestClock, Differenz, ModulClock;
   UINT32            OldDifferenz;
   UINT16            cnt;
   UINT16            RegValue;
   UINT32            CSIGnCKR;
   UINT32            CSIGnDAP;
   UINT8             HWChannel;

//   printf("Here we go for SPI setting up\n");

   if (Channel > 4) HWChannel = Channel-8;
   else             HWChannel = Channel;

   // adapted to HW M033 / 70F3544
   if (HWChannel == 0)    ModulClock = GetDomainClock(CLK_DOMAIN_108);  //48,000,000
   if (HWChannel == 4)    ModulClock = GetDomainClock(CLK_DOMAIN_011);  //48,000,000

   OldDifferenz = ModulClock;
   //jprintf("ModulClock: %d", ModulClock);

   // start from ungeueltige / unimplemented protocol settings
   if ( ((Mode & BIT6) == BIT6) ||     // currently only 3 - Line or 4 - Line with CS permissible
        ((Mode & BIT4) == BIT4) ||     // currently only master mode permissible
        ((Mode & BIT2) == BIT2)  )     // currently only 8 -bit mode permissible
   {
      return(ERROR_SER | SER_ERR_PROTOCOL);
   }

   if ((Mode & BIT5) == BIT5)
   {
      CSIG_CS_Ch_ON |=  (1<<Channel);  // Set Chip-select bit
   }
   else
   {
      CSIG_CS_Ch_ON &= ~(1<<Channel);  // Clear Chip-select bit
   }

   SetChipSelect(ALL_CS_INAKTIV);      // all CS lines inaktiv (high)

   // alternative port-configuration for SPI-support
   switch (HWChannel)
   {
      case 0:  // CSIG0 Master INIC/E2P
      {
         // port-pin-configuration 
         // CSIG0SC @ P3_5    CLK                              // the module controls the I/O-line
         //PMCn = 1 means alternative mode, PFC=1,PFCE=1,PM=0 means alt mode 4, PIPC mean IO DIRECTION IS CONTORLLED BY ALT FUNCTION
         PMC3 |= BIT5; PFC3 |= BIT5; PFCE3 |= BIT5; PM3 &= ~BIT5; PIPC3 |= BIT5;

         // CSIG0S0 @ P3_6  MOSI//                                    // the module controls the I/O-line
         // Cfg is same as P3_5, alternative mode, direction is depends on mode
         PMC3 |= BIT6; PFC3 |= BIT6; PFCE3 |= BIT6; PM3 &= ~BIT6; PIPC3 |= BIT6;

         // CSIG0SI @ P3_7  MISO//                                    // Filter-Setting
         // Set to alternative input mode, and enable input filter
         PMC3 |= BIT7; PFC3 |= BIT7; PFCE3 |= BIT7; PM3 |= BIT7;  FCLA24CTL2=0x80;  // angepasst an CSIG0SI (M033)

         break;
      }

      case 4:  // CSIG4 Master DSP/FPGA
      {
         // port-pin-configuration
         // CSIG4SC @ P25_5    CLK                              // the module controls the I/O-line
         PMC25 |= BIT5; PFC25 &= ~BIT5; PFCE25 |= BIT5; PM25 &= ~BIT5; PIPC25 |= BIT5;

         // CSIG4S0 @ P25_4 MOSI  //                                    // the module controls the I/O-line
         PMC25 |= BIT4; PFC25 &= ~BIT4; PFCE25 |= BIT4; PM25 &= ~BIT4; PIPC25 |= BIT4;

         // CSIG4SI @ P25_3  MISO//                                    // Filter-Setting
         PMC25 |= BIT3; PFC25 &= ~BIT3; PFCE25 |= BIT3; PM25 |= BIT3; FCLA7CTL3=0x80;

         break;
      }

      default:
      {
         jprintf("CSIG Channel Not Existed!");
            return(ERROR_SER | SER_ERR_CHANNEL);
      }
   }

   // calculating Settings for communication clock
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
      // Register
      case 0x00:
      {
         CSIGnDAP = 0;
         CSIGnCKR = BIT17;
         break;
      }

      case 0x01:
      {
         CSIGnDAP = BIT16;
         CSIGnCKR = BIT17;
         break;
      }

      case 0x02:
      {
         CSIGnDAP = 0;           // CSIGnCFG0 register
         CSIGnCKR = 0;           // CSIGnCTL1 register
         break;
      }

      case 0x03:
      {
         CSIGnDAP = BIT16;
         CSIGnCKR = 0;
         break;
      }
   }

    // ab hier Zugriff ueber Pointer auf Registersatz
   csig = (volatile struct SPI_CSIG*)(csig_base[HWChannel]);

   // channel configuration                        - only master
   csig->ctl0   = 0x01;        // must be
   csig->ctl0  |= 0x80;
   csig->ctl1  = (0x00010000 | CSIGnCKR); // interrupt generation when free for next data | SCK initial level selection
   csig->ctl2  = RegValue;                // master-mode
   csig->cfg0  = (0x08000000 | CSIGnDAP); // 8 bit data length | Data-Phase-Selection
   csig->ctl0  = 0xE1;                    // enable CSIG, TX+RX+PWR


   // with interrupt-handling
   if (HWChannel == 0)
   {
      ICCSIG0IR = 0x0000;     // enable receive interrupt
      ICCSIG0IC = 0x0000;     // enable tranmit interrupt
   }

   if (HWChannel == 4)
   {
      ICCSIG4IR = 0x0000;     // enable receive interrupt
      ICCSIG4IC = 0x0000;     // enable tranmit interrupt
   }

//   jprintf("CSIG%d baudrate: %d, RegCTL0/1 been set to %X, %X", Channel, ModulClock / (2*RegValue), csig->ctl0, csig->ctl1 );

   return(0);
}


// ------------------------------------------------------------------------------
BOOL SpiWrite_String_CSIG(UINT8 Channel, UINT8 Number, UINT8* SendData, UINT8* ReceiveData)
{
   UINT8    *pData;
   UINT8    cnt;
   UINT32   time1;


   pData = SendData;


   if ( (Channel == 0) || (Channel == 8) )
   {
      CSIG_TransmitCounter_Ch0 = Number;
      CSIG_ReceiveCounter_Ch0  = 0;


      // kopiert Daten in den Interrupt-Transmitbuffer um
      for (cnt = 0; cnt < Number; cnt++)
      {
         CSIG_TransmitBuffer_Ch0[cnt] = *pData;
         //jprintf("Data: 0x%2X", CSIG_TransmitBuffer_Ch0[cnt]);
         pData++;  // erhoehe DataPointer
      }

      if (Channel == 0) SetChipSelect(0x00 | CS_LEVEL_AKTIV);
      if (Channel == 8) SetChipSelect(0x08 | CS_LEVEL_AKTIV);

      CSIG_TransmitCounter_Ch0--;                     // 1 Byte wird ohne Interrupt uebertragen
      CSIG_TransmitIndex_Ch0 = 1;                     // Buffer-Index, Startwert fuer ISR

      CSIG0TX0H = CSIG_TransmitBuffer_Ch0[0];         // stumble interrupt driven sending to

      time1 = timGetTime();
      do
      {
         if (timGetTime() > (time1+2000))             // 2000ms time
         {
            jprintf("Ch0 TimeOut  expected: %d receive: %d", Number, CSIG_ReceiveCounter_Ch0);
            return(FALSE);
         }
      }
      while (CSIG_ReceiveCounter_Ch0 != Number);      // warte bis alle Daten empfangen wurden
      if (Channel == 0) SetChipSelect(0x00 | CS_LEVEL_INAKTIV);         // only channel 0 inaktiv
      if (Channel == 8) SetChipSelect(0x08 | CS_LEVEL_INAKTIV);         // only channel 8 inaktiv

      // Copy of Interruptbuffers the over passed Buffer Address ( ReceiveData )
      {
         if (CSIG_ReceiveCounter_Ch0 < sizeof (CSIG_ReceiveBuffer_Ch0))
            memcpy( ReceiveData, CSIG_ReceiveBuffer_Ch0, CSIG_ReceiveCounter_Ch0);
      }
   }

   if ( (Channel == 4) || (Channel == 12))
   {
      CSIG_TransmitCounter_Ch4 = Number;
      CSIG_ReceiveCounter_Ch4  = 0;


      // copied data to the interrupt Transmitbuffer
      for (cnt = 0; cnt < Number; cnt++)
      {
         CSIG_TransmitBuffer_Ch4[cnt] = *pData;
         pData++;  // Increment Data Pointer
      }

//      jprintf("Send push Channel %d \n", Channel);
      
      if (Channel == 4)  SetChipSelect(0x04 | CS_LEVEL_AKTIV);           // CS Channel 4  aktiv
      if (Channel == 12) SetChipSelect(0x0C | CS_LEVEL_AKTIV);           // CS Channel 12 aktiv

      CSIG_TransmitCounter_Ch4--;                     // 1 byte to retransmit without interrupt
      CSIG_TransmitIndex_Ch4 = 1;                     // Buffer index starting value for ISR

      CSIG4TX0H = CSIG_TransmitBuffer_Ch4[0];         // push interruptge exaggerated Send To

      time1 = timGetTime();
      do
      {
         if (timGetTime() > (time1+2000))             // 2000ms Time
         {
            jprintf("expected: %d     receive: %d", Number, CSIG_ReceiveCounter_Ch4);
            return(FALSE);
         }
      }
      while (CSIG_ReceiveCounter_Ch4 != Number);      // Waiting until all data is received
      if (Channel == 4)  SetChipSelect(0x04 | CS_LEVEL_INAKTIV);         // only channel 7  inaktiv
      if (Channel == 12) SetChipSelect(0x0C | CS_LEVEL_INAKTIV);         // only channel 15 inaktiv

      //Copy of Interruptbuffers the over passed Buffer Address ( ReceiveData )
      {
         if (CSIG_ReceiveCounter_Ch4 < sizeof (CSIG_ReceiveBuffer_Ch4))
            memcpy( ReceiveData, CSIG_ReceiveBuffer_Ch4, CSIG_ReceiveCounter_Ch4);
      }
   }

   return(TRUE);
}


// -------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Erweiterungen um SPI Treiber fuer MBC Kommunikation nutzen zu koennen
// Typ der verwendeten SPI-Schnittstelle: CSIB
#define COMM_BURST_SIZE  1             // bei einer CSIB ist mehr nicht moeglich
static unsigned   lock_CH4 = 0;

#define CSIG_SPI_CH4_DELAY_S    0 // 100      // must be adapted (delay-time between CS aktiv and transfer starts)
#define CSIG_SPI_CH4_DELAY_E    0 // 100      // must be adpated (delay-time between transfer-ends and CS inaktiv)

#define CSIG_SPI4_CS_IPC_ACTIVATE    {SETPORT_0(25, 6);}  // is adapted to HW M034 , port configuration ( port list ) is set to OD High
#define CSIG_SPI4_CS_IPC_DEACTIVATE  {SETPORT_1(25, 6);}  //



UINT8 ZeroBuffer[4] = {0,0,0,0};
// ------------------------------------------------------------------------------
void Read16byte(UINT8 Channel)
{
   UINT8 cnt;
   UINT8 rxBuffer[4];


   for (cnt = 0; cnt < 4; cnt++)          // call multiple of 4 from
   {
         CSIG_SPI4_CS_IPC_ACTIVATE;
         DelayLoop(CSIG_SPI_CH4_DELAY_S);
         SpiWrite_String_CSIG(4, 4, ZeroBuffer, rxBuffer);
         DelayLoop(CSIG_SPI_CH4_DELAY_E);
         CSIG_SPI4_CS_IPC_DEACTIVATE;
   }
}


// ------------------------------------------------------------------------------
void SPI_CSIG_Receive_CH4(unsigned int Channel)
{
   static BOOL CntFlag = FALSE;
   BOOL PortIn;

   if (lock_CH4)   return;

   PortIn = INPORT_BOOL(10, 10);

//   printf("ENTRY REC CH4\n");

   // whether SHARC would like to be read
   if ((!PortIn) || CntFlag)
   {
      Read16byte(Channel);

      PortIn = INPORT_BOOL(10, 10);

      // not finished yet
      if (!PortIn)
      {
         CntFlag = TRUE;      // sets read operation continues at the next call
      }
      else                    // done
      {
         CntFlag = FALSE;
         // nevertheless read a few bytes to free SHARC Transmit Buffer
         Read16byte(Channel);
      }
   }
}


// ------------------------------------------------------------------------------
BOOL SPI_CSIG_Send_CH4(unsigned Channel, UINT8 *volatile Buffer, unsigned len)
{
   UINT16   cnt = 0;
   UINT8    rest = (len % 4);   // Rest 0 to 3
   UINT8    rxBuffer[4];
   UINT8    TxData[4] = {0,0,0,0};
   UINT8    len4      = (len >> 2);



   lock_CH4 = 1;

   for (cnt = 0; cnt < len4; cnt++)
   {
      CSIG_SPI4_CS_IPC_ACTIVATE;
      DelayLoop(CSIG_SPI_CH4_DELAY_S);

      SpiWrite_String_CSIG(4, 4, (Buffer+cnt*4), rxBuffer);

      DelayLoop(CSIG_SPI_CH4_DELAY_E);
      CSIG_SPI4_CS_IPC_DEACTIVATE;
   }

   cnt = (cnt * 4);  // to arrive at correct position Buffer

   if (rest > 0)     // if there is a radical of the must will pay for
   {
      switch (rest)
      {
         case 1:
         {
            TxData[0] = Buffer[cnt];      // the rest of TxData [ ] remains 0
            break;
         }

         case 2:
         {
            TxData[0] = Buffer[cnt];      // the rest of TxData [ ] remains 0
            TxData[1] = Buffer[cnt+1];
            break;
         }

         case 3:
         {
            TxData[0] = Buffer[cnt];      // the rest of TxData [ ] remains 0
            TxData[1] = Buffer[cnt+1];
            TxData[2] = Buffer[cnt+2];
            break;
         }
      }

      CSIG_SPI4_CS_IPC_ACTIVATE;
      DelayLoop(CSIG_SPI_CH4_DELAY_S);
      SpiWrite_String_CSIG(4, 4, TxData, rxBuffer);
      DelayLoop(CSIG_SPI_CH4_DELAY_E);
      CSIG_SPI4_CS_IPC_DEACTIVATE;
   }

   lock_CH4 = 0;

   return(TRUE);  // back return only true if all data has been sent
}


// -------------------------------------------------------------------------------------------------
static UINT32 SPI_CSIG_MSG_Master_Init (UINT8 Channel)
{
   // previously oeffnen interface " normal "
   // HW channel
   if (Channel == 4)    // IPC for SHARC
   {
      msgRegisterHandle_Ch4 = msgRegister(SPI+Channel,
         "SPI", Channel, 0, SPI_CSIG_Receive_CH4, SPI_CSIG_Send_CH4, NULL);
      return (SPIMSG_NO_ERROR);     // 0x00000000
   }
   return(ERROR_SER | SPIMSG_INIT_ERROR);
}


// ------------------------------------------------------------------------------
UINT32 spi_extopen (UINT8 Channel, UINT8 len, UINT8 * cmd)
{
   UINT32   baudrate = 0;
   UINT8    protocol = 0;
   unsigned index    = 0;
   unsigned fifo     = 0xFF;


   while (1)
   {
      switch (cmd[index++])
      {
         case  0x00: // protocol
            protocol = cmd[index++];
         break;

         case  0x01: // baudrate;
            baudrate = readLong(&cmd[index]);
            index+=4;
         break;

         case  0x06: // add command channel
         {
            fifo = cmd[index++];
         }
         break;

         default:
            //jprintf("unknown tag %02x at tag position %d\n", cmd[index-1], index);
            return(0xFFFFFFFF);
      }
      if (index == len) break;   // all tags proceeded

      if (index > len)
      {
         //jprintf("length error at tag position %d\n", index);
         return(0xFFFFFFFE);
      }
   }

   // permissible only for Channel 1
   if (fifo == 0)
   {
      if (Channel == 4)   // IPC for SHARC
      {
         // close selected SPI
         CsigClose(4);

         if (CsigSpiOpen[4] == FALSE)
         {
            //jprintf("open channel: %d %d 0x%X", channel, baudrate, protocol);
            //CalculateSpiSettings(channel, baudrate, protocol);
            CsigOpen(Channel, baudrate, protocol);

            CSIG_IPC_Ch_ON |= BIT4; // Data are deflected in ISR
         }
      }

      // Hook now interface
      SPI_CSIG_MSG_Master_Init(Channel);

      return(0x00000000);
   }
   return(0xFFFFFFFD);

}

// -------------------------------------------------------------------------------------------------
// start ISR Routinen -----------------------------------------------------------
// Channel (0) transmit interrupt ISR -------------------------------------------
__interrupt void INTCSIG0IC()
{
   if (CSIG_TransmitCounter_Ch0 > 0)
   {
      CSIG0TX0H = CSIG_TransmitBuffer_Ch0[CSIG_TransmitIndex_Ch0];
      CSIG_TransmitCounter_Ch0--;
      CSIG_TransmitIndex_Ch0++;
   }
}


// Channel (0) receive interrupt ISR --------------------------------------------
__interrupt void INTCSIG0IR()
{
   UINT8 data;

   data = CSIG0RX0;

   if ( (CSIG_IPC_Ch_ON & BIT0) == BIT0)    // IPC Channel 0 is ON
   {
      msgReceive(msgRegisterHandle_Ch0, &data, 1);
   }
   else
   {
      // Receive-Interrupt Buffer Ch 0
      if (CSIG_ReceiveCounter_Ch0 < sizeof (CSIG_ReceiveBuffer_Ch0))
      CSIG_ReceiveBuffer_Ch0[CSIG_ReceiveCounter_Ch0] = data;
   }

   CSIG_ReceiveCounter_Ch0++;

}


// Master Channel (4) transmit interrupt ISR ------------------------------------
__interrupt void INTCSIG4IC()
{
   if (CSIG_TransmitCounter_Ch4 > 0)
   {
      CSIG4TX0H = CSIG_TransmitBuffer_Ch4[CSIG_TransmitIndex_Ch4];
      CSIG_TransmitCounter_Ch4--;
      CSIG_TransmitIndex_Ch4++;
   }
}


// Channel (4) receive interrupt ISR --------------------------------------------
__interrupt void INTCSIG4IR()
{
   UINT8 data;

   data = CSIG4RX0;

   if ( (CSIG_IPC_Ch_ON & BIT4) == BIT4)    // IPC Channel 4 ist ON
   {
      msgReceive(msgRegisterHandle_Ch4, &data, 1);
   }
   else
   {
      // Receive-Interrupt Buffer Ch 4
      if (CSIG_ReceiveCounter_Ch4 < sizeof (CSIG_ReceiveBuffer_Ch4))
      CSIG_ReceiveBuffer_Ch4[CSIG_ReceiveCounter_Ch4] = data;
   }

   CSIG_ReceiveCounter_Ch4++;
}

#endif // (CP_CSIG0 || CP_CSIG4 || CP_CSIG6) || CP_CSIG7))

// must always pure
volatile UINT32 unused_isr_cnt = 0;
#pragma ghs interrupt
void unused_isr(void)
{
   unused_isr_cnt++;
   return;
}






