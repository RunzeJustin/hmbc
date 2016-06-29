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
#include "global.h"                 // globale Datenstrukturen
#include "gateway.h"
#include "ints.h"						   // Interruptverwaltung
#include <string.h>          		   // für memcpy
#include "tool.h"
#include "ser.h"
#include "buffer.h"
#include "timer.h"
#include "hw_init.h"


//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*


#if (CP_CMDDEV_RS232 || CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)

//*--------------------------------------------------------------------------*
//* Ummappen der verschiedenen Clocks beim SJ3H fuer dieses File             *
//*--------------------------------------------------------------------------*

#define FRQ                40e6    	// Taktfrequenz in Hz


//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

static BOOL    SerOpen           (UINT8 Channel, UINT32 Baudrate, UINT8 Protocol);
static BOOL    SerClose          (UINT8 Channel);
       BOOL    SerWriteChar      (UINT8 Channel, UINT8 Data);
static void    SerSetRTS         (UINT8 Channel);
static void    SerClrRTS         (UINT8 Channel);

static BOOL    RS232_Send        (UINT8 Channel, UINT8 *Buffer, UINT32 len);

#if (CP_SER11)
static BOOL    SerWriteBreak     (UINT8 Channel);
static UINT16  SerReadBuf        (UINT8 Channel, UINT8 *Buffer);

static UINT8   LIN_Add_Parity    (UINT8 Identifier);
static UINT8   LIN_Calc_Checksum (UINT8 *Data,   UINT8 Anzahl);
static UINT32  LIN_Send_Header   (UINT8 Channel, UINT8 Identifier);
static UINT32  LIN_Send_Byte     (UINT8 Channel, UINT8 Data);
static UINT32  LIN_Send_Data     (UINT8 Channel, UINT8 *Data, UINT8 Anzahl);
static UINT32  LIN_Read_Data     (UINT8 Channel, UINT8 *Data, UINT8 Anzahl);

static UINT32  FRT_Send_Data     (UINT8 Channel, UINT8 *Data, UINT8 Anzahl);
#endif


//*--------------------------------------------------------------------------*
//* Local defines                                                            *
//*--------------------------------------------------------------------------*

#define  ASC_BUF_SIZE                  256            // nur gerade Werte
#define  ASC_BUF_SIZE_RTSLIMIT         32             // Grenzwert fuer Wegnahme von RTS

//*--------------------------------------------------------------------------*
//* Interrupt handling macros                                                *
//*--------------------------------------------------------------------------*

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
#if ((CPU_TYPE==CPU_NEC_V850E2_DF3554)&&(CP_SER11==1))   // L122
   // ICLMA11IR      Reception completion interrupt
   // ICLMA11IS      Status interrupt
   // MKLMA11IR
   #define  Ux11_RX_CONFIG_INTVECTOR    {ICLMA11IR = INIT_LMA11IR; ICLMA11IS = INIT_LMA11IS;}
   #define  Ux11_RX_INT_ENABLE          {MKLMA11IR = 0; MKLMA11IS = 0;}
   #define  Ux11_RX_INT_DISABLE         {MKLMA11IR = 1; MKLMA11IS = 1;}

   // Port P0_6 --> TX, Port P0_7--> RX
   //                                   RX --> Input,      RX Portfunktion,    TX Portfunktion
   #define  Ux11_PORT_UNCONFIG          {SETPORT_IN(0,7); SETPORT_PORT(0,7); SETPORT_PORT(0,6);}

   //                                                                       TX Output,        RX Input,         TX Alternative,   RX Alternative     Bit 7: filter bypass enabled for signal URTE11RX
   #define  Ux11_PORT_CONFIG            {PFC0 &= ~0x00C0; PFCE0 &= ~0x00C0; SETPORT_OUT(0,6); SETPORT_IN(0,7); SETPORT_ALT(0,6); SETPORT_ALT(0,7); FCLA7CTL1=0x80;}

   //                                   TX to 1          Output
   #define  Ux11_TX_PORT_SET            {SETPORT_1(0,6); SETPORT_OUT(0,6);}

   //                                   TX to 0          Output
   #define  Ux11_TX_PORT_CLR            {SETPORT_0(0,6); SETPORT_OUT(0,6);}
#endif



//*--------------------------------------------------------------------------*
//* register macros                                                          *
//*--------------------------------------------------------------------------*

// URTEn status register 1
#define URTEnBSF  (1 << 4)
#define URTEnDCE  (1 << 3)
#define URTEnPE   (1 << 2)
#define URTEnFE   (1 << 1)
#define URTEnOVE  (1 << 0)

// URTEn status register 0
#define URTEnSSBR (1 << 6)
#define URTEnSSBT (1 << 5)
#define URTEnSSR  (1 << 1)
#define URTEnSST  (1 << 0)

// URTEn control register 1
#define URTEnSLD        (1 << 1)
#define URTEnSLG        (1 << 2)
#define URTEnSLP(val)   (val << 6)
#define URTEnCLG        (1 << 8)
#define URTEnBLG(val)   (val << 12)
#define URTEnSLBM       (1 << 15)

//*-----------------------------------------------------------------------*
//* Type Definitions                                                      *
//*-----------------------------------------------------------------------*

#define SER_CH_NUM   11           // höchste Kanalnummer


// Prefix FRT : Statemaschine speziell fuer Frontkontroller
typedef enum  {   Statemachine_Off, CMD_No_Message,
                  FRT_No_Message, FRT_First_DLE_received, FRT_STX_received,
                  FRT_Data_received, FRT_DLE_received, FRT_ETX_received,
              }   tMessage;

typedef struct
{
   BOOL           opened,     // channel opened?
                  sending;    // is channel sending a the moment?
   tMessage       MsgStatus;  // fuer Kommandobetrieb
   UINT8          TimerNr,
                  Checksum;
   UINT16         Commandbyte_Index,
                  Commandbyte_Length;
   UINT32         frErr,      // number of frame errors
                  parErr,     // number of parity errors
                  ovrErr,     // number of overrun errors
                  brkErr;     // number of break errors

   UINT8          Protocol;                  // LIN protocol type
   UINT32         Sync_Delimiter_Time,       // fuer LIN-Unterstuetzung
                  InFrame_Response_Time,     // fuer LIN-Unterstuetzung
                  InterByte_Time,            // fuer LIN-Unterstuetzung
                  InFrame_Response_Timeout;  // fuer LIN-Unterstuetzung
   UINT8          ACK_Counter,               // fuer Frontkontrollerprotokoll
                  NAK_Counter,               // fuer Frontkontrollerprotokoll
                  MSG_Counter;               // fuer Frontkontrollerprotokoll
   TBuffer        *MsgBuf,                   // fuer Frontkontrollerprotokoll
                  *inBuf;                    // input buffer
   UINT32         CommIndex;
} TSerCon;

//*-----------------------------------------------------------------------*
//* Local Variables                                                       *
//*-----------------------------------------------------------------------*

// serial channel structures for each channel
static TSerCon serChs[SER_CH_NUM+1] =
{
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,TIMER_RS232_DEV_3, 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,TIMER_RS232_DEV_5, 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,0,                 0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,TIMER_RS232_DEV_10,0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
   FALSE,FALSE,Statemachine_Off,TIMER_RS232_DEV_11,0,0,0,0,0,0,0,SER_8N1,0,0,0,0,0,0,0,(TBuffer *)NULL,(TBuffer *)NULL,0,
};

//****************************************************************************
//* Function     :  SerielUart_CMD                                           *
//****************************************************************************
//* Description :                                                            *
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

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
void SerielUart_CMD (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Error;
UINT16   i;
UINT8    Anzahl;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Grobtest auf Kommandosyntax
   if (((commandPtr[0] == 10) && (commandPtr[2] == 0x00)) ||
       ((commandPtr[0] == 4)  && (commandPtr[2] == 0x01)) ||
                                (commandPtr[2] == 0x02)  ||
       ((commandPtr[0] == 5)  && (commandPtr[2] == 0x03)) ||
       ((commandPtr[0] == 4)  && (commandPtr[2] == 0x04)) ||
       ((commandPtr[0] >= 6)  && (commandPtr[0] <= 13) && (commandPtr[2] == 0x22)) ||
       ((commandPtr[0] == 6)  && (commandPtr[2] == 0x23)) ||
       ((commandPtr[0] >= 3)  && (commandPtr[2] == 0x80)))
   {}
   else
   {  writeLong(resultPtr+4, ERROR_SER | ERROR_COMMAND);
      return;
   }

   if (commandPtr[3] > SER_CH_NUM)
   {  // Interpreter Commandfehler defaultmässig schicken
      writeLong(resultPtr+4, ERROR_SER | SER_ERR_CHANNEL);
      return;
   }

   switch (commandPtr[2])
   {
//*--------------------------------------------------------------------------*
//*	Open Serial Interface                                                  *
//*--------------------------------------------------------------------------*
      case 0x00:  if (commandPtr[8] == SER_CMD_BIOS)
                  {   // Bioscontroll Kommandomode hier abweisen
                      Error = ERROR_SER | SER_ERR_PROTOCOL;
                      break;
                  }

                  if (SerOpen(commandPtr[3], readLong (commandPtr+4), commandPtr[8]))
                  {
                     // im Moderegister setzen
                     switch (commandPtr[3])
                     {
                        case 0 : setMode(MODE_SER0, 1);
                                 break;
                        case 1 : setMode(MODE_SER1, 1);
                                 break;
                        case 2 : setMode(MODE_SER2, 1);
                                 break;
                        default: break;
                     }
                  }
                  else
                     Error = ERROR_SER | SER_ERR_OPEN;
                  break;

//*--------------------------------------------------------------------------*
//*	Close Serial Interface                                                 *
//*--------------------------------------------------------------------------*

      case 0x01:  if (SerClose(commandPtr[3]))   // channel
                  {
                     // im Moderegister loeschen
                     switch (commandPtr[3])
                     {
                        case 0 : setMode(MODE_SER0, 0);
                                 break;
                        case 1 : setMode(MODE_SER1, 0);
                                 break;
                        case 2 : setMode(MODE_SER2, 0);
                                 break;
                        default: break;
                     }
                  }
                  else
                     Error = ERROR_SER | SER_ERR_CLOSE;
                  break;

//*--------------------------------------------------------------------------*
//*	Write to Serial Interface                                              *
//*--------------------------------------------------------------------------*

      case 0x02:  if (commandPtr[0] < 5) // no data to send
                  {
                     Error = ERROR_SER | ERROR_COMMAND;
                     break;
                  }

                  if (!(serChs[commandPtr[3]].Protocol & SER_BUS_ACTIVE))
                  {  // Basic
                  for (i = 4; i < commandPtr[0]; i++)
                  {
                     if (!SerWriteChar(commandPtr[3], commandPtr[i]))
                     {
                        Error =  ERROR_SER | SER_ERR_TIMEOUT;
                        break;
                     }
                  }
                  }
                  else
                     if (serChs[commandPtr[3]].Protocol == SER_FRT_AUDI)
                     {
                        Error = FRT_Send_Data(commandPtr[3], commandPtr+4, commandPtr[0]-4);
                     }

                  break;

//*--------------------------------------------------------------------------*
//*	Read from Serial Interface                                             *
//*--------------------------------------------------------------------------*

      case 0x03:  // noch nicht implementiert
                  Error = ERROR_SER | ERROR_COMMAND;
                  break;

      case 0x04:  if (!(serChs[commandPtr[3]].Protocol & SER_BUS_ACTIVE))
                  {  // Basic
                  resultPtr[0] = SerReadBuf(commandPtr[3], resultPtr+8) + 8;
                  }
                  else
                     if (serChs[commandPtr[3]].Protocol == SER_FRT_AUDI)
                     {  __DI();
                        // Test, ob Daten da
                        if(bufItems(serChs[commandPtr[3]].MsgBuf) >=5)
                        {  // zuerst die Errornummer abholen
                           Error = Get_Fifo_32(serChs[commandPtr[3]].MsgBuf);

                           //dann die Anzahl der Daten
                           bufGet(serChs[commandPtr[3]].MsgBuf, &Anzahl);

                           for (i = 0; i < Anzahl  ; i++)
                           {  // Datenbytes laden
                              bufGet(serChs[commandPtr[3]].MsgBuf, resultPtr+ i +9);
                           }

                           // Laenge der Antwort
                           resultPtr[0] = Anzahl + 9;

                           // Anzahl der Messages , die noch im Buffer sind, anzeigen
                           resultPtr[8] = --serChs[commandPtr[3]].MSG_Counter;
                        }
                        // set global interrupt enable flag
                        __EI();
                     }
                  break;

//*--------------------------------------------------------------------------*
//*	Write LIN-Message to Serial Interface                                  *
//*--------------------------------------------------------------------------*

      case 0x22:  if (!serChs[commandPtr[3]].opened)
                  {
                     Error = ERROR_SER | SER_ERR_CHANNEL;
                     break;
                  }

                  // vorerst nur fuer LIN 2.0
                  if (serChs[commandPtr[3]].Protocol != SER_LIN_20)
                  {
                     Error = ERROR_SER | SER_ERR_PROTOCOL;
                     break;
                  }

                  // Identifier 0x00..0x3F erlaubt, checken
                  if (commandPtr[4] > 0x3F)
                  {
                     Error = ERROR_SER | SER_ERR_PROTOCOL;
                     break;
                  }

                  // Write komplette Message
                  Error = LIN_Send_Header (commandPtr[3], commandPtr[4]);

                  // gesendeten Identifier mit Paritybits in die Antwort kopieren
                  resultPtr[8] = LIN_Add_Parity (commandPtr[4]);

                  // Alle Datenbytes in die Antwort kopieren
                  memcpy (resultPtr+9, commandPtr+5, commandPtr[0]-5);

                  // Laenge der Antwort
                  resultPtr[0] = commandPtr[0]+5;

                  // Bei LIN 2.0 geht der Identifier mit in die Checksumme ein
                  // Checksum in die Antwort kopieren
                  if (commandPtr[4] < 0x3C)
                  resultPtr[commandPtr[0]+4] = LIN_Calc_Checksum (resultPtr+8, commandPtr[0]-4);
                  else
                     // Identifier 0x3c to 0x3F entsprechend Spec LIN 2.0 mit klassischem Checksumme
                     resultPtr[commandPtr[0]+4] = LIN_Calc_Checksum (resultPtr+9, commandPtr[0]-5);

                  if (Error != NO_ERROR)
                     break;

                  // Daten und Checksum senden
                  Error = LIN_Send_Data (commandPtr[3], resultPtr+9, commandPtr[0]-4);

                  break;

//*--------------------------------------------------------------------------*
//*	Read LIN-Message from Serial Interface                                 *
//*--------------------------------------------------------------------------*

      case 0x23:  if (!serChs[commandPtr[3]].opened)
                  {
                     Error = ERROR_SER | SER_ERR_CHANNEL;
                     break;
                  }

                  // vorerst nur fuer LIN 2.0
                  if (serChs[commandPtr[3]].Protocol != SER_LIN_20)
                  {
                     Error = ERROR_SER | SER_ERR_PROTOCOL;
                     break;
                  }

                  // Identifier 0x00..0x3F erlaubt, checken
                  if (commandPtr[4] > 0x3F)
                  {
                     Error = ERROR_SER | SER_ERR_PROTOCOL;
                     break;
                  }

                  // 1- 8 Datenbytes lesbar, checken
                  if ((commandPtr[5] < 1) || (commandPtr[5] > 8))
                  {
                     Error = ERROR_SER | SER_ERR_PROTOCOL;
                     break;
                  }

                  // empfangene Daten mit 0xff vorbesetzen fuer den Fehlerfall
                  memset (resultPtr+9, 0xFF, 9);

                  Error = LIN_Send_Header (commandPtr[3], commandPtr[4]);

                  // gesendeten Identifier mit Paritybits in die Antwort kopieren
                  resultPtr[8] = LIN_Add_Parity (commandPtr[4]);

                  // Datenbytes und Checksum einlesen
                  Error = Error | LIN_Read_Data (commandPtr[3], resultPtr+9, commandPtr[5]+1);

                  // Laenge der Antwort
                  resultPtr[0] = commandPtr[5]+10;

                  // bei Fehler auf Checksumberechnung verzichten
                  if (Error != NO_ERROR)
                     break;

                  // Bei LIN 2.0 geht der Identifier mit in die Checksumme ein
                  // errechnete Checksum gegen empfangene Checksum testen
                  if (commandPtr[4] < 0x3C)
                  {
                  if (resultPtr[commandPtr[5]+9] != LIN_Calc_Checksum (resultPtr+8, commandPtr[5]+1))
                     Error =  ERROR_SER | SER_ERR_LIN_CHECKSUM;
                  }
                  else
                  {  // Identifier 0x3c to 0x3F entsprechend Spec LIN 2.0 mit klassischem Checksumme
                     if (resultPtr[commandPtr[5]+9] != LIN_Calc_Checksum (resultPtr+9, commandPtr[5]))
                        Error =  ERROR_SER | SER_ERR_LIN_CHECKSUM;
                  }
                  break;

//*--------------------------------------------------------------------------*
//*	Get Errors from Serial Interface                                       *
//*--------------------------------------------------------------------------*

      case 0x80:  // request all error counters
                  for (i=0; i<=SER_CH_NUM; i++ )
                  {
                     if (serChs[i].opened)
                        printf ("ASC%d Errors: Frame - %d, Parity - %d, Overrun - %d, Break - %d\n",
                                 i,
                                 serChs[i].frErr,
                                 serChs[i].parErr,
                                 serChs[i].ovrErr,
                                 serChs[i].brkErr);
                  }
                  break;

  }  // of switch(commandPtr[2])

//*--------------------------------------------------------------------------*
//* zum Schluss Ergebnisstring ausgeben                                      *
//*--------------------------------------------------------------------------*

   // Ergebnis senden
   writeLong(resultPtr+4, Error);
}
#endif

//****************************************************************************
//* Function     :   SerOpen                                                 *
//****************************************************************************
//* Description  :   Initialisierung einer ser. Schnittstelle                *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

static BOOL SerOpen (UINT8 Channel, UINT32 Baudrate, UINT8 Protocol)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Wert_Baudrate;
UINT8    Protocol_LowNibble;

UINT8 volatile Dummy;

UINT16   Control1,
         Control2;

UINT32   P_Clk;

#if (CP_DEBUG == 18)
   UINT32   Temp;
#endif

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (FALSE);

   // check wether the channel is already open
   if (serChs[Channel].opened)
      return (FALSE);

//*--------------------------------------------------------------------------*
//* Baudrate- und Kontrollregister errechnen                                 *
//*--------------------------------------------------------------------------*

   switch (Channel)
   {
      case 11:
         // eingestellten Clock erfragen
         P_Clk = GetDomainClock (CLK_DOMAIN_011);     // 48,000,000 hz
         break;

      default:
         return(FALSE);
   }

   // Vorteiler fix auf 1 setzen wegen maximaler Baudrategenauigkeit
   Wert_Baudrate = (UINT32)((P_Clk / (float)(Baudrate*2)) +0.5);

   // Test, ob Wertebereich ok
   if ((Wert_Baudrate > 0xFFF) || (Wert_Baudrate < 4 ))
      return(FALSE);

   // Baudrateregister setzen
   Control2 = (UINT16)Wert_Baudrate;

//*--------------------------------------------------------------------------*

   serChs[Channel].Protocol = Protocol;

   if (!(Protocol & SER_BUS_ACTIVE))
   {  // Basic
      Protocol_LowNibble = Protocol & 0x0F;
      serChs[Channel].MsgStatus = Statemachine_Off;
   }
   else
   {  // ein Busprotokoll mit Statemaschine
      if (Protocol == SER_CMD_BIOS)
      {  // Datenformat fuer Bioscontrol ist fest vorgegeben
         Protocol_LowNibble = SER_8N1;

         serChs[Channel].MsgStatus = CMD_No_Message;
         timStopTimer(serChs[Channel].TimerNr);
      }
      else
         if (Protocol == SER_LIN_20)
         {  // Datenformat fuer LIN 2.0 ist fest vorgegeben
            Protocol_LowNibble = SER_8N1;

            // Timerwerte in [us]
            serChs[Channel].Sync_Delimiter_Time    = us((UINT32)(1.0 *  (1e6 / Baudrate))); // (1 T)
            serChs[Channel].InFrame_Response_Time  = us((UINT32)(2.0 *  (1e6 / Baudrate))); // (2 T)
            serChs[Channel].InterByte_Time         = us((UINT32)(1.0 *  (1e6 / Baudrate))); // (1 T)

            // Timeout in [ms]
            serChs[Channel].InFrame_Response_Timeout  = 100;
            serChs[Channel].MsgStatus = Statemachine_Off;

            // Debug
            #if (CP_DEBUG == 18)
            printf ("LIN 2.0 ausgewaehlt\n");
            #endif
         }
         else
            if (Protocol == SER_FRT_AUDI)
            {  // Datenformat fuer Frontkontroller AUDI ist fest vorgegeben
               Protocol_LowNibble = SER_8E1;

               serChs[Channel].MsgStatus = FRT_No_Message;

               // max. Laenge nach Protokoll sind 64 Bytes
               serChs[Channel].Commandbyte_Length = 64;

               // Parameter fuer maximale ACK-Wartezeit [ms] umwidmen
               serChs[Channel].InFrame_Response_Timeout = 1000;

               // Quittungssteuerung ruecksetzen
               serChs[Channel].ACK_Counter = serChs[Channel].NAK_Counter = 0;

               // Messagezaehler ruecksetzen
               serChs[Channel].MSG_Counter = 0;

               // create Message buffer
               if ((serChs[Channel].MsgBuf = bufCreate(FRT_MESSAGE_BUF_SIZE)) == NULL)
               {  // creation of Message buffer not possible
                  printf ("     No Memory available for Message-Buffer ASC%d\n", Channel);
                  return (FALSE);
               }

               // Debug
               #if (CP_DEBUG == 18)
               printf ("AUDI Frontkontrollerprotokoll ausgewaehlt\n");
               #endif
            }
   } // if (!(Protocol & SER_BUS_ACTIVE))

//*--------------------------------------------------------------------------*

   // Datenbits; Parity und Stopbits einstellen
   switch (Protocol_LowNibble)
   {  case SER_8N1:  Control1 = URTEnSLD | URTEnCLG;
                     break;
      case SER_8N2:  Control1 = URTEnSLD | URTEnCLG |  URTEnSLG;
                     break;
      case SER_8E1:  Control1 = URTEnSLD | URTEnCLG |              URTEnSLP(3);
                     break;
      case SER_8E2:  Control1 = URTEnSLD | URTEnCLG |  URTEnSLG |  URTEnSLP(3);
                     break;
      case SER_8O1:  Control1 = URTEnSLD | URTEnCLG |              URTEnSLP(2);
                     break;
      case SER_8O2:  Control1 = URTEnSLD | URTEnCLG |  URTEnSLG |  URTEnSLP(2);
                     break;
      case SER_7N1:  Control1 = URTEnSLD ;
                     break;
      case SER_7N2:  Control1 = URTEnSLD |             URTEnSLG;
                     break;
      case SER_7E1:  Control1 = URTEnSLD |                         URTEnSLP(3);
                     break;
      case SER_7E2:  Control1 = URTEnSLD |             URTEnSLG |  URTEnSLP(3);
                     break;
      case SER_7O1:  Control1 = URTEnSLD |                         URTEnSLP(2);
                     break;
      case SER_7O2:  Control1 = URTEnSLD |             URTEnSLG |  URTEnSLP(2);
                     break;
      default:       return (FALSE);
   }

   // Debug
   #if (CP_DEBUG == 18)
   printf ("Sollwert Baudrate @ %8.0f Mhz: %d    Wert_Vorteiler : 0x%02X    Wert_Baudrate : 0x%02X\n", FRQ, Baudrate, Wert_Vorteiler,Wert_Baudrate);

   // wirkliche Baudrate
   Temp = P_Clk / (Wert_Baudrate*2);

   // Baudratefehler errechnen
   printf ("Istwert Baudrate : %d    Abweichung : %2.3f Prozent\n", Temp, (float)(Temp - Baudrate)/ Baudrate *100.0);
   #endif

//*--------------------------------------------------------------------------*
//* und in die richtige Schnittstelle einsetzen                              *
//*--------------------------------------------------------------------------*

   switch(Channel)
   {

      case 3:
         #if (CP_SER3  || (CP_CMDDEV_RS232 & DEV_3))
         // LIN Master LMA3 im UART through mode
         LMA3CTLL = 0x0000;
         LMA3CTLH = 0x0000;

         // Power-down (OFF)
         URTE3PW = 0;
         // kein datacheck
         URTE3SLDC = 0;

         // setup baudrate and mode
         URTE3CTL1 = Control1;
         URTE3CTL2 = Control2;

         // LIN-Break : 13 bit
         URTE3CTL1 |= URTEnBLG(5);

         // start UART power
         URTE3PW = 1;

         // Enable Ports, Receiver and Transmitter
         Ux3_PORT_CONFIG;
         URTE3RXE = 1;
         URTE3TXE = 1;

         // Dummyread
         Dummy = URTE3RX;

         // Interrupts configurieren und scharfmachen
         Ux3_RX_CONFIG_INTVECTOR;
         Ux3_RX_INT_ENABLE;
         break;
         #else // CP_SER3
         return (FALSE);
         #endif

      case 5:
         #if (CP_SER5  || (CP_CMDDEV_RS232 & DEV_5))
         // LIN Master LMA5 im UART through mode
         LMA5CTLL = 0x0000;
         LMA5CTLH = 0x0000;

         // Power-down (OFF)
         URTE5PW = 0;
         // kein datacheck
         URTE5SLDC = 0;

         // setup baudrate and mode
         URTE5CTL1 = Control1;
         URTE5CTL2 = Control2;

         // LIN-Break : 13 bit
         URTE5CTL1 |= URTEnBLG(5);

         // start UART power
         URTE5PW = 1;

         // Enable Ports, Receiver and Transmitter
         Ux5_PORT_CONFIG;
         URTE5RXE = 1;
         URTE5TXE = 1;

         // Dummyread
         Dummy = URTE5RX;

         // Interrupts configurieren und scharfmachen
         Ux5_RX_CONFIG_INTVECTOR;
         Ux5_RX_INT_ENABLE;
         break;
         #else // CP_SER5
         return (FALSE);
         #endif

      case 10:
         #if (CP_SER10  || (CP_CMDDEV_RS232 & DEV_10))
         // LIN Master LMA10 im UART through mode
         LMA10CTLL = 0x0000;
         LMA10CTLH = 0x0000;

         // Power-down (OFF)
         URTE10PW = 0;
         // kein datacheck
         URTE10SLDC = 0;

         // setup baudrate and mode
         URTE10CTL1 = Control1;
         URTE10CTL2 = Control2;

         // LIN-Break : 13 bit
         URTE10CTL1 |= URTEnBLG(5);

         // start UART power
         URTE10PW = 1;

         // Enable Ports, Receiver and Transmitter
         Ux10_PORT_CONFIG;
         URTE10RXE = 1;
         URTE10TXE = 1;

         // Dummyread
         Dummy = URTE10RX;

         // Interrupts configurieren und scharfmachen
         Ux10_RX_CONFIG_INTVECTOR;
         Ux10_RX_INT_ENABLE;
         break;
         #else // CP_SER10
         return (FALSE);
         #endif

      case 11:
         #if (CP_SER11  || (CP_CMDDEV_RS232 & DEV_11))
         // LIN Master LMA11 im UART through mode
         LMA11CTLL = 0x0000;
         LMA11CTLH = 0x0000;

         // Power-down (OFF)
         URTE11PW = 0;
         // kein datacheck
         URTE11SLDC = 0;

         // setup baudrate and mode
         URTE11CTL1 = Control1;
         URTE11CTL2 = Control2;

         // LIN-Break : 13 bit
         URTE11CTL1 |= URTEnBLG(5);

         // start UART power
         URTE11PW = 1;

         // Enable Ports, Receiver and Transmitter
         Ux11_PORT_CONFIG;
         URTE11RXE = 1;
         URTE11TXE = 1;

         // Dummyread
         Dummy = URTE11RX;

         // Interrupts configurieren und scharfmachen
         Ux11_RX_CONFIG_INTVECTOR;
         Ux11_RX_INT_ENABLE;
         break;
         #else // CP_SER11
         return (FALSE);
         #endif

      default:
         return(FALSE);
   } // of switch

   #if (CP_DEBUG == 18)
   printf ("** ASC%d : Initializing\n", Channel);
   #endif

   // create input buffer
   if ((serChs[Channel].inBuf = bufCreate(ASC_BUF_SIZE)) == NULL)
   {  // creation of input buffer not possible
      printf ("     No Memory available for RX-Buffer ASC%d\n", Channel);
      return (FALSE);
   }

   // Handler ruecksetzen
   serChs[Channel].opened  = TRUE;        // mark channel as being opened
   serChs[Channel].sending = FALSE;       // channel is not sending
   serChs[Channel].frErr   = 0;           // number of frame errors
   serChs[Channel].parErr  = 0;           // number of parity errors
   serChs[Channel].ovrErr  = 0;           // number of overrun errors
   serChs[Channel].brkErr  = 0;           // number of breaks

   #if (CP_DEBUG == 18)
   printf("     Done!\n");
   #endif

   // nur wenn kein Busprotokoll
   if (!(serChs[Channel].Protocol & SER_BUS_ACTIVE))
      SerSetRTS (Channel);


   #if (CP_CMDDEV_RS232 & DEV_3)
   if (Channel == 3)
   serChs[Channel].CommIndex = msgRegister(RS232+Channel, "UART", Channel, 0, NULL, RS232_Send, NULL);
   #endif

   #if (CP_CMDDEV_RS232 & DEV_5)
   if (Channel == 5)
   serChs[Channel].CommIndex = msgRegister(RS232+Channel, "UART", Channel, 0, NULL, RS232_Send, NULL);
   #endif

   #if (CP_CMDDEV_RS232 & DEV_10)
   if (Channel == 10)
   serChs[Channel].CommIndex = msgRegister(RS232+Channel, "UART", Channel, 0, NULL, RS232_Send, NULL);
   #endif

   #if (CP_CMDDEV_RS232 & DEV_11)
   if (Channel == 11)
   serChs[Channel].CommIndex = msgRegister(RS232+Channel, "UART", Channel, 0, NULL, RS232_Send, NULL);
   #endif

   return (TRUE);
} // end of function SerOpen


//****************************************************************************
//* Function     :   SerClose                                                *
//****************************************************************************
//* Description  :   Schliessen einer ser. Schnittstelle                     *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

static BOOL SerClose(UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (FALSE);

   // check wether the channel is not open
   if (!serChs[Channel].opened)
      return (TRUE);

   // nur wenn kein Busprotokoll
   if (!(serChs[Channel].Protocol & SER_BUS_ACTIVE))
      SerClrRTS (Channel);

   // delete all buffer
   bufDelete (serChs[Channel].inBuf);
   bufDelete (serChs[Channel].MsgBuf);

   switch (Channel)
   {
      case 3:
         #if (CP_SER3 || (CP_CMDDEV_RS232 & DEV_3))
         Ux3_RX_INT_DISABLE;

         // Disable Receiver, Transmitter and ports
         URTE3RXE = 0;
         URTE3TXE = 0;
         Ux3_PORT_UNCONFIG;

         // Power-down (OFF)
         URTE3PW = 0;
         break;
         #else
         return (FALSE);
         #endif

      case 5:
         #if (CP_SER5 || (CP_CMDDEV_RS232 & DEV_5))
         Ux5_RX_INT_DISABLE;

         // Disable Receiver, Transmitter and ports
         URTE5RXE = 0;
         URTE5TXE = 0;
         Ux5_PORT_UNCONFIG;

         // Power-down (OFF)
         URTE5PW = 0;
         break;
         #else
         return (FALSE);
         #endif

      case 10:
         #if (CP_SER10 || (CP_CMDDEV_RS232 & DEV_10))
         Ux10_RX_INT_DISABLE;

         // Disable Receiver, Transmitter and ports
         URTE10RXE = 0;
         URTE10TXE = 0;
         Ux10_PORT_UNCONFIG;

         // Power-down (OFF)
         URTE10PW = 0;
         break;
         #else
         return (FALSE);
         #endif

      case 11:
         #if (CP_SER11 || (CP_CMDDEV_RS232 & DEV_11))
         Ux11_RX_INT_DISABLE;

         // Disable Receiver, Transmitter and ports
         URTE11RXE = 0;
         URTE11TXE = 0;
         Ux11_PORT_UNCONFIG;

         // Power-down (OFF)
         URTE11PW = 0;
         break;
         #else
         return (FALSE);
         #endif

      default:
         return(FALSE);
   }

   // Handler löschen
   serChs[Channel].opened   = FALSE;
   serChs[Channel].sending  = FALSE;
   serChs[Channel].Protocol = SER_8N1;

   return (TRUE);
} // of function SerClose


//****************************************************************************
//* Function     :   SerWriteChar                                            *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL SerWriteChar (UINT8 Channel, UINT8 Data)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (FALSE);

   // check wether channel opened
   if (!serChs[Channel].opened)
      return (FALSE);

   // send data
   switch (Channel)
   {
      case 3:
         #if (CP_SER3  || (CP_CMDDEV_RS232 & DEV_3))
         if ((serChs[3].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
         {  // warten, bis CTS true
            timSetMarker(TIMER_TX_GLOBAL);
            while (!SER3_CTS)
            {
               if (timTimeout(TIMER_TX_GLOBAL, SER_CTS_TIMEOUT))
                  return (FALSE);
            }
         }

         // Transmitter leerlaufen lassen
         while (URTE3STR0 & URTEnSST)
         {}
         // Daten senden
         URTE3TX = Data;
         return(TRUE);
         #else // CP_SER3
         return (FALSE);
         #endif

      case 5:
         #if (CP_SER5  || (CP_CMDDEV_RS232 & DEV_5))
         if ((serChs[5].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
         {  // warten, bis CTS true
            timSetMarker(TIMER_TX_GLOBAL);
            while (!SER5_CTS)
            {
               if (timTimeout(TIMER_TX_GLOBAL, SER_CTS_TIMEOUT))
                  return (FALSE);
            }
         }

         // Transmitter leerlaufen lassen
         while (URTE5STR0 & URTEnSST)
         {}
         // Daten senden
         URTE5TX = Data;
         return(TRUE);
         #else // CP_SER5
         return (FALSE);
         #endif

         case 10:
            #if (CP_SER10  || (CP_CMDDEV_RS232 & DEV_10))
            if ((serChs[10].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
            {  // warten, bis CTS true
               timSetMarker(TIMER_TX_GLOBAL);
               while (!SER10_CTS)
               {
                  if (timTimeout(TIMER_TX_GLOBAL, SER_CTS_TIMEOUT))
                     return (FALSE);
               }
            }

            // Transmitter leerlaufen lassen
            while (URTE10STR0 & URTEnSST)
            {}
            // Daten senden
            URTE10TX = Data;
            return(TRUE);
            #else // CP_SER10
            return (FALSE);
            #endif

      case 11:
         #if (CP_SER11  || (CP_CMDDEV_RS232 & DEV_11))
         if ((serChs[11].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
         {  // warten, bis CTS true
            timSetMarker(TIMER_TX_GLOBAL);
            while (!SER11_CTS)
            {
    	         if (timTimeout(TIMER_TX_GLOBAL, SER_CTS_TIMEOUT))
                  return (FALSE);
            }
         }

         // Transmitter leerlaufen lassen
         while (URTE11STR0 & URTEnSST)
  	      {}
         // Daten senden
         URTE11TX = Data;
         return(TRUE);
         #else // CP_SER11
         return (FALSE);
         #endif

      default:
         return(FALSE);
   }
} // of function SerWriteChar


//****************************************************************************
//* Function     :   SerWriteBreak                                           *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static BOOL SerWriteBreak (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (FALSE);

   // check wether channel opened
   if (!serChs[Channel].opened)
      return (FALSE);

   // send data
   switch (Channel)
   {
      case 3:
      case 5:
      case 10:
      case 11:
         #if (0)
         // Transmitter leerlaufen lassen
         while (_Ux0TSF)
  	      {}
         // Reception Trigger ein
         _Ux0SRT = 1;
         // Break senden
         _Ux0STT = 1;
         // Transmitter leerlaufen lassen
         while (_Ux0TSF)
  	      {}
         return(TRUE);
         #else // CP_SER11
         return (FALSE);
         #endif

      default:
         return(FALSE);
   }
} // of function SerWriteBreak
#endif

//****************************************************************************
//* Function     :   SerSetRTS                                               *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

static void SerSetRTS (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (Channel)
   {
      case 3:
         #if (CP_SER3  || (CP_CMDDEV_RS232 & DEV_3))
         {  if ((serChs[3].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER3_RTS_ACTIVATE
         else
            SER3_RTS_DEACTIVATE;
         }
         break;
         #endif

      case 5:
         #if (CP_SER5  || (CP_CMDDEV_RS232 & DEV_5))
         {  if ((serChs[5].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER5_RTS_ACTIVATE
         else
            SER5_RTS_DEACTIVATE;
         }
         break;
         #endif

      case 10:
         #if (CP_SER10  || (CP_CMDDEV_RS232 & DEV_10))
         {  if ((serChs[10].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER10_RTS_ACTIVATE
         else
            SER10_RTS_DEACTIVATE;
         }
         break;
         #endif

      case 11:
         #if (CP_SER11  || (CP_CMDDEV_RS232 & DEV_11))
         {  if ((serChs[11].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER11_RTS_ACTIVATE
         else
            SER11_RTS_DEACTIVATE;
         }
         break;
         #endif
   } // of switch
}

//****************************************************************************
//* Function     :   SerClrRTS                                               *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

static void SerClrRTS (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   switch (Channel)
   {
      case 3:
         #if (CP_SER3  || (CP_CMDDEV_RS232 & DEV_3))
         {  if ((serChs[3].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER3_RTS_DEACTIVATE
         else
            SER3_RTS_ACTIVATE;
         }
         break;
         #endif

      case 5:
         #if (CP_SER5  || (CP_CMDDEV_RS232 & DEV_5))
         {  if ((serChs[5].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER5_RTS_DEACTIVATE
         else
            SER5_RTS_ACTIVATE;
         }
         break;
         #endif

      case 10:
         #if (CP_SER10  || (CP_CMDDEV_RS232 & DEV_10))
         {  if ((serChs[10].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER10_RTS_DEACTIVATE
         else
            SER10_RTS_ACTIVATE;
         }
         break;
         #endif

      case 11:
         #if (CP_SER11  || (CP_CMDDEV_RS232 & DEV_11))
         {  if ((serChs[11].Protocol & SER_RTS_POLARITY) == SER_RTS_POLARITY)
            SER11_RTS_DEACTIVATE
         else
            SER11_RTS_ACTIVATE;
         }
         break;
         #endif
   } // of switch
}


//****************************************************************************
//* Function     :   SerReadBuf                                              *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Parameter    :	          									                    *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT16 SerReadBuf (UINT8 Channel, UINT8 *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT16 count;
BOOL Result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (0);

   // check wether channel opened
   if (!serChs[Channel].opened)
      return (0);

   // get data from queue
   count = 0;

   do
   {  // Ints hier sperren, da BUFFER.C nicht interruptsicher
      __DI();
      Result = bufGet(serChs[Channel].inBuf, Buffer);

      // set global interrupt enable flag
      __EI();

      if (Result)
      {  count++;
         Buffer++;
         if (count >= 244)
            break;
      }
   }
   while (Result);

   // RTS-Steuerung ?
   if ((serChs[Channel].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
   {  // Test, ob RTS wieder gesetzt werden kann
      if(bufSpace(serChs[Channel].inBuf) > ASC_BUF_SIZE_RTSLIMIT)
         SerSetRTS (Channel);
   }

   return (count);
} // of function SerReadBuf
#endif

//****************************************************************************
//* Function     :  PutMsg                                                   *
//****************************************************************************
//* Description :   Kopiert alle empfangenen Bytes in den Messagebuffer      *
//*                 Aufbau : UINT32 Error,UINT8 Laenge                       *
//*                          0..64 Databytes                                 *                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   TRUE, wenn in Ordnung                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL PutMsg (UINT8 Channel, UINT32 Error)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Wert;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Statemaschine fuer neue Message vorbereiten
   serChs[Channel].MsgStatus = FRT_No_Message;
   timStopTimer(serChs[Channel].TimerNr);

   // Mindestlaenge im Buffer ueberpruefen : Inhalt von InBuf + 5 Bytes Header
   if (bufSpace(serChs[Channel].MsgBuf) < (bufItems(serChs[Channel].inBuf)+5))
   {  // Message mangels Speicherplatz verwerfen
      bufFlush(serChs[Channel].inBuf);
      return(FALSE);
   }

   // Messagezaehler erhoehen
   serChs[Channel].MSG_Counter++;

   // zuerst die Errornummer
   Insert_Fifo_32 (Error, serChs[Channel].MsgBuf);

   // dann die Anzahl der vorhandenen Datenbytes
   bufPutFiFo(serChs[Channel].MsgBuf, bufItems(serChs[Channel].inBuf));

   // und dann noch die Daten selber ablegen
   while (bufGet(serChs[Channel].inBuf, &Wert))
      bufPutFiFo(serChs[Channel].MsgBuf, Wert);

   return(TRUE);
}  // end of function PutMsg


//****************************************************************************
//* Function     :  PutChar                                                  *
//****************************************************************************
//* Description :                                                            *
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

void PutChar (UINT8 Channel, UINT8 RX_Char)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Dummy;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (serChs[Channel].Protocol == SER_CMD_BIOS)
   {
      // Comm modul bedienen
      msgReceive (serChs[Channel].CommIndex, &RX_Char, 1);
      return;
   }

//*--------------------------------------------------------------------------*

   switch (serChs[Channel].MsgStatus)
   {
      case Statemachine_Off:
         // S-Command received, no Cmd Channel
         // check for space on the receive queue
         if(bufSpace(serChs[Channel].inBuf) == 0)
         {  // 1 Byte im Buffer freimachen und neu belegen
            bufGet(serChs[Channel].inBuf, &Dummy);
         }
         // put received data on the receive queue
         bufPutFiFo(serChs[Channel].inBuf, RX_Char);
         break;
//*--------------------------------------------------------------------------*
      case CMD_No_Message :
         break;

//*--------------------------------------------------------------------------*
//* DLE-STX Protokoll mit kompletter Statemaschine fuer Frontkontroller      *
//*                                                                          *
//* Regeln :      DLE STX startet immer ein neues Frame                      *
//*               DLE im Datenstrom muss verdoppelt sein                     *
//*               DLE ETX beendet das Frame                                  *
//*               Ein Timeout-Timer gibt die maximale Uebertragungszeit vor  *
//*--------------------------------------------------------------------------*

      case FRT_No_Message :
         switch (RX_Char)
         {  case DLE :  serChs[Channel].MsgStatus = FRT_First_DLE_received;
                        break;

            case ACK :  serChs[Channel].ACK_Counter++;
                        break;

            case NAK :  serChs[Channel].NAK_Counter++;;
                        break;
         }
         break;
//*--------------------------------------------------------------------------*
      case FRT_First_DLE_received :
         if (RX_Char == STX)
         {  serChs[Channel].MsgStatus = FRT_STX_received;
            serChs[Channel].Checksum = DLE + STX;
            serChs[Channel].Commandbyte_Index = 0;
            timSetMarker(serChs[Channel].TimerNr);
         }
         else
         {  // Fehler erkannt; Message speichern und neu starten
            PutMsg (Channel, ERROR_SER | SER_ERR_FRT_MISSINGSTX);
         }
         break;
//*--------------------------------------------------------------------------*
      case FRT_STX_received :
      case FRT_Data_received :
         // Checksum auch bei DLE-Verdoppelung aufaddieren
         serChs[Channel].Checksum += RX_Char;

         if (RX_Char != DLE)
         {  // max. Laenge ueberschritten ?
            if (serChs[Channel].Commandbyte_Index++ > serChs[Channel].Commandbyte_Length)
            {  // Fehler erkannt; Message speichern und neu starten
               PutMsg (Channel, ERROR_SER | SER_ERR_FRT_MAXLEN);
            }
            else
               bufPutFiFo(serChs[Channel].inBuf, RX_Char);
         }
         else
            serChs[Channel].MsgStatus = FRT_DLE_received;
         break;
//*--------------------------------------------------------------------------*
      case FRT_DLE_received :
         // Checksum immer aufaddieren
         serChs[Channel].Checksum += RX_Char;

         switch (RX_Char)
         {  case ETX :  serChs[Channel].MsgStatus = FRT_ETX_received;
                        break;

            case DLE :  // max. Laenge ueberschritten ?
                        if (serChs[Channel].Commandbyte_Index++ > serChs[Channel].Commandbyte_Length)
                        {  // Fehler erkannt; Message speichern und neu starten
                           PutMsg (Channel, ERROR_SER | SER_ERR_FRT_MAXLEN);
                        }
                        else
                        {
                           serChs[Channel].MsgStatus = FRT_Data_received;
                           bufPutFiFo(serChs[Channel].inBuf, RX_Char);
                        }
                        break;

            default :   // Fehler erkannt; Message speichern und neu starten
                        PutMsg (Channel, ERROR_SER | SER_ERR_FRT_WRONGDLE);
                        break;
         }
         break;
//*--------------------------------------------------------------------------*
      case FRT_ETX_received :
         // Frame beendet
         timStopTimer(serChs[Channel].TimerNr);

         // jetzt noch die Checksum überprüfen
         if (RX_Char == serChs[Channel].Checksum)
         {  // kein Fehler erkannt; Message speichern und neu starten
            PutMsg (Channel, NO_ERROR);
            // und ACK schicken
            SerWriteChar (Channel, ACK);
         }
         else
         {
            // Fehler erkannt; Message speichern und neu starten
            PutMsg (Channel, ERROR_SER | SER_ERR_FRT_CHKSUM);
         }
         break;

   } // of switch (serChs[Channel].MsgStatus)
}


//****************************************************************************
//* Function     :  Handle_RX_Error                                          *
//****************************************************************************
//* Description :                                                            *
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

void Handle_RX_Error (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Fehler protokollieren und ruecksetzen
   switch(Channel)
   {
      #if (CP_SER3 || (CP_CMDDEV_RS232 & DEV_3))
      case 3 :
               if (URTE3STR1 & URTEnFE)
                  serChs[3].frErr++;

               if (URTE3STR1 & URTEnPE)
                  serChs[3].parErr++;

               if (URTE3STR1 & URTEnOVE)
                  serChs[3].ovrErr++;

               // clear all errors
               URTE3CLOV = URTE3CLF = URTE3CLP = 1;
               break;
      #endif

      #if (CP_SER5 || (CP_CMDDEV_RS232 & DEV_5))
      case 5 :
               if (URTE5STR1 & URTEnFE)
                  serChs[5].frErr++;

               if (URTE5STR1 & URTEnPE)
                  serChs[5].parErr++;

               if (URTE5STR1 & URTEnOVE)
                  serChs[5].ovrErr++;

               // clear all errors
               URTE5CLOV = URTE5CLF = URTE5CLP = 1;
               break;
      #endif

      #if (CP_SER10 || (CP_CMDDEV_RS232 & DEV_10))
      case 10 :
               if (URTE10STR1 & URTEnFE)
                  serChs[10].frErr++;

               if (URTE10STR1 & URTEnPE)
                  serChs[10].parErr++;

               if (URTE10STR1 & URTEnOVE)
                  serChs[10].ovrErr++;

               // clear all errors
               URTE10CLOV = URTE10CLF = URTE10CLP = 1;
               break;
      #endif

      #if (CP_SER11 || (CP_CMDDEV_RS232 & DEV_11))
      case 11 :
               if (URTE11STR1 & URTEnFE)
                  serChs[11].frErr++;

               if (URTE11STR1 & URTEnPE)
                  serChs[11].parErr++;

               if (URTE11STR1 & URTEnOVE)
                  serChs[11].ovrErr++;

               // clear all errors
               URTE11CLOV = URTE11CLF = URTE11CLP = 1;
               break;
      #endif
   }


   // Statemaschinen ruecksetzen
   switch (serChs[Channel].Protocol)
   {
      case SER_CMD_BIOS:
         break;

      case SER_LIN_20:
         serChs[Channel].MsgStatus = Statemachine_Off;
         break;

      case SER_FRT_AUDI:
         serChs[Channel].MsgStatus = FRT_No_Message;
         break;

      default:
         serChs[Channel].MsgStatus = Statemachine_Off;
         break;
   }
} // of function Handle_RX_Error

//****************************************************************************
//* Functionen       INTLMA3IR, INTLMA3IS                                    *
//****************************************************************************
//* Description :   Interrupt Service Routinen für die Ser. Schnittstelle 3  *
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

#if (CP_SER3  || (CP_CMDDEV_RS232 & DEV_3))
#pragma ghs interrupt
void INTLMA3IR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // optionale RTS-Steuerung
   if ((serChs[3].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
      if(bufSpace(serChs[3].inBuf) <= ASC_BUF_SIZE_RTSLIMIT)
         SerClrRTS(3);

   // Char genau einmal abholen
   RX_Char = URTE3RX;

   // Char verarbeiten
   PutChar (3, RX_Char);
} // end of function

#pragma ghs interrupt
void INTLMA3IS (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 volatile Dummy;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Char als Dummy abholen
   Dummy = URTE3RX;

   // Overrun, Parity oder Framingerror auswerten
   if (URTE3STR1 & (URTEnFE | URTEnPE | URTEnOVE))
       Handle_RX_Error(3);
} // end of function
#endif

//****************************************************************************
//* Functionen       INTLMA5IR, INTLMA5IS                                    *
//****************************************************************************
//* Description :   Interrupt Service Routinen für die Ser. Schnittstelle 5  *
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

#if (CP_SER5  || (CP_CMDDEV_RS232 & DEV_5))
#pragma ghs interrupt
void INTLMA5IR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // optionale RTS-Steuerung
   if ((serChs[5].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
      if(bufSpace(serChs[5].inBuf) <= ASC_BUF_SIZE_RTSLIMIT)
         SerClrRTS(5);

   // Char genau einmal abholen
   RX_Char = URTE5RX;

   // Char verarbeiten
   PutChar (5, RX_Char);
} // end of function

#pragma ghs interrupt
void INTLMA5IS (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 volatile Dummy;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Char als Dummy abholen
   Dummy = URTE5RX;

   // Overrun, Parity oder Framingerror auswerten
   if (URTE5STR1 & (URTEnFE | URTEnPE | URTEnOVE))
       Handle_RX_Error(5);
} // end of function
#endif

//****************************************************************************
//* Functionen       INTLMA10IR, INTLMA10IS                                  *
//****************************************************************************
//* Description :   Interrupt Service Routinen für die Ser. Schnittstelle 10 *
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

#if (CP_SER10  || (CP_CMDDEV_RS232 & DEV_10))
#pragma ghs interrupt
void INTLMA10IR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // optionale RTS-Steuerung
   if ((serChs[10].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
      if(bufSpace(serChs[10].inBuf) <= ASC_BUF_SIZE_RTSLIMIT)
         SerClrRTS(10);

   // Char genau einmal abholen
   RX_Char = URTE10RX;

   // Char verarbeiten
   PutChar (10, RX_Char);
} // end of function

#pragma ghs interrupt
void INTLMA10IS (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 volatile Dummy;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Char als Dummy abholen
   Dummy = URTE10RX;

   // Overrun, Parity oder Framingerror auswerten
   if (URTE10STR1 & (URTEnFE | URTEnPE | URTEnOVE))
       Handle_RX_Error(10);
} // end of function
#endif

//****************************************************************************
//* Functionen       INTLMA11IR, INTLMA11IS                                  *
//****************************************************************************
//* Description :   Interrupt Service Routinen für die Ser. Schnittstelle 11 *
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

#if (CP_SER11  || (CP_CMDDEV_RS232 & DEV_11))
#pragma ghs interrupt
void INTLMA11IR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // optionale RTS-Steuerung
   if ((serChs[11].Protocol & (SER_BUS_ACTIVE | SER_ENABLE_RTSCTS)) == SER_ENABLE_RTSCTS)
      if(bufSpace(serChs[11].inBuf) <= ASC_BUF_SIZE_RTSLIMIT)
         SerClrRTS(11);

   // Char genau einmal abholen
   RX_Char = URTE11RX;

   // Char verarbeiten
   PutChar (11, RX_Char);
} // end of function

#pragma ghs interrupt
void INTLMA11IS (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 volatile Dummy;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Char als Dummy abholen
   Dummy = URTE11RX;

   // Overrun, Parity oder Framingerror auswerten
   if (URTE11STR1 & (URTEnFE | URTEnPE | URTEnOVE))
       Handle_RX_Error(11);
} // end of function
#endif


//****************************************************************************
//* Function     :   RS232_Init                                              *
//****************************************************************************
//* Description  :   Initialisierung der ser. Schnittstelle 0                *
//*                  und des Timeout Timers                                  *
//*                                                                          *
//* Parameter    :	Baudrate als Skalar          									  *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :  bei 32 Mhz nur 115200 Baudrate moeglich                  *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if CP_CMDDEV_RS232
BOOL RS232_Init (UINT8 Channel, UINT32 Baudrate)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // check for channel range
   if (Channel > SER_CH_NUM)
      return (FALSE);

   // schliessen , falls schon geoeffnet
   if (!SerClose(Channel))
      return(FALSE);

   // oeffnen
   if (!SerOpen(Channel, Baudrate, SER_CMD_BIOS))
      return(FALSE);

   return(TRUE);
} // end of function RS232_Init
#endif

//****************************************************************************
//* Function     :	RS232_Send                                              *
//****************************************************************************
//* Description  :   Inhalt des Buffers senden                               *
//*                                                                          *
//* Parameter    :   Buffer                                                  *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*						2. Stopbit wird per Software erzeugt !!                 *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if CP_CMDDEV_RS232
BOOL RS232_Send  (UINT8 Channel, UINT8 *Buffer, UINT32 len)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

   UINT32 i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   for (i = 0; i < len ; i++)
   {
      SerWriteChar (Channel, Buffer[i]);
   }

   return(TRUE);
} // of function RS232_Send
#endif


//****************************************************************************
//* Function     :	LIN_Calc_Checksum                                       *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :   Char                                                    *
//*                                                                          *
//* Returnvalue  :   Char                                                    *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT8 LIN_Calc_Checksum (UINT8 *Data, UINT8 Anzahl)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT16   Checksum;

#define CARRYBIT 0x100

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Checksum = 0;

   // Checksum mit Add-Carry Emulation erechnen
   for ( ;Anzahl >0; Anzahl-- )
   {
      Checksum = Checksum + *Data++;

      if (Checksum & CARRYBIT)
         Checksum = Checksum +1 - CARRYBIT;
   }

   return (~(UINT8)Checksum);
} // of function LIN_Calc_Checksum
#endif

//****************************************************************************
//* Function     :	LIN_Add_Parity                                          *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :   Char                                                    *
//*                                                                          *
//* Returnvalue  :   Char                                                    *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT8 LIN_Add_Parity (UINT8 Identifier)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Paritybits loeschen
   Identifier &= 0x3F;

   // P0 errechnen
   if ((Identifier ^ (Identifier>>1) ^ (Identifier>>2) ^ (Identifier>>4)) & 0x01)
      Identifier |= 0x40;

   // P1 errechnen
   if (!(((Identifier>>1) ^ (Identifier>>3) ^ (Identifier>>4) ^ (Identifier>>5)) & 0x01))
      Identifier |= 0x80;

   return (Identifier);
} // of function LIN_Add_Parity
#endif

//****************************************************************************
//* Function     :	LIN_Send_Header                                         *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :   Char                                                    *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT32  LIN_Send_Header (UINT8 Channel, UINT8 Identifier)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Error;
UINT8    RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Receiverbuffer von allen Echos loeschen
   bufFlush(serChs[Channel].inBuf);

   // Synch Break erzeugen
   SerWriteBreak (Channel);

   // dann auf Break-Echo warten (typ. 0,5-1 Bitzeiten)
   timSetMarker(TIMER_TX_GLOBAL);

   while (!bufGet(serChs[Channel].inBuf, &RX_Char))
   {
      if (timTimeout(TIMER_TX_GLOBAL, serChs[Channel].InFrame_Response_Timeout))
      {  // gar kein Empfang; Fehler
         return (ERROR_SER | SER_ERR_LIN_BIT1);
      }
   }

   // Sync Delimiter abwarten
   DelayLoop (serChs[Channel].Sync_Delimiter_Time);

   // Sync Field schreiben
   Error = LIN_Send_Byte(Channel, 0x55);

   if (Error != NO_ERROR)
      return (Error);

   // Interbyte Space abwarten
   DelayLoop (serChs[Channel].InterByte_Time);

   // Ident Field schreiben
   Error = LIN_Send_Byte(Channel, LIN_Add_Parity(Identifier));

   return (Error);
} // of function LIN_Send_Header
#endif

//****************************************************************************
//* Function     :	LIN_Send_Data                                           *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT32 LIN_Send_Data (UINT8 Channel, UINT8 *Data, UINT8 Anzahl)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // InFrame_Response_Time erzeugen
   DelayLoop (serChs[Channel].InFrame_Response_Time);

   // Daten und Checksum schreiben
   for ( ;Anzahl >0; Anzahl-- )
   {  Error = LIN_Send_Byte (Channel, *Data++);

      if (Error != NO_ERROR)
         break;

      // Interbyte Space abwarten
      DelayLoop (serChs[Channel].InterByte_Time);
   }

   return (Error);
} // of function LIN_Send_Data
#endif

//****************************************************************************
//* Function     :	LIN_Send_Byte                                           *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :   Char                                                    *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT32 LIN_Send_Byte (UINT8 Channel, UINT8 Data)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 RX_Char;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Receiverbuffer von allen Echos loeschen
   bufFlush(serChs[Channel].inBuf);

   SerWriteChar(Channel, Data);

   // dann auf Echo warten (typ. 8 Bitzeiten)
   timSetMarker(TIMER_TX_GLOBAL);

   while (!bufGet(serChs[Channel].inBuf, &RX_Char))
   {
      if (timTimeout(TIMER_TX_GLOBAL, serChs[Channel].InFrame_Response_Timeout))
      {  // gar kein Empfang; Fehler
         return (ERROR_SER | SER_ERR_LIN_BIT1);
      }
   }

   // Echobyte ungleich; Fehlerin der Busleitung
   if (RX_Char != Data)
      return (ERROR_SER | SER_ERR_LIN_BIT2);

   return (NO_ERROR);
} // function LIN_Send_Byte
#endif

//****************************************************************************
//* Function     :	LIN_Read_Data                                           *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT32 LIN_Read_Data (UINT8 Channel, UINT8 *Data, UINT8 Anzahl)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Receiverbuffer von allen Echos loeschen
   bufFlush(serChs[Channel].inBuf);

   // auf Char vom LIN-Slave warten
   timSetMarker(TIMER_TX_GLOBAL);

   // auf Daten und Checksum warten
   for ( ;Anzahl >0; Anzahl-- )
   {  // 1 - 8 Datenbytes möglich
      while (!bufGet(serChs[Channel].inBuf, Data))
      {
         if (timTimeout(TIMER_TX_GLOBAL, serChs[Channel].InFrame_Response_Timeout))
         {  // Timeout
            return (ERROR_SER | SER_ERR_LIN_RESPONSETIME);
         }
      }
      Data++;
   } // if Anzahl

   return (NO_ERROR);
} // of function LIN_Read_Data
#endif

//****************************************************************************
//* Function     :	FRT_Send_Data                                           *
//****************************************************************************
//* Description  :                                 								  *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if (CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
static UINT32 FRT_Send_Data (UINT8 Channel, UINT8 *Data, UINT8 Anzahl)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
   UINT8    TX_Checksum;
   UINT16   i;
   UINT32   Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // empfangene ACK´s loeschen
   serChs[Channel].ACK_Counter = 0;

//*--------------------------------------------------------------------------*

   // Checksum von Header und Footer setzen
   TX_Checksum = 0x25;

   // Startsequenz
   SerWriteChar (Channel, DLE);
   SerWriteChar (Channel, STX);

	for (i = 0; i < Anzahl ; i++)
  	{  // Checksumberechnung mitlaufen lassen
      TX_Checksum += Data[i];
      SerWriteChar (Channel, Data[i]);

      // DLE-Verdoppelung
      if (Data[i] == DLE)
      {  // Checksum auch hier addieren
         TX_Checksum += DLE;
         SerWriteChar (Channel, DLE);
      }
   }

   // Endesequenz
   SerWriteChar (Channel, DLE);
   SerWriteChar (Channel, ETX);

   // Checksum ohne DLE-Verdoppelung senden
   SerWriteChar (Channel, TX_Checksum);

//*--------------------------------------------------------------------------*

   // Warten auf ACK mit Timeout; NAK ignorieren, da Testsoftware
   timSetMarker(TIMER_TX_GLOBAL);

   do
   {}
   while ((serChs[Channel].ACK_Counter == 0) && !timTimeout(TIMER_TX_GLOBAL, serChs[Channel].InFrame_Response_Timeout));

   if (serChs[Channel].ACK_Counter == 0)
      Error = ERROR_SER | SER_ERR_FRT_ACKTIMEOUT;

   // empfangene ACK´s loeschen
   serChs[Channel].ACK_Counter = 0;

   return(Error);
} // of function FRT_Send_Data
#endif


#if (CP_DEBUG == 18)
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
      case 0x01 :
         // Statemaschine Zustände
         switch (serChs[commandPtr[3]].MsgStatus)
         {  case FRT_No_Message :            printf ("FRT_No_Message ");
                  	                        break;
            case FRT_First_DLE_received :    printf ("FRT_First_DLE_received");
                  	                        break;
            case FRT_STX_received :          printf ("FRT_STX_received");
                  	                        break;
            case FRT_Data_received :         printf ("FRT_Data_received");
                  	                        break;
            case FRT_DLE_received :          printf ("FRT_DLE_received");
                  	                        break;
            case FRT_ETX_received :          printf ("FRT_ETX_received");
                  	                        break;

            case CMD_No_Message :            printf ("CMD_No_Message ");
                  	                        break;

            default :                        printf ("unknown");
               	                           break;
         }
         break;



      case 0x02:
         bufPutFiFo(serChs[2].MsgBuf, commandPtr[3]);
         break;

      case 0x03:
         printf ("buf items  %8X", bufItems (serChs[2].MsgBuf));
         break;

    } // end of case

   // Error auswerten und rückmelden
   writeLong(resultPtr+4, Error);

} // end of Debug_Info
#endif // #if CP_DEBUG

//*--------------------------------------------------------------------------*
//* nuetzliche Templates                                                     *
//*--------------------------------------------------------------------------*

   #if 0
   if (Baudrate != 115200)
   {
      SETPORT_0(0,6);
      DelayLoop (us(300));
      SETPORT_1(0,6);
  }
  else
  {
      SETPORT_0(0,6);
      DelayLoop (us(100));
      SETPORT_1(0,6);
  }
   #endif

#endif // #if (CP_CMDDEV_RS232 || CP_SER3 || CP_SER5 || CP_SER10 || CP_SER11)
