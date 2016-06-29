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
#include "tool.h"                 	// für hex2str usw.
#include "ints.h"						   // Interruptverwaltung
#include "timer.h"                  // für Wartezeit
#include <string.h>          		   // für memset
#include "mostmsg.h"
#include "gateway.h"
#include "buffer.h"
#include "global.h"
#if (CP_OS81050 || CP_OS81110)
   #include "inic.h"
#endif
#if (CP_OS8104_IIC || CP_OS8104_PAR)
   #include "os8104.h"
#endif

#include "ser.h"

// Leerdefinition fuer Makros in portlist.h
/*       // IHE augeschaltet
#define INPORT(PORTNAME,PARALLEL,BIT)
#define PP_ENDPORTDEF(PORTNAME)
#define OD_ENDPORTDEF(PORTNAME)
#define PP_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)
#define OD_IOPORT(PORTNAME,PARALLEL,BIT,INITDIR,INITLVL,PWDDIR,PWDLVL)
#define PP_PORTDEF(PORTNAME)
#define OD_PORTDEF(PORTNAME)

#include "portlist.h"
*/

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

#if CP_CMDDEV_MOST		         // Hauptschalter für das ganze File ab hier

//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

// Fehlervariable nur für die Interruptroutine
UINT32  	MOSTMSG_INT_Error;

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*


//*--------------------------------------------------------------------------*
//* physikalischer Mostring                                                  *
//*--------------------------------------------------------------------------*

UINT8     MOST_State = MOST_NOT_INITIALIZED;
UINT8     OS8104_bXCR_Initvalue;

//*--------------------------------------------------------------------------*
//* Mostmessage                                                              *
//*--------------------------------------------------------------------------*

// Kanalnummer zum Parametersatz
UINT8    MOST_Selected_Channel = NONE;
UINT32   Gateway_MOST_CommIndex[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


#if (CP_CMDDEVICE == OS81050)
   #define  PACKETSIZE                 MOSTMSG_MDP_DATALEN
   BOOL     mostMsgInitialized;
   UINT8    InicOutMsg[INIC_MDP_LEN];
   UINT8    InicIncMsg[INIC_MDP_LEN];
   UINT8    MOSTMSG_RX_Buf[PACKETSIZE];

   UINT32   INIC_Send_MO_Ctrl_Data(UINT32 Laenge);
   UINT32   INIC_Write_MO_Ctrl_Data(UINT32 Laenge, UINT8 *buf);
   UINT32   INIC_Read_MO_Ctrl_Data(UINT32 *Laenge, UINT8 *src, UINT8 *buf);
   BOOL     MostMsgRunning(void);
   void     INIC_Get_MO_Ctrl_Data(void);
#endif

#if (CP_CMDDEVICE == OS81110)
   BOOL     mostMsgInitialized;
#endif

// Zustandsvariable
UINT8 	MOSTMSG_RX_State = MOSTMSG_RX_DISABLED;
UINT8 	MOSTMSG_TX_State = MOSTMSG_TX_DISABLED;

UINT32   MOSTMSG_RX_Count;
BOOL     MOSTMSG_RX_NewData = FALSE;

UINT32   MOSTMSG_PollCredits = 0;

//*--------------------------------------------------------------------------*
//* Gemeinsame Definitionen fuer OS8104                                      *
//*--------------------------------------------------------------------------*

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
   #define  PACKETSIZE           47
   static   TBuffer  *MOSTMSG_Buf = NULL;       // Data buffer
   UINT16   MOSTMSG_Messagezaehler = 0;
   UINT16   MOSTMSG_Response_Address = 0;
   BOOL     MOSTMSG_Response = FALSE;
   UINT8    MOSTMSG_RX_Buf[PACKETSIZE];

   #define  MOSTMSG_SIZE         20U              // vollstaendige Message
   #define  MOSTMSG_BUF_SIZE     (MOSTMSG_MAX * MOSTMSG_SIZE)

   #define  Send_MO_Ctrl_Data    Send_OS8104_Data
   UINT32   Read_OS8104_AllData  (void);

   void     OS8104_Get_MO_Ctrl_Data (void);
#endif

//*--------------------------------------------------------------------------*
//* Defines                                                                  *
//*--------------------------------------------------------------------------*

// Kommunikation über IIC mit OS8104
#if (CP_CMDDEVICE == OS8104_IIC)
	#define Write_MO_Ctrl_Data       IIC_Write_OS8104_Data

   #define Read_OS8104_Data         IIC_Read_OS8104_Data
   #define Write_OS8104_Register    IIC_Write_OS8104_Register
   #define Read_OS8104_Register     IIC_Read_OS8104_Register

   #define Init_OS8104              IIC_Init_OS8104

   #define OS8104_INT_FUNCTION      OS8104_IIC_INT_FUNCTION
   #define OS8104_INT_DISABLE       OS8104_IIC_INT_DISABLE
   #define OS8104_INT_ENABLE        OS8104_IIC_INT_ENABLE
   #define OS8104_AINT_IN           OS8104_IIC_AINT_IN
   #define OS8104_AINT_FUNCTION     OS8104_IIC_AINT_FUNCTION
   #define OS8104_AINT_DISABLE      OS8104_IIC_AINT_DISABLE
   #define OS8104_AINT_ENABLE       OS8104_IIC_AINT_ENABLE
#endif

// Kommunikation über Parallelport mit OS8104
#if (CP_CMDDEVICE == OS8104_PAR)
	#define Write_MO_Ctrl_Data       PAR_Write_OS8104_Data

   #define Read_OS8104_Data         PAR_Read_OS8104_Data
   #define Write_OS8104_Register    PAR_Write_OS8104_Register
   #define Read_OS8104_Register     PAR_Read_OS8104_Register

   #define Init_OS8104              PAR_Init_OS8104

   #define OS8104_INT_FUNCTION      OS8104_PAR_INT_FUNCTION
   #define OS8104_INT_DISABLE       OS8104_PAR_INT_DISABLE
   #define OS8104_INT_ENABLE        OS8104_PAR_INT_ENABLE
   #define OS8104_AINT_IN           OS8104_PAR_AINT_IN
   #define OS8104_AINT_FUNCTION     OS8104_PAR_AINT_FUNCTION
   #define OS8104_AINT_DISABLE      OS8104_PAR_AINT_DISABLE
   #define OS8104_AINT_ENABLE       OS8104_PAR_AINT_ENABLE
#endif

// Kommunikation über OS81050
#if (CP_CMDDEVICE == OS81050)
	#define Write_MO_Ctrl_Data       INIC_Write_MO_Ctrl_Data
   #define Send_MO_Ctrl_Data        INIC_Send_MO_Ctrl_Data
#endif

// Kommunikation über OS81110
#if (CP_CMDDEVICE == OS81110)
   UINT8    MOSTMSG_RX_Buf[INIC_SPI_PACKETSIZE];
#endif


//****************************************************************************
//* Function     :  Send_OS8104_Data                                         *
//****************************************************************************
//* Description  :  Messagebuffer verschicken und Erfolg testen              *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  : 	Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      : 	Buffer sind korrekt beladen                             *
//*						Full Duplex ist schwierig wegen dem RBE-Flag !!         *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
UINT32 Send_OS8104_Data (UINT32 Laenge)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Quadlets;
UINT32   Error;
UINT8    bPSTX;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Most Packet Data verschicken
   // Packetdatenlaenge in Quadlets errechnen und eintragen
   if (Laenge > PACKETSIZE)
   {  // Maximum begrenzen
      Quadlets = 0x0D;
   }
   else
   {  // Targetadresse +Packetdatenlaengenbyte addieren
      Laenge += 3;

      Quadlets = (UINT8)(Laenge & 0x000000FF) >>2;

      // aufrunden
      if (Laenge & 0x03)
         Quadlets++;
   }

   // und mit Interruptschutz eintragen
   OS8104_AINT_DISABLE;
   OS8104_INT_DISABLE;

   // letzte Sendung schon fertig ?
   timSetMarker(TIMER_TX_GLOBAL);
   do
   {
      Error |= Read_OS8104_Register (OS8104_bPSTX, &bPSTX);
      // auf "Message Transmitted" warten maximal bis zum Timeout von 10 ms
      if (timTimeout(TIMER_TX_GLOBAL, 10))
      {  // Fehler hochmelden
   	   return (Error | MOSTMSG_TX_TARGET_RESPONSE_ERROR);
      }
   }
   while (bPSTX & 0x80);

   Error |= Write_OS8104_Register (OS8104_bPLDT, Quadlets);

   // ASTX im Packet Start Register setzen und damit die Übertragung starten
   Error |= Write_OS8104_Register (OS8104_bPSTX, 0x80);

   // ab jetzt Interrupt erlaubt
   OS8104_AINT_ENABLE;
   OS8104_INT_ENABLE;

   // eventuelle Interrupt-Fehler mit einsammeln
   return (MOSTMSG_INT_Error | Error);
}  // end of Send_OS8104_Data
#endif

//****************************************************************************
//* Function     :	Send_OS8104_CtlMessage                                  *
//****************************************************************************
//* Description  :  Controll Messagebuffer verschicken und Erfolg testen     *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  : 	Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      : 	Buffer sind korrekt beladen                             *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
UINT32  Send_OS8104_CtlMessage (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   Error;
UINT8    bMSGS,
			bXTS;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // alle alten Fehler loeschen, aber RX nicht beeintraechtigen
   // und  "Message Transmitted Interrupt" zurücksetzen
	// STX im Message Controll Register setzen und damit die Übertragung starten
   Error = Write_OS8104_Register (OS8104_bMSGC, 0x8E);

 	timSetMarker(TIMER_TX_GLOBAL);
	// Statusregister pollen maximal bis zum Timeout 0.1 s
   do
   {  // auf "Message Transmitted" warten
    	Error |= Read_OS8104_Register  (OS8104_bMSGS, &bMSGS);
		if (timTimeout(TIMER_TX_GLOBAL, 100))
		   return (Error | MOSTMSG_TX_TIMEOUT_ERROR);
   }
	while ((bMSGS & 0x02) == 0);

	// erstmal  "Message Transmitted Interrupt" zurücksetzen ohne Rücksicht auf das RBE-Flag
	Error |= Write_OS8104_Register (OS8104_bMSGC, 0x02);

   //	auf Fehler untersuchen
   if ((bMSGS & 0x40) != 0x40)
	{ 	// Fehlerart bestimmen
		Error |= Read_OS8104_Register (OS8104_bXTS, &bXTS);

      if (Error == NO_ERROR)
		   switch  (bXTS)
		   {	case 0x00 : return (MOSTMSG_TX_TARGET_RESPONSE_ERROR);
			   case 0x11 : return (MOSTMSG_TX_XMIT_TYPE_ERROR);
	  		   case 0x20 : return (MOSTMSG_TX_BAD_CRC_ERROR);
		 	   case 0x21 : return (MOSTMSG_TX_REC_BUFFER_FULL_ERROR);
         }
   } // of if bMSGS & 0x40

	return (Error);
}  // end of Send_OS8104_CtlMessage
#endif

//****************************************************************************
//* Function     :	Send_Paket                                              *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR) || (CP_CMDDEVICE == OS81050))
UINT32 Send_Paket (UINT8 Channel, UINT32 Laenge, UINT8 *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 	Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   #if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
   OS8104_AINT_DISABLE;
   OS8104_INT_DISABLE;
   #endif

   // Sendebuffer beladen
 	Error |= Write_MO_Ctrl_Data (Laenge , Buffer);

   // und eine Message verschicken
   Error |= Send_MO_Ctrl_Data (Laenge);

   // bei Fehler abbrechen
 	if ((Error != NO_ERROR) && (Error != MOSTMSG_TX_TARGET_RESPONSE_ERROR))
	{
      // Neustart der Schnittstelle nur bei internen Problemen
      MOSTMSG_Init(Channel);
   }

	return (Error);
} // end of function Send_Paket
#endif

//****************************************************************************
//* Function     :	Spi_RW_1Byte                                            *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if (CP_CMDDEVICE == OS81110)
void Spi_RW_1Byte (UINT8 TX, UINT8* RX)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*
   // strukturtreuer Treiber
   // Wert schreiben und lesen
   //SpiWrite_1Word (INIC_SPI_CHANNEL, (UINT8)TX);
   //SpiReadMaster_1Char (INIC_SPI_CHANNEL, RX);

   // wenn CSIG SPI verwendet wird        1 Datenbyte lesen/schreiben
   SpiWrite_String_CSIG(INIC_SPI_CHANNEL, 1, &TX, RX);
}
#endif

//****************************************************************************
//* Function     :	Spi_WriteMessage                                        *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if (CP_CMDDEVICE == OS81110)
BOOL Spi_WriteMessage (UINT8 Channel, UINT8 *Buffer, UINT32 len)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // MDP-Header schreiben
   // PML
   Spi_RW_1Byte ((UINT8)(((len+8)>>8) & 0xFF), NULL);
   Spi_RW_1Byte ((UINT8)((len+8) & 0xFF), NULL);

   // PMHL
   Spi_RW_1Byte (0x05, NULL);

   // FPH
   Spi_RW_1Byte (0x0C, NULL);

   // Prio
   Spi_RW_1Byte (0x01, NULL);

   // Targetaddr
   Spi_RW_1Byte ((UINT8)((Gateway_MOST[Channel].TargetNodeadress >>8) & 0xFF), NULL);
   Spi_RW_1Byte ((UINT8)(Gateway_MOST[Channel].TargetNodeadress & 0xFF), NULL);

   // Filler
   Spi_RW_1Byte (0x00, NULL);

   // PMB
   Spi_RW_1Byte ((UINT8)((len >>8) & 0xFF), NULL);
   Spi_RW_1Byte ((UINT8)(len & 0xFF), NULL);

   // Data
   for (i= 0; i<len ; i++ )
   {
      Spi_RW_1Byte (*Buffer++, NULL);
   }

   return (TRUE);
} // of function
#endif

//****************************************************************************
//* Function     :	Spi_ReadMessage                                         *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if (CP_CMDDEVICE == OS81110)
BOOL Spi_ReadMessage (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    PMLL,
         PMLH;
UINT32   i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // MDP-Header lesen
   // PML
   Spi_RW_1Byte (0, &PMLH);
   Spi_RW_1Byte (0, &PMLL);

   // Anzahl der Daten in der Message
   MOSTMSG_RX_Count = (((UINT32)PMLH<<8) + (UINT32)PMLL) -8;

   // Plausibilitaetstest gegen Bufferabstuerze
   if ((MOSTMSG_RX_Count < 1) || (MOSTMSG_RX_Count > INIC_SPI_PACKETSIZE))
   {
      return (FALSE);
   }

   // 8 Bytes MDP Info vorspulen,
   for (i= 0; i<8 ; i++ )
   {
      Spi_RW_1Byte (0, NULL);
   }

   // Data lesen
   for (i= 0; i< MOSTMSG_RX_Count ; i++ )
   {
      Spi_RW_1Byte (0, MOSTMSG_RX_Buf+i);
   }

   // Comm modul bedienen
   msgReceive (Gateway_MOST_CommIndex[MOST_Selected_Channel], MOSTMSG_RX_Buf, MOSTMSG_RX_Count);

   return (TRUE);
} // of function
#endif

//****************************************************************************
//* Function     :	MOST_Send                                               *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR) || (CP_CMDDEVICE == OS81050))
BOOL MOST_Send (UINT8 Channel, UINT8 *Buffer, UINT32 len)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

static UINT8   *pTX_Start,
               *pTX_Stop;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (MOSTMSG_TX_State == MOSTMSG_TX_ENABLED)
   {
      // Anfang der Message; Startwerte setzen
      pTX_Start = Buffer;	         // Zeiger auf erstes Nutzbyte
      pTX_Stop  = Buffer + len; 	   // Zeiger auf erste freie Bufferadresse nach den Nutzdaten
   }

   // Sendeauftrag merken
   MOSTMSG_TX_State = MOSTMSG_TX_BUSY;

   // Credit geben
   MOSTMSG_PollCredits = POLL_CREDITS;

   // Timer neu aufziehen
   timSetMarker (TIMER_MOST_DEV_0);

   // zurueck; auf Handshake von der Gegenseite warten
   if (MOSTMSG_RX_State == MOSTMSG_RX_ENABLED)
      return(FALSE);

   MOSTMSG_RX_State = MOSTMSG_RX_ENABLED;

//*--------------------------------------------------------------------------*
//*	jetzt ein Packet-Einzelteil verschicken  	                             *
//*--------------------------------------------------------------------------*

   if (pTX_Stop > pTX_Start)
	{  // keine weitere Fehlerbehandlung
      Send_Paket (Channel, pTX_Stop - pTX_Start , pTX_Start);
      pTX_Start += PACKETSIZE;
   }

   // Message komplett ?
   if (pTX_Stop <= pTX_Start)
   {
      MOSTMSG_TX_State = MOSTMSG_TX_ENABLED;
      return(TRUE);
   }
   else
     return (FALSE);
      } // end of function MOST_Send
#endif

#if (CP_CMDDEVICE == OS81110)
BOOL MOST_Send (UINT8 Channel, UINT8 *Buffer, UINT32 len)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Wert;
BOOL     Result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   INIC_SPI_CS_ACTIVATE;

   // Status Schreibanforderung
   Spi_RW_1Byte (0x06, &RX_Wert);

   if (RX_Wert & 0x02)
   {  // Message schicken
      Result = Spi_WriteMessage (Channel, Buffer, len);
   }
   else
      Result = FALSE;

   INIC_SPI_CS_DEACTIVATE;

   // SPI INT auf Leseanforderung setzen
   INIC_SPI_CS_ACTIVATE;
   Spi_RW_1Byte (0x01, &RX_Wert);
   INIC_SPI_CS_DEACTIVATE;

   return(Result);
} // end of function MOST_Send
#endif

//****************************************************************************
//* Function     :	MOST_0_Receive                                          *
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
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR) || (CP_CMDDEVICE == OS81050))
void MOST_0_Receive (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Get new Message if INIC
   #if (CP_CMDDEVICE == OS81050)
   if (!MostMsgRunning())
      return;

   inicProceedNewMsg();
   INIC_Get_MO_Ctrl_Data();
   #endif

   // Get new Message if OS8104
   #if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
   if (!OS8104_AINT_IN)
   {
      OS8104_Get_MO_Ctrl_Data();
   }
   #endif

//*--------------------------------------------------------------------------*
//* Handshake Slave                                                          *
//* ===============                                                          *
//* Es wird nur geantwortet und niemals von selber gesendet                  *
//*--------------------------------------------------------------------------*

   // Interrupt Flag fuer RX-Buffer testen
   if ((Gateway_MOST[MOST_Selected_Channel].Format == MOSTFORMAT_PACKET_SLAVE) && MOSTMSG_RX_NewData)
   {
      if (MOSTMSG_RX_Count > 0)
      {  // Comm modul bedienen
         msgReceive (Gateway_MOST_CommIndex[MOST_Selected_Channel], MOSTMSG_RX_Buf, MOSTMSG_RX_Count);
      }

      MOSTMSG_RX_NewData = FALSE;
      MOSTMSG_RX_State   = MOSTMSG_RX_RECEIVED;

      if (MOSTMSG_TX_State == MOSTMSG_TX_ENABLED)
      {  // Sync senden (0-Laenge)
         Send_Paket (MOST_Selected_Channel, 0 , NULL);
         MOSTMSG_RX_State = MOSTMSG_RX_ENABLED;
      }

      return;
   }  // if Slave

//*--------------------------------------------------------------------------*
//* Handshake Master                                                         *
//* ================                                                         *
//* Es wird hier geantwortet                                                 *
//* Fuer jedes SYNC vom Slave wird ein Credit abgezogen                      *
//* Bei 0 Credits wird kein SYNC mehr verschickt                             *
//*--------------------------------------------------------------------------*

   if (Gateway_MOST[MOST_Selected_Channel].Format == MOSTFORMAT_PACKET_MASTER)
   {  // hier nur Master
      if (MOSTMSG_RX_NewData)
      {
         if (MOSTMSG_RX_Count > 0)
         {  // Comm modul bedienen
            msgReceive (Gateway_MOST_CommIndex[MOST_Selected_Channel], MOSTMSG_RX_Buf, MOSTMSG_RX_Count);
            // Credit geben
            MOSTMSG_PollCredits = POLL_CREDITS;
         }
         else
         {  // Sync empfangen (0-Laenge)
            // Credit abziehen
            if (MOSTMSG_PollCredits > 0)
               MOSTMSG_PollCredits--;
         }

         MOSTMSG_RX_NewData = FALSE;
         MOSTMSG_RX_State   = MOSTMSG_RX_RECEIVED;

         // nur fuer Master
         if ((MOSTMSG_TX_State == MOSTMSG_TX_ENABLED) && (MOSTMSG_PollCredits > 0))
         {  // Sync senden (0-Laenge)
            Send_Paket (MOST_Selected_Channel, 0 , NULL);
            MOSTMSG_RX_State = MOSTMSG_RX_ENABLED;
         }

         // Timer neu aufziehen
         timSetMarker (TIMER_MOST_DEV_0);
         return;
      } // if MOSTMSG_RX_NewData

//*--------------------------------------------------------------------------*
//* Handshake Master Trigger                                                 *
//* ========================                                                 *
//* Nach jedem Timeout wird ein SYNC verschickt                              *
//*--------------------------------------------------------------------------*

      if (timTimeout(TIMER_MOST_DEV_0, POLL_TIMEOUT))
      {  // Timer neu aufziehen
         timSetMarker (TIMER_MOST_DEV_0);
         // Sync senden (0-Laenge)
         Send_Paket (MOST_Selected_Channel, 0 , NULL);
         MOSTMSG_RX_State = MOSTMSG_RX_ENABLED;
      }

      return;
   }  // if Master

} // end of function MOST_0_Receive
#endif

#if (CP_CMDDEVICE == OS81110)
void MOST_0_Receive (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    RX_Wert;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Kontrolle  SPI INT, muss fuer RX 0 sein
   if (INIC_SPIINT_PIN)
      return;

   INIC_SPI_CS_ACTIVATE;

   // Status Leseanforderung
   Spi_RW_1Byte (0x01, &RX_Wert);

   if (RX_Wert & 0x05)
   {  // Message lesen
      Spi_ReadMessage (MOST_Selected_Channel);
   }

   INIC_SPI_CS_DEACTIVATE;

   // SPI INT auf neue Leseanforderung setzen
   INIC_SPI_CS_ACTIVATE;
   Spi_RW_1Byte (0x01, &RX_Wert);
   INIC_SPI_CS_DEACTIVATE;

   return;
}
#endif

//****************************************************************************
//* Function     :	MOSTMSG_Init                                            *
//****************************************************************************
//* Description  :  OS8104 initialisieren und Variable rücksetzen            *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :  Error                                                    *
//*                                                                          *
//* Changed Var. :  MOSTMSG_RX_State                                         *
//*                 MOSTMSG_TX_State                                         *
//*						                                                        *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

// Einzelbits zum Verodern
#define INIT_TRANSCEIVER         0x0001


UINT32 MOSTMSG_Init (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 	Error;
UINT16   InitChecklist;

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
UINT8    bPCTS,
         bNPR;
#endif

#if (CP_CMDDEVICE == OS81110)
UINT8    Dummy;
#endif

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Sicherheitshalber
   Channel = DEVICE_NR(Channel);

   // Init Checkliste  erstellen
   if (MOST_Selected_Channel == NONE)
     // Kaltstart nach Poweron
     InitChecklist = INIT_TRANSCEIVER;
   else
   {
      // Sicherheitshalber
      MOST_Selected_Channel = DEVICE_NR(MOST_Selected_Channel);
      InitChecklist = 0x0000;

      if (Gateway_MOST[Channel].Clock != Gateway_MOST[MOST_Selected_Channel].Clock)
         InitChecklist |= INIT_TRANSCEIVER;
   }

   // Kanalnummer jetzt fuers naechste Mal merken
   MOST_Selected_Channel = Channel;

//*--------------------------------------------------------------------------*

   #if (CP_DEBUG == 20)
   RS232_Init (0, 115200);
   Debug_Format_Print ("MOSTMSG_Init "  );
   Debug_Format_Print ("\r\n\r\n");
   #endif

   #if ((CP_CMDDEVICE == OS81050) || (CP_CMDDEVICE == OS81110))
   mostMsgInitialized = FALSE;
   #endif

   // Statemaschinen initialisieren
   MOSTMSG_RX_State = MOSTMSG_RX_DISABLED;
 	MOSTMSG_TX_State = MOSTMSG_TX_DISABLED;

   Error = NO_ERROR;

//*--------------------------------------------------------------------------*
//* OS81xx0 initialisieren                                                   *
//*--------------------------------------------------------------------------*

   #if ((CP_CMDDEVICE == OS81110) || (CP_CMDDEVICE == OS81050))
   printf("** MOSTMsg-Init:\n");
   INIC_FOTON_ACTIVATE;
   // carry out reset sequence for OS81050/OS81110
   // should be done in Autorun with command 'X 23 00 01 '
   Error = inicReInit();

   // break at error
   if(Error != NO_ERROR)
   {
      printf("     Error while HwInit OS81050/OS81110: 0x%08X\n", Error);
      return (Error);
   }
   else
   {
      printf("     HwInit of OS81050/OS81110 successful\n");
   }

   #if (INIC_SPI_CONFIF_MBC)
   // SPI schliessen , falls schon geoeffnet
   //if (!SerClose(INIC_SPI_CHANNEL))  Error = ERROR_SER | SER_ERR_CLOSE;
     if (!CsigClose(INIC_SPI_CHANNEL)) Error = ERROR_SER | SER_ERR_CLOSE;

   // SPI fuer Packetdaten oeffnen
   // SPI 2 NEC Master SPI Mode 0 + CS 8 MBaud, no waits
   //SerOpen(INIC_SPI_CHANNEL, INIC_SPI_PACKET_BAUDRATE, 0x21, FALSE, 0, 0, 0, 0);
   CsigOpen(INIC_SPI_CHANNEL, INIC_SPI_PACKET_BAUDRATE, INIC_SPI_PROTOCOL);

   // SPI INT auf neue Leseanforderung setzen
   INIC_SPI_CS_ACTIVATE;
   Spi_RW_1Byte (0x01, NULL);
   INIC_SPI_CS_DEACTIVATE;
   #endif

      // wichtig fuer Unlocksteuerung
      if (Gateway_MOST[Channel].Clock == MOSTCLOCK_SLAVE)
         MOST_State = MOST_SLAVE_OK;
      else
         MOST_State = MOST_MASTER_OK;
   #endif

//*--------------------------------------------------------------------------*
//* Initialisierung vervollständigen                                         *
//*--------------------------------------------------------------------------*

   // Fehlerspeicher für Interruptroutine rücksetzen
	MOSTMSG_INT_Error = NO_ERROR;

	MOSTMSG_RX_NewData = FALSE;

	#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR) || (CP_CMDDEVICE == OS81050))
   // Timer starten
   timSetMarker (TIMER_MOST_DEV_0);
   #endif

   // System ist ab jetzt  initialisiert
   MOSTMSG_RX_State = MOSTMSG_RX_ENABLED;
 	MOSTMSG_TX_State = MOSTMSG_TX_ENABLED;

 	#if (CP_CMDDEVICE == OS81050)
 	mostMsgInitialized = TRUE;
   #endif

   #if (CP_CMDDEV_MOST & DEV_0)
   // beim Comm-Modul anmelden
   if (Channel == 0)
      Gateway_MOST_CommIndex[Channel] = msgRegister(MOST+Channel, "MOST", Channel, 0, MOST_0_Receive, MOST_Send, NULL);
   #endif

   #if (CP_CMDDEVICE == OS81110)
   // SPI INT auf Leseanforderung setzen
   INIC_SPI_CS_ACTIVATE;
   Spi_RW_1Byte (0x01, &Dummy);
   INIC_SPI_CS_DEACTIVATE;

   timStopTimer(TIMER_MOST_DEV_0);
   #endif

   return (Error);
} // end of function MOSTMSG_Init


//-------------------------------------------------------------------------
//  Function     :  Write_MO_Ctrl_Data
//-------------------------------------------------------------------------
//  Description  :  writes transmit control data set to data area of mostmsg
//                  control data 0 is the length of the message
//                  packet
//
//  Parameters   :  pointer to buffer, length
//
//  Return Value :  error code
//
//  Changed Var. :
//
//  Comment      :
//-------------------------------------------------------------------------
//  Quality      :     (x) not tested  ( ) partly tested  ( ) fully tested
//-------------------------------------------------------------------------

#if (CP_CMDDEVICE == OS81050)
UINT32 INIC_Write_MO_Ctrl_Data(UINT32 Laenge, UINT8 *buf)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

 	UINT8	   Offset;
   UINT32   Zaehler;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Begrenzung
   if (Laenge > PACKETSIZE)
      Laenge = PACKETSIZE;

   // set to start af data area
   Offset = INIC_MDP_STARTOFDATA;

   // als erstes Längenangabe getreu dem Paketprotokoll schreiben
   InicOutMsg[Offset++]= (UINT8)(Laenge & 0x000000FF);

   // write data area of most-control-message
   for (Zaehler  = 0; Zaehler < Laenge ; Zaehler++)
      InicOutMsg[Offset++] = *buf++;

   return (NO_ERROR);
} // end of function INIC_Write_MO_Ctrl_Data
#endif

//-------------------------------------------------------------------------
//  Function     :   INIC_Send_MO_Ctrl_Data via OS81050
//-------------------------------------------------------------------------
//  Description  :  send message buffer to INIC
//
//  Parameters   :  -none-
//
//  Return Value :  error code
//
//  Changed Var. :  -none-
//
//  Comment      :  Buffer sind korrekt beladen
//
//-------------------------------------------------------------------------
//  Quality      :     (x) not tested  ( ) partly tested  ( ) fully tested
//-------------------------------------------------------------------------

#if (CP_CMDDEVICE == OS81050)
UINT32 INIC_Send_MO_Ctrl_Data(UINT32 Laenge)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

   UINT32   Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // calculate protocol length
   if (Laenge > PACKETSIZE)
   {  // Maximum begrenzen
      Laenge = PACKETSIZE;
   }

   // create Fifo Protocol Header
   writeWord(InicOutMsg, (UINT8)(Laenge & 0x000000FF)+9);       // PML
   writeWord(InicOutMsg+8, (UINT8)(Laenge & 0x000000FF)+1);     // PMB Length

   InicOutMsg[2]  = 0x05;     // PMHL
   InicOutMsg[3]  = INIC_FPH_FIFONO_MDP | INIC_FPH_FIFOMT_DAT;  // FPH
   InicOutMsg[4]  = 0x01;     // PRIO
   InicOutMsg[7]  = 0x00;     // Fillbyte

   if (inicUMostTrgtAddr != 0x0000U)
      writeWord(InicOutMsg+5, inicUMostTrgtAddr);
   else
   {
      if (Gateway_MOST[0].TargetNodeadress == 0)
         writeWord(InicOutMsg+5, MOST_BROADCAST_GROUPADDR);
      else
         writeWord(InicOutMsg+5, Gateway_MOST[0].TargetNodeadress);
   }

   inicReadReqMsgs();

   // send Msg to Inic
   Error = inicSendAndWaitAck(inic_MostMsgBuf, InicOutMsg);

   return(Error);
}
#endif


//-------------------------------------------------------------------------
//  Function     :  INIC_Get_MO_Ctrl_Data
//-------------------------------------------------------------------------
//  Description  :  service routine for MOST
//
//  Parameters   :  -none-
//
//  Returnvalue  :  -none-
//
//  Changed Var. :  MOSTMSG_INT_Error
//
//  Comment      :
//-------------------------------------------------------------------------
//  Quality      :     (x) not tested  ( ) partly tested  ( ) fully tested
//-------------------------------------------------------------------------

#if (CP_CMDDEVICE == OS81050)
void INIC_Get_MO_Ctrl_Data(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // *******************************************
   // * throw away all AckMsg coming from Inic *
   // *******************************************

   while (1)
   {
      // ****************************
      // * read msg from ringbuffer *
      // ****************************
      if (inicMsgRead(inic_MostMsgBuf, InicIncMsg) == 0)
      {  // no more messages available in buffer
         return;
      }
      // ********************************
      // * Filter out AckMsgs from Inic *
      // ********************************
      // Acknowledge from Inic
      if (inicTestIfAckMsg(InicIncMsg)==FALSE)
         break;
   }

   // Message now available in InicIncMsg

   // **************************************
   // **Analyze header to get control-Data *
   // **************************************

   // Test for MDP; Packet Data Length is volatile
   if (((InicIncMsg[3U] & INIC_FPH_FIFONO_MASK) == INIC_FPH_FIFONO_MDP) &&
       ((InicIncMsg[3U] & INIC_FPH_FIFOMT_MASK) == INIC_FPH_FIFOMT_DAT))
   {  // Incoming Msg correct for Mostmsg:
      if (Gateway_MOST[0].TargetNodeadress == 0)
         // Take the new received trgt addr to send answer
         inicUMostTrgtAddr = readWord(InicIncMsg+5);
   }
   else
      return;


   if (!MOSTMSG_RX_NewData)
   {  // Mostmessage abholen
      MOSTMSG_INT_Error |= INIC_Read_MO_Ctrl_Data(&MOSTMSG_RX_Count, &InicIncMsg[INIC_MDP_STARTOFDATA], MOSTMSG_RX_Buf);
      MOSTMSG_RX_NewData = TRUE;
   }

}  // end of INIC_Get_MO_Ctrl_Data
#endif

//****************************************************************************
//* Function     :   Interruptroutine für OS8104 Packet Data                 *
//****************************************************************************
//* Description  :   Interrupt P2.xx bearbeiten  (CAPCOMxx)                  *
//*                  Fehler innerhalb vom Interrupt auswerten                *
//*						TX oder RX Int moeglich                                 *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :   MOSTMSG_INT_Error                                       *
//*                                                                          *
//* Comment      :	Interrupt wird  bei Fehler ausgeschaltet                *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
void OS8104_Get_MO_Ctrl_Data (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    bPCTS;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   OS8104_INT_DISABLE;

   // Statusregister im MOST-Chip auslesen
   MOSTMSG_INT_Error |= Read_OS8104_Register (OS8104_bPCTS, &bPCTS);

//*--------------------------------------------------------------------------*

   if (bPCTS & 0x10)
   {  // RX-Overflow ; Message einfach verwerfen; kein Restart erforderlich
      // Flag ruecksetzen
      MOSTMSG_INT_Error |= Write_OS8104_Register (OS8104_bPCTC,0x10);
   }

   if (bPCTS & 0x08)
   {  // CRC-Fehler ; Message einfach verwerfen; kein Restart erforderlich
      // Flag ruecksetzen
      MOSTMSG_INT_Error |= Write_OS8104_Register (OS8104_bPCTC,0x08);
   }

//*--------------------------------------------------------------------------*

   // nachsehen, ob TX Interrupt
   if (bPCTS & 0x02)
   {
      // TX-Flag ruecksetzen
      MOSTMSG_INT_Error |= Write_OS8104_Register (OS8104_bPCTC,0x02);
   }

//*--------------------------------------------------------------------------*

   // nachsehen, ob RX Interrupt
   if (bPCTS & 0x01)
   {
      if (!MOSTMSG_RX_NewData)
      {  // Mostmessage abholen
         MOSTMSG_INT_Error |= Read_OS8104_Data (&MOSTMSG_RX_Count, MOSTMSG_RX_Buf);
         MOSTMSG_RX_NewData = TRUE;
      }

      // Unlock RX-Buffer
      MOSTMSG_INT_Error |= Write_OS8104_Register (OS8104_bPCTC,0x01);
   } // if (bPCTS & 0x01)

   OS8104_INT_ENABLE;

}  // end of OS8104_Get_MO_Ctrl_Data
#endif

//****************************************************************************
//* Function     :   Interruptroutine für OS8104 Mostmessage                 *
//* alternativ   :   Routine für OS880x                                      *
//****************************************************************************
//* Description  :   Interruptrumpf; bei Bedarf mit Leben fuellen            *
//*                  Fehler innerhalb vom Interrupt auswerten                *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :   MOSTMSG_INT_Error                                       *
//*                                                                          *
//* Comment      :	Interrupt wird  bei Fehler ausgeschaltet                *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
#pragma ghs interrupt
void OS8104_INT_FUNCTION (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 bMSGS;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Statusregister im MOST-Chip auslesen
   MOSTMSG_INT_Error |= Read_OS8104_Register  (OS8104_bMSGS, &bMSGS);

   // nachsehen, ob die Interruptquelle stimmt
   if ((bMSGS & 0x01) != 0x01)
      MOSTMSG_INT_Error |= MOSTMSG_RX_UNKNOWNINT_ERROR;

   // Einlesen der Most Controlmessage fuer z-Befehl
   MOSTMSG_INT_Error |= Read_OS8104_AllData ();

//*--------------------------------------------------------------------------*
//*	MOST-Chip wieder bereit machen                                         *
//*--------------------------------------------------------------------------*

   // Auf jeden Fall Interrupt im OS8104 zurücksetzen
   MOSTMSG_INT_Error |= Write_OS8104_Register (OS8104_bMSGC, 0x41);

}  // end of OS8104_INT_FUNCTION
#endif

//-------------------------------------------------------------------------
//  Function     : mostMsgRunning
//-------------------------------------------------------------------------
//  Description  : This function returns the init state for most command
//                 channel.
//
//  Parameter    :

//
//  Returnvalue  : BOOL- true if mostmsg-channel is running
//
//  Changed Var. : -none-
//
//  Comment      :
//-------------------------------------------------------------------------
//  Quality      : ( ) not tested  ( ) partly tested  ( ) fully tested
//-------------------------------------------------------------------------

#if ((CP_CMDDEVICE == OS81050) || (CP_CMDDEVICE == OS81110))
BOOL MostMsgRunning(void)
{
   if (!mostInitialized) return(FALSE);
   if (!inicHwInitialized) return(FALSE);
   if (!mostMsgInitialized) return(FALSE);
   return(TRUE);
}
#endif

//-------------------------------------------------------------------------
//  Function     :  INIC_Read_MO_Ctrl_Data
//-------------------------------------------------------------------------
//  Description  :  reads received control data set 1 - 16
//                  control data 0 interpreted as length
//
//  Parameters   :  pointer to length,
//                  pointer to SrcBuffer,
//                  pointer to DstBuffer,
//
//  Return Value :  error code
//
//  Changed Var. :
//
//  Comment      :
//-------------------------------------------------------------------------
//  Quality      :     (x) not tested  ( ) partly tested  ( ) fully tested
//-------------------------------------------------------------------------

#if (CP_CMDDEVICE == OS81050)
UINT32 INIC_Read_MO_Ctrl_Data(UINT32 *Laenge, UINT8 *src, UINT8 *buf)
{
   //----------------------------------------------------------------------
   //  Local Variables
   //----------------------------------------------------------------------
   UINT32   Zaehler;

   //----------------------------------------------------------------------
   // Start of Function
   // Most Format 3 or 4
   //----------------------------------------------------------------------

	// read length information first
   *Laenge = (UINT32)*src++;

   // Begrenzung
   if (*Laenge > PACKETSIZE)
      *Laenge = PACKETSIZE;

   // alle Restdatenbytes abholen
   for(Zaehler = 0; Zaehler < *Laenge ; Zaehler++)
      *buf++ = *src++;

 	return(NO_ERROR);
} // end of function INIC_Read_MO_Ctrl_Data
#endif


//****************************************************************************
//* Function     :	MOSTMSG_WR                                              *
//****************************************************************************
//* Description  :  OS8104 ansprechen                                        *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//*						                                                        *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
void MOSTMSG_WR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32  Error;
UINT16   i;
static UINT8   Last_OS8104_bXCR_Value;
static UINT8   Last_MOST_State;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // check for correct command
   if (((commandPtr[3] == 0x00) && (commandPtr[0] == 4)) ||
       ((commandPtr[3] == 0x01) && (commandPtr[0] == 4)) ||
       ((commandPtr[3] == 0x01) && (commandPtr[0] == 5)) ||
       ((commandPtr[3] == 0x02) && (commandPtr[0] >= 3)) ||
       ((commandPtr[3] == 0x03) && (commandPtr[0] == 30))||
       ((commandPtr[3] == 0x80) && (commandPtr[0] == 5)) ||
       ((commandPtr[3] == 0x81) && (commandPtr[0] == 5) && (commandPtr[4] == 0x00)) ||
       ((commandPtr[3] == 0x81) && (commandPtr[0] == 5) && (commandPtr[4] == 0x01))
       )
   {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

//*--------------------------------------------------------------------------*

   switch(commandPtr[3])
   {
      case 0x00:
         // angelegten Buffer löschen
         bufDelete(MOSTMSG_Buf);
         // sicherheitshalber
         MOSTMSG_Buf = NULL;
         break;

//*--------------------------------------------------------------------------*

      case 0x01:
         printf("** MOST-Messagebuffer : Initializing\n");

         // Speicher auf Heap reservieren
 	      // vorher eventuell schon angelegten Buffer löschen
 	      bufDelete(MOSTMSG_Buf);

         // Buffer fuer Packet Data
         if ((MOSTMSG_Buf = bufCreate(MOSTMSG_BUF_SIZE)) == NULL)
 	      {
 	         printf("     No Memory available for MOST Control or Packet buffer\n");
 	         Error = MOSTMSG_RX_BUFFERALLOCATE_ERROR;
 	      }

         MOSTMSG_Messagezaehler = 0;

         // wenn noch kein Most gestartet wurde, dann defaultmaessig Kanal 0 initialisieren
         if (MOSTMSG_RX_State == MOSTMSG_RX_DISABLED)
            Error = MOSTMSG_Init(0);

 	      // report success
 	      printf("     Done!\n");
         break;

//*--------------------------------------------------------------------------*

      case 0x02:
         // Zugriff nur, wenn auch initialisiert
         if (MOSTMSG_Buf == NULL)
         {
            Error = MOSTMSG_TX_ACCESS_ERROR;
            break;
         }

         // Sendebuffer im OS8104 beladen;
         // OS8104 Interrupt sicherheitshalber ausschalten wegen nicht reentranter Zugriffsfunktion
         OS8104_INT_DISABLE;
         OS8104_AINT_DISABLE;

         for (i=0; i<=20; i++)
            Error |= Write_OS8104_Register (OS8104_bXPRI+i, commandPtr[4+i]);

         // und abschicken
         Error |= Send_OS8104_CtlMessage ();

         OS8104_INT_ENABLE;
         OS8104_AINT_ENABLE;
         break;

//*--------------------------------------------------------------------------*

      case 0x03:
         // Zugriff nur, wenn auch initialisiert
         if (MOSTMSG_Buf == NULL)
         {
            Error = MOSTMSG_TX_ACCESS_ERROR;
            break;
         }

         // Sendebuffer im OS8104 beladen;
         // OS8104 Interrupt sicherheitshalber ausschalten wegen nicht reentranter Zugriffsfunktion
         OS8104_INT_DISABLE;
         OS8104_AINT_DISABLE;

         for (i=0; i<=20; i++)
            Error |= Write_OS8104_Register (OS8104_bXPRI+i, commandPtr[4+i]);

         // Vergleichsadresse laden
         MOSTMSG_Response_Address = readWord (commandPtr+6);
         MOSTMSG_Response = FALSE;

         // und abschicken
         Error |= Send_OS8104_CtlMessage ();

         OS8104_INT_ENABLE;
         OS8104_AINT_ENABLE;

         if (Error != MOSTMSG_NO_ERROR)
            break;

         // Wartezeit auf eventuelle Antwort
         timWait (readLong(commandPtr+26));

         // Ergebnis auswerten
         if (commandPtr[25] == 0)
         {  if (MOSTMSG_Response)
               Error = MOSTMSG_RESPONSE_TIMEOUT;
         }
         else
         {  if (!MOSTMSG_Response)
               Error = MOSTMSG_RESPONSE_TIMEOUT;
         }
         break;

//*--------------------------------------------------------------------------*

       case 0x80:
         // nichts tun, nur aus Kompatibilitaetsgruenden ohne Fehler schlucken
         break;

//*--------------------------------------------------------------------------*

      case 0x81:
         // nur bei initialisiertem MOST
         if (MOST_Selected_Channel != NONE)
         {  // OS8104 Interrupt sicherheitshalber ausschalten wegen nicht reentranter Zugriffsfunktion
            OS8104_INT_DISABLE;
            OS8104_AINT_DISABLE;

            if (commandPtr[4] == 0)
            {
               if (MOST_State != MOST_LIGHT_OFF)
               {  // letzten Wert merken
                  Read_OS8104_Register (OS8104_bXCR, &Last_OS8104_bXCR_Value);
                  // OE ausschalten, Bypass ausschalten, Slavebit belassen
                  Write_OS8104_Register (OS8104_bXCR, (Last_OS8104_bXCR_Value & ~0x40) | 0x02);

                  Last_MOST_State = MOST_State;
                  MOST_State = MOST_LIGHT_OFF;
                  LED_BUSLOCK_DEACTIVATE;
               }
            }

            if (commandPtr[4] == 1)
            {
               if (MOST_State == MOST_LIGHT_OFF)
               {  // letzten Wert wiederherstellen
                  Write_OS8104_Register (OS8104_bXCR, Last_OS8104_bXCR_Value);
                  MOST_State = Last_MOST_State;
               }
            }
            OS8104_INT_ENABLE;
            OS8104_AINT_ENABLE;
         }  // of MOST_Selected_Channel
         break;
   }  // of switch(commandPtr[3])

   // Ergebnis senden
   writeLong(resultPtr+4, Error);

} // end of function
#endif


//****************************************************************************
//* Function     :	MOSTMSG_RD                                              *
//****************************************************************************
//* Description  :  OS8104 ansprechen                                        *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//*						                                                        *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
void MOSTMSG_RD (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32  Error;
UINT16   i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // check for correct command
   if (((commandPtr[3] == 0x02) && (commandPtr[0] == 4)) ||
       ((commandPtr[3] == 0x01) && (commandPtr[0] == 4)))
   {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

//*--------------------------------------------------------------------------*

   // Zugriff nur, wenn auch initialisiert
   if (MOSTMSG_Buf == NULL)
   {
      Error = MOSTMSG_RX_ACCESS_ERROR;
   }
   else
      switch(commandPtr[3])
      {
         case 0x01:  // Disable  OS8104 Interrupts
                     OS8104_INT_DISABLE;

                      // Delete  all Messages in Buffer
                     bufFlush(MOSTMSG_Buf);
                     MOSTMSG_Messagezaehler = 0;

                     // Enable OS8104 Interrupt
                     OS8104_INT_ENABLE;
                     break;

         case 0x02:  // Disable Interrupts
                     OS8104_INT_DISABLE;

                     // Message da ?
                     if (MOSTMSG_Messagezaehler > 0)
                     {
                        MOSTMSG_Messagezaehler--;
                        resultPtr[8] = MOSTMSG_Messagezaehler;

                        // Message abholen
                        for (i = 0; i < MOSTMSG_SIZE; i++)
                           bufGet(MOSTMSG_Buf, resultPtr+9+i);

                        // Länge einsetzen
                        resultPtr [0] =  MOSTMSG_SIZE +9;
                     }
                     else
                     {  // No Message in Buffer , zwangsynchronisieren
                        bufFlush(MOSTMSG_Buf);
                     }

                     // Enable OS8104 Interrupt
                     OS8104_INT_ENABLE;
                     break;
      } // of switch


   // Ergebnis senden
   writeLong(resultPtr+4, Error);

} // end of function
#endif

//****************************************************************************
//* Function     :	Read_OS8104_AllData                                     *
//****************************************************************************
//* Description  :  OS8104 ansprechen                                        *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//*						                                                        *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

#if ((CP_CMDDEVICE == OS8104_IIC) || (CP_CMDDEVICE == OS8104_PAR))
UINT32 Read_OS8104_AllData (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   i,
         Error;
UINT16   Source_Address;
UINT8    Wert;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // nur, wenn auch initialisiert
   if (MOSTMSG_Buf == NULL)
      return (NO_ERROR);

   Error = NO_ERROR;
   Source_Address = 0;

   // wenn voll, dann ältesten Eintrag überschreiben
   if (bufSpace (MOSTMSG_Buf) < MOSTMSG_SIZE)
   {  // n Byte im Buffer freimachen und neu belegen
      for (i=0; i<MOSTMSG_SIZE; i++)
         bufGet (MOSTMSG_Buf, &Wert);
   }
   else
      MOSTMSG_Messagezaehler++;

//*--------------------------------------------------------------------------*

   // Message einspeichern
   for (i=0; i< MOSTMSG_SIZE; i++)
   {
      // Message aus OS8104 auslesen
      Error |= Read_OS8104_Register  (OS8104_bRTYP+i, &Wert);
      bufPutFiFo (MOSTMSG_Buf, Wert);

      // Senderadresse rausfischen
      if ((OS8104_bRTYP+i) == OS8104_bRSAH)
         Source_Address |= Wert<<8;

      if ((OS8104_bRTYP+i) == OS8104_bRSAL)
         Source_Address |= Wert;
   }

   // Vergleich der Adressen
   if (Source_Address == MOSTMSG_Response_Address)
      MOSTMSG_Response = TRUE;

   return(Error);
} // end of function Read_OS8104_AllData
#endif



#if (INIC_SPI_CONFIF_MBC)
void MOST_TestRing (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (!MOST1_ERR_ISACTIVE)
   {  // Most wieder durchschalten
      switch (MOST_State)
   {
         case MOST_SLAVE_OK :
         case MOST_MASTER_OK:
            // anzeigen
            LED_BUSLOCK_ACTIVATE;

            // Kommunikation neu anwerfen
            if (timTimeout(TIMER_MOST_DEV_0, REQ_READ_TIMEOUT))
            {
               // SPI fuer Packetdaten oeffnen, falls noch nicht geschehen
               // SPI 2 NEC Master SPI Mode 0 + CS 8 MBaud, no waits
               CsigOpen(INIC_SPI_CHANNEL, INIC_SPI_PACKET_BAUDRATE, INIC_SPI_PROTOCOL);

               // SPI INT auf neue Leseanforderung setzen
               INIC_SPI_CS_ACTIVATE;
               Spi_RW_1Byte (0x01, NULL);
               INIC_SPI_CS_DEACTIVATE;

               timStopTimer(TIMER_MOST_DEV_0);
            }
            break;

         case MOST_SLAVE_UNLOCK_DETECTED :
            // Leseanforderung bei Licht an verzoegert starten
            timSetMarker (TIMER_MOST_DEV_0);
            MOST_State = MOST_SLAVE_OK;
            break;

         case MOST_MASTER_UNLOCK_DETECTED :
         case MOST_MASTER_WAIT_RESTART:
            // Leseanforderung bei Licht an verzoegert starten
            timSetMarker (TIMER_MOST_DEV_0);
            MOST_State = MOST_MASTER_OK;
            break;
      } // of switch MOST_State
   }

//*--------------------------------------------------------------------------*

   else
	{
      // Most abbauen
      switch (MOST_State)
      {
         case MOST_MASTER_OK:
            LED_BUSLOCK_DEACTIVATE;
            // Fehlerzaehler bei jeder Aenderung hochzaehlen
            Gateway_MOST_Errorcounter++;
            MOST_State = MOST_MASTER_UNLOCK_DETECTED;
            break;

         case MOST_SLAVE_OK:
            LED_BUSLOCK_DEACTIVATE;
            // Fehlerzaehler bei jeder Aenderung hochzaehlen
            Gateway_MOST_Errorcounter++;
            MOST_State = MOST_SLAVE_UNLOCK_DETECTED;
            break;

         case MOST_MASTER_UNLOCK_DETECTED :
            if (Gateway_MOST[0].Format == MOSTFORMAT_MOST150_AUTO)
            {  // Timer 300 ms starten
               timSetMarker(TIMER_BYPASSCTL);
               MOST_State = MOST_MASTER_WAIT_RESTART;
            }
            break;

         case MOST_MASTER_WAIT_RESTART :
            if (timTimeout(TIMER_BYPASSCTL, 1000))
            {  // ICM schicken
               Error = inicNWStartup();
               printf("     inicNWStartup : %s\n", Error ? "failed" : "suceeded");
               MOST_State = MOST_MASTER_UNLOCK_DETECTED;
            }
            break;
      } // of switch MOST_State
   }
}
#endif


//****************************************************************************
//* Function     :	Debug_Info                                              *
//****************************************************************************
//* Description  :                                                           *
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

#if (CP_DEBUG == 1)
void Debug_Info (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   writeLong(resultPtr+4, NO_ERROR);

   switch  (commandPtr[2])
   {
  	   case 0x00 :
       	printf ("MOSTMSG-Init\n");
         writeLong(resultPtr+4, MOSTMSG_Init(commandPtr[3]));
			break;

      case 0x11:
         // Buffer schreiben
         printf ("MOST_Send\n");
         resultPtr[8] = MOST_Send (commandPtr[3], commandPtr+8, readLong(commandPtr+4));
         resultPtr[0] = 9;
         break;

      case 0x12:
         // Buffer schreiben
         printf ("MOST_0_Receive\n");
         MOST_0_Receive ();
         break;

      case 0x20:
         INIC_SPI_CS_ACTIVATE;
         Spi_RW_1Byte (commandPtr[3], commandPtr+8);
         INIC_SPI_CS_DEACTIVATE;
         resultPtr[0] = 9;
         break;

      case 0x21:
         while (1)
         {
            INIC_SPI_CS_ACTIVATE;
            INIC_SPI_CS_DEACTIVATE;
         }
         break;

      #if 0
      case 0x80:
         printf ("WR_SPIINT_ERROR %04X \n", WR_SPIINT_ERROR);
         printf ("RD_SPIINT_ERROR0 %04X \n", RD_SPIINT_ERROR0);
         printf ("RD_SPIINT_ERROR1 %04X \n", RD_SPIINT_ERROR1);
         printf ("RD_SPIINT_ERROR2 %04X \n", RD_SPIINT_ERROR2);
         printf ("WR_STATUS_ERROR %04X \n", WR_STATUS_ERROR);
         printf ("RD_STATUS_ERROR %04X \n", RD_STATUS_ERROR);
         break;
      #endif

   }
}

#endif // #if CP_DEBUG

#endif // #if CP_CMDDEV_MOST
