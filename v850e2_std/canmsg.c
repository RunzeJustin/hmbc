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

#include "tool.h"                 	// für hex2str usw.
#include "ints.h"						   // Interruptverwaltung
#include "timer.h"                  // für Wartezeit
#include <string.h>          		   // für memset
#include "canmsg.h"
#include "gateway.h"
#include "global.h"
#include "can.h"
#include "ser.h"

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

#if CP_CMDDEV_CAN		         // Hauptschalter für das ganze File ab hier

//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

// Fehlervariable nur für die Interruptroutine
UINT32  	CANMSG_INT_Error;

// Steuervariable für Interruptweiterleitung aus CAN
UINT8    CANMSG_INT_Enable = 0;

// Kanalnummer zum Parametersatz
UINT8    CAN_Selected_Channel = NONE;
UINT32   Gateway_CAN_CommIndex[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

// Zustandsvariable
UINT8 	CANMSG_RX_State = CANMSG_RX_DISABLED;
UINT8 	CANMSG_TX_State = CANMSG_TX_DISABLED;


//****************************************************************************
//* Function     :	CAN_Send                                                *
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

BOOL CAN_Send (UINT8 Channel, UINT8 *Buffer, UINT32 len)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

static UINT8    *pTX_Start,
                *pTX_Stop;
UINT32 	Error;

static BOOL     newBuffer = TRUE;
static UINT8    seq;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   Error = NO_ERROR;

   // Sicherheitshalber
   Channel = DEVICE_NR(Channel);

  	// Startwerte setzen
   if ((!newBuffer) && (seq != Buffer[2]))
      newBuffer = TRUE;

  	if (newBuffer)
  	{
      pTX_Start = Buffer;	         // Zeiger auf erstes Nutzbyte
      pTX_Stop  = Buffer + len; 	   // Zeiger auf erste freie Bufferadresse nach den Nutzdaten
      seq = Buffer[2];
   }

//*--------------------------------------------------------------------------*
//*	jetzt die Packet-Einzelteile verschicken  	                          *
//*--------------------------------------------------------------------------*

   do
	{
      // Sendebuffer  beladen und verschicken
 		Error |= Send_CAN_Data (pTX_Stop - pTX_Start , pTX_Start);

      // bei Fehler abbrechen
 		if (Error != NO_ERROR)
		{
         // Neustart der Schnittstelle nur bei internen Problemen
         //CANMSG_Init(Channel);

         // Fehler nicht hochmelden
         newBuffer = FALSE;
         return (FALSE);
      }
      pTX_Start += CANMSG_LAENGE;
   }
	while (pTX_Stop > pTX_Start);
   newBuffer = TRUE;

	return (TRUE);
} // end of function CAN_Send


//****************************************************************************
//* Function     :	CANMSG_MarkColdstart                                    *
//****************************************************************************
//* Description  :                                                           *
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

void CANMSG_MarkColdstart (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Kaltstart von CAN erfolgt bei naechster Anwahl
   CAN_Selected_Channel = NONE;
}


//****************************************************************************
//* Function     :	CANMSG_Init                                             *
//****************************************************************************
//* Description  :   initialisieren und Variable rücksetzen                  *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :  Error                                                    *
//*                                                                          *
//* Changed Var. :  CANMSG_RX_State                                          *
//*                 CANMSG_TX_State                                          *
//*						                                                        *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  (X) partly tested  ( ) fully tested   *
//****************************************************************************

// Einzelbits zum Verodern
#define INIT_RX_OBJECT        0x0001
#define INIT_TX_OBJECT        0x0002
#define INIT_TRANSCEIVER      0x0004
#define INIT_EXT_HW           0x0008

UINT32 CANMSG_Init (UINT8 Channel)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 	      Error,
               i,
               InitChecklist;

TCAN_Obj       NewCANObj;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Sicherheitshalber
   Channel = DEVICE_NR(Channel);

   // Init Checkliste  erstellen
   if (CAN_Selected_Channel == NONE)
     // Kaltstart nach Poweron
     InitChecklist = INIT_EXT_HW + INIT_TRANSCEIVER + INIT_RX_OBJECT + INIT_TX_OBJECT;
   else
   {
      // Sicherheitshalber
      CAN_Selected_Channel = DEVICE_NR(CAN_Selected_Channel);
      InitChecklist = 0x0000;

      if (Gateway_CAN[Channel].RXidentifier != Gateway_CAN[CAN_Selected_Channel].RXidentifier)
         InitChecklist |= INIT_RX_OBJECT;

      if (Gateway_CAN[Channel].TXidentifier != Gateway_CAN[CAN_Selected_Channel].TXidentifier)
         InitChecklist |= INIT_TX_OBJECT;

      if (Gateway_CAN[Channel].Baudrate != Gateway_CAN[CAN_Selected_Channel].Baudrate)
         InitChecklist |= INIT_TRANSCEIVER + INIT_RX_OBJECT + INIT_TX_OBJECT;

      if (Gateway_CAN[Channel].Transceiver != Gateway_CAN[CAN_Selected_Channel].Transceiver)
         InitChecklist = INIT_EXT_HW + INIT_TRANSCEIVER + INIT_RX_OBJECT + INIT_TX_OBJECT;;
   }

   // Kanalnummer jetzt fuers naechste Mal merken
   CAN_Selected_Channel = Channel;

//*--------------------------------------------------------------------------*

   // Zeit fuer eventuelles Busoff Recovery
   timWait (10);

   Error = CANMSG_NO_ERROR;

   // CAN Interrupt bzw. Schalter dafuer ausschalten, um sichere Initialisierung zu ermöglichen
   CANMSG_INT_Enable = 0;

	// Statemaschinen initialisieren
   CANMSG_RX_State = CANMSG_RX_DISABLED;
 	CANMSG_TX_State = CANMSG_TX_DISABLED;

//*--------------------------------------------------------------------------*
//* CAN-Hardware initialisieren                                              *
//*--------------------------------------------------------------------------*

   if (InitChecklist & INIT_EXT_HW)
   {
      // CAN-Transceiver einschalten ; Bausteinabhaengig
      CAN_TRANSCEIVER_ENABLE;

      // CAN-Transceiver sicherheitshalber aufwecken , sonst Verlust der ersten 3 Flanken an RX
      // Bausteinabhaengig
      CAN_TRANSCEIVER_WAKEUP;
   }

   if (InitChecklist & INIT_TRANSCEIVER)
   {
      #if CP_CAN3
      // Grundinitialisierung
      CAN3_Init(Gateway_CAN[Channel].Baudrate);
      #else
         #error Can init missing in canmsg.c
      #endif
   }

//*--------------------------------------------------------------------------*
//* RX-Object  setzen                                                        *
//*--------------------------------------------------------------------------*

   if (InitChecklist & INIT_RX_OBJECT)
   {
      // 8-bit Data Bytes vorbesetzen , technisch nicht noetig
      for(i=0; i < 8; i++)
         NewCANObj.ubData[i] = 0;

      NewCANObj.ubMsgObjNr = CAN_RX_OBJECT;
      // 8-bit Message Configuration Register 8 Databytes , Standard Identifier, Receive
      NewCANObj.ubMsgCfg = 0x80;
      // standard (11-bit)/extended (29-bit) identifier
         NewCANObj.ulArbitr = Gateway_CAN[Channel].RXidentifier;
      // alle Bits sind signifikant
      NewCANObj.ulMaskr =  0x000007FF;

      // RX Message Objekt konfigurieren
      CAN_bRequestMsgObj(CAN_RX_OBJECT);
      CAN_vConfigMsgObj(CAN_RX_OBJECT, &NewCANObj);
   }

//*--------------------------------------------------------------------------*
//* TX-Object  setzen                                                        *
//*--------------------------------------------------------------------------*

   if (InitChecklist & INIT_TX_OBJECT)
   {
      NewCANObj.ubMsgObjNr = CAN_TX_OBJECT;
      // 8-bit Message Configuration Register 8 Databytes , Standard Identifier, Transmit
      NewCANObj.ubMsgCfg = 0x88;
      // standard (11-bit)/extended (29-bit) identifier
         NewCANObj.ulArbitr = Gateway_CAN[Channel].TXidentifier;
      // restliche Einstellungen wie beim RX-Object

      // TX Message Objekt konfigurieren
      CAN_bRequestMsgObj(CAN_TX_OBJECT);
      CAN_vConfigMsgObj(CAN_TX_OBJECT, &NewCANObj);
   }

//*--------------------------------------------------------------------------*
//* Initialisierung vervollständigen                                         *
//*--------------------------------------------------------------------------*

   // Fehlerspeicher für Interruptroutine rücksetzen
	CANMSG_INT_Error = CANMSG_NO_ERROR;

   // CAN Interrupt bzw. Schalter jetzt einschalten
   CANMSG_INT_Enable = 1;

   // System ist ab jetzt  initialisiert
   CANMSG_RX_State = CANMSG_RX_ENABLED;
 	CANMSG_TX_State = CANMSG_TX_ENABLED;

 	// beim Comm-Modul anmelden
 	Gateway_CAN_CommIndex[Channel] = msgRegister(CAN+Channel, "CAN", Channel, 0, NULL, CAN_Send, NULL);

   return (Error);
} // end of function CANMSG_Init


#endif // #if CP_CMDDEV_CAN

