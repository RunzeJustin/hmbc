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
#include "global.h"                 // globale Datenstrukturen
#include "ints.h"                 	// für die Interruptverwaltung
#include "can.h"                  	// CAN-Defines
#include "tool.h"
#include "timer.h"
#include <string.h>                 // für memset
#include "gateway.h"
#include "hw_init.h"

#if CP_CMDDEV_CAN
   #include "canmsg.h"              // CAN-Modul
#endif


#if CP_CAN3


//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

BOOL           CAN3_Init         (UINT32 Baudrate);

// Low-Level
static   void  CAN_vGetMsgObj		(UINT8 ObjNr, TCAN_Obj *pstObj);
void           CAN_vConfigMsgObj	(UINT8 ObjNr, TCAN_Obj *pstObj);
BOOL           CAN_bRequestMsgObj(UINT8 ObjNr);
void           Get_CANMSG        (void);

static   BOOL  CAN_vTransmit		(UINT8 ObjNr);
static   void  CAN_vLoadData		(UINT8 ObjNr, UINT8 *pubBuffer);
static   BOOL 	CAN_bDelMsgObj		(UINT8 ObjNr);
static   BOOL 	CAN_bValMsgObj		(UINT8 ObjNr);


//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

TCAN_Obj    ReceivedCANObj[CAN_MAX_RECEIVE_MSG];
INT8        CAN_ReceiveMsgCount;
UINT8       CAN_Error;

UINT32      INTFCNWUP_Counter,
            INTFCN3TRX_Counter,
            INTFCN3REC_Counter,
            INTFCN3ERR_Counter;

//****************************************************************************
//* Function     : CAN3_Init                                                 *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function initializes the CAN component. It effects all       *
//*        necessary configurations of the SFR, depending on the selected    *
//*        operating mode. The configuration determines whether the CAN      *
//*        interrupts are to be released, and the priority of the            *
//*        released interrupt.                                               *
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

BOOL CAN3_Init(UINT32 Baudrate)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    i,j;
UINT32   P_Clk;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   INTFCNWUP_Counter = 0;
   INTFCN3TRX_Counter = 0;
   INTFCN3REC_Counter = 0;
   INTFCN3ERR_Counter = 0;

   // CAN-Interrupts ausschalten waehrend der Initialisierung
   ICFCNWUP  |= INT_DISABLE;
   ICFCN3ERR |= INT_DISABLE;
   ICFCN3REC |= INT_DISABLE;
   ICFCN3TRX |= INT_DISABLE;

#if (CPU_TYPE == CPU_NEC_V850E2_DF3558)
   #if (CP_CAN3 == 1)
   // setup P41 and P42 as CAN3 function pins
   PFC4  |= 0x0006;
   PFCE4 |= 0x0006;
   SETPORT_OUT(4,1);
   SETPORT_IN(4,2);
   SETPORT_ALT(4,1);
   SETPORT_ALT(4,2);
   #else
      #error CAN3 portconfig unknown
   #endif
#else
   #error cputype unknown
#endif

   // eingestellten Clock erfragen
   P_Clk = GetDomainClock (CLK_DOMAIN_115);

   if ((P_Clk != 40000000) && (P_Clk != 24000000))
      printf ("   CAN3-Error : Clock of Domain 115 not 24 Mhz or 40 Mhz\n");

   // Softreset CAN
   FCN3GMCLCTL |= FCNnGMCLSESR;

   // TimeOut-Timer  aufziehen
   timSetMarker(TIMER_WAIT_SIGNAL);
   //  maximal 100 ms auf Ausfuehrung warten, RXD muss dazu 1 sein !!
   do
   {
      // Fehlerabbruch
      if (timTimeout(TIMER_WAIT_SIGNAL, 100))
      {
         printf ("   CAN3-Error : softreset timeout\n");
         return (FALSE);
      }
   }
   while ((FCN3GMCLCTL & FCNnGMCLSORF) != 0);

   // Error in message ram ?
   if ((FCN3GMCLCTL & FCNnGMCLSORF) != 0)
   {
      printf ("   CAN3-Error : messagebuffer ram error\n");
      return (FALSE);
   }


   // Messagezähler und Fehler zurücksetzen
   CAN_ReceiveMsgCount = 0;
   CAN_Error = 0;

//*--------------------------------------------------------------------------*
//* Initialize Bit-Timing                                                    *
//* refer to datasheet                                                       *
//* Set Bittiming @40Mhz/24Mhz sampling point 75%                            *
//*--------------------------------------------------------------------------*

   // Vorteiler Clock setzen  fcanmod = fcan/1 = 24/40 Mhz
   FCN3GMCSPRE = 0x00;

   // Enable CAN
   FCN3GMCLCTL |= BIT_SET(FCNnGMCLPWOM);

   // Set CAN  to initialization mode
   FCN3CMCLCTL = BIT_CLEAR(FCNnCMCLCLOP0) | BIT_CLEAR(FCNnCMCLCLOP1) | BIT_CLEAR(FCNnCMCLCLOP2);


   if (P_Clk == 40000000)
   {
      switch (Baudrate)
      {
         case 83300 :
         case 83333 :   // Prescaler  ftq = fcanmod / 30
                        FCN3CMBRPRS = 29;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 100000 :  // Prescaler  ftq = fcanmod / 25
                        FCN3CMBRPRS = 24;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 125000 :  // Prescaler  ftq = fcanmod / 20
                        FCN3CMBRPRS = 19;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 250000 :  // Prescaler  ftq = fcanmod / 10
                        FCN3CMBRPRS = 9;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 500000 :  // Prescaler  ftq = fcanmod / 5
                        FCN3CMBRPRS = 4;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 1000000 : // Prescaler  ftq = fcanmod / 5
                        FCN3CMBRPRS = 4;
                        // SJW =1; TSEG2 = 2, TSEG1 = 5; TQ = 8
                        FCN3CMBTCTL = 0x0104;
                        break;

         default:       return(FALSE);
      }
   }

//*--------------------------------------------------------------------------*

   if (P_Clk == 24000000)
   {
      switch (Baudrate)
      {
         case 83300 :
         case 83333 :   // Prescaler  ftq = fcanmod / 18
                        FCN3CMBRPRS = 17;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 100000 :  // Prescaler  ftq = fcanmod / 15
                        FCN3CMBRPRS = 14;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 125000 :  // Prescaler  ftq = fcanmod / 12
                        FCN3CMBRPRS = 11;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 250000 :  // Prescaler  ftq = fcanmod / 6
                        FCN3CMBRPRS = 5;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 500000 :  // Prescaler  ftq = fcanmod / 3
                        FCN3CMBRPRS = 2;
                        // SJW =1; TSEG2 = 4, TSEG1 = 11; TQ = 16
                        FCN3CMBTCTL = 0x030A;
                        break;

         case 1000000 : // Prescaler  ftq = fcanmod / 3
                        FCN3CMBRPRS = 2;
                        // SJW =1; TSEG2 = 2, TSEG1 = 5; TQ = 8
                        FCN3CMBTCTL = 0x0104;
                        break;

         default:       return(FALSE);
      }
   }

//*--------------------------------------------------------------------------*
//* Initialize all 8 global Masks                                            *
//*--------------------------------------------------------------------------*

   // alle Bits in der Maske signifikant
   FCN3CMMKCTL01W = 0x00000000;
   FCN3CMMKCTL03W = 0x00000000;
   FCN3CMMKCTL05W = 0x00000000;
   FCN3CMMKCTL07W = 0x00000000;
   FCN3CMMKCTL09W = 0x00000000;
   FCN3CMMKCTL11W = 0x00000000;
   FCN3CMMKCTL13W = 0x00000000;
   FCN3CMMKCTL15W = 0x00000000;

//*--------------------------------------------------------------------------*
//* Initialize messagebuffer (only minimum)                                  *
//* refer to datasheet 19.8.2                                                *
//*--------------------------------------------------------------------------*

   // all object are invalid and have to be configured first
   for (i=0; i <= CAN_MAX_OBJ; i++)
   {
      CAN3_B_OBJ[i].FCNnMmCTL = BIT_CLEAR(FCNnMmCLNH) | BIT_CLEAR(FCNnMmCLMW) | BIT_CLEAR(FCNnMmCLRY) |
                                BIT_CLEAR(FCNnMmCLTR) | BIT_CLEAR(FCNnMmCLDN) | BIT_CLEAR(FCNnMmCLIE);

      // Message type
      CAN3_A_OBJ[i].FCNnMmSTRB = 0x00;

      // Identifier
      CAN3_B_OBJ[i].FCNnMmMID0H = 0;
      CAN3_B_OBJ[i].FCNnMmMID1H = 0;

      // Data length
      CAN3_A_OBJ[i].FCNnMmDTLGB = 0;

      // Daten zu Debugzwecken zuruecksetzen
      for (j=0;j<8 ;j++)
      {
         CAN3_A_OBJ[i].FCNnMmDATx[j].B = 0;
      }
   }

//*--------------------------------------------------------------------------*
//* Initialize interrupts                                                    *
//* refer to datasheet 19.??                                                 *
//*--------------------------------------------------------------------------*

   // Clear Int Statusregister
   FCN3CMISCTL = BIT_CLEAR(FCNnCMISITSF0) | BIT_CLEAR(FCNnCMISITSF1) | BIT_CLEAR(FCNnCMISITSF2) | BIT_CLEAR(FCNnCMISITSF3) |
                 BIT_CLEAR(FCNnCMISITSF4) | BIT_CLEAR(FCNnCMISITSF5) | BIT_CLEAR(FCNnCMISITSF6);

   //  enable CAN interrupts
   //  set interrupt priority level in file ints.h
   ICFCNWUP  = INIT_FCNWUP;
   ICFCN3ERR = INIT_FCN3ERR;
   ICFCN3REC = INIT_FCN3REC;
   ICFCN3TRX = INIT_FCN3TRX;

   // Receive interrupt and CAN error status interrupt on
   FCN3CMIECTL = BIT_SET(FCNnCMISITSF1) | BIT_SET(FCNnCMISITSF2);

//*--------------------------------------------------------------------------*
//* Switch on CAN                                                            *
//*--------------------------------------------------------------------------*

   // clear valid bit and errorcounter
   FCN3CMCLCTL = FCNnCMCLSERC | BIT_CLEAR(FCNnCMCLCLVL);

   // no powersave mode, normal operation
   FCN3CMCLCTL = BIT_CLEAR(FCNnCMCLCLAL) | BIT_CLEAR(FCNnCMSECLPS0) | BIT_CLEAR(FCNnCMSECLPS1) |
                 BIT_SET(FCNnCMCLCLOP0)  | BIT_CLEAR(FCNnCMCLCLOP1) | BIT_CLEAR(FCNnCMCLCLOP2);

   // Fehlerregister loeschen
   FCN3CMLCSTR = 0x00;

   return(TRUE);
} // end of function


//*--------------------------------------------------------------------------*
//* Interrupt Funktionsrumpf; unbenutzt                                      *
//*--------------------------------------------------------------------------*

#pragma ghs interrupt
void INTFCNWUP (void)
{
   INTFCNWUP_Counter++;
}

//*--------------------------------------------------------------------------*
//* Interrupt Funktionsrumpf; unbenutzt                                      *
//*--------------------------------------------------------------------------*

#pragma ghs interrupt
void INTFCN3TRX (void)
{
   INTFCN3TRX_Counter++;
}

//****************************************************************************
//* Function     : INTFCN3REC                                                *
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

#pragma ghs interrupt
void INTFCN3REC (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

  UINT8  Objectnummer;
  UINT16 FCN3CMRGRX_Wert;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   INTFCN3REC_Counter++;

   if (!CAN3_TRANSCEIVER_ACTIVE)
   {  // CAN Transceiver schlaeft noch
      CAN_Error |= 0x80;
   }

   // Clear Statusregister
   FCN3CMISCTL = BIT_CLEAR(FCNnCMISITSF1);

   // Receive History List Register genau einmal auslesen
   // Zeiger wechselt Inhalt bei jedem Auslesen und ist bei FCNnCMRGSSPM gesetzt ungueltig
   while (!((FCN3CMRGRX_Wert = FCN3CMRGRX) & FCNnCMRGSSPM))
   {
      // History list overflow bit loeschen wenn gesetzt
      if (FCN3CMRGRX_Wert & FCNnCMRGRVFF)
         FCN3CMRGRX = BIT_CLEAR(FCNnCMRGCLRV);

      // Nummer des CAN-objects von 0-CAN_MAX_OBJ;
      Objectnummer = FCN3CMRGRX_Wert >> 8;

      //sicherheitshalber noch testen, abweisen falls doch eine ungueltige Nummer durchschluepft
      if (Objectnummer <= CAN_MAX_OBJ)
      {
         #if CP_CMDDEV_CAN
         // Test ob Gatewayobjekt
         if (Objectnummer == CAN_RX_OBJECT)
         {
            // Bios MSG Receive
            CAN3_B_OBJ[CAN_RX_OBJECT].FCNnMmCTL = BIT_CLEAR(FCNnMmCLDN);
            Get_CANMSG();
         }
         else
         #endif

         {  // Messagebuffer gegen Überlauf sichern
            TEST_MSGCOUNTER;

            // Message abholen
            do
            {
               CAN3_B_OBJ[Objectnummer].FCNnMmCTL = BIT_CLEAR(FCNnMmCLDN);
               CAN_vGetMsgObj(Objectnummer, &ReceivedCANObj[CAN_ReceiveMsgCount]);
            }
            // nachsehen, ob die Message waehrenddessen nochmal aktualisiert wurde
            while (CAN3_B_OBJ[Objectnummer].FCNnMmCTL & (FCNnMmMUCF | FCNnMmDTNF));

            // MessageZähler hochzählen
            INC_MSGCOUNTER;
         } // of else
      } // of if
   } // of while
}  // end of ISR


//****************************************************************************
//* Function     : INTC0ERR                                                  *
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

#pragma ghs interrupt
void INTFCN3ERR (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   INTFCN3ERR_Counter++;

   // Clear Statusregister CAN error and protocol Error
   FCN3CMISCTL = BIT_CLEAR(FCNnCMISITSF2) | BIT_CLEAR(FCNnCMISITSF3);

   if (!CAN3_TRANSCEIVER_ACTIVE)
   {  // CAN Transceiver schlaeft noch
      CAN_Error |= 0x80;
      return;
   }

   switch (FCN3CMLCSTR)  // (Last Error Code)
   {
      case 1:  // Stuff Error
               // More than 5 equal bits in a sequence have occurred
               // in a part of a received message where this is not
               // allowed.
               CAN_Error |= 0x01;
               break;

      case 2:  // Form Error
               // A fixed format part of a received frame has the
               // wrong format.
               CAN_Error |= 0x02;
               break;

      case 3:  // Ack Error
               // The message this CAN controller transmitted was
               // not acknowledged by another node.
               CAN_Error |= 0x04;
               break;

      case 4:  // Bit1 Error
               // During the transmission of a message (with the
               // exeption of the arbitration field), the device
               // wanted to send a recessive level ("1"), but the
               // monitored bus value was dominant.
               CAN_Error |= 0x08;
               break;

      case 5:  // Bit0 Error
               // During the transmission of a message (or acknow-
               // ledge bit, active error flag, or overload flag),
               // the device wanted to send a dominant level ("0"),
               // but the monitored bus value was recessive. During
               // busoff recovery this staus is set each time a
               // sequence of 11 recessive bits has been monitored.
               // This enables the CPU to monitor the proceeding of
               // the busoff recovery sequence (indicating the bus
               // is not stuck at dominant or continously disturbed).

               CAN_Error |= 0x10;
               break;

      case 6:  // CRC Error
               // The CRC check sum was incorrect in the message
               // received.
               CAN_Error |= 0x40;
               break;

      default:
               break;

   } // of switch (FCN3CMLCSTR)

   // Fehler loeschen
   FCN3CMLCSTR = 0;

   // alle anderen moeglichen Interruptereignisse auch abpruefen
   if (FCN3CMINSTR != 0)  // if BOFF u.s.w.
   {
      // Indicates when the CAN controller is in busoff state.
      if (FCN3CMINSTR & FCNnCMINBOFF)
      {
         // Set CAN  to initialization mode
         FCN3CMCLCTL = BIT_CLEAR(FCNnCMCLCLOP0) | BIT_CLEAR(FCNnCMCLCLOP1) | BIT_CLEAR(FCNnCMCLCLOP2);
         i = 0;
         //  maximal 20 ms auf Ausfuehrung warten, RXD muss dazu 1 sein !!
         do
         {
            DelayLoop (us(100));
            // Fehlerabbruch
            if (i++ > 200)
               break;
         }
         while ((FCN3CMCLCTL & (FCNnCMCLCLOP0 | FCNnCMCLCLOP1 | FCNnCMCLCLOP2)) != 0);

         // set to normal operation; forced recovery after busoff
         FCN3CMCLCTL = FCNnCMCLSERC | BIT_SET(FCNnCMCLCLOP0) | BIT_CLEAR(FCNnCMCLCLOP1) | BIT_CLEAR(FCNnCMCLCLOP2);
      }
   }
}  // end ISR


//****************************************************************************
//* Function     :	Read_CAN_Data                                           *
//****************************************************************************
//* Description  :   CAN-Datensatz 1 - 8 auslesen                            *
//*						nur soviele Bytes lesen, wie in DLC angegeben           *
//*                  und maximal 8                                           *
//* Parameter    :   Zeiger auf  Buffer , Zeiger auf Laenge                  *
//*                                                                          *
//* Returnvalue  :   Error                                                   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if CP_CMDDEV_CAN
UINT32  Read_CAN_Data ( UINT32 *Laenge, UINT8 *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32   i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // als erstes Längenangabe lesen
   *Laenge = CAN3_A_OBJ[CAN_RX_OBJECT].FCNnMmDTLGB;

   for (i=0; i < *Laenge; i++)
   {  // Daten auslesen
      *Buffer++ = CAN3_A_OBJ[CAN_RX_OBJECT].FCNnMmDATx[i].B;
   }

   // Ruecksprung mit OK - Quittung
 	return(CANMSG_NO_ERROR);
} // end of function Read_CAN_Data
#endif


//****************************************************************************
//* Function     :   Get_CANMSG                                              *
//****************************************************************************
//* Description  :   Das RX-Objekt der BIOS-Fernsteuerung ohne weitere       *                                                          *
//*                  Kontrolle zeitoptimiert einlesen                        *
//*                  Der Identifier wird als richtig  vorausgesetzt          *
//*                  Der Laengencode muss 8 sein                             *
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

#if CP_CMDDEV_CAN
void Get_CANMSG(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 RX_Char[CANMSG_LAENGE];
UINT32 RX_Anzahl;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (CAN_Error)
   {
      // CAN-Fehler zurücksetzen
      CAN_Error = 0;
      CANMSG_INT_Error |= CANMSG_RX_CAN_ERROR;
   }

   // nachsehen, ob die Message waehrenddessen nochmal ueberschrieben wurde
   if (CAN3_B_OBJ[CAN_RX_OBJECT].FCNnMmCTL & FCNnMmMOWF)
   {  // Indicates that the CAN controller has stored a new
      // message into this object
      // ie. the previously stored message is lost.
      CAN3_B_OBJ[CAN_RX_OBJECT].FCNnMmCTL = BIT_CLEAR(FCNnMmCLMW);
      // Fehler merken
      CANMSG_INT_Error |= CANMSG_RX_PACKET_FORMAT_ERROR;
   }
   else
   {  // Message da, Auswertung, wenn erlaubt
      if (CANMSG_INT_Enable)
      {
         // Message und Gesamtlänge  abholen und in RX-Buffer eintragen
         CANMSG_INT_Error |= Read_CAN_Data (&RX_Anzahl, RX_Char);

         // Comm modul bedienen
         msgReceive (Gateway_CAN_CommIndex[CAN_Selected_Channel], RX_Char, RX_Anzahl);
      }
   }
} // end of function
#endif


//****************************************************************************
//* Function     :	Send_CAN_Data                                           *
//****************************************************************************
//* Description  :   CAN-Datensatz 1 - 8 schreiben                           *
//*						nur soviele Bytes schreiben, wie in Laenge angegeben    *
//*                  und maximal 8                                           *
//*                  Nachricht ueber CAN verschicken                         *
//*                  Das CAN-Object mus korrekt gesetzt sein                 *
//*                                                                          *
//* Parameter    :  Zeiger auf Buffer, Laenge                                *
//*                                                                          *
//* Returnvalue  :  Error                                                    *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                   													  *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

#if CP_CMDDEV_CAN
UINT32  Send_CAN_Data ( UINT32 Laenge, UINT8 *Buffer)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT32 	i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Begrenzung
   if (Laenge > CANMSG_LAENGE)
      Laenge = CANMSG_LAENGE;

   if (CAN3_B_OBJ[CAN_TX_OBJECT].FCNnMmCTL & FCNnMmTRQF)
      return (CANMSG_TX_TIMEOUT_ERROR);

   // Zugriffsberechtigung auf das Object holen
   while (CAN3_B_OBJ[CAN_TX_OBJECT].FCNnMmCTL & FCNnMmRDYF)
   {
     // Clear Readybit
     CAN3_B_OBJ[CAN_TX_OBJECT].FCNnMmCTL = BIT_CLEAR(FCNnMmCLRY);
   }

   // als erstes Laenge schreiben  (DLC)
   CAN3_A_OBJ[CAN_TX_OBJECT].FCNnMmDTLGB = (UINT8)(Laenge & 0x000000FF);

   for (i=0; i < Laenge; i++)
   {  // neue Daten schreiben, aber hoechstens 8 Bytes
      CAN3_A_OBJ[CAN_TX_OBJECT].FCNnMmDATx[i].B = *Buffer++;
   }

   // Message wieder freigeben
   CAN3_B_OBJ[CAN_TX_OBJECT].FCNnMmCTL = BIT_SET(FCNnMmCLRY);

   // Objekt jetzt senden
   CAN3_B_OBJ[CAN_TX_OBJECT].FCNnMmCTL = BIT_SET(FCNnMmCLTR);

   // eventuelle CAN-Fehler aus der Vorgeschichte hier beruecksichtigen
   if (CAN_Error)
   {
      // CAN-Fehler zurücksetzen
      CAN_Error = 0;
      return (CANMSG_TX_CAN_ERROR);
   }
   else
   {
      // Ruecksprung mit OK - Quittung
      return (CANMSG_NO_ERROR);
   }
} // end of function Send_CAN_Data
#endif


//****************************************************************************
//* Function     : CAN_vGetMsgObj                                            *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function fills the forwarded SW message object with the      *
//*        content of the chosen HW message object.                          *
//*                                                                          *
//*        The structure of the SW message object is defined in the          *
//*        header file CAN.H (see TCAN_Obj).                                 *
//*                                                                          *
//*                                                                          *
//* Parameter    : Number of the message object to be read                   *
//*                Pointer on a message object to be filled by this function.*
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void CAN_vGetMsgObj (UINT8 ObjNr, TCAN_Obj *pstObj)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Dlc,
         Dir,
         Xtd,
         i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   pstObj->ubMsgObjNr = ObjNr;

   // Bitfelder einsammeln
   // Laenge ?
   Dlc =  CAN3_A_OBJ[ObjNr].FCNnMmDTLGB;

   // TX oder RX?
   if ((CAN3_A_OBJ[ObjNr].FCNnMmSTRB & (FCNnMmSSMT0 | FCNnMmSSMT1 | FCNnMmSSMT2 | FCNnMmSSMT3)) == 0)
      Dir = 1;
   else
      Dir = 0;

   // extended Identifier ?
   Xtd = (CAN3_B_OBJ[ObjNr].FCNnMmMID1H & FCNnMmSSIE) >> 15;

   // und nach C161-Core CAN Definition wieder zusammenbauen und abspeichern
   pstObj->ubMsgCfg = (Dlc << 4) | (Dir << 3) | (Xtd << 2);

//*--------------------------------------------------------------------------*

   // Identifier einlesen
   if (Xtd)
   {  // extended identifier
      pstObj->ulArbitr = (((UINT32)CAN3_B_OBJ[ObjNr].FCNnMmMID1H & 0x1FFF) << 16) + CAN3_B_OBJ[ObjNr].FCNnMmMID0H;
   }
   else
   {  // standard identifier
      pstObj->ulArbitr = ((UINT32)CAN3_B_OBJ[ObjNr].FCNnMmMID1H & 0x1FFF) >> 2;
   }

   // Maske unbenutzt
   pstObj->ulMaskr = 0x00000000;

//*--------------------------------------------------------------------------*

   // immer alle Datenbytes unabhängig von DLC abholen
   for(i = 0; i <8; i++)
   {
      pstObj->ubData[i] = CAN3_A_OBJ[ObjNr].FCNnMmDATx[i].B;
   }

} // end of function


//****************************************************************************
//* Function     : CAN_vTransmit                                             *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function triggers the CAN controller to send the             *
//*        selected message.                                                 *
//*        If the selected message object is a TRANSMIT OBJECT then          *
//*        this function triggers the sending of a data frame.               *
//*        If however the selected message object is a RECEIVE OBJECT        *
//*        this function triggers the sending of a remote frame.             *
//*                                                                          *
//* Parameter    : Number of the message object to be sent (1-14)            *
//*                                                                          *
//* Returnvalue  : FALSE, wenn Sende-Timeout abgelaufen                      *
//*                TRUE, wenn gesendet                                       *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

BOOL CAN_vTransmit(UINT8 ObjNr)
{
   // warten, bis letzter Sendevorgang erfolgreich und Object frei
   timSetMarker(TIMER_TX_GLOBAL);
   // Statusregister pollen maximal bis zum Timeout 0.1 s
   do
   {  // auf "Message Transmitted" warten
    	if (timTimeout(TIMER_TX_GLOBAL, 100))
         return (FALSE);
   }
   while (CAN3_B_OBJ[ObjNr].FCNnMmCTL & FCNnMmTRQF);

   // senden
   CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_SET(FCNnMmCLTR);

   return(TRUE);
} // end of function


//****************************************************************************
//* Function     : CAN_vConfigMsgObj                                         *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function sets up the message objects. This includes          *
//*        the 8 data bytes, the identifier (11- or 29-bit), the data        *
//*        number (0-7 bytes) and the XTD-bit.                               *
//*        The message is not sent; for this the function                    *
//*        CAN_vTransmit must be called.                                     *
//*                                                                          *
//*        The structure of the SW message object is defined in the          *
//*        header file CAN.H (see TCAN_Obj).                                 *
//*                                                                          *
//* Parameter    : Number of the message object to be configured (1-15)      *
//*                Pointer on a message object                               *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void CAN_vConfigMsgObj(UINT8 ObjNr, TCAN_Obj *pstObj)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8    Dlc,
         Dir,
         Xtd,
         i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // DLC , Dir und XTD werden neu gesetzt werden, aus der C161-Darstellung extrahieren
   Dlc = (pstObj->ubMsgCfg & 0xF0) >> 4;
   Dir = (pstObj->ubMsgCfg & 0x08) >> 3;
   Xtd = (pstObj->ubMsgCfg & 0x04) >> 2;

//*--------------------------------------------------------------------------*

   // Message Configuration Register setzen
   if (Dir)
      // Transmit
      CAN3_A_OBJ[ObjNr].FCNnMmSTRB = FCNnMmSSAM;
   else
      // Receive ohne Maske
      CAN3_A_OBJ[ObjNr].FCNnMmSTRB = FCNnMmSSAM | FCNnMmSSMT0;

//*--------------------------------------------------------------------------*

   // Identifier setzen
   if (Xtd)
   {  // extended identifier, Arbitrierung und XTD neu eintragen
      CAN3_B_OBJ[ObjNr].FCNnMmMID1H = ((UINT16)Xtd<<15) | (UINT16)((pstObj->ulArbitr >> 16) & 0x1FFF);
      CAN3_B_OBJ[ObjNr].FCNnMmMID0H = (UINT16)(pstObj->ulArbitr & 0x0000FFFF);
   }
   else
   {
      // standard identifier; Arbitrierung und XTD neu eintragen
      CAN3_B_OBJ[ObjNr].FCNnMmMID1H = (UINT16)((pstObj->ulArbitr << 2) & 0x1FFF);
      CAN3_B_OBJ[ObjNr].FCNnMmMID0H = 0x0000;
   }

//*--------------------------------------------------------------------------*

   // DLC schreiben
   CAN3_A_OBJ[ObjNr].FCNnMmDTLGB = Dlc;

//*--------------------------------------------------------------------------*

   if (Dir)   // if transmit direction
   {  // alle Datenbytes unabhängig von DLC eintragen
      for(i = 0; i < 8; i++)
      {
         CAN3_A_OBJ[ObjNr].FCNnMmDATx[i].B = pstObj->ubData[i];
      }
      // TX ohne Interrupt
      CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_CLEAR(FCNnMmCLIE) | BIT_CLEAR(FCNnMmCLMW) |
                                    BIT_CLEAR(FCNnMmCLDN) | BIT_CLEAR(FCNnMmCLTR);
   }
   else
      // RX mit Interrupt
      CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_SET(FCNnMmCLIE)   | BIT_CLEAR(FCNnMmCLMW) |
                                    BIT_CLEAR(FCNnMmCLDN) | BIT_CLEAR(FCNnMmCLTR);

//*--------------------------------------------------------------------------*

   // Message wieder freigeben
   CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_SET(FCNnMmCLRY);

} // end of function


//****************************************************************************
//* Function     : CAN_vLoadData                                             *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        If a hardware TRANSMIT OBJECT has to be loaded with data          *
//*        but not with a new identifier, this function may be used          *
//*        instead of the function CAN_vConfigMsgObj.                        *
//*        The message object should be accessed by calling the function     *
//*        CAN_bRequestMsgObj before calling this function. This             *
//*        prevents the CAN controller from working with invalid data.       *
//*                                                                          *
//* Parameter    : Number of the message object to be configured (1-15)      *
//*                Pointer on a data buffer                                  *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void CAN_vLoadData(UINT8 ObjNr, UINT8 *pubData)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 i;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // alle Datenbytes unabhängig von DLC eintragen
   for(i = 0; i < 8; i++)
   {
      CAN3_A_OBJ[ObjNr].FCNnMmDATx[i].B = *(pubData++);
   }

   // Message wieder freigeben
   CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_SET(FCNnMmCLRY);

} // end of function


//****************************************************************************
//* Function     : CAN_bRequestMsgObj                                        *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        If a TRANSMIT OBJECT is to be reconfigured it must first be       *
//*        accessed. The access to the transmit object is exclusive.         *
//*        This function checks whether the choosen message object is        *
//*        still executing a transmit request, or if the object can          *
//*        be accessed exclusively.                                          *
//*        After the message object is reserved, it can be reconfigured      *
//*        by using the function CAN_vConfigMsgObj or CAN_vLoadData.         *
//*        Both functions enable access to the object for the CAN            *
//*        controller.                                                       *
//*        By calling the function CAN_vTransmit transfering of data         *
//*        is started.                                                       *
//*                                                                          *
//* Parameter    : Number of the message object                              *
//*                                                                          *
//* Returnvalue  : 0 message object is busy (a transfer is actice), else 1   *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

BOOL CAN_bRequestMsgObj(UINT8 ObjNr)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Zugriffsberechtigung holen
   while (CAN3_B_OBJ[ObjNr].FCNnMmCTL & FCNnMmRDYF)
   {
     // Clear Readybit
     CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_CLEAR(FCNnMmCLRY);
   }
   return(1);

} // end of function


//****************************************************************************
//* Function     : CAN_bDelMsgObj                                            *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function marks the selected message object as not valid.     *
//*        This means that this object cannot be sent or received.           *
//*                                                                          *
//* Parameter    : Number of the message object                              *
//*                                                                          *
//* Returnvalue  : 1 the message object was deleted, else 0                  *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

BOOL CAN_bDelMsgObj(UINT8 ObjNr)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   CAN3_B_OBJ[ObjNr].FCNnMmCTL = BIT_CLEAR(FCNnMmCLRY) | BIT_CLEAR(FCNnMmCLTR) |
                                 BIT_CLEAR(FCNnMmCLDN) | BIT_CLEAR(FCNnMmCLIE);
   CAN3_A_OBJ[ObjNr].FCNnMmSTRB &= ~FCNnMmSSAM;

   return(1);

} // end of function


//****************************************************************************
//* Function     : CAN_bValMsgObj                                            *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function test the selected message object on beeing valid.   *
//*                                                                          *
//* Parameter    : Number of the message object                              *
//*                                                                          *
//* Returnvalue  : 1 the message object was valid, else 0                    *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

BOOL CAN_bValMsgObj (UINT8 ObjNr)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if ((CAN3_A_OBJ[ObjNr].FCNnMmSTRB & FCNnMmSSAM) == FCNnMmSSAM)
      return(1);
   else
      return(0);

} // end of function


//****************************************************************************
//* Function     : CAN_WR                                                    *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*        This function configures CAN-Objekts, loads new data to existing  *
//*        CAN-Objects, and sends a CAN-Object.                              *
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

void CAN_WR(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
   UINT8 ObjNr;                // CAN-Objekt welches geschrieben werden soll;
   UINT8 i;                    // Laufvariable
   UINT32 Error;
   TCAN_Obj      NewCANObj;
   UINT8 anz = commandPtr[0];
   UINT8 cmd = commandPtr[3];

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

#if CP_CAN1
   // Unterverteiler auf CAN1-Routinen
   if (cmd & 0x20)
   {  CAN1_WR ();
      return;
   }
#endif

   Error = NO_ERROR;

   // check for correct command
   if (((cmd == 0x30) && (anz ==  5)) ||
       ((cmd == 0x31) && (anz == 13)) ||
       ((cmd == 0x32) && (anz == 22)) ||
       ((cmd == 0x33) && (anz ==  5)) ||
       ((cmd == 0x3E) && (anz  >  3)) ||   // neu: meldet Schnittstelle dem MBC-Handler an
       ((cmd == 0x3F) && (anz ==  8))
       )   // send data mit warten auf Antwort

   {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }

   if (cmd == 0x3F)
   {
      #if CP_CMDDEV_CAN
      // vormerken fuer Gatewaybetrieb
      CANMSG_MarkColdstart ();
      #endif
      // Neu initialisieren
      if (!CAN3_Init(readLong(commandPtr+4)))
      {  // Baudrate war ungueltig
         Error = ERROR_CAN | ERROR_CAN_BAUDRATE_ERROR;
      }
   }
   else
   {
      ObjNr = commandPtr[4];
      //                    neue KWP2000 Kommandos
      if (ObjNr <= CAN_MAX_OBJ)
      {
         switch (cmd)         {
            case 0x30:
               if (CAN_bValMsgObj(ObjNr))
               {  // CAN-Objekt senden
                  if (!CAN_vTransmit(ObjNr))
                     Error = ERROR_CAN | ERROR_CAN_HW;
               }
               else
                  // CAN-Objekt war ungueltig
                  Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
               break;

            case 0x31:
               if (CAN_bValMsgObj(ObjNr))
               {  CAN_bRequestMsgObj(ObjNr);
                  // neue Daten einfüllen
                  CAN_vLoadData(ObjNr, commandPtr+5);
               }
               else
                  // CAN-Objekt war ungueltig
                  Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
               break;

            case 0x32:
               // Neues CAN-Objekt definieren
               NewCANObj.ubMsgObjNr = ObjNr;
               // 8-bit Message Configuration Register
               NewCANObj.ubMsgCfg = commandPtr[5];
               // standard (11-bit)/extended (29-bit) identifier
               NewCANObj.ulArbitr = readLong (commandPtr+6);
               // standard (11-bit)/extended (29-bit) Mask
               NewCANObj.ulMaskr  = readLong (commandPtr+10);

               // 8-bit Data Bytes
               for(i=0; i < 8; i++)
               {
                  NewCANObj.ubData[i] = commandPtr[14+i];
               }

               // Message Objekt konfigurieren
               CAN_bRequestMsgObj(ObjNr);
               CAN_vConfigMsgObj(ObjNr, &NewCANObj);
               break;

            case 0x33:
               if (CAN_bValMsgObj(ObjNr))
                  // CAN-Objekt loeschen
                  CAN_bDelMsgObj(ObjNr);
               else
                  // CAN-Objekt war ungueltig
                  Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
            break;

            // ------------------------------------------------------------------
            case 0x3E:
               Device_Init(CAN+3);
            break;
         } // end of switch
      }
      else
      {
         // Fehler "falsche Objekt Nummer" senden
         Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
      }
   } //  else if (commandPtr[3] == 0x1F)


   // CAN-Fehler mit aufmaskieren
   if (CAN_Error)
   {
      Error =  Error | ERROR_CAN | CAN_Error;
      // CAN-Fehler zurücksetzen
      CAN_Error = 0;
   }

   // Ergebnis senden
   writeLong(resultPtr+4, Error);
} // end of function


//****************************************************************************
//* Function     : CAN_RD                                                    *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*         This function sends the last CAN-Message from the RECEIVE-Buffer *
//*         (ReceivedCANObj[]).                                              *
//*         If no Objects are received this function sends an error code.    *
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

void CAN_RD(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8 i;                         // Laufvariable
UINT8 cmd = commandPtr[3];       // eigentlicher Befehl
UINT8 ObjNr;                     // Objekt Nummer des zu lesenden Objektes
UINT32 Error;
TCAN_Obj      TempCANObj;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

#if CP_CAN1
   // Unterverteiler auf CAN2-Routinen
   if ((commandPtr[3]) & 0x20)
   {  CAN1_RD ();
      return;
   }
#endif

   Error = NO_ERROR;

   // check for correct command
   if (((cmd == 0x30) && (commandPtr[0] == 4)) ||
       ((cmd == 0x31) && (commandPtr[0] == 5)) ||
       ((cmd == 0x32) && (commandPtr[0] == 4))
       )  // get KWP2000 status
   {}
   else
   {  // Interpreter Commandfehler defaultmässig schicken
      return;
   }


   switch (cmd)
   {
      case 0x30 :  // letzte Message des Receive-Message-Buffer auslesen
			if (CAN_ReceiveMsgCount)
  			{
            // Empfangene CAN-Messages vorhanden
    			// Anzahl der Messages noch im Puffer
    			resultPtr[8] = CAN_ReceiveMsgCount-1;
            // Füllbyte
            resultPtr[9] = 0xdd;
    		   // Message-Objekt-Nr. (1...32)
    			resultPtr[10] = ReceivedCANObj[CAN_ReceiveMsgCount-1].ubMsgObjNr;
            // Message-MSCFG
    		   resultPtr[11] = ReceivedCANObj[CAN_ReceiveMsgCount-1].ubMsgCfg;
    			// Message-ID
            writeLong(resultPtr+12, ReceivedCANObj[CAN_ReceiveMsgCount-1].ulArbitr);
            // Message-Mask
            writeLong(resultPtr+16, ReceivedCANObj[CAN_ReceiveMsgCount-1].ulMaskr);
     		   // Message-Data
    		   resultPtr[0] = (ReceivedCANObj[CAN_ReceiveMsgCount-1].ubMsgCfg & 0xF0) >> 4;
    		   for (i=0; i < resultPtr[0]; i++)
   		   {
       		   resultPtr[20+i] = ReceivedCANObj[CAN_ReceiveMsgCount-1].ubData[i];
    		   }
    		   resultPtr[0] += 20;

    		   // Buffer runterzählen
    		   if (CAN_ReceiveMsgCount > 0)
               CAN_ReceiveMsgCount--;
         }
  		   break;

      case 0x31:
         // Message Objekt abfragen
         ObjNr = commandPtr[4];

         if (ObjNr <= CAN_MAX_OBJ)
         {
            if (CAN_bValMsgObj(ObjNr))
            {
               // Objekt erstmal ohne Interrupt aus der Hardware auslesen
               CAN_vGetMsgObj(ObjNr, &TempCANObj);

               // Füllbytes
               resultPtr[8] = 0xdd;
  			      resultPtr[9] = 0xdd;
  			      // Message-Objekt-Nr. (0...CAN_MAX_OBJ)
 			      resultPtr[10] =  ObjNr;
               // Message-Configuration Register
               resultPtr[11] = TempCANObj.ubMsgCfg;
               // Message-ID Register
               writeLong(resultPtr+12, TempCANObj.ulArbitr);
               // Message-Mask Register
               writeLong(resultPtr+16, TempCANObj.ulMaskr);

               // Message-Data
    		      resultPtr[0] = (TempCANObj.ubMsgCfg & 0xF0) >> 4;
    		      for (i=0; i < resultPtr[0]; i++)
   		      {
       		      resultPtr[20+i] = TempCANObj.ubData[i];
    		      }
    		      resultPtr[0] += 20;
            }
            else
            {
               // Fehler
    		      Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
            }
         }
         else
         {
            // Fehler
    		   Error = ERROR_CAN | ERROR_CAN_WRONG_OBJ;
         }
         break;

      case 0x32 :  // Receive-Message-Buffer loeschen
         // Messagezähler zurücksetzen
         CAN_ReceiveMsgCount = 0;
      break;

      // ---------------------------------------------------------------------
   } // of switch (commandPtr...

   // CAN-Fehler mit aufmaskieren
   if (CAN_Error)
   {
      Error =  Error | ERROR_CAN | CAN_Error;
      // CAN-Fehler zurücksetzen
      CAN_Error = 0;
   }

   // Ergebnis senden
   writeLong(resultPtr+4, Error);

} // end of function


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

#if (CP_DEBUG == 5)
void Debug_Info (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

  // Fallunterscheidung für verschiedene Testbetriebsarten
  switch ( commandPtr[2])

    {
        case 0x00:   CAN3_Init(readLong (commandPtr+3));
							break;

        case 0x80:
            printf ("Can FCN3 Message control register number : %d\n",(UINT32)commandPtr[3]);
            printf ("FCNnMmDAT0B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[0].B);
            printf ("FCNnMmDAT1B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[1].B);
            printf ("FCNnMmDAT2B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[2].B);
            printf ("FCNnMmDAT3B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[3].B);
            printf ("FCNnMmDAT4B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[4].B);
            printf ("FCNnMmDAT5B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[5].B);
            printf ("FCNnMmDAT6B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[6].B);
            printf ("FCNnMmDAT7B  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDATx[7].B);

            printf ("FCNnMmDTLGB  : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmDTLGB);
            printf ("FCNnMmSTRB   : %02X\n",(UINT32)CAN3_A_OBJ[commandPtr[3]].FCNnMmSTRB);

            printf ("FCNnMmMID0H  : %04X\n",(UINT32)CAN3_B_OBJ[commandPtr[3]].FCNnMmMID0H);
            printf ("FCNnMmMID1H  : %04X\n",(UINT32)CAN3_B_OBJ[commandPtr[3]].FCNnMmMID1H);
            printf ("FCNnMmCTL    : %04X\n",(UINT32)CAN3_B_OBJ[commandPtr[3]].FCNnMmCTL);

            printf ("\n");
            break;

         case 0x81:
            printf ("Can FCN3 Interruptcounter number\n");
            printf ("INTFCNWUP_Counter  : %d\n",INTFCNWUP_Counter);
            printf ("INTFCN3TRX_Counter : %d\n",INTFCN3TRX_Counter);
            printf ("INTFCN3REC_Counter : %d\n",INTFCN3REC_Counter);
            printf ("INTFCN3ERR_Counter : %d\n",INTFCN3ERR_Counter);

            printf ("\n");
            break;

   } // end of case

   writeLong(resultPtr+4, 0);

} // end of Debug_Info
#endif // #if CP_DEBUG

// ------------------------------------------------------------------------------

#endif // CP_CAN3

