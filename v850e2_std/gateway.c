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
#include "global.h"

#include "tool.h"
#include <string.h>          		   // fuer memset
#include "gateway.h"


#include "ser.h"
#include "spi.h"

//*--------------------------------------------------------------------------*
//* Global variables                                                         *
//*--------------------------------------------------------------------------*

// schneller Arbeitsvariablensatz
UINT8    Gateway_Source[4],
         Gateway_Destination[4];

UINT32   Gateway_RS232_Baudrate[12];
UINT32   Gateway_SPI_Baudrate[6];

struct tGateway_MOST   Gateway_MOST[16];
struct tGateway_CAN    Gateway_CAN[16];


UINT32   Gateway_MOST_Errorcounter = 0;
UINT32   Gateway_CAN_Errorcounter = 0;

BOOL     Restart_Request = FALSE;

//*--------------------------------------------------------------------------*
//* Einstellparameter                                                        *
//*--------------------------------------------------------------------------*


// Parametersatz
struct tParameter 		Parameter;

//*--------------------------------------------------------------------------*
//* local variables                                                          *
//*--------------------------------------------------------------------------*

// Initialisierungsreihenfolge der Interfaces; Scriptinterface ist implizit
const UINT8 Device_Init_List [COMM_INTERFACES-1] =  COMM_INTERFACES_LIST;

// Unterstuetzte Clockmodes MOST
const BOOL MOST_Clock_List [4] = MOST_CLOCK_LIST;

// Unterstuetzte Transceivermodes CAN
const BOOL CAN_Transceiver_List [2] = CAN_TRANSCEIVER_LIST;

//*--------------------------------------------------------------------------*
//* prototypes                                                               *
//*--------------------------------------------------------------------------*

void Set_Parameter   (void);
BOOL Parameter_Ok    (void);


//****************************************************************************
//* Function     :	Set_Parameter                                           *
//****************************************************************************
//* Description  :   Parameter in die Arbeitsvariable laden                  *
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

void Set_Parameter (void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Verbindungsparameter  laden
   memcpy (Gateway_Source, Parameter.Gateway_Source, sizeof(Parameter.Gateway_Source));
   memcpy (Gateway_Destination, Parameter.Gateway_Destination, sizeof(Parameter.Gateway_Destination));

	// Baudrate RS232 laden
   memcpy (Gateway_RS232_Baudrate,Parameter.Gateway_RS232_Baudrate, sizeof(Parameter.Gateway_RS232_Baudrate));

   // Baudrate SPI laden
   memcpy (Gateway_SPI_Baudrate,Parameter.Gateway_SPI_Baudrate, sizeof(Parameter.Gateway_SPI_Baudrate));

	// Mostparameter laden
	memcpy (Gateway_MOST,Parameter.Gateway_MOST, sizeof(Parameter.Gateway_MOST));

   // CAN-Parameter auswaehlen
   memcpy (Gateway_CAN,Parameter.Gateway_CAN, sizeof(Parameter.Gateway_CAN));

} // end of Set_Parameter


//****************************************************************************
//* Function     :	Device_Exist                                            *
//****************************************************************************
//* Description  :   Device auf Gültigkeit testen                            *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   TRUE für ok,                                            *
//*						FALSE für undefiniert                                   *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL Device_Exist (UINT8 Device)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (Device == NONE)
      return(TRUE);

   switch (DEVICE_TYPE(Device))
   {
      case RS232 :   // Devices einzeln auf Vorhandensein ueberpruefen
                     if (!(CP_CMDDEV_RS232 & (1 << DEVICE_NR(Device))))
                        return(FALSE);
                     break;

      case MOST  :   // Devices einzeln auf Vorhandensein ueberpruefen
                     if (!(CP_CMDDEV_MOST & (1 << DEVICE_NR(Device))))
                        return(FALSE);
                     break;

      case IIC   :   // Devices einzeln auf Vorhandensein ueberpruefen
                     if (!(CP_CMDDEV_IIC & (1 << DEVICE_NR(Device))))
                        return(FALSE);
                     break;

      case CAN   :   // Devices einzeln auf Vorhandensein ueberpruefen
                     if (!(CP_CMDDEV_CAN & (1 << DEVICE_NR(Device))))
                        return(FALSE);
                     break;

      case SPI   :   // Devices einzeln auf Vorhandensein ueberpruefen
                     if (!(CP_CMDDEV_SPI & (1 << DEVICE_NR(Device))))
                        return(FALSE);
                     break;

      default:       return(FALSE);
   }

   return(TRUE);
} // end of Device_Exist


//****************************************************************************
//* Function     :	Device_Init                                             *
//****************************************************************************
//* Description  :   Device initialisieren                                   *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   TRUE für ok,                                            *
//*						FALSE für undefiniert                                   *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL Device_Init (UINT8 Device)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (!Device_Exist(Device))
      return(FALSE);

   switch (DEVICE_TYPE(Device))
   {
      #if CP_CMDDEV_RS232
      case RS232 :   RS232_Init (DEVICE_NR(Device), Gateway_RS232_Baudrate[DEVICE_NR(Device)]);
                     break;
      #endif
      #if CP_CMDDEV_MOST
      case MOST  :   MOSTMSG_Init (DEVICE_NR(Device));
                     break;
      #endif
      #if CP_CMDDEV_IIC
      case IIC   :   IICMSG_Init ();
                     break;
      #endif
      #if CP_CMDDEV_CAN
      case CAN   :   CANMSG_Init (DEVICE_NR(Device));
                     break;
      #endif
      #if CP_CMDDEV_SPI
      case SPI   :   SPIMSG_Init (DEVICE_NR(Device), Gateway_SPI_Baudrate[DEVICE_NR(Device)]);
                     break;
      #endif

      default:       return(FALSE);
   }
   return(TRUE);
} // end of Device_Init


//****************************************************************************
//* Function     :	Device_Close                                            *
//****************************************************************************
//* Description  :   Device schliessen                                       *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   TRUE für ok,                                            *
//*						FALSE für undefiniert                                   *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL Device_Close (UINT8 Device)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (!Device_Exist(Device))
      return(FALSE);

   switch (DEVICE_TYPE(Device))
   {
      #if CP_CMDDEV_RS232
      case RS232 :   break;
      #endif
      #if CP_CMDDEV_MOST
      case MOST  :   break;
      #endif
      #if CP_CMDDEV_IIC
      case IIC   :   break;
      #endif
      #if CP_CMDDEV_CAN
      case CAN   :   break;
      #endif
      #if CP_CMDDEV_SPI
      case SPI   :   SPIMSG_Close (DEVICE_NR(Device));
                     break;
      #endif

      default:       return(FALSE);
   }
   return(TRUE);
} // end of Device_Close

//****************************************************************************
//* Function     :	Parameter_Ok                                            *
//****************************************************************************
//* Description  :   Parameter auf Gültigkeit testen                         *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :   TRUE für ok,                                            *
//*						FALSE für 1 oder mehr Parameter ausserhalb des Bereichs *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

BOOL Parameter_Ok (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 	i,j;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Gatewaysource und Gatewaydestination auf generelle Gueltigkeit und
   // auf Doppel testen; kein Wert darf zweimal vorkommen
   for (i=0; i<4; i++)
   {
      if (!Device_Exist (Parameter.Gateway_Source[i]))
         return(FALSE);

      if (!Device_Exist (Parameter.Gateway_Destination[i]))
         return(FALSE);

      if (!((Parameter.Gateway_Source[i] == NONE) && (Parameter.Gateway_Destination[i] == NONE)))
      {
         // gleiche Adresse zwar moeglich aber nicht erlaubt (unechtes Gateway)
         if (Parameter.Gateway_Source[i] == Parameter.Gateway_Destination[i])
            return(FALSE);

         // Umleitung auf NONE nicht gestattet
         if ((Parameter.Gateway_Source[i] == NONE) || (Parameter.Gateway_Destination[i] == NONE))
            return(FALSE);

         // auf Doppel scannen
         for (j = 0; j < i ; j++)
         {
            if ((Parameter.Gateway_Source[j]      == Parameter.Gateway_Source[i]) ||
                (Parameter.Gateway_Destination[j] == Parameter.Gateway_Source[i]) ||
                (Parameter.Gateway_Source[j]      == Parameter.Gateway_Destination[i]) ||
                (Parameter.Gateway_Destination[j] == Parameter.Gateway_Destination[i]))
               return(FALSE);

            // Umleitung auf NONE nicht gestattet
            if ((Parameter.Gateway_Source[j] == NONE) || (Parameter.Gateway_Destination[j] == NONE))
               return(FALSE);
         } // of j
      }
   }  // of for i


   // alle moeglichen Baudrates RS232 auf Bereich überprüfen
   for (i=0; i<12; i++)
   {
      if ((Parameter.Gateway_RS232_Baudrate[i] > 1000000)
         || (Parameter.Gateway_RS232_Baudrate[i] < 2400))
            return (FALSE);
   }

   // alle moeglichen Baudrates SPI auf Bereich überprüfen
   for (i=0; i<6; i++)
   {
      if ((Parameter.Gateway_SPI_Baudrate[i] > 10000000)
         || (Parameter.Gateway_SPI_Baudrate[i] < 1000))
            return (FALSE);
   }

   // MOST-Parameter fuer alle Instanzen ueberpruefen
   for (i=0; i<16; i++)
   {
      if (Parameter.Gateway_MOST[i].Clock >= MOSTCLOCK_UNGUELTIG)
         return (FALSE);

      // Clockmode pruefen, ob unterstuetzt
      if (!MOST_Clock_List[Parameter.Gateway_MOST[i].Clock])
         return (FALSE);

      // nur noch 4 Formate unterstuetzen
      if ((Parameter.Gateway_MOST[i].Format != MOSTFORMAT_PACKET_SLAVE) &&
          (Parameter.Gateway_MOST[i].Format != MOSTFORMAT_PACKET_MASTER) &&
          (Parameter.Gateway_MOST[i].Format != MOSTFORMAT_MOST150) &&
          (Parameter.Gateway_MOST[i].Format != MOSTFORMAT_MOST150_AUTO))
         return (FALSE);

      // Gateway Nodeadressen auf Bereich überprüfen
	   if (Parameter.Gateway_MOST[i].Nodeadress > 0x0FFF)
         return (FALSE);

      if (Parameter.Gateway_MOST[i].TargetNodeadress > 0x0FFF)
         return (FALSE);
   }

   // CAN-Parameter fuer alle Instanzen ueberpruefen
   for (i=0; i<16; i++)
   {
     // CAN-Parameter ueberpruefen
      if (Parameter.Gateway_CAN[i].RXidentifier > 0x07FF)
         return (FALSE);

      if (Parameter.Gateway_CAN[i].TXidentifier > 0x07FF)
         return (FALSE);

      // Baudrate auf Bereich überprüfen
      if ((Parameter.Gateway_CAN[i].Baudrate > 1000000) || (Parameter.Gateway_CAN[i].Baudrate < 80000))
         return (FALSE);

      // Wartezeit auf Bereich überprüfen
      if (Parameter.Gateway_CAN[i].TXwait > 10000)
         return (FALSE);

      // Transceivereinstellung
      if (Parameter.Gateway_CAN[i].Transceiver >= CAN_UNGUELTIG)
         return (FALSE);

      // Transceivereinstellung pruefen, ob unterstuetzt
      if (!CAN_Transceiver_List[Parameter.Gateway_CAN[i].Transceiver])
         return (FALSE);
   }

   // Test bestanden
	return (TRUE);
} // end of Parameter_Ok


//****************************************************************************
//* Function     :	Init_Gateway_Standard                                   *
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

void Init_Gateway_Standard (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT8 	i;
UINT32   CAN_Baudrate;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Gatewayparameter  0 setzen
   for (i=0; i<4; i++)
   {
      Parameter.Gateway_Source[i] = NONE;
   }

   // Gatewayparameter  0 setzen
   for (i=0; i<4; i++)
   {
      Parameter.Gateway_Destination[i] = NONE;
   }

   // Standard-Baudrate in alle einsetzen
   for (i=0; i<12; i++)
   {
      Parameter.Gateway_RS232_Baudrate[i] = BAUDRATE_RS232;
   }

   // Standard-Baudrate in alle einsetzen
   for (i=0; i<6; i++)
   {
      Parameter.Gateway_SPI_Baudrate[i] = BAUDRATE_SPI;
   }

   // Baudrate aus config.h
   CAN_Baudrate = GATEWAY_CAN_BAUDRATE;

   // MOST-Parameter fuer alle Instanzen setzen
   for (i=0; i<16; i++)
   {
      Parameter.Gateway_MOST[i].Clock              = MOSTMODE_CLOCK;
      Parameter.Gateway_MOST[i].Format             = MOSTMODE_FORMAT;
      Parameter.Gateway_MOST[i].Nodeadress         = MOST_NODEADRESS;
      Parameter.Gateway_MOST[i].TargetNodeadress   = MOST_TARGETNODEADRESS;
   }

   // CAN-Parameter fuer alle Instanzen setzen
   for (i=0; i<16; i++)
   {
      Parameter.Gateway_CAN[i].RXidentifier  = GATEWAY_CAN_RXID;
      Parameter.Gateway_CAN[i].TXidentifier  = GATEWAY_CAN_TXID;
      Parameter.Gateway_CAN[i].Baudrate      = CAN_Baudrate;
      Parameter.Gateway_CAN[i].TXwait        = 0;
      Parameter.Gateway_CAN[i].Transceiver   = CAN_FAULTTOLERANT;
   }

//*--------------------------------------------------------------------------*

  	// in die Arbeitsvariable übertragen
   Set_Parameter ();

   // die Kommunikation nach Startliste  initialisieren
   for (i=0; i < (COMM_INTERFACES-1) ; i++)
      Device_Init (Device_Init_List[i]);

}  // end of Init_Gateway_Standard


//****************************************************************************
//* Function     :	Gateway_Interpreter                                     *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :	-                                                       *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :	                                                        *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (X) fully tested   *
//****************************************************************************

void Gateway_Interpreter (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

UINT32 	Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

	// Error value default command use
	Error = NO_ERROR;

	// standard command copy ever
	resultPtr[1]  = commandPtr[1];
 resultPtr[3]  = commandPtr[3];

 // Mark response
	resultPtr[2]  = 0xFF;

 printf("CMD[3] is %x\n", commandPtr[3]);

	  // Case analysis for sub-commands
 	 switch ( commandPtr[3])
   {

//*--------------------------------------------------------------------------*
//*	Start new Devivetreiber                                                  *
//*--------------------------------------------------------------------------*

      case 0xA1 : if (commandPtr[0] != 5)
                  {
                     resultPtr[0] = 8;
                     Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                     // Error value used and Confirming
						               writeLong(resultPtr+4, Error);
                  }
            						else
            						{
            						   resultPtr[0] = 8;

                                 // perform
            						   if  (!Device_Init (commandPtr[4]))
                                    Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;

                                 // Error value used and Confirming
            						   writeLong(resultPtr+4, Error);
                  }
						break;

//*--------------------------------------------------------------------------*
//*	to query or set parameters                                               *
//*--------------------------------------------------------------------------*

      case 0xCC : // Test whether the device exists

                  printf("CMD02 is CC\n");
      
                  if (!Device_Exist (commandPtr[4]))
                  {
                     Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                     resultPtr[0] = 8;
                  }
                  else
                  {
                     if (commandPtr[0] == 0x05)
                     {  // read parameter
         				             switch(DEVICE_TYPE(commandPtr[4]))
              						      {
              						         case RS232 :
                                            writeLong(resultPtr+8, Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])]);
                                            resultPtr[0] = 12;
                                            break;

                             case SPI :
                                            writeLong(resultPtr+8, Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])]);
                                            resultPtr[0] = 12;
                                            break;

                             case CAN :
                                            // determine value
                          							           writeWord(resultPtr+8, Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier);
                                            writeWord(resultPtr+10, Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier);
                          							           writeLong(resultPtr+12, Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate);
                                            writeWord(resultPtr+16, Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait);
                                            resultPtr[18] = Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver;
              		              					         resultPtr[0] = 19;
                                            break;

                             case MOST :
                                            // determine value
                                            resultPtr[8]  = Gateway_MOST[DEVICE_NR(commandPtr[4])].Format;
                                            writeWord(resultPtr+9, Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress);
                                            writeWord(resultPtr+11, Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress);
                                            resultPtr[13] = Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock;
                                            resultPtr[0] = 14;
                                            break;

                             default:
                                            Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                                            resultPtr[0] = 8;
                                            break;
                 			       } // of switch (DEVICE_TYPE
                     }
						               else
            						   {  // set parameters
            						      switch(DEVICE_TYPE(commandPtr[4]))
            						      {
            						         case RS232 :
            						            Parameter.Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])] = readLong (commandPtr+6);
                                          // Security check for validity
                                          if (Parameter_Ok() && (commandPtr[0] == 10))
                                          {
                                             Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])] = Parameter.Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])];
                                          }
                                          else
                                          { 	// make change undo
            								                         Parameter.Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])] = Gateway_RS232_Baudrate[DEVICE_NR(commandPtr[4])];
            		                               Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
            							                       }
                                          resultPtr[0] = 8;
                                          break;

                            case SPI :
            						            Parameter.Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])] = readLong (commandPtr+6);
                                          // Security check for validity
                                          if (Parameter_Ok() && (commandPtr[0] == 10))
                                          {
                                             Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])] = Parameter.Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])];
                                          }
                                          else
                                          { 	// make change undo
            								                         Parameter.Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])] = Gateway_SPI_Baudrate[DEVICE_NR(commandPtr[4])];
            		                               Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
            							                       }
                                          resultPtr[0] = 8;
                                          break;

            				            case CAN :
            						            // Security check for validity for all at once
                              				Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier = readWord (commandPtr+5);
                              				Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier = readWord (commandPtr+7);
                              				Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate     = readLong (commandPtr+9);

                                          // optional parameter waiting time
                                          if (commandPtr[0] >= 15)
                                             Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait = readWord (commandPtr+13);

                                           // optional parameters Transceiver
                                          if (commandPtr[0] == 16)
                                             Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver = commandPtr[15];

                                          // Security check for validity
                                          if (Parameter_Ok() && ((commandPtr[0] == 13) || (commandPtr[0] == 15) || (commandPtr[0] == 16)) )
                                          { 	// Enter parameters immediately
                                        					Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier = Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier;
                                        					Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier = Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier;
                                        					Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate     = Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate;
                                        					Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait       = Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait;
                                        					Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver  = Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver;
                                      				}
                              				        else
                                      				{ 	// make change undo
                                        					Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier = Gateway_CAN[DEVICE_NR(commandPtr[4])].RXidentifier;
                                        					Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier = Gateway_CAN[DEVICE_NR(commandPtr[4])].TXidentifier;
                                        					Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate     = Gateway_CAN[DEVICE_NR(commandPtr[4])].Baudrate;
                                        					Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait       = Gateway_CAN[DEVICE_NR(commandPtr[4])].TXwait;
                                        					Parameter.Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver  = Gateway_CAN[DEVICE_NR(commandPtr[4])].Transceiver;
                                        					Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                                      				}

                                          resultPtr[0] = 8;
                                          break;

                             case MOST :
            						                Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Format           = commandPtr [5];
            						                Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress       = readWord (commandPtr+6);
                              				Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress = readWord (commandPtr+8);
                              				Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock            = commandPtr [10];

                                      				if (Parameter_Ok() && (commandPtr[0] == 11))
                                          { 	// Enter parameters immediately
                                      				 	Gateway_MOST[DEVICE_NR(commandPtr[4])].Format           = Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Format;
                                      					 Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress       = Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress;
                                      					 Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress = Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress;
                                      					 Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock            = Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock;
                                      				}
                                          else
                                      				{ 	// make change undo
                                      					Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Format           = Gateway_MOST[DEVICE_NR(commandPtr[4])].Format;
                                      					Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress       = Gateway_MOST[DEVICE_NR(commandPtr[4])].Nodeadress;
                                      					Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress = Gateway_MOST[DEVICE_NR(commandPtr[4])].TargetNodeadress;
                                      					Parameter.Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock            = Gateway_MOST[DEVICE_NR(commandPtr[4])].Clock;
                                      					Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                                      				}

                                          resultPtr[0] = 8;
                                          break;

            						         default:
                                          Error = ERROR_GATEWAY | GATEWAY_PARAM_ERROR;
                                          resultPtr[0] = 8;
                                          break;
                                    } // of switch (DEVICE_TYPE
                                 } // if (commandPtr[0]
                               } // if (!Device_Exist

                               // Insert Error value
             					             writeLong(resultPtr+4, Error);
                               // and report back
                               break;

//*--------------------------------------------------------------------------*
//*                                                                          *
//*--------------------------------------------------------------------------*

      default :  	Error = ERROR_GATEWAY | GATEWAY_CMD_ERROR;
                  printf("CMD ERROR\n");
                  resultPtr[0] = 8;

						// Error value used and Confirming
						writeLong(resultPtr+4, Error);
                  break;

   } // end of switch

} // end of Gateway_Interpreter


