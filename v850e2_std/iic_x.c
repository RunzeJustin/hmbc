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
//*	Generischer C-Treiber als Include fuer C-Files                         *
//*--------------------------------------------------------------------------*

#include "hw_init.h"

// folgende Defines sind auch vordefinierbar
#ifndef SDA_HIGH
   #define   SDA_HIGH         { SDA_DIR = 1; }
#endif

#ifndef SDA_LOW
   #define   SDA_LOW          { SDA = 0; SDA_DIR = 0; }
#endif

#ifndef SCK_HIGH
   #define   SCK_HIGH         { SCK_DIR = 1; }
#endif

#ifndef SCK_LOW
   #define   SCK_LOW          { SCK = 0; SCK_DIR = 0; }
#endif

#ifndef SET_FPGA_REG_ADD
   #define  SET_FPGA_REG_ADD  {;}
#endif


#define   T    (us(500.0/IICxKBPS))   // Wait for half the period of the bit rate
#define   T2   (us(250.0/IICxKBPS))   // Wait for fourth period of the bit rate

//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

// Low-Level
void           IIC_InitX         (void);
static BOOL    IIC_StrtX         (void);
static void    IIC_StopX         (void);
static BOOL    IIC_WrtX          (UINT8 ToSend);
static UINT8   IIC_RdX           (void);
static UINT8   IIC_RdaX          (void);
static void    IIC_Master_AckX   (void);
static void    IIC_Master_NAckX  (void);
static BOOL    IIC_Test_AckX     (void);
static BOOL    IIC_SendX         (UINT8 ToSend);
static UINT8   IIC_ReceiveX      (void);
static BOOL    SCK_HighX         (void);

//****************************************************************************
//* Function     : SCK_HighX                                                 *
//****************************************************************************
//* Description  :                                                           *
//*                                                                          *
//* Clockstretching:   Wartet bis das angesprochene Device die Clock-        *
//*                    leitung freigibt. Wird diese Freigabe nicht           *
//*                    gegeben, wird nach Ablauf des Timeouts                *
//*                    ein Fehler zurückgegeben                              *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  : true,  wenn Clockleitung freigegeben wurde                *
//*                false, wenn Clockleitung nicht freigegeben wurde.         *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static BOOL SCK_HighX( void )
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT32 WaitCount;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   SCK_HIGH;

   WaitCount = 0;

   while ((SCK_IN == 0) && (WaitCount < MAXWAIT))
   {
      WaitCount++;
   }

   if (WaitCount == MAXWAIT)
   {
      return( FALSE );
   }
   else
   {
      return( TRUE );
   }
} // end of function


//****************************************************************************
//* Function     : IIC_WriteX_Str                                            *
//****************************************************************************
//* Description  : Schreibt eine Sequenz deren Länge übergeben wird an einen *
//*                Slave, dessen Adresse ebenfalls übergeben wird (I2C_BUSx) *
//*                                                                          *
//* Parameter    : Adresse                                                   *
//*                Pointer auf Schreibbuffer & Laenge                        *
//*                                                                          *
//* Returnvalue  : UINT32 ErrorWert                                          *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

UINT32 IIC_WriteX_Str (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length, UINT8 Mode)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8    i;
UINT32   Error;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   SET_FPGA_REG_ADD;
   Error = NO_ERROR;

#if IICxINIT
   // IIC Initialisieren
   IIC_InitX();
#endif

   // Start- Bedienung setzen
   if (!IIC_StrtX())
   {
      return (ERROR_I2C | ERROR_I2C_BUS_BUSY);
   }

   // I2C-Adresse write, when Receive complete, SDA is low, and Return the true by IIC_WrtX()
   if (!IIC_WrtX(Addr))
   {
      return (ERROR_I2C | ERROR_I2C_NO_DEVICE);
   }
   else
   {  //when slave there and ready
      for (i = 0 ;i < WR_Length; i++)
      {
         // dann ganze Sequenz in einer Schleife ausgeben
         if (!IIC_WrtX(WR_Buf[i]))
         {  //wenn Slave nicht quittiert, dann Senden abbrechen
            Error = ERROR_I2C | ERROR_I2C_NO_ACK;
            break;
         }
      }
   }

   if (Mode != 0x10)
      IIC_StopX();
   // Ruecksprung mit OK - Quittung
   return (Error);
} // end of function


//****************************************************************************
//* Function     : IIC_ReadX_Str                                             *
//****************************************************************************
//* Description  : Liest eine Sequenz deren Länge übergeben wird von einen   *
//*                Slave, dessen Adresse ebenfalls übergeben wird (I2C_BUSx) *
//*                Bei Mode = 1 wird das erste gelesene Byte als Länge       *
//*                gewertet und auch in den Buffer geschrieben               *
//*                                                                          *
//* Parameter    : Adresse                                                   *
//*                Pointer auf Lesebuffer & Laenge                           *
//*                Pointer auf Schreibbuffer & Laenge                        *
//*                                                                          *
//* Returnvalue  : UINT32 ErrorWert                                          *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************
UINT32 IIC_ReadX_Str (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length, UINT8 *RD_Buf, UINT8 RD_Length, UINT8 Mode)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8  i,
               Wert;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   SET_FPGA_REG_ADD;

   // Set Start Operation
   if (!IIC_StrtX())
   {
      return (ERROR_I2C | ERROR_I2C_BUS_BUSY);
   }

   //if no data followed after the address byte(D4 Page22), then set bit R / W equal to '1' means Master work as Receiver
   //Address is 7 bit(Ignor bit 0, not need to shift), R/W bit is bit0
   if (WR_Length == 0)
   {
      // Write I2C address
      if (!IIC_WrtX(Addr | 0x01))
      {
         return (ERROR_I2C | ERROR_I2C_NO_DEVICE);
      }
   }
   // there follow data after the address byte in the I2C bus to be sent
   else
   {
      // Write I2C address, since it need to write, so bit 0 is '0'
      if (!IIC_WrtX(Addr))
      {
         return (ERROR_I2C | ERROR_I2C_NO_DEVICE);
      }

      // then write whole sequence in a loop
      for (i = 0; i < WR_Length; i++)
      {
         IIC_WrtX(WR_Buf[i]);
      }

      // Set readback request
      IIC_StrtX();       //Start Condition
      if (!IIC_WrtX(Addr | 0x01))
      {
         return (ERROR_I2C | ERROR_I2C_NO_ACK);
      }
   } // else WR_Length==0

//*--------------------------------------------------------------------------*

   // automatic read the length, determination for CD drive query, if desired
   // There are 1 length L + L picked up x data bytes
   // with maximum limitation
   if (Mode == 1)
   {
      Wert = IIC_RdaX();
      RD_Length--;
      if (Wert < RD_Length)
         RD_Length = Wert;
      *RD_Buf++ = Wert;
   }

//*--------------------------------------------------------------------------*

   // Read automatic length determination for IIC gateway operation
   // There are 1 length L + ( L - 1 ) collected x data bytes
   if (Mode == 2)
   {
      RD_Length = IIC_RdaX();
      *RD_Buf++ = RD_Length;

      // String mit Null oder 1 Bytes Laenge nicht zulässig
      if (RD_Length < 2)
      {
         IIC_StopX();
         return (IICMSG_RX_PACKET_FORMAT_ERROR);
      }
      RD_Length--;
   }

//*--------------------------------------------------------------------------*

   // Read automatic length determination for INIC operation
   // There are 2 lengthbyte L_high and L_low + L picked up x data bytes
   if (Mode == INIC_I2C_MODE)
   {
      // Highbyte of Duration only read , not using
      Wert = IIC_RdaX();
      *RD_Buf++ = Wert;

      // read and evaluate the low byte length
      Wert = IIC_RdaX();
      *RD_Buf++ = Wert;

      // length limit
      if (Wert < RD_Length)
         RD_Length = Wert;
   }

//*--------------------------------------------------------------------------*

   // read data from the I2C bus
   for (i = 0; i < (RD_Length-1); i++)
   {  // Read entire sequence in a loop
      *RD_Buf++ = IIC_RdaX();
   }

   // read last byte without ACK , thus slave recognizes end
   *RD_Buf++ = IIC_RdX();

   // Set to final stop operation
   IIC_StopX();
   // Return jump with OK 
   return(NO_ERROR);
} // end of function


//****************************************************************************
//* Function     : IIC_StrtX                                                 *
//****************************************************************************
//* Description  : Triggering a start condition on the bus                   *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static BOOL IIC_StrtX (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
BOOL status = TRUE;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   DelayLoop (T);
   SDA_HIGH;

#if (IICxCS==1)
   // Macro SCH_HIGH durch Funktion: SCK_HighX(void) ersetzt
   status = SCK_HighX();
#else
   SCK_HIGH;
#endif
   DelayLoop (T);
   SDA_LOW;
   DelayLoop (T);
   return (status);

} // end of function


//****************************************************************************
//* Function     : IICStopX                                                  *
//****************************************************************************
//* Description  : Ausloesen einer Stop Bedingung auf dem Bus                *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static void IIC_StopX(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   DelayLoop (T);
   SDA_LOW;

#if (IICxCS==1)
   // Macro SCH_HIGH durch Funktion: SCK_HighX(void) ersetzt
   SCK_HighX();
#else
   SCK_HIGH;
#endif

   DelayLoop (T);
   SDA_HIGH;
} // end of function


//****************************************************************************
//* Function     : IIC_InitX                                                 *
//****************************************************************************
//* Description  : Triggering a start / stop condition on the bus            *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void IIC_InitX (void)
{
   SET_FPGA_REG_ADD;
   IIC_StrtX () ;
   IIC_StopX () ;
   DelayLoop (T);
} // end of function


//****************************************************************************
//* Function     : IIC_Master_AckX                                           *
//****************************************************************************
//* Description  : Senden eines Ack vom Master                               *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static void IIC_Master_AckX(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   DelayLoop (T);
   SDA_LOW;

#if (IICxCS==1)
   // replaced SCK_HighX (void ) : Macro SCH_HIGH by function
   SCK_HighX();
#else
   SCK_HIGH;
#endif
   DelayLoop (T);
   SCK_LOW;

} // end of function


//****************************************************************************
//* Function     : IIC_Master_NAckX                                          *
//****************************************************************************
//* Description  : Senden eines Nicht-Ack vom Master                         *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static void IIC_Master_NAckX(void)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   DelayLoop (T);
   SDA_HIGH;

#if (IICxCS==1)
   // Macro SCH_HIGH durch Funktion: SCK_HighX(void) ersetzt
   SCK_HighX();
#else
   SCK_HIGH;
#endif
   DelayLoop (T);
   SCK_LOW;

} // end of function


//****************************************************************************
//* Function     : IIC_Test_AckX                                             *
//****************************************************************************
//* Description  : Empfangen eines Ack vom Slave                             *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static BOOL IIC_Test_AckX (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8 i=0;
BOOL result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   DelayLoop (T);
   SDA_HIGH;

#if (IICxCS==1)
   // Macro SCH_HIGH durch Funktion: SCK_HighX(void) ersetzt
   SCK_HighX();
#else
   SCK_HIGH;
#endif

   do
   {
      DelayLoop (T);
      result = SDA_IN;
      if(result==0)
         break;
   }
   while(i++ < ACKxTRIALS);
   SCK_LOW;

   // Ignore ACK from slave device
#if (IICxIGNORE_ACK==1)
   return (1);
#else
   return (!result);
#endif
} // end of function


//****************************************************************************
//* Function     : IIC_SendX                                                 *
//****************************************************************************
//* Description  : Senden von 8 Bit Daten auf dem Bus                        *
//*                                                                          *
//*                                                                          *
//* Parameter    : UINT8: Datum                                              *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static BOOL IIC_SendX (UINT8 ToSend)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT16 n;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   SCK_LOW;
   DelayLoop (T);

   for (n = 0; n < 8; n++)
   {
      SCK_LOW;
      DelayLoop (T2);
      if (ToSend & 0x80)
      {
         SDA_HIGH;
      }
      else
      {
         SDA_LOW;
      }
      DelayLoop (T2);

//Used for ACK????, I think it's must needed
#if (IICxCS==1)
      if( SCK_HighX() )
      {
         DelayLoop (T);
         ToSend <<= 1;
      }
      else
         return (FALSE);
#else
      SCK_HIGH;
      DelayLoop (T);
      ToSend <<= 1;
#endif

   }
   SCK_LOW;
   return (TRUE);
} // end of function


//****************************************************************************
//* Function     : IIC_ReceiveX                                              *
//****************************************************************************
//* Description  : Empfangen von 8 Bit Daten auf dem Bus                     *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static UINT8 IIC_ReceiveX (void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
int n;
UINT8 received;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   SCK_LOW;
   SDA_HIGH;
   DelayLoop (T);
   received = 0;
   for (n = 0; n < 8; n ++)
   {

#if (IICxCS==1)
      // Macro SCH_HIGH durch Funktion: SCK_High0(void) ersetzt
      SCK_HighX();
#else
      SCK_HIGH;
#endif

      DelayLoop (T);
      received <<= 1;
      if (SDA_IN == 1) received |= 1;
      SCK_LOW;
      DelayLoop (T);
   }
   SCK_LOW;
   return (received);
} // end of function


//****************************************************************************
//* Function     : IICWrtX                                                   *
//****************************************************************************
//* Description  : Senden von 8 Bit Daten auf dem Bus mit anschl Test von Ack*
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static BOOL IIC_WrtX(UINT8 ToSend)
{
//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   if (!IIC_SendX (ToSend))
   {
     return FALSE;
   }
   return IIC_Test_AckX ();
} // end of function


//****************************************************************************
//* Function     : IICRdX                                                    *
//****************************************************************************
//* Description  : Lesen von 8 Bit Daten auf dem Bus mit anschliessendem NAck*
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static UINT8 IIC_RdX(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8 result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*
   result = IIC_ReceiveX();
   IIC_Master_NAckX();
   return(result);
} // end of function


//****************************************************************************
//* Function     : IICRdaX                                                   *
//****************************************************************************
//* Description  : Sending 8-bit data on the bus plus Ack                    *
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
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

static UINT8 IIC_RdaX(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*
UINT8 result;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*
   result = IIC_ReceiveX();
   IIC_Master_AckX();
   return(result);
} // end of function

