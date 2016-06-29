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


#ifndef _GATEWAY_H
#define _GATEWAY_H

//*--------------------------------------------------------------------------*
//* alle Gateway Errors                                                      *
//*--------------------------------------------------------------------------*

// da Fehler kombiniert vorkommen können, ist das Ver-Odern verschiedener Fehler erlaubt

#define GATEWAY_PARAM_ERROR		   0x0001      // ungültiger Parameter
#define GATEWAY_CMD_ERROR     	   0x0002	   // unbekanntes Kommando

//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*

extern void    Gateway_Interpreter     (void); 	// Subinterpreter Gateway
extern void    Init_Gateway_Standard   (void);

//*--------------------------------------------------------------------------*
//* Typen                                                                    *
//*--------------------------------------------------------------------------*

struct tGateway_MOST
   {  // einstellbare Parameter für den Gatewaybetrieb ueber MOST
      UINT8       Clock,
                  Format;
	  	UINT16		Nodeadress,
					   TargetNodeadress;
   };

struct tGateway_CAN
   {  // einstellbare Parameter für den Gatewaybetrieb ueber CAN
      UINT16		RXidentifier,
					   TXidentifier;
      UINT32      Baudrate;
      UINT16      TXwait;
      UINT8       Transceiver;
   };

// Referenzparameter
struct tParameter
   {  // einstellbare Parameter aus dem Eeprom
		UINT8		   Gateway_Parameterversion,
                  Gateway_Source[4],
                  Gateway_Destination[4];

      UINT32      Gateway_RS232_Baudrate[12];
      UINT32      Gateway_SPI_Baudrate[6];


      struct tGateway_MOST    Gateway_MOST[16];
      struct tGateway_CAN     Gateway_CAN[16];
   };

//*--------------------------------------------------------------------------*
//* externe Variable                                                         *
//*--------------------------------------------------------------------------*

// schneller Arbeitsvariablensatz

extern UINT32     Gateway_RS232_Baudrate[12];
extern UINT32     Gateway_SPI_Baudrate[6];

extern struct tGateway_MOST   Gateway_MOST[16];
extern struct tGateway_CAN    Gateway_CAN[16];


extern UINT32     Gateway_MOST_Errorcounter;
extern UINT32     Gateway_CAN_Errorcounter;

// Parametersatz
extern struct tParameter      Parameter;


extern BOOL Restart_Request;

//*--------------------------------------------------------------------------*
//* Global Defines                                                           *
//*--------------------------------------------------------------------------*

// für die Datensenken und Datenquellen Einstellung
#define RS232           0x00
#define MOST            0x10
#define IPC             0x20
#define IIC             0x30
#define CAN             0x40
#define JTAG            0x50
#define SCRIPT          0x60
#define SALIERI         0x70
#define SPI             0x80
#define SHRDMEM         0x90
#define NONE            0xFF

// fuer Deviceverwaltung
#define DEV_0           (1<<0)
#define DEV_1           (1<<1)
#define DEV_2           (1<<2)
#define DEV_3           (1<<3)
#define DEV_4           (1<<4)
#define DEV_5           (1<<5)
#define DEV_6           (1<<6)
#define DEV_7           (1<<7)
#define DEV_8           (1<<8)
#define DEV_9           (1<<9)
#define DEV_10          (1<<10)
#define DEV_11          (1<<11)
#define DEV_12          (1<<12)
#define DEV_13          (1<<13)
#define DEV_14          (1<<14)
#define DEV_15          (1<<15)

#define DEV_ALL         0xFFFF


// zum Extrahieren der Einstellung aus den Datensenken und Datenquellen
#define DEVICE_TYPE(A)  (A & 0xF0)
#define DEVICE_NR(A)    (A & 0x0F)


// für den Verbindungsaufbau
#define GATEWAY         0x00
#define BIOS            0x01

// für die MOST-Treiberauswahl
#define MOSTDRIVER_IIC 	   0           // Kommunikation über IIC mit  OS8104
#define MOSTDRIVER_PAR     1				// Kommunikation über Parallelport mit OS8104
#define MOSTDRIVER_MOPS    2				// Kommunikation über Mops (FPGA) mit OS8104
#define MOSTDRIVER_FPGA    3           // Kommunikation über Bioscontrol-FPGA mit OS8104
#define MOSTDRIVER_SSC 		4         	//	Kommunikation über IIC mit  OS880x (8804 oder 8805)
#define MOSTDRIVER_INIC    5           // Kommunikation über OS81050

// fuer die Mostformate
#define MOSTFORMAT_8104          0
#define MOSTFORMAT_SAHARA        1
#define MOSTFORMAT_STANDARD      2
#define MOSTFORMAT_PACKET        3
#define MOSTFORMAT_PACKET2       4
#define MOSTFORMAT_PACKET_SLAVE  5
#define MOSTFORMAT_PACKET_MASTER 6
#define MOSTFORMAT_MOST150       7
#define MOSTFORMAT_MOST150_AUTO  8     // Unlock recovery
#define MOSTFORMAT_UNGUELTIG     9


// fuer die Mostclocks
#define MOSTCLOCK_44K1HZ      0
#define MOSTCLOCK_SLAVE       1
#define MOSTCLOCK_48KHZ       2
#define MOSTCLOCK_SPDIF       3
#define MOSTCLOCK_UNGUELTIG   4

// fuer CAN
#define CAN_FAULTTOLERANT     0
#define CAN_HIGHSPEED         1
#define CAN_UNGUELTIG         2

// Chipauswahl fuer MOST
#define OS8104_IIC            0
#define OS8104_PAR            1
#define OS81050               2
#define OS81110               3


#endif // _GATEWAY_H

