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


#ifndef _CONFIG_H
#define _CONFIG_H

//*--------------------------------------------------------------------------*
//* Enthält alle projektabhängigen Definitionen                              *
//*--------------------------------------------------------------------------*

#include "cpu_def.h"
#include "gateway.h"
#include "config_comm.h"
#include "define.h"
#include "moduls.h"

//*--------------------------------------------------------------------------*
//* Setup Kommunikation                                                      *
//*--------------------------------------------------------------------------*

#define CP_CMDDEVICE          3     // bei mehreren Chips Kommandobaustein waehlen
                                    // OS8104 IIC  0
                                    // OS8104 PAR  1
                                    // OS81050     2
                                    // OS81110     3
// 0 : kein Device vorhanden    DEV_0..DEV_F : entsprechende Devices vorhanden
// verodern mehrer Devices moeglich  aber nur mit Klammern (...)
#define CP_CMDDEV_MOST        DEV_0       // registration beim MBC Handler
#define CP_CMDDEV_IIC         0
#define CP_CMDDEV_CAN         0           // DEV_0
#define CP_CMDDEV_SPI         0
#define CP_CMDDEV_RS232       DEV_11

// Initialisierungsreihenfolge der Interfaces; Scriptinterface ist implizit
// Anzahl :  COMM_INTERFACES -1
// RS232, MOST, IPC, IIC, CAN
// low nibble : Instance of Device 0..F
#define COMM_INTERFACES_LIST  { RS232+11, MOST+0, NONE}

#define CP_STARTMELDUNG       1     // Startmeldung automatisch ausgeben

// Unterstuetzte Clockmodes MOST: TRUE = Unterstuetzt
// MOSTCLOCK_44K1HZ, MOSTCLOCK_SLAVE, MOSTCLOCK_48KHZ, MOSTCLOCK_SPDIF
#define MOST_CLOCK_LIST       { TRUE, TRUE, TRUE, TRUE }

// Unterstuetzte Transceivermodes CAN: TRUE = Unterstuetzt
// CAN_FAULTTOLERANT, CAN_HIGHSPEED
#define CAN_TRANSCEIVER_LIST  { TRUE, TRUE }

//*--------------------------------------------------------------------------*
//* abgeleitete Defines                                                      *
//*--------------------------------------------------------------------------*

#define CP_INIC               (CP_OS81050 || CP_OS81110)

//*--------------------------------------------------------------------------*
//* Debugschalter                                                            *
//*--------------------------------------------------------------------------*

#define CP_DEBUG              0     // y-Befehl als Entwicklungswerkzeug
                                    // 0  = Debugging OFF
                                    // 1  = Debugging MOST
                                    // 2  = Debugging SSC
                                    // 3  = Debugging HPIPC
                                    // 4  = Debugging UHF-Receive
                                    // 5  = CAN3
                                    // 6  = CAN0
                                    // 7  = Debugging MOSTMOPS
                                    // 8  = Debugging MOSTFPGA
                                    // 9  = Debugging IIC5 FPGA
                                    // 10 = SSC2
                                    // 11 = Virtuell ADC
                                    // 12 = SSC3
                                    // 13 = IIC Message
                                    // 14 = Debugging IIC6 FPGA
                                    // 15 = Debugging CANMSG
                                    // 16 = Spezialtextmode ein fuer RS232
                                    // 17 = Timertest
                                    // 18 = SER
                                    // 19 = SPI 0-5
                                    // 20 = Debug Tracer CANMSG

#define CP_DEBUG_TOOL         0     // Hilfsfunktionen in TOOL.C einschalten

//*--------------------------------------------------------------------------*
//* CPU-Quartzfrequenz hier richtig eintragen                                *
//*--------------------------------------------------------------------------*

// Quartzfrequenz
//#define  MAIN_OSZ    20             // 20MHz
#define  MAIN_OSZ    12             // 12 MHz beim M033


// Duration Timer " DelayLoop ( ) " , for the time being trimmed fix at 80 Mhz CPU frequency
#define us(A)  (UINT32)(A*20) // Microsecond duration

//*--------------------------------------------------------------------------*
//* Standard-Versionsstring für BIOS-Control                                 *
//*--------------------------------------------------------------------------*

#define Date         "2016-06-02"
#define Time         "08:08:00"
#define Version      "MOSTAMP-L122-B1-MBC_Ver:-T01"
#define BoardVersion  "L122-1010-10JD"
#define BoardVersion2 "L122-1010-10JD"


// Default baud rate , not everything is possible at every clock
#define BAUDRATE_RS232        115200
#define BAUDRATE_SPI          125000
                                             // RS232 Receive

#define MODE_STARTVALUE       0x00000021     // Value after reset

#define HOSTTIMEOUTVALUE      60000          // maximum wait time [ ms ] on the first valid command
                                               //after startup , if enabled in the Makefile

#define CP_SPI_J5_BOOT   0                   // Jacinto IPL Bootcontrol


//-------------------------------------------------------------------------
//  Autorunskript
//-------------------------------------------------------------------------

// Erste freie Adresse nach Programmende aus Linkerfile importieren
#pragma ghs startdata
extern const char  EndProg[];
#pragma ghs enddata

// select one
// #define AUTORUNSCRIPT         0x20000              // Startadresse fix
#define AUTORUNSCRIPT           (UINT32)&EndProg      // Startadresse  variable

//*--------------------------------------------------------------------------*
//* Adressrange Scripte                                                      *
//*--------------------------------------------------------------------------*

   // alle Adressen ausserhalb werden zurueckgewiesen
   #define SCRIPT_RANGE1_STARTADDRESS     0x00000000
   #define SCRIPT_RANGE1_ENDADDRESS       0xFFFFFFFF

   #define SCRIPT_RANGE2_STARTADDRESS     0x00000000
   #define SCRIPT_RANGE2_ENDADDRESS       0xFFFFFFFF

//*--------------------------------------------------------------------------*
//* Businterface                                                             *
//*--------------------------------------------------------------------------*

// keine Hardware dafuer vorhanden
#define BUSINTERFACE_MISSING           0

// keines gewaehlt; alles I/O
#define NO_BUS                         1

// Ram
#define CS2_RAM_4M_16BIT_MPX           2




// gewaehlt ist :
#define BUSINTERFACE                   BUSINTERFACE_MISSING

// wenn 1, dann wird Businterface schon beim CPU-Start gesetzt, ansonsten Funktion selber aufrufen
#define SET_BUSINTERFACE_EARLY         1

//*--------------------------------------------------------------------------*
//* System-Timer, Led  und Clock                                             *
//*--------------------------------------------------------------------------*

// diese Funktion wird im 1 kHz Timerinterupt aufgerufen; bitte kurz (!!) halten
// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
//#define SERVE_WATCHDOG     {_PCT1 = !_PCT1}
#define SERVE_WATCHDOG {static UINT32 watch_cnt; if (watch_cnt & 0x01) {SETPORT_1(11,2);} else {SETPORT_0(11,2);} watch_cnt++;}

// diese Funktion wird im CPU_Init aufgerufen
// hier Leerdefinition einsetzen, wenn nicht unterstuetzt

// fplss/4
//#define SET_CLOCKOUT {PCLM = 0x10; _PFCE913=1;_PFC913=0;_PMC913=1; }
#define SET_CLOCKOUT {}

// define if not done in portlist
#define LED_MESSAGE_ACTIVATE     {LED_YELLOW_ACTIVATE;}
#define LED_MESSAGE_DEACTIVATE   {LED_YELLOW_DEACTIVATE;}

//*--------------------------------------------------------------------------*
//* I2C                                                                      *
//*--------------------------------------------------------------------------*

#define MAXWAIT         0x00080000     //  1 Wait = ?? ns (noch nachmessen)

//*--------------------------------------------------------------------------*

#define IIC0_CS            1           //  I2C Nr.0 clock stretching on / off???
#define IIC0_KBPS          240.0       //  I2C-Nr.0 Datenrate; M033 : 100 kHz gemessen
#define IIC0_IGNORE_ACK    0           //  I2C-Nr.0 Acknowledegefehler ignorieren
#define IIC0_ACK_TRIALS    100         //  I2C-Nr.0 Warten auf ACK vom Slave
#define IIC0_INIT          1           //  I2C-Nr.0 Start-Stop Init einschalten

// Portpins allocated to IIC, P25_12 for SDA1, P25_13 for SCL1, P1_14 for IIC MUX Control
#define IIC0_HWINIT        {SETPORT_PORT(25, 12); SETPORT_PORT(25, 13);}

#define IIC0_SDA_IN       (INPORT_BOOL(25,12))
#define IIC0_SDA_HIGH     {SETPORT_IN(25, 12);}
#define IIC0_SDA_LOW      {SETPORT_0(25, 12); SETPORT_OUT(25, 12);}

#define IIC0_SCK_IN       (INPORT_BOOL(25, 13))
#define IIC0_SCK_HIGH     {SETPORT_IN(25, 13);}
#define IIC0_SCK_LOW      {SETPORT_0(25, 13); SETPORT_OUT(25, 13);}

//*--------------------------------------------------------------------------*
//*          Marco Deifne for IIC1--- HW I2C                                 *
//*--------------------------------------------------------------------------*
#define IIC1_CS            1           //  I2C Nr.0 clock stretching on / off???
#define IIC1_KBPS          240.0       //  I2C-Nr.0 Datenrate; M033 : 100 kHz
#define IIC1_IGNORE_ACK    0           //  I2C-Nr.0 Acknowledge Ignore
#define IIC1_ACK_TRIALS    100         //  I2C-Nr.0 Waiting for ACK from the slave
#define IIC1_INIT          1           //  I2C-Nr.0 Start-Stop Init einschalten

// Portpins allocated to IIC, P25_12 for SDA1, P25_13 for SCL1
#define IIC1_HWINIT        {SETPORT_PORT(25, 10); SETPORT_PORT(25, 11);}

#define IIC1_SDA_IN       (INPORT_BOOL(25,10))
#define IIC1_SDA_HIGH     {SETPORT_IN(25, 10);}
#define IIC1_SDA_LOW      {SETPORT_0(25, 10); SETPORT_OUT(25, 10);}

#define IIC1_SCK_IN       (INPORT_BOOL(25, 11))
#define IIC1_SCK_HIGH     {SETPORT_IN(25, 11);}
#define IIC1_SCK_LOW      {SETPORT_0(25, 11); SETPORT_OUT(25, 11);}

//*--------------------------------------------------------------------------*
//* SER                                                                      *
//*--------------------------------------------------------------------------*

// allgemein
// Wartezeit bei CTS-Steuerung
#define SER_CTS_TIMEOUT       2000       // [ms]

// beim Frontkontrollerprotokoll
#define FRT_MESSAGE_BUF_SIZE   ((64+5) * 32)     // max 32 Messages je 64 Bytes (je 5 Bytes Overhead)

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SER3_RTS_ACTIVATE     {}
#define SER3_RTS_DEACTIVATE   {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SER3_CTS              TRUE

// RTS und CTS Portpins bei Bedarf aus der Alternatefunction holen (z.B. Hardware INT)
#define SER3_INIT             {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SER5_RTS_ACTIVATE     {}
#define SER5_RTS_DEACTIVATE   {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SER5_CTS              TRUE

// RTS und CTS Portpins bei Bedarf aus der Alternatefunction holen (z.B. Hardware INT)
#define SER5_INIT             {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SER10_RTS_ACTIVATE     {}
#define SER10_RTS_DEACTIVATE   {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SER10_CTS              TRUE

// RTS und CTS Portpins bei Bedarf aus der Alternatefunction holen (z.B. Hardware INT)
#define SER10_INIT             {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SER11_RTS_ACTIVATE     {}
#define SER11_RTS_DEACTIVATE   {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SER11_CTS              TRUE

// RTS und CTS Portpins bei Bedarf aus der Alternatefunction holen (z.B. Hardware INT)
#define SER11_INIT             {}

//*--------------------------------------------------------------------------*
//* SPI                                                                      *
//*--------------------------------------------------------------------------*

// allgemein

// Wartezeit, bis DSP angelaufen ist und den Interrupt erstmalig setzt
#define SPIMSG_SLAVE_DSP_TIMEOUT       70    // ms max. Wartezeit fuer eine komplette Message
#define SPIMSG_SLAVE_CS_WAIT_TIMEOUT   5000  // noch ueberpruefen; Laufzeittimer 500 us

// Master Mode
// transfer data width is 16 bit
// transfer/receive MSB first
// mit Enable; kein Chipselect
#define SPIMSG_MASTER_DSP_SPIMODE      0x61  //0x41
#define SPIMSG_SLAVE_DSP_SPIMODE       0x15

// Master Mode
// transfer data width is 8 bit
// transfer/receive MSB first
#define SPIBOOT_DSP_SPIMODE            0x22

// Kriterium fuer Target spannungslos
// hier :  DSP_RESET = 0
//#define SPIMSG_TARGET_POWERFAIL        !_P612
#define SPIMSG_TARGET_POWERFAIL        FALSE


// speziell fuer MBC-Treiber von Ingo Heidemann
#define SPIMSG_REQUEST                 _P50
#define SPIMSG_CNT                     33
#define SPIMSG_CHANNEL                 2
#define SPIMSG_CONFIG                  {_PM50=1; _PCS2=1; _PMCS2=0;} // Int Eingang, CS inaktiv

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
// direkt im SPI Treiber angegeben, da mehrere CS Leitungen pro SPI HW Channel verwendet werden
#define SPI0_CS_ACTIVATE         {;}  // angepasst
#define SPI0_CS_DEACTIVATE       {;}


// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI0_CS_READ             TRUE

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI0_CS_Read       NULL      // Vektor
#define SPI0_CS_INT_CONFIGURE    {}
#define SPI0_CS_INT_DISABLE      {}
#define SPI0_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI0_REQ_ACTIVATE        {}
#define SPI0_REQ_DEACTIVATE      {}
#define SPI0_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI0_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI0_MASTER              1
#define INT_SPI0_MSG_Read        NULL      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
#define SPI0_MSG_INT_CONFIGURE   {}
#define SPI0_MSG_INT_CATCH       {}
#define SPI0_MSG_INT_CLOSE       {}
#define SPI0_MSG_REQ_CONFIGURE   {}

#define SPI0_MSG_DSPRESET_ON     {}
#define SPI0_MSG_DSPRESET_OFF    {}
#define SPI0_MSG_INT_DISABLE     {}
#define SPI0_MSG_INT_ENABLE      {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
// direkt im SPI Treiber angegeben, da mehrere CS Leitungen pro SPI HW Channel verwendet werden
#define SPI1_CS_ACTIVATE      {;}  // angepasst
#define SPI1_CS_DEACTIVATE    {;}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI1_CS_READ             TRUE

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI1_CS_Read       NULL      // Vektor
#define SPI1_CS_INT_CONFIGURE    {}
#define SPI1_CS_INT_DISABLE      {}
#define SPI1_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI1_REQ_ACTIVATE        {}
#define SPI1_REQ_DEACTIVATE      {}
#define SPI1_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI1_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI1_MASTER              1
#define INT_SPI1_MSG_Read        NULL      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
#define SPI1_MSG_INT_CONFIGURE   {}
#define SPI1_MSG_INT_CATCH       {}
#define SPI1_MSG_INT_CLOSE       {}
#define SPI1_MSG_REQ_CONFIGURE   {}

#define SPI1_MSG_DSPRESET_ON     {}
#define SPI1_MSG_DSPRESET_OFF    {}
#define SPI1_MSG_INT_DISABLE     {}
#define SPI1_MSG_INT_ENABLE      {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SPI2_CS_ACTIVATE         {_PCS2 = 0; _PMCS2 = 0;}
#define SPI2_CS_DEACTIVATE       {_PCS2 = 1; _PMCS2 = 0;}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI2_CS_READ             TRUE

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI2_CS_Read       NULL      // Vektor
#define SPI2_CS_INT_CONFIGURE    {}
#define SPI2_CS_INT_DISABLE      {}
#define SPI2_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI2_REQ_ACTIVATE        {}
#define SPI2_REQ_DEACTIVATE      {}
#define SPI2_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI2_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI2_MASTER              1
#define INT_SPI2_MSG_Read        NULL      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
#define SPI2_MSG_INT_CONFIGURE   {}
#define SPI2_MSG_INT_CATCH       {}
#define SPI2_MSG_INT_CLOSE       {}
#define SPI2_MSG_REQ_CONFIGURE   {}

#define SPI2_MSG_DSPRESET_ON     {}
#define SPI2_MSG_DSPRESET_OFF    {}
#define SPI2_MSG_INT_DISABLE     {}
#define SPI2_MSG_INT_ENABLE      {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SPI3_CS_ACTIVATE         {}
#define SPI3_CS_DEACTIVATE       {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI3_CS_READ             TRUE

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI3_CS_Read       NULL      // Vektor
#define SPI3_CS_INT_CONFIGURE    {}
#define SPI3_CS_INT_DISABLE      {}
#define SPI3_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI3_REQ_ACTIVATE        {}
#define SPI3_REQ_DEACTIVATE      {}
#define SPI3_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI3_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI3_MASTER              1
#define INT_SPI3_MSG_Read        NULL      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
#define SPI3_MSG_INT_CONFIGURE   {}
#define SPI3_MSG_INT_CATCH       {}
#define SPI3_MSG_INT_CLOSE       {}
#define SPI3_MSG_REQ_CONFIGURE   {}

#define SPI3_MSG_DSPRESET_ON     {}
#define SPI3_MSG_DSPRESET_OFF    {}
#define SPI3_MSG_INT_DISABLE     {}
#define SPI3_MSG_INT_ENABLE      {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SPI4_CS_ACTIVATE         {}
#define SPI4_CS_DEACTIVATE       {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI4_CS_READ             TRUE

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI4_CS_Read       NULL      // Vektor
#define SPI4_CS_INT_CONFIGURE    {}
#define SPI4_CS_INT_DISABLE      {}
#define SPI4_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI4_REQ_ACTIVATE        {}
#define SPI4_REQ_DEACTIVATE      {}
#define SPI4_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI4_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI4_MASTER              1
#define INT_SPI4_MSG_Read        NULL      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
#define SPI4_MSG_INT_CONFIGURE   {}
#define SPI4_MSG_INT_CATCH       {}
#define SPI4_MSG_INT_CLOSE       {}
#define SPI4_MSG_REQ_CONFIGURE   {}

#define SPI4_MSG_DSPRESET_ON     {}
#define SPI4_MSG_DSPRESET_OFF    {}
#define SPI4_MSG_INT_DISABLE     {}
#define SPI4_MSG_INT_ENABLE      {}

//*--------------------------------------------------------------------------*

// hier Leerdefinition einsetzen, wenn nicht unterstuetzt
#define SPI5_CS_ACTIVATE         {}
#define SPI5_CS_DEACTIVATE       {}

// hier TRUE setzen, wenn nicht unterstuetzt; ansonsten PIN-Name angeben
#define SPI5_CS_READ             P_SPI_AT_CS

// Interruptgesteuerte CS-Auswertung fuer Slave-Betrieb
//#define INT_SPI5_CS_Read       NULL      // Vektor
#define SPI5_CS_INT_CONFIGURE    {}
#define SPI5_CS_INT_DISABLE      {}
#define SPI5_CS_INT_ENABLE       {}

// Request Steuerung
#define SPI5_REQ_ACTIVATE        {P_SPI_UM_REQ=0;DP_SPI_UM_REQ=0;}
#define SPI5_REQ_DEACTIVATE      {DP_SPI_UM_REQ=1;}
#define SPI5_INIT                {}

// hier FALSE  setzen, wenn nicht unterstuetzt; TRUE heisst warten
#define SPI5_EN_WAIT             FALSE

// Konfiguration fuer Gatewaymode zum DSP
#define SPI5_MASTER              0
#define INT_SPI5_MSG_Read        INTTP7CC0      // Vektor

// hier Leerdefinitionen einsetzen, wenn nicht unterstuetzt
// die SPI 0 ist beim UMOST II parallelgeschaltet, hier unbedingt deaktivieren und Pins auf Input setzen
// aktiven Pullup aktivieren an SPI_AT_CS
#define SPI5_MSG_INT_CONFIGURE   {SerClose(0);_PM40=_PM41=_PM42=1;_PF40=_PF41=_PF42=0;CONFIG_INTTP7CC0_P69_FALLINGEDGE;P_SPI_PULLUP1=1;DP_SPI_PULLUP1=0;}
#define SPI5_MSG_INT_CATCH       {}
// Chipselect und SPI5-Pins wieder in Eingang verwandeln; Opendrain und alternate Functions immer abklemmen
#define SPI5_MSG_INT_CLOSE       {SerClose(5);_PM66=_PM67=_PM68=_PM69=1;_PF66=_PF67=_PF68=_PF69=0;_PMC69=0;}

// aktiven Pullup aktivieren an UM_REQ
// DSP Request in Ausgangsstellung auf Eingang; Opendrain und alternate Functions immer abklemmen
#define SPI5_MSG_REQ_CONFIGURE   {P_SPI_PULLUP3=1;DP_SPI_PULLUP3=0;_PM610=1;_PMC610=0;_PF610=0;}

#define SPI5_MSG_DSPRESET_ON     {}
#define SPI5_MSG_DSPRESET_OFF    {}
#define SPI5_MSG_INT_DISABLE     {_TP7CCMK0 = 1;}
#define SPI5_MSG_INT_ENABLE      {_TP7CCMK0 = 0;}

//*--------------------------------------------------------------------------*
//* NEC-SelfFlash-Lib                                                        *
//*--------------------------------------------------------------------------*

// freier Speicher der benoetigt wird um SelfLib auslagern zu koennen
#pragma ghs startdata
extern const UINT32 ghs_ramstart[];
#pragma ghs enddata
#define START_INTERNAL_RAM      ((UINT32)ghs_ramstart)  // Anfang internes NEC RAM

// Portpins anpassen je nach HW
#define   PIN_DATA_CONNECTED_TO_FLMD0  _PCS3    // Pin steuert Pegel von FLMD0
#define   PIN_MODE_CONNECTED_TO_FLMD0  _PMCS3   // Mode-Bit

#define  INT_OFF     __DI()
#define  INT_ON      __EI()


//*--------------------------------------------------------------------------*
//* ADC                                                                      *
//*--------------------------------------------------------------------------*

#define ADC_AV_CONV_NUMBER	   4		   // numbers of adc conversions used
												   // for averedging

//*--------------------------------------------------------------------------*
//* RDS Kanal 1,2 und 3                                                      *
//* in INTS.H Interrupts konfigurieren !                                     *
//*--------------------------------------------------------------------------*

#define RDS_BUF_SIZE          250         // Buffer size here enough space to store data
                           				   // of 247 * 8Bit / (1100Bit/s) = 1.8s
#define RDS_MAX_DATA          247  		   // max number of bytes returned in one answer

#define P_DATARDS1            _P32              // RDS1 Datenleitung; Clock ist P01 TIP60
#define INT_RDS1_Read         INTTP6CC0         // Vektor
#define RDS1_INT_MASK         _TP6CCMK0
// Set timer to freerunning capture mode falling edge TIP60
#define RDS1_INT_CONFIGURE    CONFIG_INTTP6CC0_P01_FALLINGEDGE


#define P_DATARDS2            _P42              // RDS2 Datenleitung; Clock ist P00 TIP61
#define INT_RDS2_Read         INTTP6CC1         // Vektor
#define RDS2_INT_MASK         _TP6CCMK1
// Set timer to freerunning capture mode falling edge TIP61
#define RDS2_INT_CONFIGURE    CONFIG_INTTP6CC1_P00_FALLINGEDGE


#define P_DATARDS3            _P613             // RDS3 Datenleitung; Clock ist P612 TIP80
#define INT_RDS3_Read         INTTP8CC0         // Vektor
#define RDS3_INT_MASK         _TP8CCMK0
// Set timer to freerunning capture mode falling edge TIP80
#define RDS3_INT_CONFIGURE    CONFIG_INTTP8CC0_P612_FALLINGEDGE

//*--------------------------------------------------------------------------*
//* ECL Support                                                              *
//*--------------------------------------------------------------------------*

#if CP_ECL
   #define USERFUNCTION_TIMER {ECL_Driver();}

   #define P_ECL_RX            INPORT_BOOL(0,8)
   #define P_ECL_TX            INPORT_BOOL(0,9)
   #define P_ECL_TX_LOW        SETPORT_0(0,9)
   #define P_ECL_TX_HIGH       SETPORT_1(0,9)

   // IC800 muss fuer ECL auf ATA6664 (SAP-Nr 2182793) umbestueckt werden, damit kein TXD-Timeout auftritt
   // mit Wakeuppuls 100 us
   #define ECL_TRANSCEIVER_INITIATOR_ENABLE     {}
   #define ECL_TRANSCEIVER_PARTICIPANT_ENABLE   {}
   #define ECL_TRANSCEIVER_DISABLE              {}
#endif

//*--------------------------------------------------------------------------*
//* Interrupteinstellungen fuer FPGA                                         *
//* in INTS.H Interrupts konfigurieren !                                     *
//*--------------------------------------------------------------------------*

#if (CP_FPGA_INFO)
   #define FPGA_BASE_ADR            0x00100000     // Basisaddresse fuer FPGA-Zugriff
   #define FPGA_VERSION_ADR         0x00000008     // Offset fuer FPGA-VersionsInfo

   #define FPGA_GPIF_REGSET_BASE    0x01
   #define FPGA_GPIF_VERSION_OFFS   (FPGA_GPIF_REGSET_BASE + 0x00)
   #define FPGA_GPIF_INTSTAT_OFFS   (FPGA_GPIF_REGSET_BASE + 0x01)
   #define FPGA_GPIF_INTMASK_OFFS   (FPGA_GPIF_REGSET_BASE + 0x02)

   #define FPGA_GPIF_INTMASK     *((volatile UINT8 *) (FPGA_BASE_ADR + FPGA_GPIF_INTMASK_OFFS))

   #define FPGA_INT_MASK         _TQ0CCMK1         // zugehöriges Interrupt Maskbit
   #define INT_FPGA              INTTQ0CC1         // Vektor

   // Set timer to freerunning capture mode falling edge TIQ01 , FPGA Intmask auch setzen
   #define FPGA_INT_CONFIGURE    {CONFIG_INTTQ0CC1_P50_FALLINGEDGE;FPGA_GPIF_INTMASK=0;}

   #define SH4_DONE_ISACTIVE     _PDH6          // Abfrage ob FPGA-Download von SH fertig
#endif

//*--------------------------------------------------------------------------*
//* MOST Controll Message Schnittstelle über OS8104/OS81050/OS81110          *
//*--------------------------------------------------------------------------*

// nachfolgend jeweils aktive Definitionen ausfüllen !
#define MOST_NODEADRESS          0x0101
#define MOST_TARGETNODEADRESS    0x0100

#define MOSTMODE_CLOCK           MOSTCLOCK_SLAVE
#define MOSTMODE_FORMAT          MOSTFORMAT_MOST150

//*--------------------------------------------------------------------------*
//* Anbindung OS8104                                                         *
//*--------------------------------------------------------------------------*

#if (CP_OS8104_IIC || CP_OS8104_PAR)
   #define OS8104_PACKETSIZE           48

   // Clocksettings
   #define OS8104_bCM1_SETTING_MASTER  0x12        // 256 Fs; PLLinput Quartz
   #define OS8104_bCM1_SETTING_SLAVE   0x10        // 256 Fs; PLLinput RX
   #define OS8104_bCM1_SETTING_SPDIF   0x11        // 256 Fs; PLLinput SPDIF

   #define OS8104_bXCR_SETTING_MASTER  0xE2        // Transmitter on, no Bypass
   #define OS8104_bXCR_SETTING_SLAVE   0x62        // Transmitter on, no Bypass
   #define OS8104_bXCR_SETTING_SPDIF   0xE2        // Transmitter on, no Bypass

   #define OS8104_bXCR_SETTING_SLAVE2  0x40        // Bypass during unlock; active Transmitter during lock
                                                   // no nodepostion increment (ABY active)
#endif

#if CP_OS8104_IIC
   // Steuermakros
   #define OS8104_IIC_RESET_ON         {_P02 = 0; _PM02 = 0;}
   #define OS8104_IIC_RESET_OFF        {_P02 = 1; _PM02 = 1;}

    // Standard Interrupt
   #define OS8104_IIC_INT_IN           _P00
   #define OS8104_IIC_INT_FUNCTION     NULL       // Vektor
   #define OS8104_IIC_INT_CONFIGURE    {;}

   #define OS8104_IIC_INT_DISABLE      {;}
   #define OS8104_IIC_INT_ENABLE       {;}

   // Asynchron Interrupt
   #define OS8104_IIC_AINT_IN          _P01
   #define OS8104_IIC_AINT_FUNCTION    NULL    // Vektor
   #define OS8104_IIC_AINT_CONFIGURE   {;}
   #define OS8104_IIC_AINT_DISABLE     {;}
   #define OS8104_IIC_AINT_ENABLE      {;}

	#define OS8104_IIC_ADRESSE    0x40   		// hängt von den Pins  AD0 und AD1 ab
	#define OS8104_IIC_MAXWAIT    0x8000    	// auf Clockstretching

	// Ummappen des vorher definierten IIC-Softwaretreibers
	// Dieser muss passend konfiguriert und eingeschaltet sein !
	#define OS8104_Write_Str      IIC_Write_1_Str
	#define OS8104_Read_Str       IIC_Read_1_Str
	#define OS8104_Init_HW        IIC_Init_1
#endif

#if CP_OS8104_PAR
   // Steuermakros
   #define OS8104_PAR_RESET_ON         {_PCT7 = 0; _PMCT7 = 0;}
   #define OS8104_PAR_RESET_OFF        {_PCT7 = 1; _PMCT7 = 1;}

    // Standard Interrupt
   #define OS8104_PAR_INT_IN           _P33
   #define OS8104_PAR_INT_FUNCTION     INTTP0CC1       // Vektor
   #define OS8104_PAR_INT_CONFIGURE    CONFIG_INTTP0CC1_P33_FALLINGEDGE
   #define OS8104_PAR_INT_DISABLE      {_TP0CCMK1 = 1;}
   #define OS8104_PAR_INT_ENABLE       {_TP0CCMK1 = 0;}

   // Asynchron Interrupt
   #define OS8104_PAR_AINT_IN          _P06
   #define OS8104_PAR_AINT_FUNCTION    INTP3    // Vektor
   #define OS8104_PAR_AINT_CONFIGURE   CONFIG_INTP3_P06_FALLINGEDGE
   #define OS8104_PAR_AINT_DISABLE     {_PMK3 = 1;}
   #define OS8104_PAR_AINT_ENABLE      {_PMK3 = 0;}

	#define  OS8104_MAXWAIT 	   10500       // ca. 5 ms Timeout auf CFL-Signal
	#define  OS8104_RESETTIME     10

   // Parallelmode
   #define  P_OS8104_RD          _PCT6   	   // /RD
   #define  P_OS8104_WR          _PCT5   	   // /WR
   #define  P_OS8104_PAD0        _P73   	   // PAD0
   #define  P_OS8104_PAD1        _P74   	   // PAD1
   #define  P_OS8104_CFL         _P72   	   // CP_FLOW
   #define  P_OS8104_ERR         _P71   	   // Error

   #define  OS8104_DATAIN 		   {PM7H = 0xFF;}
   #define  OS8104_DATAOUT       {PM7H = 0x00;}
   #define  OS8104_DATAPORT      P7H

   #define  OS8104_PAD_00        {P_OS8104_PAD0 = 0;}
   #define  OS8104_PAD_01    	   {P_OS8104_PAD0 = 1;}
#endif

//*--------------------------------------------------------------------------*
//* IIC-Anbindung OS81xxx                                                    *
//* alternativ kann ein 2. gleicher Baustein als EXTERN definiert werden     *
//* und mit Steckverbinder angeschlossen werden (Programmeranwendung)        *
//*--------------------------------------------------------------------------*
// Konfiguration uebernommen vom M034
#if CP_INIC
	#define INIC_I2C_ADDR               0x40
	#define INIC_SPI_CHANNEL            0                 // SPI channel for Inic, CISG 0 when M033

	// SPI protocol without CS control (will be done separately in most msg.c)
	#define INIC_SPI_PROTOCOL           0x01              // SPI-Protokoll

	#define INIC_SPI_PACKETSIZE         1014              // for OS81110 and SPI
   #define INIC_SPI_CONFIF_MBC         1
   #define INIC_SPI_PACKET_BAUDRATE    1000000
   #define INIC_SPI_CS_ACTIVATE        SETPORT_0(3,4);   // IHE anpassen {_PDH2 = 0; _PMDH2 = 0;}  Modify it to CL40
   #define INIC_SPI_CS_DEACTIVATE      SETPORT_1(3,4);   // IHE anpassen {_PDH2 = 1; _PMDH2 = 0;}  Modify it to CL40
   #define INIC_FOTON_ACTIVATE         SETPORT_1(11,0);
   #define INIC_FOTON_DEACTIVATE       SETPORT_0(11,0);

	// Remapping of the previously defined IIC software driver
   // This has to be suitably configured and turned on !
   #define OS81xxx_WRITE_STR           IIC_Write_0_Str
   #define OS81xxx_EXTERN_WRITE_STR    IIC_Write_0_Str

   #define OS81xxx_READ_STR            IIC_Read_0_Str
   #define OS81xxx_EXTERN_READ_STR     IIC_Read_0_Str

   #define MOST_PLL_CRYSTAL            1                 // 0 = PLL source is MOST
                                                         // 1 = PLL source is crystal
   #define INIC_ACK_TIMEOUT            4000              // Timeout waiting for Ack Msg from Inic
   #define INIC_RX_DEADTIME            180               // Wartezeit nach I2C-RX in [us]

   #define RMCK_FS_DEFAULT             RMCK_256FS        // defaultstetting for use in InicConfig

   #define INIC_CONFIGURE_PORTS        {;}               // IHE eventuell anpassen !!!!!!!!!!!!!!!!!
   #define INIC_EXTERN_CONFIGURE_PORTS {;}               //

   // Assign appropriate timer when Interrupt used , then the timer must be reserved PMn is mode,0=output
   #define TIMER_INIC                  TIMER_WAIT_SIGNAL

   #define INIC_RES_ON                 {SETPORT_0(1, 9); SETPORT_OUT(1, 9);}  //{_Pn = 0; _PMn = 0;}
   #define INIC_EXTERN_RES_ON          {;}

   #define INIC_RES_OFF                {SETPORT_1(1, 9); SETPORT_OUT(1, 9);}  //{_Pn = 1; _PMn = 0;}
   #define INIC_EXTERN_RES_OFF         {;}

   #define INIC_ERR_BOOT_HIGH          {SETPORT_1(3, 2); SETPORT_OUT(3, 2);}  // M034: {_P33 = 1; _PM33 = 0;}
   #define INIC_EXTERN_ERR_BOOT_HIGH   {;}

   #define INIC_ERR_BOOT_LOW           {SETPORT_0(3, 2); SETPORT_OUT(3, 2);}  // M034: {_P33 = 0; _PM33 = 0;}
   #define INIC_EXTERN_ERR_BOOT_LOW    {;}

   #define INIC_ERR_BOOT_INPUT         {SETPORT_IN(3, 2);}                      // M034: {_PM33 = 1;}
   #define INIC_EXTERN_ERR_BOOT_INPUT  {;}

   // Configuration of the request interrupt from Gateway partners ; IIC slave
   #define OS81xxx_INT_PIN             INPORT_BOOL(1, 14)        // M034: _P92        // Interrupt Signal for I2C
   #define OS81xxx_EXTERN_INT_PIN      INPORT_BOOL(1, 14)        // M034: _P92        // Interrupt Signal for II2C



   #define INIC_SPIINT_PIN             INPORT_BOOL(1, 15)              // M034: _P915  // Int-Eingang SPI

   #define MOST1_ERR_ISACTIVE          INPORT_BOOL(3, 2)             // M034: _P36   // undef bei UMOST150, da in der portlist.h definiert
   #define LED_BUSLOCK_ACTIVATE        {;}                           // undef bei UMOST150, da in der portlist.h definiert
   #define LED_BUSLOCK_DEACTIVATE      {;}                           // undef bei UMOST150, da in der portlist.h definiert
#endif





//*--------------------------------------------------------------------------*
//* // allgemeingültige Definitionen                                         *
//*--------------------------------------------------------------------------*

#define MOSTMSG_MAX           64U         // max number of Control Messages in buffer

//*--------------------------------------------------------------------------*
//* Message ueber  CAN                                                       *
//*--------------------------------------------------------------------------*

#define GATEWAY_CAN_RXID      0x7FF
#define GATEWAY_CAN_TXID      0x7FE
#define GATEWAY_CAN_BAUDRATE  500000


// general definitions
#define CAN_RX_OBJECT         62          // Hardware CAN-Object Empfang
#define CAN_TX_OBJECT         63          // Hardware CAN-Object Senden

#define KWP2000_RX_OBJECT     60          // Hardware CAN-Object Empfang
#define KWP2000_TX_OBJECT     61          // Hardware CAN-Object Senden


// Portpins sind in portlist.h definiert !
// CAN-Transceiver TJA1055 oder TJA1041 einschalten
#define CAN0_TRANSCEIVER_ENABLE   {SETPORT_1(12,4);SETPORT_OUT(12,4);SETPORT_1(12,3);SETPORT_OUT(12,3);}
#define CAN3_TRANSCEIVER_ENABLE   {SETPORT_1(12,0);SETPORT_OUT(12,0);SETPORT_1(27,4);SETPORT_OUT(27,4);}

// CAN-Transceiver sicherheitshalber aufwecken , sonst Verlust der ersten 3 Flanken an RX
#define CAN0_TRANSCEIVER_WAKEUP   {SETPORT_1(21,8);SETPORT_OUT(21,8);DelayLoop (us(30));SETPORT_0(21,8);}
#define CAN3_TRANSCEIVER_WAKEUP   {SETPORT_1(27,3);SETPORT_OUT(27,3);DelayLoop (us(30));SETPORT_0(27,3);}

// Aktiv, wenn Can Transceiver ein
// falls in der Hardware nicht unterstuetzt, dann TRUE setzen
// Beispiel : 6727 : TJA1041A Pin 7 (INH) als Indikator
//#define CAN_TRANSCEIVER_ACTIVE   (!_PDH4)
#define CAN0_TRANSCEIVER_ACTIVE   (!(INPORT_BOOL(12,7)))
#define CAN1_TRANSCEIVER_ACTIVE   TRUE
#define CAN2_TRANSCEIVER_ACTIVE   TRUE
#define CAN3_TRANSCEIVER_ACTIVE   (!(INPORT_BOOL(12,2)))

//*--------------------------------------------------------------------------*
//* FPGA configuration                                                       *
//*--------------------------------------------------------------------------*
// definition der FPGA info-database
// hier koennt ein array von info-datensaetzen stehen

#define FPGA_NO  1                           // Number of FPGAs
#define def_FPGA_INFO { { FPGA_BASE_ADR, FPGA_VERSION_ADR  } }
#if FPGA_NO > 1
      #error "some items of the fpga-info-database are undefined"
#endif



#endif // _CONFIG_H

