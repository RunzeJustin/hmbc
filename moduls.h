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


#ifndef _MODULS_H
#define _MODULS_H

//*--------------------------------------------------------------------------*
//* Compiler-Schalter für die einzelnen Module Flash Bios und Rambios        *
//*--------------------------------------------------------------------------*

#define CP_SER3               0     // S-Befehl, DF3558 : 1 : Standard / 2..n : Alternativpinout
#define CP_SER5               0     // S-Befehl, DF3558 : 1 : Standard / 2..n : Alternativpinout
#define CP_SER10              0     // S-Befehl, DF3558 : 1 : Standard / 2..n : Alternativpinout
#define CP_SER11              1     // S-Befehl, DF3558 : 1 : Standard / 2..n : Alternativpinout


// beim 3554 sind die CSIG Instanzen 0+4+7 vorhanden
#define CP_CSIG0              1     // konfiguriert als SPI --> MCU_INIC_E2P_SPI
#define CP_CSIG4              1     // konfiguriert als SPI --> wird nicht verwendet
#define CP_CSIG7              0     // konfiguriert als SPI --> MCU_DSP_FPGA_SPI


#define CP_CSIH0              0     // konfiguriert als SPI
#define CP_CSIH1              0     // konfiguriert als SPI
#define CP_CSIH2              0     // konfiguriert als SPI

#define CP_CSIE0              0     // S-Befehl für serielle synchrone Schnittstelle 0
#define CP_CSIE1              0     // S-Befehl für serielle synchrone Schnittstelle 1

#define CP_SPI_TESTPATTERN    0     // SPI-Pruefmustergenerator

#define CP_IIC0               1     // I2C-Bus Nr.0    P25.10 + P25.11
#define CP_IIC1               1     // I2C-Bus Nr.0    P25.12 + P25.13

#define CP_BUFFER             1     // Buffer-Unterstuetzung
#define CP_ADC                1     // ADC Modul
#define CP_VIRTUELL_ADC       1     // virtual ADC Support
#define CP_PORT_RW            1     // Port lesen/setzen/löschen Modul
#define CP_SPARMODUS          0     // Stromsparmodus Modul
#define CP_PWM                1     // PWM Modul, DF3558 : 1 : Standard / 2..n : Alternativpinout
#define CP_CAN0               0     // CAN0-Modul
#define CP_CAN1               0     // CAN1-Modul
#define CP_CAN2               0     // CAN2-Modul
#define CP_CAN3               0     // CAN3-Modul
#define SW_SCRIPT             1     // CP_SCRIPT and CP_AUTORUN are not allowed anymore
#define CP_RAM                1     // Lese- und Schreibzugriffe auf das RAM
#define CP_RDS1               0 	   // RDS-Daten Kanal 1
#define CP_RDS2               0 		// RDS-Daten Kanal 2
#define CP_RDS3               0     // RDS-Daten Kanal 3
#define CP_ECL                0     // Electrical Controline (MOST)
#define CP_FPGA               0     // FPGA-Lader
#define CP_FPGA_INFO          0     // FPGA-Info can be read from NECV850
#define CP_DSPBOOT            0     // Booten des Texas Instruments DSP ueber SPI
#define CP_OS81050            0     // Inic 25 Mhz Most
#define CP_OS81110            1     // Inic 150 Mhz Most
#define CP_OS8104_IIC         0     // OS8104 Unterstuetzung einschalten
#define CP_OS8104_PAR         0     // OS8104 Unterstuetzung einschalten
#define CP_SELFFLASH          0     // SelfFlash-Modul
#define CP_ECL                0     // ECL (Electrical Control Line Most)
#endif // _MODULS_H