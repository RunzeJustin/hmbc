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


#ifndef _V850_COMMAND_H
#define _V850_COMMAND_H

//****************************************************************************
//*  Befehle fuer Mikrocontroller                                            *
//****************************************************************************

#define ADC_LESEN   	            CMD_ADC_READ
#define FPGA_PROGRAMMIEREN       CMD_FPGA_PROG
#define IIC_LESEN                CMD_IIC_READ
#define IIC_SCHREIBEN            CMD_IIC_WRITE
#define MOST_LESEN               CMD_MOST_READ
#define MOST_SCHREIBEN           CMD_MOST_WRITE
#define PORT_LESEN               CMD_PORT_READ
#define PORT_SCHREIBEN           CMD_PORT_WRITE
#define CONTROLLERTYP            CMD_CONTROLLERTYP
#define RAM_OPERATION            CMD_RAM_OPERATION
#define IMPULS_ERZEUGUNG         CMD_IMPULS_CREATION
#define XTD_COMMAND              CMD_XTD_COMMAND
#define TEXTAUSGABE              CMD_ASCII_DATA
#define DIAGNOSE                 CMD_DIAGNOSTIC
#define RDS_LESEN                CMD_RDS_READ
#define SER_OPERATION            CMD_SER_OPERATION
#define XTD_READ                 CMD_XTD_READ
#define XTD_WRITE                CMD_XTD_WRITE
#define XTD_INIT                 CMD_HW_INIT
#define SCRIPT_CONTROLL          CMD_SCRIPT_EXECUTE

#define GATEWAY_BEFEHL           CMD_GATEWAY
#define DEBUG_BEFEHL             CMD_DEBUG

//****************************************************************************
//*  Befehle fuer Sub-Interpreter bei X-Befehl                               *
//****************************************************************************

#define SUBCMD_WARTEN            CMD_X_WAIT
#define SUBCMD_CD_BEFEHL	      CMD_X_CDS
#define SUBCMD_STROMSPARMODUS    CMD_X_PWDN  // Mikrocontroller Stromsparmodi
#define SUBCMD_DSPBOOT           CMD_X_DA6xx_DSP
#define SUBCMD_PWM_ERZEUGUNG     0x20        // Erzeugung von PWM-Signalen
#define SUBCMD_DISPL_SCHREIBEN   0x21        // Displaytreiber beschreiben
#define SUBCMD_JUMPCALLADDR      0x22        // Sub-Kommando Call Absolute Addr
#define SUBCMD_INIC              CMD_X_INIC

//****************************************************************************
//*  Befehle fuer Sub-Interpreter bei Z-Befehl                               *
//****************************************************************************

#define SUBCMD_CAN_SCHREIBEN     CMD_Z_CAN
#define SUBCMD_IPC_WRITE         CMD_Z_IPC
#define SUBCMD_UHFREC_WRITE      CMD_Z_UHF
#define SUBCMD_HPIPC_WRITE       CMD_Z_HPIPC
#define SUBCMD_MOSTMSG_WRITE     CMD_Z_MOSTMSG

//****************************************************************************
//*  Befehle fuer Sub-Interpreter bei z-Befehl                               *
//****************************************************************************

#define SUBCMD_SSC_LESEN         CMD_Z_SAHARA_SSC
#define SUBCMD_CAN_LESEN         CMD_Z_CAN
#define SUBCMD_AUTORUNINFO_READ  CMD_Z_AUTO
#define SUBCMD_IPC_READ          CMD_Z_IPC
#define SUBCMD_UHFREC_READ       CMD_Z_UHF
#define SUBCMD_HPIPC_READ        CMD_Z_HPIPC
#define SUBCMD_MOSTMSG_READ      CMD_Z_MOSTMSG

//****************************************************************************
//*  Befehle fuer Sub-Interpreter bei H-Befehl                               *
//****************************************************************************

/*
 * XXX: im "hbbios_command.h" noch nicht definiert
 */

#define HWINIT_FPGA              0x00U
#define HWINIT_FPGA_VER          0x01U
#define HWINIT_MOST              0x02U
#define HWINIT_SAHA_SSC          0x03U
#define HWINIT_CDROMDEC          0x04U
#define HWINIT_MP3               0x05U
#define HWINIT_ATA               0x06U
#define HWINIT_SERF              0x07U
#define HWINIT_FLASH_SEG         0x08U
#define HWINIT_PCI               0x09U
#define HWINIT_IPC               0x0AU
#define HWINIT_RDS               0x0BU
#define HWINIT_DISPLAY           0x0CU
#define HWINIT_SALIERI           0x0DU
#define HWINIT_CDIRQ             0x0EU
#define HWINIT_SPI               0x0FU
#define HWINIT_I2C_CORAL         0x10U
#define HWINIT_PCCARD16          0x11U
#define HWINIT_UNILINK           0x12U
#define HWINIT_MMC               0x13U
#define HWINIT_FAN               0x14U
#define HWINIT_MOSTMSG           0x15U
#define HWINIT_LAN               0x16U
#define HWINIT_INIC              0x17U
#define HWINIT_I2C               0x18U

#define HWINIT_BLOAD_TST         0xFFU

//****************************************************************************
//*  Befehle fuer Sub-Interpreter bei D-Befehl                               *
//****************************************************************************

/*
 * XXX: im "hbbios_command.h" noch nicht definiert
 */
#define SUBCMD_ASCIIMESSAGE      0x4A     // ASCII-Message ein/ausschalten
#define SUBCMD_DEBUGMESSAGE      0x4D     // DEBUG-Message ein/ausschalten
#define SUBCMD_BOARD_VERSION     0x01     // get project and board version
#define SUBCMD_EMODULTESTER      0xE0     // special commands emodultester

//****************************************************************************
//* Module Error Numbers (Bit 31-24=Modul, 23-0=Sub-Errornumber)             *
//****************************************************************************

#define ERROR_UHFREC             ERROR_UHF

//****************************************************************************
//* Module Error Numbers (Bit 31-24=Modul, 23-0=Sub-Errornumber)             *
//****************************************************************************

#define ERROR_COMMAND            0x00010000U    // Fehler in der Kommando-Syntax



#endif // _V850_COMMAND_H
