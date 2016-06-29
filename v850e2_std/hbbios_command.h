/*
//-------------------------------------------------------------------------
//
//        _____   _____   _   _    ___
//       |  ___| |  ___| | | | |  |   `
//       | |_    | |_    | |_| |  | [] |
//       |  _|   |  _|   |  _  |  |  _ <
//       | |___  | |___  | | | |  | |_| |
//       |_____| |_____| |_| |_|  |____/
//
//
//   Project:          H/B BIOS
//   (c) copyright     Harman-Becker Automotive Systems
//-----------------------------------------------------------------------
//   $Header: //BiosGroup/trunk/BC_Projects/0000/0000-0000-00XY/cpu/src/std/hbbios_command.h#27 $
//   $Change: 1669388 $
//-----------------------------------------------------------------------
*/

#if !defined(_HBBIOS_COMMAND_H_)
#define _HBBIOS_COMMAND_H_

// Module Error Numbers (Bit 31-24=Modul, 23-0=Sub-Errornumber)
#define NO_ERROR           0x00000000U
#define ERROR_INTERPRT     0x01000000U
#define ERROR_MOST         0x02000000U
#define ERROR_CDDEC        0x03000000U
#define ERROR_FPGAPORT     0x04000000U
#define ERROR_ADC          0x05000000U
#define ERROR_DAC          0x06000000U
#define ERROR_I2C          0x07000000U
#define ERROR_MEM          0x08000000U
#define ERROR_CC           0x09000000U
#define ERROR_SER          0x0A000000U
#define ERROR_DIAG         0x0B000000U
#define ERROR_FPGA         0x0C000000U
#define ERROR_FLASH        0x0D000000U
#define ERROR_ATA          0x0E000000U
#define ERROR_TEMIC        0x0F000000U
#define ERROR_I2SYN        0x10000000U
#define ERROR_SAHA_SSC     0x11000000U
#define ERROR_FC           0x12000000U
#define ERROR_GRPH         0x13000000U
#define ERROR_RDS          0x14000000U
#define ERROR_HWINIT       0x15000000U
#define ERROR_CAN          0x16000000U
#define ERROR_TUNER        0x17000000U
#define ERROR_MOST_CMD     0x18000000U
#define ERROR_GATEWAY      0x19000000U
#define ERROR_IPC          0x1A000000U
#define ERROR_DMA          0x1B000000U
#define ERROR_REG          0x1C000000U
#define ERROR_ZIP          0x1D000000U
#define ERROR_SERF         0x1E000000U
#define ERROR_DVD_PLAY     0x1F000000U
#define ERROR_FUJI_FLASH   0x20000000U
#define ERROR_CDS          0x21000000U
#define ERROR_ICB2         0x22000000U
#define ERROR_AUTO         0x23000000U
#define ERROR_SPI          0x24000000U
#define ERROR_MDL          0x25000000U // Motorola DSP Loader
#define ERROR_MP3          0x26000000U
#define ERROR_PORT         0x27000000U
#define ERROR_PCI          0x28000000U
#define ERROR_UHF          0x29000000U // UHF Receiver
#define ERROR_PWM          0x2A000000U
#define ERROR_SALIERI      0x2A000000U
#define ERROR_OLED         0x2B000000U
#define ERROR_STARTPAT     0x2C000000U // StartPattern&Startup
#define ERROR_ADJCTRL      0x2D000000U // Error with Adjust Data Control
#define ERROR_ATTINY       0x2E000000U // Atmel Tiny programming
#define ERROR_MMC          0x2F000000U // Multi Media Card
#define ERROR_I2C_CORAL    0x30000000U // I2C @ Coral
#define ERROR_SPI_FPGA     0x31000000U // FPGA SPI
#define ERROR_DA6xx_DSP    0x32000000U // TI DSP DA6xx
#define ERROR_PCCARD16     0x33000000U // PC Card 16
#define ERROR_FPGATEST     0x34000000U // FPGA test error
#define ERROR_SAHAUPD      0x35000000U // Sahara I²C Update
#define ERROR_PWDOWN       0x36000000U // Power-Down Module
#define ERROR_EMIF         0x37000000U // EMIF boot module
#define ERROR_SDC          0x38000000U // SDCard Module
#define ERROR_SELFREFR     0x39000000U // Self-Refresh test function
#define ERROR_EMIF_IPC     0x3A000000U // EMIF ipc module
#define ERROR_NANDFLASH    0x3B000000U // NAND-Flash device
#define ERROR_LANADAPTER   0x3C000000U // Lan-Adapter-Module
#define ERROR_TEMPSENSOR   0x3D000000U // Thermal sensor chip
#define ERROR_INIC         0x3E000000U // MOST INIC device
#define ERROR_I2S          0x3F000000U // I2S interface
#define ERROR_CPLD         0x40000000U // CPLD for BlueCore
#define ERROR_LIN          0x40000000U
#define ERROR_DIAGFSE      0x41000000U // FSE Diagnosis
#define ERROR_SSC2         0x41000000U
#define ERROR_SSC3         0x42000000U
#define ERROR_IICMSG       0x43000000U
#define ERROR_SCRIPT       0x44000000U // script module
#define ERROR_FS           0x45000000U // file system error
#define ERROR_USB          0x46000000U // USB stack
#define ERROR_INT          0x47000000U // Interrupt module error
#define ERROR_SHRDMEM      0x48000000U // shared Memory Communication
#define ERROR_NVIDIA       0x49000000U // nVIDIA error codes
#define ERROR_LED          0x4A000000U // LED modul error codes
#define ERROR_TCPIP        0x4B000000U // TCP/IP networking (LWIP stack)
#define ERROR_FPGA_TRC     0x4C000000U // FPGA tracer
#define ERROR_MDOC         0x4D000000U // MDOC
#define ERROR_P1J          0x4E000000U // PLL1 Jitter error
#define ERROR_AC97         0x4F000000U // AC97 Codec Wolfson
#define ERROR_AUDIO        0x50000000U // audio routing error codes
#define ERROR_C6XXX        0x51000000U // error codes from C6XXX DIAG shell
#define ERROR_APIX         0x52000000U // error codes for APIX Display Interface
#define ERROR_ECL          0x53000000U // error codes forElectrical Control Line (MOST) commands


// Error Numbers !!! not for future use !!!!
#define ERROR_SIZE         0x00000001U
#define ERROR_TYPE         0x00000002U
#define ERROR_ERASE        0x00000004U
#define ERROR_WRITE        0x00000008U
#define ERROR_VERIFY       0x00000010U
#define ERROR_TIMEOUT      0x00000020U
#define ERROR_BUS_BUSY     0x00000040U
#define ERROR_NO_RESPONSE  0x00000080U
#define ERROR_NO_ACK       0x00000100U
#define ERROR_WRONG_VALUE  0x00000200U
#define ERROR_CRC          0x00000400U
#define ERROR_FLASH_BUSY   0x00000800U
#define ERROR_SDRAM        0x00001000U
#define ERROR_COMMAND      0x00010000U
#define ERROR_FORMAT       0x00020000U
#define ERROR_MALLOC       0x00080000U
#define ERROR_UNKNOWN      0xFFFFFFFFU

// Commands //
#define CMD_GATEWAY            '#'        // 0x40: Gateway Command
#define CMD_ADC_READ           'a'        // 0x61: ADC Werte lesen
#define CMD_DAC_WRITE          'A'        // 0x41: DAC Werte schreiben
#define CMD_CDDEC_READ         'c'        // 0x63: read CDROM decoder registers
#define CMD_CDDEC_WRITE        'C'        // 0x43: write CDROM decoder registers
#define CMD_ASCII_DATA         'd'        // 0x64: Die folgenden Bytes sind ASCII-Zeichen
#define CMD_DIAGNOSTIC         'D'        // 0x44: Diagnosemodus
#define CMD_SCRIPT_EXECUTE     'e'        // 0x65: execute script
#define CMD_EXT_AUDIO          'E'        // 0x45: Extended audio routing
#define CMD_FPGA_TEST          'f'        // 0x66: FPGA testen
#define CMD_FPGA_PROG          'F'        // 0x46: FPGA Programmieren
#define CMD_GRAPHICS           'G'        // 0x47: Graphic command
#define CMD_GO                 'g'        // runs a program from a specified address
#define CMD_HW_INIT            'H'        // runs HW initialization
#define CMD_IIC_READ           'i'        // 0x69: lesen vom IIC-Bus
#define CMD_IIC_WRITE          'I'        // 0x49: schreiben auf IIC-Bus
#define CMD_TAB2_READ          'j'        // lesen vom TAB2-Bus
#define CMD_TAB2_WRITE         'J'        // schreiben auf TAB2-Bus
#define CMD_CC_CMD             'K'        // sending of a control command to the CC driver
#define CMD_CD_BEFEHL          'L'        // Senden einer Steueranweisung zum CD-Treiber
#define CMD_FLASH_OP           'm'        // 0x6D: FLASH Operation (Löschen, Schreiben, Prüfen)
#define CMD_POWERDOWNMODES     'M'        // microcontroller power down modes
#define CMD_TCPIP              'n'        // TCP/IP networking (LWIP stack)
#define CMD_MOST_READ          'o'        // 0x6F: lesen vom MOST-Chip OS8104 (parallel)
#define CMD_MOST_WRITE         'O'        // 0x4F: schreiben auf MOST-Chip OS8104 (parallel)
#define CMD_PORT_READ          'p'        // 0x70: Port lesen (bitweise, byteweise, Control Register)
#define CMD_PORT_WRITE         'P'        // 0x50: Port schreiben  (bitweise, byteweise, Control Register)
#define CMD_RDS_READ           'q'        // 0x71: read RDS data
#define CMD_CPUREG_READ        'r'        // 0x72: CPU Register lesen
#define CMD_CPUREG_WRITE       'R'        // 0x52: CPU Registerschreiben
#define CMD_CONTROLLERTYP      's'        // 0x73: Statusabrage fuer Mikrocontroller
#define CMD_SER_OPERATION      'S'        // 0x53: carries out serial driver commands
#define CMD_RAM_OPERATION      't'        // 0x74: Schreibt oder liest das RAM
#define CMD_TEMIC_WRITE        'T'        // TEMIC-Bausteinschreiben
#define CMD_IMPULS_CREATION    'U'        // Erzeugung von Impulsen
#define CMD_DRIVE_WRITE        'V'        // write to drives (atapi, philips-m3, dvd, ...)
#define CMD_DRIVE_READ         'v'        // read from drives (atapi, philips-m3, dvd, ...)
#define CMD_PWM_CREATION       'W'        // Erzeugung von PWM-Signalen
#define CMD_WAIT               'w'        // Wartet eine bestimmte Zeit
#define CMD_ERROR              'x'        // Fehler
#define CMD_XTD_COMMAND        'X'        // extended command
#define CMD_DEBUG              'y'        // temporary debug commands
#define CMD_XTD_READ           'z'        // extended read command
#define CMD_XTD_WRITE          'Z'        // extended write command

// HW Init Commands ('H')

#define CMD_H_FPGA              0x00U
#define CMD_H_FPGA_VER          0x01U
#define CMD_H_MOST              0x02U
#define CMD_H_SAHA_SSC          0x03U
#define CMD_H_CDROMDEC          0x04U
#define CMD_H_MP3               0x05U
#define CMD_H_ATA               0x06U
#define CMD_H_SERF              0x07U
#define CMD_H_FLASH_SEG         0x08U
#define CMD_H_PCI               0x09U
#define CMD_H_IPC               0x0AU
#define CMD_H_RDS               0x0BU
#define CMD_H_DISPLAY           0x0CU
#define CMD_H_SALIERI           0x0DU
#define CMD_H_CDIRQ             0x0EU
#define CMD_H_SPI               0x0FU
#define CMD_H_PCCARD16          0x11U
#define CMD_H_UNILINK           0x12U
#define CMD_H_MMC               0x13U
#define CMD_H_FAN               0x14U
#define CMD_H_MOSTMSG           0x15U
#define CMD_H_LAN               0x16U
#define CMD_H_INIC              0x17U
#define CMD_H_I2C               0x18U
#define CMD_H_SDC               0x19U
#define CMD_H_SHRDMEM           0x1AU
#define CMD_H_GRAPHICS          0x1BU
#define CMD_H_BLOAD_TST         0xFFU

// Extended Commands ('X')
                                        // frei da nicht benutzt :-)
#define CMD_X_WAIT           0x01U      // wait function
#define CMD_X_DMA            0x02U      // DMA controller functions
#define CMD_X_REG            0x03U      // regulator functions
#define CMD_X_ZIP            0x04U      // ZIP commands
#define CMD_X_DVD_PLAY       0x05U      // commands for DVD player
#define CMD_X_FC_EXE         0x06U      // front controler execution control
#define CMD_X_WD             0x07U      // watchdog commands
#define CMD_X_TUN_SEARCH     0x08U      // tuner search function
#define CMD_X_CDS            0x09U      // single CD interrupt messages
#define CMD_X_AUTO1          0x0AU      // autorun command
#define CMD_X_SPI            0x0BU      // SPI commands
#define CMD_X_MDL            0x0CU      // Motorola DSP Flasher
#define CMD_X_MALLOC         0x0DU      // malloc/free functions
#define CMD_X_PCI            0x0EU      // PCI commands
#define CMD_X_START_CTRL     0x0FU      // handling of testpattern for startup control
#define CMD_X_ADJ_CTRL       0x10U      // handling of adj data buffers
#define CMD_X_ATTINY         0x11U      // Atmel Tiny programmin commands
#define CMD_X_SPI_FPGA       0x12U      // FPGA SPI commands
#define CMD_X_DA6xx_DSP      0x13U      // TI DA6xx DSP
#define CMD_X_PCCARD16       0x14U      // PC Card 16
                                        // frei da FPGA-Test -> D B0
#define CMD_X_SAHAUPDT       0x16U      // Sahara I²C Update
#define CMD_X_ETH_E2P        0x17U      // Ethernet EEPROM Commands
#define CMD_X_EMIF           0x18U      // EMIF commands
#define CMD_X_SDC            0x19U      // SDCard Module
#define CMD_X_SELFREFR       0x1AU      // Self-Refresh test commands
#define CMD_X_LANADAPTER     0x20U      // LAN-Adapter board
#define CMD_X_NANDFLASH      0x21U      // Nand flash device
#define CMD_X_TEMPSENSOR     0x22U      // Thermal sensor chip on FSE
#define CMD_X_INIC           0x23U      // INIC device
#define CMD_X_I2S            0x24U      // I2S interface of FSE
#define CMD_X_CPLD           0x25U      // BlueCore CPLD
#define CMD_X_DIAGNOSIS      0x26U      // FSE Diagnosis circuit
#define CMD_X_FS             0x27U      // file system commands
#define CMD_X_USB            0x28U      // USB stack
#define CMD_X_INT            0x29U      // Interrupt commands
#define CMD_X_SDIO           0x2aU      // SDIO commands
#define CMD_X_FPGA_TRC       0x30U      // FPGA tracer commands
#define CMD_X_MDOC           0x31U      // MDOC commands
#define CMD_X_P1J            0x32U      // PLL1 Jitter test
#define CMD_X_WLAN           0x33U      // WLAN commands
#define CMD_X_BT             0x34U      // BT commands
#define CMD_X_FILESYS        0x46U      // file system commands
#define CMD_X_APIX           0x47U      // APIX commands
#define CMD_X_KLEER          0x48U      // kleer commands
#define CMD_X_ECL            0x49U      // Electrical Control Line (MOST) commands

#define CMD_X_PWDN           0x8FU      // Powerdown (V850, migrated from "X 0F"))

#define CMD_X_AC97           0xACU      // AC97 commands/Wolfson
#define CMD_X_CMD_LIST       0xC0U      // prints the command list

// Debug Commands ('y')
#define CMD_Y_THREAD         0x00U      // cthread diag functions
#define CMD_Y_MEM            0x01U      // memory debug functions
#define CMD_Y_REBOOT         0xFFU      // reboot function

// Extended Read/Write Commands ('Z','z')
#define CMD_Z_SAHARA_SSC     0x00U      // read byte from SAHARA SSC input queue
#define CMD_Z_FC             0x01U      // read/write front controller
#define CMD_Z_I2SYNC         0x02U      // read/write from/to I2Sync channel
#define CMD_Z_CAN            0x03U      // read/write via CAN
#define CMD_Z_AUTO           0x04U      // read/write Autorun Information
#define CMD_Z_IPC            0x05U      // read/write IPC commands
#define CMD_Z_FUJI_FLASH     0x06U      // read/write FUJI flash commands
#define CMD_Z_ICB            0x07U      // read/write Inter-CPU-Bus
#define CMD_Z_UHF            0x08U      // read/write UHF Receiver
#define CMD_Z_HPIPC          0x09U      // read/write High Performance IPC
#define CMD_Z_ADSP21161      0x0AU      // read/write ADSP 21161 (SPI)
#define CMD_Z_SAL            0x0BU      // Salieri commands
#define CMD_Z_MOSTMSG        0x0CU      // read/write MOST message
#define CMD_Z_DSPIPC         0x0DU      // read/write DSPIPC message (generic)

#define CMD_Z_LIN            0x40U      // read/write LIN
#define CMD_Z_SSC2           0x41U      // read/write SSC2

#endif
