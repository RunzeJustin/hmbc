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


//-------------------------------------------------------------------------
//  Includes
//-------------------------------------------------------------------------

#include "config.h"
#include "define.h"
#include "hbbios.h"
#include "global.h"                 // globale Datenstrukturen
#include "tool.h"                 	// für hex2str usw.
#include "ints.h"						   // Interruptverwaltung
#include "timer.h"                  // für Wartezeit

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "buffer.h"
#include "port.h"
#include "gateway.h"
#include "inic.h"

#if CP_CMDDEV_MOST
   #include "mostmsg.h"
#endif

#include "iic.h"
#include "iic_0.h"
/*
#include "iic_1.h"
#include "iic_2.h"
#include "iic_3.h"
#include "iic_4.h"
#include "iic_5.h"
#include "iic_6.h"
#include "iic_7.h"
*/

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if CP_INIC

//-------------------------------------------------------------------------
//  Makros for textoutputs in parts with communication via Most
//  Background:
//  in case of communication via most it is not possible to send a message
//  via command channel if this channels has blocked because of error
//
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
//  Debug
//-------------------------------------------------------------------------

//#define     INIC_XCM_DEBUG 1

//-------------------------------------------------------------------------
//  Prototypes
//-------------------------------------------------------------------------

UINT16   inicMsgRead(inic_MsgBuf_t MsgBuf, UINT8 *BufPtrDst);
UINT32   inicHwInit(UINT8 Mode);
UINT32   inicReInit(void);
UINT32   inicWaitForAnswer(void);
UINT32   inicWaitForAck(UINT8 Ack_Type,UINT32 time_in_ms);


UINT32   inicConfig(void);
void     inicHandleAcksRxMsg(UINT8 *buf);
void     inicStoreRxMsg(UINT8 *buf);
void     inicStoreMCM (UINT8 *buf);
UINT32   inicProceedNewMsg(void);

void     interp_inicFlashCommand(UINT8 *buf, UINT8 num_wr, UINT8 num_rd);
UINT32   inicFlashIPF(UINT8 *buf, UINT32 num_wr);
UINT32   inicFlashCommand(UINT8 *in_buf, UINT8 num_wr, UINT8 num_rd, UINT8 *ret_buf);
UINT32   inicFlashFirmware(UINT32 p_adr, UINT8 *buf, UINT32 num_wr);
UINT32   inicFlashConfigString(UINT32 p_adr, UINT8 *buf, UINT32 num_wr);
UINT32   inicReadReqMsgs(void);
UINT32   inicSendAndWaitAck(inic_MsgBuf_t MsgBuf, UINT8 *buf);
UINT32   inicSendWaitXCM (UINT8 mode, const UINT8 *cmd, UINT8 *buf, UINT32 Timeout);



//----------------------------------------------------------------
//  definitions of functions to flash Inic-Chip
//----------------------------------------------------------------
UINT32 pfProgramStart(void);
UINT32 pfEraseFlashEnable(void);
UINT32 pfEraseFlashPartitions(UINT8 startP, UINT8 numP);
UINT32 pfWriteProgramMemory(UINT32 adress, UINT32 len, UINT8 *pdata);
UINT32 pfSelFlashPage(UINT8 page);
UINT32 pfEraseInfoBlock(void);
UINT32 pfWriteInfoBlock(UINT8 adress, UINT8 len, UINT8 *pdata);
UINT32 pfClearCRC(void);
UINT32 pfGetCRC(UINT8 *crcdata);


#define INIC_GET_ADDR_LOGICAL      0
#define INIC_GET_ADDR_PHYSICAL      1

typedef struct inic_flash_params_s
{
   UINT8 first_partition;
   UINT8 last_partition ;
} inic_flash_params_t;

static inic_flash_params_t  os81050_fp = {8U,126U };
static inic_flash_params_t  os81110_fp = {6U,127U };
static inic_flash_params_t* current_fp = &os81050_fp;

UINT8   inic_debug_msg_level = 0;

#if (INIC_DEBUG_ON == 1)
   UINT32  count_InicReInits = 0;
   UINT32  count_InicRetries = 0;
   #define TRACE_CNT_MAX       200
   #define TRACE_CNT_LINESIZE  25
   #define TRACE_MSGTYPE_RX         0x01U
   #define TRACE_MSGTYPE_TX         0x02U
   #define TRACE_MSGTYPE_TX_RETRY   0x03U
   UINT8   tracebuffer[TRACE_CNT_MAX][TRACE_CNT_LINESIZE+5];
   UINT8   trace_msg_cnt = 0;
   UINT8   trace_active = 1;
   UINT32  inic_timestamp = 0; // muss in INTTM0EQ0 hochgezaehlt werden

   UINT32  trace_timestamp[TRACE_CNT_MAX];
#endif

//-------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------

static  UINT32  sh4i2c2inic (UINT32  theRet);

static  BOOL    inicInitBuffers(void);
#if (!BIOSLOADER)
static  UINT32  inicGetInfo();
static  UINT16  inicGetAddr(UINT8 AddrMode);
#endif

//-------------------------------------------------------------------------
// dynamic Functions
//-------------------------------------------------------------------------

typedef UINT32 TOS81xxx_Write_Str   (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length, UINT8 Mode);
typedef UINT32 TOS81xxx_Read_Str	   (UINT8 Addr, UINT8 *WR_Buf, UINT8 WR_Length,
                                     UINT8 *RD_Buf, UINT8 RD_Length, UINT8 Mode);

// Function pointers for dynamic driving couplings???
static TOS81xxx_Write_Str           *OS81xxx_Write_Str   = NULL;
static TOS81xxx_Read_Str            *OS81xxx_Read_Str    = NULL;

//-------------------------------------------------------------------------
// global Variables
//-------------------------------------------------------------------------

UINT16 own_NodeAddr    = 0;
UINT16 own_NodePosAddr = 0;
UINT32 inic_device = 0;
UINT8  inic_hwversion = 0;

static volatile UINT32  inicMsgLostCnt = 0;
static volatile UINT32  inicMostMsgLostCnt = 0;
static volatile UINT32  inicMostMCMLostCnt = 0;
static volatile UINT32  inicIntCounter = 0;

static UINT8 inicAccessMode;

// StatusByte for handling InicMessages
static volatile UINT8 InicStatus = 0x00;

//#define  INIC_READ_NEW_MSG     (1 << 0x00U)   // wird nicht mehr benoetigt
#define  INIC_READING_MSG      (1 << 0x01U)
#define  INIC_FPGABUF_OVERRUN  (1 << 0x02U)
#define  INIC_WAIT_ACK_MOSTMSG (1 << 0x03U)
#define  INIC_WAIT_ACK_ICM     (1 << 0x04U)
#define  INIC_WAIT_ACK_MCM     (1 << 0x05U)
#define  INIC_WAIT_ACK_MDP     (1 << 0x06U)
#define  INIC_WAIT_ACK_ALL     (1 << 0x07U)

TBuffer  *inicInBuffers[] =
{
   NULL, NULL,NULL,
};
#define  INIC_INBUF_SIZE     0x800U     // size of ringbuffer for Rx of inic messages
#define  INIC_PORTMSG_MAX_LEN    128


// added for use with Gateway and MostCmd-Channel
BOOL    mostInitialized = FALSE;            // will be set in MOSTInit
BOOL    inicHwInitialized = FALSE;
UINT16  inicUMostTrgtAddr = 0;


#define CONFCMD_VALID_MASTER  (1 << 0x00U)    // command valid for Inic as Master
#define CONFCMD_VALID_SLAVE   (1 << 0x01U)    // command valid for inic as Slave
#define CONFCMD_CHECK_DEVMODE (1 << 0x02U)    // command answer has to be checked for later decitions
#define CONFCMD_VALID_ALL     0xFFU           // before decision of SLAVE or MASTER accept all

//-------------------------------------------------------------------------
//  local Variables
//-------------------------------------------------------------------------

#if (!BIOSLOADER)
static   UINT8    mg_InicCommand[MSG_LENGTH_MAX];
#endif

// Variables for ICM-Trigger (command "X 23 11..."
// temporary buffer to send and Capure ICMs during command X 23 11
UINT8    inic_temp_ICM_msgbuf[INIC_PORTMSG_MAX_LEN+8];
BOOL     inic_ICM_ResponseTrigger   = FALSE;
UINT16   inic_ICM_FktID             = 0;

// Variables for MCM-Trigger (command "X 23 12..."
// temporary buffer to send and Capure ICMs during command X 23 11
UINT8    inic_temp_MCM_msgbuf[INIC_PORTMSG_MAX_LEN+8];
BOOL     inic_MCM_ResponseTrigger   = FALSE;
UINT16   inic_MCM_FktID             = 0;

BOOL     Extern_Inic = FALSE;

//----------------------------------------------------------------
//  definitions of standard messages to configure Inic-Chip
//----------------------------------------------------------------

#if CP_OS81050
   #define FIFOSTATUS_D6   (1<<6)
#endif
#if CP_OS81110
   #define FIFOSTATUS_D6   (0<<6)
#endif

// !! im Statusbyte (letztes Byte) muss mindestens D6 ausmaskiert werden
// D6 wird  beim OS81050 1 und beim OS81110 0 gelesen und ist undefiniert !!
// defines of ackknowledge messages to and from Inic OS81110
UINT8 INIC_ACK_MCM[] =           { 0x00, 0x03, 0x02, 0x02, 0x04 | FIFOSTATUS_D6};
UINT8 INIC_ACK_ICM[] =           { 0x00, 0x03, 0x02, 0x12, 0x04 | FIFOSTATUS_D6 };
UINT8 INIC_ACK_MDP[] =           { 0x00, 0x03, 0x02, 0x0A, 0x04 | FIFOSTATUS_D6 };
UINT8 INIC_ACK_ALL[] =           { 0x00, 0x03, 0x02, 0x1A, 0x04 | FIFOSTATUS_D6 };


// defines for startup sequence
UINT8 INIC_ALL_FIFO_SYNC[] =     { 0x00, 0x03, 0x02, 0x18, 0x80 };
UINT8 INIC_DISABLE_WD[] =        { 0x00, 0x0B, 0x01, 0x14, 0x30, 0xA0, 0x00, 0x05, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };

UINT8 INIC_ATTACH_SEMI[] =       { 0x00, 0x08, 0x01, 0x14, 0x30, 0x00, 0x00, 0x02, 0x01, 0x02 };
UINT8 INIC_ATTACH_FULL[] =       { 0x00, 0x08, 0x01, 0x14, 0x30, 0x00, 0x00, 0x02, 0x02, 0x02 };

// ClockMode (0x204) OS81050 only
UINT8 INIC_CLK_SL_44K1[] =       { 0x00, 0x08, 0x01, 0x14, 0x20, 0x42, 0x00, 0x02, 0x00, 0x00 };
UINT8 INIC_CLK_MA_44K1[] =       { 0x00, 0x08, 0x01, 0x14, 0x20, 0x42, 0x00, 0x02, 0x01, 0x00 };

// RMCK (0x201)
UINT8 INIC_SET_RMCK[] =          { 0x00, 0x07, 0x01, 0x14, 0x20, 0x12, 0x00, 0x01, RMCK_FS_DEFAULT };
UINT8 INIC_GET_DEVMODE[] =       { 0x00, 0x06, 0x01, 0x14, 0x50, 0x21, 0x00, 0x00 };
UINT8 INIC_GET_VERSION[] =       { 0x00, 0x07, 0x01, 0x14, 0x20, 0x01, 0x00, 0x01, 0x00 };

// DeviceMode (0x502)
UINT8 INIC_DEVMODE_MASTER[]=     { 0x00, 0x07, 0x01, 0x14, 0x50, 0x22, 0x00, 0x01, 0x01 };
UINT8 INIC_DEVMODE_SLAVE[] =     { 0x00, 0x07, 0x01, 0x14, 0x50, 0x22, 0x00, 0x01, 0x00 };

// Set NodeAddress
//#define NADRHi (UINT8)((Gateway_MOST[0].Nodeadress & 0xFF00)>>8)
//#define NADRLo (UINT8)((Gateway_MOST[0].Nodeadress & 0x00FF))
//UINT8 INIC_SET_NODEADDR[]  = { 0x00, 0x0C, 0x05, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0x32, 0x00, 0x02, NADRHi, NADRLo };
//UINT8 INIC_SET_NODEADDR[]  = { 0x00, 0x0C, 0x05, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0x32, 0x00, 0x02, 0x01, 0x01 };

UINT8 INIC_NWSTARTUP[] =         { 0x00, 0x06, 0x01, 0x14, 0x50, 0x02, 0x00, 0x00};

UINT8 INIC_GET_NODEADDR[] =      { 0x00, 0x0A, 0x05, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00 ,0x31, 0x00, 0x00};
UINT8 INIC_GET_PHYS_NODEADDR[] = { 0x00, 0x0A, 0x05, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00 ,0x21, 0x00, 0x00};

//----------------------------------------------------------------
//  definitions for UMOST 150 sockets
//----------------------------------------------------------------

#if INIC_SPI_CONFIF_MBC
// Open INIC SPI-Port Mode 0  Int Treshold 0
UINT8 INIC_SOCKET1[]    = { 0x00, 0x0A, 0x01, 0x14, 0x40, 0x02, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00};

// Create Socket Packet Data into Inic
UINT8 INIC_SOCKET2[]    = { 0x00, 0x0B, 0x01, 0x14, 0x40, 0x32, 0x00, 0x05, 0x04, 0x00, 0x01, 0xFF, 0xFF};

// Create Socket Packet Data outoff Inic
UINT8 INIC_SOCKET3[]    = { 0x00, 0x0B, 0x01, 0x14, 0x40, 0x32, 0x00, 0x05, 0x04, 0x01, 0x01, 0xFF, 0xFF};
#endif

//----------------------------------------------------------------
//  definitions for UMOST 150 Communication Channel
//----------------------------------------------------------------

#define BROADCAST_ADDR                 0x03C8

   // BIOS : Bios Controll Message
   #define BIOS_FBLOCK_ID      0x50  // Kommunikation
   #define BIOS_INST_ID        0x00  // keine
   #define BIOS_FKT_ID_SET     0xC000 // Hersteller intern
   #define BIOS_FKT_ID_GET     0xC001 // Hersteller intern
   #define BIOS_FKT_ID_RESULT  0xC00C // Hersteller intern
   #define BIOS_FKT_ID_ERROR   0xC00F // Hersteller intern
   #define BIOS_MAX_DATA       			10
	#define INIC_XCM_RESPONSE_TIMEOUT	2000

static UINT8 BIOS_Ram[1024];

#if (INIC_DEBUG_ON == 1)
//-------------------------------------------------------------------------
// void   inicStoreDebugTrace(....)
//-------------------------------------------------------------------------
// Function: captures inic messages into trace buffer
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

void   inicStoreDebugTrace(UINT8 *buffer, UINT8 Kennung)
{
   UINT16 i,msglength;

   if ( trace_active == 0)
   {
      return;
   }

   if (  trace_msg_cnt >= TRACE_CNT_MAX)
   {
      return;
   }


   trace_timestamp[trace_msg_cnt] = inic_timestamp;

   msglength = buffer[0]*256+buffer[1];

   // clear line before writing;
   for (i=0;i<(TRACE_CNT_LINESIZE+5) ;i++ )
   {
      tracebuffer[trace_msg_cnt][i] = 0;
   }
   tracebuffer[trace_msg_cnt][0] = Kennung;
   for (i=0;i<(msglength+2) ; i++)
   {
        tracebuffer[trace_msg_cnt][i+1] = buffer[i];
   }
   trace_msg_cnt++;
}


//-------------------------------------------------------------------------
// void inicShowTrace(void)
//-------------------------------------------------------------------------
// Function: formatted output of inic communication trace
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
void inicShowTrace(void)
{
   UINT8 line,i;
   INT8 str[256];
   UINT16  msglen;

   printf("* Startup-Trace for incoming msgs\n");

   for (line=0;line <trace_msg_cnt ;line++ )
   {

       if (tracebuffer[line][0]== TRACE_MSGTYPE_RX)
       sprintf(str,"->RX    ");

       if (tracebuffer[line][0]== TRACE_MSGTYPE_TX)
       sprintf(str,"<-TX    ");

       if (tracebuffer[line][0]== TRACE_MSGTYPE_TX_RETRY)
       sprintf(str,"<-RETRY ");

       sprintf(str,"%s %8dms ",str,(UINT32)trace_timestamp[line]);

       // get length of msgs
       msglen =  (tracebuffer[line][1]*265) + tracebuffer[line][2];
       if (msglen >= TRACE_CNT_LINESIZE)
       {
          msglen =  TRACE_CNT_LINESIZE+1;
       }

       for (i=0 ;i < (msglen+2) ;i++ )
       {
           sprintf(str,"%s%02X",str,(UINT8)tracebuffer[line][i+1]);
       }
       printf("%s\n",str);
   }
}


//-------------------------------------------------------------------------
// UINT32  OS81xxx_Write_Str_Debug(....)
//-------------------------------------------------------------------------
// Function: trace outgoing msg to inic
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

//   muss fuer Debugging in config.h umdefiniert werden, damit alle msg zu tracen sind
//   #define OS81xxx_Write_Str        OS81xxx_Write_Str_Debug

UINT32  OS81xxx_Write_Str_Debug(UINT8 Addr, UINT8 *buffer, UINT8 length, UINT8 Mode)
{
UINT32 error;
         inicStoreDebugTrace(buffer, TRACE_MSGTYPE_TX);

         error = IIC_Write_2_Str( Addr, buffer , length , Mode);

         return(error);
}
#endif

//-------------------------------------------------------------------------
// void inicHwInit()
//-------------------------------------------------------------------------
// Function: first init of inic after startup
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

UINT32 inicHwInit(UINT8 Mode)
{
UINT32 subError;

   // Treiberumschaltung erforderlich ?
   if ((Mode & 0xF0) == 0x10)
      Extern_Inic = TRUE;
   else
      Extern_Inic = FALSE;

   // High 4 bit masked
   Mode &= 0x0F;

   inicAccessMode = 0xFFU;     // set to invalid mode for detection init
   subError = NO_ERROR;

   // Driver Dock????
   if (!Extern_Inic)
   {
      INIC_CONFIGURE_PORTS;
      OS81xxx_Write_Str = (TOS81xxx_Write_Str*)&OS81xxx_WRITE_STR;
      OS81xxx_Read_Str = (TOS81xxx_Read_Str*)&OS81xxx_READ_STR;
   }
   else
   {
      INIC_EXTERN_CONFIGURE_PORTS;
      OS81xxx_Write_Str = (TOS81xxx_Write_Str*)&OS81xxx_EXTERN_WRITE_STR;
      OS81xxx_Read_Str = (TOS81xxx_Read_Str*)&OS81xxx_EXTERN_READ_STR;
   }

   // ******************************
   // * checking selected InitMode *
   // ******************************

   switch (Mode)
   {
      case INIC_ACCESS_I2C:
         printf("     inicHwInit I2C\n");
         break;
      case INIC_ACCESS_FLASH:
            printf("     inicHwInit FLASHER\n");
         break;
      default:
         printf("     invalid start mode selcted for inic\n");
         return(ERROR_INIC | ERR_INIC_NOTINIT);

   }
   #if CP_CMDDEV_MOST
   mostInitialized = FALSE;
   if (inicInBuffers[inic_MostMsgBuf] != NULL)             // delete buffer used for most msg channel
   {
      bufDelete(inicInBuffers[inic_MostMsgBuf]);
      inicInBuffers[inic_MostMsgBuf] = NULL;
      printf("     inic MOST buffer already been deleted!!!!!\n");      
   }
   #endif

   inicAccessMode = Mode;

   if (inicInBuffers[inic_CtrlMsgBuf] != NULL)
   {                                           // delete buffer for use via X 23 02-commands
      bufDelete(inicInBuffers[inic_CtrlMsgBuf]);
      inicInBuffers[inic_CtrlMsgBuf] = NULL;
      printf("     inic Ctrl buffer already been deleted!!!!!\n"); 
   }

   if (inicInBuffers[inic_MostMCMBuf] != NULL)
   {                                           // delete buffer for use via X 23 13-commands
      bufDelete(inicInBuffers[inic_MostMCMBuf]);
      inicInBuffers[inic_MostMCMBuf] = NULL;
      printf("     inic MCM buffer already been deleted!!!!!\n"); 
   }

   inicHwInitialized=FALSE;


   // ***********************************
   // * Init Inic for I2C communication *
   // ***********************************
   if (inicAccessMode == INIC_ACCESS_I2C)
   {
      printf("     Reset device...\n");// Reset INIC device
      // Reset INIC device
      if (!Extern_Inic)
      {
         INIC_RES_ON;
         timWait(25);
         INIC_RES_OFF;
      }
      else
      {
         INIC_EXTERN_ERR_BOOT_INPUT;
         INIC_EXTERN_RES_ON;
         timWait(25);
         INIC_EXTERN_RES_OFF;
      }

      // wait 100ms because INIC V1.09
      // causes wrong interrupt after reset
      timWait(100);
      printf("     Done\n");
   }

   #if (!BIOSLOADER)
   // ***********************************
   // * Init Inic for Flasher           *
   // ***********************************
   if (inicAccessMode == INIC_ACCESS_FLASH)
   {
            // ***********************************************
            // * do init for flash mode                      *
            // * handled case: inic direct connected to SH   *
            // ***********************************************

      // run inic startup sequence for boot in flasher mode
      // for the implementation with inic connected direct at cpu via i2c
      //           ___     _______________________________________      ________
      // _RST         |___|                                       |____|
      //
      //           _____                                        ________________
      // ERR/_BOOT      |_____XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
      //
      //             Reset Inic
      //                 Set for (programming mode
      //                    Boot Inic in programming mode
      //                      programming mode

      if (!Extern_Inic)
      {
                                    // _ERR/BOOT   _RST
      INIC_ERR_BOOT_HIGH;           //
      INIC_RES_OFF;                 //     '1'       '1'
         timWait(2);                   //
      INIC_RES_ON;                  //     '1'       '0'
         timWait(10);                  //
      INIC_ERR_BOOT_LOW;            //     '0'       '0'
         timWait(2);                   //
      INIC_RES_OFF;                 //     '0'       '1'
         timWait(40);                  //
      INIC_ERR_BOOT_INPUT;          //     Inic now booted for programming
      }
      else
      {
         // _ERR/BOOT   _RST
         INIC_EXTERN_ERR_BOOT_HIGH;    //
         INIC_EXTERN_RES_OFF;          //     '1'       '1'
         timWait(2);                   //
         INIC_EXTERN_RES_ON;           //     '1'       '0'
         timWait(10);                  //
         INIC_EXTERN_ERR_BOOT_LOW;     //     '0'       '0'
         timWait(2);                   //
         INIC_EXTERN_RES_OFF;          //     '0'       '1'
         timWait(40);                  //
         INIC_EXTERN_ERR_BOOT_INPUT;   //     Inic now booted for programming
      }

      inicHwInitialized=TRUE;

      return(subError);
   }
   #endif


   // ***********************
   // * common part of init *
   // ***********************
   // create input buffer --------------------------------------
   #if CP_CMDDEV_MOST
   printf("     create inicInBuffers[inic_MostMsgBuf]...\n");
   if((inicInBuffers[inic_MostMsgBuf] = bufCreate(INIC_INBUF_SIZE)) == NULL)
   { // creation of input buffer not possible
      printf("     error: inicInBuffers[inic_MostMsgBuf] failed!\n");
      return(ERROR_INIC | ERR_INIC_BUFCREATE);
   }
      printf("     inicInBuffers[inic_MostMsgBuf] starting @0x%08X\n",(UINT32)inicInBuffers[inic_MostMsgBuf]);
   #endif

   printf("     create inicInBuffers[inic_CtrlMsgBuf]...\n");
   if((inicInBuffers[inic_CtrlMsgBuf] = bufCreate(INIC_INBUF_SIZE)) == NULL)
   { // creation of input buffer not possible
      printf("     error: inicInBuffers[inic_CtrlMsgBuf] failed!\n");
      return(ERROR_INIC | ERR_INIC_BUFCREATE);
   }
      printf("     inicInBuffers[inic_CtrlMsgBuf] starting @0x%08X\n",(UINT32)inicInBuffers[inic_CtrlMsgBuf]);

   printf("     create inicInBuffers[inic_MostMCMBuf]...\n");
   if((inicInBuffers[inic_MostMCMBuf] = bufCreate(INIC_INBUF_SIZE)) == NULL)
   { // creation of input buffer not possible
      printf("     error: inicInBuffers[inic_MostMCMBuf] failed!\n");
      return(ERROR_INIC | ERR_INIC_BUFCREATE);
   }
      printf("     inicInBuffers[inic_MostMCMBuf] starting @0x%08X\n",(UINT32)inicInBuffers[inic_MostMCMBuf]);


   inicInitBuffers();

   inicHwInitialized=TRUE;

   // wait for first answer after InicReset
   inicWaitForAnswer();

   return(subError);
}



// ------------------------------------------------------------------------
BOOL newMsgReady(void)  // reagiert auf falling edge
{
   /*
   static   BOOL OldPinLevel = TRUE;
   BOOL          NewPinLevel;


   // holt Pin Level
   NewPinLevel = OS81xxx_INT_PIN;

   // erkennt Flankenwechsel
   if (NewPinLevel != OldPinLevel)
   {
      OldPinLevel = NewPinLevel;
      if (NewPinLevel == FALSE)     return(TRUE);  // neue Msg liegt an
   }
   return(FALSE);                                  // keine neue Msg
   */

   if (OS81xxx_INT_PIN)
   {
    return(FALSE);
    printf("MsgReady is Flase. \n");    
   }
   else           
    return(TRUE);
}
// ------------------------------------------------------------------------




//-------------------------------------------------------------------------
// void inicReInit()
//-------------------------------------------------------------------------
// Function: first init of inic after startup
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
UINT32 inicReInit(void)
{
UINT32 subError;

#if (INIC_DEBUG_ON == 1)
   count_InicReInits++;   // DEBUG_ONLY
#endif

   subError = NO_ERROR;
   inicHwInitialized=FALSE;

   // run inicHwInit
   subError = inicHwInit(INIC_ACCESS_I2C);

   if (subError == NO_ERROR)
   {
      inicHwInitialized=TRUE;

      timSetMarker(TIMER_INIC);
      while(1)
      {
         if (timTimeout(TIMER_INIC, 1000))
         {
            break;
         }

         if (newMsgReady())
         {
            inicProceedNewMsg();
            break;
         }
      }
      subError = inicConfig();

   }
   return(subError);
}


//-------------------------------------------------------------------------
// inicMsgRead(inic_MsgBuf_t MsgBuf, UINT8 *BufPtrDst)
//-------------------------------------------------------------------------
// Function:  Reads one Message from the inicInBuffer
//-------------------------------------------------------------------------
// Input:   Ptr to destination buffer
//
// Output:  number of bytes transfered from ring buffer
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

UINT16 inicMsgRead(inic_MsgBuf_t MsgBuf, UINT8 *BufPtrDst)
{
UINT16 i,length;
UINT8  lenHi,lenLo;

      length = 0;
      if (bufItems(inicInBuffers[MsgBuf]) != 0)
      {       
         bufGet(inicInBuffers[MsgBuf], BufPtrDst);             // read msg length from FiFo
         lenHi = *BufPtrDst++;
         bufGet(inicInBuffers[MsgBuf], BufPtrDst);             // read msg length from FiFo
         lenLo = *BufPtrDst++;
         length = (lenHi << 8)+lenLo;

         for (i=0;i<length ;i++ )                  // transfer 1 message from FiFo
         {                                         // to given buffer for application
            bufGet(inicInBuffers[MsgBuf], (BufPtrDst++));
         }
         return(length+2);
      }
      else
      {
         return(length);
      }
}


//-------------------------------------------------------------------------
// inicTestIfAckMsg(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   test if msg given in buf is a AckMsg from Inic to EHC
//
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
BOOL inicTestIfAckMsg(UINT8 *buf)
{
      if (memcmp(buf, INIC_ACK_MCM, sizeof(INIC_ACK_MCM)) == 0)
      {
         return(TRUE);
      }

      // ICM-Acknowledge from Inic
      if (memcmp(buf, INIC_ACK_ICM, sizeof(INIC_ACK_ICM)) == 0)
      {
         return(TRUE);
      }

      // MDP-Acknowledge from Inic
      if (memcmp(buf, INIC_ACK_MDP, sizeof(INIC_ACK_MDP)) == 0)
      {
         return(TRUE);
      }

      // ALL-Acknowledge from Inic ; ignore contents of status message
      if (memcmp(buf, INIC_ACK_ALL, sizeof(INIC_ACK_ALL)-1) == 0)
      {
         return(TRUE);
      }

      return(FALSE);
}

//-------------------------------------------------------------------------
// inicStoreRxMsg(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   test for buffer to store incomming message from inic
//
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  enum for selected buffer
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
void inicStoreRxMsg(UINT8 *buf)
{
   inic_MsgBuf_t SelBuffer;
   UINT16   aPML;
   UINT32   spaceRemain;
   UINT8    i;


   // *****************************************************
   // * dispatcher between MostMsg and Inic BiosCommand   *
   // *****************************************************
   //
   // get length of incoming message
   aPML = readWord(buf);
   // select standard buffer
   SelBuffer = inic_CtrlMsgBuf;

   #if CP_CMDDEV_MOST
   if (MostMsgRunning())
   {  // Test for MDP; Packet Data Length is volatile
      if (((buf[3U] & INIC_FPH_FIFONO_MASK) == INIC_FPH_FIFONO_MDP) &&
          ((buf[3U] & INIC_FPH_FIFOMT_MASK) == INIC_FPH_FIFOMT_DAT))
            // Incoming Msg correct for Mostmsg:
            SelBuffer = inic_MostMsgBuf;
   }

   // check also Ack-Msg before to dispatch for different use
   if (InicStatus & INIC_WAIT_ACK_MOSTMSG)
   {
      // MDP-Acknowledge from Inic
      if (memcmp(buf, INIC_ACK_MDP, sizeof(INIC_ACK_MDP)) == 0)
      {
         SelBuffer = inic_MostMsgBuf;                      // select mostmsg-buffer
         //  InicStatus &= ~INIC_WAIT_ACK_MOSTMSG;         // clr Ack wait status Mostmsg
      }
   }
   #endif

   spaceRemain = bufSpace(inicInBuffers[SelBuffer]);

   if ((aPML + 2) < spaceRemain)                 // check for available space in FiFo
   {
      for (i= 0; i<(aPML+2) ;i++ )              // transfer new received message into FiFo
      {                                           // to transfer for use in main loop
         bufPutFiFo(inicInBuffers[SelBuffer],buf[i]);
      }
   }
   else
   {  // count lost Messages from Inic
      if (SelBuffer == inic_CtrlMsgBuf)
         inicMsgLostCnt++;

      if (SelBuffer == inic_MostMsgBuf)
         inicMostMsgLostCnt++;
   }
} // of function


//-------------------------------------------------------------------------
// inicStoreMCM(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   store incomming MCM message from inic
//
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  enum for selected buffer
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
void inicStoreMCM (UINT8 *buf)
{
   UINT16   aPML;
   UINT32   spaceRemain;
   UINT8    i;

   // get length of incoming message
   aPML = readWord(buf);

   if (((buf[3U] & INIC_FPH_FIFONO_MASK) != INIC_FPH_FIFONO_MCM) ||
       ((buf[3U] & INIC_FPH_FIFOMT_MASK) != INIC_FPH_FIFOMT_DAT))
   return;

   spaceRemain = bufSpace(inicInBuffers[inic_MostMCMBuf]);

   if ((aPML + 2) < spaceRemain)                 // check for available space in FiFo
   {
      for (i= 0; i<(aPML+2) ;i++ )              // transfer new received message into FiFo
      {                                           // to transfer for use in main loop
         bufPutFiFo(inicInBuffers[inic_MostMCMBuf],buf[i]);
      }
   }
   else
   {  // count lost Messages from Inic
      inicMostMCMLostCnt++;
   }
} // of function


//-------------------------------------------------------------------------
// BOOL inic_Capture_ICM_Response(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   Get a copy of a special ICM-response
//
//-------------------------------------------------------------------------
// Input:   buf
//
// Output:  BOOL
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------
// X 230200000000 0000   0007  01 14  403  C  0  001     00    // old response
// X 231100000000                     403  C             00    // new response
//                |      |     |  |   |    |  |  |       |
//     Bios Länge_|      |     |  |   |    |  |  |       |
//     Inic PML  ________|     |  |   |    |  |  |       |
//     Inic PMHL_______________|  |   |    |  |  |       |
//     Inic FPH___________________|   |    |  |  |       |
//     Inic FktID_____________________|    |  |  |       |
//     Inic OpType_________________________|  |  |       |
//     Inic TelID_____________________________|  |       |
//     Inic Length_______________________________|       |
//     Inic Data 0..n____________________________________|

BOOL inic_Capture_ICM_Response(UINT8 *buf)
{
   UINT8    i;

   // Test for ICM and compare Trigger Function ID with Function ID of this response
   if (((buf[3U] & INIC_FPH_FIFONO_MASK) != INIC_FPH_FIFONO_ICM) ||
       ((buf[3U] & INIC_FPH_FIFOMT_MASK) != INIC_FPH_FIFOMT_DAT) ||
        (buf[2U] != 1) ||
        (inic_ICM_FktID != ((buf [4] << 4) + (buf [5] >>4))))
      return(FALSE);

   // Message is matching, get a copy in bioscontrol format

   //calc bioscontrol length
   inic_temp_ICM_msgbuf[0] = buf[1] +4;

   // copy Function ID + Opcode
   inic_temp_ICM_msgbuf[8] = buf [4];
   inic_temp_ICM_msgbuf[9] = buf [5];

   // copy data
   for (i= 0; i< buf[7] ; i++ )
   {
      inic_temp_ICM_msgbuf[i +10] = buf [i + 8];
   }

   return(TRUE);
} // of function


//-------------------------------------------------------------------------
// BOOL inic_Capture_MCM_Response(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   Get a copy of a special MCM-response
//
//-------------------------------------------------------------------------
// Input:   buf
//
// Output:  BOOL
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------
// X 230200000000 0000   000D  06 04  00 0100 0001 400 F  0  002 0402    // old response
// X 231200000000                     00 0100 0001 400 F         0402    // new response
//                |      |     |  |   |  |    |    |   |  |  |   |
//     Bios Länge_|      |     |  |   |  |    |    |   |  |  |   |
//     Inic PML  ________|     |  |   |  |    |    |   |  |  |   |
//     Inic PMHL_______________|  |   |  |    |    |   |  |  |   |
//     Inic FPH___________________|   |  |    |    |   |  |  |   |
//     Inic FDH TgtDevType____________|  |    |    |   |  |  |   |
//     Inic FDH SrcDevID_________________|    |    |   |  |  |   |
//     Inic FDH FunAdr________________________|    |   |  |  |   |
//     Inic FktID__________________________________|   |  |  |   |
//     Inic OpType_____________________________________|  |  |   |
//     Inic TelID_________________________________________|  |   |
//     Inic Length___________________________________________|   |
//     Inic Data 0..n____________________________________________|

BOOL inic_Capture_MCM_Response(UINT8 *buf)
{
   UINT8    i;

   // Test for MCM and compare Trigger Function ID with Function ID of this response
   if (((buf[3U] & INIC_FPH_FIFONO_MASK) != INIC_FPH_FIFONO_MCM) ||
       ((buf[3U] & INIC_FPH_FIFOMT_MASK) != INIC_FPH_FIFOMT_DAT) ||
        (buf[2U] != 6) ||
        (inic_MCM_FktID != ((buf [9] << 4) + (buf [10] >>4))))
      return(FALSE);

   // Message is matching, get a copy in bioscontrol format

   //calc bioscontrol length
   inic_temp_MCM_msgbuf[0] = buf[1] +4;

   // copy FDH
   inic_temp_MCM_msgbuf[8]  = buf [4];
   inic_temp_MCM_msgbuf[9]  = buf [5];
   inic_temp_MCM_msgbuf[10] = buf [6];
   inic_temp_MCM_msgbuf[11] = buf [7];
   inic_temp_MCM_msgbuf[12] = buf [8];

   // copy Function ID + Opcode
   inic_temp_MCM_msgbuf[13] = buf [9];
   inic_temp_MCM_msgbuf[14] = buf [10];

   // copy data
   for (i= 0; i< buf[12] ; i++ )
   {
      inic_temp_MCM_msgbuf[i +15] = buf [i + 13];
   }

   return(TRUE);
} // of function


//-------------------------------------------------------------------------
// BOOL Read_BIOS_Data (UINT16 Address, UINT8 Count, UINT8 *Data)
//-------------------------------------------------------------------------
// Function:
//
//-------------------------------------------------------------------------
// Input:
//
// Output:
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------

BOOL Read_BIOS_Data (UINT16 Address, UINT16 Count, UINT8 *Data)
{
   // printf ("Read_BIOS_Data : Address %04X Count %04X  \n",(UINT32)Address, (UINT32)Count);

   if ((Count > BIOS_MAX_DATA) || (Count == 0))
   {
      printf ("Error : 0 < Count <= BIOS_MAX_DATA\n");
      return(false);
   }

   if ((Address+Count) > sizeof (BIOS_Ram))
   {
      printf ("Error : max Memory address = %d\n", sizeof (BIOS_Ram)-1);
      return(false);
   }

   memcpy (Data, BIOS_Ram+Address, Count);
   return(TRUE);
} // of function


//-------------------------------------------------------------------------
// BOOL Write_BIOS_Data (UINT16 Address, UINT8 Count, UINT8 *Data)
//-------------------------------------------------------------------------
// Function:
//
//-------------------------------------------------------------------------
// Input:
//
// Output:
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------

BOOL Write_BIOS_Data (UINT16 Address, UINT16 Count, UINT8 *Data)
{
   // printf ("Write_BIOS_Data : Address %04X Count %04X  \n",(UINT32)Address, (UINT32)Count);

   if ((Count > BIOS_MAX_DATA) || (Count == 0))
   {
      printf ("Error : 0 < Count <= BIOS_MAX_DATA\n");
      return(false);
   }

   if ((Address+Count) > sizeof (BIOS_Ram))
   {
      printf ("Error : max Memory address = %d\n", sizeof (BIOS_Ram)-1);
      return(false);
   }

   memcpy (BIOS_Ram+Address, Data, Count);
   return(TRUE);
} // of function


//-------------------------------------------------------------------------
// BOOL inic_Capture_BIOS_Response(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   Get a copy of a special MCM-response
//
//-------------------------------------------------------------------------
// Input:   buf
//
// Output:  BOOL
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------
// X 230200000000 0000   000D  06 04  00 0100 0001 400 F  0  002 0402    // old response
// X 231200000000                     00 0100 0001 400 F         0402    // new response
//                |      |     |  |   |  |    |    |   |  |  |   |
//     Bios Länge_|      |     |  |   |  |    |    |   |  |  |   |
//     Inic PML  ________|     |  |   |  |    |    |   |  |  |   |
//     Inic PMHL_______________|  |   |  |    |    |   |  |  |   |
//     Inic FPH___________________|   |  |    |    |   |  |  |   |
//     Inic FDH TgtDevType____________|  |    |    |   |  |  |   |
//     Inic FDH SrcDevID_________________|    |    |   |  |  |   |
//     Inic FDH FunAdr________________________|    |   |  |  |   |
//     Inic FktID__________________________________|   |  |  |   |
//     Inic OpType_____________________________________|  |  |   |
//     Inic TelID_________________________________________|  |   |
//     Inic Length___________________________________________|   |
//     Inic Data 0..n____________________________________________|

BOOL inic_Capture_BIOS_Response(UINT8 *buf)
{
   // Test for BIOS and Optype get
   if (((buf[3U] & INIC_FPH_FIFONO_MASK) != INIC_FPH_FIFONO_MCM) ||
       ((buf[3U] & INIC_FPH_FIFOMT_MASK) != INIC_FPH_FIFOMT_DAT) ||
        (buf[2U] != 6) ||
        (BIOS_FBLOCK_ID != buf [7]) ||
        (BIOS_INST_ID   != buf [8]) ||
        (BIOS_FKT_ID_SET != ((buf [9] << 8) + buf [10])))
      return(FALSE);

   // Message is matching, proceed
   // printf ("BIOS_FKT_ID_SET detected\n");

   Write_BIOS_Data (readWord(buf+13), buf[12]-2, buf+15);
   return(TRUE);
} // of function


//-------------------------------------------------------------------------
// BOOL inic_Capture_BIOS_Request(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   Get and proceed an BIOS-response
//
//-------------------------------------------------------------------------
// Input:   buf
//
// Output:  BOOL
//-------------------------------------------------------------------------
// A.Kutzki, EEHB, Tel. 07248 / 711857
//-------------------------------------------------------------------------
// X 230200000000 0000   000D  06 04  00 0100 0001 400 F  0  002 0402    // old response
// X 231200000000                     00 0100 0001 400 F         0402    // new response
//                |      |     |  |   |  |    |    |   |  |  |   |
//     Bios Länge_|      |     |  |   |  |    |    |   |  |  |   |
//     Inic PML  ________|     |  |   |  |    |    |   |  |  |   |
//     Inic PMHL_______________|  |   |  |    |    |   |  |  |   |
//     Inic FPH___________________|   |  |    |    |   |  |  |   |
//     Inic FDH TgtDevType____________|  |    |    |   |  |  |   |
//     Inic FDH SrcDevID_________________|    |    |   |  |  |   |
//     Inic FDH FunAdr________________________|    |   |  |  |   |
//     Inic FktID__________________________________|   |  |  |   |
//     Inic OpType_____________________________________|  |  |   |
//     Inic TelID_________________________________________|  |   |
//     Inic Length___________________________________________|   |
//     Inic Data 0..n____________________________________________|

BOOL inic_Capture_BIOS_Request (void)
{
   // Test for BIOS and Optype get
   if ((BIOS_FBLOCK_ID == commandPtr[6]) &&
       (BIOS_INST_ID   == commandPtr[7]) &&
       (BIOS_FKT_ID_GET == ((commandPtr[8] << 8) + commandPtr[9])))
   {
      // Message is matching, proceed
      // printf ("BIOS_FKT_ID_GET detected\n");

      // copy
      resultPtr[8]  = 0;
      resultPtr[9]  = commandPtr[4];
      resultPtr[10] = commandPtr[5];
      resultPtr[11] = BIOS_FBLOCK_ID;
      resultPtr[12] = BIOS_INST_ID;

      if (Read_BIOS_Data (readWord(commandPtr+10), readWord(commandPtr+12), resultPtr+15))
      {
         // No Error, copy Function ID and insert opcode C
         resultPtr[13] = BIOS_FKT_ID_RESULT >> 8;
         resultPtr[14] = BIOS_FKT_ID_RESULT & 0x00FF;
         //calc bioscontrol length
         resultPtr[0] = readWord(commandPtr+12) +15;
      }
      else
      {
         // Error, copy Function ID and insert opcode F
         resultPtr[13] = BIOS_FKT_ID_ERROR >> 8;
         resultPtr[14] = BIOS_FKT_ID_ERROR & 0x00FF;
         //calc bioscontrol length
         resultPtr[0] = 15;
      }
      writeLong(resultPtr+4, NO_ERROR);
      return(TRUE);
   }

   // Test for BIOS and Optype get
   if ((BIOS_FBLOCK_ID == commandPtr[6]) &&
       (BIOS_INST_ID   == commandPtr[7]) &&
       (BIOS_FKT_ID_SET == ((commandPtr[8] << 8) + commandPtr[9])))
   {
      // Message is matching, make a copy of data
      // printf ("BIOS_FKT_ID_SET detected\n");
      Write_BIOS_Data (readWord(commandPtr+10), commandPtr[0]-12, commandPtr+12);
   }

   return(FALSE);
  } // of function


//-------------------------------------------------------------------------
// inicHandleIncMsgAcks(UINT8 *buf)
//-------------------------------------------------------------------------
// Function:   Handles Acks for incoming Messages
//
//-------------------------------------------------------------------------
// Input:   buffer containing incomming InicMessage
//
// Output:
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

void inicHandleAcksRxMsg(UINT8 *buf)
{

   // **********************************************
   // * do Ack-Handling for the incoming messages  *
   // **********************************************

   // MCM-Acknowledge from Inic
   if (memcmp(buf, INIC_ACK_MCM, sizeof(INIC_ACK_MCM)) == 0)
   {  // clr Ack wait bit
      InicStatus &= ~INIC_WAIT_ACK_MCM;
      return;
   }

   // ICM-Acknowledge from Inic
   if (memcmp(buf, INIC_ACK_ICM, sizeof(INIC_ACK_ICM)) == 0)
   {  // clr Ack wait bit
      InicStatus &= ~INIC_WAIT_ACK_ICM;
      return;
   }

   // MDP-Acknowledge from Inic
   if (memcmp(buf, INIC_ACK_MDP, sizeof(INIC_ACK_MDP)) == 0)
   {  // clr Ack wait bit
      InicStatus &= ~INIC_WAIT_ACK_MDP;
      return;
   }

   // ALL-Acknowledge from Inic ; ignore contents of status message
   if (memcmp(buf, INIC_ACK_ALL, sizeof(INIC_ACK_ALL)-1) == 0)
   {  // clr Ack wait bit
      InicStatus &= ~INIC_WAIT_ACK_ALL;
      return;
   }

   // *********************************************************
   // * test start of incoming msg to get needed Ack-message  *
   // *********************************************************

   switch (buf[3U] & INIC_FPH_FIFONO_MASK)
   {
      case INIC_FPH_FIFONO_MDP:
         // MDP from Inic
         // if no wait, breaks in commands with heavy traffic occured
         DelayLoop(us(INIC_RX_DEADTIME));
         OS81xxx_Write_Str( INIC_I2C_ADDR, INIC_ACK_MDP, readWord(INIC_ACK_MDP)+2, 0);
         break;

      case INIC_FPH_FIFONO_MCM:
         // MCM from Inic
         // if no wait, breaks in commands with heavy traffic occured
         DelayLoop(us(INIC_RX_DEADTIME));
         OS81xxx_Write_Str( INIC_I2C_ADDR, INIC_ACK_MCM, readWord(INIC_ACK_MCM)+2, 0);
         break;

      case INIC_FPH_FIFONO_ICM:
         // ICM from Inic
         // if no wait, breaks in commands with heavy traffic occured
         DelayLoop(us(INIC_RX_DEADTIME));
         OS81xxx_Write_Str( INIC_I2C_ADDR, INIC_ACK_ICM, readWord(INIC_ACK_ICM)+2, 0);
         break;

      case INIC_FPH_FIFONO_ALL:
         // ICM from Inic
         // if no wait, breaks in commands with heavy traffic occured
         DelayLoop(us(INIC_RX_DEADTIME));
         OS81xxx_Write_Str( INIC_I2C_ADDR, INIC_ACK_ALL, readWord(INIC_ACK_ALL)+2, 0);
         break;

      default:
         break;
   }
}


//-------------------------------------------------------------------------
// inicProceedNewMsg (Inic controlling via I2C)
//-------------------------------------------------------------------------
// Function:  function for cyclic call in main loop
//            checks if inic-interrupt was detected and transfers
//            the message from inic into inicInBuffer
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// W.Doczkal, EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
// Funktion wird regelmaessig gepollt:
// MBC-Handler --> MOST_O_Receive() --> inicProceedNewmsg()
UINT32 inicProceedNewMsg(void)
{
   static   UINT8 PortMessage[INIC_PORTMSG_MAX_LEN];
   UINT32   aStatus;
   UINT16   aBytes;


   if (inicAccessMode != INIC_ACCESS_I2C) return(ERROR_INIC | ERR_INIC_IO);
   if (inicHwInitialized == FALSE)        return(ERROR_INIC | ERR_INIC_NOTINIT);

   if (newMsgReady() == FALSE)   return(NO_ERROR); // es gibt nichts zu machen

   // read Data from Inic
   aStatus = OS81xxx_Read_Str (INIC_I2C_ADDR, NULL, 0,
                              (UINT8*) PortMessage,
                              (UINT8)(sizeof(PortMessage)), INIC_I2C_MODE);
   #if (INIC_DEBUG_ON == 1)
      inicStoreDebugTrace(PortMessage, TRACE_MSGTYPE_RX);
   #endif

   aBytes = readWord (PortMessage);

   if ((aBytes == 0) && (aStatus == NO_ERROR))
      aStatus = (ERROR_INIC | ERR_INIC_PROT);

   if (aStatus != NO_ERROR)
      return (sh4i2c2inic(aStatus));

   //  check if inic_ctrl or bios-comm-msg
   //  and store message into neccessary buffer
   inicStoreRxMsg(PortMessage);

   // do Ack-Handling for the incoming messages
   inicHandleAcksRxMsg(PortMessage);

   // copy a special ICM response based on a FunctionID
   // ony one copy possible after trigger is armed
   if (inic_ICM_ResponseTrigger)
      if (inic_Capture_ICM_Response(PortMessage))
         inic_ICM_ResponseTrigger = FALSE;

   // copy a special MCM response based on a FunctionID
   // ony one copy possible after trigger is armed
   if (inic_MCM_ResponseTrigger)
   {
      if (inic_Capture_MCM_Response(PortMessage))
         inic_MCM_ResponseTrigger = FALSE;
      else
         inicStoreMCM(PortMessage);
   }
   else
      inicStoreMCM(PortMessage);

   // look for BIOS Messages
   inic_Capture_BIOS_Response(PortMessage);

   return(sh4i2c2inic(aStatus));
}


//-------------------------------------------------------------------------
// inicReadReqMsgs
//-------------------------------------------------------------------------
// Function: read out all old messages from inic
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------
UINT32 inicReadReqMsgs(void)
{
    UINT32  error = NO_ERROR;

    while (newMsgReady())
    {
        error = inicProceedNewMsg();
    }
    return(error);
}


//-------------------------------------------------------------------------
// inicWaitForAck
//-------------------------------------------------------------------------
// Function: wait some time for Inic to proceed last message and
//           read return when Ack_MCM was received
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

UINT32 inicWaitForAck(UINT8 Ack_Type,UINT32 time_in_ms)
{
   timSetMarker(TIMER_INIC);

   while(InicStatus & Ack_Type)
   {
      inicProceedNewMsg();
      if (timTimeout(TIMER_INIC, time_in_ms))
      {
          printf("** Inic: Timeout waiting for Ack Msg\n");
          return(ERROR_INIC | ERR_INIC_CMD_TIMEOUT);
      }
   }
   return(NO_ERROR);
}


//-------------------------------------------------------------------------
// inicWaitForAnswer
//-------------------------------------------------------------------------
// Function: waits for answer from Inic (OS81050)
//           returns after 1st msg received after call
//           or timeout is reached
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

UINT32 inicWaitForAnswer(void)
{
   timSetMarker(TIMER_INIC);

   while(1)
   {
      if (timTimeout(TIMER_INIC, 500))
         return(ERROR_INIC | ERR_INIC_CMD_TIMEOUT);

      if (newMsgReady())
      {
         inicProceedNewMsg();
         return(NO_ERROR);
      }
   }
}


//-------------------------------------------------------------------------
// inicConfigIC2
//-------------------------------------------------------------------------
// Function: send inic configuration messages via i2c
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

#define TIMWAIT_NEXT_CFGCMD 5     // wait time between command for inicConfig

UINT32 inicConfig(void)
{
UINT32 sub_error;
UINT32 error;
//UINT16 testaddr;
UINT8  inicTestMsg[80];
UINT16 aPml;

UINT8 inic_cfg_mode;          // internal Mode handling during startup configuration

   inic_cfg_mode = CONFCMD_VALID_ALL;       // all messges are alloewd

  	sub_error = 0;
  	printf("** Inic: Config\n");

  	if ( inicHwInitialized == FALSE)
  	{
     	printf("     Error: inicHWInit not done!\n");
     	return(0xFFFFFFFF);
  	}

  	#if CP_CMDDEV_MOST
   mostInitialized = FALSE;
  	#endif

  	error = NO_ERROR;


// -----------------------------------------
//   detect if firmware in Inic is running
// -----------------------------------------

  	timSetMarker(TIMER_INIC);
   while(1)
   {
      inicProceedNewMsg();
      aPml = inicMsgRead(inic_CtrlMsgBuf, (UINT8*)&inicTestMsg[0]);
      
      if (aPml != 0)
         break;
      
      if (timTimeout(TIMER_INIC, 1000))
      {      
         error = ERR_INIC_STARTUP_FAILED;
         break;
      }
   }

  	if (error != NO_ERROR)
  	{
     	jprintf("Fehlerstelle 1");

     	printf("     no message found in buffer! Please reset INIC and try again.\n");
     	printf("     if error repeats INIC may have no firmware\n");
     	return(error);
  	}
  	else
     	inicInitBuffers();

  	// Read out 1st Msg after HW-Init
  	//  inicWaitForAnswer();

  	// Send all FiFo Sync
  	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_ALL_FIFO_SYNC);
  	if (sub_error) printf("** error inicConfig cmd(1): INIC_ALL_FIFO_SYNC\n");
  	error |= sub_error;

  	inicReadReqMsgs();

  	// dummy access, to check if INIC reacts on commands
  	error = inicSendWaitXCM (INIC_WAIT_ACK_ICM, INIC_GET_DEVMODE, inic_temp_ICM_msgbuf, 1000);
  	if (error != NO_ERROR)
  	{
  	  jprintf("Fehlerstelle 2");

  	  printf("      no message found in buffer! Please reset INIC and try again.\n");
  	  printf("      if error repeats INIC may have no firmware\n");
  	  return(error);
  	}

  	// Disable Watchdog
  	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf,INIC_DISABLE_WD);
  	if (sub_error) printf("** error InicConfig cmd(2): INIC_DISABLE_WD\n");
  	error |= sub_error;

  	inicReadReqMsgs();

  	// Set Attachment
  	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf,INIC_ATTACH_SEMI);
  	if (sub_error) printf("** error inicConfig cmd(3): INIC_ATTACH_SEMI\n");
  	error |= sub_error;

  	inicReadReqMsgs();

  	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf,INIC_ATTACH_FULL);
  	if (sub_error) printf("** error inicConfig cmd(4): INIC_ATTACH_FULL\n");
  	error |= sub_error;

  	inicReadReqMsgs();

  	// Set Device Mode
   if ((Gateway_MOST[0].Clock == MOSTCLOCK_48KHZ) || (Gateway_MOST[0].Clock == MOSTCLOCK_SPDIF))
  	{
    	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_DEVMODE_MASTER);
      if (sub_error) printf("** error inicCOnfig cmd(5): INIC_DEVMODE_MASTER\n");
  	}
  	else
  	{
      sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_DEVMODE_SLAVE);
      if (sub_error) printf("** error inicCOnfig cmd(5): INIC_DEVMODE_SLAVE\n");
  	}
  	error |= sub_error;

  	inicReadReqMsgs();

  	#if CP_OS81050
      // Set ClockMode ; OS81050 only
  	#if (MOST_PLL_CRYSTAL == 1)
    	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_CLK_MA_44K1);
      if (sub_error) printf("** error inicConfig cmd(6): INIC_CLK_MA_44K1\n");
  	#else
    	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_CLK_SL_44K1);
      if (sub_error) printf("** error inicConfig cmd(6): INIC_CLK_SL_44K1\n");
  	#endif
  	error |= sub_error;

  	inicReadReqMsgs();
  	#endif

  	// Set RMCK
  	sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_SET_RMCK);
  	if (sub_error) printf("** error inicConfig cmd(8): INIC_SET_RMCK\n");
  	error |= sub_error;

  	inicReadReqMsgs();

   if ((Gateway_MOST[0].Clock == MOSTCLOCK_48KHZ) || (Gateway_MOST[0].Clock == MOSTCLOCK_SPDIF))
   {
      // Inic NW-Startup
      sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_NWSTARTUP);
      if (sub_error) printf("** error inicConfig cmd(9): INIC_NWSTARTUP\n");
      error |= sub_error;

      inicReadReqMsgs();
   }

	timWait (50);


   #if INIC_SPI_CONFIF_MBC
	// Set hardwaredependant Socket-Commands for SPI
   sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_SOCKET1);
   if (sub_error) printf("** error inicConfig cmd(10): INIC_SOCKET1\n");
   else
      printf("     open INIC SPI-Port Mode 0  Int Treshold 0\n");
   error |= sub_error;

   inicReadReqMsgs();

   timWait (50);

   sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_SOCKET2);
   if (sub_error) printf("** error inicConfig cmd(11): INIC_SOCKET2\n");
   else
      printf("     create Socket Packet Data into Inic\n");
   error |= sub_error;

   inicReadReqMsgs();

   timWait (50);

   sub_error = inicSendAndWaitAck(inic_CtrlMsgBuf, INIC_SOCKET3);
   if (sub_error) printf("** error inicConfig cmd(12): INIC_SOCKET3\n");
   else
      printf("     create Socket Packet Data outoff Inic\n");
   error |= sub_error;

   inicReadReqMsgs();
   #endif



   error = inicSendWaitXCM (INIC_WAIT_ACK_ICM, INIC_GET_DEVMODE, inic_temp_ICM_msgbuf, 1000);
   inic_cfg_mode = inic_temp_ICM_msgbuf[10];

   printf("     detect device mode:");
   if (inic_cfg_mode)
      printf(" master\n");
   else
      printf(" slave\n");

   printf("     get own Network Adresses:");
   own_NodeAddr = inicGetAddr(INIC_GET_ADDR_LOGICAL);  // for testing on own address when sendeing in mostmsg
   own_NodePosAddr = inicGetAddr(INIC_GET_ADDR_PHYSICAL);  // for testing on own address when sendeing in mostmsg
   printf(" logical: %04x; physical: %04x\n", (UINT32)own_NodeAddr, (UINT32)own_NodePosAddr);

   error = inicSendWaitXCM (INIC_WAIT_ACK_ICM, INIC_GET_VERSION, inic_temp_ICM_msgbuf, 1000);
   inic_device = (inic_temp_ICM_msgbuf[12]<<16) + (inic_temp_ICM_msgbuf[13]<<8) + inic_temp_ICM_msgbuf[14];
   inic_hwversion =  inic_temp_ICM_msgbuf[16];  // get Hardware Release number, 0 = A...
   printf("     => INIC: OS%05x; HWVer: %c; FWVer:%2x.%02x.%02x; Conf:%2x.%02x.%02x; API:%2x.%02x.%02x; Date: %02x/%02x/%02x\n",
          inic_device, inic_hwversion -1 +'A',
          (UINT32)inic_temp_ICM_msgbuf[18],(UINT32)inic_temp_ICM_msgbuf[19],(UINT32)inic_temp_ICM_msgbuf[20],
          (UINT32)inic_temp_ICM_msgbuf[24],(UINT32)inic_temp_ICM_msgbuf[25],(UINT32)inic_temp_ICM_msgbuf[26],
          (UINT32)inic_temp_ICM_msgbuf[27],(UINT32)inic_temp_ICM_msgbuf[28],(UINT32)inic_temp_ICM_msgbuf[29],
          (UINT32)inic_temp_ICM_msgbuf[22],(UINT32)inic_temp_ICM_msgbuf[23],(UINT32)inic_temp_ICM_msgbuf[21]);

   if ((inic_device != 0x81050) && (inic_device != 0x81110))
   {
      printf("      unknown inic device!\n");
      error = ERR_INIC_STARTUP_FAILED;
   }

   // read rest of messages from inic to ringbuffer
   timSetMarker(TIMER_INIC);
   while(1)
   {
      if (timTimeout(TIMER_INIC, 20))
         break;

      if (newMsgReady())
      {
         timSetMarker(TIMER_INIC);                    // restart timeout
         inicProceedNewMsg();
      }
   }

   if (error == NO_ERROR)
      mostInitialized = TRUE;

   #if CP_CMDDEV_MOST
   #else
   if (error == NO_ERROR)
      printf("     Done.\n");
   else
      printf("     failed!\n");
   #endif

   return(error);
}


//-------------------------------------------------------------------------
// inicSendAndWaitAck
//-------------------------------------------------------------------------
// Function:
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

UINT32 inicSendAndWaitAck(inic_MsgBuf_t MsgBuf, UINT8 *buf)
{
UINT32 error;
UINT8  MsgType,AckType;
UINT8  retries;

   // detect MsgType from given buffer and set according wait bit
   MsgType = *(buf+3) & INIC_FPH_FIFONO_MASK;
   switch(MsgType)
   {
      case INIC_FPH_FIFONO_ICM:
         AckType = INIC_WAIT_ACK_ICM;         // set Ack wait bit
         break;

      case INIC_FPH_FIFONO_MCM:
         AckType = INIC_WAIT_ACK_MCM;         // set Ack wait bit
         break;

      case INIC_FPH_FIFONO_MDP:
         AckType = INIC_WAIT_ACK_MDP;         // set Ack wait bit
         break;

      case INIC_FPH_FIFONO_ALL:
         AckType = INIC_WAIT_ACK_ALL;         // set Ack wait bit
         break;
   }

   InicStatus |= AckType;         // set Ack wait bit

   #if CP_CMDDEV_MOST
   if (MsgBuf == inic_MostMsgBuf)
       InicStatus |= INIC_WAIT_ACK_MOSTMSG;           // to differ between X23-command and MostMsg
   #endif

   for (retries = 3; retries > 0 ; retries--)
   {
      // send given Msg to inic
      error = OS81xxx_Write_Str(INIC_I2C_ADDR, buf, readWord(buf)+2, 0);

      if (inicWaitForAck(AckType,100) != NO_ERROR)
      {
         // if no ack or answer is detected retry
         error = ERROR_INIC | ERR_INIC_IO;
      #if (INIC_DEBUG_ON == 1)
         count_InicRetries++;
      #endif
      }
      else
      {
         break;
      }
   }
   return(error);
}


//-------------------------------------------------------------------------
// inicInitBuffers
//-------------------------------------------------------------------------
// Function:  disable INIC interrupts
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
// Wolfgang Doczkal , EEHB, Tel. 07248 / 711564
//-------------------------------------------------------------------------

static BOOL inicInitBuffers(void)
{
   if (inicInBuffers[inic_CtrlMsgBuf] != NULL)
      bufFlush(inicInBuffers[inic_CtrlMsgBuf]);

   if (inicInBuffers[inic_MostMCMBuf] != NULL)
      bufFlush(inicInBuffers[inic_MostMCMBuf]);

   InicStatus = 0;
   inicMsgLostCnt = 0;           // Counter for lost inic-Messages
   inicMostMsgLostCnt = 0;
   inicMostMCMLostCnt = 0;
   return(TRUE);
}


//-------------------------------------------------------------------------
#if (!BIOSLOADER)
void interp_inicFlashCommand(UINT8 *buf, UINT8 num_wr, UINT8 num_rd)
{
   UINT32 i;
   UINT32 error;
   UINT8 i2c_buffer[256];

   error = inicFlashCommand(buf, num_wr, num_rd, i2c_buffer);

   if (error)
   {
      writeLong(resultPtr+4,error);
   }
   else
   {
      for(i=0; i<num_rd; i++)
         resultPtr[8U+i]=i2c_buffer[i];

      writeLong(resultPtr+4, error);
      resultPtr[0U]  = 8U+num_rd;
   }
   return;
}

//-------------------------------------------------------------------------
// inicFlashCommand
//-------------------------------------------------------------------------
// Function: send a I2C command to flash OS81050 an return OS81050 command answer
//           OS81050 must be started in Config-Mode!!
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
// G. Kern, , EEHB, Tel. 07248 / 71-3042
//-------------------------------------------------------------------------
UINT32   inicFlashCommand(UINT8 *in_buf, UINT8 num_wr, UINT8 num_rd, UINT8 *ret_buf)
{
   UINT32 error;

   if(inic_debug_msg_level>1)
      printf("** Inic: FlashCommand: cmd=0x%02x\n",(UINT32)in_buf[0]);

   if ((error = OS81xxx_Write_Str(INIC_I2C_ADDR, in_buf, num_wr, 0)) != NO_ERROR)
   {
      printf ("-- write failed: 0x%x\n", error);
      return(error);
   }

   timSetMarker(TIMER_INIC);

   while(!newMsgReady())
   {
      if (timTimeout(TIMER_INIC, 10000))
      {
         printf ("-- timeout\n");
         return(ERROR_INIC | ERR_INIC_CMD_TIMEOUT);
      }
   }

   // Read Flash message from command port
   error = sh4i2c2inic(OS81xxx_Read_Str (INIC_I2C_ADDR, NULL, 0, ret_buf, num_rd, 0));

   if (error != NO_ERROR)
   {
      printf ("-- read failed: 0x%x\n", error);
      return(error);
   }

   if(inic_debug_msg_level>1)
      printf("      num_rd:%d, ret_buf[0]:0x%02x\n", (UINT32)num_rd, (UINT32)ret_buf[0]);

   return(NO_ERROR);
}



UINT32 inicFlashIPF(UINT8 *buf, UINT32 num_wr)
{
#define IPF_MAIN_HEADER_LENGTH 0x06U
#define IPF_DATA_HEADER_LENGTH 0x0AU

   UINT8 block_num=0;
   UINT8 *header;
   UINT32 StartAdressFlash, StartAdressHostMem, ImageLength, num_rd;
   UINT32 error;

   header=buf;

   if(inic_debug_msg_level>0) printf("** Inic: FlashIPF\n");
   if(inic_debug_msg_level>0) printf("      Magicbyte: 0x%02x\n", (UINT32)header[0]);
   if(inic_debug_msg_level>0) printf("      ChipID:    0x%02x\n", (UINT32)header[1]);

if (header[1] == 0x0F)
   {
      current_fp = &os81050_fp;
   }
   else if (header[1] == 0x10)
   {
      current_fp = &os81110_fp;
   }
   else
   {
      printf(0,"   Error: Unknown ChipID found\n");
   }

   // check valid IPF file format
   num_rd=0;
   StartAdressHostMem=(UINT32)buf+IPF_MAIN_HEADER_LENGTH;
   header=(UINT8*)StartAdressHostMem;
   num_rd+=IPF_MAIN_HEADER_LENGTH;

   while(1)
   {
      StartAdressFlash = readLong(header+2);
      ImageLength  = readLong(header+6);
      StartAdressHostMem=(UINT32)buf+num_rd+IPF_DATA_HEADER_LENGTH;

      if (num_rd == num_wr)
      {
         break;  // All Blocks detected corrat; start program
      }

      if (num_rd > num_wr)
      {
         return(ERROR_INIC | ERR_INIC_BAD_IPF_FORMAT);
      }

      if(inic_debug_msg_level>0) printf("      DataBlockNr: %d\n", (UINT32)block_num);
      if(inic_debug_msg_level>0) printf("         ImageID:              0x%02x\n", (UINT32)header[0]);
      if(inic_debug_msg_level>0) printf("         StartAdress(Flash):   0x%08x\n", StartAdressFlash);
      if(inic_debug_msg_level>0) printf("         ImageLength:          0x%08x\n", ImageLength);
      if(inic_debug_msg_level>0) printf("         StartAdress(Mem):     0x%08x\n", StartAdressHostMem);

      num_rd+=IPF_DATA_HEADER_LENGTH+ImageLength;
      header=(UINT8*)((UINT32)buf+num_rd);
      block_num++;
   }

   if ((error = pfProgramStart()) != NO_ERROR) return(error);

   if ((error = pfEraseFlashEnable()) != NO_ERROR) return(error);

   // programm all blocks
   block_num=0;
   num_rd=0;
   StartAdressHostMem=(UINT32)buf+IPF_MAIN_HEADER_LENGTH;
   header=(UINT8*)StartAdressHostMem;
   num_rd+=IPF_MAIN_HEADER_LENGTH;

   while(1)
   {
      UINT8 image_type;
      StartAdressFlash = readLong(header+2);
      ImageLength  = readLong(header+6);
      StartAdressHostMem=(UINT32)buf+num_rd+IPF_DATA_HEADER_LENGTH;

      image_type=header[0];

      if (num_rd == num_wr)
      {
         return(NO_ERROR);  // All Blocks detected
      }

      if(inic_debug_msg_level>0) printf("      DataBlockNr: %d\n", (UINT32)block_num);
      switch(image_type)
      {
         case 1:
            error=inicFlashFirmware(StartAdressFlash, (UINT8*)StartAdressHostMem, ImageLength);
         break;
         case 2:
            error=inicFlashConfigString(StartAdressFlash, (UINT8*)StartAdressHostMem, ImageLength);
         break;
         default:
            printf("unknown image_type: %d\n", (UINT32)image_type);
            return(ERROR_INIC | ERR_INIC_BAD_IPF_FORMAT);
      }
      if (error != NO_ERROR)
         return error;

      if(inic_debug_msg_level>0) printf("      DataBlock finished\n");
      num_rd+=IPF_DATA_HEADER_LENGTH+ImageLength;
      header=(UINT8*)((UINT32)buf+num_rd);
      block_num++;
   }

}


UINT32 inicFlashFirmware(UINT32 p_adr, UINT8 *buf, UINT32 num_wr)
{
   UINT32 error;
   UINT8 compl_buffer[8];


   if(inic_debug_msg_level>1) printf("** Inic: FlashFirmware\n");
   if(inic_debug_msg_level>1) printf("      p_adr:0x%08x, num_wr:0x%08x\n",p_adr,num_wr);


   if(inic_debug_msg_level>0) printf("         Erasing...\n");


   // Erase INIC-Firmware and Persistent Memory
   if ((error = pfEraseFlashPartitions(current_fp->first_partition,current_fp->last_partition - current_fp->first_partition + 1U)) != NO_ERROR) return(error);


   if(inic_debug_msg_level>0) printf("            finished\n");

   if ((error = pfClearCRC()) != NO_ERROR) return(error);

   if(inic_debug_msg_level>0) printf("         Programming...\n");

   // check the Flash Pages to program
   if(p_adr & 0x10000U)
   {
      // Flash only the second half
      if ((error = pfSelFlashPage(1U)) != NO_ERROR) return(error);
      if ((error = pfWriteProgramMemory(p_adr, num_wr, buf)) != NO_ERROR) return(error);
   }
   else
      if((p_adr+num_wr) >= 0x10000U)
      {
         // Flash both halfes
         if(inic_debug_msg_level>0) printf("      Programming 1. Page\n");
         if(inic_debug_msg_level>1) printf("      p_adr:0x%08x,cnt:0x%08x\n", p_adr, 0x10000U - p_adr);
         if ((error = pfSelFlashPage(0U)) != NO_ERROR) return(error);
         if ((error = pfWriteProgramMemory(p_adr, 0x10000U - p_adr, buf)) != NO_ERROR) return(error);

         if(inic_debug_msg_level>0) printf("      Programming 2. Page\n");
         if(inic_debug_msg_level>1) printf("      p_adr:0x%08x,cnt:0x%08x\n", 0x10000U, num_wr -(0x10000U - p_adr));
         if ((error = pfSelFlashPage(1U)) != NO_ERROR) return(error);
         if ((error = pfWriteProgramMemory(0x10000U , num_wr -(0x10000U - p_adr), buf+(0x10000U-p_adr))) != NO_ERROR) return(error);
      }
      else
      {
         // Flash only the first half
         if ((error = pfSelFlashPage(0U)) != NO_ERROR) return(error);
         if ((error = pfWriteProgramMemory(p_adr, num_wr, buf)) != NO_ERROR) return(error);
      }

   if(inic_debug_msg_level>0) printf("            finished\n");

   if ((error = pfGetCRC(compl_buffer)) != NO_ERROR) return(error);

   if(!( (compl_buffer[0]==0xffU) && (compl_buffer[4]==0x00U) && (compl_buffer[5]==0x00U)))
   {
      if(inic_debug_msg_level>1) printf("      compl_buffer[0]:0x%02x\n",(UINT32)compl_buffer[0]);
      if(inic_debug_msg_level>1) printf("      compl_buffer[4]:0x%02x\n",(UINT32)compl_buffer[4]);
      if(inic_debug_msg_level>1) printf("      compl_buffer[5]:0x%02x\n",(UINT32)compl_buffer[5]);
      return(ERROR_INIC | ERR_INIC_BAD_FLASH_CRC);
   }

   if(inic_debug_msg_level>0) printf("         CRC o.k.\n");

   return(NO_ERROR);
}


UINT32 inicFlashConfigString(UINT32 p_adr, UINT8 *buf, UINT32 num_wr)
{
   UINT32 error;
   UINT8 compl_buffer[8];


   if(inic_debug_msg_level>1) printf("** Inic: FlashConfigString\n");
   if(inic_debug_msg_level>1) printf("      p_adr:0x%08x, num_wr:0x%08x\n",p_adr,num_wr);

   if(inic_debug_msg_level>0) printf("         Erasing...\n");

   if ((error = pfEraseInfoBlock()) != NO_ERROR) return(error);

   if(inic_debug_msg_level>0) printf("            finished\n");

   if ((error = pfClearCRC()) != NO_ERROR) return(error);

   if(inic_debug_msg_level>0) printf("         Programming...\n");

   if ((error = pfWriteInfoBlock(p_adr, num_wr, buf)) != NO_ERROR) return(error);

   if(inic_debug_msg_level>0) printf("            finished\n");

   if ((error = pfGetCRC(compl_buffer)) != NO_ERROR) return(error);

   if(!( (compl_buffer[0]==0xffU) && (compl_buffer[4]==0x00U) && (compl_buffer[5]==0x00U)))
   {
      if(inic_debug_msg_level>1) printf("      compl_buffer[0]:0x%02x\n",(UINT32)compl_buffer[0]);
      if(inic_debug_msg_level>1) printf("      compl_buffer[4]:0x%02x\n",(UINT32)compl_buffer[4]);
      if(inic_debug_msg_level>1) printf("      compl_buffer[5]:0x%02x\n",(UINT32)compl_buffer[5]);
      return(ERROR_INIC | ERR_INIC_BAD_FLASH_CRC);
   }

   if(inic_debug_msg_level>0) printf("         CRC o.k.\n");

   return(NO_ERROR);
}


UINT32 pfProgramStart(void)
{
   UINT32 error;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   i2c_buffer[0] = 0x0d;
   i2c_buffer[1] = 0x00;
   i2c_buffer[2] = 0x00;
   i2c_buffer[3] = 0x00;

   error = inicFlashCommand(i2c_buffer, 4, 1, compl_buffer);

   return(error);
}


UINT32 pfEraseFlashEnable(void)
{
   UINT32 error=NO_ERROR;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   if(inic_debug_msg_level>1) printf("** inicFlashCommand: pfEraseFlashEnable\n");

   i2c_buffer[0] = 0x0f;
   i2c_buffer[1] = 0x00;
   #if CP_OS81050
   i2c_buffer[2] = 0x02;
   #endif
   #if CP_OS81110
   i2c_buffer[2] = 0x07;
   #endif
   i2c_buffer[3] = 0x00;
   i2c_buffer[4] = 0x00;

   error = inicFlashCommand(i2c_buffer, 5, 1, compl_buffer);

   return(error);
}


UINT32 pfEraseFlashPartitions(UINT8 startP, UINT8 numP)
{
   UINT32 error=NO_ERROR;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   if(inic_debug_msg_level>1) printf("** inicFlashCommand: pfEraseFlashPartitions\n");

   i2c_buffer[0] = 0x0c;
   i2c_buffer[1] = startP;
   i2c_buffer[2] = numP;
   i2c_buffer[3] = 0x00;

   error = inicFlashCommand(i2c_buffer, 4, 1, compl_buffer);
   if(inic_debug_msg_level>1) printf("      startP:0x%02x,numP:0x%02x\n",(UINT32)startP,(UINT32)numP);

   return(error);
}


UINT32 pfWriteProgramMemory(UINT32 adress, UINT32 len, UINT8 *pdata)
{
   UINT32 error=NO_ERROR, i;
   UINT8 i2c_buffer[40];
   UINT8 compl_buffer[40];
   UINT32 l_adr, adr_offset;

   if(inic_debug_msg_level>1) printf("** inicFlashCommand: pfWriteProgramMemory\n");
   if(inic_debug_msg_level>1) printf("      adress:0x%08x,len:0x%08x,pdata:0x%08x\n",adress,len,(UINT32)pdata);
   adr_offset=0;
   while(len > 0 && error == NO_ERROR)
   {
       l_adr = adress+adr_offset;
       if(len>32)
       {
          i2c_buffer[0] = 0x01;
          i2c_buffer[1] = (UINT8)(l_adr>>8);
          i2c_buffer[2] = (UINT8)l_adr;
          i2c_buffer[3] = 32;
          for(i=0; i<32; i++)
             i2c_buffer[4+i]=pdata[adr_offset+i];
          len-=32;
          error = inicFlashCommand(i2c_buffer, 36, 1, compl_buffer);
          if(inic_debug_msg_level>1) printf("      pdata[adr_offset]:0x%02x,adr_offset:0x%08x,len:%d\n",(UINT32)pdata[adr_offset],adr_offset,32);
          if(inic_debug_msg_level>1) printf("      i2c_buffer[0]:0x%02x,l_adr:0x%08x\n",(UINT32)i2c_buffer[0],l_adr);
          adr_offset += 32;
       }
       else
       {
          i2c_buffer[0] = 0x01;
          i2c_buffer[1] = (UINT8)(l_adr>>8);
          i2c_buffer[2] = (UINT8)l_adr;
          i2c_buffer[3] = len;
          for(i=0; i<len; i++)
             i2c_buffer[4+i]=pdata[adr_offset+i];
          error = inicFlashCommand(i2c_buffer, 4+len, 1, compl_buffer);
          if(inic_debug_msg_level>1) printf("      pdata[adr_offset]:0x%02x,adr_offset:0x%08x,len:%d\n",(UINT32)pdata[adr_offset],adr_offset,len);
          if(inic_debug_msg_level>1) printf("      i2c_buffer[0]:0x%02x,l_adr:0x%08x\n",(UINT32)i2c_buffer[0],l_adr);
          len=0;
       }

   }
   return(error);
}


UINT32 pfSelFlashPage(UINT8 page)
{
   UINT32 error;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   i2c_buffer[0] = 0x06;
   i2c_buffer[1] = page;

   error = inicFlashCommand(i2c_buffer, 2, 1, compl_buffer);

   return(error);
}

UINT32 pfEraseInfoBlock(void)
{
   UINT32 error;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   i2c_buffer[0] = 0xcc;
   i2c_buffer[1] = 0x67;

   error = inicFlashCommand(i2c_buffer, 2, 1, compl_buffer);

   return(error);
}

UINT32 pfWriteInfoBlock(UINT8 adress, UINT8 len, UINT8 *pdata)
{
   UINT32 error=NO_ERROR, i;
   UINT8 i2c_buffer[40];
   UINT8 compl_buffer[40];
   UINT32 l_adr, adr_offset;

   if(inic_debug_msg_level>1) printf("** inicFlashCommand: pfWriteInfoBlock\n");
   if(inic_debug_msg_level>1) printf("      adress:0x%08x,len:0x%08x,pdata:0x%08x\n",(UINT32)adress,(UINT32)len,(UINT32)pdata);
   adr_offset=0;
   while(len > 0)
   {
       l_adr = adress+adr_offset;
       if(len>32)
       {
          i2c_buffer[0] = 0xc1;
          i2c_buffer[1] = 0x00;
          i2c_buffer[2] = (UINT8)l_adr;
          i2c_buffer[3] = 32;
          for(i=0; i<32; i++)
             i2c_buffer[4+i]=pdata[adr_offset+i];
          len-=32;
          error = inicFlashCommand(i2c_buffer, 36, 1, compl_buffer);
          if(inic_debug_msg_level>1) printf("      pdata[adr_offset]:0x%02x,adr_offset:0x%08x,len:%d\n",(UINT32)pdata[adr_offset],adr_offset,32);
          if(inic_debug_msg_level>1) printf("      i2c_buffer[0]:0x%02x,l_adr:0x%08x\n",(UINT32)i2c_buffer[0],l_adr);
          adr_offset += 32;
       }
       else
       {
          i2c_buffer[0] = 0xc1;
          i2c_buffer[1] = 0x00;
          i2c_buffer[2] = (UINT8)l_adr;
          i2c_buffer[3] = len;
          for(i=0; i<len; i++)
             i2c_buffer[4+i]=pdata[adr_offset+i];
          error = inicFlashCommand(i2c_buffer, 4+len, 1, compl_buffer);
          if(inic_debug_msg_level>1) printf("      pdata[adr_offset]:0x%02x,adr_offset:0x%08x,len:%d\n",(UINT32)pdata[adr_offset],adr_offset,(UINT32)len);
          if(inic_debug_msg_level>1) printf("      i2c_buffer[0]:0x%02x,l_adr:0x%08x\n",(UINT32)i2c_buffer[0],l_adr);
          len=0;
       }
   }
   return(error);

}

UINT32 pfClearCRC(void)
{
   UINT32 error;
   UINT8 i2c_buffer[8];
   UINT8 compl_buffer[8];

   i2c_buffer[0] = 0xe5;
   i2c_buffer[1] = 0x00;
   i2c_buffer[2] = 0x00;
   i2c_buffer[3] = 0x00;

   error = inicFlashCommand(i2c_buffer, 4, 1, compl_buffer);

   return(error);
}


UINT32 pfGetCRC(UINT8 *crcdata)
{
   UINT32 error;
   UINT8 i2c_buffer[8];

   i2c_buffer[0] = 0xe6;
   i2c_buffer[1] = 0x00;
   i2c_buffer[2] = 0x00;
   i2c_buffer[3] = 0x02;

   error = inicFlashCommand(i2c_buffer, 4, 6, crcdata);

   return(error);
}
#endif



UINT32 inicSendWaitXCM (UINT8 mode, const UINT8 *cmd, UINT8 *buf, UINT32 Timeout)
{
   volatile BOOL *inic_ResponseTrigger;
   UINT32 error = NO_ERROR;
//#define INIC_XCM_DEBUG
#ifdef INIC_XCM_DEBUG
   unsigned i;
#endif

   if (cmd == commandPtr)  // interpret as HBBIOS command
   {
      //printf("bioscommand\n");

      // Inic PML
      buf[0] = 0;
      buf[1] = cmd[0];
      // Inic PMHL
      switch (mode)
      {
         case INIC_WAIT_ACK_ICM:
            buf[2] = 0x01;
            // Inic FPH
            buf[3] = 0x14;
            // Inic FktID and Inic OpType
            memcpy(&buf[4], &cmd[4], 2);
            buf[6] = 0;
            buf[7] = cmd[0]-6;
            // parameters, if any
            memcpy(&buf[8], &cmd[6], ((buf[6]<<8)+buf[7]));
            break;

         case INIC_WAIT_ACK_MCM:
            buf[2] = 0x05;
            // Inic FPH
            buf[3] = 0x04;
            // Inic TargetDevID, Functionaddress, FktID and Inic OpType
            memcpy(&buf[4], &cmd[4], 6);
            buf[10] = 0;
            buf[11] = cmd[0]-10;
            // parameters, if any
            memcpy(&buf[12], &cmd[10], ((buf[10]<<8)+buf[11]));
            break;
      }
   } else
      memcpy(buf, cmd, ((cmd[0] << 8) + cmd[1])+2);

   // at this point "buf" should contain the message in the same format, as stored in the command constants (e.g. INIC_GET_VERSION)
#ifdef INIC_XCM_DEBUG
   printf("c:");
   for (i=0; i<buf[1]+2; i++)
      printf("%02x ",(UINT32)buf[i]);
   printf(" => 0x%08x\n", error);
#endif

   // now "buf" will be converted to a valid port message
   switch (mode)
   {
      case INIC_WAIT_ACK_ICM:
         inic_ResponseTrigger = &inic_ICM_ResponseTrigger;
         inic_ICM_FktID = (buf [4] << 4) + (buf [5] >>4);
         break;
      case INIC_WAIT_ACK_MCM:
         inic_ResponseTrigger = &inic_MCM_ResponseTrigger;
         inic_MCM_FktID = (buf [8] << 4) + (buf [9] >>4);
         break;
      default:
         return(ERROR_INIC | ERR_INIC_PARAM_INVALID);
   }
   *inic_ResponseTrigger = true;

   #if CP_CMDDEV_MOST
   // MCM-ACK-MSG-Filter ON
   // needed to differ between MCM for Command and HBBIOS-Communication
   if ((msgGetCurrentInterface() == ifMOST) && (MostMsgRunning()))
   {
      InicStatus &= ~INIC_WAIT_ACK_MOSTMSG;
   }
   #endif

   InicStatus |= mode;

   // here you should log the port message
#ifdef INIC_XCM_DEBUG
   printf("p:");
   for (i=0; i<buf[1]+2; i++)
      printf("%02x ",(UINT32)buf[i]);
   printf(" => 0x%08x\n", error);
#endif
   error =  OS81xxx_Write_Str(INIC_I2C_ADDR, buf, readWord(buf)+2, 0);
   if (!error)
      error = inicWaitForAck(mode,INIC_ACK_TIMEOUT);

   #if CP_CMDDEV_MOST
   // MCM-ACK-MSG-Filter OFF
   if ((msgGetCurrentInterface() == ifMOST) && (MostMsgRunning()))
   {
      InicStatus |= INIC_WAIT_ACK_MOSTMSG;
   }
   #endif

   if (error)
      return(error);

   timSetMarker(TIMER_INIC);
   while(*inic_ResponseTrigger)
   {
      if (newMsgReady())
      {
            inicProceedNewMsg();
      }

      if (timTimeout(TIMER_INIC, Timeout))
      {
         error = (ERROR_INIC | ERR_INIC_ICM_TIMEOUT);
			*inic_ResponseTrigger = false;
         return(error);
      }
   }

   // with some luck, you'll find the answer again in "buf"
#ifdef INIC_XCM_DEBUG
   printf("r:");
   for (i=0; i<buf[0]; i++)
      printf("%02x ",(UINT32)buf[i]);
   printf(" => 0x%08x\n", error);
#endif
   return(error);
}

//-------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// inicCommand (void)
//-------------------------------------------------------------------------
// Function: perform tests with MOST INIC device
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
#if (!BIOSLOADER)
void  inicCommand(void)
{
//   TInicPortMessage  *aInicMsg;
   UINT8             aFPH;
   UINT16            ii, aPml  = 0;
   UINT16            BytesLeft = 0;
   UINT32            len = 0;
   UINT32            error;
   UINT32            temp;
   UINT16            NewUMostTrgtAddr;

   // Clear error memory
   error = NO_ERROR;
   temp = 0;
   NewUMostTrgtAddr = 0;
   // Select INIC test
   switch(commandPtr[3U])
   {
      case 0xFFU:
         writeLong(resultPtr+4, NO_ERROR);
         break;


      // *****************************************
      // * inic init commands                    *
      // *****************************************
      // X 23 00 ...
      case 0x00U: // Perform INIC hardware reset

         switch (commandPtr[4])
         {
         // X 23 00 00
         // InicHwInit = complete Init Of all elements
         case 0x00U:
            if (commandPtr[0] == 5)
            {
               printf("** Inic: execute HwInitStartup...\n");
               error = inicHwInit(INIC_ACCESS_I2C);
            }
            else if (commandPtr[0] == 6)
            {
               // X 23 00
               printf("** Inic: execute HwInit(Mode: 0x%02X)...\n",(UINT32)commandPtr[5] );
               error = inicHwInit(commandPtr[5]);
            }
            else
            {
                error = (ERROR_INIC | ERR_INIC_COMMAND);                 // on syntax error leave command
            }
         break;

         // X 23 00 01
         // inic ReInit
         case 0x01U:
            printf("** Inic: execute ReInit...\n");
            error = inicReInit();
               break;

         // X 23 00 02
         // Flush Inic Buffers
         case 0x02U:
            printf("** Inic: execute BufferInit...\n");
           // BufferInit has always to be done after Inic-Init-Functionality
           if (inicInitBuffers() == FALSE)
           {
              error = (ERROR_INIC | ERR_INIC_NOTINIT);
           }
               break;

         // X 23 00 03
         // read all pending messages from inic
         case 0x03U:
               temp = inicMsgLostCnt;
               timSetMarker(TIMER_INIC);
            while (1)
            {
               while (newMsgReady())
               {
                  timSetMarker(TIMER_INIC);
                  error = inicProceedNewMsg();
                  if (error)
                  {
                     break;
                  }
               }
               if (timTimeout(TIMER_INIC, 200))
               {
                  break;
               }
            }
            if (temp != inicMsgLostCnt)
            {
               temp = inicMsgLostCnt - temp;
               printf("** Inic: Warning! InputOverun: %d Messages lost!\n", temp);
            }
           break;

            // -------------------------------------------------
            // modify target address for communication to uMost
            //--------------------------------------------------
            // X 23 00 04 [UINT16 Addr]
            // set new Trgt address (Addr of uMost Board)
            case 0x04U:
               if (commandPtr[0] != 7)
               {
                   error = (ERROR_INIC | ERR_INIC_COMMAND);            // on syntax error leave command
               }
               else
               {
                   printf("   Set new TrgtAddr for communication to uMost\n");
                   //inicUMostTrgtAddr =  ((commandPtr[5] << 8) + commandPtr[6]);
                    NewUMostTrgtAddr =  ((commandPtr[5] << 8) + commandPtr[6]);


                   if (((NewUMostTrgtAddr >= 0x0010U) && (NewUMostTrgtAddr <= 0x02FFU))
                      || ((NewUMostTrgtAddr >= 0x0500U) && (NewUMostTrgtAddr <= 0x0FEFU))
                      || (NewUMostTrgtAddr == 0x0000U))
                        {

                           inicUMostTrgtAddr = NewUMostTrgtAddr;
                           printf("   set to 0x%04X\n", (UINT32)inicUMostTrgtAddr);
                           if (inicUMostTrgtAddr == 0x0000U)
                           {
                              printf("  set to use default from config: 0x%04X \n",(UINT32)Gateway_MOST[0].TargetNodeadress);
                           }
                        }
                        else
                        {
                           printf("   value 0x%04X given vas invalid\n", (UINT32)inicUMostTrgtAddr);
                           printf("   valid range is: 0x0010...0x02FF\n");
                           printf("              and: 0x0500...0x0FEF\n");
                           printf("            value: 0x0000 resets to use defaults: 0x%04X\n",(UINT32)Gateway_MOST[0].TargetNodeadress);
                           error = (ERROR_INIC | ERR_INIC_PARAM_INVALID);            // on syntax error leave command
                        }
               }
               break;
          }
         writeLong(resultPtr+4, error);
         break;

      // *****************************************
      // * inic send commands                    *
      // *****************************************
      // X 23 01 ...
      case 0x01U:
         // Syntax:
         // X 23 01 lenCmd InicCommand-String

         if (inicHwInitialized == FALSE)
         {
             writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_NOTINIT);
             break;
         }
         // Extract payload size from command
         aPml = readWord(commandPtr+4);

         if (commandPtr[0] != ((UINT8)(aPml+6)))
         {
            printf("   error with command length\n");
            writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_COMMAND);  // on syntax error leave command
            break;
         }

         for (ii = 0; ii < aPml; ii++)
         {
            mg_InicCommand[ii] = commandPtr[6U+ii];
         }

         //printf("    Sending INIC command...\n");

         // *************************************************
         // * detect acknowledge type to wait for           *
         // *************************************************
         aFPH = mg_InicCommand[3] & INIC_FPH_FIFONO_MASK;

         switch (aFPH)
         {
            case INIC_FPH_FIFONO_MCM:
               // printf("Send MCM ...00...\n");
               // MCM will be send to inic
      #if CP_CMDDEV_MOST
               // !!!!!!!!!!!!!!!!!!
               #if 0
               if ((Gateway_Destination == MOST) && (MostMsgRunning()))
               {
                  InicStatus &= ~INIC_WAIT_ACK_MOSTMSG;   //DEBUG_MOSTMSG
               }
               #endif
      #endif
               InicStatus |= INIC_WAIT_ACK_MCM;
               error =  OS81xxx_Write_Str(INIC_I2C_ADDR, mg_InicCommand, readWord(mg_InicCommand)+2, 0);
               if (!error)
               {
                  error = inicWaitForAck(INIC_WAIT_ACK_MCM,INIC_ACK_TIMEOUT);
               }
      #if CP_CMDDEV_MOST
               // !!!!!!!!!!!!!!!!!!
               #if 0
               if ((Gateway_Destination == MOST) && (MostMsgRunning()))
               {
                  InicStatus |= INIC_WAIT_ACK_MOSTMSG;   //DEBUG_MOSTMSG
               }
               #endif
      #endif
               break;

            case INIC_FPH_FIFONO_MDP:
               // printf("Send MDP ...01...\n");
               // MDP will be send to inic
               InicStatus |= INIC_WAIT_ACK_MDP;
               error =  OS81xxx_Write_Str(INIC_I2C_ADDR, mg_InicCommand, readWord(mg_InicCommand)+2, 0);
               if (!error)
               {
                  error = inicWaitForAck(INIC_WAIT_ACK_MDP,INIC_ACK_TIMEOUT);
               }
               break;

            case INIC_FPH_FIFONO_ICM:
               // printf("Send ICM ...10...\n");
               // ICM will be send to inic
               InicStatus |= INIC_WAIT_ACK_ICM;
               error =  OS81xxx_Write_Str(INIC_I2C_ADDR, mg_InicCommand, readWord(mg_InicCommand)+2, 0);
               if (!error)
               {
                  error = inicWaitForAck(INIC_WAIT_ACK_ICM,INIC_ACK_TIMEOUT);
               }
               break;

            case INIC_FPH_FIFONO_ALL:
               // printf("Send ALL ...11...\n");
               // ICM will be send to inic
               InicStatus |= INIC_WAIT_ACK_ALL;
               error =  OS81xxx_Write_Str(INIC_I2C_ADDR, mg_InicCommand, readWord(mg_InicCommand)+2, 0);
               if (!error)
               {
                  error = inicWaitForAck(INIC_WAIT_ACK_ALL,INIC_ACK_TIMEOUT);
               }
               break;

            default:
               break;
         }

         writeLong(resultPtr+4, error);
         break;


         // *****************************************
         // * inic send ICM commands and get answer *
         // *****************************************
         // Open INIC Streaming Port SCK/FSY Out Mode Delay64FS16Bit
         // X 23 01    000C   000A  01 14  400  2  0  004     03 00 FF 00    // alt
         // X 23 11                        400  2             03 00 FF 00    // neu
         //            |      |     |  |   |    |  |  |       |
         // Bios Länge_|      |     |  |   |    |  |  |       |
         // Inic PML  ________|     |  |   |    |  |  |       |
         // Inic PMHL_______________|  |   |    |  |  |       |
         // Inic FPH___________________|   |    |  |  |       |
         // Inic FktID_____________________|    |  |  |       |
         // Inic OpType_________________________|  |  |       |
         // Inic TelID_____________________________|  |       |
         // Inic Length_______________________________|       |
         // Inic Data 0..n____________________________________|

         // X 23 11 ...
         case 0x11U:
            // Syntax:
            // X 23 11 FktID [3 Nibble]  OpType [1 Nibble] Data[0..n Bytes]
            if (inicHwInitialized == false)
            {
                set_result(ERROR_INIC | ERR_INIC_NOTINIT);
                break;
            }

            // use temporary buffer for ICM to inic
            // same buffer will return the reply captured in bioscontrol format

            error = inicSendWaitXCM (INIC_WAIT_ACK_ICM, commandPtr, inic_temp_ICM_msgbuf, 1000);

            //
            // inic_temp_ICM_msgbuf now contains answer in BiosControl-Format
            //

            // copy FktId Error into Errorcode to force marking of ICM-Result error in output
            if (!error)
            {
               memcpy((void *)&resultPtr[8], &inic_temp_ICM_msgbuf[8], inic_temp_ICM_msgbuf[0]-8);

               // if ((inic_temp_ICM_msgbuf[9] & 0x0F) == 0x0F)
               //    error = ERROR_INIC | ERR_INIC_ICM_FKTID;
               len = inic_temp_ICM_msgbuf[0]-8;
            }

            set_result_len (error, len);
            break;


         // *****************************************
         // * inic send MCM commands and get answer *
         // *****************************************
         // INIC.OpenPort.StartResult.StreamPort.Output.Seq256Fs
         // X 23 01    0010   000E  05 04 0401 0000 400 2 0 004 0300000D    // alt
         // X 23 12                       0401 0000 400 2       0300000D    // neu
         //            |      |     |  |  |    |    |   | | |   |
         // Bios Länge_|      |     |  |  |    |    |   | | |   |
         // Inic PML  ________|     |  |  |    |    |   | | |   |
         // Inic PMHL_______________|  |  |    |    |   | | |   |
         // Inic FPH___________________|  |    |    |   | | |   |
         // Inic FDH TgtDevID_____________|    |    |   | | |   |
         // Inic FDH FunAdr____________________|    |   | | |   |
         // Inic FktID______________________________|   | | |   |
         // Inic OpType_________________________________| | |   |
         // Inic TelID____________________________________| |   |
         // Inic Length_____________________________________|   |
         // Inic Data 0..n______________________________________|

         // X 23 12 ...
         case 0x12U:
            // Syntax:
            // X 23 12 TgtDevID[2 Byte] FunAdr [2 Byte] FktID [3 Nibble]  OpType [1 Nibble] Data[0..n Bytes]
            if (inicHwInitialized == false)
            {
                set_result(ERROR_INIC | ERR_INIC_NOTINIT);
                break;
            }

            // check BIOS  (UMOST CONTROLL MESSAGE)
            if (inic_Capture_BIOS_Request())
               break;

            // use temporary buffer for ICM to inic
            // same buffer will return the reply captured in bioscontrol format

            // prepare response trigger
            // write result to this buffer (bioscontrol command format including length byte)
            if (readWord(&commandPtr[4]) != BROADCAST_ADDR)
            {
               error = inicSendWaitXCM (INIC_WAIT_ACK_MCM, commandPtr, inic_temp_MCM_msgbuf, INIC_XCM_RESPONSE_TIMEOUT);

               //
               // inic_temp_MCM_msgbuf now contains answer in BiosControl-Format
               //

               // copy FktId Error into Errorcode to force marking of ICM-Result error in output
               if (!error)
               {
                  memcpy((void *)&resultPtr[8], &inic_temp_MCM_msgbuf[8], inic_temp_MCM_msgbuf[0]-6);

                  // if ((inic_temp_MCM_msgbuf[14] & 0x0F) == 0x0F)
                  //    error = ERROR_INIC | ERR_INIC_ICM_FKTID;

                  len = inic_temp_MCM_msgbuf[0]-8;
               }
            }
            else
            {
               inicSendWaitXCM (INIC_WAIT_ACK_MCM, commandPtr, inic_temp_MCM_msgbuf, 0);
            }

            set_result_len (error, len);
            break;

         // *****************************************
         // * read MCM message from InicInBuffer    *
         // *****************************************
         // X 23 13
         case 0x13U:
               // Command: X 23 13
               // Answer:  X 23 13 Error BytesLeft MOST-Message

               if (commandPtr[0] != 4)
               {
                  writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_COMMAND);
                  break;
               }

               aPml = inicMsgRead(inic_MostMCMBuf, (UINT8*)&resultPtr[10U]);
               if (inicHwInitialized == FALSE)
               {
                   writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_NOTINIT);
                   break;
               }

               if (aPml > 4)
               {
                  // cut PML, PMHL,FPH
                  memmove(&resultPtr[10U], &resultPtr[14U], aPml-2);
                  aPml -=4;
               }

               len = bufItems(inicInBuffers[inic_MostMCMBuf]);
               if (len > 0x0000FFFF)
               {
                     writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_PROT);
                     break;
               }
               BytesLeft =  (UINT16)(len & 0x0000FFFF);
               writeWord(resultPtr+8, BytesLeft);
               writeLong(resultPtr+4, error);
               resultPtr[0] =  10U + aPml;

            break;

      // *****************************************
      // * read message from InicInBuffer        *
      // *****************************************
      // X 23 02
      case 0x02U:
      #if CP_CMDDEV_MOST
               // !!!!!!!!!!!!!!!!!!
               #if 0
               if ((Gateway_Destination == MOST) && (MostMsgRunning()))
               {
                  InicStatus |= INIC_WAIT_ACK_MOSTMSG;   //DEBUG_MOSTMSG
               }
               #endif
      #endif


            // Command: X 23 02
            // Answer:  X 23 02 Error BytesLeft MOST-Message

            if (commandPtr[0] != 4)
            {
               writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_COMMAND);
               break;
            }

            aPml = inicMsgRead(inic_CtrlMsgBuf, (UINT8*)&resultPtr[10U]);
            if (inicHwInitialized == FALSE)
            {
                writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_NOTINIT);
                break;
            }

            if (aPml == 0)
            {
               writeWord(resultPtr+10, aPml);
               // add 2 byte in length to get always BytesLeft and MsgLength
               aPml +=2;
            }
            len = bufItems(inicInBuffers[inic_CtrlMsgBuf]);
            if (len > 0x0000FFFF)
            {
                  writeLong(resultPtr+4, ERROR_INIC | ERR_INIC_PROT);
                  break;
            }
            BytesLeft =  (UINT16)(len & 0x0000FFFF);
            writeWord(resultPtr+8, BytesLeft);
            writeLong(resultPtr+4, error);
            resultPtr[0] =  10U + aPml;

         break;

         // ************************************************
         // * call subinterpreter for inicFlashCommand     *
         // ************************************************
         case 0x04U:
            interp_inicFlashCommand(
                //commandPtr[4U],           // bus number
                //commandPtr[7U],           // slave address
                (UINT8 *)&commandPtr[8U], // data buffer
                commandPtr[5] - 1U,       // number of bytes to write
                commandPtr[6]);           // number of bytes to read
            break;

         case 0x05U:
            inic_debug_msg_level = commandPtr[12U];
            error = inicFlashIPF((UINT8 *)readLong(commandPtr+4), readLong(commandPtr+8));
            writeLong(resultPtr+4, error);
            inic_debug_msg_level = 0;
            break;

       // ************************************************
       // * print out test/debug values  (for test only) *
       // ************************************************
       case 0x0FU: // default Configuration
            // Reset INIC device
            if (commandPtr[0] != 5)
            {
               error = (ERROR_INIC | ERR_INIC_COMMAND);
            }
            switch(commandPtr[4U])
            {
               case 0x00: // IInic default configuration
                  printf("   proceed inic default configuration \n");
                  error = inicConfig();
                  break;
               case 0x01: // Inic Status Info
                  inicGetInfo();
                  break;
               case 0x02:
                  error = inicProceedNewMsg();

                  if (error != NO_ERROR)
                  {
                     printf("   error: inic not initialized\n");
                  }
                  else
                  {
                     printf("   inicProceedNewMsg done\n");
                  }
                  break;
               default:
                  error = ERROR_INTERPRT | ERROR_COMMAND;
                  break;
            }

            writeLong(resultPtr+4, error);
            break;

      default:
         writeLong(resultPtr+4, ERROR_INTERPRT | ERROR_COMMAND);
         return;
   }

}
#endif

//-------------------------------------------------------------------------
// inicGetAddr (UINT8 AddrMode)
//-------------------------------------------------------------------------
// Function: read NodeAddress of INIC (OS81050)
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
#if (!BIOSLOADER)
static UINT16 inicGetAddr(UINT8 AddrMode)
{
   UINT16   aNodeAddr;
   UINT32   error;

   if (AddrMode == INIC_GET_ADDR_PHYSICAL)
   {
      error = inicSendWaitXCM (INIC_WAIT_ACK_MCM, INIC_GET_PHYS_NODEADDR, inic_temp_MCM_msgbuf, 2000);
   }
   else
   {
      error = inicSendWaitXCM (INIC_WAIT_ACK_MCM, INIC_GET_NODEADDR, inic_temp_MCM_msgbuf, 2000);
   }

   if (error != NO_ERROR)
   {
     printf("      No address could be retrieved from INIC.\n");
     return(0);
   }

   aNodeAddr = inic_temp_MCM_msgbuf[15U];
   aNodeAddr = (aNodeAddr<<8) + inic_temp_MCM_msgbuf[16U];

   return aNodeAddr;

}
#endif

//-------------------------------------------------------------------------
// inicNWStartup
//-------------------------------------------------------------------------
// Function: Send ICM NW Startup to INIC (OS81050)
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
UINT32 inicNWStartup(void)
{
UINT32 error = NO_ERROR;

   if ((Gateway_MOST[0].Clock == MOSTCLOCK_48KHZ) || (Gateway_MOST[0].Clock == MOSTCLOCK_SPDIF))
   {
      // Inic NW-Startup
      error = inicSendWaitXCM (INIC_WAIT_ACK_ICM, INIC_NWSTARTUP, inic_temp_ICM_msgbuf, 1000);

      // test OPtype on Error
      if (error == NO_ERROR)
         if ((inic_temp_ICM_msgbuf[9] & 0x0F) != 0x0C)
            error = ERROR_INIC | ERR_INIC_ICM_TIMEOUT;
   }
   else
      error = ERROR_INIC | ERR_INIC_PARAM_INVALID;

   return(error);
}

//-------------------------------------------------------------------------
// inicGetInfo (void)
//-------------------------------------------------------------------------
// Function: printout testcounters and address-defines from config
//-------------------------------------------------------------------------
// Input:   none
//
// Output:  none
//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
#if (!BIOSLOADER)
static UINT32 inicGetInfo()
{
   UINT16   aNodeAddr;

#if (INIC_DEBUG_ON == 1)
  trace_active = 0;
#endif
   printf("  \n");
   printf("** Inic Status Info:\n");
   if (inicHwInitialized == FALSE)
   {
      printf("   No inicHwInit done at startup\n");
#if (INIC_DEBUG_ON == 1)
      trace_active = 1;
#endif
      return ERROR_INIC | ERR_INIC_NOTINIT;
   }

#if CP_CMDDEV_MOST
      printf("   SW_MOST_CMD-Mode activated\n");
#else
      printf("   BIOS-Mode activated\n");
#endif
   printf("   I2C communication selected\n");

   if (mostInitialized == FALSE)
   {
      printf("   No inicConfig done at startup\n");
      return NO_ERROR;
   }
      printf(" * Address settings for communication to uMost:\n");


      if (Gateway_MOST[0].TargetNodeadress == 0)
         printf("   TrgtAddr set by last RxMsg:   0x%04X\n",(UINT32)inicUMostTrgtAddr);
      else
         printf("   fixed TrgtAddr used:          0x%04X\n",(UINT32)Gateway_MOST[0].TargetNodeadress);



   aNodeAddr = inicGetAddr(INIC_GET_ADDR_LOGICAL);    // read NodeAddr from INIC
   if (aNodeAddr != 0)
   {
      printf("   NodeAddr read from INIC:      0x%04X\n",(UINT32)aNodeAddr);
   }
   else
   {
      printf("** NodeAddr (read from Inic):    failed \n");
   }

   aNodeAddr = inicGetAddr(INIC_GET_ADDR_PHYSICAL);    // read NodeAddr from INIC
   if (aNodeAddr != 0)
   {
      printf("   NodePosAddr read from INIC:   0x%04X\n",(UINT32)aNodeAddr);
   }
   else
   {
      printf("** NodePosAddr (read from Inic): failed \n");
   }
   printf(" \n");
   printf(" * interrupt statistics inic communication:\n");
   printf("   Msg Request from Inic:        0x%08X\n",inicIntCounter);
   printf("   Inic CtrlChan Msgs lost:      0x%08X\n",inicMsgLostCnt);
   printf("   Inic BiosComm Msgs lost:      0x%08X\n",inicMostMsgLostCnt);
   printf("   Inic CtrlChan MCM Msgs lost:  0x%08X\n",inicMostMCMLostCnt);

   #if (INIC_DEBUG_ON == 1)
   printf(" * Debug Counters:\n");
   printf("   InicReInits executed:         0x%08X\n",count_InicReInits);
   printf("   CmdRetries executed:          0x%08X\n",count_InicRetries);


   inicShowTrace();
   trace_active = 1;
   #endif
   return NO_ERROR;
}
#endif

static   UINT32    sh4i2c2inic (UINT32  theRet)
{
   if (theRet != NO_ERROR)
      return(ERROR_INIC | ERR_INIC_IO);
   else
      return(NO_ERROR);
}


#endif // #if CP_INIC
