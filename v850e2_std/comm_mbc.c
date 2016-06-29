/*
 *-----------------------------------------------------------------------
 *
 *     ____   ____           ____   _    _
 *   /  ___| |  __ \       /  ___| | |  | |
 *   | \___  | |  | |  __  | \___  | |__| |
 *   \___  \ | |  | | |__| \___  \ |  __  |
 *    ___/ / | |__| |       ___/ / | |  | |
 *   |____/  |_____/       |____/  |_|  |_|
 *
 *
 *   (c) copyright 2010    Harman-Becker Automotive Systems
 *-----------------------------------------------------------------------
 *   communication handler; replaces gateway.c and interprt_io.c
 *-----------------------------------------------------------------------
 *   $Header:$
 *   $Change:$
 *-----------------------------------------------------------------------
*/

//********************************************************************************************************
// Includes
//********************************************************************************************************
#include "hbbios_define.h"
#ifndef CPU_FAMILY
   #error "CPU_FAMILY must be defined (see hbbios_define.h)"
#endif

#ifndef CPU_FAMILY_C6000
   #error "CPU_FAMILY_C6000 must be defined (see hbbios_define.h)"
#endif

#ifndef CPU_FAMILY_NECV850
   #error "CPU_FAMILY_NECV850 must be defined (see hbbios_define.h)"
#endif


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>    // definitions for varable parameter list

#include "config.h"
#include "define.h"                     // BOOL, UINT8, etc. [needed by interprt.h]
#include "hbbios.h"
#include "global.h"
#include "hbbios_command.h"


#include "timer.h"     // timer functionality

#if (SW_SCRIPT == 1)
   #include "script.h"
#endif

#include "interprt.h"

#if (CPU_FAMILY != CPU_FAMILY_NECV850)
#include "int.h"
#endif

#if (CPU_FAMILY != CPU_FAMILY_C6000)
#if (SW_LED==1)
   #include "led.h"
#endif
#endif

#include "comm.h"

//********************************************************************************************************

#ifndef SOH
   #define SOH       0x01U
#endif

#ifndef EOT
   #define EOT       0x04U
#endif

#ifndef SYN
   #define SYN       0x16U
#endif

#ifndef COMM_USE_CTHREAD
   #error "COMM_USE_CTHREAD must be defined in config_comm.h"
#endif

#ifndef COMM_MAX_RETRY
   #define COMM_MAX_RETRY 3
#endif

#ifndef COMM_BROADCAST_MAX_RETRY
   #define COMM_BROADCAST_MAX_RETRY 1
#endif

#ifndef COMM_TIMEOUT
   #define COMM_TIMEOUT 500
#endif

#ifndef COMM_BROADCAST_TIMEOUT
   #define COMM_BROADCAST_TIMEOUT 150
#endif

#ifndef COMM_RECOVERY_TIME
   #define COMM_RECOVERY_TIME 100
#endif

#ifndef COMM_BROADCAST_RECOVERY_TIME
   #define COMM_BROADCAST_RECOVERY_TIME 0
#endif

#ifndef COMM_SEND_TIMEOUT
   #define COMM_SEND_TIMEOUT 100
#endif

#ifndef COMM_GATEWAY_TIMEOUT
   #define COMM_GATEWAY_TIMEOUT 100
#endif

#ifndef COMM_LOG_ENABLE
   #define COMM_LOG_ENABLE 1
#endif

#ifndef COMM_CMD_ENABLE
   #define COMM_CMD_ENABLE 1
#endif

#ifndef COMM_ERROR_GEN
   #define COMM_ERROR_GEN 1
#endif

#ifndef COMM_GATEWAY
   #define COMM_GATEWAY 1
#endif

#if ((COMM_CMD_ENABLE == 0) && (COMM_LOG_ENABLE >= 1))
   #error "COMM_CMD_ENABLE must be '1' to use the COMM_LOG (see config_comm.h)"
#endif

#if ((COMM_CMD_ENABLE == 0) && (COMM_ERROR_GEN == 1))
   #error "COMM_CMD_ENABLE must be '1' to use the error generator (see config_comm.h)"
#endif

#if (COMM_USE_CTHREAD == 1)
   #if (CPU_FAMILY == CPU_FAMILY_C6000)
      #include <cthread.h>
   #else
      #include "cthread.h"
   #endif
   extern void commandActive (int);
#else
   #define cthread_yield(n) ;
#endif


#define COMM_BUF_SIZE  260 // standard HBBIOS buffers extended with 4 bytes flags
#define COMM_POS_SEQ   256 // location of the sequence number

#define COMM_FIFO_SIZE (COMM_FREE_SIZE >> 1)

#define COMM_FREE_MASK (COMM_FREE_SIZE-1)

#define COMM_FIFO_MASK  (COMM_FIFO_SIZE-1)


//********************************************************************************************************
// types
//********************************************************************************************************
typedef struct
{
   uint8_t interface;
   char *name;
   uint8_t channel;
   void (*func_receive)(unsigned);
   BOOL (*func_send)(unsigned, uint8_t * volatile buffer, unsigned);
   uint32_t (*func_init)(unsigned);     // called, when the related gateway is selected
   void (*func_info)(unsigned);
   void (*func_receive_save)(unsigned);
   BOOL (*func_send_save)(unsigned, uint8_t * volatile buffer, unsigned);
#if (COMM_CMD_ENABLE == 1)
   uint32_t txRetransmit;
   uint32_t txRetransmitDiff;
   uint32_t txLost;      // number of unrecoverable errors
   uint32_t txLostDiff;
   uint32_t txError;      // number of timouts during tx send
   uint32_t txErrorDiff;
   uint32_t txBufDiff;
   uint32_t rxBufDiff;
   uint32_t txRawDiff;
   uint32_t rxRawDiff;
#endif

   uint8_t              fifoDepth;
   uint8_t              fifoDepthMax;
   uint8_t              fifoDepthSend;
   uint8_t              lastSync;   // workaround for compatibility to BCMP 2.9.0.0

   // rx counters to export
   uint32_t             rxRaw;      // received bytes
   volatile uint32_t    rxBufWr;    // received packets (not proceeded)
   volatile uint32_t    rxBufRd;    // received packets (proceeded => rxBufWr-rxBufRd = number of packets in buffer)
   volatile uint32_t    rxIndex;    // number of received bytes of an incomplete packet
   volatile uint32_t    txAckWr;    // number of received ACH/NAK (not proceeeded)
   volatile uint32_t    txAckRd;    // number of received ACH/NAK (proceeeded)
   // tx counters to export
   uint32_t             txRaw;      // transmitted bytes
   volatile uint32_t    txBufWr;    // transmitted packets (not sent)
   volatile uint32_t    txBufRd;    // transmitted packets (sent => txBufWr-txBufRd = number of packets in queue)

   volatile uint32_t    txAck[COMM_FIFO_SIZE];

   uint8_t * volatile   txBuf[COMM_FIFO_SIZE];

   uint8_t              txSeq;
   uint32_t             txTimeout;
   uint8_t              txRetry;
   BOOL                 txText;

   uint8_t              txErrorGen;
#if (COMM_ERROR_GEN == 1)
   uint32_t             txErrorPat;
#endif

   volatile uint32_t    rxNak;

   uint8_t * volatile   rxBuf[COMM_FIFO_SIZE];

   uint8_t * volatile   rxMsg;
   uint8_t              rxCrc;
   unsigned             rxState;
   unsigned             rxStateReturn;
   uint8_t              rxSeq;

   uint8_t              currentRouting;      // switched to gatewayRouting with # xx 01 00
   uint8_t              gatewayRouting;      // modified with # xx 02 D4 D5
   uint8_t              textRouting;         // modified with # xx 04 D4 D5
   unsigned             flags;
} io_config_t;

enum rxStates { S_INIT=0, S_SOH, S_EOT, S_SYN, S_DATA, S_ACK, S_NAK, S_WAIT, S_DLE, S_DLE_I, S_BUF };
enum acknak   { ACK_TX = 0x40000000, NAK_TX = 0x01000000, NAK_LEN = 0x02000000, NAK_SIGN = 0x03000000,
                NAK_BUF = 0x04000000, NAK_SEQ = 0x05000000, NAK_QUE = 0x06000000, NAK_CRC = 0x07000000 };

#if (COMM_LOG_ENABLE >= 1)
   enum msgLog_id {  msgLog_RX=0, msgLog_TX=1, msgLog_TX_RESEND=2, msgLog_BROAD=3, msgLog_TX_ERR=4,
                     msgLog_ERR_GEN=5, msgLog_RX_ACK=6, msgLog_RX_NAK=7, msgLog_TX_ACK=8, msgLog_TX_NAK=9,
                     msgLog_TX_DEL=10, msgLog_TX_FIFO=11, msgLog_TX_SYNC=12 };
   const uint8_t typ_hex[] = { 0x00, 0x01, 0xF1, 0xFF, 0xEE, 0xCC, 0xA0, 0xB0, 0xA1, 0xB1, 0x88, 0xF0, 0xF1, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7 };
   const char *typ_arr[] = { "rx", "tx", "resend", "broadc.", "tx_err", "errGen", "rx ack", "rx nak", "tx ack", "tx nak", "tx delay",
                             "tx_fifo", "tx_sync",
                             "ext_0", "ext_1", "ext_2", "ext_3", "ext_4", "ext_5", "ext_6", "ext_7" };
   uint32_t   dif_arr[21][COMM_INTERFACES];
   char * msgLog_base = 0;
   unsigned msgLog_size = 0;
   static unsigned msgLog_filter = 0;
   static char * msgLog_ptr;
   static unsigned msgLog_mode = 0;

   static uint8_t nakLogMsg[256];

#endif

enum gw_type { GATEWAY_PARAMETER_ERROR = 1, GATEWAY_COMMAND_ERROR = 2, GATEWAY_TRANSMIT_ERROR = 0x100 };

// CMD and GATEWAY configuration
io_config_t io_config[COMM_INTERFACES];

static uint8_t io_config_no = 0;
static uint8_t nrOfBuffers = 4;  // needed for system
static uint32_t msgBuffers[COMM_FREE_SIZE*COMM_BUF_SIZE / sizeof(uint32_t)];

uint8_t  * volatile resultPtr;
uint8_t  * volatile commandPtr;

static uint32_t currentTimeOut = COMM_TIMEOUT;
static unsigned currentRetry = COMM_MAX_RETRY;
static unsigned currentRecoveryTime = COMM_RECOVERY_TIME;

volatile uint32_t    freeBufRd;
volatile uint32_t    freeBufWr;
uint8_t * volatile   freeBuf[COMM_FREE_SIZE];

#if defined(COMM_LED_STATUS_CHANNEL) && (COMM_LED_STATUS_COUNT > 0)
static uint32_t ledSpeed;
#endif

//----- Interfaces -----
static uint8_t currentInterface;

//-----------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------
#if(SW_LED==1)
   static void msgMainLoopStatus (int what);
#endif
#if (COMM_ERROR_GEN == 1)
   static int msgRng(int seed);
#endif

static void msgSend (unsigned ch, uint8_t * volatile buffer);
static BOOL msgGetFree(uint8_t * volatile * entry);
static BOOL msgPutFree(uint8_t * volatile  entry);
static void msgNak(unsigned ch, uint32_t nak);
static void msgTxResend(unsigned ch);
static void msgHandleAcks(void);
static uint32_t msgProceedSend (unsigned ch, uint8_t * volatile buffer, unsigned len);
static uint8_t msgGetInterfaceIndex (uint8_t interface);
static BOOL msgGateway (unsigned ch, BOOL biosMode);
static void msgDummyReceive(unsigned x);
static BOOL msgDummySend(unsigned x, uint8_t * volatile buffer, unsigned y);
static uint32_t msgSendFifoDepth(uint8_t ch);
static uint32_t msgSendSync(uint8_t ch, uint8_t sync);

//********************************************************************************************************

#ifdef COMM_CHECK_BUFFERS
// for usage with debugger only: set break point to "cnt++"
void msgCheckBuffers(void)
{
   unsigned n, m;
   static unsigned cnt = 0;

   for (n=freeBufRd; n<freeBufWr; n++)
   {
      if (freeBuf[n & COMM_FREE_MASK] == (uint8_t *)0xDEADBEEF)
      {
         cnt++;
         break;
      }
      for (m=freeBufRd+n; m<freeBufWr; m++)
      {
         if (freeBuf[n & COMM_FREE_MASK] == freeBuf[m & COMM_FREE_MASK])
         {
            cnt++;
            break;
         }
      }
   }
}
#endif


//********************************************************************************************************
// msgGetFree
//--------------------------------------------------------------------------------------------------------
// Function:   get a buffer from free queue (if available)
//--------------------------------------------------------------------------------------------------------
// Input:      pointer
// Output:     true, if entry contains free buffer; else false
//--------------------------------------------------------------------------------------------------------
BOOL msgGetFree(uint8_t * volatile * entry)
{
   if (freeBufWr - freeBufRd == 0)
      return(false);
   else
   {
      *entry = freeBuf[freeBufRd & COMM_FREE_MASK];
   #if (COMM_DEBUG == 1)
      freeBuf[freeBufRd & COMM_FREE_MASK] = (uint8_t *)0xDEADBEEF;
   #endif
      freeBufRd++;
      return(true);
   }
}


//********************************************************************************************************
// msgPutFree
//--------------------------------------------------------------------------------------------------------
// Function:   give buffer back to free pool
//--------------------------------------------------------------------------------------------------------
// Input:      pointer
// Output:     true, if OK; false should not happen
//********************************************************************************************************
BOOL msgPutFree(uint8_t * volatile  entry)
{
   if (freeBufWr - freeBufRd < COMM_FREE_SIZE)
   {
      freeBuf[freeBufWr & COMM_FREE_MASK] = entry;
      freeBufWr++;
      return(true);
   }
   else
      return(false);
}


//********************************************************************************************************
// msgSendASCII()
//--------------------------------------------------------------------------------------------------------
// Function:   - can be called by every function
//             - encapsulates a string into the text-Command 'd'
//             - broadcast as long as the target interface is not defined
//--------------------------------------------------------------------------------------------------------
// Input:   char* (pointer to a String)
// Output:  none
//********************************************************************************************************
void msgSendASCII(char * pmessage)
{
   uint32_t SA;                      // Bits SA (Send ASCII)
   uint32_t len;
   BOOL     test;
   uint8_t  * volatile textPtr;

   SA  = getMode(MODE_SA);
   if (! SA)
      return;
   if ((currentInterface != ifNone) && (io_config[currentInterface].txText == false))
      return;

   if (! *pmessage)
      return;

   // construct d-message:
   len = strlen (pmessage);
   if (len > 253)
      len = 253;

   COMM_ATOM_LOCK;
   test = msgGetFree(&textPtr);
   COMM_ATOM_UNLOCK;

   if (test)
   {
      textPtr[0] = len + 2;
      textPtr[1] = 'd';
      memcpy (&textPtr[2], pmessage, len);

      if (currentInterface == ifNone)
      {
         msgBroadCast(textPtr, false); // broadcast doesn't swallow the buffer
         msgPutFree(textPtr);          // therefore we need to give it back here
      }
      else
      {
         msgSend(currentInterface, textPtr);

#ifndef COMM_QUEUE_ASCII
         // call receiver function and wait for ack (means: d-messages are not queued and therefore slow, but save)
         while (io_config[currentInterface].txBufWr - io_config[currentInterface].txBufRd > 0)
            msgHandleAcks();
#else
         // call receiver function and wait for ack (if not interrupt driven), to avoid lost acks
         if ((void *)io_config[currentInterface].func_receive != NULL)
            while (io_config[currentInterface].txBufWr - io_config[currentInterface].txBufRd > 0)
               msgHandleAcks();
#endif
      }
   }
}


#ifdef COMM_JPRINTF
//********************************************************************************************************
// jprintf
//--------------------------------------------------------------------------------------------------------
// Function:   prints a formatted string via JTAG interface; all usual printf format strings are useable
//--------------------------------------------------------------------------------------------------------
// Input:   format - pointer to the format string
//          ...    - variable arguments referring to the format expressions given in the format string
// Output:  length of created string
//********************************************************************************************************
int jprintf(const char *format, ...)
{
   va_list  arg;           // argument list
   int result;

   va_start(arg, format);
   result = vsprintf(str, format, arg);
#if (COMM_USE_CTHREAD == 1)
   printf ("%s\n", str);
#else
   msgSendASCII(str);
#endif
   va_end(arg);

   return(result);
}
#endif

#ifdef COMM_DPRINTF
//********************************************************************************************************
// dprintf
//--------------------------------------------------------------------------------------------------------
// Function:   prints a formatted string for debug purposes; all usual printf format strings are useable
//--------------------------------------------------------------------------------------------------------
// Input:   format - pointer to the format string
//          ...    - variable arguments referring to the format expressions given in the format string
// Output:  length of created string
//********************************************************************************************************
int dprintf(const char *format, ...)
{
   va_list  arg;           // argument list
   int result;

   if(debug_msg_on)
   {
      va_start(arg, format);
      result = vsprintf(str, format, arg);
#ifdef COMM_DPRINTF_NO_AUTOLINEFEED
      printf ("%s", str);
#else
      printf ("%s\n", str);
#endif
      va_end(arg);
      return(result);
   }
   else
   {
      return(0U);
   }
}
#endif


//********************************************************************************************************
// msgNak
//--------------------------------------------------------------------------------------------------------
// Function:   set NAK tag with sequence number
//--------------------------------------------------------------------------------------------------------
// Input:      ch: index to access the belonging structure
//             nak: NAK reason for communication logger
// Output:     none
//********************************************************************************************************
void msgNak(unsigned ch, uint32_t nak)
{
   if (io_config[ch].rxNak == 0)
   {
#if (COMM_LOG_ENABLE >= 1)
      memcpy(&nakLogMsg[0], (void *)io_config[ch].rxMsg, io_config[ch].rxIndex);
#endif
      io_config[ch].rxNak = nak | (io_config[ch].rxIndex << 8) | io_config[ch].rxSeq;
   }
}


//********************************************************************************************************
// msgReceive
//--------------------------------------------------------------------------------------------------------
// Function:   this function must be called from each receive interface; msgReceive decodes HBBIOS
//             messages and feeds ACK and NAK into separate fifos
//--------------------------------------------------------------------------------------------------------
// Input:      ch: index to access the belonging structure
//                 => you get this value as result of calling msgRegister()
//             buffer: pointer to the array of bytes you want to feed in
//                 => serial interfaces may tranport just a single byte here
//             len:  number of bytes to work through
// Output:     none
//********************************************************************************************************
void msgReceive (unsigned ch, uint8_t * volatile buffer, unsigned len)
{
   unsigned n;
   BOOL flag;

   io_config[ch].rxRaw+=len;

   for (n=0; n<len; n++)
   {
      switch (io_config[ch].rxState)
      {
         case S_DATA:
            if (buffer[n] == DLE)
               io_config[ch].rxState = S_DLE;
            else
               if (io_config[ch].rxIndex == 256)
               {
                  msgNak(ch, NAK_LEN);
                  io_config[ch].rxState = S_WAIT;
               }
               else
               {
                  io_config[ch].rxMsg[io_config[ch].rxIndex++] = buffer[n];
                  io_config[ch].rxCrc += buffer[n];
               }
            break;

         case S_DLE:
            switch (buffer[n])
            {
               case DLE:
                  if (io_config[ch].rxIndex == 256)
                  {
                     msgNak(ch, NAK_LEN);
                     io_config[ch].rxState = S_WAIT;
                  }
                  else
                  {
                     io_config[ch].rxMsg[io_config[ch].rxIndex++] = DLE;
                     io_config[ch].rxCrc += DLE;
                     io_config[ch].rxState = S_DATA;
                  }
                  break;
               case EOT:
                  io_config[ch].rxState = S_EOT;
                  break;
               case SYN:
                  io_config[ch].rxState = S_SYN;
                  break;
               default:
                  msgNak(ch, NAK_SIGN);
                  io_config[ch].rxState = S_WAIT;
                  break;
            }
            break;

         case S_INIT:   // ready to receive new message
            io_config[ch].rxStateReturn = S_INIT;
            if (buffer[n] == DLE)
               io_config[ch].rxState = S_DLE_I;
            break;

         case S_DLE_I:
            switch (buffer[n])
            {
               case SOH:
                  if (io_config[ch].rxStateReturn == S_INIT)
                     io_config[ch].rxState = S_SOH;
                  else
                     io_config[ch].rxState = io_config[ch].rxStateReturn;
                  break;
               case ACK:
                  io_config[ch].rxState = S_ACK;
                  break;
               case NAK:
                  io_config[ch].rxState = S_NAK;
                  break;
               case 'B':
                  io_config[ch].rxState = S_BUF;
                  break;
               case SYN:
                  io_config[ch].rxState = S_SYN;
                  break;
               case DLE:
                   io_config[ch].rxState = S_DLE_I;
                   break;
               default:
                  io_config[ch].rxState = io_config[ch].rxStateReturn;
                  break;
            }
            break;

         case S_ACK:
            io_config[ch].txAck[io_config[ch].txAckWr & COMM_FIFO_MASK] = buffer[n] | ACK_TX;
            io_config[ch].txAckWr++;
            io_config[ch].rxState = io_config[ch].rxStateReturn;
            break;

         case S_NAK:
            io_config[ch].txAck[io_config[ch].txAckWr & COMM_FIFO_MASK] = buffer[n] | NAK_TX;
            io_config[ch].txAckWr++;
            io_config[ch].rxState = io_config[ch].rxStateReturn;
            if (io_config[ch].lastSync == buffer[n])
            {
               if (io_config[ch].fifoDepthSend == 0)
                  io_config[ch].fifoDepthSend = 2;
               if ((1 < io_config[ch].fifoDepthSend) && (io_config[ch].fifoDepthSend < 6))
                  io_config[ch].fifoDepthSend++;
            }
            break;

         case S_BUF:
            if (buffer[n] > 0)
            {
               io_config[ch].fifoDepth = buffer[n];
               if (buffer[n] > io_config[ch].fifoDepthMax)
               {
                  io_config[ch].fifoDepth = io_config[ch].fifoDepthMax;
                  io_config[ch].fifoDepthSend = 1;
               }
            }
            io_config[ch].rxState = io_config[ch].rxStateReturn;
            break;

         case S_SOH:
            io_config[ch].rxIndex = 0;
            if (buffer[n] == io_config[ch].rxSeq)
            {
               flag = (io_config[ch].rxMsg != NULL);
               if (!flag)
               {  // get new buffer; use LOCK macro for non IRQ interfaces
                  if (io_config[ch].func_receive != NULL)
                     COMM_ATOM_LOCK;
                  flag = msgGetFree(&io_config[ch].rxMsg);
                  if (io_config[ch].func_receive != NULL)
                     COMM_ATOM_UNLOCK;
               }
               if (flag)
               {  // got new buffer
               #ifdef COMM_CHECK_BUFFERS
                  msgCheckBuffers();
               #endif
                  io_config[ch].rxCrc = 0;
                  io_config[ch].rxState = S_DATA;
               }
               else
               {  // couldn't get a new buffer
                  msgNak(ch, NAK_BUF);
                  io_config[ch].rxState = S_WAIT;
               }
            }
            else
            {
               msgNak(ch, NAK_SEQ | (buffer[n] << 16));
               io_config[ch].rxState = S_WAIT;
            }
            break;

         case S_EOT:
            io_config[ch].rxCrc += buffer[n];
            if ((io_config[ch].rxCrc & 0xFF) == 0)
            {
               if (io_config[ch].rxBufWr - io_config[ch].rxBufRd < io_config[ch].fifoDepthMax)
               {
                  io_config[ch].rxMsg[COMM_POS_SEQ] = io_config[ch].rxSeq;   // seq number for ACK
                  io_config[ch].rxBuf[io_config[ch].rxBufWr & COMM_FIFO_MASK] = io_config[ch].rxMsg;
                  io_config[ch].rxBufWr++;
                  io_config[ch].rxMsg = NULL;
                  io_config[ch].rxSeq = (io_config[ch].rxSeq+1) & 0xFF;
                  io_config[ch].rxState = S_INIT;
               }
               else
               {
                  msgNak(ch, NAK_QUE);
                  io_config[ch].rxState = S_WAIT;
               }
            }
            else
            {
               msgNak(ch, NAK_CRC);
               io_config[ch].rxState = S_WAIT;
            }
            break;

         case S_SYN:
            io_config[ch].rxSeq = buffer[n];
            io_config[ch].rxState = S_INIT;
            break;

         case S_WAIT:   // wait for SYN after sending NAK, ignore new packets
            io_config[ch].rxStateReturn = S_WAIT;
            if (buffer[n] == DLE)
               io_config[ch].rxState = S_DLE_I;
            break;

      }  // switch
   }  // for

} // end of function msgReceive


//********************************************************************************************************
// msgReceiveRaw
//--------------------------------------------------------------------------------------------------------
// Function:   can be called to transmit internally generated HBBIOS messages
//
//--------------------------------------------------------------------------------------------------------
// Input:      ch: index to access the belonging structure
//                 => you get this value as result of calling msgRegister()
//             buffer: pointer to the array of bytes you want to feed in
//                 => serial interfaces may tranport just a single byte here
//             len:  number of bytes to work through
// Output:     none
//********************************************************************************************************
void msgReceiveRaw (unsigned ch, uint8_t * volatile buffer, unsigned len)
{
   BOOL flag;

   io_config[ch].rxRaw+=len;

   if ((io_config[ch].currentRouting == ch) && (io_config[ch].flags & COMM_FLAGS_ROUTE))
      return;  // ignore message

   flag = (io_config[ch].rxMsg != NULL);
   if (!flag)
   {  // get initial buffer; use LOCK macro for non IRQ interfaces
      if (io_config[ch].func_receive != NULL)
         COMM_ATOM_LOCK;
      flag = msgGetFree(&io_config[ch].rxMsg);
      if (io_config[ch].func_receive != NULL)
         COMM_ATOM_UNLOCK;
   }
   if (flag)  // got new buffer
   {
      if (io_config[ch].rxBufWr - io_config[ch].rxBufRd < io_config[ch].fifoDepthMax)
      {
         memcpy(io_config[ch].rxMsg, buffer, len);
         io_config[ch].rxMsg[COMM_POS_SEQ] = io_config[ch].rxSeq++;
         io_config[ch].rxBuf[io_config[ch].rxBufWr & COMM_FIFO_MASK] = io_config[ch].rxMsg;
         io_config[ch].rxBufWr++;
         io_config[ch].rxMsg = NULL;
      } else // couldn't get a new buffer
      {
      io_config[ch].txLost++;
      }
   }
} // end of function msgReceiveRaw


#if (COMM_ERROR_GEN == 1)
//********************************************************************************************************
// msgRng
//--------------------------------------------------------------------------------------------------------
// Function:   pseudo random function for optional error generator
//--------------------------------------------------------------------------------------------------------
// Input:      seed: inital value
// Output:     result (use as nect seed)
//********************************************************************************************************
int msgRng(int seed)
{
   unsigned int lo;
   unsigned int hi;
   // white noise generator
   // Park-Millrt "minimal standard" 31 bit pseudo-random number generator
   // source: http://www.firstpr.com.au/dsp/rand31/
   lo = 16807 * (seed & 0xFFFF);
   hi = 16807 * (seed >> 16);
   lo += (hi & 0x7FFF) << 16;
   lo += hi >> 15;
   if ( lo > 0x7FFFFFFF ) lo -= 0x7FFFFFFF;
   return (int)lo;
}
#endif


//********************************************************************************************************
// msgDummyReceive/Send
//--------------------------------------------------------------------------------------------------------
// Function:   dummy function to disable receive/send function
//--------------------------------------------------------------------------------------------------------
// Output:     false, to force transmit failure
//********************************************************************************************************
void msgDummyReceive(unsigned x)
{
}

BOOL msgDummySend(unsigned x, uint8_t * volatile buffer, unsigned y)
{
   return(false);
}


//********************************************************************************************************
// msgControlInterface
//--------------------------------------------------------------------------------------------------------
// Function:   enable/disable interface by replace/restore receive/send-function pointers
//             with msgDummyTransceive
//--------------------------------------------------------------------------------------------------------
// Input:      ch: channel to send; enable/disable
//********************************************************************************************************
void msgControlInterface(unsigned ch, BOOL enable)
{
   if (enable)
   {
      io_config[ch].func_receive = io_config[ch].func_receive_save;
      io_config[ch].func_send = io_config[ch].func_send_save;
   }
   else
   {
      io_config[ch].func_receive = msgDummyReceive;
      io_config[ch].func_send = msgDummySend;
   }
}


//********************************************************************************************************
// msgGetCounters
//--------------------------------------------------------------------------------------------------------
// Function:   give back pointer to message counters
//--------------------------------------------------------------------------------------------------------
// Input:      ch
//********************************************************************************************************
extern msgCounters_t * msgGetCounters(unsigned ch)
{
   return((msgCounters_t *)&io_config[ch].rxRaw);
}


//********************************************************************************************************
// msgProceedSend
//--------------------------------------------------------------------------------------------------------
// Function:   call of the send function; try as long as we get false
//             or till the send timeout is over; feed in errors if defined
//--------------------------------------------------------------------------------------------------------
// Input:      ch: channel to send
//             buffer: coded buffer
//             len: number of valid bytes in buffer (max. 518), 0 => encoding needed
// Output:     none
//********************************************************************************************************
uint32_t msgProceedSend (unsigned ch, uint8_t * volatile buffer, unsigned len)
{
   uint32_t time = timGetTime();
#if (COMM_ERROR_GEN == 1)
   static int seed = 42;
#endif

   if (len == 0)
   {  // do mbc encoding
      uint8_t txMsg[COMM_MBC_MAX] = { DLE, SOH };
      uint8_t chkSum = 0;
      unsigned n;

      txMsg[2] = buffer[COMM_POS_SEQ];
      len = 3;
      for (n=0; n < buffer[0]; n++)
      {
         if (buffer[n] == DLE)
            txMsg[len++] = DLE;
         chkSum += buffer[n];
         txMsg[len++] = buffer[n];
      }
      txMsg[len++] = DLE;
      txMsg[len++] = EOT;
      txMsg[len++] = (~chkSum+1) & 0xFF;
      buffer = txMsg;
   }

#if (COMM_ERROR_GEN == 1)
   if (io_config[ch].txErrorGen > 0)
   {
      seed = msgRng(seed);
      if ((seed & io_config[ch].txErrorPat) == 0)
      {
         buffer[seed % len] = ~buffer[seed % len];
   #if (COMM_LOG_ENABLE >= 1)
         msgLog(msgLog_ERR_GEN, ch, buffer, len, seed % len);
   #endif
      }
   }
#endif

   io_config[ch].txRaw+=len;

   while (!io_config[ch].func_send(io_config[ch].channel, buffer, len))
   {
      // call receiver function (if not interrupt driven), to resolve possible dead-lock situations
      if ((void *)io_config[ch].func_receive != NULL)
         io_config[ch].func_receive(io_config[ch].channel);
      if (timDifference(time) > COMM_SEND_TIMEOUT)
      {
      #if (COMM_CMD_ENABLE == 1)
         io_config[ch].txError++;    // count messages, which couldn't be sent
      #endif
         io_config[ch].txText = false; // switch of text output
      #if (COMM_LOG_ENABLE >= 1)
         msgLog(msgLog_TX_ERR, ch, buffer, len, 0);
      #endif
         return(1); // internal error to avoid cascading timeouts e.g. in msgResend()
      }
   }
   return(NO_ERROR);
}


//********************************************************************************************************
// msgFifoDepth
//--------------------------------------------------------------------------------------------------------
// Function:   send fifoDepth
//--------------------------------------------------------------------------------------------------------
// Input:      ch
// Output:     none
//********************************************************************************************************
uint32_t msgSendFifoDepth(uint8_t ch)
{
   uint8_t txMsg[3] = { DLE, 'B' };

   if (io_config[ch].flags & COMM_FLAGS_RAWDATA)
      return(NO_ERROR);

   txMsg[2] = io_config[ch].fifoDepthMax;
#if (COMM_LOG_ENABLE >= 1)
   msgLog(msgLog_TX_FIFO, ch, NULL, 0, io_config[ch].fifoDepthMax);
#endif

   return(msgProceedSend(ch, txMsg, 3));
}


//********************************************************************************************************
// msgSendSync
//--------------------------------------------------------------------------------------------------------
// Function:   send sync
//--------------------------------------------------------------------------------------------------------
// Input:      ch, sync symbol
// Output:     none
//********************************************************************************************************
uint32_t msgSendSync(uint8_t ch, uint8_t sync)
{
   uint8_t txMsg[3] = { DLE, SYN };

   if (io_config[ch].flags & COMM_FLAGS_RAWDATA)
      return(NO_ERROR);

   txMsg[2] = sync;
#if (COMM_LOG_ENABLE >= 1)
   msgLog(msgLog_TX_SYNC, ch, NULL, 0, sync);
#endif

   return(msgProceedSend(ch, txMsg, 3));
}


//********************************************************************************************************
// msgResend
//--------------------------------------------------------------------------------------------------------
// Function:   this function is called as answer on a NAK or a timeout; at first it sends a SYNC-message
//             to synchronize the sequence counter of the receiver; then it resend all messages stored
//             in the tx-buffer, which havn't received an ACK yet and wait COMM_RECOVERY_TIME to give
//             the receiver time to clear the situation
//--------------------------------------------------------------------------------------------------------
// Input:      ch: channel, which must proceed the resend
// Output:     none
//********************************************************************************************************
void msgTxResend(unsigned ch)
{
   uint32_t ptr;

   timWait(currentRecoveryTime);

   // cleanup ack queue
   while ((io_config[ch].txAckWr - io_config[ch].txAckRd) > 0)
   {
   #if (COMM_DEBUG == 1)
      io_config[ch].txAck[io_config[ch].txAckRd & COMM_FIFO_MASK] = 0xDEADBEEF;
   #endif
      io_config[ch].txAckRd++;
   }

   if (msgSendSync(ch, io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK][COMM_POS_SEQ]))
      return;  // forget about resending in the case of transmit timeout

   if (io_config[ch].fifoDepthSend < 6)
      msgSendFifoDepth(ch);

   // set pointer to first element in fifo
   ptr = io_config[ch].txBufRd;
   while (io_config[ch].txBufWr - ptr > 0)
   {  // resend all messages (without moving pointers!)
      msgProceedSend(ch, io_config[ch].txBuf[ptr & COMM_FIFO_MASK], 0);
   #if (COMM_CMD_ENABLE == 1)
      io_config[ch].txRetransmit++;
   #endif

   #if (COMM_LOG_ENABLE >= 1)
      msgLog(msgLog_TX_RESEND, ch, io_config[ch].txBuf[ptr & COMM_FIFO_MASK], io_config[ch].txBuf[ptr & COMM_FIFO_MASK][0], io_config[ch].txBuf[ptr & COMM_FIFO_MASK][COMM_POS_SEQ]);
   #endif

      ptr++;
   }
}


//********************************************************************************************************
// msghandleAcks
//--------------------------------------------------------------------------------------------------------
// Function:   this fucntion is called cyclically from msgPoll andthe send functions
//             (if there is no more credit); it does the following for each interface:
//             - poll receive interface (if not interrupt driven)
//             - send rx NAKs if any
//             - free all tx buffers, which have received an ACK or call msgTxResend in the case of a NAK
//             - if timeout; resend and inc. retry counter
//             - if max retry reached, clear all tx buffers
//--------------------------------------------------------------------------------------------------------
// Input:      none
// Output:     none
//********************************************************************************************************
void msgHandleAcks(void)
{
   uint32_t ackMsg;
   unsigned ch;
   uint8_t ackBuf;
   uint8_t ackDiff;
   uint8_t txMsg[3] = { DLE, NAK };

   for (ch=0; ch<io_config_no; ch++)
   {
      // call receiver function (if not interrupt driven)
      if ((void *)io_config[ch].func_receive != NULL)
         io_config[ch].func_receive(io_config[ch].channel);

      // send buffer allignment message, if needed
      if (io_config[ch].fifoDepthSend == 1)
      {
         msgSendFifoDepth(ch);
         io_config[ch].fifoDepthSend = 0;
      }

      // send Rx NAKs (if any)
      if (io_config[ch].rxNak != 0)
      {
         while (io_config[ch].rxBufWr - io_config[ch].rxBufRd > 0)
         {  // free all receive buffers
            msgPutFree(io_config[ch].rxBuf[io_config[ch].rxBufRd & COMM_FIFO_MASK]);

         #if (COMM_DEBUG == 1)
            io_config[ch].rxBuf[io_config[ch].rxBufRd & COMM_FIFO_MASK] = (uint8_t *)0xDEADBEEF;
         #endif
            io_config[ch].rxBufRd++;
         }

         txMsg[2] = (uint8_t)io_config[ch].rxNak;
         msgProceedSend(ch, txMsg, 3);

      #if (COMM_LOG_ENABLE >= 1)
         msgLog(msgLog_RX_NAK, ch, &nakLogMsg[0], (io_config[ch].rxNak>>8) & 0xFF, io_config[ch].rxNak);
      #endif
         io_config[ch].rxNak = 0;
      }

      if ((io_config[ch].txAckWr - io_config[ch].txAckRd) > 0)
      {
         // get message from ackFifo
         ackMsg = io_config[ch].txAck[io_config[ch].txAckRd & COMM_FIFO_MASK];
      #if (COMM_DEBUG == 1)
         io_config[ch].txAck[io_config[ch].txAckRd & COMM_FIFO_MASK] = 0xDEADBEEF;
      #endif
         io_config[ch].txAckRd++;

         if (ackMsg & ACK_TX)
         {  // received ack
            if (io_config[ch].txBufWr - io_config[ch].txBufRd > 0)
            {
               ackBuf = 0xFF & io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK][COMM_POS_SEQ];

               if (ackBuf == (ackMsg & 0xFF))
               {  // if buffer in queue AND seqNr of this buffer == ackSeq => free
               #if (COMM_LOG_ENABLE >= 1)
                  msgLog(msgLog_TX_ACK, ch, NULL, 0, ackBuf);
               #endif
                  // free buffer
                  msgPutFree(io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK]);

               #if (COMM_DEBUG == 1)
         		   io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK] = (uint8_t *)0xDEADBEEF;
               #endif
                  io_config[ch].txBufRd++;

                  io_config[ch].txText = true;
               } else
               {  // ACK doesn't match! Try to recover or resend
               #if (COMM_LOG_ENABLE >= 1)
                  msgLog(msgLog_TX_NAK, ch, NULL, 0, (2 << 24) | (ackBuf << 8) | (ackMsg & 0xff));
               #endif
                  ackDiff = ((uint8_t)ackMsg) - ackBuf+1;
                  if (ackDiff > 3)  // allow not more than 3 consecutive lost acks
                  msgTxResend(ch);
                  else
                     while (ackDiff > 0)
                     {
                        ackBuf = io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK][COMM_POS_SEQ];
                     #if (COMM_LOG_ENABLE >= 1)
                        msgLog(msgLog_TX_NAK, ch, NULL, 0, (4 << 24) | ackDiff << 8 | ackBuf);
                     #endif
                        // free buffer
                        msgPutFree(io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK]);
                        io_config[ch].txBufRd++;
                        io_config[ch].txText = true;
                        ackDiff--;
                     }
               }
            } else
            { // ACK without belonging buffer; log only and remove ack
            #if (COMM_LOG_ENABLE >= 1)
               msgLog(msgLog_TX_NAK, ch, NULL, 0, (3 << 24) | ackMsg);
            #endif
            }
         } else   // received nak
         {
         #if (COMM_LOG_ENABLE >= 1)
            msgLog(msgLog_TX_NAK, ch, NULL, 0, ackMsg);
         #endif
            msgTxResend(ch);
         }
      }

      if (io_config[ch].txBufWr - io_config[ch].txBufRd > 0)
      {
         if (timDifference(io_config[ch].txTimeout) > currentTimeOut)
         {
            msgTxResend(ch);
            io_config[ch].txRetry++;
            io_config[ch].txTimeout = timGetTime();
            io_config[ch].txText = false;
         }

         if (io_config[ch].txRetry >= currentRetry)
         {
            while (io_config[ch].txBufWr - io_config[ch].txBufRd > 0)
            {  // delete all buffers (move them to the free queue)
               msgPutFree(io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK]);

            #if (COMM_DEBUG == 1)
      		   io_config[ch].txBuf[io_config[ch].txBufRd & COMM_FIFO_MASK] = (uint8_t *)0xDEADBEEF;
            #endif
               io_config[ch].txBufRd++;
            #if (COMM_CMD_ENABLE == 1)
               io_config[ch].txLost++;
            #endif
               io_config[ch].txText = false;
            }
         }
      }
   }

#if (COMM_USE_CTHREAD == 1)
{  // don't call cthread_yield() when not running in main thread!!!!!
   int  s = splhigh();
   if (cthread_context(s) == 0)
   cthread_yield();
   splx(s);
}
#endif

}


//********************************************************************************************************
// msgBroadCast
//--------------------------------------------------------------------------------------------------------
// Function:   this function is called from msgSendASCII, as long as the current interface is undefined
//             (set to ifNone); it's also used to send the initial s-message from main();
//             the respond forces the startup logger to stop (if defined)
//--------------------------------------------------------------------------------------------------------
// Input:      buffer: pointer on message to broadcast => buffer is still and must be released manually
// Output:     none
//********************************************************************************************************
void msgBroadCast(uint8_t * volatile buffer, BOOL logOff)
{
   unsigned ch;
   BOOL flag;
   unsigned error;

   currentRecoveryTime = COMM_BROADCAST_RECOVERY_TIME;
   currentTimeOut = COMM_BROADCAST_TIMEOUT;
   currentRetry = COMM_BROADCAST_MAX_RETRY;

   for (ch=0; ch<io_config_no; ch++)
      if ((io_config[ch].txText) && !(io_config[ch].flags & COMM_FLAGS_RAWDATA))
      {
         error = 0;
         if (io_config[ch].fifoDepthSend == 0)
         {
            io_config[ch].fifoDepthSend=2;
            error = msgSendSync(ch, 0);   // sync to 0
            if (error == 0)
               error = msgSendFifoDepth(ch);
         }

         if (error)
            io_config[ch].txText = false;
         else
         {
            buffer[COMM_POS_SEQ] = io_config[ch].txSeq++;   // add sequence number to match later with txAckFifo

            // all buffers must be copied
            COMM_ATOM_LOCK;
            msgGetFree(&io_config[ch].txBuf[io_config[ch].txBufWr & COMM_FIFO_MASK]);
            COMM_ATOM_UNLOCK;

            memcpy(io_config[ch].txBuf[io_config[ch].txBufWr & COMM_FIFO_MASK], buffer, COMM_BUF_SIZE);
            io_config[ch].txBufWr++;

            io_config[ch].txTimeout = timGetTime();
            io_config[ch].txRetry = 0;

            msgProceedSend(ch, buffer, 0);
         #if (COMM_LOG_ENABLE >= 1)
            msgLog(msgLog_BROAD, ch, buffer, buffer[0], buffer[COMM_POS_SEQ]);
         #endif
         }
      }

   do
   {  // check all interfaces in one loop
      flag = false;
      for (ch=0; ch<io_config_no; ch++)
      {
         if (!(io_config[ch].flags & COMM_FLAGS_RAWDATA))
         {
            msgHandleAcks();
            if (io_config[ch].txBufWr - io_config[ch].txBufRd > 0)
               flag = true;
         }
      }
   } while (flag);

   if (logOff)
   {
#if (COMM_LOG_ENABLE >= 1)
      msgLog_base = 0;              // stop startup logging
#endif
   }

   currentRecoveryTime = COMM_RECOVERY_TIME;
   currentTimeOut = COMM_TIMEOUT;
   currentRetry = COMM_MAX_RETRY;
}


//********************************************************************************************************
// msgSend
//--------------------------------------------------------------------------------------------------------
// Function:   send function for non-coded messages called from msgPoll and msgSendASCII; if we have still
//             space on txBuffer (credit not depleted), just insert the message into queue and send out;
//             otherwise call msgHandleAcks() till situation is resolved (e.g. we received ACKs to clear
//             the buffer or we run into a timeout)
//--------------------------------------------------------------------------------------------------------
// Input:      ch: channel to send
//             buffer: HBBIOS message to send
// Output:     none
//********************************************************************************************************
void msgSend (unsigned ch, uint8_t * volatile buffer)
{

#if (COMM_LOG_ENABLE >= 1)
   uint32_t time = timGetTime();
#endif

   // reset timeout and retry counter
   io_config[ch].txTimeout = timGetTime();
   io_config[ch].txRetry = 0;
   // call msgHandleAcks, while credit is reached
   while (io_config[ch].txBufWr - io_config[ch].txBufRd >= io_config[ch].fifoDepth)
      msgHandleAcks();
#if (COMM_LOG_ENABLE >= 1)
   time = timDifference(time);
   if (time > 1)
      msgLog(msgLog_TX_DEL, ch, NULL, 0, time);
#endif

   buffer[COMM_POS_SEQ] = io_config[ch].txSeq++;   // add sequence number to match later with txAckFifo
   io_config[ch].txBuf[io_config[ch].txBufWr & COMM_FIFO_MASK] = buffer;
   io_config[ch].txBufWr++;

   if (io_config[ch].flags & COMM_FLAGS_RAWDATA)
   {
      msgProceedSend(ch, buffer, buffer[0]);
#if (COMM_LOG_ENABLE >= 1)
      msgLog(msgLog_TX, ch, buffer, buffer[0], ((io_config[ch].txBufWr - io_config[ch].txBufRd) << 8 ) | buffer[COMM_POS_SEQ]);
#endif
      if (io_config[ch].flags & COMM_FLAGS_AUTOACK)
      {  // feed in auto ack
         io_config[ch].txAck[io_config[ch].txAckWr & COMM_FIFO_MASK] = buffer[COMM_POS_SEQ] | ACK_TX;
         io_config[ch].txAckWr++;
      }
   }
   else
      if (!msgProceedSend(ch, buffer, 0))  // len=0 means encoding required
      {
#if (COMM_LOG_ENABLE >= 1)
   msgLog(msgLog_TX, ch, buffer, buffer[0], ((io_config[ch].txBufWr - io_config[ch].txBufRd) << 8 ) | buffer[COMM_POS_SEQ]);
#endif
   }

} // end of function msgSend


#if (SW_SCRIPT == 1)
//********************************************************************************************************
// msgScript
//--------------------------------------------------------------------------------------------------------
// Function:   handle the startup of autorun scripts from packet and flash
//--------------------------------------------------------------------------------------------------------
// Input:
// Output:     none
//********************************************************************************************************
unsigned msgScript(uint32_t auto_packet, uint32_t auto_flash, uint32_t auto_delay)
{
   static unsigned state = S_INIT;
   static unsigned cnt = 0;
   static uint32_t time;
   static uint32_t script;
   static uint32_t script_next;
   static uint32_t script_delay;
   unsigned error;

   switch (state)
   {
      case S_INIT:  // init
         script = auto_packet;
         script_next = auto_flash;
         script_delay = auto_delay;
         state = S_SOH;
         break;

      case S_SOH:
         cnt++;
         if (script == 0)
            state = S_SYN;
         else
         {
            time = timGetTime();
            state = S_WAIT;
         }
         if (cnt > 2)
            state = S_EOT;
         break;

      case S_WAIT:
         if (timDifference(time) > script_delay)
         {
            printf("   script @ 0x%08x => ", script);
            error = scriptStart(script);
            if (error)
            {
               if (error == (ERROR_SCRIPT | SCRIPT_ERROR_NOSCRIPT_FOUND))
                  printf("not found\n");
               else
                  printf("killed\n");
            }
            else
            {
               script_delay = 0; // no need to wait twice
               printf("started\n");
            }
            state = S_SYN;
         }
         break;

      case S_SYN:
         if (scriptReady())
         {
            script = script_next;
            state = S_SOH;
         }
         break;

      default:
         return(0);
   }

   return(1);

}
#endif


//********************************************************************************************************
// msgPoll
//--------------------------------------------------------------------------------------------------------
// Function:   poll interfaces for new messages and handle gateway; this function is called from main loop
//--------------------------------------------------------------------------------------------------------
// Input:      broadcast == true: avoid setting of current_interface in msgPoll
//             when current_interface starts with ifNone, all messages (usually the startup screen) are
//             broadcasted to all reponding interfaces
// Output:     return true, as long as an script is running
//********************************************************************************************************
BOOL msgPoll(BOOL broadcast)
{
   uint8_t  ch;
   uint8_t txMsg[3] = { DLE, ACK };

#if(SW_LED==1)
   msgMainLoopStatus (-1);
#endif    // Poll Inomming Msgs from multiple moduls

   // check all command channels
   for (ch=0; ch<io_config_no; ch++)
   {
      // handle Acks
      msgHandleAcks();

      // work messages of a receive interface (if any); check target fifo before empty the receive fifo
      if ((io_config[ch].rxBufWr - io_config[ch].rxBufRd > 0) &&
          (io_config[io_config[ch].currentRouting].txBufWr - io_config[io_config[ch].currentRouting].txBufRd <
           io_config[io_config[ch].currentRouting].fifoDepth))
      {
         // get message from buffer
         commandPtr = io_config[ch].rxBuf[io_config[ch].rxBufRd & COMM_FIFO_MASK];
      #if (COMM_DEBUG == 1)
         io_config[ch].rxBuf[io_config[ch].rxBufRd & COMM_FIFO_MASK] = (uint8_t *)0xDEADBEEF;
      #endif
         io_config[ch].rxBufRd++;

      #if (COMM_LOG_ENABLE >= 1)
         msgLog(msgLog_RX, ch, commandPtr, commandPtr[0], commandPtr[COMM_POS_SEQ]);
      #endif

         if (!(io_config[ch].flags & COMM_FLAGS_RAWDATA))
         {
            // insert sequence number into ack package and send out
            txMsg[2] = commandPtr[COMM_POS_SEQ];
            msgProceedSend(ch, txMsg, 3);

         #if (COMM_LOG_ENABLE >= 1)
            msgLog(msgLog_RX_ACK, ch, NULL, 0, commandPtr[COMM_POS_SEQ]);
         #endif
         }

         if (io_config[ch].currentRouting == ch)
         {  // no gateway, commandPtr must run through interpreter()

            // LED ein
         #if(SW_LED==1)
            msgMainLoopStatus (0);
         #endif

            if (!broadcast)
            {  // don't let text output follow the current interface, when (COMM_FLAGS_RAWDATA && !COMM_FLAGS_TEXTFOLLOW)
               if (!((io_config[ch].flags & COMM_FLAGS_RAWDATA) && !(io_config[ch].flags & COMM_FLAGS_TEXTFOLLOW)))
                  currentInterface = io_config[ch].textRouting;
            }

            // filter incomming d-messages and s-messages (from other nodes)
            if ((commandPtr[1] == CMD_ASCII_DATA) || ((commandPtr[1] == CMD_CONTROLLERTYP) && commandPtr[0]>4))
            {
               msgPutFree(commandPtr); // free commandPtr
               commandPtr = NULL;
            }
            else
            {

               switch (commandPtr[1])
               {
               #if (COMM_GATEWAY == 1)
                  case CMD_GATEWAY:
                     msgGateway(ch, true); // BIOS mode
                     break;
               #endif

               #if (COMM_CMD_ENABLE == 1)
                  case CMD_DIAGNOSTIC:
                  #if (COMM_USE_CTHREAD == 1)
                     commandActive(1);
                  #endif
                     if (commandPtr[2] == 0x4F)
                        msgCmd();
                     else
                        interpreter();
                     #if (COMM_USE_CTHREAD == 1)
                        commandActive(0);
                     #endif
                     break;
               #endif

                  default:
                  #if (COMM_USE_CTHREAD == 1)
                     commandActive(1);
                  #endif
                     interpreter();
                  #if (COMM_USE_CTHREAD == 1)
                     commandActive(0);
                  #endif
                     break;
               }

               msgSend(ch, resultPtr);
               resultPtr = commandPtr; // new resultPtr needed, while commandPtr is not needed any more
               commandPtr = NULL;
            }

            // LED zuruecksetzen
         #if(SW_LED == 1)
            msgMainLoopStatus (1);
         #endif


         }
      #if (COMM_GATEWAY == 1)
         else
         {  // gateway mode; commandPtr[2] is used to identify HBBIOS answers
            if ((commandPtr[1] == CMD_GATEWAY) && (commandPtr[2]!=0xFF) && msgGateway(ch, false))
            {
               msgSend(ch, resultPtr);
               resultPtr = commandPtr; // new resultPtr needed, while commandPtr is not needed any more
               commandPtr = NULL;
            }
            else
            {
            #ifdef COMM_GATEWAY_USR_STATUSLED
               COMM_GATEWAY_USR_STATUSLED;
            #endif
               if ((commandPtr[1] == CMD_ASCII_DATA) && (io_config[ch].textRouting != ch))
                  msgSend(io_config[ch].textRouting, commandPtr);
               else
                  msgSend(io_config[ch].currentRouting, commandPtr);
            }
         }
      #endif
      } // end of if(io_config[i].interface != ifNone)

   } // end of for(i=0; i<io_config_no; i++)

#if (SW_SCRIPT == 1)
{
   static unsigned autorun_in_progress = 1;

   if (autorun_in_progress)
      autorun_in_progress = msgScript(0,0,0);   // init from msgInit()

#if (COMM_USE_CTHREAD == 1)
   commandActive(1);
#endif
   scriptPrintMsg();

   return(autorun_in_progress);
}
#else
   return(false);
#endif

}


//********************************************************************************************************
// msgGetInterfaceIndex
//--------------------------------------------------------------------------------------------------------
// Function:   search current index for a given interface type needed in msgGateway to map interface
//             names to real interface number
//--------------------------------------------------------------------------------------------------------
// Input:      interface
// Output:     index of io_config structures; else ifNone (0xFF)
//********************************************************************************************************
uint8_t msgGetInterfaceIndex (uint8_t interface)
{
   uint8_t i;

   for (i=0; i<io_config_no; i++)
      if (io_config[i].interface == interface)
         return(i);
   return (ifNone);
}


//********************************************************************************************************
// msgGetCurrentInterface
//--------------------------------------------------------------------------------------------------------
// Function:   give back the current interface to interprt (s-message)
//--------------------------------------------------------------------------------------------------------
// Input:      none
// Output:     current interface
//********************************************************************************************************
unsigned msgGetCurrentInterface(void)
{
   return (io_config[currentInterface].interface);
}


#if (COMM_GATEWAY == 1)
static uint8_t currentMode = 0x01; // BIOS

//********************************************************************************************************
// msgSetGateway
//--------------------------------------------------------------------------------------------------------
// Function:   set gateway mode
//--------------------------------------------------------------------------------------------------------
// Input:      src, dst: switch gateway from src to dst
//             cmd: 2: bidirectional; 3: unidirectional 4: text routing
//             activateGW: if true: switch to gateway mode and activate settings immediately
// Output:     error
//********************************************************************************************************
uint32_t msgSetGateway (unsigned src, unsigned dst, unsigned cmd, BOOL activateGW)
{
   uint8_t newGwSource;
   uint8_t newGwDestination;
   uint32_t error = NO_ERROR;
   unsigned n;

   newGwSource = msgGetInterfaceIndex ( (InterfaceType)src);
   newGwDestination = msgGetInterfaceIndex ( (InterfaceType)dst);

   if ((newGwSource == ifNone) || (newGwDestination == ifNone))
      error = ERROR_GATEWAY | GATEWAY_PARAMETER_ERROR;
   else  // set parameters bi-directional
      if (cmd == 4)
      {
         io_config[newGwSource].textRouting = newGwDestination;
      } else
      {
         io_config[newGwSource].gatewayRouting = newGwDestination;
         if (cmd == 2)
            io_config[newGwDestination].gatewayRouting = newGwSource;
         // activate new settings immediately when being in gateway mode
         if (activateGW)
         {
            for (n=0; n<io_config_no; n++)
               io_config[n].currentRouting = io_config[n].gatewayRouting;
            currentMode = 0x00;
         }
      }

   return(error);
}


//********************************************************************************************************
// msgGateway
//--------------------------------------------------------------------------------------------------------
// Function:   gateway interpreter; there are 4 cases:
//             1. HBBIOS mode and counter == 0 => run through gateway interpreter
//             2. HBBIOS mode and counter > 0 => generate error, because we don't know the receiver
//             3. GATEWAY mode and counter == 0 => run through gateway interpreter
//             4. GATEWAY mode and counter > 0  => decrement counter and pass to next node
//--------------------------------------------------------------------------------------------------------
// Input:      biosmode: true, if called from bios mode
// Output:     true, if resultPtr must be sendM false, if commandPtr must be send
//********************************************************************************************************
BOOL msgGateway (unsigned ch, BOOL biosMode)
{
   uint32_t error = NO_ERROR;
   unsigned len = 0;
   unsigned n, m;
   unsigned catchedRxBuffers;
   uint32_t time;
   BOOL  timeOut;

   if (biosMode)
   {
      // check command
      if (commandPtr[2] != 0)
      {
         // pass through not possible
         printf("** Gateway Error:\n");
         printf("     target is not running in gateway mode and can not\n");
         printf("     transmit gateway messages to following nodes!\n");
         set_result(ERROR_GATEWAY | GATEWAY_TRANSMIT_ERROR);
         return (true);    // send resultPtr
      }
   }
   else
   {
      // check command
      if (commandPtr[2] != 0)
      {
         // decrement phyiscal address and pass through till target has been found
         commandPtr[2]--;
         return (false);   // send commandPtr
      }
   }

   catchedRxBuffers = 0;
   time = timGetTime();
   do
   {
      msgHandleAcks();
	   for (n=0; n< io_config_no; n++)
	      if (io_config[n].rxMsg != NULL)
	         catchedRxBuffers++;
	   if (commandPtr != NULL)
	      catchedRxBuffers++;
      timeOut = (timDifference(time) > COMM_GATEWAY_TIMEOUT);
   } while (((COMM_FREE_SIZE + freeBufRd - freeBufWr - catchedRxBuffers) > 2) && !timeOut);

   if (!timeOut)
      switch ( commandPtr[3])
      {
         case 0x00:
         // redundant command deleted => use "s"
           break;

         //--------------------------------------------------------------------------
         // check or set mode
         // Note: this commands acts now as a master switch. In BIOS-Mode everything is routed 1:1
         // In gateway mode you can still route everything 1:1 or you can change the routing for each interface
         // independent. So in principle there is no real need to use the bios mode at all, it's just
         // for backward compatibility
         //--------------------------------------------------------------------------
         case 0x01:
            if (commandPtr[0] == 0x04)
            {  // Wert ermitteln
               resultPtr[8] = currentMode;
               len = 1;
            }
            else
            {
               if (commandPtr[0] == 0x05)
               {
                  switch (commandPtr[4])
                  {
                     case 0x00:  // gateway mode
                        currentMode = 0x00;  // gateway
                        for (n=0; n<io_config_no; n++)
                           io_config[n].currentRouting = io_config[n].gatewayRouting;
                        break;
                     case 0x01:
                        currentMode = 0x01;  // BIOS
                        for (n=0; n<io_config_no; n++)
                           io_config[n].currentRouting = n; // 1:1
                        break;
                     default:
                        error = ERROR_GATEWAY | GATEWAY_PARAMETER_ERROR;
                        break;
                  }
               }
               else
                  error = ERROR_GATEWAY | GATEWAY_PARAMETER_ERROR;
            }
            break;

         case 0x02:
         case 0x03:  //unidirectional settings (experimental)
         case 0x04:
            switch (commandPtr[0])
            {
               case 0x04:
                  m = 8;
                  for (n=0; n<io_config_no; n++)
                  {
                     if (commandPtr[3] == 4)
                     {
                        if (io_config[n].textRouting != n)
                        {
                           resultPtr[m++] = io_config[n].interface;
                           resultPtr[m++] = io_config[io_config[n].textRouting].interface;
                        }
                     }
                     else
                     {
                        if (io_config[n].gatewayRouting != n)
                        {
                           resultPtr[m++] = io_config[n].interface;
                           resultPtr[m++] = io_config[io_config[n].gatewayRouting].interface;
                        }
                     }
                  }
                  len = m-8;
                  break;
               case 0x06:
                  error = msgSetGateway(commandPtr[4], commandPtr[5], commandPtr[3], (currentMode == 0x00) );
                  break;

               default:
                  error = ERROR_GATEWAY | GATEWAY_PARAMETER_ERROR;
                  break;
            }
            break;

         case 0x08:  // topologie detection
         case 0x09:
            if (commandPtr[0] == 0x04)
            {
               m = 8;
               resultPtr[m++] = io_config[ch].interface;
               for (n=0; n<io_config_no; n++)
                  if (n != ch)
                     if ((commandPtr[3] == 0x08) || ((n>0) && io_config[n].txText))
                        resultPtr[m++] = io_config[n].interface;
               len = m-8;
            }
            else
               error = ERROR_GATEWAY | GATEWAY_PARAMETER_ERROR;
            break;

         default:
         #ifdef COMM_GATEWAY_USR_INTERPRETER
            COMM_GATEWAY_USR_INTERPRETER;
         #else
            error = ERROR_GATEWAY | GATEWAY_COMMAND_ERROR;
            break;
         #endif

      } // end of switch
   else
      error = ERROR_GATEWAY | GATEWAY_COMMAND_ERROR;

   set_result_len(error, len);
   resultPtr[2] = 0xFF; // mark message as answer
   return(true); // send resultPtr
} // end of msgGateway
#endif // COMM_GATEWAY == 1


#if (COMM_LOG_ENABLE >= 1)
//********************************************************************************************************
// msgLog
//--------------------------------------------------------------------------------------------------------
// Function:   used to trace the bios communication
//--------------------------------------------------------------------------------------------------------
// Input:      typ: log description
//             dst: channel, which have to be logged
//             buffer: message to log or NULL for logs without payload
//             len: length of the payload
//             seq: sequence number
// Output:     none
//********************************************************************************************************
void msgLog(unsigned typ, unsigned dst, uint8_t * volatile buffer, unsigned len, uint32_t seq)
{
   uint32_t time;
#if (COMM_LOG_ENABLE < 2)
   BOOL raw;
   unsigned mod;
   unsigned n,m;
#endif

   if ((msgLog_base == 0) || (dst == msgLog_filter))
      return;

   time = timGetTime();

#if (COMM_LOG_ENABLE < 2)
   if (msgLog_mode < 2)
   {  // log mode 0 and 1 using sprintf
      sprintf(&msgLog_ptr[0], "@%9dms %7s %9s%2d", time, typ_arr[typ], io_config[dst].name, io_config[dst].channel);
      msgLog_ptr += 0x20;

      sprintf(&msgLog_ptr[0], "#%8x P0x%8x D%7dms", seq, buffer, time-dif_arr[typ][dst]);
      msgLog_ptr += 0x20;

      if (buffer != NULL)
      {
         if (msgLog_mode == 0)
         {
            memcpy(&msgLog_ptr[0], (void *)buffer, len);
            msgLog_ptr += ((len / 0x20) * 0x20);
            mod = len % 0x20;
            if (mod != 0)
            {
               memset(&msgLog_ptr[mod], ' ', 0x20-mod);
               msgLog_ptr += 0x20;
            }
         } else
         {
            raw = (typ > 4); // use raw logging

            if (raw)
            {
               sprintf(&msgLog_ptr[0], "%4d:", len);
               msgLog_ptr += 5; m=0;
            }
            else
            {
               sprintf(&msgLog_ptr[0], "%4d:%c ", len, buffer[1]);
               msgLog_ptr += 7; m=2;
            }
            if (!raw && (buffer[1] == CMD_ASCII_DATA))
            {
               len -= 2;
               memcpy(&msgLog_ptr[0], (void *)&buffer[2], len);
               msgLog_ptr += len;
            } else
            {
               for (n=m; n<len; n++)
               {
                  if ((n % 4) == 0)
                     msgLog_ptr += 1;
                  sprintf(&msgLog_ptr[0], "%02x ", buffer[n]);
                  msgLog_ptr += 2;
               }
            }
         }
      }
      if (msgLog_mode != 0)
      {
         sprintf(&msgLog_ptr[0],"\n");
         msgLog_ptr++;
      }

      if (msgLog_ptr > (msgLog_base + msgLog_size))
      {
         strcpy(&msgLog_ptr[0x00], "^^^^^^ CONTINUE AT THE TOP ^^^^\n");
         msgLog_ptr = msgLog_base; // ring buffer
      }

      strcpy(&msgLog_ptr[0x00], "====== END OF RING BUFFER =====\n");
   } else
#endif // #if (COMM_LOG_ENABLE < 2)

   {  // log mode 2 for ultra slow systems
      // format: TTTTTTTT DDDDDDDD FF dd tt ssssssss FF SSSSSSSSSSSSSSSS => with:
      // TTTTTTTT = absolute time in ms
      // DDDDDDDD = difference to command of the same type in ms
      // FF = separator
      // dd = destination (logical interface number as defined)
      // tt = type => with: 00=rx 01=tx F1=txresend FF=broadcast EE=errGen A0=rxAck B0=rxNak A1=txAck B1=txNak 88=txDelay
      writeLong(&msgLog_ptr[0], time);
      writeLong(&msgLog_ptr[4], time-dif_arr[typ][dst]);
      msgLog_ptr[8] = 0xFF;
      msgLog_ptr[9] = dst;
      msgLog_ptr[10] = typ_hex[typ];
      writeLong(&msgLog_ptr[11], seq);
      msgLog_ptr[15] = 0xFF;
      memcpy(&msgLog_ptr[16], (void *)buffer, (len > 16 ? 16 : len));

      msgLog_ptr += 0x20;

      if (msgLog_ptr > (msgLog_base + msgLog_size))
         msgLog_ptr = msgLog_base; // ring buffer

      memset(&msgLog_ptr[0x00], 0xFF, 0x20);
   }

   dif_arr[typ][dst] = time;
}
#endif


//********************************************************************************************************
// msgMainLoopStatus
//--------------------------------------------------------------------------------------------------------
// Function:   Tell (via the status LED) what the main loop is doing:
//             <0 - at top of loop,
//             =0 - about to call interpreter(),
//             >0 - interpreter() done.used to trace the bios communication
//--------------------------------------------------------------------------------------------------------
// Input:      what: see above
// Output:     none
//********************************************************************************************************
#if(SW_LED==1)
void msgMainLoopStatus (int what)
{
#if defined(COMM_LED_STATUS_CHANNEL) && (COMM_LED_STATUS_COUNT > 0)
   static int cnt = 0;
   int on;

   if (++cnt >= ledSpeed || what > 0)
      cnt = 0;

   on = (cnt >= (ledSpeed / 2)) || (what == 0);

   if (on)
      ledON(COMM_LED_STATUS_CHANNEL);
   else
      ledOFF(COMM_LED_STATUS_CHANNEL);
#endif
}
#endif

//********************************************************************************************************
// msgLedSpeed
//--------------------------------------------------------------------------------------------------------
// Function:   change blinking speed of the LED
//--------------------------------------------------------------------------------------------------------
// Input:      div
// Output:     none
//********************************************************************************************************
void msgLedSpeed (unsigned div)
{
#if defined(COMM_LED_STATUS_CHANNEL) && (COMM_LED_STATUS_COUNT > 0)
   if (div >=1)
      ledSpeed = COMM_LED_STATUS_COUNT / div;
#endif
}


//********************************************************************************************************
// msgInit
//--------------------------------------------------------------------------------------------------------
// Function:   call once from main, directly after setting up irq, timer and MMU (if any); starts startup
//             logger (if defined)
//--------------------------------------------------------------------------------------------------------
// Input:      none:
// Output:     none
//********************************************************************************************************
void msgInit (unsigned defaultInterface)
{
   unsigned n = 0;
   unsigned m;

#if defined(COMM_LED_STATUS_CHANNEL) && (COMM_LED_STATUS_COUNT > 0)
   ledSpeed = COMM_LED_STATUS_COUNT;
#endif

   COMM_ATOM_INIT;

   freeBufRd = 0;
   freeBufWr = 0;

   // assign all buffers into free buffer queue
   for (m=0; m < COMM_FREE_SIZE; m++)
   {
      msgPutFree((uint8_t*)&msgBuffers[n]);
      n+=(COMM_BUF_SIZE / sizeof(uint32_t));
   }

   // init result and text pointer
   msgGetFree(&resultPtr);

#if (COMM_LOG_ENABLE >= 1)
#ifdef COMM_STARTUP_LOGGING
   msgLog_base = (char *)COMM_STARTUP_LOGGING;
#ifndef COMM_STARTUP_LOGGING_SIZE
   msgLog_size = 0x4000;
#else
   msgLog_size = COMM_STARTUP_LOGGING_SIZE;
#endif
   msgLog_filter = 0xFF;
   msgLog_mode = 1;
   msgLog_ptr = msgLog_base;
#endif
#endif

#if (SW_SCRIPT == 1)
   scriptInit();  // init script as first interface
#endif

   if ((defaultInterface >= COMM_INTERFACES) && (defaultInterface < ifNone))
      currentInterface = COMM_INTERFACES-1;
   else
      currentInterface = defaultInterface;

#if (SW_SCRIPT == 1)
   if (currentInterface != ifNone)
      io_config[0].textRouting = currentInterface;
#endif
}


//********************************************************************************************************
// msgRegister
//--------------------------------------------------------------------------------------------------------
// Function:   each interface must be registered, before it can be used; during testing you should switch
//             COMM_DEBUG to 1, to get error messages
//--------------------------------------------------------------------------------------------------------
// Input:      interface: this is the interface identifier used from BiosControl (e.g. 0x90 for shrdMem)
//             name: 9 char text, which shows up in interface info (D 4F 08)
//             channel: instance of this interface; it's used as parameter for the callback functions
//             fifoDepth: set 0, for system default or define in the range of the settings defined in
//                        config_comm.h
//             len: length of the payload
//             seq: sequence number
// Output:     none
//********************************************************************************************************
unsigned msgRegister(unsigned interface,         // interface + instance
                     char *name,                 // limit to 9 chars
                     unsigned channel,           // local channel, given as parameter for callbacks
                     unsigned fifoDepth,         // 0 for system default, defined in config_comm.h
                     void (*func_receive)(unsigned),  // optional, not needed for pure irq interfaces (use NULL)
                     BOOL (*func_send)(unsigned, uint8_t * volatile buffer, unsigned), // optional
                     void (*func_info)(unsigned),   // if not used as function pointer, nr of additonal arguments
                     ...
                     )

{
   unsigned n;
   va_list ptr;
   unsigned flags = 0;

   if (((unsigned)func_info > 0) && ((unsigned)func_info < 2))
   {  // used as number of addtional parameters
      va_start(ptr, func_info);
      flags = va_arg(ptr, unsigned);
      va_end(ptr);

      func_info = NULL;
   }

   // if entry (based on func_send and channel) is already valid; in this case refresh name and interface
   for (n=0; n<io_config_no; n++)
   {
      if (((io_config[n].func_send_save == func_send) || (io_config[n].func_send_save == msgDummySend)) && (io_config[n].channel == channel))
      {
         msgControlInterface(n, true);
         io_config[n].name = name;
         io_config[n].interface = interface;
         return(n);
      }
   }

   if (fifoDepth == 0)     // use default
      fifoDepth = COMM_BUF_DEPTH;

   if (fifoDepth == 0xFF)  // use all remaining buffers
      fifoDepth = (COMM_FREE_SIZE - nrOfBuffers) / 2;

#if (COMM_DEBUG == 1)
{
   unsigned error = 0;

   if (io_config_no >= COMM_INTERFACES)
      error |= 1; // too many interfaces; increase COMM_INTERFACES

   if (fifoDepth*2 + nrOfBuffers > COMM_FREE_SIZE)
      error |= 2; // not enough buffers; increase COMM_FREE_SIZE (as multiple of 2)

   if (func_send == NULL)
      func_send = msgDummySend;

   if (error != 0)
   {
      currentInterface = ifNone; // switch to broadcast
      printf("Interface %2x(%d) can't be intialized. Error Code: %d\n", interface, channel, error);
      return(error);
   }
}
#endif

   io_config[io_config_no].interface = interface;
   io_config[io_config_no].name = name;
   io_config[io_config_no].channel = channel;
   io_config[io_config_no].fifoDepth = fifoDepth;
   io_config[io_config_no].fifoDepthMax = fifoDepth;
   io_config[io_config_no].fifoDepthSend = 0;
   io_config[io_config_no].func_receive = func_receive;
   io_config[io_config_no].func_receive_save = func_receive;
   io_config[io_config_no].func_send = func_send;
   io_config[io_config_no].func_send_save = func_send;
   io_config[io_config_no].func_info = func_info;

   io_config[io_config_no].txText  = true;
   io_config[io_config_no].txAckRd = 0;
   io_config[io_config_no].txAckWr = 0;
   io_config[io_config_no].txBufRd = 0;
   io_config[io_config_no].txBufWr = 0;
   io_config[io_config_no].rxBufRd = 0;
   io_config[io_config_no].rxBufWr = 0;
   io_config[io_config_no].rxMsg = NULL;
   io_config[io_config_no].rxNak = 0;
   io_config[io_config_no].currentRouting = io_config_no;  // initial 1:1 routing
   io_config[io_config_no].gatewayRouting = io_config_no;  // initial 1:1 routing
   io_config[io_config_no].textRouting = io_config_no;  // initial 1:1 routing
   io_config[io_config_no].flags = flags;

   nrOfBuffers+=fifoDepth*2;

   return(io_config_no++); // return index needed by physical interface
}


#if (COMM_CMD_ENABLE == 1)
//********************************************************************************************************
// msgCmd
//--------------------------------------------------------------------------------------------------------
// Function:   D 4F commands:
//             D 4F 00 AAAAAAAA SSSSSSSS FF (MM)
//             => start logger at address AAAAAAAA with size of ring buffer SSSSSSSS, filter FF (optional)
//             filter can be used to ignore the messages of ONE interface (default 0xFF = no filter)
//             mode = 01 can be used to set up ASCII mode; these logs can be downloaded and viewed with a
//             text editor
//             D 4F 01 => switch off logger
//             D 4F 08 => view statistics
//             D 4F 0E II XX => insert error in tx path of interface II and hit in average each (1 << XX)
//             message
//--------------------------------------------------------------------------------------------------------
// Input:      none
// Output:     none
//********************************************************************************************************
void msgCmd(void)
{
   uint32_t error = NO_ERROR;

   unsigned i;

   if (commandPtr[0] < 4)
      error = (ERROR_DIAG | ERROR_COMMAND);
   else
      switch(commandPtr[3])
      {
      #if (COMM_LOG_ENABLE >= 1)
         case 0x00:
            msgLog_base = (char *)readLong(&commandPtr[4]);
            msgLog_size = readLong(&commandPtr[8]);
            if (commandPtr[0] <= 12)
               msgLog_filter = 0xFF; // no filter
            else
               msgLog_filter = commandPtr[12];
            msgLog_ptr = msgLog_base;
            if (commandPtr[0]>13)
               msgLog_mode = commandPtr[13];
            else
               msgLog_mode = 0;
            printf("   => logging started with filter %02x and mode %02x\n", msgLog_filter, msgLog_mode);
            break;

         case 0x01:
            printf("   => stop logging at %08X\n", msgLog_base);
            msgLog_base = 0;
            break;
      #endif

         case 0x02:
            i = msgGetInterfaceIndex(commandPtr[4]);
            if (i != ifNone)
            {
               printf("   %s interface %s\n", (commandPtr[5]) ? "enable" : "disable", io_config[i].name);
               msgControlInterface(i, commandPtr[5]);
            }
            else
               printf("   wrong parameters\n");
            break;

         case 0x08:
               printf("communication interfaces: (free buffers = %d)\n", freeBufWr-freeBufRd);
               printf("interface(fd/fm) (##/cr/gr/tr) | receive  +diff | transmit +diff | repeated +diff");
               if ((commandPtr[0]>4) && (commandPtr[4] == 1))
                  printf(" | lost     +diff | txError  +diff | rxRaw    +    diff | txRaw    +    diff | (ee)\n");
               else
                  printf("\n");
               for (i=0; i<io_config_no; i++)
               {
                  printf("%09s(%2d/%2d) (%02x/%02x/%02x/%02x) %1s %8d +%4d | %8d +%4d | %8d +%4d",
                         io_config[i].name, (uint32_t)io_config[i].fifoDepth, (uint32_t)io_config[i].fifoDepthMax,

                         (uint32_t)io_config[i].interface,
                         (uint32_t)io_config[io_config[i].currentRouting].interface,
                         (uint32_t)io_config[io_config[i].gatewayRouting].interface,
                         (uint32_t)io_config[io_config[i].textRouting].interface,
                         (io_config[i].func_receive == msgDummyReceive) ? "O" : ((io_config[i].txText) ? "|" : "X"),
                         io_config[i].rxBufWr, io_config[i].rxBufWr-io_config[i].rxBufDiff,
                         io_config[i].txBufWr, io_config[i].txBufWr-io_config[i].txBufDiff,
                         io_config[i].txRetransmit, io_config[i].txRetransmit-io_config[i].txRetransmitDiff
                         );
                  if ((commandPtr[0]>4) && (commandPtr[4] == 1))
                     printf(" | %8d +%4d | %8d +%4d | %8d +%8d | %8d +%8d | (%02x)\n",
                            io_config[i].txLost, io_config[i].txLost-io_config[i].txLostDiff,
                            io_config[i].txError, io_config[i].txError-io_config[i].txErrorDiff,
                            io_config[i].rxRaw, io_config[i].rxRaw-io_config[i].rxRawDiff,
                            io_config[i].txRaw, io_config[i].txRaw-io_config[i].txRawDiff,
                            (uint32_t)io_config[i].txErrorGen
                            );
                  else
                     printf("\n");

                  io_config[i].rxBufDiff = io_config[i].rxBufWr;
                  io_config[i].txBufDiff = io_config[i].txBufWr;
                  io_config[i].txLostDiff = io_config[i].txLost;
                  io_config[i].txErrorDiff = io_config[i].txError;
                  io_config[i].rxRawDiff = io_config[i].rxRaw;
                  io_config[i].txRawDiff = io_config[i].txRaw;
                  io_config[i].txRetransmitDiff = io_config[i].txRetransmit;
                  if ((commandPtr[0]>4) && (commandPtr[4] == 1))
                     if ((void *)io_config[i].func_info != NULL)
                        io_config[i].func_info(io_config[i].channel);
               }
               printf("\n");
            break;

      #if (COMM_ERROR_GEN == 1)
         case 0x0E:
            if (commandPtr[4] < io_config_no)
            {
               if ((commandPtr[5] > 0) && (commandPtr[5] < 30))
               {
                  io_config[commandPtr[4]].txErrorGen = commandPtr[5];
                  io_config[commandPtr[4]].txErrorPat = (1 << commandPtr[5]) -1;
               }
               else
                  io_config[commandPtr[4]].txErrorGen = 0;
            }
            break;
      #endif

         default:
            error = (ERROR_DIAG | ERROR_COMMAND);
            break;
      }

   set_result(error);
}
#endif   // (COMM_CMD_ENABLE == 1)

