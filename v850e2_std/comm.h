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

#ifndef _comm_h
#define _comm_h

#ifdef USE_COMM_SBC
#include "comm_sbc.h"
#endif

#ifdef USE_COMM_DBC
// this variant is used for old Jacinto/Davinci Porjects only!
#include "comm_dbc.h"
#endif

#if ((!defined USE_COMM_SBC) && (!defined USE_COMM_DBC))
// assume MBC

typedef struct
{
   uint32_t    rxRaw;      // received bytes
   uint32_t    rxBufWr;    // received packets (not proceeded)
   uint32_t    rxBufRd;    // received packets (proceeded => rxBufWr-rxBufRd = number of packets in buffer)
   uint32_t    rxIndex;    // number of received bytes of an incomplete packet
   uint32_t    txAckWr;    // number of received ACH/NAK (not proceeeded)
   uint32_t    txAckRd;    // number of received ACH/NAK (proceeeded)
   uint32_t    txRaw;      // transmitted bytes
   uint32_t    txBufWr;    // transmitted packets (not sent)
   uint32_t    txBufRd;    // transmitted packets (sent => txBufWr-txBufRd = number of packets in queue)
} msgCounters_t;

//--------------------------------------------------------------------------
// extern functions
//--------------------------------------------------------------------------
extern void msgSendASCII(char * pmessage);
extern void msgBroadCast(uint8_t * volatile buffer, BOOL logOff);
extern void msgReceive (unsigned ch, uint8_t * volatile buffer, unsigned len);
extern void msgReceiveRaw (unsigned ch, uint8_t * volatile buffer, unsigned len);
extern void msgControlInterface(unsigned ch, BOOL enable);
extern BOOL msgPoll(BOOL broadcast);
extern unsigned msgGetCurrentInterface(void);
extern msgCounters_t * msgGetCounters(unsigned ch);
extern uint32_t msgSetGateway (unsigned src, unsigned dst, unsigned cmd, BOOL activateGW);
extern void msgInit (unsigned defaultInterface);
extern unsigned msgScript(uint32_t auto_packet, uint32_t auto_flash, uint32_t auto_delay);
extern void msgLedSpeed (unsigned div);
extern void msgLog(unsigned typ, unsigned dst, uint8_t * volatile buffer, unsigned len, uint32_t seq);
extern unsigned msgRegister(unsigned interface,  // interface + instance
                     char *name,                // limit to 9 chars
                     unsigned channel,           // local channel, given as parameter for callbacks)
                     unsigned fifoDepth,         // 0 for system default, defined in config_comm.h
                     void (*func_receive)(unsigned),  // optional, not needed for pure irq interfaces (use NULL)
                     BOOL (*func_send)(unsigned, uint8_t * volatile buffer, unsigned), // mandatory
                     void (*func_info)(unsigned),   // optional info function or number of additional parameters
                     ...
                     );
extern void msgCmd(void);

extern char * msgLog_base;
extern unsigned msgLog_size;

#define COMM_MBC_MAX   520 // max size of an MBC encoded packet

#endif

#include "config_comm.h"

// use these entries to log external functions within the comm message tracer
// example: msgLog(msgLog_Ext0, ch(received during msgRegister()), NULL or buffer to print, 0 or len of buffer, optinal pointer or ms timer))
#define msgLog_Ext0 13
#define msgLog_Ext1 14
#define msgLog_Ext2 15
#define msgLog_Ext3 16
#define msgLog_Ext4 17
#define msgLog_Ext5 18
#define msgLog_Ext6 19
#define msgLog_Ext7 20

#define COMM_FLAGS_ROUTE         1  // don't pass data to interpreter; routing only
#define COMM_FLAGS_RAWDATA       2  // send data raw as HBBIOS message (no encoding); don't send broadcasts; don't send ACK, NAK, ...
#define COMM_FLAGS_AUTOACK       4  // send auto ack (only together with RAWDATA)
#define COMM_FLAGS_TEXTFOLLOW    8  // let text output interface follow the current interface (only together with RAWDATA)

#endif   // ifndef _comm_h

