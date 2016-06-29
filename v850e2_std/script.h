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
//   Project:          BIOSControl SH4a
//   (c) copyright     Harman-Becker Automotive Systems
//   Author:           Uwe Schumacher (EEHB, Tel 1550)
//-----------------------------------------------------------------------



//-----------------------------------------------------------------------
// Modul        : script
//-----------------------------------------------------------------------
// Overview     : script communication driver for the SHx controller.
//-----------------------------------------------------------------------
#ifndef _SCRIPT_H
#define _SCRIPT_H

//-----------------------------------------------------------------------
// Type Definitions
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
#define SCRIPT_NO_COMMAND                 0x0000
#define SCRIPT_START_FOUND                0x0001
#define SCRIPT_COMMAND_FOUND              0x0002
#define SCRIPT_FINISHED_OK                0x0003

#define SCRIPT_ERROR_INVALID_ADDRESS      0x1000
#define SCRIPT_ERROR_NOSCRIPT_FOUND       0x2000
#define SCRIPT_ERROR_PARSER               0x2100
#define SCRIPT_ERROR_UNEXPECTED_COMMAND   0x2200
#define SCRIPT_ERROR_INVALID_EXPRESSION   0x2300
#define SCRIPT_ERROR_NESTED_LOOP          0x2400
#define SCRIPT_ERROR_RESULT               0x2500


//-----------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------
#if ((! defined USE_COMM_DBC) && (! defined USE_COMM_SBC))
   extern void scriptInit(void);
   extern void scriptMsgReceive(unsigned ch);
   extern BOOL scriptMsgSend (unsigned channel, uint8_t * buffer, unsigned len_seq);
   extern uint32_t scriptStart (uint32_t addr);
   extern BOOL scriptReady (void);
   extern void scriptPrintMsg(void);
#else
   extern BOOL scriptMsgReceive(XINT8PTR buffer);
#endif

   extern void scriptCmd(void);

#endif // #ifndef _SCRIPT_H



