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

#ifndef _INIC_H_
#define _INIC_H_

#define INIC_DEBUG_ON    0        // activates special functions for message trace


// Defines -----------------------------------------------------------------
#define  INIC_ACCESS_I2C          0x00U     // Inic mode communication via I2C
#define  INIC_ACCESS_MLB          0x01U     // Inic mode communication via MLB (Media Local Bus)
#define  INIC_ACCESS_FLASH        0x02U     // Inic start in flash mode

// INIC device error codes
#define  ERR_INIC_NO_ERROR        0x00000000U
#define  ERR_INIC_OPEN            0x00000001U
#define  ERR_INIC_TXIO            0x00000002U  // error on control port tx access
#define  ERR_INIC_RXIO            0x00000004U  // error on control port rx access
#define  ERR_INIC_IO              0x00000006U  // interface access error
#define  ERR_INIC_PROT            0x00000008U  // protocol error
#define  ERR_INIC_BUFCREATE       0x00000010U  // error with creation of Buffer
#define  ERR_INIC_INTINSTALL      0x00000020U  // error with installing IR-Handler
#define  ERR_INIC_NOTINIT         0x00000040U  // Inic functionality not initialized
#define  ERR_INIC_CMD_TIMEOUT     0x00000080U  // Inic Response timeout
#define  ERR_INIC_BAD_FLASH_PARAM 0x00000100U  // Bad Parameters for FlashCommand
#define  ERR_INIC_BAD_FLASH_CRC   0x00000200U  // CRC-Error after Flashing
#define  ERR_INIC_BAD_IPF_FORMAT  0x00000400U
#define  ERR_INIC_SEL_COMPORT     0x00000800U  // selected com port not implemented
#define  ERR_INIC_STARTUP_FAILED  0x00001000U  // inic does not send Startup-Msg (firmware missing)
#define  ERR_INIC_COMMAND         0x00010000U  // Error with command syntax
#define  ERR_INIC_PARAM_INVALID   0x00020000U  // Error with invalid parameters given
#define  ERR_INIC_ICM_TIMEOUT     0x00040000U  // No response on ICM-command or MCM-Comand
#define  ERR_INIC_ICM_FKTID       0x00041000U  // Funktion ID Error on ICM-command or MCM-Comand




#define  INICTGTDEVTYP_LOGICAL   0x00
#define  INICTGTDEVTYP_PHYSICAL  0x01
#define  INICTGTDEVTYP_BROADCAST 0x02
#define  INICTGTDEVTYP_GROUP     0x03
#define  INICTGTDEVTYP_ALTERNAT  0x04

// Fifo Protocol Header definitions
#define  INIC_FPH_FIFONO_MCM  (0x00 << 3)
#define  INIC_FPH_FIFONO_MDP  (0x01 << 3)
#define  INIC_FPH_FIFONO_ICM  (0x02 << 3)
#define  INIC_FPH_FIFONO_ALL  (0x03 << 3)
#define  INIC_FPH_FIFONO_MASK (0x03 << 3)

#define  INIC_FPH_FIFOMT_CMD  (0x00 << 1)
#define  INIC_FPH_FIFOMT_STS  (0x01 << 1)
#define  INIC_FPH_FIFOMT_DAT  (0x02 << 1)
#define  INIC_FPH_FIFOMT_MASK (0x03 << 1)

#define  INIC_FPH_SCF0P_LST   0
#define  INIC_FPH_SCF0P_EXT   1

#define  OFFSET_INICMSG_PML       0
#define  OFFSET_INICMSG_PMH       2
#define  OFFSET_INICMSG_PMH_PMHL  2
#define  OFFSET_INICMSG_PMH_FPH   3

// mask for following control bytes
//#define  SCF0P            0x01U        // Syntax Control Field present



#define MSG_TYPE_ERROR                    0xFFU    // internal value for MsgType when protocol error detected

//
// definitions for RMCK settings
//
#define RMCK_FS_DISABLED   0x00U
#define RMCK_64FS          0x01U
#define RMCK_128FS         0x02U
#define RMCK_256FS         0x03U
#define RMCK_384FS         0x04U
#define RMCK_512FS         0x05U
#define RMCK_768FS         0x06U
#define RMCK_1024FS        0x07U
#define RMCK_1536FS        0x08U



// Fifo Data Header definitions
#define MOSTMSG_MCM_DATALEN               11U      // with OS81050 standard most format
#define MOSTMSG_MDP_DATALEN               47U      // with OS81050 packet most format

#define INIC_MCM_LEN                      24U      // max .length Portmessage
#define INIC_MDP_LEN                      60U      // max .length Portmessage

#define INIC_MCM_STARTOFDATA              12U      // Position vom 1 Nutzbyte in Portmessage
#define INIC_MDP_STARTOFDATA              10U      // Position vom 1 Nutzbyte in Portmessage


// FktIDs ---------------------------
#define  INIC_FKTID_OPENPORT              0x400

// Most addresses ---------------------------
#define  MOST_BROADCAST_GROUPADDR         0x03C8    // used for broadcastmsg to find communication port

// Makros
#define  INIC_GET_PML(pml)                xx

// Types -------------------------------------------------------------------


typedef enum
{
   inic_CtrlMsgBuf = 0,
   inic_MostMsgBuf = 1,
   inic_MostMCMBuf = 2,
} inic_MsgBuf_t;


typedef struct
{
   UINT8 SCF0P    : 1;
   UINT8 FIFOMt   : 2;
   UINT8 FIFONo   : 2;
   UINT8          : 3;
} TInicFifoPH;

typedef struct
{
   UINT8 SCIF1P     : 1;
   UINT8            : 3;
   UINT8 RetryP     : 1;
   UINT8 PrioP      : 1;
   UINT8 HandleP    : 1;
   UINT8 TimeSP     : 1;
} TInicSCF0;

typedef struct
{
   UINT8 SCIF2P     : 1;
   UINT8            : 3;
   UINT8 FunAdrP    : 1;
   UINT8 TgtDevIDP  : 1;
   UINT8 SrcDevIDP  : 1;
   UINT8 TgtDevTypP : 1;
} TInicSCF1;

typedef struct
{
   UINT8            : 1;
   UINT8 TxCancel   : 1;
   UINT8 TxRetry    : 1;
   UINT8            : 4;
   UINT8 SyncC      : 1;
} TInicFifoCommandMessage;

typedef struct
{
   UINT8            : 1;
   UINT8 AutoCan    : 1;
   UINT8 MsgStatus  : 4;
   UINT8 SlotAv     : 1;
   UINT8 SyncS      : 1;
} TInicFifoStatusMessage;

typedef struct
{
   UINT32   TimeS;
   UINT8    Handle;
   UINT8    Prio;
   UINT16   Retry;
   UINT8    TgtDevTyp;
   UINT16   SrcDevID;
   UINT16   TgtDevID;
   UINT16   FunAdr;
} TInicFifoDH;

typedef struct
{
   UINT16   length;
} TInicPortMessageLength;

typedef struct
{
   UINT8 length;
} TInicPortMessageHeaderLength;

typedef struct
{
   UINT32 Length        : 12;
   UINT32 TelID         : 4;
   UINT32 OpType        : 4;
   UINT32 FuncID        : 12;
} TInicPortMessageBodyCMF;

typedef struct
{
   TInicPortMessageBodyCMF cmf;
   UINT8                   Data[45];
} TInicPortMessageBodyICM;

typedef union
{
   TInicFifoStatusMessage  Status;
   TInicFifoCommandMessage Command;
} TInicFifoMessageField;

typedef struct
{
  TInicFifoPH  fph;
  TInicFifoMessageField fmf;
} TInicPortMessageHeaderBodyICM;

typedef union
{
   TInicPortMessageHeaderBodyICM icm;
} TInicPortMessageHeaderBody;

typedef struct
{
   TInicPortMessageHeaderLength     pmhl;
   TInicPortMessageHeaderBody       pmhb;
} TInicPortMessageHeader;

typedef union
{
   TInicPortMessageBodyICM icm;
} TInicPortMessageBody;

typedef struct
{
   TInicPortMessageLength  pml;
   TInicPortMessageHeader  pmh;
   TInicPortMessageBody    pmb;
} TInicPortMessage;

//ICM:   .{pml}.
//       .{pmh.pmhl.length}.
//       .{pmh.pmhb.icm.fph}.
//       .{pmb.icm.{cmf.{FuncID.OpType.TelID.Length}}}
//       .{pmb.icm.Data[0..44]}

// Prototypes --------------------------------------------------------------


extern   void   inicCommand (void);

extern   UINT32 inicProceedNewMsg (void);


// added for use with Gateway

extern   UINT16 inicUMostTrgtAddr;         // TrgtAddr to communicate with UMost

extern   UINT32 inicConfig(void);

extern   BOOL   mostInitialized;            // will be set in MOSTInit
extern   BOOL   inicHwInitialized;          // flag set after inicHwInit was done

extern   BOOL   inicTestIfAckMsg(UINT8 *buf);

extern   UINT32 inicSendCommand(UINT8 *buf);

extern   UINT32 inicSendAndWaitAck(inic_MsgBuf_t MsgBuf, UINT8 *buf);

extern   UINT32 inicReadReqMsgs(void);

extern   UINT32 inicWaitForAnswer(void);

extern   UINT32 inicNWStartup(void);


#endif // #ifndef _INIC_H_
