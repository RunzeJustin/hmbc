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

#ifndef _CAN_H
#define _CAN_H

//****************************************************************************
// @Definition of a structure for the CAN data
//****************************************************************************

// The following data type serves as a software message object. Each access to
// a hardware message object has to be made by forward a pointer to a software
// message object (TCAN_Obj). The data type has the following fields:
//
// ubMsgCfg:
// this byte has the same structure as the message configuration register of a
// hardware message object. It contains the "Data Lenght Code" (DLC), the "Extended
// Identifier" (XTD) and the "Message Direction" (DIR, read only access).
//
//         7     6     5      4    3     2     1     0
//      |-----------------------------------------------|
//      |        DLC            | DIR | XTD |  0  |  0  |
//      |-----------------------------------------------|
//
// ulArbitr:
// this field is four bytes long and contains either the 11-bit identifier
// or the 29-bit identifier (as a HEX-Value)
//
// ubData[8]:
// 8 bytes containing the data of a frame
//
//

typedef struct
{
   UINT8  ubMsgObjNr; // Message Object Nr. (1...31)
   UINT8  ubMsgCfg;   // 8-bit Message Configuration Register
   UINT32 ulArbitr;   // standard (11-bit)/extended (29-bit) identifier
   UINT32 ulMaskr;    // standard (11-bit)/extended (29-bit) mask
   UINT8  ubData[8];  // 8-bit Data Bytes
}  TCAN_Obj;

//*--------------------------------------------------------------------------*
//* Defines for ErrorMsgs                                                    *
//*--------------------------------------------------------------------------*

// Sub Error Numbers (Bit 31-24=Modul, 23-0=Sub-Errornumber)
#define ERROR_CAN_WRONG_OBJ      0x00000100U
#define ERROR_CAN_NO_NEW_OBJ     0x00000200U
#define ERROR_CAN_HW             0x00000300U
#define ERROR_CAN_BAUDRATE_ERROR 0x00000400U
#define ERROR_KWP2000_TIME_OUT   0x00001000U
#define ERROR_KWP2000_PROTOCOL   0x00002000U
#define ERROR_KWP2000_RESPONSE   0x00004000U

//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

// High-Level
extern BOOL CAN0_Init   (UINT32 Baudrate);
extern void CAN0_WR     (void);
extern void CAN0_RD     (void);

extern BOOL CAN1_Init   (UINT32 Baudrate);
extern void CAN1_WR     (void);
extern void CAN1_RD     (void);

extern BOOL CAN2_Init   (UINT32 Baudrate);
extern void CAN2_WR     (void);
extern void CAN2_RD     (void);

extern BOOL CAN3_Init   (UINT32 Baudrate);
extern void CAN3_WR     (void);
extern void CAN3_RD     (void);

// Bios Message
#if CP_CMDDEV_CAN
   extern void    CAN_vConfigMsgObj ( UINT8 ObjNr,   TCAN_Obj *pstObj);
   extern BOOL    CAN_bRequestMsgObj( UINT8 ObjNr);
   extern UINT32  Send_CAN_Data     ( UINT32 Laenge,  UINT8 *Buffer);
#endif

//*--------------------------------------------------------------------------*
//* Defines                                                                  *
//*--------------------------------------------------------------------------*

#define CAN_MAX_RECEIVE_MSG   20
#define CAN_MAX_SEND_MSG      20

#define CAN_MAX_OBJ           127 // letztes cAN-Object bei CAN3

//*--------------------------------------------------------------------------*
//* Makros                                                                   *
//*--------------------------------------------------------------------------*

#define TEST_MSGCOUNTER {if (CAN_ReceiveMsgCount>CAN_MAX_RECEIVE_MSG-1)CAN_ReceiveMsgCount=CAN_MAX_RECEIVE_MSG-1;}
#define INC_MSGCOUNTER  {CAN_ReceiveMsgCount++;}

//*--------------------------------------------------------------------------*
//* Defines Registeraccess                                                   *
//*--------------------------------------------------------------------------*

// Structure for a single CAN object

typedef struct
{
   UINT8    B;                // Message Data
   UINT8    Fuellbytes[3];
} candat_t;

struct can_a_obj
{
   candat_t FCNnMmDATx[8];    // Message Data 0 .. 7              ; Offset 0x1000

   UINT8    FCNnMmDTLGB;      // Data Length Register             ; Offset 0x1020
   UINT8    Fuellbytes1[3];

   UINT8    FCNnMmSTRB;       // Message Configuration Register   ; Offset 0x1024
   UINT8    Fuellbytes2[27];  // zum Auffuellen auf 64 Bytes
};

struct can_b_obj
{
   UINT16   FCNnMmMID0H;      // Lower Arbitration Register       ; Offset 0x9028
   UINT8    Fuellbytes1[6];

   UINT16   FCNnMmMID1H;      // Upper Arbitration Register       ; Offset 0x9030
   UINT8    Fuellbytes2[6];

   UINT16   FCNnMmCTL;        // Message Control Register         ; Offset 0x9038
   UINT8    Fuellbytes3[46];  // zum Auffuellen auf 64 Bytes
};

#define FCN3_base    0xFF4E0000


// Messageobjekte im Prozessor (memory mapped)
#define CAN3_A_OBJ ((volatile struct can_a_obj *) (FCN3_base + 0x1000))
#define CAN3_B_OBJ ((volatile struct can_b_obj *) (FCN3_base + 0x9028))

// Bits setzen und loeschen
//* refer to datasheet 25.6
#define BIT_CLEAR(A)    A
#define BIT_SET(A)      A<<8


#endif // _CAN_H

