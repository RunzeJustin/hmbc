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


//*************************************************************************
//* Modul        : buffer.h (V1.0)                                        *
//*************************************************************************
//* Overview     : Defines and prototypes for the buffer functions        *
//*                implemented in "buffer.c"                              *
//*************************************************************************


#ifndef _BUFFER_H_ // avoid multiple inclusions
#define _BUFFER_H_


//*-----------------------------------------------------------------------*
//* Type Definitions                                                      *
//*-----------------------------------------------------------------------*

typedef struct
{
   UINT32 size;     	// number of items that can be buffered
   UINT32 items;    	// topic number of items
   UINT32 first;    	// pointer to first item in queue
   UINT32 last;     	// pointer to last item in queue
   void   *buf;     	// pointer to the buffer
} TBuffer;


//*-----------------------------------------------------------------------*
//* Prototypes                                                            *
//*-----------------------------------------------------------------------*

#ifndef _BUFFER_NO_EXTERNALS_ // no externals for "buffer.c"

extern TBuffer *bufCreate     (UINT32 size);
extern void    bufDelete      (TBuffer *buffer);
extern BOOL    bufPutFiFo     (TBuffer *buffer, UINT8 data);
extern BOOL    bufGet         (TBuffer *buffer, UINT8 *data);
extern UINT32  bufItems       (TBuffer *buffer);
extern UINT32  bufSpace       (TBuffer *buffer);
extern void    bufFlush       (TBuffer *buffer);


#endif // #define _BUFFER_H_

#endif // #ifndef _ADC_H_

