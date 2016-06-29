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
//* Modul        : buffer.c (V1.0)                                        *
//*************************************************************************
//* Overview     : Implementation of FIFO and LIFO buffers.               *
//*************************************************************************


//*-----------------------------------------------------------------------*
//* Include files                                                         *
//*-----------------------------------------------------------------------*

#include "config.h"
#include "define.h"

#define _BUFFER_NO_EXTERNALS_ // no declaration of externals
#include "buffer.h"
#include <stdlib.h>
#include "tool.h"

#if (CP_BUFFER)

//*-----------------------------------------------------------------------*
//* Prototypes                                                            *
//*-----------------------------------------------------------------------*

// global functions
TBuffer  *bufCreate  (UINT32 size);
void 	   bufDelete   (TBuffer *buffer);
BOOL 		bufPutFiFo  (TBuffer *buffer, UINT8 data);
BOOL 		bufGet      (TBuffer *buffer, UINT8 *data);
UINT32   bufItems    (TBuffer *buffer);
UINT32   bufSpace    (TBuffer *buffer);
void 	   bufFlush    (TBuffer *buffer);

// local functions


//*-----------------------------------------------------------------------*
//* Global functions                                                      *
//*-----------------------------------------------------------------------*


//*************************************************************************
//* Function     : bufCreate                                              *
//*************************************************************************
//* Description  : Creates a buffer.                                      *
//*                                                                       *
//* Parameter    : size     - number of items to be stored                *
//*                                                                       *
//* Returnvalue  : pointer to the buffer structure if sucessful;          *
//*                NULL if not                                            *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

TBuffer *bufCreate(UINT32 size)
{
   TBuffer *temp;

   // check size
   if (size & 1)
   {  printf("     Malloc error, only even size allowed\n");
      return(NULL);
   }

   // obtain memory for the buffer structure
   temp = malloc(sizeof(TBuffer));
   if(NULL == temp) // allocation failed
   {
      return(NULL);
   }

   // obtain memory for the buffer
   temp->buf = malloc(size);
   if(NULL == temp->buf) // allocation failed
   {
      free(temp);
      return(NULL);
   }

   // initialize buffer structure
   temp->size = size;
   temp->items = 0U;
   temp->first = 0U;
   temp->last = 0U;

   return(temp);
}


//*************************************************************************
//* Function     : bufDelete                                              *
//*************************************************************************
//* Description  : Deletes a buffer.                                      *
//*                                                                       *
//* Parameter    : -none-                                                 *
//*                                                                       *
//* Returnvalue  : -none-                                                 *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

void bufDelete(TBuffer *buffer)
{  if (buffer != NULL)
   {  // free allocated memory
      free(buffer->buf);
      free(buffer);
   }
}


//*************************************************************************
//* Function     : bufPutFiFo                                             *
//*************************************************************************
//* Description  : Puts data at the end of the queue.                     *
//*                                                                       *
//* Parameter    : buffer - pointer to the buffer structure               *
//*                data   - pointer to the data to store                  *
//*                                                                       *
//* Returnvalue  : TRUE, if sucessful; FALSE if buffer full               *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

BOOL bufPutFiFo(TBuffer *buffer, UINT8 data)
{
   // check for space on the queue
   if(buffer->items == buffer->size) // buffer full
   {
      return(FALSE);
   }

   // put data on the queue
   ((UINT8 *)buffer->buf)[buffer->last] = data;

   buffer->items++;

   // increase last to put the data on the queue
   buffer->last++;
   if(buffer->last == buffer->size)
   {
      buffer->last = 0U;
   }

   return(TRUE);
}


//*************************************************************************
//* Function     : bufGet                                                 *
//*************************************************************************
//* Description  : Gets data from the queue.                              *
//*                                                                       *
//* Parameter    : buffer - pointer to the buffer structure               *
//*                data   - pointer to the store location                 *
//*                                                                       *
//* Returnvalue  : TRUE, if sucessful; FALSE if buffer empty              *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

BOOL bufGet(TBuffer *buffer, UINT8 *data)
{

   // check for items on the queue
   if(0U == buffer->items) // buffer empty
   {
      return(FALSE);
   }

   // get data from the queue
   *data = ((UINT8 *)buffer->buf)[buffer->first];

   buffer->items--;

   // increase first to put the data on the queue
   buffer->first++;
   if(buffer->first == buffer->size)
   {
      buffer->first = 0U;
   }

   return(TRUE);
}


//*************************************************************************
//* Function     : bufItems                                               *
//*************************************************************************
//* Description  : Returns the number of items on the queue.              *
//*                                                                       *
//* Parameter    : buffer - pointer to the buffer structure               *
//*                                                                       *
//* Returnvalue  : number of items on the queue                           *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

UINT32 bufItems(TBuffer *buffer)
{
   return(buffer->items);
}


//*************************************************************************
//* Function     : bufSpace                                               *
//*************************************************************************
//* Description  : Returns the number of free places in the queue.        *
//*                                                                       *
//* Parameter    : buffer - pointer to the buffer structure               *
//*                                                                       *
//* Returnvalue  : number of free places on the queue                     *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      :                                                        *
//*-----------------------------------------------------------------------*
//* Quality      : ( ) not tested  ( ) partly tested  (X) fully tested    *
//*************************************************************************

UINT32 bufSpace(TBuffer *buffer)
{
   return(buffer->size - buffer->items);
}


//*************************************************************************
//* Function     : bufFlush                                               *
//*************************************************************************
//* Description  : Flushes the contents of the queue.                     *
//*                                                                       *
//* Parameter    : buffer - pointer to the buffer structure               *
//*                                                                       *
//* Returnvalue  : -none-                                                 *
//*                                                                       *
//* Changed Var. : -none-                                                 *
//*                                                                       *
//* Comment      : -none-                                                 *
//*-----------------------------------------------------------------------*
//* Quality      : (x) not tested  ( ) partly tested  ( ) fully tested    *
//*************************************************************************

void bufFlush(TBuffer *buffer)
{
   buffer->items = 0U;
   buffer->first = 0U;
   buffer->last = 0U;
}


#endif //  #if (CP_BUFFER)

