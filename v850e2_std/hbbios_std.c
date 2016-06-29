/*
 *-----------------------------------------------------------------------
 *
 *        _____   _____   _   _    ___
 *       |  ___| |  ___| | | | |  |   `
 *       | |_    | |_    | |_| |  | [] |
 *       |  _|   |  _|   |  _  |  |  _ <
 *       | |___  | |___  | | | |  | |_| |
 *       |_____| |_____| |_| |_|  |____/
 *
 *
 *   Project:          BIOSControl SH
 *   (c) copyright     Harman-Becker Automotive Systems
 *-----------------------------------------------------------------------
 */

#include "define.h"    // global definitions
#include "config.h"
#include "global.h"    // global definitions
#include "hbbios.h"

#include <stdarg.h>

//#include "interprt.h"

// Local Functions --------------------------------------------------------


//*************************************************************************
// Implementation
//*************************************************************************/

//*************************************************************************
// writeWord()                                                   2002-07-18
//-------------------------------------------------------------------------
// Description:
//    writes a Word to the specified address (big-endian).
//
// Parameter:
//    addr  - address where the Word is written to
//    ldata - the Word to be written (native endian)
//
//*************************************************************************/
void writeWord(volatile void* addr, uint16_t wdata)
{
   volatile uint8_t *ptr = addr;

   ptr[1] = wdata;
   ptr[0] = wdata >> 8;
}

//*************************************************************************
// writeLong()                                                   2002-07-18
//-------------------------------------------------------------------------
// Description:
//    writes a LongWord to the specified address (big-endian).
//
// Parameter:
//    addr  - address where the LongWord is written to
//    ldata - the LongWord to be written (native endian)
//
//*************************************************************************/
void writeLong(volatile void* addr, uint32_t ldata)
{
   unsigned i;
   volatile uint8_t *ptr = addr;

   ptr += 4;
   for (i=4 ; i>0 ; i-=1)
   {
      *--ptr = ldata;
      ldata >>= 8;
   }
}

//*************************************************************************
// readWord()                                                   2002-07-18
//-------------------------------------------------------------------------
// Description:
//    reads a Word from the specified address (big-endian).
//
// Parameter:
//    addr  - address where the Word is read from.
//
//*************************************************************************/
uint16_t readWord(volatile void* addr)
{
   uint16_t temp;
   volatile uint8_t *ptr = addr;

   temp = (ptr[0] << 8)
        |  ptr[1];

   return(temp);
}

//*************************************************************************
// readLong()                                                   2002-07-18
//-------------------------------------------------------------------------
// Description:
//    reads a LongWord from the specified address (big-endian).
//
// Parameter:
//    addr  - address where the LongWord is read from.
//
//*************************************************************************/
uint32_t readLong(volatile void* addr)
{
   uint32_t temp, i;
   volatile uint8_t *ptr = addr;


   for (temp=0, i=4 ; i ; i-=1)
      temp = (temp << 8) + *ptr++;

   return(temp);
}


//*************************************************************************
// swapWord()                                                    2002-07-18
//-------------------------------------------------------------------------
// Description:
//    swaps the bytes of a Word. i.e: 0x1234 to 0x3412
//
// Parameter:
//    ldata - the LongWord to be swapped
//
// Return:
//    value   - the swapped LongWord
//*************************************************************************/
uint16_t swapWord(uint16_t wdata)
{
   uint16_t temp;

   temp =   ((wdata & 0xFF00U) >> 8 )
          | ((wdata & 0x00FFU) << 8 );
   return(temp);
}

//*************************************************************************
// swapLong()                                                    2002-07-18
//-------------------------------------------------------------------------
// Description:
//    swaps the bytes of a LongWord. i.e: 0x12345678 to 0x78563412
//
// Parameter:
//    ldata - the LongWord to be swapped
//
// Return:
//    value   - the swapped LongWord
//*************************************************************************/
uint32_t swapLong(uint32_t ldata)
{
   uint32_t temp;

   temp =   ((ldata & 0xFF000000U) >> 24)
          | ((ldata & 0x00FF0000U) >> 8 )
          | ((ldata & 0x0000FF00U) << 8 )
          | ((ldata & 0x000000FFU) << 24);
   return(temp);
}


//*************************************************************************
// getMODE()                                                     2001-03-21
//-------------------------------------------------------------------------
// Description:
//    Gets the corresponding bit(s) of the Mode register.
//
// Parameter:
//    bitname - bit name within the Mode register
//
// Return:
//    value   - value of the corresponding bit(s)
//*************************************************************************/
uint32_t getMode(uint32_t bitname)
{
   uint32_t value;
   switch (bitname)
   {
      case MODE_TRM:   // Text Result Mode
         value = (*modePtr & 0x00000003) >> MODE_TRM; // 2 Bit
         break;

      case MODE_CM:    // Command Mode
         value = (*modePtr & 0x0000000C) >> MODE_CM;  // 2 Bit
         break;

      case MODE_EM:    // Error Mode
         value = (*modePtr & 0x00000010) >> MODE_EM;  // 1 Bit
         break;

      case MODE_SA:    // Send ASCII
         value = (*modePtr & 0x00000020) >> MODE_SA;  // 1 Bit
         break;

      case MODE_IC:    // Internal Command
         value = (*modePtr & 0x00000040) >> MODE_IC;  // 1 Bit
         intern_cmd = (value>0 ? true : false);
         break;

      case MODE_SD:    // Send Debug
         value = (*modePtr & 0x00000080) >> MODE_SD;  // 1 Bit
         break;

      case MODE_WKU:   // Wake Up
         value = (*modePtr & 0x00001F00) >> MODE_WKU; // 5 Bit
         break;

      case MODE_FE:    // Flash Erase
         value = (*modePtr & 0x00010000) >> MODE_FE;  // 1 Bit
         break;

      case MODE_STOP:  // Stop
         value = (*modePtr & 0x00020000) >> MODE_STOP; // 1 Bit
         break;

      case MODE_FST:   // Flash STatus
         value = (*modePtr & 0x000C0000) >> MODE_FST; // 2 Bit
         break;

      case MODE_ZIP:   // Zip Module
         value = (*modePtr & 0x00100000) >> MODE_ZIP; // 2 Bit
         break;

      case MODE_SER0:  // Serial Interface 0
         value = (*modePtr & 0x00200000) >> MODE_SER0; // 1 Bit
         break;

      case MODE_SER1:  // Serial Interface 1
         value = (*modePtr & 0x00400000) >> MODE_SER1; // 1 Bit
         break;

      case MODE_SER2:  // Serial Interface 2
         value = (*modePtr & 0x00800000) >> MODE_SER2; // 1 Bit
         break;

      case MODE_VW:   // Verified Write      (1 Bit)
         value = (*modePtr & 0x01000000) >> MODE_VW;   // 1 Bit
         break;

      default:
         printf("** Error reading Mode Register @ 0x%08X",(uint32_t)modePtr);
         printf("     desired bit is not defined!");
         value = 0U;
   }

   return(value);
}


//*************************************************************************
// getModeReg()                                                  2002-06-07
//-------------------------------------------------------------------------
// Description:
//    Gets the complete mode register.
//
// Parameter:
//    -none-
//
// Return:
//    mode register contents
//*************************************************************************/
uint32_t getModeReg(void)
{
   return(*modePtr);
}


//*************************************************************************
// setMODE()                                                     2001-03-21
//-------------------------------------------------------------------------
// Description:
//    Sets the corresponding bit(s) in the Mode register.
//
// Parameter:
//    value   - value of the corresponding bit(s)
//    bitname - bit name within the Mode register
//
// Return:
//    none
//*************************************************************************/
void setMode(uint32_t bitname, uint32_t value)
{
   switch (bitname)
   {
      case MODE_TRM:   // Text Result Mode   (2 Bit)
         value = (uint32_t)(value << MODE_TRM) & 0x00000003U;
         *modePtr &= ~0x00000003U;
         *modePtr |= value;
         break;

      case MODE_CM:    // Command Mode       (2 Bit)
         value = (uint32_t)(value << MODE_CM) & 0x0000000CU;
         *modePtr &= ~0x0000000CU;
         *modePtr |= value;
         break;

      case MODE_EM:    // Error Mode         (1 Bit)
         value = (uint32_t)(value << MODE_EM) & 0x00000010U;
         *modePtr &= ~0x00000010U;
         *modePtr |= value;
         break;

      case MODE_SA:    // Send ASCII         (1 Bit)
         value = (uint32_t)(value << MODE_SA) & 0x00000020U;
         *modePtr &= ~0x00000020U;
         *modePtr |= value;
         break;

      case MODE_IC:    // Internal Command   (1 Bit)
         value = (uint32_t)(value << MODE_IC) & 0x00000040U;
         intern_cmd = (value==0x00000040U ? true : false);
         *modePtr &= ~0x00000040U;
         *modePtr |= value;
         break;

      case MODE_SD:    // Send Debug         (1 Bit)
         value = (uint32_t)(value << MODE_SD) & 0x00000080U;
         *modePtr &= ~0x00000080U;
         *modePtr |= value;
         break;

      case MODE_WKU:   // Wake Up            (5 Bit)
         value = (uint32_t)(value << MODE_WKU) & 0x00001F00U;
         *modePtr &= ~0x00001F00U;
         *modePtr |= value;
         break;

      case MODE_FE:    // Flash Erase        (1 Bit)
         value = (uint32_t)(value << MODE_FE) & 0x00010000U;
         *modePtr &= ~0x00010000U;
         *modePtr |= value;
         break;

      case MODE_STOP:  // Stop               (1 Bit)
         value = (uint32_t)(value << MODE_STOP) & 0x00020000U;
         *modePtr &= ~0x00020000U;
         *modePtr |= value;
         break;

      case MODE_FST:   // Flash STatus       (2 Bit)
         value = (uint32_t)(value << MODE_FST) & 0x000C0000U;
         *modePtr &= ~0x000C0000U;
         *modePtr |= value;
         break;

      case MODE_ZIP:   // Zip Module         (1 Bit)
         value = (uint32_t)(value << MODE_ZIP) & 0x00100000U;
         *modePtr &= ~0x00100000U;
         *modePtr |= value;
         break;

      case MODE_SER0:   // SER0 Module       (1 Bit)
         value = (uint32_t)(value << MODE_SER0) & 0x00200000U;
         *modePtr &= ~0x00200000U;
         *modePtr |= value;
         break;

      case MODE_SER1:   // SER1 Module       (1 Bit)
         value = (uint32_t)(value << MODE_SER1) & 0x00400000U;
         *modePtr &= ~0x00400000U;
         *modePtr |= value;
         break;

      case MODE_SER2:   // SER2 Module       (1 Bit)
         value = (uint32_t)(value << MODE_SER2) & 0x00800000U;
         *modePtr &= ~0x00800000U;
         *modePtr |= value;
         break;

      case MODE_VW:   // Verified Write      (1 Bit)
         value = (uint32_t)(value << MODE_VW) & 0x01000000U;
         *modePtr &= ~0x01000000U;
         *modePtr |= value;
         break;

      default:
         printf("** Error setting Mode Register @ 0x%08X",(uint32_t)modePtr);
         printf("     desired bit is not defined!");
   }
}


//*************************************************************************
// setModeReg()                                                  2002-06-07
//-------------------------------------------------------------------------
// Description:
//    Sets the complete mode register.
//
// Parameter:
//    val - value to be set in mode register
//
// Return:
//    -none-
//*************************************************************************/
void setModeReg(uint32_t val)
{
   *modePtr = val;
}



//*************************************************************************
// set_error_unknown(), set_result()           B. Achauer        2005-06-09
//-------------------------------------------------------------------------
// Description:
//    sets error and result pointer after a BIOS function
//
// Parameter:
//    commandPtr[]
//    ec    - error code to return
//    narg  - number of additional uint32_t to return (values follow narg)
//    len   - number of additional bytes to return (already in resultPtr[])
//
// Side effects:
//    resultPtr[], *errorPtr
//
// Return:
//    ec.
//
//************************************************************************/

uint32_t
set_error_unknown ()
{
   return set_result (ERROR_INTERPRT | ERROR_COMMAND);
}

uint32_t
set_result (uint32_t ec)
{
   return set_result_args (ec, 0);
}

uint32_t
set_result_args (uint32_t ec, unsigned narg, ...)
{
   va_list  args;
   unsigned i;


   if (ec)
      narg = 0;

   va_start (args, narg);
   for (i=0 ; i<narg ; i+=1)
      writeLong (&resultPtr[8 + 4*i], va_arg (args, uint32_t));
   va_end (args);

   return set_result_len (ec, 4 * narg);
}

uint32_t
set_result_len (uint32_t ec, unsigned len)
{
   *errorPtr = ec;
   writeLong (&resultPtr[4], ec);
   resultPtr[3] = commandPtr[3];
   resultPtr[2] = commandPtr[2];
   resultPtr[1] = commandPtr[1];
   RESULT_LENGTH(8 + len);

   return ec;
}

