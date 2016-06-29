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
 *   script communication handler
 *-----------------------------------------------------------------------
 *   $Header:$
 *   $Change:$
 *-----------------------------------------------------------------------
*/

//-----------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------
#include "config.h"
#include "define.h"
#include "hbbios.h"
#include "global.h"

#if (SW_SCRIPT == 1)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "comm.h"
#include "timer.h"
#include "script.h"

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------

/* commands */
#define AUTO_START      "@start" // start line
#define AUTO_STOP       "@stop"  // stop line
#define AUTO_LOOP       "@loop"  // loop start line
#define AUTO_ELOOP      "@eloop" // loop end line
#define AUTO_SLEEP      "@sleep" // loop end line
#define AUTO_LABEL      "@label" // loop end line
#define AUTO_KEYCHAR    "@"      // key char of a control command

#define AUTO_LINE_LEN   200      // max. length of a line in the script

/* states */
#define AUTO_INIT       0x00U
#define AUTO_RUN        0x01U
#define AUTO_EXIT       0x02U
#define AUTO_RUN_LOOP   0x03U

#ifndef SYN
   #define SYN       0x16U
#endif


/*
 * local variables
 */
static char      autoLine[AUTO_LINE_LEN]; /**< extracted line of the script */
static char     *autoPos;                 /**< position inside the script */
static char     *autoLoopStartAddr;       /**< position of a loop */
static uint32_t  autoLoopCnt;             /**< number of loops */
static uint32_t  autoSleepCnt = 0;        /**< number of ms to sleep */
static uint32_t  autoJump = 0;            /**< means no jump activ */
static uint32_t  autoSleepTime;
static uint32_t  autoLineRestore;

static unsigned  state;
static unsigned  loopCnt;
static unsigned  line;
static unsigned  scriptDebug = 0;

static uint8_t interfaceID;
static unsigned msgCount = 0;

static uint8_t *pScript = NULL;
static unsigned killNextScript = 0;

static char msgStr[200];

static BOOL ignoreResultErrors = false;
static uint8_t lastResult[256];

//-----------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------
static BOOL autoPutCommand(volatile uint8_t *buffer);
static BOOL autoStart(void);
static BOOL autoStop(void);
static uint32_t autoGetLoopCnt(void);
static BOOL autoLoop(void);
static BOOL autoLabel(void);
static BOOL autoEloop(void);
static BOOL autoCmd(void);
static void autoPurgeLine(void);
static BOOL autoGetLine(void);

static uint32_t scriptGetLine (volatile uint8_t *scriptPtr, volatile uint8_t *buffer, BOOL init);


/**
 * This function extracts a command from the read line in autoLine[] and writes
 * it into the command buffer.
 *
 * @param   buffer  pointer to write the command to
 * @return  true, if sucessful; false if not
 */
BOOL autoPutCommand(volatile uint8_t *buffer)
{
   uint8_t len;  // command length
   uint32_t i;   // position inside the line
   uint8_t num;

   /* check the command length */
   if(0U == (strlen(autoLine) % 2U)) // even length?
      return false;

   /* command letter */
   len = 1U;
   buffer[len] = autoLine[0U];
   len++;

   /* parameters */
   for (i = 1; i < strlen(autoLine); i += 2U)
   {
      if((autoLine[i] >= '0') && (autoLine[i] <= '9'))
      {
         num = ((uint8_t)(autoLine[i] - '0')) << 4U;
      }
      else
      {
         if((autoLine[i] >= 'A') && (autoLine[i] <= 'Z'))
         {
            num = ((uint8_t)(autoLine[i] - 'A' + 10U)) << 4U;
         }
         else
         {
            if((autoLine[i] >= 'a') && (autoLine[i] <= 'z'))
            {
               num = ((uint8_t)(autoLine[i] - 'a' + 10U)) << 4U;
            }
            else
            {
               return(false);
            }
         }
      }

      if((autoLine[i + 1U] >= '0') && (autoLine[i + 1U] <= '9'))
      {
         num += (uint8_t)(autoLine[i + 1U] - '0');
      }
      else
      {
         if((autoLine[i + 1U] >= 'A') && (autoLine[i + 1U] <= 'Z'))
         {
            num += (uint8_t)(autoLine[i + 1U] - 'A' + 10U);
         }
         else
         {
            if((autoLine[i + 1U] >= 'a') && (autoLine[i + 1U] <= 'z'))
            {
               num += (uint8_t)(autoLine[i + 1U] - 'a' + 10U);
            }
            else
            {
               return(false);
            }
         }
      }

      buffer[len] = num;
      len++;
   }

   buffer[0U] = len;

   return true;
}


/**
 * This function checks wether the start pattern is present or not.
 *
 * @return   true, if start pattern present; false, if not
 */
BOOL autoStart(void)
{
   if (strcmp(autoLine, AUTO_START) == 0)
      return true;
   else
      return false;
}


/**
 * This function checks wether the stop pattern is present or not.
 *
 * @return   true, if stop pattern present; false, if not
 */
BOOL autoStop(void)
{
   if(strcmp(autoLine, AUTO_STOP) == 0)
      return true;
   else
      return false;
}


/**
 * This function gets the argument of the @loop() keyword.
 *
 * @return  0xFFFFFFFFU if no argument was found, i.e. infinite loop
 */
uint32_t autoGetLoopCnt(void)
{
   char loopCnt[20U];
   uint8_t i, j;

   i = 0U;
   while(autoLine[i] != '(')
   {
      i++;
   }

   i++; j = 0U;
   while(autoLine[i] != ')')
   {
      if((autoLine[i] >= '0') ||
         (autoLine[i] <= '9'))
      {
         loopCnt[j] = autoLine[i];
         j++;
         if(j == 19U)
         {
            break;
         }
      }
      i++;
   }
   loopCnt[j] = '\0';

   if(j == 0U)
   {
      return(0xFFFFFFFFU);
   }
   else
   {
      return(atoi(loopCnt));
   }
}


/**
 * This function checks wether a loop pattern is present or not.
 *
 * @return  true, if loop pattern present; false, if not
 */
BOOL autoLoop(void)
{
   if(strstr(autoLine, AUTO_LOOP) != NULL)
   {
      if (autoJump)
         return true;
      state = AUTO_RUN_LOOP;
      loopCnt = 0;
      autoLoopStartAddr = autoPos;
      autoLoopCnt = autoGetLoopCnt();
      autoLineRestore = line;
      return true;
   }
   else
   {
      return false;
   }
}


/**
 * This function checks wether a end of loop pattern is present or not.
 *
 * @return  true, if end of loop pattern present; false, if not
 */
BOOL autoEloop(void)
{
   if(strcmp(autoLine, AUTO_ELOOP) == 0)
      return true;
   else
      return false;
}


/**
 * This function checks wether a loop pattern is present or not.
 *
 * @return  true, if sleep time pattern present; false, if not
 */
BOOL autoSleep(void)
{
   if(strstr(autoLine, AUTO_SLEEP) != NULL)
   {
      if (autoJump)
         return true;
      autoSleepTime = timGetTime();
      autoSleepCnt = autoGetLoopCnt();
      return true;
   }
   else
   {
      return false;
   }
}


/**
 * This function checks wether a label pattern is present or not.
 *
 * @return  true, if label pattern present; false, if not
 */
BOOL autoLabel(void)
{
   if(strstr(autoLine, AUTO_LABEL) != NULL)
   {
      if (autoJump == autoGetLoopCnt())
         autoJump = 0;
      return true;
   }
   else
   {
      return false;
   }
}


/**
 * This function checks wether a BIOS command is present or not.
 *
 * @return  true, if BIOS command present; false, if not
 */
BOOL autoCmd(void)
{
   if(strstr(autoLine, AUTO_KEYCHAR) != NULL)
      return false;
   else
      return true;
}



/**
 * This function removes all comments and whitespaces from the line read.
 */
void autoPurgeLine(void)
{
   uint32_t i, j;

   i = 0U; j = 0U;
   while(autoLine[i] != '\0')
   {
      if(autoLine[i] == '/') // comment reached
      {
         break;
      }

      if(!isspace((int)autoLine[i])) // all except white spaces
      {
         autoLine[j++] = autoLine[i++];
         continue;
      }

      i++; // goto next character
   }
   autoLine[j] = '\0';
}


/**
 * This function isolates a line from the start script.
 *
 * @return  true, if line could be isolated, false, if not
 */
BOOL autoGetLine(void)
{
   uint32_t i = 0;

   // read the next line until CR or LF reached
   while(i < AUTO_LINE_LEN - 1U)
   {
      autoLine[i] = *autoPos;
      autoPos++;
      if((autoLine[i] == '\r') || (autoLine[i] == '\n')) // end of line
      {                                                  // reached
         if (autoLine[i] == '\n')
            line++;
         autoLine[i] = 0U;
         // position to next start of line
         while((*autoPos == '\r') || (*autoPos == '\n'))
         {
            if (*autoPos == '\n')
               line++;
            autoPos++;
         }
         autoPurgeLine(); // removes comments and whitespaces
         return(true);
      }
      else
      {
         if(i >= AUTO_LINE_LEN - 1U) // line too large
         {
            return(false);
         }
      }
      i++;
   }

   return(false); // just to avoid comiler warnings
}



/**
 * Execution of the commands saved as script in memory
 *
 * @return  state of the current line
 */
uint32_t scriptGetLine (volatile uint8_t *scriptPtr, volatile uint8_t *buffer, BOOL init)
{

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // initialisieren erwuenscht ?
   if (init)
   {
      autoPos = (char *)scriptPtr;
      state = AUTO_INIT;
      line = 0U;
      return (SCRIPT_NO_COMMAND);
   }
   else
   {  // get next line ----------------------------------------------------
      if(!autoGetLine()) // if no line available
      {
         if(line <= 1U)
         {
            return(ERROR_SCRIPT | SCRIPT_ERROR_NOSCRIPT_FOUND);
         }
         else
         {
            sprintf(msgStr, "   script: no valid expression found => stopped at line %d: %s", line, autoLine);
            return (SCRIPT_ERROR_INVALID_EXPRESSION);
         }
      }

      // statemachine for script interpretation ---------------------------
      switch(state)
      {
         case AUTO_INIT:
            if(autoStart()) // start pattern found?
            {
               state = AUTO_RUN;
               return (SCRIPT_START_FOUND);
            }
            sprintf(msgStr, "   script: no script found at address 0x%08X", (uint32_t)scriptPtr);
            return (SCRIPT_ERROR_NOSCRIPT_FOUND);

         case AUTO_RUN:
            if(autoLine[0U] == '\0') // empty line
            {
               return (SCRIPT_NO_COMMAND);
            }

            if(autoCmd()) // topic line contains BIOS command
            {
               if(!autoPutCommand(buffer))
               {
                  sprintf(msgStr, "   script: command could not be parsed => stopped at line %d: %s", line, autoLine);
                  return (SCRIPT_ERROR_PARSER);
               }
               return(SCRIPT_COMMAND_FOUND);
            }

            if(autoStop()) // end of script reached
            {
               sprintf(msgStr, "   script: end of script reached => stopped at line %d", line);
               return (SCRIPT_FINISHED_OK);
            }

            if(autoSleep())
            {
               return(SCRIPT_NO_COMMAND);
            }

            if(autoLoop()) // start of loop reached
            {
               if (!autoJump)
               {
                  state = AUTO_RUN_LOOP;
                  loopCnt = 0;
               }
               return (SCRIPT_NO_COMMAND);
            }

            if(autoEloop()) // end of loop reached
            {
               if (autoJump)
                  return (SCRIPT_NO_COMMAND);
               else
               {
                  sprintf(msgStr, "   script: @eloop without @loop command found => stopped at line %d: %s", line, autoLine);
                  return (SCRIPT_ERROR_UNEXPECTED_COMMAND);
               }
            }

            if(autoLabel()) // label command
            {
               return (SCRIPT_NO_COMMAND);
            }

            sprintf(msgStr, "   script: unexpected command found => stopped at line %d: %s", line, autoLine);
            return (SCRIPT_ERROR_UNEXPECTED_COMMAND);

         case AUTO_RUN_LOOP:
            if(autoLine[0U] == '\0') // empty line
            {
               return (SCRIPT_NO_COMMAND);
            }

            if(autoCmd()) // topic line contains BIOS command
            {
               if(loopCnt < autoLoopCnt)
               {
                  if(!autoPutCommand(buffer))
                  {
                     sprintf(msgStr, "   script: command could not be parsed => stopped at line %d: %s", line, autoLine);
                     return(SCRIPT_ERROR_PARSER);
                  }
                  return(SCRIPT_COMMAND_FOUND);
               }
               return (SCRIPT_NO_COMMAND);
            }

            if(autoEloop()) // end of loop reached
            {
               loopCnt++;
               if(loopCnt < autoLoopCnt)
               {
                  autoPos = autoLoopStartAddr;
                  line = autoLineRestore;
                  return (SCRIPT_NO_COMMAND);
               }
               else
               {
                  state = AUTO_RUN;
                  return (SCRIPT_NO_COMMAND);
               }
            }

            if(autoStop()) // end of script reached
            {
               sprintf(msgStr, "   script: end of script reached => stopped at line %d", line);
               return(SCRIPT_FINISHED_OK);
            }

            if(autoSleep())
            {
               return(SCRIPT_NO_COMMAND);
            }

            if(autoLoop()) // nested loop found
            {
               sprintf(msgStr, "   script: nested @loop command found => stopped at line %d: %s", line, autoLine);
               return (SCRIPT_ERROR_NESTED_LOOP);
            }

            sprintf(msgStr, "   script: unexpected command found => stopped at line %d: %s", line, autoLine);
            return (SCRIPT_ERROR_UNEXPECTED_COMMAND);

         default:
            sprintf(msgStr, "   script: unexpected command found => stopped at line %d: %s", line, autoLine);
            return (SCRIPT_ERROR_UNEXPECTED_COMMAND);
      }
   }
} // of function SCRIPT_GetLine



/**
 * start script from CMD or autorun (see main)
 *
 * @return  error
 */
uint32_t scriptStart(uint32_t addr)
{
   uint32_t error = 0;

   msgStr[0] = 0;
   if (killNextScript)
   {
      killNextScript--;
      error = ERROR_SCRIPT | SCRIPT_ERROR_INVALID_ADDRESS;
      pScript = NULL;
   }
   else
   {
      pScript = (uint8_t *) addr;
      msgCount = 0;
      // init script
      scriptGetLine (pScript, NULL, true);

      // test first line (is there a script)
      if (scriptGetLine (pScript, NULL, false) != SCRIPT_START_FOUND)
      {
         error = ERROR_SCRIPT | SCRIPT_ERROR_NOSCRIPT_FOUND;
         pScript = NULL;
      }
   }
   return(error);
}



/**
 * script commands
 *
 * @return  error
 */
void scriptCmd (void)
{
   uint8_t  length = 0x00U;
   unsigned scriptLine;
   uint32_t error;
   uint32_t script;
   uint8_t  len;
   uint8_t  cmd;

   len = commandPtr[0];
   cmd = commandPtr[2];
   error = NO_ERROR;

   scriptLine = line;

   switch (cmd)
   {
      case 0x00:
         // Start/Restart a Script
         if (len != 8)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }
         script = readLong(&commandPtr[4]);
         printf("   script @ 0x%08x => ", script);
         ignoreResultErrors = (commandPtr[3] == 1);
         error = scriptStart(script);
         if (error)
         {
            if (error == (ERROR_SCRIPT | SCRIPT_ERROR_NOSCRIPT_FOUND))
               printf("not found\n");
            else
               printf("killed\n");
         }
         else
            printf("started\n");
         break;

      case 0x01:
         // Stop Script
         if (len != 3)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         if (pScript)
         {
            printf("   script @ 0x%08X => stopped at line %d\n", (uint32_t) pScript, scriptLine);
            pScript = NULL;
         }
         else
         if (killNextScript < 2)
         {
            printf("   next script will be killed\n");
            killNextScript++;
         }
         break;

      case 0x02:
         // Get Script Info
         if (len != 3)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         printf("   script @ 0x%08X => ", (uint32_t) pScript);
         if (pScript == NULL)
            printf("stopped\n");
         else
         {
            if (timDifference(autoSleepTime) < autoSleepCnt)
               printf("sleeping");
            else
               printf("running");
            printf(" at line %d\n", scriptLine);
         }
         writeWord(&resultPtr[12], scriptLine);
         writeLong(&resultPtr[8],  (uint32_t)pScript);
         length = 0x06U;
         break;

      case 0x03:
         // set/reset ignoreResultErrors
         if (len != 4)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         ignoreResultErrors = (commandPtr[3] == 1);
         break;

      case 0x10:
         // simple compare of the last result => generate error if not equal
         if (len < 3)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         if (memcmp(&lastResult[commandPtr[3]], &commandPtr[4], commandPtr[0]-4) != 0)
            error = ERROR_SCRIPT | ERROR_COMMAND;
         break;

      case 0x11:
         // simple compare of the last result => generate error if equal
         if (len < 3)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         if (memcmp(&lastResult[commandPtr[3]], &commandPtr[4], commandPtr[0]-4) == 0)
            error = ERROR_SCRIPT | ERROR_COMMAND;
         break;

      case 0x20:
      case 0x21:
         if (len < 3)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         if (cmd == 0x20)
         { // 0x20: simple compare of the last result => jump to label commandPtr[4] if not equal or different length
            if (((lastResult[0]-commandPtr[4]) < (len-5)) ||
                (memcmp(&lastResult[commandPtr[4]], &commandPtr[5], MIN(len-5, lastResult[0]-commandPtr[4])) != 0))
               autoJump = commandPtr[3];
         } else
         { // 0x21: simple compare of the last result => jump to label commandPtr[4] if equal
            if (((lastResult[0]-commandPtr[4]) >= (len-5)) &&
                (memcmp(&lastResult[commandPtr[4]], &commandPtr[5], MIN(len-5, lastResult[0]-commandPtr[4])) == 0))
               autoJump = commandPtr[3];
         }
         break;

      case 0x2F:
         if (len != 4)
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         autoJump = commandPtr[3];
         break;

      case 0x30:
      case 0x31:
      case 0x32:
      {
         uint8_t newLastResult[16];
         unsigned n;

         if ((len <5) || (len > 20))
         {
            set_result(ERROR_SCRIPT | ERROR_COMMAND);
            return;
         }

         length = len-4;
         for (n=0; n<(length); n++)
         {
            switch(cmd)
            {
               case 0x30:
                  newLastResult[n] = commandPtr[4+n] & lastResult[commandPtr[3]+n];
                  break;
               case 0x31:
                  newLastResult[n] = commandPtr[4+n] | lastResult[commandPtr[3]+n];
                  break;
               case 0x32:
                  newLastResult[n] = commandPtr[4+n] ^ lastResult[commandPtr[3]+n];
                  break;
            }
         }

         memcpy(&resultPtr[8], newLastResult, length);
         memcpy(&lastResult[8], newLastResult, length);
      }
         break;

      case 0xFF:
         printf("script debug mode ");
         scriptDebug = commandPtr[3];
         if (scriptDebug)
            printf("on\n");
         else
            printf("off\n");
         break;

      default:
         error = ERROR_SCRIPT | ERROR_COMMAND;
         break;
   } // end of switch

   // Ergebnis senden
   set_result_len(error,length);
   return;

} // end of function scriptCmd



/**
 * receive message from script; synchronized with returning HBBIOS answers via msgCount
 *
 * @return  nothing
 */
void scriptMsgReceive(unsigned ch)
{
   uint32_t result;
   uint8_t buffer[256];

   if (pScript == NULL)
   {
      // kein Script angegeben
      return;
   }
   else
   {  // eine Zeile abholen
      if (killNextScript)
      {
         killNextScript--;
         printf("   script @ 0x%08x stopped\n", pScript);
         pScript = NULL;
         return;
      }

      if (msgCount == 0)
      {
         if (autoSleepCnt)
         {
            if (timDifference(autoSleepTime) <= autoSleepCnt)
               return;
            else
               autoSleepCnt = 0;
         }

         result = scriptGetLine (pScript, buffer, false);

         if (autoJump)
            return;

         // und auswerten
         switch (result)
         {
            case SCRIPT_NO_COMMAND:   // nichts tun
               return;

            case SCRIPT_COMMAND_FOUND:
               msgReceiveRaw(interfaceID, buffer, buffer[0]);
               msgCount++;
               return;

            case SCRIPT_FINISHED_OK :  // Zeiger auf das Script loeschen und fertig
               pScript = NULL;
               return;

            default:                   // Zeiger auf das Script loeschen und fertig
               pScript = NULL;
               return;
         }
      }
   }
}


void scriptPrintMsg(void)
{
   if (msgStr[0] != 0)
   {
      printf("%s\n", msgStr);
      msgStr[0] = 0;
   }
}



/**
 *  tx message to script; synchronized with returning HBBIOS answers via msgCount
 *
 * @return  nothing
 */
BOOL scriptMsgSend (unsigned channel, uint8_t * buffer, unsigned len_seq)
{
   uint32_t error;

   if (buffer[1] != CMD_ASCII_DATA)
   {
      if (msgCount > 0)
         msgCount--; // synchronize on HBBIOS commands

      error = readLong(&buffer[4]);
      if ((error != 0) && !ignoreResultErrors)
      {
         sprintf(msgStr, "   script: error in received result => stopped at line %d: %s  error: %08x", line, autoLine, error);
         pScript = NULL;
      }
      if ((buffer[1] != CMD_GATEWAY) && (buffer[1] != CMD_SCRIPT_EXECUTE))
      {
         memcpy(lastResult, buffer, buffer[0]);
      }
      if (scriptDebug)
      {
         unsigned n;
         sprintf(msgStr, "  lastResult: ");
         for (n=0; n<buffer[0]; n++)
            sprintf(msgStr, "%s%02x", msgStr, lastResult[n]);
      }
   }

   return(true);
}


/**
 * add script as CMD channel; called once from main
 *
 * @return  nothing
 */
void scriptInit (void)
{
   interfaceID = msgRegister (ifScript, "Script", 0, 1, scriptMsgReceive, scriptMsgSend, (void *)1,
                              COMM_FLAGS_RAWDATA | COMM_FLAGS_AUTOACK);

}


BOOL scriptReady (void)
{
   return (pScript == NULL);
}


#endif   // #if (SW_SCRIPT == 1)


