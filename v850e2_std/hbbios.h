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
 *   $Header: //BiosGroup/trunk/BC_Projects/0000/0000-0000-00XY/cpu/src/std/hbbios.h#11 $
 *   $Change: 1381183 $
 *-----------------------------------------------------------------------
 */

#ifndef _HBBIOS_H_
#define _HBBIOS_H_

#include <stdint.h>

/* command list entry */
typedef struct cmd_list_s
{
   const char *cmd;
   const int   size;
   uint32_t  (*handler)(void);
   const char *descr;
} cmd_list_t;

extern cmd_list_t _cmd_tab_start;
extern cmd_list_t _cmd_tab_end;

/**
 * adds a command to the command list
 *
 * @param   cmd    command pattern that must fit (ex. "X\xc0")
 * @param   size   size of the command pattern   (ex. 2)
 * @param   hdlr   function that should be called to handle the command
 * @param   descr  short description
 */
#define ADD_CMD(cmd, size, hdlr, descr) \
const cmd_list_t _cmd_##hdlr __attribute__ ((used,section (".cmd_tab"))) = { cmd, size, hdlr, descr }

extern void   writeWord(volatile void* addr, uint16_t wdata);
extern void   writeLong(volatile void* addr, uint32_t ldata);
extern uint16_t readWord(volatile void* addr);
extern uint32_t readLong(volatile void* addr);
extern uint16_t swapWord(uint16_t wdata);
extern uint32_t swapLong(uint32_t ldata);

extern uint32_t getMode(uint32_t bitname);
extern uint32_t getModeReg(void);
extern void setMode(uint32_t bitname, uint32_t value);
extern void setModeReg(uint32_t val);

extern int  printf (const char *format, ...) __attribute__((format(printf, 1, 2)));

extern void flush_output (void);

// #define jprintf(fmt, ...)	printf (fmt "\n", ## __VA_ARGS__)	-- not yet ...
extern int jprintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
extern int iprintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
extern int dprintf(const char *format, ...) __attribute__((format(printf, 1, 2)));

extern uint32_t set_error_unknown (void);
extern uint32_t set_result        (uint32_t ec);
extern uint32_t set_result_args   (uint32_t ec, unsigned narg, ...);
extern uint32_t set_result_len    (uint32_t ec, unsigned len);


extern void __assert (const char *, int, const char *) __attribute((noreturn));
extern void _exit (int)                  __attribute((noreturn));
extern void abort (void)                 __attribute((noreturn));
extern void exit  (int)                  __attribute((noreturn));
extern void panic (const char *fmt, ...) __attribute((noreturn));
extern void reset (void)                 __attribute((noreturn));

/*
**  "standard" macros:
*/
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#define MAX(a, b)	((a) > (b) ? (a) : (b))

#define DIM(array)	(sizeof(array) / sizeof((array)[0]))

#endif
