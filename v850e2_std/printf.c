/*---------------------------------------------------------------------------------------
   $Header: //BiosGroup/trunk/BC_Projects/0000/0000-0000-00XY/cpu/src/dsp_c6xxx_std/printf.c#4 $
   $Change: 1362536 $
  ----------------------------------------------------------------------------------------*/

/*	[sys/lib/libsa] */
/*	$NetBSD: printf.c,v 1.16 2003/08/07 16:32:29 agc Exp $	*/
/*	$NetBSD: subr_prf.c,v 1.10 2003/08/07 16:32:30 agc Exp $	*/

/*-
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)printf.c	8.1 (Berkeley) 6/11/93
 */

//#include <sys/cdefs.h>
//#include <sys/types.h>
typedef unsigned long u_long;
typedef unsigned int u_int;

#include <stdarg.h>


#include "config.h"
#include "define.h"
#include "hbbios.h"                   /* jprintf */
#include "tool.h"

/*
 * Scaled down version of printf(3).
 */

#define NBBY	8	/* number of bits in a byte; NetBSD has this in <sys/types.h> */


static void kprintn(void (*)(int), u_long, int, int, int);
static void kdoprnt(void (*)(int), const char *, va_list);
static void jputc(int);

int
vprintf(const char *fmt, va_list ap)
{

	kdoprnt(jputc, fmt, ap);
   return 0;
}

/* Flags used during conversion: */

#define ZEROPAD	0x0002                  /* %0.. */
#define LONGINT	0x0004			/* %l.. */
#define UPCASE	0x0010                  /* %X (vs. %x) -- look at kprintn before changing this!	*/

static void
kdoprnt(void (*put)(int), const char *fmt, va_list ap)
{
	char *p;
	int ch;
	unsigned long ul;
	int flags;
        int width, n;

	for (;;) {
		while ((ch = *fmt++) != '%') {
			if (ch == '\0')
				return;
			put(ch);
		}
		flags = 0;
                width = 0;
reswitch:	switch (ch = *fmt++) {
		case 'l':
			flags |= LONGINT;
			goto reswitch;
		case '0':
                	flags |= ZEROPAD;
                	goto reswitch;
		case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
			n = 0;
			do {
				n = 10 * n + ch - '0';
				ch = *fmt++;
			} while (ch >= '0' && ch <= '9');
			width = n;
			fmt -= 1;
			goto reswitch;
		case 'c':
			ch = va_arg(ap, int);
				put(ch & 0x7f);
			break;
		case 's':
			p = va_arg(ap, char *);
                        for (n=0 ; p[n] ; n+=1)
                        	;
		        while (--width >= n)
        			put(' ');
			while ((ch = *p++))
				put(ch);
			break;
		case 'd':
			ul = (flags & LONGINT) ?
			    va_arg(ap, long) : va_arg(ap, int);
			if ((long)ul < 0) {
				put('-');
				ul = -(long)ul;
			}
			kprintn(put, ul, 10, width, flags);
			break;
		case 'o':
			ul = (flags & LONGINT) ?
			    va_arg(ap, u_long) : va_arg(ap, u_int);
			kprintn(put, ul, 8, width, flags);
			break;
		case 'u':
			ul = (flags & LONGINT) ?
			    va_arg(ap, u_long) : va_arg(ap, u_int);
			kprintn(put, ul, 10, width, flags);
			break;
		case 'p':
			put('0');
			put('x');
			flags |= LONGINT;
			goto plain_hex;
		case 'X':
			flags |= UPCASE;
			/* fall through */
		case 'x':
		plain_hex:
			ul = (flags & LONGINT) ?
			    va_arg(ap, u_long) : va_arg(ap, u_int);
			kprintn(put, ul, 16, width, flags);
			break;
		default:
			put('%');
			if ((flags & LONGINT))
				put('l');
			if (ch == '\0')
				return;
			put(ch);
		}
	}
}

static void
kprintn(void (*put)(int), unsigned long ul, int base, int width, int flags)
{
	const char *digit = "0123456789abcdef0123456789ABCDEF" + (flags & UPCASE);
	char *p, buf[(sizeof(long) * NBBY / 3) + 1]; /* hold a long in base 8 */

	p = buf;
	do {
		*p++ = digit[ul % base];
	} while (ul /= base);
        while (--width >= p - buf)
        	put((flags & ZEROPAD) ? '0' : ' ');
	do {
		put(*--p);
	} while (p > buf);
}


int
printf(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);

   return 0;
}

static void
jputc (int ch)
{
   static char buffer[256];
   static int  bpos = 0;

   if (ch != '\n')
      buffer[bpos++] = ch;

   if (ch == '\n' || bpos >= sizeof(buffer) - 1)
   {
      buffer[bpos] = 0;
      jprintf ("%s", bpos ? buffer : " ");

      bpos = 0;
   }
}
