/*
 *  SPI compatible (WS2801) to WS2811/WS2812(b) protocol translator
 *  for an attiny85 running at 16MHz (internal OSC, PLL prescaled).
 *
 *  It uses the entire SRAM as circular buffer, bitbanging bytes out
 *  while they come in.  This means the number of LEDs that can be
 *  driven is limited by the difference in input & output speed.
 *  (there may possibly be other factors that limit the maximum number
 *   of LEDs though.)
 *
 *  Try to get the net SPI bitrate slightly above the (high-speed) WS2811
 *  protocol speed (which is 800Kbps) to be able to drive the most LEDs.
 *  Although speeds up to 2MHz seem to work fine as well.
 *
 *  The code cannot handle a full buffer, so try not to fill the buffer
 *  completely.
 *                            ___________
 *                          _|           |_
 *                       __/ | O         | \__  VCC
 *                          _|           |_
 *                       __/ | attiny85  | \__  SCK
 *                          _|           |_
 *     WS2811/WS2812(b)  __/ |           | \__
 *                          _|           |_
 *                  GND  __/ |___________| \__  MOSI
 *
 *
 * Set PLL clock source:
 * avrdude -v -c usbtiny -pt85 -U lfuse:w:0xe1:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
 *
 * Copyright (c) 2015 Erik Bosman <erik@minemu.org>
 *
 * Permission  is  hereby  granted,  free  of  charge,  to  any  person
 * obtaining  a copy  of  this  software  and  associated documentation
 * files (the "Software"),  to deal in the Software without restriction,
 * including  without  limitation  the  rights  to  use,  copy,  modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the
 * Software,  and to permit persons to whom the Software is furnished to
 * do so, subject to the following conditions:
 *
 * The  above  copyright  notice  and this  permission  notice  shall be
 * included  in  all  copies  or  substantial portions  of the Software.
 *
 * THE SOFTWARE  IS  PROVIDED  "AS IS", WITHOUT WARRANTY  OF ANY KIND,
 * EXPRESS OR IMPLIED,  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY,  FITNESS  FOR  A  PARTICULAR  PURPOSE  AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM,  DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT,  TORT OR OTHERWISE,  ARISING FROM, OUT OF OR IN
 * CONNECTION  WITH THE SOFTWARE  OR THE USE  OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * (http://opensource.org/licenses/mit-license.html)
 */

#include <avr/io.h>

.section .bss

/* use ALL THE MEMORY! */
#define BUFSTART ( RAMSTART )
#define BUFEND   ( RAMEND + 1 )

#define OUTPIN 4
#define OUTDDR DDRB
#define OUTPORT PORTB
#define BUFCACHE 3 /* start transmitting as soon as 3 bytes have been received */

#if ( BUFCACHE > 255 ) != 0
	#error "buffer cache size should fit in a 8 bit register"
#endif

#define TIMEOUT_CYCLES (50*16)
#define TIMEOUT_LOOP_COUNT(cycles, cycles_per_iteration) ( (cycles+cycles_per_iteration-1)/cycles_per_iteration )

#define rUNUSED r7

#define rSTATUS0     r8
#define rSHIFTDATA0  r9
#define rSTATUS1     r10
#define rSHIFTDATA1  r11
#define rSTATUS2     r12
#define rSHIFTDATA2  r13
#define rSTATUS3     r14
#define rNEWDATA     r15

#define rTMP r16
#define rLOOP r17
#define rLOW r18
#define rHIGH r19
#define rRESETUSI r20
#define rDATA r21
#define rHIBUFEND r22
#define rSAVEDSREG r23

#define loLOAD r26
#define hiLOAD r27
#define loSTORE r28
#define hiSTORE r29
#define loBUF r30
#define hiBUF r31

#define pLOAD  X
#define pSTORE Y
#define pBUF   Z

.section .text

.global main

.org 0x0
main:

init:

/* Load constants */
ldi rLOW, 0                   /* bitbang low                                                */
ldi rHIGH, _BV(OUTPIN)        /* bitbang high                                               */
ldi rHIBUFEND, hi8(BUFEND)    /* register needed for compare-with-carry                     */
ldi loBUF, lo8(BUFSTART)      /* use Z to store a pointer to the start of the buffer        */
ldi hiBUF, hi8(BUFSTART)      /*                                                            */

/* - Clear data start/available flags
 * - Initialise 4 bit counter with 1 in order to get the overflow flag
 *   immediately after the 8'th bit has been shifted in.
 */
ldi rRESETUSI,_BV(USISIF)|_BV(USIOIF)|(1<<USICNT0)

/* Output pin setup */
out _SFR_IO_ADDR(OUTDDR),  rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW

/* SPI slave setup */
ldi rTMP, _BV(USICS1) /* USI in input-only-mode, external clock */
out _SFR_IO_ADDR(USICR), rTMP

wait_for_data:
out _SFR_IO_ADDR(USISR), rRESETUSI /* Clear(!) data available flag, set counter */

movw loLOAD,  loBUF  /* pLOAD  = pBUF */
movw loSTORE, loBUF  /* pSTORE = pBUF */

wait_for_communication:
sbis _SFR_IO_ADDR(USISR), USISIF
rjmp wait_for_communication

cache_loop:
ldi rLOOP, TIMEOUT_LOOP_COUNT(TIMEOUT_CYCLES, 7)
timeout_loop:
dec rLOOP
in rSTATUS1, _SFR_IO_ADDR(USISR)
in rNEWDATA, _SFR_IO_ADDR(USIDR)
sbrc rSTATUS1, USIOIF
rjmp cache_data
brne timeout_loop
rjmp wait_for_data /* timeout, data considered noise, start over */
cache_data:
in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
st pSTORE+, rNEWDATA
cpi loSTORE, lo8(BUFSTART+BUFCACHE)
brne cache_loop
ld rDATA, pLOAD+
eor rSTATUS1, rSTATUS1

rjmp bit7_poll

              /* BIT BANG */                    /* BUFFER STORES */                    /* BUFFER LOADS */

              /* None of the bitbanging functions touch the flags, yay \o/ */

                                                /* store buffer updates do use flags */

                                                                                       /* buffer loads need to not mess */
                                                                                       /* with store buffer flags       */

/* split in two because of free cycles to do an rjmp */
bit7_poll_2ndhalf:
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit6_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit6_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 6
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 +  1  */                                                                          nop
/*10 +  1  */                                                                          nop
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit5_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit5_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 5
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 + 1|2 */                                   brne 1f
/*10 + 1|0 */                                   movw loSTORE, loBUF   /* make sure the pLOAD == pSTORE (exit condition) is on buffer-boundary */
1:                                                                    /* wrapped pointers (assuming no more bytes are read this iteration) */
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit4_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit4_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                                                          cpi loLOAD, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                                                          cpc hiLOAD, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 4
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 +  1  */                                                                          in rSAVEDSREG, _SFR_IO_ADDR(SREG)
/*10 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/*11 +  1  */                                   cpc hiSTORE, rHIBUFEND
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit3_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit3_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 3
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 + 1|2 */                                                                          sbrc rSAVEDSREG, SREG_Z
/*10 + 1|0 */                                                                          movw loLOAD, loBUF
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit2_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit2_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                                                          cp loLOAD, loSTORE
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                                                          cpc hiLOAD, hiSTORE
/* 5 + 1|2 */ sbrs rDATA, 2
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 +  1  */                                                                          in rSAVEDSREG, _SFR_IO_ADDR(SREG)
/*10 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/*11 +  1  */                                   cpc hiSTORE, rHIBUFEND
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit1_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit1_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 1
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 + 1|2 */                                                                          sbrc rSAVEDSREG, SREG_Z
/*10 + 2|0 */                                                                          rjmp bit1_exit
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit0_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop



bit0_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 0
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 +  2  */                                                                          ld rDATA, pLOAD+
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit7_store
/*17 +  1  */                                   in rSTATUS0,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA0, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   nop

bit7_poll:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   cpi loSTORE, lo8(BUFEND)
/* 2 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   cpc hiSTORE, rHIBUFEND
/* 5 + 1|2 */ sbrs rDATA, 7
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 9 +  2  */                                                                          rjmp bit7_poll_2ndhalf


/* We have received a byte, what now? */

bit7_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 7
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          nop
/*16 +  1  */                                                                          nop
/*17 +  1  */                                                                          nop
/*18 +  1  */                                   rjmp bit6_poll



bit6_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 6
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          nop
/*16 +  1  */                                                                          nop
/*17 +  1  */                                                                          nop
/*18 +  1  */                                   rjmp bit5_poll



bit5_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 5
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          nop
/*16 +  1  */                                                                          nop
/*17 +  1  */                                                                          nop
/*18 +  1  */                                   rjmp bit4_poll



bit4_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 4
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          cpi loLOAD, lo8(BUFEND)
/*16 +  1  */                                                                          cpc hiLOAD, rHIBUFEND
/*17 +  1  */                                                                          in rSAVEDSREG, _SFR_IO_ADDR(SREG)
/*18 +  1  */                                   rjmp bit3_poll



bit3_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 3
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                                                          sbrc rSAVEDSREG, SREG_Z
/*16 + 1|0 */                                                                          movw loLOAD, loBUF
/*17 +  1  */                                                                          nop
/*18 +  1  */                                   rjmp bit2_poll



bit2_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 2
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          eor rSAVEDSREG, rSAVEDSREG /* clear Z flag, as there is data in the buffer */
/*16 +  1  */                                                                          nop
/*17 +  1  */                                                                          nop
/*18 +  1  */                                   rjmp bit1_poll



bit1_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 1
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                                                          nop /* poll version checked for data */
/*16 +  1  */                                                                          nop /* we know there is as data was  */
/*17 +  1  */                                                                          nop /* just stored                   */
/*18 +  1  */                                   rjmp bit0_poll



bit0_store:
/*18 + 1|2 */                                   brne 1f
/*19 + 1|0 */                                   movw loSTORE, loBUF
1:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/* 2 +  1  */                                   nop
/* 3 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/* 4 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA2
/* 5 + 1|2 */ sbrs rDATA, 0
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/* 8 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA1
/* 9 + 1|2 */                                   sbrc rSTATUS0, USIOIF
/*10 + 1|0 */                                   mov rNEWDATA, rSHIFTDATA0
/*11 +  2  */                                   st pSTORE+, rNEWDATA
/*13 +  1  */                                   eor rSTATUS0, rSTATUS0
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  2  */                                                                          ld rDATA, pLOAD+
/*17 +  1  */                                                                          nop
/*18 +  2  */                                   rjmp bit7_poll



bit1_exit:
/* finish bit1 */
/*12 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rNEWDATA,    _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*16 + 2|0 */                                   rjmp bit0_store                            /* second chance :-P */
/*17 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/*18 +  1  */                                   in rSHIFTDATA1, _SFR_IO_ADDR(USIDR)
/*19 +  1  */                                   movw loSTORE, loBUF
;bit0_exit:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                                                          movw loLOAD, loBUF
/* 2 +  1  */                                   in rSTATUS2,    _SFR_IO_ADDR(USISR)
/* 3 +  1  */                                   in rSHIFTDATA2, _SFR_IO_ADDR(USIDR)
/* 4 +  1  */                                   eor rSTATUS0, rSTATUS0
/* 5 + 1|2 */ sbrs rDATA, 0
/* 6 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 7 +  1  */                                   in rSTATUS3,    _SFR_IO_ADDR(USISR)
/* 8 +  1  */                                   in rDATA,       _SFR_IO_ADDR(USIDR)    /* loads rDATA */
/* 9 + 1|2 */                                   sbrc rSTATUS3, USIOIF
/*10 + 2|0 */                                   rjmp bit0_another_chance
/*11 +  1  */                                                                          nop
/*12 +  1  */                                   in rSTATUS1,    _SFR_IO_ADDR(USISR)
/*13 +  1  */                                   in rDATA,       _SFR_IO_ADDR(USIDR)
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 + 1|2 */                                   sbrs rSTATUS1, USIOIF
/*16 + 2|0 */                                   rjmp trail_timeout
/*17 +  1  */                                   in rUNUSED,     _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/*18 +  2  */                                   rjmp bit7_poll

bit0_another_chance:
/*12 + 1|2 */                                   sbrc rSTATUS2, USIOIF
/*13 + 1|0 */                                   mov rDATA, rSHIFTDATA2                 /* loads rDATA */
/*14 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/*15 +  1  */                                   in rUNUSED, _SFR_IO_ADDR(USIBR)  /* clears USIOIF in one cycle */
/*16 + 1|2 */                                   sbrc rSTATUS1, USIOIF
/*17 + 1|0 */                                   mov rDATA, rSHIFTDATA1                 /* loads rDATA */
/*18 +  2  */                                   rjmp bit7_poll

trail_timeout:
ldi rLOOP, TIMEOUT_LOOP_COUNT(TIMEOUT_CYCLES,9)
trail_timeout_loop:
/* 0 +  1  */ in rSTATUS1,    _SFR_IO_ADDR(USISR)
/* 1 +  1  */ in rDATA,       _SFR_IO_ADDR(USIDR)
/* 2 + 1|2 */ sbrc rSTATUS1, USIOIF
/* 3 + 1|0 */ in rUNUSED,     _SFR_IO_ADDR(USIBR)
/* 4 + 1|2 */ sbrc rSTATUS1, USIOIF
/* 5 + 2|0 */ rjmp bit7_poll
/* 6 +  1  */ dec rLOOP
/* 7 + 1|2 */ brne trail_timeout_loop

rjmp wait_for_data

