#include <avr/io.h>

/*
 * FAILED :-( SPI compatible (WS2801) to WS2811/WS2812(b) protocol translator
 */

.section .bss

/* use ALL THE MEMORY! */
#define BUFSTART ( RAMSTART )
#define BUFEND   ( RAMEND + 1 )

#define OUTPIN 4
#define OUTDDR DDRB
#define OUTPORT PORTB
#define BUFCACHE 3 /* start transmitting when 3 bytes have been received */

#if ( BUFCACHE > 255 ) != 0
	#error "buffer cache size should fit in a 8 bit register"
#endif

#define TIMEOUT_CYCLES (50*8)
#define TIMEOUT_LOOP_COUNT(cycles, cycles_per_iteration) ( (cycles+cycles_per_iteration-1)/cycles_per_iteration )

#define rTMP r16
#define rLOOP r17
#define rLOW r18
#define rHIGH r19
#define rRESETSPI r20
#define rDATA r21
#define rNEWDATA r22
#define rHIBUFEND r23

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
main:

init:

/* Load constants */
ldi rLOW, 0                   /* bitbang low                                               */
ldi rHIGH, _BV(OUTPIN)        /* bitbang high                                              */
ldi rRESETSPI,_BV(USISIF)|_BV(USIOIF)  /* data start/available flags + 0 in 4 bit counter  */
ldi rHIBUFEND, hi8(BUFEND)    /* register needed for compare-with-carry                    */
ldi loBUF, lo8(BUFSTART)      /* use Z to store a pointer to the start of the buffer       */
ldi hiBUF, hi8(BUFSTART)      /*                                                           */

/* Output pin setup */
out _SFR_IO_ADDR(OUTDDR),  rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW

/* SPI slave setup */
;ldi rTMP, _BV(USIWM0)|_BV(USICS1) /* USI in three-wire mode, external clock */
ldi rTMP, _BV(USICS1) /* USI in input-only-mode, external clock */
out _SFR_IO_ADDR(USICR), rTMP

wait_for_data:
out _SFR_IO_ADDR(USISR), rRESETSPI /* Clear(!) data available flag, reset counter(!) */

movw loLOAD,  loBUF  /* pLOAD  = pBUF */
movw loSTORE, loBUF  /* pSTORE = pBUF */

wait_for_communication:
sbis _SFR_IO_ADDR(USISR), USISIF
rjmp wait_for_communication

cache_loop:
ldi rLOOP, TIMEOUT_LOOP_COUNT(TIMEOUT_CYCLES, 5)
timeout_loop:
dec rLOOP
sbic _SFR_IO_ADDR(USISR), USIOIF
rjmp cache_data
brne timeout_loop
rjmp wait_for_data /* timeout, data considered noise, start over */
cache_data:
in rNEWDATA, _SFR_IO_ADDR(USIBR)
st pSTORE+, rNEWDATA
sbi _SFR_IO_ADDR(USISR), USIOIF
cpi loSTORE, lo8(BUFSTART+BUFCACHE)
brne cache_loop
ld rDATA, pLOAD+
rjmp bit7

bit0_exit:
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  1  */ ldi rLOOP, TIMEOUT_LOOP_COUNT(TIMEOUT_CYCLES, 4)
tail_timeout_loop:
sbic _SFR_IO_ADDR(USISR), USIOIF
rjmp extra_data
dec rLOOP
brne tail_timeout_loop
rjmp wait_for_data

extra_data:
sbi	_SFR_IO_ADDR(USISR), USIOIF /* Clear data available flag */
in rDATA, _SFR_IO_ADDR(USIBR)
rjmp bit7

/* None of the bitbanging functions touch the flags, yay \o/ */

              /* bit bang */                           /* Check for & store new input */
bit4:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          cpi loSTORE, lo8(BUFEND-1) /* calc before increment */
/* 2 + 1|2 */ sbrs rDATA, 4
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 + 1|2 */                                          sbis _SFR_IO_ADDR(USISR), USIOIF
/* 5 + 2|0 */                                          rjmp bit4_nodata
/* 6 +  1  */                                          in rNEWDATA, _SFR_IO_ADDR(USIBR)
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          sbi _SFR_IO_ADDR(USISR), USIOIF /* clear data-avail-bit */
bit3_newdata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 3
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          st pSTORE+, rNEWDATA
/* 6 +  1  */                                          cpc hiSTORE, rHIBUFEND
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 + 1|2 */                                          brne 1f
/* 9 + 1|0 */                                          movw loSTORE, loBUF /* pSTORE = pBUF */
1:

              /* bit bang */                           /* Check for & store new input */
bit2:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          cpi loSTORE, lo8(BUFEND-1) /* calc before increment */
/* 2 + 1|2 */ sbrs rDATA, 2
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 + 1|2 */                                          sbis _SFR_IO_ADDR(USISR), USIOIF
/* 5 + 2|0 */                                          rjmp bit2_nodata
/* 6 +  1  */                                          in rNEWDATA, _SFR_IO_ADDR(USIBR)
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          sbi _SFR_IO_ADDR(USISR), USIOIF /* clear data-avail-bit */
bit1_newdata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 1
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          st pSTORE+, rNEWDATA
/* 6 +  1  */                                          cpc hiSTORE, rHIBUFEND
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 + 1|2 */                                          brne 1f
/* 9 + 1|0 */                                          movw loSTORE, loBUF /* pSTORE = pBUF */
1:

              /* bit bang */                           /* Check for end of buffer / Load rDATA */

bit0:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          cp loSTORE, loLOAD
/* 2 + 1|2 */ sbrs rDATA, 0
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  1  */                                          cpc hiSTORE, hiLOAD
/* 5 + 1|2 */                                          breq bit0_exit /* We'll ignore the buffer full condition */
/* 6 +  1  */                                          nop
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          ld rDATA, pLOAD+

              /* bit bang */                           /* Check for & store new input */
bit7:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          cpi loSTORE, lo8(BUFEND-1) /* calc before increment */
/* 2 + 1|2 */ sbrs rDATA, 7
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 + 1|2 */                                          sbis _SFR_IO_ADDR(USISR), USIOIF
/* 5 + 2|0 */                                          rjmp bit7_nodata
/* 6 +  1  */                                          in rNEWDATA, _SFR_IO_ADDR(USIBR)
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          sbi _SFR_IO_ADDR(USISR), USIOIF
bit6_newdata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 6
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          st pSTORE+, rNEWDATA
/* 6 +  1  */                                          cpc hiSTORE, rHIBUFEND
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 + 1|2 */                                          brne 1f
/* 9 + 1|0 */                                          movw loSTORE, loBUF /* pSTORE = pBUF */
1:

              /* bit bang */                           /* Wrap-around for load pointer */
bit5:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          cpi loLOAD, lo8(BUFEND)
/* 2 + 1|2 */ sbrs rDATA, 5
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  1  */                                          cpc hiLOAD, rHIBUFEND
/* 5 + 1|2 */                                          brne 1f
/* 6 + 1|0 */                                          movw loLOAD, loBUF /* pLOAD = pBUF */
1:
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp bit4

/* less cycle-intensive branches: */

bit7_nodata:
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp 1f
1:
bit6_nodata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 6
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          rjmp 1f
1:
/* 6 +  1  */                                          nop
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp bit5


bit4_nodata:
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp 1f
1:
bit3_nodata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 3
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          rjmp 1f
1:
/* 6 +  1  */                                          nop
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp bit2


bit2_nodata:
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp 1f
1:
bit1_nodata:
/* 0 +  1  */ out _SFR_IO_ADDR(OUTPORT), rHIGH
/* 1 +  1  */                                          nop
/* 2 + 1|2 */ sbrs rDATA, 1
/* 3 + 1|0 */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 4 +  2  */                                          rjmp 1f
1:
/* 6 +  1  */                                          nop
/* 7 +  1  */ out _SFR_IO_ADDR(OUTPORT), rLOW
/* 8 +  2  */                                          rjmp bit0

