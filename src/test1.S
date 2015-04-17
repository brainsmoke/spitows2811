#include <avr/io.h>

/*
 * SPI compatible (WS2801) to WS2811/WS2812(b) protocol translator
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


#define CLEAR_AVAIL_BIT cbi _SFR_IO_ADDR(USISR), USIOIF
#define TEST_AVAIL_BIT sbis _SFR_IO_ADDR(USISR), USIOIF
#define SPI_IN_REG

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
;ldi rTMP, _BV(USIWM0)|_BV(USICS1)|_BV(USIOIE) /* USI in three-wire mode, external clock */
ldi rTMP, _BV(USICS1) /* USI in input-only-mode, external clock */
out _SFR_IO_ADDR(USICR), rTMP

reset:
out _SFR_IO_ADDR(USISR), rRESETSPI /* Clear(!) data available flag, reset counter(!) */

wait_for_communication:
sbis _SFR_IO_ADDR(USISR), USISIF
rjmp wait_for_communication

ldi rLOOP, 50
dec rLOOP
loop:
out _SFR_IO_ADDR(OUTPORT), rHIGH
nop
nop
nop
nop
nop
out _SFR_IO_ADDR(OUTPORT), rLOW
nop
sbic _SFR_IO_ADDR(USIDR), 0
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
;nop
;sbic _SFR_IO_ADDR(USIBR), 0
;out _SFR_IO_ADDR(OUTPORT), rHIGH
;out _SFR_IO_ADDR(OUTPORT), rLOW
nop
sbic _SFR_IO_ADDR(USISR), USIOIF
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
in rTMP, _SFR_IO_ADDR(USISR)
sbrc rTMP, USIOIF
ldi rLOOP, 50
dec rLOOP

sbrc rTMP, USIOIF
sbi _SFR_IO_ADDR(USISR), USIOIF

sbrc rTMP, 7
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 6
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 5
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 4
out _SFR_IO_ADDR(OUTPORT), rHIGH
sbrc rTMP, 3
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 2
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 1
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
sbrc rTMP, 0
out _SFR_IO_ADDR(OUTPORT), rHIGH
out _SFR_IO_ADDR(OUTPORT), rLOW
brne loop
rjmp reset

