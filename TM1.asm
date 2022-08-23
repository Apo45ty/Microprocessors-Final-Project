; Antonio Tapia Maldonado
; 841-09-8784
; Problem: Develop a program that 
; Solution: 

#include "msp430.h"

;-------------------------------------------------------------
;                Binary to BCD macro converter
; Inputs:
;	- none
; Outputs:
;	- none
;
; Description: This macro assumes that you have initialized the stack 
; pointer.  
;-------------------------------------------------------------
BIN2BCD		MACRO	A
		LOCAL	Finish,B2BCD
                push R6
		mov	#0,R6
B2BCD	
		cmp	#0,A
		jz	Finish
		dec	A
		clrc
		dadd.w	#1,R6
		jmp B2BCD
Finish		mov	R6,A
                pop R6
		ENDM
		
#define   BCD1             0230h
#define   BCD2             0232h
#define   BCD3             0234h
#define   BCD4             0236h
#define   TACOU            0238h
#define   REPCOU           023Ah
#define   CONS             023Eh
#define   SEC              0240h
#define   INTFLAGS         0242h
#define   TIMMERDELAY_COU  0244h
#define   FACT             0246h
#define   INT              0248h
#define   RESULT           024Ah
#define   FACTBCD          024Ch
#define   INTBCD           024Eh
#define   TEMP             0250h

CANINT1 equ 1
CANINT2 equ 2
TIMMERDELAY equ 5; in secons
INTDELAY equ 50 ;in mili seconds

;-------------------------------------------------------------------------------
ORG   0C000h     ; Program Start Begining of flash memory
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
;                   Multiplication Sub-Routine
; Inputs:
;	- R12 first in fixpoint 8.8
;       - R13 second in fixpoint 8.8
; Outputs:
;	- R12 in fixpoint 8.8
;       - &Frac fractional part in in fixpoint 0.16
;       - &Int integer part in in fixpoint 16.0
;
; Description: This part of the code multiplies by the constant values 0.0479 .
;-------------------------------------------------------------------------------
mult:     push R4
          push R5
	  push R6
          push R7
	  push R8
	  mov #0,&FACT
	  mov #0,&INT
	  mov #0,R4
	  mov R12,R5
again     cmp #16,R4
	  jz multEnd
          inc R4
          rra R5
	  jnc again
	  mov R13,R6
	  mov #0,R7
          mov #1,R8
Loop	  cmp R4,R8
	  jeq LoopEnd
          rla  R6
	  rlc  R7
	  inc R8
	  jmp Loop
LoopEnd	  add R6,&FACT
	  addc R7,&INT
	  jmp again
	  
multEnd   mov #0,&(RESULT)
          mov.b &(INT),&(RESULT+1)
          add.b &(FACT+1),&(RESULT)
	  mov &(RESULT),R12
	  pop R8
	  pop R7
	  pop R6
	  pop R5
	  pop R4
	  ret
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
;  The interrupt service routine for the MSP430 Timer_A0
; Inputs:
;	- none
; Outputs:
;	- none
; Description: This subroutine assumes that you have initialized the stack 
; pointer. 
;-------------------------------------------------------------------------------					
TIMERAINT   inc &TACOU
	    bit #CANINT2, &INTFLAGS
	    jnz secs
	    ;bic.w #GIE,SR
	    push &TIMMERDELAY_COU
	    sub &TACOU,&TIMMERDELAY_COU
	    cmp #INTDELAY,&TIMMERDELAY_COU
	    jnc secs
	    bis #CANINT2, &INTFLAGS
	    pop &TIMMERDELAY_COU
	    ;bis.w #GIE,SR
secs        cmp #1000,&TACOU;
 	    jnc TIMERAEND2 
	    mov.w #0, &TACOU;
	    inc &SEC
	    cmp #TIMMERDELAY,&SEC
	    jnc TIMERAEND2
TIMERAEND:  bic.w #GIE,SR
	    mov.b #2,&P2OUT
            ;mutiliply const by rep count
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    rla &REPCOU
	    mov.w &REPCOU,R12
	    mov.w &CONS,R13
	    call #mult
	    
	    push R8
	    mov &FACT,&TEMP
	    mov #0,&FACTBCD
	    mov #0,R8
Loop2	    cmp #4,R8
	    jeq LoopEnd2
	    inc R8
	    rla  &TEMP
	    jnc Loop2
; DO LOOK UP TABLE
	    cmp #1,R8
	    jne case2
	    add #3200h,&FACTBCD ;50
case2	    cmp #2,R8
	    jne case3
	    add #1900h,&FACTBCD ;25
case3	    cmp #3,R8
	    jne case4
	    add #0C00h,&FACTBCD ;12
case4	    cmp #4,R8
	    jne case5
	    add #0600h,&FACTBCD ;6
case5       cmp #5,R8
	    jne case6
	    add #0000h,&FACTBCD ;
case6	    cmp #6,R8
	    jne case7
	    add #0000h,&FACTBCD ;
case7	    cmp #7,R8
	    jne case8
	    add #0000h,&FACTBCD ;
case8	    add #0000h,&FACTBCD ;
	    jmp Loop2
LoopEnd2    pop R8

	    BIN2BCD &FACTBCD
	    
	    mov &INT,&INTBCD
	    BIN2BCD &INTBCD
	    
	    mov.b &(INTBCD),&(TEMP+1)
            add.b &(FACTBCD),&(TEMP)
	    
            mov.b &TEMP, &BCD4;
	  
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP 
	    mov.b &TEMP, &BCD3;
	  
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP
	    mov.b &TEMP, &BCD2;
	  
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP
	    RRA.w &TEMP
	    mov.b &TEMP, &BCD1; 
	    
	    mov.w #0, &SEC;
	    mov.w #0, &REPCOU;
	    mov.b #0, &P2OUT
	    bis.w #GIE,SR
TIMERAEND2:
	    reti     
;-------------------------------------------------------------------------------
;                   LED 01 SubRoutine
; Inputs:
;	- none
; Outputs:
;	- none
;
; Description: This subroutine assumes that you have initialized the stack 
; pointer.  
;-------------------------------------------------------------------------------
LED1:    bic.b #00000100b,&P2IFG
	 bic.w #GIE,SR
         bit #CANINT1, &INTFLAGS
         jz endLED1
	 bit #CANINT2, &INTFLAGS
	 jz endLED1
	 inc &REPCOU
	 bic.w #CANINT1+CANINT2, &INTFLAGS
	 mov &TACOU,&TIMMERDELAY_COU
pollLED1 bit.b #00000100b,&P2IN
	 jnz pollLED1
         bis.w #GIE,SR
	 bis #CANINT1, &INTFLAGS
endLED1: reti
;-------------------------------------------------------------------------------
;                   Delay Routine 
; Inputs:
;	- R7 pass the total number of loops for the second loop
;       - R8 pass the total number of loops for the first loop
; Outputs:
;	- none
;
; Description: This subroutine assumes that you have initialized the stack 
; pointer. And it delays program execution for aproximately 65,000*5 clock 
; cycles for the MSP430G2553 in ultra low power runs at 1 MHz clock frequency. 
; From this we can deduct that the clock executes an instruction every 333khz 
; and thus this delay without considering the delay associated with the 
; subroutine overhead is 325khz or have a second
;-------------------------------------------------------------------------------
delay:     push R15 			; Save the state of register 15
	   push R6			; Save the state of register 6
	  
;Main loop for toogle
Wait1:	   mov.w   R7,R6                ; Set the main counter clock 
Wait2:	   mov.w   R8,R15               ; Set the secondary counter clock
L1:        dec   R15                    ; Decrement the main timer
           jnz   L1                     ; Secondary Delay over?
	   dec   R6			; Decremnt the second timer
	   jnz   Wait2			; Start over if not done
	  
	   pop R6 			; Restore the state of register 6
	   pop R15			; Restore the state of register 15
	   ret
;-------------------------------------------------------------------------------
;                   Main Program Routine
; Inputs:
;	- none
; Outputs:
;	- none
;
; Description:
;-------------------------------------------------------------------------------
RESET:     mov   #03FEh,SP                 ; Initialize stackpointerTBCTL
StopWDT:   mov   #WDTPW+WDTHOLD,&WDTCTL    ; Stop WDT

;Set up Timmer A
	   bis.b   #LFXT1S_2,&BCSCTL3           ; ACLK = VLO (Very Low Clock 12KHz)
           mov.w   #10h,&CCTL0                  ; CCR0 interrupt enabled, bit 4 = 1
           mov.w   #12,&CCR0                    ; Count 12 to CCR0 MILISECS
           mov.w   #TASSEL_1+MC_1,&TACTL        ; ACLK source (~12KHz), UP mode (count up to CCR0 value)
	   
;Set up IO ports
	   bis.b #11111111b,&P1DIR        ; P1.0-P1.7 as output port
           bis.b #00000000b,&P2OUT        ; Clear output register
	   mov.b #11111011b,&P2DIR        ; P2.2 input port 
	   bis.b #00000100b,&P2IE         ; enable P2.0 interrupt
           mov.b #11111011b,&P2IES        ; P2.2 hi/low edge
           bic.b #00000100b,&P2IFG        ; P2.2 IFG Cleared
           bis.w #GIE,SR                  ; enable interrupts 
	   
;Set up Heap me
           mov.w #0,  &BCD1                ; Clear BCD Value 1
           mov.w #0,  &BCD2                ; Clear BCD Value 2
           mov.w #0,  &BCD3                ; Clear BCD Value 3
           mov.w #0,  &BCD4                ; Clear BCD Value 4
           mov.w #0,  &TACOU               ; Milisecond counter  
	   mov.w #0,  &TIMMERDELAY_COU     ; Last p2 interrupt time
	   mov.w #0,  &SEC                 ; Second counter
	   mov.w #0,  &REPCOU              ; Number of Reps per minute
	   mov.b #00001100b,  &CONS         ; Constant value, This part of the code multiplies by the constant values 0.0479 . 
                                           ; Which is the constant value mins*KiloMeters/Hours and it is the result of the 
                                           ; following values: 
                                           ; Radius * minutesRemainingFrom * convertionFromRpm * Km/cm * mins/H 
	   mov.w #CANINT1+CANINT2, &INTFLAGS
           mov.w #0001h,R7                ; set the delay parameter 1
           mov.w #0060h,R8                ; set the delay parameter 2
	   
main:      mov.b  &BCD1,&P1OUT            ; Move BCD Value 1 to output register
	   bic.b  #11110000b,&P1OUT       ; Clear the MSN of P1OUT
           bis.b  #00010000b,&P1OUT       ; turn on first Seven Segment 
           call   #delay                  ; Call the delay subroutine with the parameters set in R7 and R8
           
	   mov.b  &BCD2,&P1OUT            ; Move BCD Value 2 to output register
	   bic.b  #11110000b,&P1OUT       ; Clear the MSN of P1OUT
           bis.b  #00100000b,&P1OUT       ; Turn on the third digit 
           call   #delay                  ; Call the delay subroutine with the parameters set in R7 and R8
           
	   mov.b  &BCD3,&P1OUT            ; Move BCD Value 3 to output register
	   bic.b  #11110000b,&P1OUT       ; Clear the MSN of P1OUT
           bis.b  #01000000b,&P1OUT       ; Turn on the third digit
           call   #delay		  ; Call the delay subroutine with the parameters set in R7 and R8
           
	   mov.b  &BCD4,&P1OUT            ; Move BCD Value 4 to output register
	   bic.b  #11110000b,&P1OUT       ; Clear the MSN of P1OUT
           bis.b  #10000000b,&P1OUT       ; Turn on the fouth digit
           call   #delay		  ; Call the delay subroutine with the parameters set in R7 and R8
	   
           jmp main                       ; Restart Main Loop   
;-------------------------------------------------------------------------------
; 	Interrupt Vectors Setup
;-------------------------------------------------------------------------------
		ORG 0FFFEh           ; MSP430 interrupt vectors
		DW  RESET            ; Define word for Address label of RESET
		ORG 0FFE6h           ; Memory address for Interrupt Vector P2.2
		DW  LED1             ; Define word for Address label Address of label 
                ORG 0FFF2h           ; MSP430 Timer_A0 Vector Address
                DW  TIMERAINT        ; 
                END