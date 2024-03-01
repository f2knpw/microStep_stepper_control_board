; µstep V2.0 firmware - PIC based microstepping motor controller
; copyright (C) 2009 Jean-Pierre Gleyzes <freedom2000@free.fr>

; based on
; PICStep v2.0 Firmware - PIC based microstepping motor controller
; Copyright (C) 2004 Alan Garfield <alan@fromorbit.com>

; evolutions from PicStep's design :
; fully ported on PIC16F57 (baseline PIC : no interruptions, different pinout...)
; adapted to Mdog's µstep board design 
; includes 1/2, 1/4, 1/8, 1/16 µstep mode
; includes different DAC tables for LMD18245T drivers
; includes idle mode current reduction :
	; after 30s 75%
	; after 90s 40%
; new in V2 possible control of current reduction mode with Mach3 "Current Hi/Low" signal
	; when set to 1 --> forces DAC to previous full power values (no loss of step !)
	; when 0 AND more than 10s inactivity on ALL axes : 75% power
	; when 0 AND more than 30s inactivity on ALL axes : 40% power

; performances :
; step interrupt detection 
	; 0 to 1 edge on pin RA1
	; time min to detect step interrupt : 			 6 cycles 	-->      t0 =  1.2 µs (with current reduction mode)
	; time min to detect step interrupt : 			 4 cycles 	-->      t0 =  0.8 µs (without current reduction mode)
	; time max to detect and process interrupt : 	43 cycles 	--> t0 + t1 =  8.6 µs (with current reduction mode)
	; time max to detect and process interrupt : 	32 cycles 	--> t0 + t1 =  6.4 µs (without current reduction mode)
	; timemax to detect and process time out :		63 cycles	--> t0 + t2 = 12.6 µs
	; timemax to recover from current reduction max 43 cycles   -->      t3 =  8.6 µs (under Mach3 control "Current Hi/Low" signal set to 1

; step width MUST be greater than 1.2 µs to be detected (with current reduction mode)
; step width MUST be greater than 0.8 µs to be detected (without current reduction mode)
; step frequency MUST NOT be greater than 1/8.6 MHz = 116279 Hz in order to NOT loose steps (with current reduction mode)
; step frequency MUST NOT be greater than 1/6.4 MHz = 156250 Hz in order to NOT loose steps (without current reduction mode)
; so to be safe :
	; use Mach3 kernel speed up to 100 kHz (maximum possible value of Mach3 in 2009 !!!)
	; set Step Pulse width to 2 or 3 µs under Mach3

; V2.1 added self test mode : don't connect your PC // port. (compile with "self_test_mode" enabled)
; Just connect the motor and the alimentations (5V + motor) and the motor will very gently turn without step and dir signals.
	

; This program is free software; you can redistribute it and/or
; modify it under the terms of the GNU General Public License
; as published by the Free Software Foundation; either version 2
; of the License, or (at your option) any later version.

; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.

; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

;**********************************************************************
;                                                                     *
;    Filename:	    16F57µstep.asm                                           *
;    Date:          25/12/2009                                                  *
;    File Version:  2.1   (christmas release !)                                               *
;                                                                     *
;    Author:        JP Gleyzes                                                  *
;    Company:                                                         *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files Required: P16F57.INC                                       *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: based on Pic Step source code                                                          *
;                                                                     *
;**********************************************************************

	list      p=16F57             ; list directive to define processor
	#include <p16f5x.inc>         ; processor specific variable definitions

	__CONFIG   _CP_OFF & _WDT_OFF & _HS_OSC

; '__CONFIG' précise les paramètres encodés dans le processeur au moment de
; la programmation. Les définitions sont dans le fichier include.
; Voici les valeurs et leurs définitions :
;	_CP_ON		Code protection ON : impossible de relire
;	_CP_OFF		Code protection OFF
;	_WDT_ON		Watch-dog en service
;	_WDT_OFF	Watch-dog hors service
;	_LP_OSC		Oscillateur quartz basse vitesse
;	_XT_OSC		Oscillateur quartz moyenne vitesse
;	_HS_OSC		Oscillateur quartz grande vitesse
;	_RC_OSC		Oscillateur à réseau RC


; please select only one option from the following

#define		SINUS_ENABLED		; choose the classical Sinus table
;#define	POWER_TORQUE_ENABLED	; choose the maximum power, maximum precision table
;#define		reverse_polarity		; for interpCNC board (reversed polarity)


#define 	TIMEOUT_ENABLED		; enable power reduction after 30s timeout of inactivity

;#define		Mach3_idle_current_control ; allows control of cirrent reduction with Mach3 (signal "Current Hi/Low" on pin RC6 of Port C)


;#define		self_test_mode		; allows to auto generate step signal to auto test the driver without PC

;*********************************************************************
;                              ASSIGNATIONS                          *
;*********************************************************************

TMR_loop1_max	EQU	H'00FE'		; max value for timer_loop1 (don't change it)

#ifdef 			Mach3_idle_current_control
TMR_loop2_max	EQU	H'0060'		; max value for timer_loop2
								; h60  --> 15,04s
								; hC0  --> 30,08s
								; hFF  --> 39,95s
#else
TMR_loop2_max	EQU	H'0090'		; max value for timer_loop2
								; h60  --> 15,04s
								; hC0  --> 30,08s
								; hFF  --> 39,95s
#endif 

;TMR_loop1_max	EQU	H'0003'		; max value for timer_loop1 for test
;TMR_loop2_max	EQU	H'0002'		; max value for timer_loop1 for test

TIME_to_Low		EQU H'00FD'		; increment to 0 before low current reduction mode



;*********************************************************************
;                   DECLARATIONS DE VARIABLES                        *
;*********************************************************************


	CBLOCK 0x008   				; début de la zone variables
	step 			: 1
	stepb			: 1
	mode 			: 1
	mode_value		: 1
	lookup 			: 1
	maxbuf			: 1
	quarter_table	: 1
	timer_loop1		: 1
	timer_loop2		: 1
	IT_timer 		: 1
	time_out_value	: 1
	time_out_value2	: 1
	in_idle_mode	: 1			; =0 if a regular current value has been loaded in the DACs (#0 if a current reduction value)
	CounterA		: 1
	CounterB		: 1
	ENDC						; Fin de la zone                        


;**********************************************************************
;                      DEMARRAGE SUR RESET                            *
;**********************************************************************

	ORG     0x7FF             ; processor reset vector
		errorlevel -306			  ; supress page boundary message
		goto    init
		errorlevel +306 		  ; enable page boundary message
		ORG     0x000




;**********************************************************************
;                     "INTERRUPTION" TIMER 0                            *
;**********************************************************************
#ifdef TIMEOUT_ENABLED
trait_timer
	decfsz 		timer_loop1, f
	GOTO 		reset_tmr0
	decfsz		timer_loop2,f
	goto 		reset_tmr0_loop2
interupt_tmr0
	movlw		TMR_loop1_max	; reload timer_loop1
	movwf		timer_loop1
	movlw		TMR_loop2_max	; reload timer_loop2
	movwf		timer_loop2
	clrf		TMR0
	retlw		0				; current reduction mode to be processed
reset_tmr0_loop2
	movlw		TMR_loop1_max	; reload timer_loop1
	movwf		timer_loop1
	clrf		TMR0
	retlw		1
reset_tmr0
	clrf		TMR0
	retlw		1

reset_timer						; reset all timer's variables
	movlw		TMR_loop1_max
	movwf		timer_loop1
	movlw		TMR_loop2_max
	movwf		timer_loop2 
	movlw		0x001
	movwf		IT_timer 		; clear timer "interrupt"
	clrf		TMR0
	retlw		0
#endif


;***************************************************************************************************

MODE_TABLE						; returns the µstepping mode
  addwf		PCL, 1
	retlw		0x001	; 1/16
	retlw		0x002	; 1/8
	retlw		0x004	; 1/4
 	retlw		0x008	; 1/2



;*********************************************************************
;                       table sinus                                  *
;*********************************************************************
; Cette fonction renvoie l'ensemble des valeurs des DAC pour
; un pas donné (max 1/16e de pas, soit 16*4 = 64 valeurs)
; Les valeurs sont envoyées aux DAC par appel de cette fonction
; avec en parametre le nouvelle position à adopter.
; Les valeurs négatives sur les DAC sont obtenues en passant
; le bit 4 à "1" quand on parcourt la table de haut en bas 
; Ainsi l'info de signe est envoyée aux LMD18245 sur leur entrée DIR. 
; Notez que M4 est connecté au bit 0, M3 au bit 1 etc. 
; Cette table tient compte de l'inversion :
	; les valeurs décodées en décimal sont les entrées du DAC
	; les valeurs codées binaires (bits 0 à 3) sont les sorties des ports
; il existe une table pou rle sens "haut --> bas" de lecture la table
; et une table pour le sens "bas --> haut" de lecture de la table. 
; Ceci permet de corriger le "bug" du datasheet du LMD et donc de changer le sens des courants uniquement quand le courant vaut 0 dans la bobine


#ifdef SINUS_ENABLED	; sinus table choosen

; 1/16 Step DAC A Table (port B)
	;ORG		0x200		; located in bank 1 = Page 2 (200h-3FFh)
STEP_TABLE
	;SINUS_TABLE_FULL : sens haut --> bas
	;signeF1F2F3F4
	addwf	PCL, 1		;	DAC A
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'00111'	;	14
	retlw	B'00111'	;	14
	retlw	B'01011'	;	13
	retlw	B'00011'	;	12
	retlw	B'01101'	;	11
	retlw	B'00101'	;	10
	retlw	B'01001'	;	9
	retlw	B'00001'	;	8
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'01100'	;	3
	retlw	B'01000'	;	1
	retlw	B'00000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'11100'	;	-3
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'10001'	;	-8
	retlw	B'11001'	;	-9
	retlw	B'10101'	;	-10
	retlw	B'11101'	;	-11
	retlw	B'10011'	;	-12
	retlw	B'11011'	;	-13
	retlw	B'10111'	;	-14
	retlw	B'10111'	;	-14
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'10111'	;	-14
	retlw	B'10111'	;	-14
	retlw	B'11011'	;	-13
	retlw	B'10011'	;	-12
	retlw	B'11101'	;	-11
	retlw	B'10101'	;	-10
	retlw	B'11001'	;	-9
	retlw	B'10001'	;	-8
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'11100'	;	-3
	retlw	B'11000'	;	-1
	retlw	B'10000'	;	0
	retlw	B'01000'	;	1
	retlw	B'01100'	;	3
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'00001'	;	8
	retlw	B'01001'	;	9
	retlw	B'00101'	;	10
	retlw	B'01101'	;	11
	retlw	B'00011'	;	12
	retlw	B'01011'	;	13
	retlw	B'00111'	;	14
	retlw	B'00111'	;	14
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15

STEP_TABLE_UP	
	;SINUS_TABLE_FULL : sens bas --> haut
	;signeF1F2F3F4
	addwf	PCL, 1		;	DAC A
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'00111'	;	14
	retlw	B'00111'	;	14
	retlw	B'01011'	;	13
	retlw	B'00011'	;	12
	retlw	B'01101'	;	11
	retlw	B'00101'	;	10
	retlw	B'01001'	;	9
	retlw	B'00001'	;	8
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'01100'	;	3
	retlw	B'01000'	;	1
	retlw	B'10000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'11100'	;	-3
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'10001'	;	-8
	retlw	B'11001'	;	-9
	retlw	B'10101'	;	-10
	retlw	B'11101'	;	-11
	retlw	B'10011'	;	-12
	retlw	B'11011'	;	-13
	retlw	B'10111'	;	-14
	retlw	B'10111'	;	-14
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'11111'	;	-15
	retlw	B'10111'	;	-14
	retlw	B'10111'	;	-14
	retlw	B'11011'	;	-13
	retlw	B'10011'	;	-12
	retlw	B'11101'	;	-11
	retlw	B'10101'	;	-10
	retlw	B'11001'	;	-9
	retlw	B'10001'	;	-8
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'11100'	;	-3
	retlw	B'11000'	;	-1
	retlw	B'00000'	;	0
	retlw	B'01000'	;	1
	retlw	B'01100'	;	3
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'00001'	;	8
	retlw	B'01001'	;	9
	retlw	B'00101'	;	10
	retlw	B'01101'	;	11
	retlw	B'00011'	;	12
	retlw	B'01011'	;	13
	retlw	B'00111'	;	14
	retlw	B'00111'	;	14
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15
	retlw	B'01111'	;	15

#endif

#ifdef POWER_TORQUE_ENABLED ; maximum torque maximum precision table choosen

STEP_TABLE				;  sens haut --> bas
	addwf	PCL, 1		;	DAC A
		;SigneM1M2M3M4
	retlw	B'10000'	;	 0
	retlw	B'01000'	;	 1
	retlw	B'01100'	;	 3
	retlw	B'00010'	;	 4
	retlw	B'00110'	;	 6
	retlw	B'00001'	;	 8
	retlw	B'00101'	;	 10
	retlw	B'00011'	;	 12
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00111'	;	 14
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00111'	;	 14
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00011'	;	 12
	retlw	B'00101'	;	 10
	retlw	B'00001'	;	 8
	retlw	B'00110'	;	 6
	retlw	B'00010'	;	 4
	retlw	B'01100'	;	 3
	retlw	B'01000'	;	 1
	retlw	B'00000'	;	 0
	retlw	B'11000'	;	 -1
	retlw	B'11100'	;	 -3
	retlw	B'10010'	;	 -4
	retlw	B'10110'	;	 -6
	retlw	B'10001'	;	 -8
	retlw	B'10101'	;	 -10
	retlw	B'10011'	;	 -12
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10111'	;	 -14
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10111'	;	 -14
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10011'	;	 -12
	retlw	B'10101'	;	 -10
	retlw	B'10001'	;	 -8
	retlw	B'10110'	;	 -6
	retlw	B'10010'	;	 -4
	retlw	B'11100'	;	 -3
	retlw	B'11000'	;	 -1

STEP_TABLE_UP			; sens bas --> haut 
	addwf	PCL, 1		;	DAC A
		;SigneM1M2M3M4
	retlw	B'00000'	;	 0
	retlw	B'01000'	;	 1
	retlw	B'01100'	;	 3
	retlw	B'00010'	;	 4
	retlw	B'00110'	;	 6
	retlw	B'00001'	;	 8
	retlw	B'00101'	;	 10
	retlw	B'00011'	;	 12
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00111'	;	 14
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00111'	;	 14
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'01111'	;	 15
	retlw	B'00011'	;	 12
	retlw	B'00101'	;	 10
	retlw	B'00001'	;	 8
	retlw	B'00110'	;	 6
	retlw	B'00010'	;	 4
	retlw	B'01100'	;	 3
	retlw	B'01000'	;	 1
	retlw	B'10000'	;	 0
	retlw	B'11000'	;	 -1
	retlw	B'11100'	;	 -3
	retlw	B'10010'	;	 -4
	retlw	B'10110'	;	 -6
	retlw	B'10001'	;	 -8
	retlw	B'10101'	;	 -10
	retlw	B'10011'	;	 -12
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10111'	;	 -14
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10111'	;	 -14
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'11111'	;	 -15
	retlw	B'10011'	;	 -12
	retlw	B'10101'	;	 -10
	retlw	B'10001'	;	 -8
	retlw	B'10110'	;	 -6
	retlw	B'10010'	;	 -4
	retlw	B'11100'	;	 -3
	retlw	B'11000'	;	 -1
#endif


;*********************************************************************
;                       INITIALISATIONS                              *
;*********************************************************************

init
	clrf	PORTA				; Sorties portA à 0
	clrf	PORTB				; sorties portB à 0
    clrf	PORTC				; sorties portC à 0

	movlw		B'11111111'		;Set RA<0:3> as inputs
	tris		PORTA 

	movlw		B'11100000'		;Set RB<0:4> as outputs
	tris		PORTB 

	movlw		B'11100000'		;Set RB<0:4> as outputs
	tris		PORTC 

	clrf		step
	clrf		lookup
	movlw		0x03F
	movwf		maxbuf 
	movlw		.48
	movwf		quarter_table	 ; loads the 1/4 value of the table to shift 1/4 period
	movwf		stepb

#ifdef TIMEOUT_ENABLED
	CLRWDT 						;Clear WDT and prescaler
	MOVLW 		B'00000011'		;Select TMR0, new prescale value and clock source(internal)
	OPTION

	call 		reset_timer 	;reset time out for current reduction
	movlw		TIME_to_Low		; set time out value to FD, 3*time_out later it will be 0
	movwf		time_out_value
	clrf		time_out_value2
	incf		time_out_value2 ; set to 1
#endif

;********************************************************
	; Monitor the mode switches 
	; J1 = RA3
	; J2 = RA2
	; jumper ON = Pin to 5V = "1 state"
		; J1	J2	µstep
		; 0	0	1/16
		; 0	1	1/8
		; 1	0	1/4
		; 1	1	1/2
	movf		PORTA, w
	andlw		B'00001100'
	movwf		mode	; RA2 and RA3 values : microstep mode
	rrf 		mode, f
	rrf 		mode, f
	movlw		B'00000111'
	andwf		mode, f
	movf		mode, w			; Load the current mode
	call		MODE_TABLE		; Get the advance value for this mode
	movwf		mode_value		; Store it for later	

;**********************************************************
;step signal detection
boucle
#ifdef self_test_mode			; self generation of pulses without PC and step signal
	
;PIC Time Delay = 0,00298820s with Osc = 20 MHz
	movlw	D'10'
	movwf	CounterB
	movlw	D'102'
	movwf	CounterA
picloop
	decfsz	CounterA,1
	goto	picloop
	decfsz	CounterB,1
	goto	picloop
; end of delay loop

	goto		process 		; step every 2,9ms --> 1 rev per second
#endif

#ifdef reverse_polarity
	btfss		PORTA, 1		; check thant signal is 1 when returning from previous step processing
	goto 		boucle
boucle2
	btfss		PORTA, 1		; Check on the PC // port pin (RA1) (for "LowAvtive" InterpCNC board)
#else
	btfsc		PORTA, 1		; Check on the PC // port pin (RA1) (for "HighActive" boards)
#endif
	goto		process			; process step pulse (assumed that pulse processing is longer than pulse width, which is the case !)


;************************************************************
; detect the time out for current reduction
#ifdef TIMEOUT_ENABLED
#ifdef Mach3_idle_current_control
	btfsc		PORTC, 6		; Check on the Mach3 "Current Hi/Low" signal is high (cut in progress) pin (RC6)
	goto 		cutting			; if cutting enabled --> reload	the previous full power step
#endif
	movf		TMR0, w			; current reduction check
	btfss		TMR0, 6 
	goto		boucle
	btfss		TMR0, 7
	goto		boucle
	call		trait_timer
	movwf		IT_timer
	decfsz		IT_timer, w		; test if zero (if not skip)
	goto 		process_time_out	; time out detected
#endif
#ifdef reverse_polarity
	goto 		boucle2			; loop for next interrupt
#else
	goto 		boucle		; loop for next interrupt
#endif


#ifdef Mach3_idle_current_control
cutting
	movf 		in_idle_mode, f	; to test if zero #0 -->  in current reduction mode
	btfsc		STATUS, Z
	goto 		boucle			; already "high current" values loaded in the DACs

; reload regular step (full current values)				
	btfss		PORTA, 0		; Check on the direction pin (RA0)
	goto		sub_step_reload		; Jump over step addition

; fetch the value in the "DOWN" table and output it to DACs
	movf		step, w			; Reload step into w
	call 		STEP_TABLE		; Get the result from the table (DOWN side)	
	movwf		PORTB			; output result to port B (DAC A)
	movf		stepb, w		; Reload stepb into w
	call 		STEP_TABLE		; Get the result from the table (DOWN side)	
	movwf		PORTC			; output result to port C (DAC B)
	goto		Go_On			; goto main loop to wait for step signal

sub_step_reload					; go in the "UP" direction
; fetch the value in the "UP" table and output it to DACs
	movf		step, w			; Reload step into w
	call 		STEP_TABLE_UP 	; Get the result from the table (UP side)
	movwf		PORTB			; output result to port B DAC A
	movf		stepb, w		; Reload stepb into w
	call 		STEP_TABLE_UP	; Get the result from the table (UP side)	
	movwf		PORTC			; output result to port C DAC B	
	goto		Go_On			; goto main loop to wait for step signal
#endif

	
#ifdef TIMEOUT_ENABLED
process_time_out			
; a time out has been detected 
;--> process it :
		; first time 30s of inactivity :  25% current reduction (75% current)
		; second time 60s of inactivity : 60% current reduction (40% current)
	call 		reset_timer
	INCFSZ 		time_out_value	; if 0 then skip next --> low current mode
	goto 		DAC_half_current


; Process DAC A for timeout "low current"
	decf		time_out_value	; stay to FF for next interrupt (keep track of low level)
	decfsz		time_out_value2 
	goto 		boucle			; already in low current mode --> do nothing

DAC_low_current	
; Process DAC A for timeout "low current"
	incf		in_idle_mode	; #0 -->  in current reduction mode
	btfss		PORTA, 0		; Check on the direction pin (RA0)
	goto		sub_step_low		; Jump over step addition

; fetch the value in the "DOWN" table and output it to DACs
	movlw		B'00100000'		; address bank : Page 1 (200h-3FFh)
	movwf		STATUS
	movf		step, w			; Reload step into w
	call		STEP_TABLE_LOW	; Get the result from the table "current reduction mode" DOWN side
	movwf		PORTB			; output result to port B (DAC A)
	movf		stepb, w		; Reload stepb into w
	call		STEP_TABLE_LOW	; Get the result from the table "current reduction mode" DOWN side
	movwf		PORTC			; output result to port C (DAC B)
	clrf		STATUS			; restore bank 0	
	goto		boucle			; boucler

sub_step_low						; go in the "UP" direction
; fetch the value in the "UP" table and output it to DACs
	movlw		B'00100000'		; address bank : Page 1 (200h-3FFh)
	movwf		STATUS
	movf		step, w				; Reload step into w
	call 		STEP_TABLE_LOW_UP 	; Get the result from the table (UP side)
	movwf		PORTB				; output result to port B DAC A
	movf		stepb, w			; Reload stepb into w
	call 		STEP_TABLE_LOW_UP	; Get the result from the table (UP side)	
	movwf		PORTC				; output result to port C DAC B	
	clrf		STATUS				; restore bank 0
	goto		boucle				; boucler


								
DAC_half_current
; Process DAC A for timeout "half current"
	incf		in_idle_mode	; #0 -->  in current reduction mode
	btfss		PORTA, 0		; Check on the direction pin (RA0)
	goto		sub_step_half	; Jump over step addition

; fetch the value in the "DOWN" table and output it to DACs
	movlw		B'01000000'		; address bank : Page 2 (400h-5FFh)
	movwf		STATUS
	movf		step, w			; Reload step into w
	call		STEP_TABLE_HALF	; Get the result from the table "current reduction mode" DOWN side
	movwf		PORTB			; output result to port B (DAC A)
	movf		stepb, w		; Reload stepb into w
	call		STEP_TABLE_HALF	; Get the result from the table "current reduction mode" DOWN side
	movwf		PORTC			; output result to port C (DAC B)
	clrf		STATUS			; restore bank 0	
	goto		boucle			; boucler

sub_step_half					; go in the "UP" direction

; fetch the value in the "UP" table and output it to DACs
	movlw		B'01000000'			; address bank : Page 2 (400h-5FFh)
	movwf		STATUS
	movf		step, w				; Reload step into w
	call 		STEP_TABLE_HALF_UP 	; Get the result from the table (UP side)
	movwf		PORTB				; output result to port B DAC A
	movf		stepb, w			; Reload stepb into w
	call 		STEP_TABLE_HALF_UP	; Get the result from the table (UP side)	
	movwf		PORTC				; output result to port C DAC B	
	clrf		STATUS				; restore bank 0
	goto		boucle				; boucler
#endif

;***********************************************************************************************
; regular step processing	(time critical loop)			
process
#ifdef self_test_mode			; self generation of pulses without PC and step signal
else
	btfss		PORTA, 0		; Check on the direction pin (RA0)
#endif
	goto		sub_step		; Jump over step addition


add_step						; go in the "DOWN" direction
	movf		mode_value, w	; Load the current µstep value
	addwf		step, w			; Add the current mode to the current position value
; Bounds check the table
	andlw		0x3F			; mask above 7th bit (clip between 0 and 63)
	movwf		step
	addwf		quarter_table, w
	andlw		0x3F
	movwf		stepb			; shift the step value to access "DACB"
; fetch the value in the "DOWN" table and output it to DACs
load_full_power_currents
	movf		step, w			; Reload step into w
	call 		STEP_TABLE		; Get the result from the table (DOWN side)	
	movwf		PORTB			; output result to port B (DAC A)
	movf		stepb, w		; Reload stepb into w
	call 		STEP_TABLE		; Get the result from the table (DOWN side)	
	movwf		PORTC			; output result to port C (DAC B)
	goto		Go_On			; skip over "UP" processing

sub_step						; go in the "UP" direction
	comf		mode_value, w	; complement to 1 (still +1 missing to get the neg value)
	addwf		step, f			; Subtract (add neg value) the current mode from the current position value	
	incf		step, w			; finish the +1 mising for complement to 2...
; Bounds check the table
	andlw		0x3F			; mask above 7th bit (clip between 0 and 63)
	movwf		step
	addwf		quarter_table, w
	andlw		0x3F
	movwf		stepb			; shift the step value to access "DACB"
; fetch the value in the "UP" table and output it to DACs
	movf		step, w			; Reload step into w
	call 		STEP_TABLE_UP 	; Get the result from the table (UP side)
	movwf		PORTB			; output result to port B DAC A
	movf		stepb, w		; Reload stepb into w
	call 		STEP_TABLE_UP	; Get the result from the table (UP side)	
	movwf		PORTC			; output result to port C DAC B	


Go_On							; reset all timers for current reduction
#ifdef TIMEOUT_ENABLED
;	call 		reset_timer : 8 following instructions... to save time !
	movlw		TMR_loop1_max
	movwf		timer_loop1
	movlw		TMR_loop2_max
	movwf		timer_loop2 
	movlw		0x001
	movwf		IT_timer 		; clear timer "interrupt"
	clrf		TMR0

	movlw		TIME_to_Low		; set time out value to FD, 3 interrupts later (60s inactivity) it will be 0
	movwf		time_out_value
	clrf		time_out_value2
	incf		time_out_value2
#ifdef Mach3_idle_current_control
	clrf		in_idle_mode	; no longer in current reduction mode
#endif
#endif
	goto		boucle			; end of the time critical loop

;***************************************************************************************




; current reduction tables located in bank 2 = Page 2 (400h-5FFh)
#ifdef TIMEOUT_ENABLED
	ORG		0x400

STEP_TABLE_HALF
	;SINUS_TABLE_HALF --> 75% idle current reduction : sens haut --> bas
	;signeF1F2F3F4
	addwf	PCL, 1		;	DAC A
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'00101'	;	10
	retlw	B'00101'	;	10
	retlw	B'01001'	;	9
	retlw	B'01001'	;	9
	retlw	B'00001'	;	8
	retlw	B'01110'	;	7
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'00010'	;	4
	retlw	B'01100'	;	3
	retlw	B'00100'	;	2
	retlw	B'01000'	;	1
	retlw	B'00000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'10100'	;	-2
	retlw	B'11100'	;	-3
	retlw	B'10010'	;	-4
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'11110'	;	-7
	retlw	B'10001'	;	-8
	retlw	B'11001'	;	-9
	retlw	B'11001'	;	-9
	retlw	B'10101'	;	-10
	retlw	B'10101'	;	-10
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'10101'	;	-10
	retlw	B'10101'	;	-10
	retlw	B'11001'	;	-9
	retlw	B'11001'	;	-9
	retlw	B'10001'	;	-8
	retlw	B'11110'	;	-7
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'10010'	;	-4
	retlw	B'11100'	;	-3
	retlw	B'10100'	;	-2
	retlw	B'11000'	;	-1
	retlw	B'10000'	;	0
	retlw	B'01000'	;	1
	retlw	B'00100'	;	2
	retlw	B'01100'	;	3
	retlw	B'00010'	;	4
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'01110'	;	7
	retlw	B'00001'	;	8
	retlw	B'01001'	;	9
	retlw	B'01001'	;	9
	retlw	B'00101'	;	10
	retlw	B'00101'	;	10
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11


STEP_TABLE_HALF_UP
	;SINUS_TABLE_HALF --> 75% idle current reduction  : sens bas --> haut
	;signesigneF1F2F3F4
	addwf	PCL, 1		;	DAC A
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'00101'	;	10
	retlw	B'00101'	;	10
	retlw	B'01001'	;	9
	retlw	B'01001'	;	9
	retlw	B'00001'	;	8
	retlw	B'01110'	;	7
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'00010'	;	4
	retlw	B'01100'	;	3
	retlw	B'00100'	;	2
	retlw	B'01000'	;	1
	retlw	B'10000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'10100'	;	-2
	retlw	B'11100'	;	-3
	retlw	B'10010'	;	-4
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'11110'	;	-7
	retlw	B'10001'	;	-8
	retlw	B'11001'	;	-9
	retlw	B'11001'	;	-9
	retlw	B'10101'	;	-10
	retlw	B'10101'	;	-10
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'11101'	;	-11
	retlw	B'10101'	;	-10
	retlw	B'10101'	;	-10
	retlw	B'11001'	;	-9
	retlw	B'11001'	;	-9
	retlw	B'10001'	;	-8
	retlw	B'11110'	;	-7
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'10010'	;	-4
	retlw	B'11100'	;	-3
	retlw	B'10100'	;	-2
	retlw	B'11000'	;	-1
	retlw	B'00000'	;	0
	retlw	B'01000'	;	1
	retlw	B'00100'	;	2
	retlw	B'01100'	;	3
	retlw	B'00010'	;	4
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'01110'	;	7
	retlw	B'00001'	;	8
	retlw	B'01001'	;	9
	retlw	B'01001'	;	9
	retlw	B'00101'	;	10
	retlw	B'00101'	;	10
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11
	retlw	B'01101'	;	11


	ORG		0x200			; located in bank 1 = Page 2 (200h-3FFh)	
STEP_TABLE_LOW
	;SINUS_TABLE_LOW --> 40% idle current reduction : sens haut --> bas
	addwf	PCL, 1		;	DAC A
	;signeF1F2F3F4
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'00010'	;	4
	retlw	B'00010'	;	4
	retlw	B'01100'	;	3
	retlw	B'01100'	;	3
	retlw	B'00100'	;	2
	retlw	B'00100'	;	2
	retlw	B'01000'	;	1
	retlw	B'01000'	;	1
	retlw	B'00000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'11000'	;	-1
	retlw	B'10100'	;	-2
	retlw	B'10100'	;	-2
	retlw	B'11100'	;	-3
	retlw	B'11100'	;	-3
	retlw	B'10010'	;	-4
	retlw	B'10010'	;	-4
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'10010'	;	-4
	retlw	B'10010'	;	-4
	retlw	B'11100'	;	-3
	retlw	B'11100'	;	-3
	retlw	B'10100'	;	-2
	retlw	B'10100'	;	-2
	retlw	B'11000'	;	-1
	retlw	B'11000'	;	-1
	retlw	B'10000'	;	0
	retlw	B'01000'	;	1
	retlw	B'01000'	;	1
	retlw	B'00100'	;	2
	retlw	B'00100'	;	2
	retlw	B'01100'	;	3
	retlw	B'01100'	;	3
	retlw	B'00010'	;	4
	retlw	B'00010'	;	4
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6


STEP_TABLE_LOW_UP
	;SINUS_TABLE_LOW --> 40% idle current reduction : sens bas --> haut
	addwf	PCL, 1		;	DAC A
	;signesigneF1F2F3F4
		retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'00010'	;	4
	retlw	B'00010'	;	4
	retlw	B'01100'	;	3
	retlw	B'01100'	;	3
	retlw	B'00100'	;	2
	retlw	B'00100'	;	2
	retlw	B'01000'	;	1
	retlw	B'01000'	;	1
	retlw	B'10000'	;	0
	retlw	B'11000'	;	-1
	retlw	B'11000'	;	-1
	retlw	B'10100'	;	-2
	retlw	B'10100'	;	-2
	retlw	B'11100'	;	-3
	retlw	B'11100'	;	-3
	retlw	B'10010'	;	-4
	retlw	B'10010'	;	-4
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'10110'	;	-6
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'11010'	;	-5
	retlw	B'10010'	;	-4
	retlw	B'10010'	;	-4
	retlw	B'11100'	;	-3
	retlw	B'11100'	;	-3
	retlw	B'10100'	;	-2
	retlw	B'10100'	;	-2
	retlw	B'11000'	;	-1
	retlw	B'11000'	;	-1
	retlw	B'00000'	;	0
	retlw	B'01000'	;	1
	retlw	B'01000'	;	1
	retlw	B'00100'	;	2
	retlw	B'00100'	;	2
	retlw	B'01100'	;	3
	retlw	B'01100'	;	3
	retlw	B'00010'	;	4
	retlw	B'00010'	;	4
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'01010'	;	5
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6
	retlw	B'00110'	;	6

#endif


 end