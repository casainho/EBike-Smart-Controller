;*******************************************************************************
;PROGRAM            :   BLDC Motor controller
;MICROCONTROLLER    :   PIC18F4431
;*******************************************************************************
;*******************************************************************************
;AUTHOR     :   Laurent Brisedoux
;DATE       :   2011
;Version    :   V1.1
;license    :   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
;               To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter
;               to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
;*******************************************************************************
;Description:
;
;This code implements Sensorless BLDC Motor control using a PIC18F4431 Microcontroller.
;
;     PIN      18: MCLR             17: PGD
;              19: AN0  < V_IN      16: PGC
;              20: AN1  < I_IN      15: PWM4 > EN_C
;              21: CP1  < BEMF_A    14: PWM5 > PWM_C
;              22: CP2  < BEMF_B    11: PWM3 > PWM_B
;              23: CP3  < BEMF_C    10: PWM2 > EN_B
;              25: AN6  < V_DC      9: PWM1  > PWM_A
;              26: AN7  < V_SC      8: PWM0  > EN_A
;              30: OSC1             3: RD5   > EN_BP
;              31: OSC2             2: RD4
;              4: PWM6  > EN_SC     1: RX    < RS232 in
;              35: CCP2 > PWM_SC    44: TX   > RS232 out
;              36: CCP1 < Throttle  43: RC5
;
;	- Timer1 is used for measure throttle impulse time; CCP1 is used to detect start and stop of throttle impulse.
;	- Low voltage detection resets the system.
;
;*******************************************************************************
;Version history:
;
;   V1.1: Fixed throttle range in non car mode, 1.1ms = 0 and 1.9ms = 100%, new sync algo in ZC locked state,
;         Made brake mode optional, new sync algo for RC locked state, new filter for RC imput.
;
;*******************************************************************************
    include     <p18f4431.inc>
;*******************************************************************************

    CONFIG  OSC = HS, FCMEN = OFF, IESO = OFF, PWRTEN = ON, BOREN = ON, BORV = 27
    CONFIG  WDTEN = OFF, T1OSCMX = OFF
    CONFIG  HPOL = HIGH, LPOL = HIGH, PWMPIN = OFF
    CONFIG  MCLRE = ON, STVREN = ON, LVP = OFF, DEBUG = ON
    CONFIG  CP0 = OFF, CP1 = OFF, CPB = OFF, CPD = OFF
    CONFIG  WRT0 = OFF, WRT1 = OFF, WRTB = OFF, WRTC = OFF, WRTD = OFF
    CONFIG  EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF

;#define MPP_FIND_DEBUG_1
;#define RC_DEBUG_2
#define STEP_DEBUG_1
;#define ZC_LOCKED_DEBUG_1
;#define UPDATE_PWM_DEBUG_1
;#define UPDATE_TIMING_DEBUG_1
;#define ZC_EARLY_DEBUG_1
;#define RESET_DEBUG_1
#define USE_RC_THR                                      ; use the RC signal to control the throttle
#define USE_RC_CMD_FILTER
;#define USE_CAR_RC                                     ; use car RC signal range 50-100% (instead of 0-100%)
;#define USE_AUTO_THR                                   ; use fix throttle value THR_AUTO_MAX
;#define USE_BUTTON_THR
#define USE_BATT
;#define HIZ_SCC_CON
;#define MANUAL_ADV
;#define MANUAL_DEMAG
#define USE_AUTO_SYNC_FILTER
;#define USE_INT0
;#define USE_UART
#define USE_BMC_PID
#define USE_BRAKE
#define USE_WINDMILL
;#define USE_DFILTER
;#define USE_DDP
#define USE_MPP_MAN                                     ; use V_MPP_MAN, act as a Soft LVC when MPPT is not used
;#define ENABLE_MPPT
;#define USE_SCC

;GEN_STATUS bits
#define TMR0_OV_FLAG    0
#define RC_LOCKED_FLAG  1
#define MPPT_RUN        2
#define MPPT_SKIP       3
#define RESET_FLAG      7

;ZC_STATUS bits
#define NEW_FLAG        0
#define EARLY_FLAG      1
#define LOCKED_FLAG     2
#define SYNCED_FLAG     3
#define P_MIN_FLAG      4
#define INIT_FLAG       6
;#define RESET_FLAG     7

;BMC_STATUS bits
#define RUN_FLAG        0
#define ACTIVE_FLAG     1
#define STEPPING_ON     2
#define AUTO_STEP       3
#define AUTO_ADV        4
#define ADV_MAX_FLAG    5
#define MAN_PWM         6
;#define RESET_FLAG     7

;SCC_STATUS bits
;#define RUN_FLAG       0
;#define ACTIVE_FLAG    1
#define CHRG_TOP        2
;#define INIT_FLAG      6
;#define RESET_FLAG     7


;PORTD bits
#define EN_BP           5

;TMR0 is used for motor timing, resolution 400ns (2.5MHz)
;Timing
#define PERIOD_MAX      0x3FFF  ;16383*0.4us=>6.5532ms, 60/(6.5532ms*6*7)=>218RPM
#define PERIOD_MIN      0x00FF  ;14005 RPM (0x3FF:0.4092ms -> 3491 RPM, 0x1FF:0.2048ms -> 6975 RPM)
#define PERIOD_TEST     0x3FFF  ;0xFFF:0.8164ms
#define K_M_RPM         d'1100' ;d'575' ;d'332' ;d'900' RPM/v
#define ADV_CMD_INIT    (d'30'*d'256'/d'60')    ;60degrees = 256
#define DEMAG_CMD_MIN   (d'15'*d'256'/d'60')    ;15 degrees
#define K_DEMAG         (d'5'*d'256'/d'60') ;DEMAG angle/PWM
#define ZC_BAD_MAX      d'32'   ;max nb of bad zc to detect sync/lock lost
#define ZC_VALID_MIN    d'16'   ;min nb of valid zc to confime sync/lock
#define INIT_COUNT_MAX  d'12'
#define SCC_SKIP_MAX    d'200'

;TMR1 is used for RC signal timing, resolution 1.6us (625kHz)
;RC signal
#define RC_MAX          0x061A  ;2.5ms max RC pulse length
#define RC_MIN          0x0138  ;0.5ms min RC pulse length
#ifdef  USE_CAR_RC
#define RC_CENTER       0x03BA  ;0x03AA ;1.5ms
#else
;#define RC_CENTER      0x04A3  ;1.90ms
#define RC_CENTER       0x02AF  ;1.10ms
#endif
#define RC_BAD_MAX      d'3'    ;max nb of bad RC pulse to detect sync/lock lost
#define RC_VALID_MIN    d'16'   ;min nb of valid RC pulse to confime sync/lock

;PWM
#define PTPERH_INIT     0x00    ;PTPERL and PTPERH are set up for a 20KHz PWM frequency
#define PTPERL_INIT     0xFF    ;PTPERH:PTPERL = ((Fosc/4)/(PWMfreq x PTMRps))-1
#define PWM_DUTY_MIN    0x8
#define PWM_DUTY_MAX    0x3E0
#define PWM_SCALE       4*(PTPERL_INIT+1)/0xFF
#define SCC_DUTY_MIN    0x4
#define SCC_DUTY_MAX    0xFE

;Throttle
#define THR_CMD_MIN     0x10        ;below that coast (0 -> brake)
#define THR_MIN_NOLOCK  0x10        ;Min throttle command to start the motor
#define THR_MAX_NOLOCK  0x40        ;Max throttle limit until the motor sync is locked
#define THR_AUTO_MAX    0xC0        ;fix throttle value

;V in
#define V_DC_MAX        (d'1600'*d'1024'/(d'327'*d'5'))
#define V_DC_HIGH       (d'1200'*d'1024'/(d'327'*d'5'))
#define V_DC_LOW        (d'600'*d'1024'/(d'327'*d'5'))
#define V_IN_LOW        (d'600'*d'1024'/(d'327'*d'5'))
#define V_DC_MIN        (d'450'*d'1024'/(d'327'*d'5'))
#define V_SC_MAX        (d'420'*d'1024'/(d'327'*d'5'))
#define V_SC_HIGH       (d'370'*d'1024'/(d'327'*d'5'))
#ifdef HIZ_SCC_CON
#define V_SC_LOW        (d'300'*d'1024'/(d'327'*d'5'))
#define V_SC_MIN        (d'270'*d'1024'/(d'327'*d'5'))
#else
#define V_SC_LOW        (d'330'*d'1024'/(d'327'*d'5'))
#define V_SC_MIN        (d'300'*d'1024'/(d'327'*d'5'))
#endif

;V MMP
#define V_MPP_MAN       (d'640'*d'1024'/(d'327'*d'5'))  ;Soft LVC when MPPT is not used
#define V_MPP_MAX       (d'1050'*d'1024'/(d'327'*d'5'))
#define V_MPP_MIN       (d'550'*d'1024'/(d'327'*d'5'))
#define V_MPP_STEP      (d'007'*d'1024'/(d'327'*d'5'))  ;min 002 (16mv!!!)
#define MPPT_PERIOD_MAX 0x20                            ;MPPT update period/frequency

;I MPP
#define I_MPP_MAX       (d'200'*d'1024'*/(d'91'*d'5'))
#define I_IN_MAX        (d'450'*d'1024'/(d'91'*d'5'))

#define DUTY_REQ_PERIOD_MAX (d'1')
#define K_THR_CMD_ACC   (d'1')      ;acceleration factor
#define K_THR_CMD_DEC   (d'128')    ;deceleration factor
#define K_ADC_1         (d'5'*d'327'/d'100')    ;5V ref, ~1/3 R divider
#define K_PIDM_P        (d'256'/d'4')
#define K_PIDM_D        (d'512'/d'4')
#define K_PIDC_P        (d'512'/d'4')
#define K_PIDC_D        (d'512'/d'4')
#define K_SCC_ERR       (d'013'*d'1024'/(d'327'*d'5'))  ;130mV C_ERR compensation
#define K_PIDB_P        (d'256'/d'4')
#define K_PIDB_D        (d'512'/d'4')

debug1on    macro
            bsf PORTC,6
            endm

debug1off   macro
            bcf PORTC,6
            endm

debug2on    macro
            bsf PORTC,7
            endm

debug2off   macro
            bcf PORTC,7
            endm

;*******************************************************************************
;RAM locations in Access bank, uninitialized
;*******************************************************************************
    UDATA_ACS
STATUS_SAVE         res 1       ;saved STATUS during Low ISR
WREG_SAVE           res 1       ;saved WREG during Low ISR
PRODH_SAVE          res 1       ;saved PRODH during Low ISR
PRODL_SAVE          res 1       ;saved PRODL during Low ISR
GEN_STATUS          res 1       ;generic status register
ZC_STATUS           res 1       ;Zero Crossing status register
BMC_STATUS          res 1       ;brushless motor controller status register
SCC_STATUS          res 1       ;Storage capacity controller status register
PWM_DUTY_H          res 1
PWM_DUTY_L          res 1
PWM_DUTY_S          res 1
PWM_DUTY            res 1
DUTY_REQ            res 1
DUTY_REQ_PERIOD     res 1
SCC_DUTY_H          res 1
SCC_DUTY_L          res 1
SCC_DUTY_S          res 1
SCC_DUTY            res 1
B_DUTY_H            res 1
B_DUTY_L            res 1
B_DUTY_S            res 1
B_DUTY              res 1
B_ERR_H             res 1
B_ERR_L             res 1
B_DIF_H             res 1
B_DIF_L             res 1
STEP_NEXT           res 1
STEP_CURRENT        res 1
ZC_VALID_CNT        res 1
ZC_BAD_CNT          res 1
ZC_OLD_H            res 1
ZC_OLD_L            res 1
ZC_NEW_H            res 1
ZC_NEW_L            res 1
ADV_TIME_H          res 1
ADV_TIME_L          res 1
PERIOD_H            res 1
PERIOD_L            res 1
INIT_COUNT          res 1
OVDCOND_TEMP        res 1
OVDCONS_TEMP        res 1
CCPR1_TEMP_H        res 1
CCPR1_TEMP_L        res 1
RC_VALID_CNT        res 1
RC_BAD_CNT          res 1
DIV_H               res 1
DIV_L               res 1
VAR_TEMP_H          res 1
VAR_TEMP_L          res 1
VAR_TEMP            res 1
WINDMILL_RETRY      res 1
VAR_LISR_H          res 1
VAR_LISR_L          res 1
VAR_LISR            res 1
THR_CMD             res 1
THR_NOLOCK          res 1
MPPT_PERIOD         res 1
V_IN_H              res 1
V_IN_L              res 1
I_IN_H              res 1
I_IN_L              res 1
V_DC_H              res 1
V_DC_L              res 1
V_SC_H              res 1
V_SC_L              res 1
V_MPP_H             res 1
V_MPP_L             res 1
P_MPP_H             res 1
P_MPP_L             res 1
C_ERR_H             res 1
C_ERR_L             res 1
C_DIF_H             res 1
C_DIF_L             res 1
P_IN_H              res 1
P_IN_L              res 1
P_OLD_H             res 1
P_OLD_L             res 1
DP_IN_H             res 1
DP_IN_L             res 1
DP_OLD_H            res 1
DP_OLD_L            res 1
DV_IN_H             res 1
DV_IN_L             res 1
V_OLD_H             res 1
V_OLD_L             res 1
V_PREV_H            res 1
V_PREV_L            res 1
I_PREV_H            res 1
I_PREV_L            res 1
SCC_SKIP_CNT        res 1
DEBUG_COUNT         res 1
#ifdef MANUAL_ADV
ADV_CMD             res 1
#endif
#ifdef MANUAL_DEMAG
DEMAG_CMD           res 1
#endif



;*******************************************************************************
;*******************************************************************************
;                       RESET AND INTERRUPT VECTORS
;*******************************************************************************
;*******************************************************************************
STARTUP     code    0x00
    goto    START               ;Reset Vector address

PROG_HIGH   code    0x08
    goto    ISR_HIGH            ;High priority ISR at 0x0008

PROG_LOW    code    0x018
    goto    ISR_LOW             ;Low priority ISR at 0x0018


;*******************************************************************************
;*******************************************************************************
;                           INITIALIZATION
;*******************************************************************************
;*******************************************************************************
PROG1   code
START
    call    INIT_PORTS
    call    INIT_VAR
    call    INIT_HSADC
    call    INIT_PCPWM
    call    INIT_IC
    call    INIT_TIMING
#ifdef USE_UART
    call    INIT_UART
#endif
    call    INIT_INTERRUPTS



;*******************************************************************************
;*******************************************************************************
;                               MAIN LOOP
;*******************************************************************************
;*******************************************************************************
MAIN_START
    call    RESET_INTERRUPTS
    call    RESET_PCPWM
    call    RESET_IC
    call    RESET_TIMING
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    bsf     SCC_STATUS,RUN_FLAG
MAIN_IDLE
    btfsc   BMC_STATUS,RUN_FLAG     ;run?
    bra     STEPPING_INIT           ;yes, go on
    call 	LOAD_TIMEOUT 			;reload timer 0 with timeout
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     MAIN_IDLE

STEPPING_INIT
#ifdef USE_WINDMILL
    movlw   0x02
    movwf   WINDMILL_RETRY
    call    WINDMILL_FIND
    movf    STEP_NEXT,W
    bnz     STEPPING_ADV_LOAD       ;windmill found?
#endif
    movlw   1
    movwf   STEP_NEXT
    call    LOAD_STEP
    call    UPDATE_TIMING
    bsf     BMC_STATUS,AUTO_ADV     ;enable auto advance
STEPPING_SWITCH
    btfss   BMC_STATUS,STEPPING_ON  ;stepping on?
    bra     STEPPING_LOOP
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    movlw   b'01000000'
    andwf   OVDCONS,W               ;preserve the state of PWM6
    iorwf   OVDCONS_TEMP,W
    movff   OVDCOND_TEMP,OVDCOND    ;switch to the next step
    movwf   OVDCONS
    bsf     INTCON,GIEL             ;Enable low priority interrupts
STEPPING_LOOP
#ifdef STEP_DEBUG_1
    movlw   6
    cpfseq  STEP_NEXT               ;step 1?
    bra     STEPPING_DEMAG
    debug1on
STEPPING_DEMAG
#endif
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    call    LOAD_DEMAG              ;load timer 0 with demagnetization delay
;   btfss   BMC_STATUS,RUN_FLAG     ;still want to run?
;   bra     MAIN_START
#ifdef UPDATE_TIMING_DEBUG_1
    debug1on
#endif
    call    UPDATE_TIMING           ;Update period and advance timing
#ifdef UPDATE_TIMING_DEBUG_1
    debug1off
#endif
    btfsc   ZC_STATUS,RESET_FLAG
    bra     MAIN_START
    movff   STEP_NEXT,STEP_CURRENT
    movlw   6
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT
    incf    STEP_NEXT               ;next step
    call    LOAD_STEP
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
#ifdef STEP_DEBUG_1
    debug1off
#endif
    call    STEP_FIND
    cpfseq  STEP_CURRENT            ;early zc?
    bra     STEPPING_ZC_WAIT        ;no, continue
STEPPING_ZC_EARLY
    movff   TMR5L, ZC_NEW_L         ;early zc, copy TMR5H:TMR5L
    movff   TMR5H, ZC_NEW_H
    bsf     ZC_STATUS,EARLY_FLAG    ;set zc early flag
    bra     STEPPING_ADV_LOAD
STEPPING_ZC_WAIT
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    LOAD_IC
#ifdef ZC_DEBUG_1
    debug1on
#endif
    call    WAIT_FOR_ZC             ;wait for tmr0 ov or zc
#ifdef ZC_DEBUG_1
    debug1off
#endif
    btfss   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     STEPPING_ZC_LATE        ;no, handle it
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     STEPPING_LOOP
STEPPING_ZC_LATE
    movff   TMR5L, ZC_NEW_L         ;no, timout, copy TMR5H:TMR5L
    movff   TMR5H, ZC_NEW_H
STEPPING_ADV_LOAD
    btfsc   BMC_STATUS,ADV_MAX_FLAG ;Max advance?
    bra     STEPPING_SWITCH
    bsf     BMC_STATUS,AUTO_STEP    ;enable auto stepping
    call    LOAD_ADVANCE            ;load timer 0 with advance delay
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     STEPPING_SWITCH


;*******************************************************************************
;*******************************************************************************
;                           INTERRUPT SERVICE ROUTINES
;*******************************************************************************
;*******************************************************************************

;*******************************************************************************
;High priority interrupt service routine
;*******************************************************************************
ISR_HIGH
    btfsc   INTCON,TMR0IF           ;Timer0 overflow Interrupt?
    bra     TIMER0_OVERFLOW
    btfsc   PIR3,IC1IF              ;IC1 Interrupt?
    bra     IC1_BEMFA_ZC
    btfsc   PIR3,IC2QEIF            ;IC2 Interrupt?
    bra     IC2_BEMFB_ZC
    btfsc   PIR3,IC3DRIF            ;IC3 Interrupt?
    bra     IC3_BEMFC_ZC
#ifdef  USE_INT0
    btfsc   INTCON,INT0IF           ;INT0 Interrupt?
    bra     INT0_FALLING
#endif
    btfsc   PIR2,LVDIF              ;Low voltage detected?
    bra     LVD_INT
    btfsc   PIR1,CCP1IF             ;CCP1 Interrupt?
    bra     CCP1_RISING
    goto    ASSERT

LVD_INT
#ifdef  RESET_DEBUG_1
    debug1off
    nop
    nop
    debug1on
#endif
    reset
    bcf     PIE2,LVDIE              ;LVD Interrupt disable
    bcf     PIR2,LVDIF              ;clear LVD interrupt flag
    retfie  FAST

TIMER0_OVERFLOW
    bsf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    bcf     INTCON,TMR0IE           ;disable timer 0 int
    btfss   BMC_STATUS,AUTO_STEP
    retfie  FAST
    btfss   BMC_STATUS,STEPPING_ON  ;stepping on?
    retfie  FAST
    movlw   b'01000000'
    andwf   OVDCONS,W               ;preserve the state of PWM6
    iorwf   OVDCONS_TEMP,W
    movff   OVDCOND_TEMP,OVDCOND    ;switch to the next step
    movwf   OVDCONS
    retfie  FAST

IC1_BEMFA_ZC
    movff   CAP1BUFL,ZC_NEW_L       ;copy captured TMR5 value for BEMF zero crossing timing
    movff   CAP1BUFH,ZC_NEW_H
    bra     IC_INT_RETURN
IC2_BEMFB_ZC
    movff   CAP2BUFL,ZC_NEW_L       ;copy captured TMR5 value for BEMF zero crossing timing
    movff   CAP2BUFH,ZC_NEW_H
    bra     IC_INT_RETURN
IC3_BEMFC_ZC
    movff   CAP3BUFL,ZC_NEW_L       ;copy captured TMR5 value for BEMF zero crossing timing
    movff   CAP3BUFH,ZC_NEW_H
IC_INT_RETURN
    bsf     ZC_STATUS,NEW_FLAG      ;set flag to indicate BEMF zero crossing
    clrf    CAP1CON                 ;clear IC1 Capture
    clrf    CAP2CON                 ;clear IC2 Capture
    clrf    CAP3CON                 ;clear IC3 Capture
    clrf    PIE3                    ;disable all IC interrupt
    clrf    PIR3                    ;clear all IC Int flag
    btfss   BMC_STATUS,AUTO_ADV
    retfie  FAST
LL_LOAD_ADVANCE
    bsf     BMC_STATUS,AUTO_STEP    ;enable auto stepping
    btfsc   BMC_STATUS,ADV_MAX_FLAG ;Max advance?
    bra     TIMER0_OVERFLOW
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    movff   ADV_TIME_H,TMR0H        ;Load the Higher byte to TMR0H
    movff   ADV_TIME_L,TMR0L        ;Load the Lower byte to TMR0L
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    retfie  FAST

#ifdef  USE_INT0
INT0_FALLING
    bcf     INTCON,INT0IE
    bcf     INTCON,INT0IF           ;clear INT0 Interrupt flag
    retfie  FAST
#endif

CCP1_RISING
    btfss   CCP1CON,0               ;should be rising edge?
    bra     CCP1_FALLING
    clrf    TMR1H                   ;reset TMR1
    clrf    TMR1L
    bcf     CCP1CON,0               ;Capture on CCP1 falling edge
    bcf     PIR1,CCP1IF             ;clear CCP1 Interrupt flag
CCP1_FALLING
    bcf     IPR1,CCP1IP             ;CCP1 Interrupt low priority
    retfie  FAST

;*******************************************************************************
;Low priority interrupt service routine
;*******************************************************************************
ISR_LOW
    movwf   WREG_SAVE               ;save off current W register contents
    movff   STATUS,STATUS_SAVE
    movff   PRODH,PRODH_SAVE
    movff   PRODL,PRODL_SAVE
    btfsc   PIR1,ADIF               ;A/D Converter Interrupt?
    bra     ADC_INT
    btfsc   PIR1,TMR1IF             ;TMR1 overflow Interrupt?
    bra     TMR1_INT
    btfsc   PIR1,CCP1IF             ;CCP1 Interrupt?
    bra     CCP1_INT
    goto    ASSERT                  ;Shouldn't be here !!!! -> fatal error
TMR1_INT
    bcf     PIR1,TMR1IF
#ifdef  USE_RC_THR
    clrf    CCPR1_TEMP_H            ;clear the servo pluse time
    clrf    CCPR1_TEMP_L
    call    UPDATE_RC_CMD
#endif
#ifdef  USE_BUTTON_THR
    clrf    THR_CMD
    movlw   0x80
    btfss   PORTC,7
    movwf   THR_CMD
#endif
#ifdef USE_INT0
    btfsc   INTCON,INT0IE
    bra     ISR_LOW_RETURN
#ifdef UART_LOGGING
    movf    PERIOD_H,W
    sendw
    movf    PERIOD_L,W
    sendw
    movf    V_DC_H,W
    sendw
    movf    V_DC_L,W
    sendw
#ifdef MANUAL_ADV
    movf    ADV_CMD,W
    sendw
#endif
#endif
    bcf     INTCON,INT0IF           ;clear INT0 Interrupt flag
    bsf     INTCON,INT0IE
#endif
    bra     ISR_LOW_RETURN

CCP1_INT
    btfsc   CCP1CON,0               ;falling edge?
    bra     CCP1_INT_RIS
    movff   CCPR1L,CCPR1_TEMP_L     ;copy captured TMR1 value for Throttle cmd
    movff   CCPR1H,CCPR1_TEMP_H
    call    UPDATE_RC_CMD
    bsf     CCP1CON,0               ;Capture on CCP1 rising edge
    bcf     PIR1,CCP1IF             ;clear CCP1 Interrupt flag
    bsf     IPR1,CCP1IP             ;CCP1 Interrupt high priority
    bra     ISR_LOW_RETURN
CCP1_INT_RIS
    clrf    TMR1H                   ;reset TMR1
    clrf    TMR1L
    bcf     CCP1CON,0               ;Capture on CCP1 falling edge
    bcf     PIR1,CCP1IF             ;clear CCP1 Interrupt flag
    bcf     IPR1,CCP1IP             ;CCP1 Interrupt low priority
    bra     ISR_LOW_RETURN
ADC_INT
    movff   ADRESH,V_IN_H           ;ADC value is group A
    movff   ADRESL,V_IN_L

    movff   ADRESH,I_IN_H           ;ADC value is group B
    movff   ADRESL,I_IN_L

    movff   ADRESH,V_DC_H           ;ADC value is group C
    movff   ADRESL,V_DC_L

    movff   ADRESH,V_SC_H           ;ADC value is group D
    movff   ADRESL,V_SC_L

#ifdef UPDATE_PWM_DEBUG_1
    debug1on
#endif
    call    UPDATE_MPPT
    btfss   BMC_STATUS,MAN_PWM
    call    UPDATE_BMC_PWM
#ifdef UPDATE_PWM_DEBUG_1
    debug1off
#endif
    bcf     PIR1,ADIF               ;clear the ADC interrupt flag
ISR_LOW_RETURN
    movff   PRODH_SAVE,PRODH
    movff   PRODL_SAVE,PRODL
    movf    WREG_SAVE,W             ;Restore WREG
    movff   STATUS_SAVE,STATUS      ;Restore STATUS
    retfie


;*******************************************************************************
;*******************************************************************************
;                       INITIALIZATION SUBROUTINES
;*******************************************************************************
;*******************************************************************************

;*******************************************************************************
;Initialize variables with default values
;*******************************************************************************
INIT_VAR
    lfsr    FSR0, 0x00
INIT_RAM
    clrf    POSTINC0
    btfss   FSR0L,7
    bra     INIT_RAM

    movlw   HIGH V_DC_MAX
    movwf   V_DC_H
    movlw   LOW V_DC_MAX
    movwf   V_DC_L

    movlw   RC_BAD_MAX          ;reset the bad RCcounter
    movwf   RC_BAD_CNT
    movlw   RC_VALID_MIN        ;reset the valid RC counter
    movwf   RC_VALID_CNT

    movlw   MPPT_PERIOD_MAX
    movwf   MPPT_PERIOD

    movlw   SCC_SKIP_MAX
    movwf   SCC_SKIP_CNT

#ifdef  USE_AUTO_THR
    movlw   THR_AUTO_MAX
    movwf   THR_CMD
;   movff   THR_NOLOCK,THR_CMD
#endif

#ifdef USE_MPP_MAN
    movlw   HIGH V_MPP_MAN
    movwf   V_MPP_H
    movlw   LOW V_MPP_MAN
    movwf   V_MPP_L
    movlw   THR_MAX_NOLOCK
    movwf   THR_NOLOCK
#endif


    return


;*******************************************************************************
;Initialize High-Speed ADC
;*******************************************************************************
INIT_HSADC
    movlw   b'00010000'         ; ADCON1 is configured such that:
    movwf   ADCON1              ; a) Vref+ and Vref- are Avdd and Avss, respectively.
                                ; b) The FIFO buffer is enable

    movlw   b'10001101'         ; ADCON2 is configured such that:
    movwf   ADCON2              ; a) The A/D result is right justified (read ADRESH before reading ADRESL)
                                ; b) The A/D acquisition time is set to 2 Tad.
                                ; c) The A/D conversion clock is set to Fosc/16.

    movlw   b'10010000'         ; interrupt every 4th write, PWM event start AD sequence
;   movlw   b'00000000'         ; No interrupt, No event trigger
    movwf   ADCON3

    movlw   b'01000100'         ; ADCHS is configured such that:
    movwf   ADCHS               ; a) Group A signal is AN0,
                                ; b) Group B signal is AN1,
                                ; c) Group C signal is AN6,
                                ; d) Group D signal is AN7,

    movlw   b'11000011'         ; ANSEL0 is configured such that:
    movwf   ANSEL0              ; a) AN0, AN1, AN6, AN7 are analog input pins.

    movlw   b'00011101'         ; ADCON0 is configured such that:
    movwf   ADCON0              ; a) Single shot mode is enabled
                                ; b) Multi-Channel mode is enabled
                                ; c) Group A,B,C and D are sampled
                                ; d) The ADC is turned on.
    movlw   b'0011101'
    movwf   LVDCON              ; Low-Voltage Detection enable, 3.93V - 4.62V Limits


    bsf     ADCON0, GO          ;then start a new conversion to init varialble
INIT_HSADC_LOOP
    btfsc   ADCON0, GO
    bra     INIT_HSADC_LOOP

    movff   ADRESH,V_PREV_H     ;ADC value is group A
    movff   ADRESL,V_PREV_L
    movff   ADRESH,I_PREV_H     ;ADC value is group B
    movff   ADRESL,I_PREV_L
    movff   ADRESH,V_DC_H       ;ADC value is group C
    movff   ADRESL,V_DC_L
    movff   ADRESH,V_SC_H       ;ADC value is group D
    movff   ADRESL,V_SC_L

    movff   V_PREV_H,V_IN_H
    movff   V_PREV_L,V_IN_L
    movff   V_PREV_H,V_OLD_H
    movff   V_PREV_L,V_OLD_L
    movff   I_PREV_H,I_IN_H
    movff   I_PREV_L,I_IN_L

    bcf     PIR1,ADIF           ;clear the ADC interrupt flag
    return


;*******************************************************************************
;Initialize PCPWM
;   NOTES:
;   1)  PTPER has 12-bit resolution, 4 LSBs of PTPERH and 8 bits of PTPERL
;   2)  In edge aligned mode, PTMR reset to zero on match with PTPER
;   3)  PDC has 14-bit resolution, 6 LSBs of PDCxH and 8 bits of PDCxL
;   4)  Lower 2 bits of PDC compared to Q clocks
;
;   5)  Resolution(of duty cycle)= log((Fosc/4)/Fpwm)/log(2) = log(5Mhz/20kHz)/log(2) = 8 bits
;       so 6 LSBs of PDCxH and 2 MSBs of PDCxL will be used.
;      (for 16kHz, resolution = log(5Mhz/16kHz)/log(2) = 8 bits, also.)
;
;*******************************************************************************
INIT_PCPWM
    movlw   b'00000000'         ; PTCON0 is configured such that:
    movwf   PTCON0              ; a) PWM postscale value is 1:16
                                ; b) PWM time base input is Fosc/4
                                ; c) PWM time base mode is free-running for edge-aligned operation

    movlw   PTPERL_INIT         ; PTPERL and PTPERH are set up.
    movwf   PTPERL
    movlw   PTPERH_INIT
    movwf   PTPERH

    movlw   b'01011111'         ; PWMCON0 is configured such that:
    movwf   PWMCON0             ; a) All PWM I/O pins enabled for PWM output.
                                ; b) All PWM I/O pairs are set to independant mode

    movlw   b'11110000'         ;PWMCON1 is configured such that:
    movwf   PWMCON1             ; a) Special event trigger post-scaler is set to 1:16
                                ; b) Special event trigger occurs when time-base is counting upwards
                                ; c) Updates from duty cycle and period buffer registers are enabled.
                                ; d) Output overrides via OVDCON are not synchronous to the PWM timebase.

    clrf    DTCON               ;DTCON for zero dead-time (relying on IRAMS cross-conduction prevention logic)

    movlw   PTPERL_INIT         ;SEVTCMP is set to PTPER_INIT
    movwf   SEVTCMPL
    movlw   PTPERH_INIT
    movwf   SEVTCMPH

    movlw   DUTY_REQ_PERIOD_MAX     ;reset DUTY_REQ_PERIOD
    movwf   DUTY_REQ_PERIOD

;   clrf    SEVTCMPH
;   setf    SEVTCMPL

    bcf     OVDCONS,6           ;disable SCC PWM
    bsf     PORTC,1             ;SCC PWM, default to high

    movlw   0x3F
    movwf   PR2                 ;PWM frequency ~ 80kHz

;   movlw   0x60
;   movwf   CCPR2L
    clrf    CCPR2L

RESET_PCPWM
    movlw   b'01000000'
    clrf    OVDCOND             ;all override
    andwf   OVDCONS             ;All motor PWM HI-Z, preserve the state of PWM6

    clrf    PDC0L               ;PDC0L, PDC1L, PDC2L, PDC3L, PDC0H, PDC1H, PDC2H, PDC3H
    clrf    PDC1L               ;   are clear so that duty cycles are initially 0.
    clrf    PDC2L
;   clrf    PDC3L
    clrf    PDC0H
    clrf    PDC1H
    clrf    PDC2H
;   clrf    PDC3H

    bsf     PTCON1, PTEN        ;PTEN bit in the PTCON1 is set to enable the PWM time base.

#ifdef  __MPLAB_DEBUGGER_ICD2
    movlw   b'10001000'         ;FLT
    movwf   FLTCONFIG           ; a) reset and enable fault condition on break-point for use with ICD2
#endif

    return

;*******************************************************************************
;Initialize IC
;   Uses Input Capture 1,2 & 3 for BEMF detection.
;*******************************************************************************
INIT_IC
    clrf    QEICON              ; Clear QEICON to make sure that QEI mode is disabled.

    movlw   b'00000000'         ; dissable CAPxCON :
    movwf   CAP1CON             ; a)no Timer5 resets on capture event
                                ; b)no Capture on every CAP1 input state change
    movwf   CAP2CON             ; a)no Timer5 resets on capture event
                                ; b)no Capture on every CAP2 input state change
    movwf   CAP3CON             ; a)no Timer5 resets on capture event
                                ; b)no Capture on every CAP3 input state change

    movlw   b'01001001'         ; T5CON is configured such that:
    movwf   T5CON               ; a) Special event reset is disabled
                                ; b) Continuous count mode is enabled
                                ; c) Timer5 input clock prescaler is 1:2
                                ; e) Timer5 is enabled

    clrf    DFLTCON             ; Digital filter disable
;   movlw   b'00111010'         ; Digital filter is configured
;   movwf   DFLTCON             ; a) IC1,2&3 filter enabled
                                ; b) noise filter clock divider = 1:4

    movlw   b'00000101'
    movwf   CCP1CON             ;Capture on CCP1 rising edge

RESET_IC
    clrf    CAP2CON             ;clear IC2 Capture
    clrf    CAP3CON             ;clear IC3 Capture
    clrf    CAP1CON             ;clear IC1 Capture

    return

;*******************************************************************************
;Initialize PORTS
;*******************************************************************************
INIT_PORTS
    movlw   b'11111111'         ; Corresponding bits of TRISA are also set to inputs.
    movwf   TRISA

    movlw   b'11111111'         ; Corresponding bits of TRISA are also set to inputs.
    movwf   TRISB

    clrf    PORTC
    movlw   b'00111101'         ; RC1, RC6, RC7
    movwf   TRISC

    clrf    PORTD
    movlw   b'11011111'         ; RD5
    movwf   TRISD

    return


;*******************************************************************************
;Initialize UART
;*******************************************************************************
INIT_UART
    movlw   0x0A        ;Baudrate = 115200
    movwf   SPBRG

    bsf     RCSTA,SPEN  ;Enable Serial Port

    movlw   0x24        ;8-bit transmission, Enable Transmission;
    movwf   TXSTA       ;Asynchronous mode with High speed transmission

;   bcf TRISC,6
;   bsf TRISC,7

    return


;*******************************************************************************
;Initialize Timings
;*******************************************************************************
INIT_TIMING
    movlw   b'10000000'         ;T0CON is configured such that:
    movwf   T0CON               ;a) TMR0 ON,
                                ;b) 16-bit operation,
                                ;c) clock source is instruction cycle clock,
                                ;d) prescalar is 1:2

    movlw   b'10110001'         ;T1CON is configured such that:
    movwf   T1CON               ;a) 16-bit operation
                                ;b) prescalar is 1:8
                                ;c) clock source is instruction cycle clock
                                ;d) TMR1 ON

    movlw   b'00000100'         ;T1CON is configured such that:
    movwf   T2CON               ;a) no prescalar
                                ;b) TMR2 ON

    movlw   b'01001001'         ; T5CON is configured such that:
    movwf   T5CON               ; a) Special event reset is disabled
                                ; b) Continuous count mode is enabled
                                ; c) Timer5 input clock prescaler is 1:2
                                ; e) Timer5 is enabled

RESET_TIMING
    movlw   HIGH PERIOD_MAX
    movwf   PERIOD_H
    movlw   LOW PERIOD_MAX
    movwf   PERIOD_L

#ifdef MANUAL_DEMAG
    movlw   DEMAG_CMD_MIN
    movwf   DEMAG_CMD
#endif

#ifdef MANUAL_ADV
    movlw   ADV_CMD_INIT
    movwf   ADV_CMD
#endif

    movlw   ZC_BAD_MAX          ;reset the counter
    movwf   ZC_BAD_CNT

    movlw   ZC_VALID_MIN        ;reset the counter
    movwf   ZC_VALID_CNT

    clrf    ZC_STATUS

    clrf    STEP_NEXT

    movlw   HIGH PERIOD_MAX
    movwf   TMR5H
    movwf   ZC_NEW_H
    movlw   LOW PERIOD_MAX
    movwf   TMR5L
    movwf   ZC_NEW_L
    clrf    ZC_OLD_H
    clrf    ZC_OLD_L

    return


;*******************************************************************************
;Initialize interrupts
;*******************************************************************************
INIT_INTERRUPTS
    bsf     INTCON2,TMR0IP          ;Timer0 overflow Interrupt high priority
;   bsf     INTCON,TMR0IE           ;Timer0 overflow Interrupt enable
    bcf     IPR1,TMR1IP             ;Timer1 overflow Interrupt low priority
    bsf     PIE1,TMR1IE             ;Timer1 overflow Interrupt enable
#ifdef USE_RC_THR
    bsf     IPR1,CCP1IP             ;CCP1 Interrupt high priority
    bsf     PIE1,CCP1IE             ;CCP1 Interrupt enable
#endif
#ifdef  USE_INT0
    bsf     INTCON,INT0IE           ;INT0 Interrupt enable
    bcf     INTCON2,INTEDG0         ;Interrupt 0 on falling edge
#endif

    bsf     IPR3,IC1IP              ;IC1 Interrupt high priority
    bsf     IPR3,IC2QEIP            ;IC2 Interrupt high priority
    bsf     IPR3,IC3DRIP            ;IC3 Interrupt high priority

    bcf     IPR1,ADIP               ;AD Converter Interrupt low priority
    bsf     PIE1,ADIE               ;AD Converter Interrupt enable

    bsf     IPR2,LVDIP              ;LVD Interrupt high priority
;   bcf     PIR2,LVDIF              ;clear LVD interrupt flag
    bsf     PIE2,LVDIE              ;LVD Interrupt enable

    movlw   b'10010011'             ;Power ON reset status bit/Brownout reset status bit
    movwf   RCON                    ;and Instruction flag bits are set
                                    ;Enable Priority levels on Interrupts

    bsf     INTCON,GIEL             ;Enable low priority interrupts
    bsf     INTCON,GIEH             ;Enable high priority interrupts


RESET_INTERRUPTS
    bcf     INTCON,TMR0IE           ;Timer0 overflow Interrupt disable
    clrf    PIE3                    ;IC Interrupt disable
    return

;*******************************************************************************
;Try to synchronize onto the ZC using the Windmill strategy
;*******************************************************************************
WINDMILL_FIND
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    movlw   b'01000000'
    clrf    OVDCOND                 ;All motor PWM HI-Z
    andwf   OVDCONS                 ;preserve the state of PWM6
    bsf     INTCON,GIEL             ;Enable low priority interrupts
    clrf    STEP_CURRENT            ;step 0
    clrf    STEP_NEXT               ;step 0
    bcf     BMC_STATUS,AUTO_ADV     ;no auto advance
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    call    LOAD_DEMAG              ;make sure coils are demagnetized
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
WINDMILL_START
    movlw   HIGH PERIOD_MAX         ;reset period
    movwf   PERIOD_H
    movlw   LOW PERIOD_MAX
    movwf   PERIOD_L
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    LOAD_IC                 ;All IC active
    call    WAIT_FOR_ZC             ;wait for tmr0 ov or zc
    btfsc   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     WINDMILL_1ST_ZC         ;yes, continue
;   call    STEP_FIND
;   bz      WINDMILL_FAILED         ;no BEMF at all
    comf    WINDMILL_RETRY,W        ;0xFF -> always retry?
    bz      WINDMILL_START          ;yes, retry
    decfsz  WINDMILL_RETRY          ;no, limited nb of retry
    bra     WINDMILL_START          ;yes, retry
    bra     WINDMILL_FAILED
WINDMILL_1ST_ZC
    movff   ZC_NEW_L,ZC_OLD_L       ;save new ZC time
    movff   ZC_NEW_H,ZC_OLD_H
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    LOAD_IC
    call    WAIT_FOR_ZC             ;wait for tmr0 ov or zc
    btfsc   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     WINDMILL_2ND_ZC         ;yes, continue
;   call    STEP_FIND
;   bz      WINDMILL_FAILED
    comf    WINDMILL_RETRY,W        ;0xFF -> always retry?
    bz      WINDMILL_START          ;yes, retry
    decfsz  WINDMILL_RETRY          ;no, limited nb of retry
    bra     WINDMILL_START          ;yes, retry
    bra     WINDMILL_FAILED
WINDMILL_2ND_ZC
    call    STEP_FIND
    bz      WINDMILL_FAILED
    movwf   STEP_NEXT
    movf    ZC_NEW_L,W              ;PERIOD = ZC_OLD - ZC_NEW (16bit sub)
    subwf   ZC_OLD_L,W
    movwf   PERIOD_L
    movf    ZC_NEW_H,W
    subwfb  ZC_OLD_H,W
    movwf   PERIOD_H
    bnn     WINDMILL_CHECK_MIN      ;ABS(PERIOD), not negative?
    comf    PERIOD_H,F              ;Complement all bytes
    comf    PERIOD_L,F
    infsnz  PERIOD_L,F              ;Inc. low byte and Skip if no carry to higher bytes
    incf    PERIOD_H,F
WINDMILL_CHECK_MIN
    call    CLIP_PERIOD_MIN         ;Clip period to min value
    btfsc   ZC_STATUS,P_MIN_FLAG    ;Was period less than minimun
    bra     WINDMILL_START          ;yes, retry
    call    UPDATE_TIMING           ;Update period and advance timing
    movlw   6
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT
    incf    STEP_NEXT               ;next step
    call    LOAD_STEP
    bsf     BMC_STATUS,AUTO_ADV     ;re-enable auto advance
    bcf     ZC_STATUS,INIT_FLAG     ;no init sequence
    return
WINDMILL_FAILED
    bcf     ZC_STATUS,NEW_FLAG
    return


;********************************************************************
; Waiting loops
;********************************************************************
WAIT_FOR_TMR0
;   bsf     OSCCON,IDLEN
    btfss   GEN_STATUS,TMR0_OV_FLAG ;Timer0 overflow ocured?
;   sleep
    bra     WAIT_FOR_TMR0           ;No
    return

WAIT_FOR_ZC
    btfsc   ZC_STATUS,NEW_FLAG      ;ZC occured?
    return
    btfss   GEN_STATUS,TMR0_OV_FLAG ;Timer0 overflow ocured?
    bra     WAIT_FOR_ZC             ;Not yet, loop ...
WAIT_TIMEOUT
    clrf    CAP2CON                 ;clear IC2 Capture
    clrf    CAP3CON                 ;clear IC3 Capture
    clrf    CAP1CON                 ;clear IC1 Capture
    clrf    PIE3                    ;disable all IC interrupt
    return


;*******************************************************************************
;fatal assert
;*******************************************************************************
ASSERT
    call    INIT_PCPWM
ASSERT_LOOP
    debug1on
    clrf    TMR0H
    clrf    TMR0L
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    debug1off
    clrf    TMR0H
    clrf    TMR0L
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     ASSERT_LOOP


;********************************************************************
; Update the throttle command
;********************************************************************
UPDATE_RC_CMD
RC_MAX_CHECK
    movlw   HIGH RC_MAX
    cpfsgt  CCPR1_TEMP_H            ;CCPR1_TEMP_H > HIGH RC_MAX ?
    bra     RC_MAX_HEQ              ;no, so check =
    bra     UPDATE_RC_BAD           ;yes, pulse too long
RC_MAX_HEQ
    cpfseq  CCPR1_TEMP_H            ;CCPR1_TEMP_H = HIGH RC_MAX ?
    bra     RC_MIN_CHECK            ;no, so CCPR1_TEMP_H < HIGH RC_MAX, pulse not too long
    movlw   LOW RC_MAX
    cpfsgt  CCPR1_TEMP_L            ;so, CCPR1_TEMP_L > LOW RC_MAX ?
    bra     RC_MIN_CHECK            ;no, pulse not too long
    bra     UPDATE_RC_BAD           ;yes, pulse too long
RC_MIN_CHECK
    movlw   HIGH RC_MIN
    cpfslt  CCPR1_TEMP_H            ;CCPR1_TEMP_H < HIGH RC_MIN ?
    bra     RC_MIN_HEQ              ;no, so check =
    bra     UPDATE_RC_BAD           ;yes, pulse too short
RC_MIN_HEQ
    cpfseq  CCPR1_TEMP_H            ;CCPR1_TEMP_H = HIGH RC_MIN ?
    bra     UPDATE_RC_OK            ;no, so CCPR1_TEMP_H > HIGH RC_MIN, pulse not too short
    movlw   LOW RC_MIN
    cpfslt  CCPR1_TEMP_L            ;so, CCPR1_TEMP_L < LOW RC_MIN ?
    bra     UPDATE_RC_OK            ;no, pulse not too short
    bra     UPDATE_RC_BAD           ;yes, pulse too short
UPDATE_RC_OK
    btfss   GEN_STATUS,RC_LOCKED_FLAG   ;RC already locked?
    bra     UPDATE_RC_OK_INIT       ;no, init RC locking
    incf    RC_VALID_CNT            ;increase RC balance
    movlw   RC_VALID_MIN
    cpfslt  RC_VALID_CNT            ;RC_VALID_CNT<RC_VALID_MIN
    movwf   RC_VALID_CNT            ;no, clip to RC_VALID_MIN
    bra     UPDATE_THR_CMD
UPDATE_RC_OK_INIT
;   btfsc   GEN_STATUS,RC_LOCKED_FLAG   ;RC already locked?
;   bra     UPDATE_THR_CMD
    decfsz  RC_VALID_CNT            ;enough consecutive RC for a lock?
    return                          ;not yet
    bsf     GEN_STATUS,RC_LOCKED_FLAG   ;RC locked!!!!!
#ifdef RC_DEBUG_2
    debug2on
#endif
    movlw   RC_VALID_MIN            ;reset RC valid counter
    movwf   RC_VALID_CNT
    movlw   RC_BAD_MAX              ;reset RC bad counter
    movwf   RC_BAD_CNT
UPDATE_THR_CMD
#ifdef  USE_CAR_RC
    movlw   HIGH RC_CENTER
    movwf   VAR_LISR_H
    movlw   LOW RC_CENTER
    movwf   VAR_LISR_L
    movf    CCPR1_TEMP_L,W          ;VAR_LISR = RC_CENTER - CCPR1_TEMP (16bit sub)
    subwf   VAR_LISR_L,F
    movf    CCPR1_TEMP_H,W
    subwfb  VAR_LISR_H,F
    bnc     UPDATE_THR_CMD_MIN      ;borrow (no carry)?
#else
    movlw   LOW RC_CENTER           ;VAR_LISR = CCPR1_TEMP - RC_CENTER (16bit sub)
    subwf   CCPR1_TEMP_L,W
    movwf   VAR_LISR_L
    movlw   HIGH RC_CENTER
    subwfb  CCPR1_TEMP_H,W
    movwf   VAR_LISR_H
    bnc     UPDATE_THR_CMD_MIN      ;borrow (no carry)?
    movlw   0xFE
    andwf   VAR_LISR_H,W
#endif
    bz      UPDATE_THR_CMD_RET      ;
    movlw   0x01
    movwf   VAR_LISR_H
    setf    VAR_LISR_L              ;MSB not zero -> clip
UPDATE_THR_CMD_RET
#ifndef USE_CAR_RC
    bcf     STATUS,C                ;clear carry
    rrcf    VAR_LISR_H,F            ;/2 to re-scale the range
    rrcf    VAR_LISR_L,F
#endif
#ifdef MANUAL_ADV
    movff   VAR_LISR_L,ADV_CMD
#else
#ifdef  USE_RC_CMD_FILTER
    clrf    VAR_LISR_H
    movlw   7
    mulwf   THR_CMD
    movf    PRODL,W
    addwf   VAR_LISR_L
    movf    PRODH,W
    addwfc  VAR_LISR_H
    rrcf    VAR_LISR_H,F
    rrcf    VAR_LISR_L,F
    rrcf    VAR_LISR_H,F
    rrcf    VAR_LISR_L,F
    rrcf    VAR_LISR_H,F
    rrcf    VAR_LISR_L,W
    movwf   THR_CMD
#else
    movff   VAR_LISR_L,THR_CMD
#endif
#endif
    return
UPDATE_THR_CMD_MIN
    clrf    VAR_LISR_H
    clrf    VAR_LISR_L
    bra     UPDATE_THR_CMD_RET

UPDATE_RC_BAD
    btfss   GEN_STATUS,RC_LOCKED_FLAG   ;RC already locked?
    bra     UPDATE_RC_BAD_INIT      ;no, init RC locking
    decfsz  RC_VALID_CNT            ;no -> decrease RC balance
    return                          ;balance not zero, keep going
#ifdef RC_DEBUG_2
    debug2off
#endif
    bcf     GEN_STATUS,RC_LOCKED_FLAG   ;RC sync lost
#ifdef MANUAL_ADV
    movlw   ADV_CMD_INIT
    movwf   ADV_CMD
#else
    clrf    THR_CMD                 ;too many bad RC pulse, coast
    bsf     THR_CMD,0
#endif
    movlw   RC_BAD_MAX              ;reset RC bad counter
    movwf   RC_BAD_CNT
    movlw   RC_VALID_MIN            ;reset RC valid counter
    movwf   RC_VALID_CNT
    return

UPDATE_RC_BAD_INIT
    movlw   RC_VALID_MIN            ;reset RC valid counter
    movwf   RC_VALID_CNT
    decfsz  RC_BAD_CNT
    return

;********************************************************************
; Update the MPPT
;********************************************************************
UPDATE_MPPT

;   btfss   GEN_STATUS,MPPT_RUN     ;MPPT running allowed?
;   bra     UPDATE_MPPT_CC          ;no, so no MPPT update
    btfsc   SCC_STATUS,ACTIVE_FLAG  ;Is the SCC runing and active
    bra     UPDATE_MPPT_START       ;yes, run MPPT
    btfss   BMC_STATUS,ACTIVE_FLAG  ;Is the BMC runing and active
    bra     UPDATE_MPPT_CC          ;no, so no MPPT update
    btfss   ZC_STATUS,LOCKED_FLAG   ;already locked?
    bra     UPDATE_MPPT_CC          ;no, so no MPPT update

UPDATE_MPPT_START



;caculate C_ERR and C_DIF
UPDATE_MPPT_CC
    movff   C_ERR_H,VAR_LISR_H      ;save C_ERR to calcute C_DIF
    movff   C_ERR_L,VAR_LISR_L
    movf    V_MPP_L,W               ;C_ERR =  V_DC - V_MPP(16bit sub)
    subwf   V_DC_L,W
    movwf   C_ERR_L
    movf    V_MPP_H,W
    subwfb  V_DC_H,W
    movwf   C_ERR_H
    movf    VAR_LISR_L,W            ;C_DIF =  C_ERR - VAR_LISR(16bit sub)
    subwf   C_ERR_L,W
    movwf   C_DIF_L
    movf    VAR_LISR_H,W
    subwfb  C_ERR_H,W
    movwf   C_DIF_H                 ;signed error difference -> C_DIF
    return

;********************************************************************
; Update the variable BMC PWM duty
;********************************************************************
UPDATE_BMC_PWM                          ;update PWM duty

V_DC_MIN_CHECK
    movlw   HIGH V_DC_MIN
    cpfslt  V_DC_H                  ;V_DC_H < HIGH V_DC_MIN ?
    bra     V_DC_MIN_HEQ            ;no, so check =
    bra     UPDATE_VDC_UV           ;yes, VDC too low
V_DC_MIN_HEQ
    cpfseq  V_DC_H                  ;V_DC_H = HIGH V_DC_MIN ?
    bra     V_DC_MAX_CHECK          ;no, so V_DC_H > HIGH V_DC_MIN, VDC ok
    movlw   LOW V_DC_MIN
    cpfsgt  V_DC_L                  ;so, V_DC_L > LOW V_DC_MIN ?
    bra     UPDATE_VDC_UV           ;no, VDC too low

V_DC_MAX_CHECK
    movlw   HIGH V_DC_MAX
    cpfsgt  V_DC_H                  ;V_DC_H > HIGH V_DC_MAX ?
    bra     V_DC_MAX_HEQ            ;no, so check =
    bra     UPDATE_VDC_OV           ;yes, VDC too high
V_DC_MAX_HEQ
    cpfseq  V_DC_H                  ;V_DC_H = HIGH V_DC_MAX ?
    bra     UPDATE_VDC_OK           ;no, so V_DC_H < HIGH V_DC_MAX, VDC ok
    movlw   LOW V_DC_MAX
    cpfslt  V_DC_L                  ;so, V_DC_L < LOW V_DC_MAX ?
    bra     UPDATE_VDC_OV           ;no, VDC too high

UPDATE_VDC_OK
#ifdef CHECK_I_MAX
I_IN_MAX_CHECK
    movlw   HIGH I_IN_MAX
    cpfsgt  I_IN_H                  ;I_IN_H > HIGH I_IN_MAX ?
    bra     I_IN_MAX_HEQ            ;no, so check =
    goto    ASSERT
I_IN_MAX_HEQ
    cpfseq  I_IN_H                  ;I_IN_H = HIGH I_IN_MAX ?
    bra     UPDATE_VDC_OK_OUT       ;no, so I_IN_H < HIGH I_IN_MAX, iIN ok
    movlw   LOW I_IN_MAX
    cpfslt  I_IN_L                  ;so, I_IN_L < LOW I_IN_MAX ?
    goto    ASSERT
UPDATE_VDC_OK_OUT
#endif

    bra     UPDATE_BMC

;Critical Over Voltage, hold BMC in reset and switch off the SCC
UPDATE_VDC_OV
    bsf     BMC_STATUS,RESET_FLAG
    bcf     SCC_STATUS,RUN_FLAG
    bra     UPDATE_BMC

;Critical Under Voltage, hold BMC in reset
UPDATE_VDC_UV
    bsf     BMC_STATUS,RESET_FLAG
;   bra     UPDATE_BMC

UPDATE_BMC                          ;update BMC state and PWM duty
    btfsc   BMC_STATUS,RESET_FLAG
    bra     UPDATE_BMC_RESET        ;reset

    movlw   THR_CMD_MIN
    cpfslt  THR_CMD                 ;THR_CMD < MIN?
    bra     UPDATE_BMC_RUN          ;no -> run

UPDATE_BMC_NORUN
    bcf     BMC_STATUS,RUN_FLAG
    movf    THR_CMD,W
    bz      UPDATE_BMC_BRAKE
    bra     UPDATE_BMC_RESET

UPDATE_BMC_BRAKE
#ifdef  USE_BRAKE
    bcf     BMC_STATUS,STEPPING_ON
    clrf    OVDCOND
    clrf    OVDCOND_TEMP
    movf    OVDCONS,W
    andlw   b'01000000'             ;preserve PWM6
    iorlw   b'00010101'
    movwf   OVDCONS
    movwf   OVDCONS_TEMP
    clrf    DUTY_REQ
;   bsf     ZC_STATUS,RESET_FLAG
    bra     UPDATE_PWM_ZERO
#else
    bra     UPDATE_BMC_RESET
#endif

UPDATE_BMC_RUN
    bsf     BMC_STATUS,RUN_FLAG     ;run mode

    btfsc   ZC_STATUS,LOCKED_FLAG   ;already locked?
    bra     UPDATE_DUTY_REQ_INIT    ;yes

    movlw   THR_MIN_NOLOCK
    cpfsgt  THR_CMD                 ;THR_CMD > MIN_NOLOCK?
    bra     UPDATE_BMC_RESET        ;not yet

    movff   THR_CMD,DUTY_REQ

    movf    THR_NOLOCK,W
    cpfsgt  THR_CMD                 ;THR_CMD > MAX_NOLOCK?
    bra     UPDATE_BMC_PID          ;no continue
    movwf   DUTY_REQ                ;limit the throttle until 1st ZC lock
    bra     UPDATE_BMC_PID

UPDATE_DUTY_REQ_INIT
;   movf    PWM_DUTY,W
;   bnz     UPDATE_DUTY_REQ_LIMIT
;   call    BMC_DUTY_INIT_CAL
;   bra     UPDATE_PWM_REG_CHK

UPDATE_DUTY_REQ_LIMIT
;DUTY_REQ update frequency
    decfsz  DUTY_REQ_PERIOD
    bra     UPDATE_BMC_PID          ;end of period not reached, no DUTY_REQ update
    movlw   DUTY_REQ_PERIOD_MAX     ;reset DUTY_REQ_PERIOD
    movwf   DUTY_REQ_PERIOD

    bsf     BMC_STATUS,ACTIVE_FLAG
    movf    DUTY_REQ,W
    cpfslt  THR_CMD                 ;THR_CMD < DUTY_REQ?
    bra     UPDATE_DUTY_HEQ         ;no
    movlw   K_THR_CMD_DEC
    subwf   DUTY_REQ,W
    bc      UPDATE_DUTY_NO_MIN
    clrf    WREG
UPDATE_DUTY_NO_MIN
    cpfslt  THR_CMD                 ;THR_CMD < DUTY_REQ - K_THR_CMD_DEC?
    movf    THR_CMD,W               ;no, THR_CMD -> DUTY_REQ
    movwf   DUTY_REQ                ;yes, DUTY_REQ - K_THR_CMD_DEC -> THR_CMD
    bra     UPDATE_BMC_PID
UPDATE_DUTY_HEQ
    addlw   K_THR_CMD_ACC
    bnc     UPDATE_DUTY_NO_MAX
    setf    WREG
UPDATE_DUTY_NO_MAX
    cpfsgt  THR_CMD                 ;THR_CMD > DUTY_REQ + K_THR_CMD_ACC?
    movf    THR_CMD,W               ;no, THR_CMD -> DUTY_REQ
    movwf   DUTY_REQ                ;yes, DUTY_REQ + K_THR_CMD_ACC -> THR_CMD
;   bra     UPDATE_BMC_PID

UPDATE_BMC_PID
#ifdef USE_BMC_PID
    movlw   K_PIDM_P                ;P component
    mulwf   C_ERR_H                 ;K_PIDM_PxC_ERR_H -> VAR_LISR_H:VAR_LISR_L
    movff   PRODH,VAR_LISR_H        ;Copy high product in  VAR_LISR_H
    movff   PRODL,VAR_LISR_L        ;Copy low product in VAR_LISR_L
    mulwf   C_ERR_L                 ;K_PIDM_PxC_ERR_L -> PRODH:PRODL
    btfsc   C_ERR_H,7               ;test sign bit
    subwf   VAR_LISR_H,F
    movf    PRODH,W                 ;Copy high product in W
    addwf   VAR_LISR_L,F            ;Add intermediate products
    clrf    WREG
    addwfc  VAR_LISR_H,F

    movf    PRODL,W                 ;Add P component to PWM_DUTY (I)
    addwf   PWM_DUTY_S,F
    movf    VAR_LISR_L,W
    addwfc  PWM_DUTY_L,F
    movf    VAR_LISR_H,W
    addwfc  PWM_DUTY_H,F

    movlw   K_PIDM_D                ;D component
    mulwf   C_DIF_H                 ;K_PIDM_DxC_DIF_H -> VAR_LISR_H:VAR_LISR_L
    movff   PRODH,VAR_LISR_H        ;Copy high product in  VAR_LISR_H
    movff   PRODL,VAR_LISR_L        ;Copy low product in VAR_LISR_L
    mulwf   C_DIF_L                 ;K_PIDM_PxC_DIF_L -> PRODH:PRODL
    btfsc   C_DIF_H,7               ;test sign bit
    subwf   VAR_LISR_H,F
    movf    PRODH,W                 ;Copy high product in W
    addwf   VAR_LISR_L,F            ;Add intermediate products
    clrf    WREG
    addwfc  VAR_LISR_H,F

    movf    PRODL,W                 ;Add D component to PWM_DUTY
    addwf   PWM_DUTY_S,F
    movf    VAR_LISR_L,W
    addwfc  PWM_DUTY_L,F
    movf    VAR_LISR_H,W
    addwfc  PWM_DUTY_H,F
    bn      UPDATE_PWM_ZERO
    movlw   0xFC
    andwf   PWM_DUTY_H,W
    bz      UPDATE_PWM_REG
    setf    PWM_DUTY_L              ;carry -> clip to MAX
    movlw   0x03
    movwf   PWM_DUTY_H

UPDATE_PWM_REG
    rrcf    PWM_DUTY_H,W            ;/2
    movwf   VAR_LISR_H
    rrcf    PWM_DUTY_L,W
    movwf   PWM_DUTY
    rrcf    VAR_LISR_H,F            ;/2
    rrcf    PWM_DUTY,F
    movf    DUTY_REQ,W
    cpfsgt  PWM_DUTY                ;PWM_DUTY > DUTY_REQ?
    bra     UPDATE_PWM_REG_CHK      ;no
#endif
UPDATE_PWM_NOPID
    movff   DUTY_REQ,PWM_DUTY       ;yes, ignore PID and use DUTY_REQ
    movlw   0x04                    ;<<2
    mulwf   PWM_DUTY
    movff   PRODH,PWM_DUTY_H
    movff   PRODL,PWM_DUTY_L
    bcf     BMC_STATUS,ACTIVE_FLAG

UPDATE_PWM_REG_CHK
    bsf     BMC_STATUS,STEPPING_ON

CLIP_PWM_DUTY_MAX                   ;Test to see if PWM_DUTY > PWM_DUTY_MAX
    movlw   HIGH PWM_DUTY_MAX
    cpfsgt  PWM_DUTY_H              ;PWM_DUTY_H > HIGH PWM_DUTY_MAX ?
    bra     CLIP_PWM_DUTY_MAX_HEQ   ;no, so check =
    bra     UPDATE_PWM_MAX          ;yes, clip it to max
CLIP_PWM_DUTY_MAX_HEQ
    cpfseq  PWM_DUTY_H              ;PWM_DUTY_H = HIGH PWM_DUTY_MAX ?
    bra     CLIP_PWM_DUTY_MIN       ;no, so PWM_DUTY_H < HIGH PWM_DUTY_MAX, no clipping needed
    movlw   LOW PWM_DUTY_MAX
    cpfsgt  PWM_DUTY_L              ;so, PWM_DUTY_L > LOW PWM_DUTY_MAX ?
    bra     CLIP_PWM_DUTY_MIN       ;no, no clipping needed
    bra     UPDATE_PWM_MAX          ;yes, clip it to max

CLIP_PWM_DUTY_MIN                   ;Test to see if PWM_DUTY < PWM_DUTY_MIN
    movlw   HIGH PWM_DUTY_MIN
    cpfslt  PWM_DUTY_H              ;PWM_DUTY_H < HIGH PWM_DUTY_MIN ?
    bra     CLIP_PWM_DUTY_MIN_HEQ   ;no, so check =
    bra     UPDATE_PWM_ZERO         ;yes, clip it to min
CLIP_PWM_DUTY_MIN_HEQ
    cpfseq  PWM_DUTY_H              ;PWM_DUTY_H = HIGH PWM_DUTY_MIN ?
    bra     UPDATE_PWM_GO           ;no, so PWM_DUTY_H > HIGH PWM_DUTY_MIN, no clipping needed
    movlw   LOW PWM_DUTY_MIN
    cpfslt  PWM_DUTY_L              ;so, PWM_DUTY_L < LOW PWM_DUTY_MIN ?
    bra     UPDATE_PWM_GO           ;no, no clipping needed
    bra     UPDATE_PWM_ZERO         ;yes, clip it to min

UPDATE_BMC_RESET
    bcf     BMC_STATUS,STEPPING_ON
    movlw   b'01000000'
    clrf    OVDCOND                 ;All motor PWM HI-Z
    clrf    OVDCOND_TEMP
    andwf   OVDCONS                 ;preserve the state of PWM6
    clrf    OVDCONS_TEMP
    clrf    DUTY_REQ
    bcf     BMC_STATUS,RESET_FLAG
    bcf     BMC_STATUS,RUN_FLAG
;   bsf     ZC_STATUS,RESET_FLAG
    bra     UPDATE_PWM_ZERO

UPDATE_PWM_ZERO
;   setf    PWM_DUTY_L              ;carry -> clip to MAX
;   movlw   0x03
;   movwf   PWM_DUTY_H
    clrf    PWM_DUTY
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    clrf    PDC0L
    clrf    PDC0H
    clrf    PDC1L
    clrf    PDC1H
    clrf    PDC2L
    clrf    PDC2H
    bcf     PWMCON1, UDIS           ;Enable updates to duty cycle and period
    return

UPDATE_PWM_MAX
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    setf    PDC0L
    setf    PDC0H
    setf    PDC1L
    setf    PDC1H
    setf    PDC2L
    setf    PDC2H
    bcf     PWMCON1, UDIS           ;Enable updates to duty cycle and period
    return

UPDATE_PWM_GO
    movf    PWM_DUTY_L,W
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    movwf   PDC0L
    movwf   PDC1L
    movwf   PDC2L
    movf    PWM_DUTY_H,W
    movwf   PDC0H
    movwf   PDC1H
    movwf   PDC2H
    bcf     PWMCON1, UDIS
    return

UPDATE_PWM_ERROR
    goto    ASSERT


;********************************************************************
; Update the variables Period and advance timing
;********************************************************************
UPDATE_TIMING
    btfss   ZC_STATUS,LOCKED_FLAG   ;already locked?
    bra     UPDATE_ZC_LOCK_INIT     ;no, init ZC locking
    btfsc   ZC_STATUS,NEW_FLAG      ;new ZC flag?
    bra     UPDATE_ZC_LOCKED        ;yes
    decfsz  ZC_VALID_CNT            ;no -> decrease ZC balance
    bra     UPDATE_ZC_PERIOD        ;balance not zero, keep going
    bcf     ZC_STATUS,SYNCED_FLAG   ;ZC sync lost
    bcf     ZC_STATUS,LOCKED_FLAG   ;ZC locked lost!!!
    bsf     ZC_STATUS,RESET_FLAG
#ifdef ZC_LOCKED_DEBUG_1
    debug1off
#endif
    return
UPDATE_ZC_LOCKED
    bcf     ZC_STATUS,NEW_FLAG      ;clear zc new flag
    incf    ZC_VALID_CNT            ;increase ZC balance
    movlw   ZC_VALID_MIN
    cpfslt  ZC_VALID_CNT            ;ZC_VALID_CNT<ZC_VALID_MIN
    movwf   ZC_VALID_CNT            ;no, clip to ZC_VALID_MIN
    bra     UPDATE_ZC_PERIOD
UPDATE_ZC_LOCK_INIT
    btfsc   ZC_STATUS,NEW_FLAG      ;new ZC flag?
    bra     UPDATE_ZC_VALID
UPDATE_ZC_BAD
    bcf     ZC_STATUS,SYNCED_FLAG   ;ZC sync lost
    movlw   ZC_VALID_MIN            ;reset zc valid counter
    movwf   ZC_VALID_CNT
    decfsz  ZC_BAD_CNT              ;too many bad ZC
    bra     UPDATE_ZC_PERIOD        ;not yet
    bcf     ZC_STATUS,LOCKED_FLAG   ;ZC locked lost!!!
    bsf     ZC_STATUS,RESET_FLAG
#ifdef ZC_LOCKED_DEBUG_1
    debug1off
#endif
;might want to reset timing here!
    return
UPDATE_ZC_VALID
    bcf     ZC_STATUS,NEW_FLAG      ;clear zc new flag
    btfsc   ZC_STATUS,SYNCED_FLAG   ;ZC already synced?
    bra     UPDATE_ZC_PERIOD
    decfsz  ZC_VALID_CNT            ;enough consecutive ZC for a sync and lock?
    bra     UPDATE_ZC_PERIOD        ;not yet
    bsf     ZC_STATUS,LOCKED_FLAG   ;ZC locked!!!!!
    bsf     ZC_STATUS,SYNCED_FLAG
#ifdef ZC_LOCKED_DEBUG_1
    debug1on
#endif
    movlw   ZC_VALID_MIN            ;reset zc valid counter
    movwf   ZC_VALID_CNT
    movlw   ZC_BAD_MAX              ;reset zc bad counter
    movwf   ZC_BAD_CNT
UPDATE_ZC_PERIOD
    movf    ZC_NEW_L,W              ;ZC_OLD = ZC_OLD - ZC_NEW (16bit sub)
    subwf   ZC_OLD_L,F
    movf    ZC_NEW_H,W
    subwfb  ZC_OLD_H,F
    bnn     UPDATE_FILTER_SELECT    ;ABS(ZC_OLD), not negative?
    comf    ZC_OLD_H,F              ;Complement all bytes
    comf    ZC_OLD_L,F
    infsnz  ZC_OLD_L,F              ;Inc. low byte and Skip if no carry to higher bytes
    incf    ZC_OLD_H,F
UPDATE_FILTER_SELECT
#ifdef USE_AUTO_SYNC_FILTER
    btfsc   ZC_STATUS,SYNCED_FLAG   ;ZC synced?
    bra     UPDATE_FILTER_SLOW
#endif
;#ifdef ZC_EARLY_DEBUG_1
;   debug1on
;#endif
;   btfss   ZC_STATUS,EARLY_FLAG    ;no sync, early ZC?
;   bra     UPDATE_FILTER_FAST      ;no, fast filter
;#ifdef ZC_EARLY_DEBUG_1
;   debug1off
;#endif
;UPDATE_FILTER_EARLY
;   movlw   0x01                    ;PERIOD = PERIOD - 1
;   subwf   PERIOD_L,F
;   clrf    WREG
;   subwfb  PERIOD_H,F
;   bra     UPDATE_PERIOD_CHECK
UPDATE_FILTER_FAST
    movf    ZC_OLD_L,W              ;NEW_PERIOD = (OLD_PERIOD + NEW_PERIOD)/2
    addwf   PERIOD_L,F
    movf    ZC_OLD_H,W
    addwfc  PERIOD_H,F
    bc      UPDATE_ERROR            ;Should always be no carry!
    rrcf    PERIOD_H,F              ;/2
    rrcf    PERIOD_L,F
    movlw   0x7F
    andwf   PERIOD_H,F
    bra     UPDATE_PERIOD_CHECK
;UPDATE_FILTER_NONE
;   movff   ZC_OLD_L,PERIOD_L       ;No filter NEW_PERIOD = ZC_OLD - ZC_NEW
;   movff   ZC_OLD_H,PERIOD_H
;   bra     UPDATE_PERIOD_CHECK
UPDATE_FILTER_SLOW
    movlw   7                       ;NEW_PERIOD = (7xOLD_PERIOD + NEW_PERIOD)/8
    mulwf   PERIOD_H                ;3xPERIOD_H -> VAR_TEMP_H:PERIOD_H
    movff   PRODL,PERIOD_H          ;Copy low product in PERIOD_H
    movff   PRODH,VAR_TEMP_H        ;Copy high product in  VAR_TEMP_H
    mulwf   PERIOD_L                ;3xPERIOD_L -> W:PERIOD_L
    movff   PRODL,PERIOD_L          ;Copy low product in PERIOD_L
    movf    PRODH,W                 ;Copy high product in  W
    addwf   PERIOD_H,F              ;Add intermediate products
    clrf    WREG
    addwfc  VAR_TEMP_H,F
    movf    ZC_OLD_L,W              ;VAR_TEMP_H:PERIOD_H:PERIOD_L =+ ZC_OLD_H:ZC_OLD_L
    addwf   PERIOD_L,F
    movf    ZC_OLD_H,W
    addwfc  PERIOD_H,F
    clrf    WREG
    addwfc  VAR_TEMP_H,F
    rrcf    VAR_TEMP_H,F            ;/2
    rrcf    PERIOD_H,F
    rrcf    PERIOD_L,F
    rrcf    VAR_TEMP_H,F            ;/2
    rrcf    PERIOD_H,F
    rrcf    PERIOD_L,F
    rrcf    VAR_TEMP_H,F            ;/2
    rrcf    PERIOD_H,F
    rrcf    PERIOD_L,F
    movlw   0x1F
    andwf   VAR_TEMP_H,F
    bnz     UPDATE_ERROR            ;Should always be zero!
UPDATE_PERIOD_CHECK
    btfsc   ZC_STATUS,LOCKED_FLAG   ;already locked?
    bra     UPDATE_PERIOD_CLIP      ;yes, go on clipping
UPDATE_PERIOD_CLIP
    call    CLIP_PERIOD_MAX         ;Clip period to max value
    call    CLIP_PERIOD_MIN         ;Clip period to min value
    movff   ZC_NEW_L,ZC_OLD_L       ;save new ZC time
    movff   ZC_NEW_H,ZC_OLD_H
    bcf     ZC_STATUS,EARLY_FLAG
UPDATE_ADV
#ifdef MANUAL_ADV
    rrcf    ADV_CMD,W
    andlw   0x7F                    ;/2, 30degrees range only
;   movf    ADV_CMD,W
#else
    call    ADV_CAL                 ;Auto advance
#endif
    bnz     TIMING_ADV_CAL          ;check if zero?
    bsf     BMC_STATUS,ADV_MAX_FLAG ;set flag so no ADV waiting!
    return
TIMING_ADV_CAL
    bcf     BMC_STATUS,ADV_MAX_FLAG
    mulwf   PERIOD_H                ;PERIOD/256 x W (ADV), high byte fist
    movff   PRODH,ADV_TIME_H        ;Copy high product in ADV_TIME_H
    movff   PRODL,ADV_TIME_L        ;Copy low product in ADV_TIME_L
    mulwf   PERIOD_L                ;lower byte
    movf    PRODH,W
    addwf   ADV_TIME_L,F            ;add the 2 product
    movlw   0
    addwfc  ADV_TIME_H,F
    comf    ADV_TIME_H,F            ;Complement all bytes
    comf    ADV_TIME_L,F
    infsnz  ADV_TIME_L,F            ;Inc. low byte and Skip if no carry to higher bytes
    incf    ADV_TIME_H,F
    return

UPDATE_ERROR
    goto    ASSERT

;********************************************************************
; Calculate the advance value using a table (hardware dependant)
;********************************************************************
#define PERIOD_ADV  0x13            ;(170Hz)2432/128->19
#define ADV_MIN     0x77            ;28 degrees

ADV_TABLE                           ;LSB,MSB in 1/256
    db      0x00,0x00
    db      0x00,0x19
    db      0x2A,0x37
    db      0x40,0x44
    db      0x48,0x4C
    db      0x51,0x55
    db      0x59,0x5D
    db      0x62,0x66
    db      0x6A,0x6E
    db      0x73,ADV_MIN

ADV_CAL
    rlcf    PERIOD_L, W             ;*2/256 -> PERIOD/128
    rlcf    PERIOD_H, W
    movwf   TBLPTRL
    movlw   PERIOD_ADV
    cpfsgt  TBLPTRL                 ;PERIOD/128 > PERIOD_ADV ?
    bra     ADV_USE_TABLE           ;no, use table
    movlw   ADV_MIN                 ;use the advance min
    return
ADV_USE_TABLE
    movlw   LOW ADV_TABLE           ;Initialize low byte of table pointer
    addwf   TBLPTRL
    clrf    TBLPTRH
    movlw   HIGH ADV_TABLE          ;Initialize high byte of table pointer
    addwfc  TBLPTRH
    clrf    TBLPTRU
    movlw   UPPER ADV_TABLE         ;Initialize upper byte of table pointer
    addwfc  TBLPTRU
    tblrd*
    movf    TABLAT, W
    return

;********************************************************************
; Clip period to max value
;********************************************************************
CLIP_PERIOD_MAX                 ;Test to see if PERIOD > PERIOD_MAX
    movlw   HIGH PERIOD_MAX
    cpfsgt  PERIOD_H            ;PERIOD_H > HIGH PERIOD_MAX ?
    bra     CLIP_PERIOD_MAX_HEQ ;no, so check =
    bra     CLIP_PERIOD_MAX_GT  ;yes, clip it to max
CLIP_PERIOD_MAX_HEQ
    cpfseq  PERIOD_H            ;PERIOD_H = HIGH PERIOD_MAX ?
    return                      ;no, so PERIOD_H < HIGH PERIOD_MAX, no clipping needed
    movlw   LOW PERIOD_MAX
    cpfsgt  PERIOD_L            ;so, PERIOD_L > LOW PERIOD_MAX ?
    return                      ;no, no clipping needed
CLIP_PERIOD_MAX_GT
    movlw   HIGH PERIOD_MAX
    movwf   PERIOD_H
    movlw   LOW PERIOD_MAX
    movwf   PERIOD_L
    return

;********************************************************************
; Clip period to min value
;********************************************************************
CLIP_PERIOD_MIN                 ;Test to see if PERIOD < PERIOD_MIN
    bcf     ZC_STATUS,P_MIN_FLAG
    movlw   HIGH PERIOD_MIN
    cpfslt  PERIOD_H            ;PERIOD_H < HIGH PERIOD_MIN ?
    bra     CLIP_PERIOD_MIN_HEQ ;no, so check =
    bra     CLIP_PERIOD_MIN_LT  ;yes, clip it to min
CLIP_PERIOD_MIN_HEQ
    cpfseq  PERIOD_H            ;PERIOD_H = HIGH PERIOD_MIN ?
    return                      ;no, so PERIOD_H > HIGH PERIOD_MIN, no clipping needed
    movlw   LOW PERIOD_MIN
    cpfslt  PERIOD_L            ;so, PERIOD_L < LOW PERIOD_MIN ?
    return                      ;no, no clipping needed
CLIP_PERIOD_MIN_LT
    movlw   HIGH PERIOD_MIN
    movwf   PERIOD_H
    movlw   LOW PERIOD_MIN
    movwf   PERIOD_L
    bsf     ZC_STATUS,P_MIN_FLAG
    return

;********************************************************************
; Calculate advance timing and reload timer 0
;********************************************************************
LOAD_ADVANCE
    bcf     INTCON,TMR0IE           ;disable timer 0 int
    btfsc   BMC_STATUS,ADV_MAX_FLAG ;Max advance?
    bra     LOAD_ADVANCE_MAX
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    movff   ADV_TIME_H,TMR0H        ;Load the Higher byte to TMR0H
    movff   ADV_TIME_L,TMR0L        ;Load the Lower byte to TMR0L
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    return
LOAD_ADVANCE_MAX
    bsf     GEN_STATUS,TMR0_OV_FLAG
    return

;********************************************************************
; Calculate demagnetization timing and reload timer 0
;********************************************************************
LOAD_DEMAG
    bcf     INTCON,TMR0IE           ;disable timer 0 int
#ifdef MANUAL_DEMAG
;   rrcf    DEMAG_CMD,W             ;Manual demag
;   andlw   0x7F                    ;/2, 30degrees range only
    bra     DEMAG_CAL
#endif
    movlw   K_DEMAG
    mulwf   PWM_DUTY                ;K_DEMAGxPWM_DUTY/256
    movff   PRODH,VAR_TEMP_H        ;Copy high product in VAR_TEMP_H
CHEK_DEMAG_MIN
    movlw   DEMAG_CMD_MIN
    cpfslt  VAR_TEMP_H
    movf    VAR_TEMP_H,W
DEMAG_CAL
    mulwf   PERIOD_H                ;PERIOD/256 x DEMAG, high byte fist
    movff   PRODH,VAR_TEMP_H        ;Copy high product in VAR_TEMP_H
    movff   PRODL,VAR_TEMP_L        ;Copy low product in VAR_TEMP_L
    mulwf   PERIOD_L                ;lower byte
    movf    PRODH,W
    addwf   VAR_TEMP_L,F            ;add the 2 product
    clrf    WREG
    addwfc  VAR_TEMP_H,F
    comf    VAR_TEMP_H,F            ;Complement all bytes
    comf    VAR_TEMP_L,F
    infsnz  VAR_TEMP_L,F            ;Inc. low byte and Skip if no carry to higher bytes
    incf    VAR_TEMP_H,F
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    movff   VAR_TEMP_H,TMR0H        ;Load the Higher byte to TMR0H
    movff   VAR_TEMP_L,TMR0L        ;Load the Lower byte to TMR0L
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    return

;********************************************************************
; Calculate timer 0 timeout from current Period and reload timer 0
;********************************************************************
LOAD_TIMEOUT
    bcf     INTCON,TMR0IE           ;disable timer 0 int
    movff   PERIOD_H,VAR_TEMP_H
    movff   PERIOD_L,VAR_TEMP_L
    comf    VAR_TEMP_H,F            ;Complement all bytes
    comf    VAR_TEMP_L,F
    infsnz  VAR_TEMP_L,F            ;Inc. low byte and Skip if no carry to higher bytes
    incf    VAR_TEMP_H,F
    bcf     GEN_STATUS,TMR0_OV_FLAG
    bcf     INTCON,TMR0IF           ;Clear TMR0IF
    movff   VAR_TEMP_H,TMR0H        ;Load the Higher byte to TMR0H
    movff   VAR_TEMP_L,TMR0L        ;Load the Lower byte to TMR0L
    bsf     INTCON,TMR0IE           ;enable timer 0 int
    return

;*******************************************************************************
; Identify the current step after ZC detection
;*******************************************************************************
STEP_TABLE                          ;LSB,MSB
    db      0,1
    db      3,2
    db      5,6
    db      4,7

STEP_FIND
    movff   PORTA, VAR_TEMP     ;read and format the comparator output
    rrcf    VAR_TEMP, F
    rrcf    VAR_TEMP, W
    andlw   0x07
    movwf   TBLPTRL
    movlw   LOW STEP_TABLE          ;Initialize low byte of table pointer
    addwf   TBLPTRL
    clrf    TBLPTRH
    movlw   HIGH STEP_TABLE         ;Initialize high byte of table pointer
    addwfc  TBLPTRH
    clrf    TBLPTRU
    movlw   UPPER STEP_TABLE        ;Initialize upper byte of table pointer
    addwfc  TBLPTRU
    tblrd*
    movf    TABLAT, W
    return

;*******************************************************************************
; Pre-load the PWM overdrive setting for the next step
;*******************************************************************************
OVDCON_TABLE                        ;LSB, MSB
    db  b'00000000',b'00000000'     ;Step 0 PWM config, phase C: X,    phase B: X,    phase A: X

    db  b'00000010',b'00000101'     ;Step 1 PWM config, phase C: H->X, phase B: L->L, phase A: X->H

    db  b'00000010',b'00010001'     ;Step 2 PWM config, phase C: X->L, phase B: L->X, phase A: H->H

    db  b'00001000',b'00010100'     ;Step 3 PWM config, phase C: L->L, phase B: X->H, phase A: H->X

    db  b'00001000',b'00000101'     ;Step 4 PWM config, phase C: L->X, phase B: H->H, phase A: X->L

    db  b'00100000',b'00010001'     ;Step 5 PWM config, phase C: X->H, phase B: H->X, phase A: L->L

    db  b'00100000',b'00010100'     ;Step 6 PWM config, phase C: H->H, phase B: X->L, phase A: L->X

    db  b'00000000',b'00010101'     ;Step 7 PWM config, phase C: L,    phase B: L,    phase A: L

LOAD_STEP
    rlcf    STEP_NEXT, W            ;2 bytes space
    andlw   0xFE
    addlw   LOW OVDCON_TABLE        ;Initialize low byte of table pointer
    movwf   TBLPTRL
    clrf    TBLPTRH
    movlw   HIGH OVDCON_TABLE       ;Initialize high byte of table pointer
    addwfc  TBLPTRH
    clrf    TBLPTRU
    movlw   UPPER OVDCON_TABLE      ;Initialize upper byte of table pointer
    addwfc  TBLPTRU
    tblrd*+
    movff   TABLAT,OVDCOND_TEMP
    tblrd*
    movff   TABLAT,OVDCONS_TEMP
    return


;*******************************************************************************
;load the IC with new step config
;*******************************************************************************
LOAD_IC     org 0x1000
    bcf     ZC_STATUS,NEW_FLAG      ;clear zc flag
    movlw   UPPER(IC_TAB)
    movwf   PCLATU
    movlw   HIGH(IC_TAB)
    movwf   PCLATH
    rlcf    STEP_CURRENT, W
    movwf   VAR_TEMP
    rlcf    VAR_TEMP, W             ;2 words space
    andlw   0xFC
    addwf   PCL
IC_TAB
    movlw   b'00001000'             ;Step 0 look out for any edge and all Input Capture
    bra     LOAD_ALLIC
    movlw   b'00000001'             ;Step 1 BEMF3, IC3 Capture on falling edge
    bra     LOAD_IC3
    movlw   b'00000010'             ;Step 2 BEMF2, IC2 Capture on raising edge
    bra     LOAD_IC2
    movlw   b'00000001'             ;Step 3 BEMF1, IC1 Capture on falling edge
    bra     LOAD_IC1
    movlw   b'00000010'             ;Step 4 BEMF3, IC3 Capture on raising edge
    bra     LOAD_IC3
    movlw   b'00000001'             ;Step 5 BEMF2, IC2 Capture on falling edge
    bra     LOAD_IC2
    movlw   b'00000010'             ;Step 6 BEMF1, IC1 Capture on raising edge
    bra     LOAD_IC1
    goto    ASSERT                  ;Should not be here
LOAD_ALLIC
    movwf   CAP2CON                 ;IC2 Capture
    movwf   CAP3CON                 ;IC3 Capture
    movwf   CAP1CON                 ;IC1 Capture
    movlw   b'00001110'             ;all IC
    clrf    PIR3                    ;clear all IC Int flag
    movwf   PIE3                    ;Enable all IC interrupt
    return
LOAD_IC3
    movwf   CAP3CON                 ;IC3 Capture
    bcf     PIR3,IC3DRIF            ;IC3 Int flag is cleared
    bsf     PIE3,IC3DRIE            ;Enable IC3 interrupt
    return
LOAD_IC2
    movwf   CAP2CON                 ;IC2 Capture
    bcf     PIR3,IC2QEIF            ;IC2 Int flag is cleared
    bsf     PIE3,IC2QEIE            ;Enable IC2 interrupt
    return
LOAD_IC1
    movwf   CAP1CON                 ;IC1 Capture
    bcf     PIR3,IC1IF              ;IC1 Int flag is cleared
    bsf     PIE3,IC1IE              ;Enable IC1 interrupt
    return



    END
