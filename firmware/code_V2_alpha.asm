;*******************************************************************************
;PROGRAM            :   Sensorless BLDC Motor controller
;MICROCONTROLLER    :   PIC18F4431
;*******************************************************************************
;*******************************************************************************
;AUTHOR     :   Laurent Brisedoux
;DATE       :   2012
;Version    :   V2.0
;license    :   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
;               To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter
;               to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
;*******************************************************************************
;Description:
;
;This code implements Sensorless BLDC Motor control using a PIC18F4431 Microcontroller.
;
;     PIN      18: MCLR             17: RC7  > PGD
;              19: AN0              16: RC6  > PGC
;              20: AN1  < T_MOT     15: PWM4 > LIN_C
;              21: CP1  < BEMF_A    14: PWM5 > HIN_C
;              22: CP2  < BEMF_B    11: PWM3 > HIN_B
;              23: CP3  < BEMF_C    10: PWM2 > LIN_B
;              24: AN5  < T_FET     5:  PWM7 > HIN_A
;              25: AN6  < V_IN      4:  PWM6 > LIN_A
;              26: AN7  < V_THR     3:  RD5  > PWR_EN
;              27: AN8  < I_IN      1:  RC7  < Power/eco switch
;              30: OSC1             43: RC5  < Brake/Red switch
;              31: OSC2             42: RC4  < BMS
;              35: CCP2 < RC_THR    41: RD3  > LED3
;              36: RC2  > LED1      40: RD2  > LED2
;              37: RC3  > DBG1     
;
;*******************************************************************************
    include     <p18f4431.inc>
;*******************************************************************************

    CONFIG  OSC = HSPLL, FCMEN = OFF, IESO = OFF, PWRTEN = ON, BOREN = ON, BORV = 42
    CONFIG  WDTEN = OFF, WDPS = 1, T1OSCMX = OFF
    CONFIG  HPOL = HIGH, LPOL = HIGH, PWMPIN = OFF
    CONFIG  MCLRE = ON, STVREN = ON, LVP = OFF, DEBUG = ON
    CONFIG  CP0 = OFF, CP1 = OFF, CPB = OFF, CPD = OFF
    CONFIG  WRT0 = OFF, WRT1 = OFF, WRTB = OFF, WRTC = OFF, WRTD = OFF
    CONFIG  EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF

#define ENABLE_SW_BKPT
#define USE_V_THR
#define USE_AUTO_SYNC_FILTER
#define USE_CENTER_ALIGNED_PWM

;GEN_STATUS bits
#define TMR0_OV_FLAG    0
#define RC_LOCKED_FLAG  1
#define BMC_RUN         2
#define ADC_FAST        3
#define LOGGING_ON      4
#define BRAKE_ON        5
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
#define MAN_PWM         0
#define MAN_DONE        1
#define AUTO_STEP       2
#define AUTO_ADV        3
#define ADV_MAX_FLAG    4
#define STEPPING_ON     5
;#define RESET_FLAG     7

;ADC_STATUS bits
#define ACQ_VIN         0
#define ACQ_VTHR        1
#define ACQ_TFET        2
#define ACQ_TMOT        3
#define UPD_BMC         4

;Step timing, TMR0 is used for motor timing, resolution 800ns (1.25MHz)
#define PERIOD_MAX      0x7FFF                  ;26.2ms => 55 RPM (RPM = 60/(PERIOD*6*7)
#define PERIOD_INIT     (PERIOD_MAX/2)
#define PERIOD_MIN      0x00FF                  ;204us => 7000 RPM
#define PERIOD_DEMAG    (d'500000'/d'800')      ;500us
#define PERIOD_DEBUG2   (PERIOD_DEMAG*2)        ;1400RPM
#define PERIOD_DEBUG    (PERIOD_DEMAG*3/5)      ;4600RPM

;Angles (60 degrees = 256)
#define ADV_CMD_INIT    (d'30'*d'256'/d'60')    ;30 degrees
#define BLANKING_ANGLE  (d'15'*d'256'/d'60')    ;15 degrees

;ZC sync logic
#define ZC_BAD_MAX      d'32'                   ;max nb of bad zc to detect sync/lock lost
#define ZC_VALID_MIN    d'16'                   ;min nb of valid zc to confime sync/lock

;V_THR signal,
#define V_THR_MIN       d'256'
#define V_THR_MAX       d'768'
#define V_THR_SCALE     (0xFFFF/(V_THR_MAX-V_THR_MIN))

;RC signal, TMR1 is used for RC signal timing, resolution 0.8us (1.25MHz)
#define RC_MAX          0x0C35                  ;2.5ms max RC pulse length
#define RC_MIN          0x0271                  ;0.5ms min RC pulse length
#define RC_CENTER       0x0520                  ;1.05ms
#define RC_RANGE        0x0465                  ;0.9ms
#define RC_SCALE        (0xFFFF/RC_RANGE)       ;scale factor
#endif
;RC sync logic
#define RC_BAD_MAX      d'3'                    ;max nb of bad RC pulse to detect sync/lock lost
#define RC_VALID_MIN    d'16'                   ;min nb of valid RC pulse to confime sync/lock

#define WINDMILL_RETRY_INIT d'2'

;PWM resolution 100ns
#define PTPER_INIT      0x00FF                  ;25,6us -> ~40KHz in free running mode, ~20KHz in center aligned
#define PTPER_MAN       (d'35000'/d'100')       ;35us 
#ifdef USE_CENTER_ALIGNED_PWM
#define PWM_DUTY_MIN    0x0                     ;Must be less than 0xFF due to optimization in the code
#define PWM_DUTY_MAX    (4*PTPER_INIT)
#define PWM_DUTY_MIN_REGEN  0x30
#else
#define PWM_DUTY_MIN    0x5                     ;Must be less than 0xFF due to optimization in the code
#define PWM_DUTY_MAX    (4*PTPER_INIT-1)
#define PWM_DUTY_MIN_REGEN  0x30
#endif

;I_CMD
#define THR_CMD_MIN     0x01                    ;below that coast (0 -> brake)
#define THR_MIN_NOLOCK  0x02                    ;Min throttle command to start the motor
#define THR_MAX_NOLOCK  0xFF                    ;100% Max throttle limit until the motor sync is locked
#define THR_AUTO_MAX    0xC0                    ;fix throttle value

;V in
#define KVIN            d'805'
#define V_IN_MAX        (d'3000'*d'1024'/(KVIN*d'5'))
#define V_IN_LOW        (d'2100'*d'1024'/(KVIN*d'5'))
#define V_IN_MIN        (d'2000'*d'1024'/(KVIN*d'5'))

;I in
#define I_ZERO_INIT     0x1FD
#define I_IN_MAX        (d'115'*d'1024'/(d'50'*d'5'))
#define I_IN_MIN        -(d'115'*d'1024'/(d'50'*d'5'))
#define I_CMD_REGEN     -(d'20'*d'1024'/(d'50'*d'5'))
#define I_RGN_LIM       -(d'5'*d'1024'/(d'50'*d'5'))
#define I_MAX           d'80'                   ;80A
#define I_CMD_SCALE     (I_MAX*d'1024'*d'256'/(d'50'*d'5'*d'255'))
#define I_LAG_MAX       (d'150000'/d'800')      ;150us
#define K_I_LAG         (I_LAG_MAX*d'128'/I_IN_MAX)

;Current PI control
#define Kc              (d'120')                ;Propotional gain 
#define Ti              (d'750')                ;Integration time in us
#define K_PIM_0         (Kc+ Kc*50/Ti)
#define K_PIM_1         (Kc)

#define ADC_SEC_CNT_INIT    (d'4')
#define IIN_OC_CNT_MAX      (d'3')

#define T_FET_MAX       ((d'50'*d'10'+d'500')*d'1024'/(d'5000'))

sleep_4ms   macro
            bsf WDTCON,SWDTEN
            sleep
            bcf WDTCON,SWDTEN
            endm

sw_bkpt     macro
#ifdef  ENABLE_SW_BKPT
#ifdef  __MPLAB_DEBUGGER_ICD2   
            call SOFT_BKPT  
#endif
#endif
            endm

poweron     macro
            bsf PORTD,5
            endm
poweroff    macro
            bcf PORTD,5
            endm
led1on      macro   
            bsf PORTC,2
            endm
led1off     macro
            bcf PORTC,2
            endm
led2on      macro   
            bsf PORTD,2
            endm
led2off     macro
            bcf PORTD,2
            endm
led3on      macro   
            bsf PORTD,3
            endm
led3off     macro
            bcf PORTD,3
            endm
debug1on    macro
            bsf PORTC,3
            endm
debug1off   macro
            bcf PORTC,3
            endm
debug2on    macro
            bsf PORTC,4
            endm
debug2off   macro
            bcf PORTC,4
            endm

;*******************************************************************************
;RAM locations in Access bank, uninitialized
;*******************************************************************************
STATUS_SAVE         res 1       ;saved STATUS during Low ISR
WREG_SAVE           res 1       ;saved WREG during Low ISR
PRODH_SAVE          res 1       ;saved PRODH during Low ISR
PRODL_SAVE          res 1       ;saved PRODL during Low ISR
GEN_STATUS          res 1       ;generic status register
ZC_STATUS           res 1       ;Zero Crossing status register
BMC_STATUS          res 1       ;brushless motor controller status register
ADC_STATUS          res 1
PWM_DUTY_H          res 1
PWM_DUTY_L          res 1
PWM_DUTY_S          res 1
PWM_DUTY            res 1
CMD_REQ             res 1
STEP_NEXT           res 1
STEP_CURRENT        res 1
STEP_MAX            res 1
STEP_TEMP           res 1
ZC_VALID_CNT        res 1
ZC_BAD_CNT          res 1
ZC_OLD_H            res 1
ZC_OLD_L            res 1
ZC_NEW_H            res 1
ZC_NEW_L            res 1
I_OLD_H             res 1
I_OLD_L             res 1
ADV_TIME_H          res 1
ADV_TIME_L          res 1
PERIOD_H            res 1
PERIOD_L            res 1
ADV_CMD             res 1
ADCON0_TEMP         res 1
OVDCOND_TEMP        res 1
OVDCONS_TEMP        res 1
PWMCON0_TEMP        res 1
CCPR2_TEMP_H        res 1
CCPR2_TEMP_L        res 1
RC_VALID_CNT        res 1
RC_BAD_CNT          res 1
VAR_TEMP            res 1
WINDMILL_RETRY      res 1
VAR_LISR_H          res 1
VAR_LISR_L          res 1
VAR_LISR            res 1
THR_CMD             res 1
I_CMD_H             res 1
I_CMD_L             res 1
I_ZERO_H            res 1
I_ZERO_L            res 1
V_IN_H              res 1
V_IN_L              res 1
I_IN_H              res 1
I_IN_L              res 1
T_FET_H             res 1
T_FET_L             res 1
T_MOT_H             res 1
T_MOT_L             res 1
V_THR_H             res 1
V_THR_L             res 1
C_ERR_H             res 1
C_ERR_L             res 1
DIV_H               res 1
DIV_L               res 1
ADC_SEC_CNT         res 1
IIN_OC_CNT          res 1
VAR_TEMP_H          res 1
VAR_TEMP_L          res 1
PERM_H              res 1
PERM_L              res 1
PERM_MAX_H          res 1
PERM_MAX_L          res 1
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

SOFT_BKPT
    debug2on
    nop
    nop
    nop
    debug2off   
    return                      ;set a HW breakpoint here

;*******************************************************************************
;*******************************************************************************
;                           MAIN CODE LOOP
;*******************************************************************************
;*******************************************************************************
PROG1   code
START
    call    INIT_PORTS
    call    INIT_VAR
    call    INIT_HSADC
    call    INIT_IC
    call    INIT_TIMING 			; Init Timer0
    call    INIT_PCPWM
    call    INIT_INTERRUPTS
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov

MAIN_START
    call    RESET_INTERRUPTS
    call    RESET_PCPWM
    call    RESET_IC
    call    RESET_TIMING
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
MAIN_IDLE
    btfsc   GEN_STATUS,BMC_RUN      ;run? ---> True if throotle is activated
    bra     MAIN_INIT               ;yes, go on
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     MAIN_IDLE

MAIN_INIT
    call    WINDMILL_FIND
    btfsc   ZC_STATUS,NEW_FLAG      ;windmill found ZC?
    bra     MAIN_BMC                ;yes, start stepping
    call    BMC_STARTUP
    btfsc   ZC_STATUS,NEW_FLAG      ;Startup found ZC?
MAIN_BMC
    call    BMC_STEPPING            ;yes, start stepping
    bra     MAIN_START              ;no re-start everything
    

;*******************************************************************************
;Sensorless stepping loop using ZC
;*******************************************************************************
BMC_STEPPING
    bsf     BMC_STATUS,AUTO_ADV     ;enable auto advance
STEPPING_ADV_LOAD
    btfsc   BMC_STATUS,ADV_MAX_FLAG ;Max advance? 
    bra     STEPPING_SWITCH
    bsf     BMC_STATUS,AUTO_STEP    ;enable auto stepping
    call    LOAD_ADVANCE            ;load timer 0 with advance delay
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
STEPPING_SWITCH
    btfss   BMC_STATUS,STEPPING_ON  ;stepping on?
    bra     STEPPING_NEW
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    movff   I_IN_L,I_OLD_L          ;save the I_IN before stepping
    movff   I_IN_H,I_OLD_H          ;save the I_IN before stepping
    bsf     INTCON,GIEL             ;re-enable low priority interrupts
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
STEPPING_NEW
    bsf     GEN_STATUS,LOGGING_ON
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    call    LOAD_BLANKING           ;load timer 0 with blanking period
    call    UPDATE_TIMING           ;Update period and advance timing
    btfsc   ZC_STATUS,RESET_FLAG
    bra     BMC_STEPPING_RETURN
    movff   STEP_NEXT,STEP_CURRENT
    movlw   6
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT
    incf    STEP_NEXT               ;next step
    call    LOAD_STEP
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    call    STEP_FIND
    cpfseq  STEP_CURRENT            ;early zc?  
    bra     STEPPING_ZC_WAIT        ;no, continue
STEPPING_ZC_EARLY
    movff   TMR5L, ZC_NEW_L         ;early zc, copy TMR5H:TMR5L
    movff   TMR5H, ZC_NEW_H
;   bsf     ZC_STATUS,EARLY_FLAG    ;set zc early flag
    bra     STEPPING_ADV_LOAD
STEPPING_ZC_WAIT
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
;   bsf     BMC_STATUS,AUTO_ADV
    call    LOAD_IC
    call    WAIT_FOR_ZC             ;wait for tmr0 ov or zc
    btfss   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     STEPPING_ZC_LATE        ;no, handle it
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    bra     STEPPING_NEW
STEPPING_ZC_LATE
    movff   TMR5L, ZC_NEW_L         ;no, timout, copy TMR5H:TMR5L
    movff   TMR5H, ZC_NEW_H
    bra     STEPPING_SWITCH         ;ZC is late, switch to next step? 

BMC_STEPPING_RETURN
    return


;*******************************************************************************
;Sensorless startup using permeance measurement
;*******************************************************************************
BMC_STARTUP
    bcf     BMC_STATUS,AUTO_ADV     ;no auto advance
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    bcf     ZC_STATUS,NEW_FLAG
    movlw   HIGH PERIOD_INIT         
    movwf   PERIOD_H
    movlw   LOW PERIOD_INIT
    movwf   PERIOD_L
    call    MAGPOS_FIND
    movff   STEP_CURRENT,STEP_NEXT
    movlw   6
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT               ;step between 1 and 6
    incf    STEP_NEXT               ;next step
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT               ;step between 1 and 6
    incf    STEP_NEXT               ;next step
    call    LOAD_STEP
STARTUP_LOOP
    btfss   GEN_STATUS,BMC_RUN      ;still want to run?
    bra     BMC_STARTUP_FAILED
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    movff   I_IN_L,I_OLD_L          ;save the current value before stepping
    movff   I_IN_H,I_OLD_H          ;save the current value before stepping
    bsf     INTCON,GIEL             ;re-enable low priority interrupts
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
    movff   STEP_NEXT,STEP_TEMP
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    call    MAGPOS_FIND
    movff   STEP_CURRENT,STEP_NEXT
    movlw   6
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT               ;step between 1 and 6
    incf    STEP_NEXT               ;next step
    cpfslt  STEP_NEXT               ;step < 6?
    clrf    STEP_NEXT               ;step between 1 and 6
    incf    STEP_NEXT               ;next step
    call    LOAD_STEP
    movf    STEP_TEMP,W
    subwf   STEP_NEXT,W
    bnn     $+4
    addlw   6                       ;Modulo 6
    movwf   VAR_TEMP
    bz      STARTUP_NOMOV           ; 0 -> hasn't moved
    movlw   4
    cpfsgt  VAR_TEMP
    bra     STARTUP_MOV             ; +1 to +4 steps
STARTUP_NOMOV
    bcf     ZC_STATUS,NEW_FLAG  
    bra     STARTUP_LOOP
STARTUP_MOV
    btfsc   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     STARTUP_COMPLETE
    bsf     ZC_STATUS,NEW_FLAG
    bra     STARTUP_LOOP

STARTUP_COMPLETE
    movlw   b'01011111'
    clrf    OVDCOND                 ;all override
    movwf   PWMCON0                 ;all PWM I/O pins enabled and set to independent mode
    clrf    OVDCONS                 ;all motor PWM HI-Z
    movlw   HIGH PERIOD_MAX      
    movwf   PERIOD_H
    movlw   LOW PERIOD_MAX
    movwf   PERIOD_L
    clrf    STEP_CURRENT
    call    LOAD_DEMAG              ;make sure coils are demagnetized
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    call    LOAD_TIMEOUT            ;reload timer 0 with timeout
    call    LOAD_IC                 ;All IC active
    call    WAIT_FOR_ZC             ;wait for tmr0 ov or zc
    btfss   ZC_STATUS,NEW_FLAG      ;ZC occured?
    bra     BMC_STARTUP             ;no, re-start from the beging
    call    STEP_FIND
    bz      BMC_STARTUP             ;no, re-start from the beging
    movwf   STEP_NEXT
    call    LOAD_STEP
    return

BMC_STARTUP_FAILED
    bcf     ZC_STATUS,NEW_FLAG
    movlw   1
    movwf   STEP_NEXT
    call    LOAD_STEP
    return

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
    btfsc   PIR2,LVDIF              ;Low voltage detected?  
    bra     LVD_INT
    btfsc   PIR2,CCP2IF             ;CCP2 Interrupt? (MUST BE the last test!!!)
    bra     CCP2_RISING
    goto    ASSERT

LVD_INT
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
    movff   I_IN_L,I_OLD_L          ;save the current value before stepping
    movff   I_IN_H,I_OLD_H          ;save the current value before stepping
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
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
    clrf    PIE3                    ;disable all IC interrupt
    clrf    CAP1CON                 ;clear IC1 Capture
    clrf    CAP2CON                 ;clear IC2 Capture
    clrf    CAP3CON                 ;clear IC3 Capture
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

CCP2_RISING
    btfss   CCP2CON,0               ;should be rising edge?
    bra     CCP2_FALLING
    clrf    TMR1H                   ;reset TMR1
    clrf    TMR1L
    bcf     CCP2CON,0               ;Capture on CCP2 falling edge
    bcf     PIR2,CCP2IF             ;clear CCP2 Interrupt flag
CCP2_FALLING
    bcf     IPR2,CCP2IP             ;CCP2 Interrupt low priority
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
    btfsc   PIR2,CCP2IF             ;CCP2 Interrupt?
    bra     CCP2_INT
    goto    ASSERT                  ;Shouldn't be here !!!! -> fatal error
TMR1_INT
    bcf     PIR1,TMR1IF
    bra     ISR_LOW_RETURN
CCP2_INT
    btfsc   CCP2CON,0               ;falling edge?
    bra     CCP2_INT_RIS
    movff   CCPR2L,CCPR2_TEMP_L     ;copy captured TMR1 value for Throttle cmd
    movff   CCPR2H,CCPR2_TEMP_H
    call    UPDATE_RC_CMD
    bsf     CCP2CON,0               ;Capture on CCP2 rising edge
    bcf     PIR2,CCP2IF             ;clear CCP2 Interrupt flag
    bsf     IPR2,CCP2IP             ;CCP2 Interrupt high priority
    bra     ISR_LOW_RETURN
CCP2_INT_RIS
    clrf    TMR1H                   ;reset TMR1
    clrf    TMR1L
    bcf     CCP2CON,0               ;Capture on CCP2 falling edge
    bcf     PIR2,CCP2IF             ;clear CCP2 Interrupt flag
    bcf     IPR2,CCP2IP             ;CCP2 Interrupt low priority
    bra     ISR_LOW_RETURN

ADC_INT
    btfss   GEN_STATUS,ADC_FAST
    bra     ADC_SECONDARY

    ; ADC current signal
    bcf     INTCON,GIEH             ;Disable high priority interrupts
    movff   ADRESH,I_IN_H           ;ADC value is group A / I_IN
    movff   ADRESL,I_IN_L
    bsf     INTCON,GIEH             ;re-enable high priority interrupts
    bcf     PIR1,ADIF               ;clear the ADC interrupt flag
    decfsz  ADC_SEC_CNT
    bra     ADC_FAST_LOOP
    bsf     ADCON2,ACQT0            ;2 Tad acquisition time
    movff   ADCON0_TEMP,ADCON0      ;setup a secondary conversion
    bsf     ADCON0, GO              ;start a new conversion
    bcf     GEN_STATUS,ADC_FAST
ADC_FAST_LOOP
    call    UPDATE_PWM
    btfss   ADC_STATUS,UPD_BMC
    bra     ISR_LOW_RETURN
    call    UPDATE_BMC
    bcf     ADC_STATUS,UPD_BMC
    bra     ISR_LOW_RETURN

ADC_SECONDARY
    call    UPDATE_ADC
    bcf     ADCON2,ACQT0            ;0 Tad acquisition time
    movlw   b'00000001'             ;Group A / I_IN is taken and converted
    movwf   ADCON0
    bsf     GEN_STATUS,ADC_FAST
    bcf     PIR1,ADIF               ;clear the ADC interrupt flag

ISR_LOW_RETURN
    movff   PRODH_SAVE,PRODH
    movff   PRODL_SAVE,PRODL
    movf    WREG_SAVE,W             ;Restore WREG
    movff   STATUS_SAVE,STATUS      ;Restore STATUS
    retfie  

UPDATE_ADC
    btfsc   ADC_STATUS,ACQ_VIN
    bra     ADC_ACQ_VIN
    btfsc   ADC_STATUS,ACQ_VTHR
    bra     ADC_ACQ_VTHR
    btfsc   ADC_STATUS,ACQ_TFET
    bra     ADC_ACQ_TFET
    btfsc   ADC_STATUS,ACQ_TMOT
    bra     ADC_ACQ_TMOT
    goto    ASSERT                  ;Shouldn't be here !!!! -> fatal error

ADC_ACQ_VIN ; ADC battery voltage
    movff   ADRESH,V_IN_H       ;ADC value is group C
    movff   ADRESL,V_IN_L
    movlw   b'00001101'         ;Group D / V_THR is next to be sample and converted
    movwf   ADCON0_TEMP
    movlw   ADC_SEC_CNT_INIT
    movwf   ADC_SEC_CNT         ;next conversions in ADC_SEC_CNT_INIT cycle...
    bcf     ADC_STATUS,ACQ_VIN
    bsf     ADC_STATUS,ACQ_VTHR
    return

ADC_ACQ_VTHR ; ADC throttle signal
    movff   ADRESH,V_THR_H      ;ADC value is group D
    movff   ADRESL,V_THR_L
    movlw   b'00000101'         ;Group B / T_FET is taken and converted
    movwf   ADCON0_TEMP
    bsf     ADCHS,GBSEL0        ;Group B signal is AN5 -> T_FET
    movlw   ADC_SEC_CNT_INIT
    movwf   ADC_SEC_CNT         ;next conversions in ADC_SEC_CNT_INIT cycle...
    bcf     ADC_STATUS,ACQ_VTHR
    bsf     ADC_STATUS,ACQ_TFET
    return

ADC_ACQ_TFET ; ADC fets temperature
    movff   ADRESH,T_FET_H      ;ADC value is group B
    movff   ADRESL,T_FET_L
    bcf     ADCHS,GBSEL0        ;Group B signal is AN1 -> T_MOT
    movlw   ADC_SEC_CNT_INIT
    movwf   ADC_SEC_CNT         ;next conversions in ADC_SEC_CNT_INIT cycle...
    bcf     ADC_STATUS,ACQ_TFET
    bsf     ADC_STATUS,ACQ_TMOT
    return

ADC_ACQ_TMOT ; ADC for what ??
    movff   ADRESH,T_MOT_H      ;ADC value is group D
    movff   ADRESL,T_MOT_L
    movlw   b'00001001'         ;Group C / V_IN is taken and converted
    movwf   ADCON0_TEMP
    movlw   ADC_SEC_CNT_INIT
    movwf   ADC_SEC_CNT         ;next conversions in ADC_SEC_CNT_INIT cycle...
    bsf     ADC_STATUS,UPD_BMC
    bcf     ADC_STATUS,ACQ_TMOT
    bsf     ADC_STATUS,ACQ_VIN
    return

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
    movlw   RC_BAD_MAX          ;reset the bad RCcounter 
    movwf   RC_BAD_CNT
    movlw   RC_VALID_MIN        ;reset the valid RC counter 
    movwf   RC_VALID_CNT
    movlw   HIGH I_ZERO_INIT
    movwf   I_ZERO_H
    movlw   LOW I_ZERO_INIT
    movwf   I_ZERO_L

    return


;*******************************************************************************
;Initialize High-Speed ADC
;*******************************************************************************
INIT_HSADC
    movlw   b'11100010'         ; ANSEL0 is configured such that:
    movwf   ANSEL0              ; a) AN1, AN5, AN6, AN7 are analog input pins.

    movlw   b'00000001'         ; ANSEL1 is configured such that:
    movwf   ANSEL1              ; AN8 is analog input pins.

    movlw   b'00010000'         ; ADCON1 is configured such that:
    movwf   ADCON1              ; a) Vref+ and Vref- are Avdd and Avss, respectively.
                                ; b) The FIFO buffer is enable

    movlw   b'10001101'         ; ADCON2 is configured such that:
    movwf   ADCON2              ; a) The A/D result is right justified (read ADRESH before reading ADRESL)
                                ; b) The A/D acquisition time is set to 2 Tad.
                                ; c) The A/D conversion clock is set to Fosc/16.

    movlw   b'00010000'         ; PWM event start AD sequence
    movwf   ADCON3

    movlw   b'01010110'         ; ADCHS is configured such that:
    movwf   ADCHS               ; a) Group A signal is AN8
                                ; b) Group B signal is AN5
                                ; c) Group C signal is AN6
                                ; d) Group D signal is AN7

    movlw   b'00000001'         ; ADCON0 is configured such that:
    movwf   ADCON0              ; a) Single shot mode is enabled
                                ; b) Single Channel mode is enabled
                                ; c) Group A / I_IN is taken and converted
                                ; d) The ADC is turned on.

    movlw   b'0011101'
    movwf   LVDCON              ; Low-Voltage Detection enable, 3.93V - 4.62V Limits 

    sleep_4ms

    bsf     ADCON2,ACQT0        ;2 Tad acquisition time

    movlw   b'00001001'         ;Group C / V_IN is taken and converted
    movwf   ADCON0
    bsf     ADCON0, GO          ;start a new conversion to init variable
    btfsc   ADCON0, GO
    bra     $-2 
    movff   ADRESH,V_IN_H       ;ADC value is group C
    movff   ADRESL,V_IN_L

    movlw   b'00001101'         ;Group D / V_THR is taken and converted
    movwf   ADCON0
    bsf     ADCON0, GO          ;start a new conversion to init variable
    btfsc   ADCON0, GO
    bra     $-2 
    movff   ADRESH,V_THR_H      ;ADC value is group D
    movff   ADRESL,V_THR_L

    movlw   b'00000101'         ;Group B / T_XXX is taken and converted
    movwf   ADCON0
    bsf     ADCHS,GBSEL0        ;Group B signal is AN5 -> T_FET
    bsf     ADCON0, GO          ;start a new conversion to init variable
    btfsc   ADCON0, GO
    bra     $-2 
    movff   ADRESH,T_FET_H      ;ADC value is group B
    movff   ADRESL,T_FET_L

    bcf     ADCHS,GBSEL0        ;Group B signal is AN1 -> T_MOT
    bsf     ADCON0, GO          ;start a new conversion to init variable
    btfsc   ADCON0, GO
    bra     $-2 
    movff   ADRESH,T_MOT_H      ;ADC value is group B
    movff   ADRESL,T_MOT_L

    movlw   b'00000001'         ;Group A / I_IN is taken and converted
    movwf   ADCON0
    bsf     ADCON0, GO          ;start a new conversion to init variable
    btfsc   ADCON0, GO
    bra     $-2 
    movff   ADRESH,I_IN_H     ;ADC value is group A
    movff   ADRESL,I_IN_L

    movlw   ADC_SEC_CNT_INIT
    movwf   ADC_SEC_CNT
    movlw   b'00001001'             ;Group C / V_IN is taken and converted
    movwf   ADCON0_TEMP

    bcf     ADCON2,ACQT0        ;0 Tad acquisition time
    bsf     GEN_STATUS,ADC_FAST
    bsf     ADC_STATUS,ACQ_VIN
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
    movlw   b'01011111'         ; PWMCON0 is configured such that:
    movwf   PWMCON0             ; a) All PWM I/O pins enabled for PWM output.
                                ; b) All PWM I/O pairs are set to independant mode
                                        
    movlw   b'00000000'         ;PWMCON1 is configured such that:
    movwf   PWMCON1             ; a) Special event trigger post-scaler is set to 1:1
                                ; b) Special event trigger occurs when time-base is counting upwards
                                ; c) Updates from duty cycle and period buffer registers are enabled.
                                ; d) Output overrides via OVDCON are not synchronous to the PWM timebase.           

    movlw   b'00000110'
    movwf   DTCON               ; 300ns dead-time                           


;   movlw   CMD_REQ_PERIOD_MAX  ;reset CMD_REQ_PERIOD
;   movwf   CMD_REQ_PERIOD

RESET_PCPWM
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    movlw   b'01011111'
    clrf    OVDCOND                 ;all override
    movwf   PWMCON0                 ;all PWM I/O pins enabled and set to independent mode
    clrf    OVDCONS                 ;all motor PWM HI-Z
#ifdef USE_CENTER_ALIGNED_PWM
    movlw   b'00000010'             ; PTCON0 is configured such that:
    movwf   PTCON0                  ; a) PWM postscale value is 1:1
                                    ; b) PWM time base input is Fosc/4
                                    ; c) Continuous Up/Down Counting mode
#else
    movlw   b'00000000'             ; PTCON0 is configured such that:
    movwf   PTCON0                  ; a) PWM postscale value is 1:1
                                    ; b) PWM time base input is Fosc/4
                                    ; c) Free Running mode
#endif
    movlw   LOW PTPER_INIT          ; PTPERL and PTPERH are set up.     
    movwf   PTPERL 
    movlw   HIGH PTPER_INIT
    movwf   PTPERH          
    movlw   0x01
    movwf   PWM_DUTY
    clrf    PWM_DUTY_H
    movlw   PWM_DUTY_MIN
    movwf   PWM_DUTY_L
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    movwf   PDC3L
    movwf   PDC1L
    movwf   PDC2L
    clrf    PDC3H
    clrf    PDC1H
    clrf    PDC2H
    movff   PWM_DUTY,SEVTCMPL
    clrf    SEVTCMPH
    bcf     PWMCON1, UDIS
#ifdef  __MPLAB_DEBUGGER_ICD2   
    movlw   b'10001000'         ;FLT 
#else
    movlw   b'00001000'         ;FLT 
#endif
    movwf   FLTCONFIG           ; a) reset and enable fault condition on break-point for use with ICD2
    bsf     PTCON1, PTEN            ;PTEN bit in the PTCON1 is set to enable the PWM time base.
    bsf     INTCON,GIEL             ;enable low priority interrupts
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

    movlw   b'01011001'         ; T5CON is configured such that:
    movwf   T5CON               ; a) Special event reset is disabled                
                                ; b) Continuous count mode is enabled
                                ; c) Timer5 input clock prescaler is 1:8
                                ; e) Timer5 is enabled

;   clrf    DFLTCON             ; Digital filter disable
    movlw   b'00111011'         ; Digital filter is configured
    movwf   DFLTCON             ; a) IC1,2&3 filter enabled
                                ; b) noise filter clock divider = 1:16

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

    clrf    PORTB
    movlw   b'11000011'         ; PWM output set to GND.
    movwf   TRISB

    clrf    PORTC
    movlw   b'11100011'         ; RC2, RC3, RC4
    movwf   TRISC

    clrf    PORTD
    movlw   b'00010011'         ; RD2, RD3, RD5, RD6/PWM6, RD7PWM7
    movwf   TRISD

    poweron

    return

;*******************************************************************************
;Initialize Timings
;*******************************************************************************
INIT_TIMING
    movlw   b'10000010'         ;T0CON is configured such that:
    movwf   T0CON               ;a) TMR0 ON, 
                                ;b) 16-bit operation, 
                                ;c) Timer0 clock input comes from prescaler output, 
                                ;d) prescalar is 1:8

    movlw   b'10110001'         ;T1CON is configured such that:
    movwf   T1CON               ;a) 16-bit operation 
                                ;b) prescalar is 1:8
                                ;c) clock source is instruction cycle clock 
                                ;d) TMR1 ON


RESET_TIMING
    movlw   HIGH PERIOD_MAX      
    movwf   PERIOD_H
    movlw   LOW PERIOD_MAX
    movwf   PERIOD_L

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
    bcf     IPR1,TMR1IP             ;Timer1 overflow Interrupt low priority

    bsf     IPR3,IC1IP              ;IC1 Interrupt high priority
    bsf     IPR3,IC2QEIP            ;IC2 Interrupt high priority
    bsf     IPR3,IC3DRIP            ;IC3 Interrupt high priority

    bcf     IPR1,ADIP               ;AD Converter Interrupt low priority
    bsf     PIE1,ADIE               ;AD Converter Interrupt enable

    bsf     IPR2,LVDIP              ;LVD Interrupt high priority
    bsf     PIE2,LVDIE              ;LVD Interrupt enable

    movlw   b'10010011'             ;Power ON reset status bit/Brownout reset status bit
    movwf   RCON                    ;and Instruction flag bits are set
                                    ;Enable Priority levels on Interrupts
    
    bsf     INTCON,GIEL             ;Enable low priority interrupts
    bsf     INTCON,GIEH             ;Enable high priority interrupts


RESET_INTERRUPTS
    bsf     PIE1,TMR1IE             ;Timer1 overflow Interrupt enable
    bcf     INTCON,TMR0IE           ;Timer0 overflow Interrupt disable
    clrf    PIE3                    ;IC Interrupt disable
    return

;*******************************************************************************
;Try to synchronize onto the ZC using the Windmill strategy
;*******************************************************************************
WINDMILL_FIND
    movlw   b'01011111'
    clrf    OVDCOND                 ;all override
    movwf   PWMCON0                 ;all PWM I/O pins enabled and set to independent mode
    clrf    OVDCONS                 ;all motor PWM HI-Z
    clrf    STEP_CURRENT            ;step 0
    clrf    STEP_NEXT               ;step 0
    bcf     BMC_STATUS,AUTO_ADV     ;no auto advance
    bcf     BMC_STATUS,AUTO_STEP    ;no auto stepping
    movlw   WINDMILL_RETRY_INIT
    movwf   WINDMILL_RETRY
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
    call    UPDATE_ADV              ;Update advance timing
    call    LOAD_STEP
    return
WINDMILL_FAILED
    bcf     ZC_STATUS,NEW_FLAG
    movlw   1
    movwf   STEP_NEXT
    call    LOAD_STEP
    call    UPDATE_ADV
    return

;*******************************************************************************
;search the magnetic postion of the rotor
;*******************************************************************************
MAGPOS_FIND
    bcf     INTCON,GIEL             ;Disable low priority interrupts
    clrf    OVDCOND                 ;switch to the next step
    movlw   b'01011111'
    clrf    OVDCOND                 ;all override
    movwf   PWMCON0                 ;all PWM I/O pins enabled and set to independent mode
    clrf    OVDCONS                 ;all motor PWM HI-Z
    bcf     BMC_STATUS,MAN_DONE
    bsf     BMC_STATUS,MAN_PWM
    call    LOAD_DEMAG              ;make sure coils are demagnetized
    bsf     INTCON,GIEL             ;Re-Enable low priority interrupts
    btfss   BMC_STATUS,MAN_DONE
    bra     $-2                     ;wait for the PWM control to be in manual mode
    bcf     PTCON1,PTEN             ;disable PWM
#ifdef USE_CENTER_ALIGNED_PWM
    bcf     PTCON0,PTMOD1           ;disable center aligned mode
#endif
    bsf     PTCON0,PTMOD0           ;enable Single-shot mode
    setf    PDC3L
    setf    PDC3H
    setf    PDC1L
    setf    PDC1H
    setf    PDC2L
    setf    PDC2H
    clrf    PTMRH
    clrf    PTMRL
    clrf    PERM_MAX_H
    clrf    PERM_MAX_L
    movlw   LOW PTPER_MAN           ; PTPERL and PTPERH are set up.     
    movwf   PTPERL 
    movwf   SEVTCMPL
    movlw   HIGH PTPER_MAN
    movwf   PTPERH          
    movwf   SEVTCMPH
    call    WAIT_FOR_TMR0           ;wait for tmr0 ov
    movlw   6
    movwf   STEP_NEXT
    movwf   STEP_CURRENT
    call    LOAD_STEP
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
    bcf     BMC_STATUS,MAN_DONE
    bsf     PTCON1,PTEN             ;enable a single shot PWM
    btfsc   PTCON1,PTEN             ;wait until the end of the single shot
    bra     $-2
    movlw   3
    movwf   STEP_NEXT
    call    LOAD_STEP
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
    bsf     PTCON1,PTEN             ;enable a single shot PWM
    movlw   1
    movwf   STEP_NEXT
    call    LOAD_STEP
    clrf    VAR_TEMP_H
    clrf    VAR_TEMP_L
    btfss   BMC_STATUS,MAN_DONE
    bra     $-2                     ;wait for the previous ADC conversion to finish
    bcf     BMC_STATUS,MAN_DONE
    btfsc   PTCON1,PTEN             ;wait until the end of the single shot
    bra     $-2

MAGPOS_LOOP
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
    bsf     PTCON1,PTEN             ;enable a single shot PWM
    btfss   BMC_STATUS,MAN_DONE
    bra     $-2                     ;wait for the current ADC conversion to finish
    bcf     BMC_STATUS,MAN_DONE
    call    CAL_PERM
    movff   STEP_NEXT,STEP_CURRENT
    movlw   6                       ;+3 modulo 6
    cpfslt  STEP_NEXT
    clrf    STEP_NEXT
    incf    STEP_NEXT
    cpfslt  STEP_NEXT
    clrf    STEP_NEXT
    incf    STEP_NEXT
    cpfslt  STEP_NEXT
    clrf    STEP_NEXT
    incf    STEP_NEXT
    call    LOAD_STEP
    btfsc   PTCON1,PTEN             ;wait until the end of the single shot
    bra     $-2
    clrf    OVDCOND                 ;switch to the next step
    movff   PWMCON0_TEMP,PWMCON0
    movff   OVDCONS_TEMP,OVDCONS
    movff   OVDCOND_TEMP,OVDCOND
    bsf     PTCON1,PTEN             ;enable a single shot PWM
    btfss   BMC_STATUS,MAN_DONE
    bra     $-2                     ;wait for the previous ADC conversion to finish
    movff   PERM_H, VAR_TEMP_H
    movff   PERM_L, VAR_TEMP_L
    bcf     BMC_STATUS,MAN_DONE
    movff   STEP_CURRENT,STEP_NEXT 
    movlw   6                       ;+1 modulo 6
    cpfslt  STEP_NEXT
    clrf    STEP_NEXT
    incf    STEP_NEXT
    call    LOAD_STEP
    btfsc   PTCON1,PTEN             ;wait until the end of the single shot
    bra     $-2

    movlw   1
    cpfseq  STEP_NEXT               ;1st step?
    bra     MAGPOS_LOOP             ;no -> loop then

    btfss   BMC_STATUS,MAN_DONE
    bra     $-2                     ;wait for the current ADC conversion to finish
    call    CAL_PERM
    call    RESET_PCPWM

MAGPOS_RETURN
    movff   STEP_MAX,STEP_CURRENT
    bcf     BMC_STATUS,MAN_PWM
    return

CAL_PERM
    movf    PERM_L,W
    subwf   VAR_TEMP_L
    movf    PERM_H,W
    subwfb  VAR_TEMP_H
    bnn     PERM_CHECK              ;ABS(PERM), not negative?
    comf    VAR_TEMP_H,F            ;Complement all bytes
    comf    VAR_TEMP_L,F
    infsnz  VAR_TEMP_L,F            ;Inc. low byte and Skip if no carry to higher bytes
    incf    VAR_TEMP_H,F
PERM_CHECK

PERM_CHECK_MAX                      ;Test to see if VAR_TEMP > PERM_MAX
    movf    PERM_MAX_H,W
    cpfsgt  VAR_TEMP_H              ;VAR_TEMP_H > PERM_MAX_H ?
    bra     PERM_CHECK_MAX_HEQ      ;no, so check =
    bra     PERM_FOUND_MAX          ;yes, new max
PERM_CHECK_MAX_HEQ
    cpfseq  VAR_TEMP_H              ;VAR_TEMP_H = PERM_MAX ?
    bra     PERM_CHECK_RETURN       ;no, so VAR_TEMP_H < PERM_MAX, no new max
    movf    PERM_MAX_L,W        
    cpfsgt  VAR_TEMP_L              ;so, VAR_TEMP_L > PERM_MAX_L ?
    bra     PERM_CHECK_RETURN       ;no, no new max
PERM_FOUND_MAX
    movff   VAR_TEMP_H, PERM_MAX_H
    movff   VAR_TEMP_L, PERM_MAX_L
    movff   STEP_CURRENT,STEP_MAX 
PERM_CHECK_RETURN
    return


;********************************************************************
; Waiting loops
;********************************************************************
WAIT_FOR_TMR0
    btfss   GEN_STATUS,TMR0_OV_FLAG ;Timer0 overflow ocured?
    bra     WAIT_FOR_TMR0           ;No
    return

WAIT_FOR_ZC
    btfsc   ZC_STATUS,NEW_FLAG      ;ZC occured?
    return
    btfss   GEN_STATUS,TMR0_OV_FLAG ;Timer0 overflow ocured?
    bra     WAIT_FOR_ZC             ;Not yet, loop ...
WAIT_TIMEOUT
    clrf    PIE3                    ;disable all IC interrupt
    clrf    CAP2CON                 ;clear IC2 Capture
    clrf    CAP3CON                 ;clear IC3 Capture
    clrf    CAP1CON                 ;clear IC1 Capture
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
    cpfsgt  CCPR2_TEMP_H            ;CCPR2_TEMP_H > HIGH RC_MAX ?
    bra     RC_MAX_HEQ              ;no, so check =
    bra     UPDATE_RC_BAD           ;yes, pulse too long
RC_MAX_HEQ
    cpfseq  CCPR2_TEMP_H            ;CCPR2_TEMP_H = HIGH RC_MAX ?
    bra     RC_MIN_CHECK            ;no, so CCPR2_TEMP_H < HIGH RC_MAX, pulse not too long
    movlw   LOW RC_MAX      
    cpfsgt  CCPR2_TEMP_L            ;so, CCPR2_TEMP_L > LOW RC_MAX ?
    bra     RC_MIN_CHECK            ;no, pulse not too long
    bra     UPDATE_RC_BAD           ;yes, pulse too long
RC_MIN_CHECK
    movlw   HIGH RC_MIN     
    cpfslt  CCPR2_TEMP_H            ;CCPR2_TEMP_H < HIGH RC_MIN ?
    bra     RC_MIN_HEQ              ;no, so check =
    bra     UPDATE_RC_BAD           ;yes, pulse too short
RC_MIN_HEQ
    cpfseq  CCPR2_TEMP_H            ;CCPR2_TEMP_H = HIGH RC_MIN ?
    bra     UPDATE_RC_OK            ;no, so CCPR2_TEMP_H > HIGH RC_MIN, pulse not too short
    movlw   LOW RC_MIN      
    cpfslt  CCPR2_TEMP_L            ;so, CCPR2_TEMP_L < LOW RC_MIN ?
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
    decfsz  RC_VALID_CNT            ;enough consecutive RC for a lock?
    return                          ;not yet
    bsf     GEN_STATUS,RC_LOCKED_FLAG   ;RC locked!!!!!
    movlw   RC_VALID_MIN            ;reset RC valid counter 
    movwf   RC_VALID_CNT
    movlw   RC_BAD_MAX              ;reset RC bad counter 
    movwf   RC_BAD_CNT
UPDATE_THR_CMD
    movlw   LOW RC_CENTER           ;VAR_LISR = CCPR2_TEMP - RC_CENTER (16bit sub)
    subwf   CCPR2_TEMP_L,W
    movwf   VAR_LISR_L
    movlw   HIGH RC_CENTER
    subwfb  CCPR2_TEMP_H,W
    movwf   VAR_LISR_H
    bnc     UPDATE_THR_CMD_ZERO     ;borrow (no carry)?
    movlw   RC_SCALE
    mulwf   VAR_LISR_H              ;RC_SCALExVAR_LISR_H -> VAR_LISR_H:VAR_LISR
    movff   PRODH,VAR_LISR_H        ;Copy high product in VAR_LISR_H
    movff   PRODL,VAR_LISR          ;Copy low product in VAR_LISR
    mulwf   VAR_LISR_L              ;RC_SCALExVAR_LISR_L -> VAR_LISR_L:-
    movff   PRODH,VAR_LISR_L
    movf    VAR_LISR,W
    addwf   VAR_LISR_L,F            ;add the 2 product
    clrf    WREG
    addwfc  VAR_LISR_H,F
    bz      UPDATE_THR_CMD_RET      ;
    setf    VAR_LISR_L              ;MSB not zero -> clip
UPDATE_THR_CMD_RET
    movff   VAR_LISR_L,THR_CMD
    return
UPDATE_THR_CMD_ZERO
    clrf    VAR_LISR_H
    clrf    VAR_LISR_L
    bra     UPDATE_THR_CMD_RET

UPDATE_RC_BAD
    btfss   GEN_STATUS,RC_LOCKED_FLAG   ;RC already locked?
    bra     UPDATE_RC_BAD_INIT      ;no, init RC locking
    decfsz  RC_VALID_CNT            ;no -> decrease RC balance
    return                          ;balance not zero, keep going
    bcf     GEN_STATUS,RC_LOCKED_FLAG   ;RC sync lost
    clrf    THR_CMD                 ;too many bad RC pulse, coast
    bsf     THR_CMD,0
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
    clrf    THR_CMD                 ;too many bad RC pulse, coast
    bsf     THR_CMD,0 
    movlw   RC_BAD_MAX              ;reset RC bad counter 
    movwf   RC_BAD_CNT
    movlw   RC_VALID_MIN            ;reset RC valid counter 
    movwf   RC_VALID_CNT
    return


;********************************************************************
; Update the PWM duty based on current command I_CMD
;********************************************************************
UPDATE_PWM
    movf    I_IN_L,W                ;I_IN = I_ZERO - I_IN (16bit sub)
    subwf   I_ZERO_L,W
    movwf   I_IN_L
    movf    I_IN_H,W
    subwfb  I_ZERO_H,W
    movwf   I_IN_H
    bnn     I_IN_MAX_CHECK
I_IN_MIN_CHECK
    movlw   HIGH I_IN_MIN
    cpfslt  I_IN_H                  ;I_IN_H < HIGH I_IN_MIN ?
    bra     I_IN_MIN_HEQ            ;no, so check =
    bra     UPDATE_IIN_OC           ;yes, IIN too low
I_IN_MIN_HEQ
    cpfseq  I_IN_H                  ;I_IN_H = HIGH I_IN_MIN ?
    bra     UPDATE_IIN_NOC          ;no, so I_IN_H > HIGH I_IN_MIN, IIN ok
    movlw   LOW I_IN_MIN
    cpfsgt  I_IN_L                  ;so, I_IN_L > LOW I_IN_MIN ?
    bra     UPDATE_IIN_OC           ;no, IIN too low
    bra     UPDATE_IIN_NOC
I_IN_MAX_CHECK
    movlw   HIGH I_IN_MAX       
    cpfsgt  I_IN_H                  ;I_IN_H > HIGH I_IN_MAX ?
    bra     I_IN_MAX_HEQ            ;no, so check =
    bra     UPDATE_IIN_OC           ;yes, IIN too high
I_IN_MAX_HEQ
    cpfseq  I_IN_H                  ;I_IN_H = HIGH I_IN_MAX ?
    bra     UPDATE_IIN_NOC          ;no, so I_IN_H < HIGH I_IN_MAX, IIN ok
    movlw   LOW I_IN_MAX
    cpfslt  I_IN_L                  ;so, I_IN_L < LOW I_IN_MAX ?
    bra     UPDATE_IIN_OC           ;no, IIN too high
;   bra     UPDATE_IIN_NOC

UPDATE_IIN_NOC
    clrf    IIN_OC_CNT
;   movf    IIN_OC_CNT
;   bz      UPDATE_PWM_INIT
;   decf    IIN_OC_CNT
    bra     UPDATE_PWM_INIT

;Critical Over Current, hold BMC in reset if it last for too long
UPDATE_IIN_OC
;   sw_bkpt
    incf    IIN_OC_CNT
    movlw   IIN_OC_CNT_MAX
    cpfsgt  IIN_OC_CNT              ;IIN_OC_CNT>IIN_OC_CNT_MAX?
    bra     UPDATE_PWM_INIT
    sw_bkpt
;   debug2on
    clrf    I_CMD_H
    clrf    I_CMD_L
    bra     UPDATE_PWM_RESET        ;reset

UPDATE_PWM_INIT
    btfss   BMC_STATUS,MAN_PWM      ;Manual PWM control?
    bra     UPDATE_PWM_RUN          ;no, normal mode, go on
    movff   I_IN_H, PERM_H
    movff   I_IN_L, PERM_L
    bsf     BMC_STATUS,MAN_DONE
    return

UPDATE_PWM_RUN
    movf    I_CMD_L,W               ;check for Z
    bnz     UPDATE_PWM_PID          ;not Z
    movf    I_CMD_H,W               ;check for Z
    bz      UPDATE_PWM_RESET        ;Zero too? reset

UPDATE_PWM_PID
    movf    OVDCOND,W               ;check for active state
    bz      UPDATE_PWM_MIN
#ifdef USE_DUTY_CONTROL
    rlcf    I_CMD_L,W
    movwf   PWM_DUTY_L
    rlcf    I_CMD_H,W
    movwf   PWM_DUTY_H
    bz      CLIP_PWM_DUTY_MIN       ;if PWM_DUTY_H=0 it can't be above 0xFF
    bra     CLIP_PWM_DUTY_MAX
#endif
    movlw   K_PIM_1                 ;e(i-1) factor
    mulwf   C_ERR_H                 ;K_PIM_1xC_ERR_H -> VAR_LISR_H:VAR_LISR_L
    movff   PRODH,VAR_LISR_H        ;Copy high product in  VAR_LISR_H
    movff   PRODL,VAR_LISR_L        ;Copy low product in VAR_LISR_L
    mulwf   C_ERR_L                 ;K_PIM_1xC_ERR_L -> PRODH:PRODL
    btfsc   C_ERR_H,7               ;test sign bit
    subwf   VAR_LISR_H,F
    movf    PRODH,W                 ;Copy high product in W
    addwf   VAR_LISR_L,F            ;Add intermediate products
    clrf    WREG
    addwfc  VAR_LISR_H,F
    movf    PRODL,W                 ;substract K1*e(i-1) correction to PWM_DUTY
    subwf   PWM_DUTY_S,F
    movf    VAR_LISR_L,W
    subwfb  PWM_DUTY_L,F
    movf    VAR_LISR_H,W
    subwfb  PWM_DUTY_H,F
    movf    I_IN_L,W                ;C_ERR =  I_CMD - I_IN(16bit sub)
    subwf   I_CMD_L,W
    movwf   C_ERR_L
    movf    I_IN_H,W        
    subwfb  I_CMD_H,W
    movwf   C_ERR_H                 ;signed error -> C_ERR
    movlw   K_PIM_0                 ;e(i-0) factor
    mulwf   C_ERR_H                 ;K_PIM_0xC_ERR_H -> VAR_LISR_H:VAR_LISR_L
    movff   PRODH,VAR_LISR_H        ;Copy high product in  VAR_LISR_H
    movff   PRODL,VAR_LISR_L        ;Copy low product in VAR_LISR_L
    mulwf   C_ERR_L                 ;K_PIM_0xC_ERR_L -> PRODH:PRODL
    btfsc   C_ERR_H,7               ;test sign bit
    subwf   VAR_LISR_H,F
    movf    PRODH,W                 ;Copy high product in W
    addwf   VAR_LISR_L,F            ;Add intermediate products
    clrf    WREG
    addwfc  VAR_LISR_H,F
    movf    PRODL,W                 ;Add e(i-0) correction to PWM_DUTY
    addwf   PWM_DUTY_S,F
    movf    VAR_LISR_L,W
    addwfc  PWM_DUTY_L,F
    movf    VAR_LISR_H,W
    addwfc  PWM_DUTY_H,F
    bn      UPDATE_PWM_MIN          ;negative DUTY not supported
    bz      CLIP_PWM_DUTY_MIN       ;if PWM_DUTY_H=0 it can't be above 0xFF

CLIP_PWM_DUTY_MAX                   ;Test to see if PWM_DUTY > PWM_DUTY_MAX (PWM_DUTY_H is not null)
    movlw   HIGH PWM_DUTY_MAX
    cpfsgt  PWM_DUTY_H              ;PWM_DUTY_H > HIGH PWM_DUTY_MAX ?
    bra     CLIP_PWM_DUTY_MAX_HEQ   ;no, so check =
    bra     UPDATE_PWM_MAX          ;yes, clip it to max
CLIP_PWM_DUTY_MAX_HEQ
    cpfseq  PWM_DUTY_H              ;PWM_DUTY_H = HIGH PWM_DUTY_MAX ?
    bra     UPDATE_PWM_GO           ;no, so PWM_DUTY_H < HIGH PWM_DUTY_MAX, no clipping needed
    movlw   LOW PWM_DUTY_MAX        
    cpfsgt  PWM_DUTY_L              ;so, PWM_DUTY_L > LOW PWM_DUTY_MAX ?
    bra     UPDATE_PWM_GO           ;no, no clipping needed
;   bra     UPDATE_PWM_MAX          ;yes, clip it to max

#ifdef USE_CENTER_ALIGNED_PWM
UPDATE_PWM_MAX
    movlw   LOW PWM_DUTY_MAX
    movwf   PWM_DUTY_L
    movlw   HIGH PWM_DUTY_MAX
    movwf   PWM_DUTY_H
    bra     UPDATE_PWM_GO
#else
UPDATE_PWM_MAX
    movlw   LOW PWM_DUTY_MAX
    movwf   PWM_DUTY_L
    movlw   HIGH PWM_DUTY_MAX
    movwf   PWM_DUTY_H
    rrcf    PWM_DUTY_H,W            ;/2
    movwf   VAR_LISR_H
    rrcf    PWM_DUTY_L,W
    movwf   PWM_DUTY
    rrcf    VAR_LISR_H,F            ;/2
    rrcf    PWM_DUTY,F
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    setf    PDC3L
    setf    PDC3H
    setf    PDC1L
    setf    PDC1H
    setf    PDC2L
    setf    PDC2H
    movlw   LOW PTPER_INIT          ;SEVTCMP is set to PTPER_INIT
    movwf   SEVTCMPL
    movlw   HIGH PTPER_INIT
    movwf   SEVTCMPH
    bcf     PWMCON1, UDIS           ;Enable updates to duty cycle and period
;#ifdef __MPLAB_DEBUGGER_ICD2   
;   movlw   b'10001000'         ;FLT 
;#else
;   movlw   b'00001000'         ;FLT 
;#endif
;   movwf   FLTCONFIG           ;reset and enable fault condition on break-point for use with ICD2
    return
#endif

CLIP_PWM_DUTY_MIN                   ;Test to see if PWM_DUTY < PWM_DUTY_MIN (PWM_DUTY_H is null)
    movlw   PWM_DUTY_MIN
    btfsc   GEN_STATUS,BRAKE_ON
;   sw_bkpt
    movlw   PWM_DUTY_MIN_REGEN
    cpfslt  PWM_DUTY_L              ;so, PWM_DUTY_L < PWM_DUTY_MIN(_REGEN) ?
    bra     UPDATE_PWM_GO           ;no, no clipping needed
    bra     UPDATE_PWM_MIN          ;yes, clip it to min

UPDATE_PWM_RESET
    movlw   b'01011111'
    clrf    OVDCOND
    movwf   PWMCON0                 ;all PWM I/O pins enabled and set to independent mode
    clrf    OVDCONS
UPDATE_PWM_MIN
    movlw   PWM_DUTY_MIN
    btfsc   GEN_STATUS,BRAKE_ON
    movlw   PWM_DUTY_MIN_REGEN
    movwf   PWM_DUTY_L              ;carry -> clip to zero
    clrf    PWM_DUTY_H
UPDATE_PWM_GO
    rrcf    PWM_DUTY_H,W            ;/2
    movwf   VAR_LISR_H
    rrcf    PWM_DUTY_L,W
    movwf   PWM_DUTY
    rrcf    VAR_LISR_H,F            ;/2
    rrcf    PWM_DUTY,F
    bnz     $+4
    incf    PWM_DUTY                ;don't let it be zero
    movf    PWM_DUTY_L,W
    bsf     PWMCON1, UDIS           ;Disable updates to duty cycle and period
    movwf   PDC3L
    movwf   PDC1L
    movwf   PDC2L
    movf    PWM_DUTY_H,W
    movwf   PDC3H
    movwf   PDC1H
    movwf   PDC2H
    movff   PWM_DUTY,SEVTCMPL
    bcf     PWMCON1, UDIS
#ifdef  __MPLAB_DEBUGGER_ICD2   
    movlw   b'10001000'         ;FLT 
#else
    movlw   b'00001000'         ;FLT 
#endif
    movwf   FLTCONFIG           ;reset and enable fault condition on break-point for use with ICD2
    return



;********************************************************************
; Update the BMC state based on THR, V_IN and T
;********************************************************************
UPDATE_BMC

    led1off
    led2off
    led3off

#ifdef USE_V_THR
    movlw   LOW V_THR_MIN           ;VAR_LISR = V_THR - V_THR_MIN (16bit sub)
    subwf   V_THR_L,W
    movwf   VAR_LISR_L
    movlw   HIGH V_THR_MIN
    subwfb  V_THR_H,W
    movwf   VAR_LISR_H
    bc      V_THR_CMD_NOTZERO       ;no borrow (carry)?
    clrf    VAR_LISR_H
    clrf    VAR_LISR_L
    bra     V_THR_CMD_END
V_THR_CMD_NOTZERO
    movlw   V_THR_SCALE
    mulwf   VAR_LISR_H              ;V_THR_SCALExVAR_LISR_H -> VAR_LISR_H:VAR_LISR
    movff   PRODH,VAR_LISR_H        ;Copy high product in VAR_LISR_H
    movff   PRODL,VAR_LISR          ;Copy low product in VAR_LISR
    mulwf   VAR_LISR_L              ;V_THR_SCALExVAR_LISR_L -> VAR_LISR_L:-
    movff   PRODH,VAR_LISR_L
    movf    VAR_LISR,W
    addwf   VAR_LISR_L,F            ;add the 2 product
    clrf    WREG
    addwfc  VAR_LISR_H,F
    bz      V_THR_CMD_END
    setf    VAR_LISR_L              ;MSB not zero -> clip
V_THR_CMD_END
    movff   VAR_LISR_L,THR_CMD
#endif

T_FET_MAX_CHECK
    movlw   HIGH T_FET_MAX      
    cpfsgt  T_FET_H                 ;T_FET_H > HIGH T_FET_MAX ?
    bra     T_FET_MAX_HEQ           ;no, so check =
T_FET_MAX_RESET
    led3on
    led1on
    sw_bkpt
    bra     UPDATE_BMC_RESET        ;yes, V_IN too high
T_FET_MAX_HEQ
    cpfseq  T_FET_H                 ;T_FET_H = HIGH T_FET_MAX ?
    bra     V_IN_MAX_CHECK          ;no, so T_FET_H < HIGH T_FET_MAX, V_IN ok
    movlw   LOW T_FET_MAX
    cpfslt  T_FET_L                 ;so, T_FET_L < LOW T_FET_MAX ?
    bra     T_FET_MAX_RESET         ;no, V_IN too high

V_IN_MAX_CHECK

V_IN_LOW_CHECK
    movlw   HIGH V_IN_LOW
    cpfslt  V_IN_H                  ;V_IN_H < HIGH V_IN_LOW ?
    bra     V_IN_LOW_HEQ            ;no, so check =
V_IN_LOW_EVENT
    sw_bkpt
    led2on
    bra     V_IN_MIN_CHECK          ;yes, V_IN low
V_IN_LOW_HEQ
    cpfseq  V_IN_H                  ;V_IN_H = HIGH V_IN_LOW ?
    bra     UPDATE_BMC_INIT         ;no, so V_IN_H > HIGH V_IN_LOW, V_IN ok
    movlw   LOW V_IN_LOW
    cpfsgt  V_IN_L                  ;so, V_IN_L > LOW V_IN_LOW ?
    bra     V_IN_LOW_EVENT          ;no, V_IN low
    bra     UPDATE_BMC_INIT         ;yes V_IN ok

V_IN_MIN_CHECK

UPDATE_BMC_INIT
    led1on
    movlw   THR_CMD_MIN
    cpfslt  THR_CMD                 ;THR_CMD < MIN?
    bra     UPDATE_BMC_RUN          ;no -> run

UPDATE_BMC_NORUN
    movf    THR_CMD,W
    bz      UPDATE_BMC_BRAKE
    bra     UPDATE_BMC_RESET

UPDATE_BMC_BRAKE
    bra     UPDATE_BMC_RESET

UPDATE_BMC_RUN
    bcf     GEN_STATUS,BRAKE_ON
    bsf     GEN_STATUS,BMC_RUN      ;run mode

    btfsc   ZC_STATUS,LOCKED_FLAG   ;already locked?
    bra     UPDATE_CMD_REQ_INIT     ;yes

    movlw   THR_MIN_NOLOCK
    cpfsgt  THR_CMD                 ;THR_CMD > MIN_NOLOCK?
    bra     UPDATE_BMC_RESET        ;not yet

    movff   THR_CMD,CMD_REQ

    movlw   THR_MAX_NOLOCK
    cpfsgt  THR_CMD                 ;THR_CMD > MAX_NOLOCK?
    bra     UPDATE_BMC_CMD          ;no continue
    movwf   CMD_REQ                 ;limit the throttle until 1st ZC lock
    bra     UPDATE_BMC_CMD

UPDATE_CMD_REQ_INIT
    movff   THR_CMD,CMD_REQ

UPDATE_BMC_CMD
    bsf     BMC_STATUS,STEPPING_ON
    movlw   HIGH I_CMD_SCALE
    mulwf   CMD_REQ
    movff   PRODH,I_CMD_H
    movff   PRODL,I_CMD_L
    movlw   LOW I_CMD_SCALE
    mulwf   CMD_REQ
    movf    PRODH,W
    addwf   I_CMD_L,F               ;add the 2 product
    movlw   0
    addwfc  I_CMD_H,F
    return

UPDATE_BMC_RESET
    bcf     BMC_STATUS,STEPPING_ON
    clrf    I_CMD_H
    clrf    I_CMD_L
    bcf     GEN_STATUS,BMC_RUN
    bcf     GEN_STATUS,LOGGING_ON
    return

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
UPDATE_ERROR
    goto    ASSERT
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
    call    CLIP_PERIOD_MAX         ;Clip period to max value
    call    CLIP_PERIOD_MIN         ;Clip period to min value
    movff   ZC_NEW_L,ZC_OLD_L       ;save new ZC time
    movff   ZC_NEW_H,ZC_OLD_H
UPDATE_ADV
    call    ADV_CAL                 ;Auto advance
    movwf   ADV_CMD
    bnz     TIMING_ADV_CAL          ;check if zero?
    bsf     BMC_STATUS,ADV_MAX_FLAG ;set flag so no ADV waiting!
    bra     UPDATE_TIMING_RETURN
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
    movf    I_OLD_H,W               ;ABS(I_OLD)
    bnn     TIMING_ADV_I_OLD_NN
    comf    I_OLD_H,F               ;Complement all bytes
    comf    I_OLD_L,F
    infsnz  I_OLD_L,F               ;Inc. low byte and Skip if no carry to higher bytes
    incf    I_OLD_H,F
TIMING_ADV_I_OLD_NN
    movlw   K_I_LAG
    btfsc   GEN_STATUS,BRAKE_ON
    movlw   0
    mulwf   I_OLD_H                 ;I_OLD/256 x W (K_I_LAG), high byte fist
    movff   PRODH,VAR_TEMP_H        ;Copy high product in VAR_TEMP_H
    movff   PRODL,VAR_TEMP_L        ;Copy low product in VAR_TEMP_L
    mulwf   I_OLD_L                 ;lower byte
    movf    PRODH,W
    addwf   VAR_TEMP_L,F            ;add the 2 product
    movlw   0
    addwfc  VAR_TEMP_H,F
    rlcf    VAR_TEMP_L,W            ;*2, K_I_LAG is only scaled by 128
    rlcf    VAR_TEMP_H,F
    addwf   ADV_TIME_L,F            ;add to ADV_TIME
    movf    VAR_TEMP_H,W
    addwfc  ADV_TIME_H,F
    bn      UPDATE_TIMING_RETURN    ;check if still negative?
    clrf    ADV_TIME_L
    clrf    ADV_TIME_H
    bsf     BMC_STATUS,ADV_MAX_FLAG ;set flag so no ADV waiting!
UPDATE_TIMING_RETURN
    return


;********************************************************************
; Calculate the advance value using a table (hardware dependant)
;********************************************************************
#define TBL_OFFSET_MAX  d'31'       ;max period for variable advance  
#define ADV_MIN         0x6E        ;25.89 degrees

ADV_TABLE                           ;LSB,MSB in 1/256
  db    0x00,0x00
  db    0x00,0x00
  db    0x00,0x00
  db    0x00,0x00
  db    0x0E,0x22
  db    0x30,0x3B
  db    0x43,0x4A
  db    0x4F,0x53
  db    0x57,0x5A
  db    0x5D,0x5F
  db    0x61,0x63
  db    0x64,0x66
  db    0x67,0x68
  db    0x69,0x6A
  db    0x6B,0x6C
  db    0x6D,0x6D
;  db    0x00,0x00
;  db    0x00,0x00
;  db    0x0E,0x22
;  db    0x30,0x3B
;  db    0x43,0x4A
;  db    0x4F,0x53
;  db    0x57,0x5A
;  db    0x5D,0x5F
;  db    0x61,0x63
;  db    0x64,0x66
;  db    0x67,0x68
;  db    0x69,0x6A
;  db    0x6B,0x6C
;  db    0x6D,0x6D
;  db    0x6E,0x6F
;  db    0x6F,0x70

ADV_CAL
    rlcf    PERIOD_L,W              ;*2/256 -> PERIOD/128
    movwf   VAR_TEMP_L
    rlcf    PERIOD_H,W
    bc      ADV_USE_MIN             ;Period > 0x7FFF, so min ADV
    movwf   VAR_TEMP_H
    rlcf    VAR_TEMP_L,W            ;*4/256 -> PERIOD/64
    rlcf    VAR_TEMP_H,W
    bc      ADV_USE_MIN             ;Period > 0x3FFF, so min ADV
    movwf   VAR_TEMP_H
    rlcf    VAR_TEMP_L,W            ;*8/256 -> PERIOD/32
    rlcf    VAR_TEMP_H,W
    bc      ADV_USE_MIN             ;Period > 0x1FFF, so min ADV
    movwf   TBLPTRL
    movlw   TBL_OFFSET_MAX
    cpfsgt  TBLPTRL                 ;PERIOD/32 > TBL_OFFSET_MAX ?
    bra     ADV_USE_TABLE           ;no, use table
ADV_USE_MIN
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
; Calculate ZC blanking period and reload timer 0
;********************************************************************
LOAD_BLANKING
    bcf     INTCON,TMR0IE           ;disable timer 0 int
    movlw   BLANKING_ANGLE
    sublw   0xFF
    addlw   0x01
    mulwf   PERIOD_H                ;PERIOD/256 x DEMAG, high byte fist
    movff   PRODH,VAR_TEMP_H        ;Copy high product in VAR_TEMP_H
    movff   PRODL,VAR_TEMP_L        ;Copy low product in VAR_TEMP_L
    mulwf   PERIOD_L                ;lower byte
    movf    PRODH,W
    addwf   VAR_TEMP_L,F            ;add the 2 product
    clrf    WREG
    addwfc  VAR_TEMP_H,F
    movf    ADV_TIME_L,W            ;add the advance
    addwf   VAR_TEMP_L,F
    movf    ADV_TIME_H,W
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
; Calculate demagnetization timing and reload timer 0
;********************************************************************
LOAD_DEMAG
    bcf     INTCON,TMR0IE           ;disable timer 0 int
    movlw   LOW PERIOD_DEMAG
    movwf   VAR_TEMP_L
    movlw   HIGH PERIOD_DEMAG
    movwf   VAR_TEMP_H
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
    movf    PORTA,W                 ;read and format the comparator output
    andlw   0x1C
    movwf   VAR_TEMP
    movf    PORTA,W                 ;read and format the comparator output
    andlw   0x1C
    cpfseq  VAR_TEMP
    bra     STEP_FIND_FILTER
STEP_FIND_LOOKUP
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

STEP_FIND_FILTER
    movf    PORTA,W                 ;read and format the comparator output
    andlw   0x1C
    cpfseq  VAR_TEMP
    movwf   VAR_TEMP
    bra     STEP_FIND_LOOKUP

;*******************************************************************************
; Pre-load the PWM overdrive setting for the next step
;*******************************************************************************
OVDCON_TABLE
    code_pack
       ;OVDCOND,    OVDCONS,    PWMCON0
    db  b'00000000',b'00000000',b'01011111' ;Step 0  phase A: X,    phase C: X,    phase B: X

    db  b'11000000',b'00000100',b'01010111' ;Step 1  phase A: X->H, phase C: H->X, phase B: L->L

    db  b'11000000',b'00010000',b'01010111' ;Step 2  phase A: H->H, phase C: X->L, phase B: L->X

    db  b'00001100',b'00010000',b'01011101' ;Step 3  phase A: H->X, phase C: L->L, phase B: X->H

    db  b'00001100',b'01000000',b'01011101' ;Step 4  phase A: X->L, phase C: L->X, phase B: H->H

    db  b'00110000',b'01000000',b'01011011' ;Step 5  phase A: L->L, phase C: X->H, phase B: H->X

    db  b'00110000',b'00000100',b'01011011' ;Step 6  phase A: L->X, phase C: H->H, phase B: X->L

    db  b'00000000',b'01010100',b'01011111' ;Step 7  phase A: L,    phase C: L,    phase B: L   

    db  b'11000000',b'00010100',b'01010111' ;Step 8  phase A: H,    phase C: L,    phase B: L

    db  b'11001100',b'00010000',b'01010101' ;Step 9  phase A: H,    phase C: L,    phase B: H

    db  b'00001100',b'01010000',b'01011101' ;Step 10 phase A: L,    phase C: L,    phase B: H

    db  b'00111100',b'01000000',b'01011001' ;Step 11 phase A: L,    phase C: H,    phase B: H

    db  b'00110000',b'01000100',b'01011011' ;Step 12 phase A: L,    phase C: H,    phase B: L

    db  b'11110000',b'00000100',b'01010011' ;Step 13  phase A: H,    phase C: H,    phase B: L

LOAD_STEP
    movlw   0x03                    ;3 bytes space
    mulwf   STEP_NEXT
    movf    PRODL,W
    addlw   LOW OVDCON_TABLE        ;Initialize low byte of table pointer
    movwf   TBLPTRL
    clrf    TBLPTRH
    movlw   HIGH OVDCON_TABLE       ;Initialize high byte of table pointer
    addwfc  TBLPTRH
    clrf    TBLPTRU
    movlw   UPPER OVDCON_TABLE      ;Initialize upper byte of table pointer  
    addwfc  TBLPTRU
    tblrd*+
    bcf     INTCON,GIEH             ;Disable high priority interrupts
    movff   TABLAT,OVDCOND_TEMP
    tblrd*+
    movff   TABLAT,OVDCONS_TEMP
    tblrd*
    movff   TABLAT,PWMCON0_TEMP
    bsf     INTCON,GIEH             ;re-enable high priority interrupts
    return

;*******************************************************************************
;load the IC with new step config, WARNING BOUNDARY
;*******************************************************************************
LOAD_IC     org 0x1800
    bcf     ZC_STATUS,NEW_FLAG      ;clear zc flag
    clrf    PIE3                    ;disable all IC interrupt
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
    movwf   CAP1CON                 ;IC1 Capture
    movwf   CAP2CON                 ;IC2 Capture
    movwf   CAP3CON                 ;IC3 Capture
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


;*******************************************************************************
; lookup tabe for 8bit division
;*******************************************************************************
DIV_TABLE
   dw 0xFFFF
   dw 0xFFFF
   dw 0x8000
   dw 0x5555
   dw 0x4000
   dw 0x3333
   dw 0x2AAA
   dw 0x2492
   dw 0x2000
   dw 0x1C71
   dw 0x1999
   dw 0x1745
   dw 0x1555
   dw 0x13B1
   dw 0x1249
   dw 0x1111
   dw 0x1000
   dw 0x0F0F
   dw 0x0E38
   dw 0x0D79
   dw 0x0CCC
   dw 0x0C30
   dw 0x0BA2
   dw 0x0B21
   dw 0x0AAA
   dw 0x0A3D
   dw 0x09D8
   dw 0x097B
   dw 0x0924
   dw 0x08D3
   dw 0x0888
   dw 0x0842
   dw 0x0800
   dw 0x07C1
   dw 0x0787
   dw 0x0750
   dw 0x071C
   dw 0x06EB
   dw 0x06BC
   dw 0x0690
   dw 0x0666
   dw 0x063E
   dw 0x0618
   dw 0x05F4
   dw 0x05D1
   dw 0x05B0
   dw 0x0590
   dw 0x0572
   dw 0x0555
   dw 0x0539
   dw 0x051E
   dw 0x0505
   dw 0x04EC
   dw 0x04D4
   dw 0x04BD
   dw 0x04A7
   dw 0x0492
   dw 0x047D
   dw 0x0469
   dw 0x0456
   dw 0x0444
   dw 0x0432
   dw 0x0421
   dw 0x0410
   dw 0x0400
   dw 0x03F0
   dw 0x03E0
   dw 0x03D2
   dw 0x03C3
   dw 0x03B5
   dw 0x03A8
   dw 0x039B
   dw 0x038E
   dw 0x0381
   dw 0x0375
   dw 0x0369
   dw 0x035E
   dw 0x0353
   dw 0x0348
   dw 0x033D
   dw 0x0333
   dw 0x0329
   dw 0x031F
   dw 0x0315
   dw 0x030C
   dw 0x0303
   dw 0x02FA
   dw 0x02F1
   dw 0x02E8
   dw 0x02E0
   dw 0x02D8
   dw 0x02D0
   dw 0x02C8
   dw 0x02C0
   dw 0x02B9
   dw 0x02B1
   dw 0x02AA
   dw 0x02A3
   dw 0x029C
   dw 0x0295
   dw 0x028F
   dw 0x0288
   dw 0x0282
   dw 0x027C
   dw 0x0276
   dw 0x0270
   dw 0x026A
   dw 0x0264
   dw 0x025E
   dw 0x0259
   dw 0x0253
   dw 0x024E
   dw 0x0249
   dw 0x0243
   dw 0x023E
   dw 0x0239
   dw 0x0234
   dw 0x0230
   dw 0x022B
   dw 0x0226
   dw 0x0222
   dw 0x021D
   dw 0x0219
   dw 0x0214
   dw 0x0210
   dw 0x020C
   dw 0x0208
   dw 0x0204
   dw 0x0200
   dw 0x01FC
   dw 0x01F8
   dw 0x01F4
   dw 0x01F0
   dw 0x01EC
   dw 0x01E9
   dw 0x01E5
   dw 0x01E1
   dw 0x01DE
   dw 0x01DA
   dw 0x01D7
   dw 0x01D4
   dw 0x01D0
   dw 0x01CD
   dw 0x01CA
   dw 0x01C7
   dw 0x01C3
   dw 0x01C0
   dw 0x01BD
   dw 0x01BA
   dw 0x01B7
   dw 0x01B4
   dw 0x01B2
   dw 0x01AF
   dw 0x01AC
   dw 0x01A9
   dw 0x01A6
   dw 0x01A4
   dw 0x01A1
   dw 0x019E
   dw 0x019C
   dw 0x0199
   dw 0x0197
   dw 0x0194
   dw 0x0192
   dw 0x018F
   dw 0x018D
   dw 0x018A
   dw 0x0188
   dw 0x0186
   dw 0x0183
   dw 0x0181
   dw 0x017F
   dw 0x017D
   dw 0x017A
   dw 0x0178
   dw 0x0176
   dw 0x0174
   dw 0x0172
   dw 0x0170
   dw 0x016E
   dw 0x016C
   dw 0x016A
   dw 0x0168
   dw 0x0166
   dw 0x0164
   dw 0x0162
   dw 0x0160
   dw 0x015E
   dw 0x015C
   dw 0x015A
   dw 0x0158
   dw 0x0157
   dw 0x0155
   dw 0x0153
   dw 0x0151
   dw 0x0150
   dw 0x014E
   dw 0x014C
   dw 0x014A
   dw 0x0149
   dw 0x0147
   dw 0x0146
   dw 0x0144
   dw 0x0142
   dw 0x0141
   dw 0x013F
   dw 0x013E
   dw 0x013C
   dw 0x013B
   dw 0x0139
   dw 0x0138
   dw 0x0136
   dw 0x0135
   dw 0x0133
   dw 0x0132
   dw 0x0130
   dw 0x012F
   dw 0x012E
   dw 0x012C
   dw 0x012B
   dw 0x0129
   dw 0x0128
   dw 0x0127
   dw 0x0125
   dw 0x0124
   dw 0x0123
   dw 0x0121
   dw 0x0120
   dw 0x011F
   dw 0x011E
   dw 0x011C
   dw 0x011B
   dw 0x011A
   dw 0x0119
   dw 0x0118
   dw 0x0116
   dw 0x0115
   dw 0x0114
   dw 0x0113
   dw 0x0112
   dw 0x0111
   dw 0x010F
   dw 0x010E
   dw 0x010D
   dw 0x010C
   dw 0x010B
   dw 0x010A
   dw 0x0109
   dw 0x0108
   dw 0x0107
   dw 0x0106
   dw 0x0105
   dw 0x0104
   dw 0x0103
   dw 0x0102
   dw 0x0101

LOAD_DIV
    movwf   DIV_L
    bcf     STATUS,C
    clrf    DIV_H
    rlcf    DIV_L
    rlcf    DIV_H
    movf    DIV_L,W
    addlw   LOW DIV_TABLE       ;Initialize low byte of table pointer
    movwf   TBLPTRL
    movff   DIV_H,TBLPTRH
    movlw   HIGH DIV_TABLE      ;Initialize high byte of table pointer
    addwfc  TBLPTRH
    clrf    TBLPTRU
    movlw   UPPER DIV_TABLE     ;Initialize upper byte of table pointer  
    addwfc  TBLPTRU
    tblrd*+
    movff   TABLAT,DIV_L
    tblrd*
    movff   TABLAT,DIV_H
    return
    END
