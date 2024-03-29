#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "image_process.h"
#include "Drive.h"

DigitalOut  Left_motor_signal(P4_6);    /* Used by motor function   */
DigitalOut  Right_motor_signal(P4_7);   /* Used by motor function   */
BusIn       dipsw( P7_15, P8_1, P2_9, P5_0 ); /* SW1 on Shield board */
//BusIn       dipsw( P7_15, P8_1, P2_9, P2_10 ); /* SW1 on Shield board */

Drive::Drive()
{
    init_MTU2_PWM_Motor();
    init_MTU2_PWM_Servo();


}

void Drive::run( int accele )
{
    if(handle_buff > 0)
        motor(accele,diff(accele));
    else
        motor(diff(accele),accele);

}

//Deff fanction
//------------------------------------------------------------------//
int Drive::diff( int pwm )
{
    int i;
    int revolution_difference[] = {
        100, 98, 96, 95, 93,
        91, 90, 88, 86, 85,
        83, 82, 80, 79, 77,
        76, 74, 73, 71, 70,
        68, 67, 65, 64, 63,
        61, 60, 58, 57, 55,
        54, 53, 51, 50, 48,
        47, 45, 44, 42, 41,
        39, 38, 36, 35, 33,
        32
    };
    i  = handle_buff;
    if( i <  0 ) i = -i;
    if( i > 45 ) i = 45;
 //   ret = revolution_difference[i] * pwm / 100;

    return revolution_difference[i] * pwm / 100;
}

//INITIALIZE MTU2 PWM FUNCTIONS
//------------------------------------------------------------------//
//MTU2_3, MTU2_4
//Reset-Synchronized PWM mode
//TIOC4A(P4_4) :Left-motor
//TIOC4B(P4_5) :Right-motor
//------------------------------------------------------------------//
void Drive::init_MTU2_PWM_Motor( void )
{
    /* Port setting for S/W I/O Contorol */
    /* alternative mode     */

    /* MTU2_4 (P4_4)(P4_5)  */
    GPIOPBDC4   = 0x0000;               /* Bidirection mode disabled*/
    GPIOPFCAE4 &= 0xffcf;               /* The alternative function of a pin */
    GPIOPFCE4  |= 0x0030;               /* The alternative function of a pin */
    GPIOPFC4   &= 0xffcf;               /* The alternative function of a pin */
    /* 2nd altemative function/output   */
    GPIOP4     &= 0xffcf;               /*                          */
    GPIOPM4    &= 0xffcf;               /* p4_4,P4_5:output         */
    GPIOPMC4   |= 0x0030;               /* P4_4,P4_5:double         */

    /* Mosule stop 33(MTU2) canceling */
    CPGSTBCR3  &= 0xf7;

    /* MTU2_3 and MTU2_4 (Motor PWM) */
    MTU2TCR_3   = 0x20;                 /* TCNT Clear(TGRA), P0��/1  */
    MTU2TOCR1   = 0x04;                 /*                          */
    MTU2TOCR2   = 0x40;                 /* N L>H  P H>L             */
    MTU2TMDR_3  = 0x38;                 /* Buff:ON Reset-Synchronized PWM mode */
    MTU2TMDR_4  = 0x30;                 /* Buff:ON                  */
    MTU2TOER    = 0xc6;                 /* TIOC3B,4A,4B enabled output */
    MTU2TCNT_3  = MTU2TCNT_4 = 0;       /* TCNT3,TCNT4 Set 0        */
    MTU2TGRA_3  = MTU2TGRC_3 = MOTOR_PWM_CYCLE;
    /* PWM-Cycle(1ms)           */
    MTU2TGRA_4  = MTU2TGRC_4 = 0;       /* Left-motor(P4_4)         */
    MTU2TGRB_4  = MTU2TGRD_4 = 0;       /* Right-motor(P4_5)        */
    MTU2TSTR   |= 0x40;                 /* TCNT_4 Start             */
}

void Drive::init_MTU2_PWM_Servo( void )
{
    /* Port setting for S/W I/O Contorol */
    /* alternative mode     */

    /* MTU2_0 (P4_0)        */
    GPIOPBDC4   = 0x0000;               /* Bidirection mode disabled*/
    GPIOPFCAE4 &= 0xfffe;               /* The alternative function of a pin */
    GPIOPFCE4  &= 0xfffe;               /* The alternative function of a pin */
    GPIOPFC4   |= 0x0001;               /* The alternative function of a pin */
    /* 2nd alternative function/output   */
    GPIOP4     &= 0xfffe;               /*                          */
    GPIOPM4    &= 0xfffe;               /* p4_0:output              */
    GPIOPMC4   |= 0x0001;               /* P4_0:double              */

    /* Mosule stop 33(MTU2) canceling */
    CPGSTBCR3  &= 0xf7;

    /* MTU2_0 (Motor PWM) */
    MTU2TCR_0   = 0x22;                 /* TCNT Clear(TGRA), P0��/16 */
    MTU2TIORH_0 = 0x52;                 /* TGRA L>H, TGRB H>L       */
    MTU2TMDR_0  = 0x32;                 /* TGRC and TGRD = Buff-mode*/
    /* PWM-mode1                */
    MTU2TCNT_0  = 0;                    /* TCNT0 Set 0              */
    MTU2TGRA_0  = MTU2TGRC_0 = SERVO_PWM_CYCLE;
    /* PWM-Cycle(16ms)          */
    MTU2TGRB_0  = MTU2TGRD_0 = 0;       /* Servo-motor(P4_0)        */
    MTU2TSTR   |= 0x01;                 /* TCNT_0 Start             */
}



//------------------------------------------------------------------//
//motor speed control(PWM)
//Arguments: motor:-100 to 100
//Here, 0 is stop, 100 is forward, -100 is reverse
//------------------------------------------------------------------//
void Drive::motor( int accele_l, int accele_r )
{

//    sw_data = dipsw.read() & 0x0f + 5;
    sw_data = dipsw.read() & 0x0f;
    sw_data += 5;
    accele_l = ( accele_l * sw_data ) / 20;
    accele_r = ( accele_r * sw_data ) / 20;

    /* Left Motor Control */
    if( accele_l >= 0 ) {
        /* forward */
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        /* reverse */
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* Right Motor Control */
    if( accele_r >= 0 ) {
        /* forward */
        Right_motor_signal = 0;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        /* reverse */
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

//------------------------------------------------------------------//
//motor speed control(PWM)
//Arguments: motor:-100 to 100
//Here, 0 is stop, 100 is forward, -100 is reverse
//------------------------------------------------------------------//
void Drive::motor2( int accele_l, int accele_r )
{
    /* Left Motor Control */
    if( accele_l >= 0 ) {
        /* forward */
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        /* reverse */
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* Right Motor Control */
    if( accele_r >= 0 ) {
        /* forward */
        Right_motor_signal = 0;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        /* reverse */
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

//------------------------------------------------------------------//
//handle Function
//------------------------------------------------------------------//
void Drive::handle( int angle )
{
    handle_buff = angle;
    /* When the servo move from left to right in reverse, replace "-" with "+" */
    MTU2TGRD_0 = SERVO_CENTER - angle * HANDLE_STEP;
}
