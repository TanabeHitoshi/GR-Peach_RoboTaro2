#ifndef DRIVE_H
#define DRIVE_H
 

//Motor PWM cycle
#define     MOTOR_PWM_CYCLE     33332   /* Motor PWM period         */

//Servo PWM cycle
#define     SERVO_PWM_CYCLE     33332   /* SERVO PWM period         */
/* 16ms   P0É”/16 = 0.48us   */
#define     SERVO_CENTER        3100    /* 1.5ms / 0.48us - 1 = 3124*/
#define     HANDLE_STEP         18      /* 1 degree value           */

//volatile int            handle_buff;    /* diff function only       */

class Drive {
public:
    Drive(void);
    int diff( int pwm );                                //image_sensorAnalog_get
    void handle( int angle );
    void motor( int accele_l, int accele_r );
    void motor2( int accele_l, int accele_r );

    volatile int            handle_buff;

private:
    void init_MTU2_PWM_Motor( void );                   //Initialize MTU2 PWM functions
    void init_MTU2_PWM_Servo( void );                   //Initialize MTU2 PWM functions

};
 
#endif
