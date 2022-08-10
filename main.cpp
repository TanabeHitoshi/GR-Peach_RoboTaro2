//------------------------------------------------------------------//
//Supported MCU:   RZ/A1H
//File Contents:   kit18 GR-peach ( Trace program )
//Version number:  Ver.1.00
//Date:            2018.10.30
//Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//------------------------------------------------------------------//
//This program supports the following kit:
//* M-S348 Image processing micon car production kit
//------------------------------------------------------------------//
//Include
//------------------------------------------------------------------//
#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "image_process.h"
#include "Drive.h"
#include "isCamera.h"
//------------------------------------------------------------------//
//Define
//------------------------------------------------------------------//
#define     THRESHOLD           180     /* Binarization function only */

//LED Color on GR-PEACH
#define     LED_OFF             0x00
#define     LED_RED             0x01
#define     LED_GREEN           0x02
#define     LED_YELLOW          0x03
#define     LED_BLUE            0x04
#define     LED_PURPLE          0x05
#define     LED_SKYBLUE         0x06
#define     LED_WHITE           0x07

//led_m_set function only
#define     RUN                 0x00
#define     STOP                0x01
#define     ERROR               0x02
#define     DEBUG               0x03
#define     CRANK               0x04
#define     LCHANGE             0x05

//ImageData_Serial_out3 function only
#define     COLOR               0x01
#define     GREY_SCALE          0x02
#define     BINARY              0x03

//------------------------------------------------------------------//
//Define(NTSC-Video)
//------------------------------------------------------------------//
#define VIDEO_INPUT_CH         (DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define VIDEO_INT_TYPE         (DisplayBase::INT_TYPE_S0_VFIELD)
#define DATA_SIZE_PER_PIC      (2u)

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_BUFFER_STRIDE    (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT    (PIXEL_VW)

//------------------------------------------------------------------//
//Constructor
//------------------------------------------------------------------//
/* Create DisplayBase object */
DisplayBase Display;

Ticker      interrput;
Serial      pc(USBTX, USBRX);

DigitalOut  LED_R(P6_13);               /* LED1 on the GR-PEACH board */
DigitalOut  LED_G(P6_14);               /* LED2 on the GR-PEACH board */
DigitalOut  LED_B(P6_15);               /* LED3 on the GR-PEACH board */
DigitalOut  USER_LED(P6_12);            /* USER_LED on the GR-PEACH board */
DigitalIn   user_botton(P6_0);          /* SW1 on the GR-PEACH board */

//BusIn       dipsw( P7_15, P8_1, P2_9, P2_10 ); /* SW1 on Shield board */

DigitalIn   push_sw(P2_13);             /* SW1 on the Motor Drive board */
DigitalOut  LED_3(P2_14);               /* LED3 on the Motor Drive board */
DigitalOut  LED_2(P2_15);               /* LED2 on the Motor Drive board */

Drive m;
isCamera c;
//------------------------------------------------------------------//
//Prototype(NTSC-video)
//------------------------------------------------------------------//
void init_Camera( void );
void ChangeFrameBuffer( void );
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type);
static void WaitVfield(const int32_t wait_count);
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type);
static void WaitVsync(const int32_t wait_count);

//------------------------------------------------------------------//
//Prototype
//------------------------------------------------------------------//
//Peripheral functions

//Interrupt function
void intTimer( void );                  /* 1ms period               */

//GR-peach board
void led_rgb(int led);
void led_m_user( int led );
unsigned char user_button_get( void );
void led_m_set( int set );
void led_m_process( void );             /* Only function for interrupt */

//Motor drive board
void led_out(int led);
unsigned char pushsw_get( void );

//Shield board
unsigned char dipsw_get( void );

//------------------------------------------------------------------//
//Prototype( Mark detect functions )
//------------------------------------------------------------------//

//------------------------------------------------------------------//
//Prototype( Debug functions )
//------------------------------------------------------------------//
void ImageData_Serial_Out1( unsigned char *ImageData, int HW, int VW );
void ImageData_Serial_Out2( unsigned char *ImageData, int HW, int VW );
void ImageData_Serial_Out3( unsigned char *ImageData, int HW, int VW, int color_pattern );
void ImageData_Serial_Out4( unsigned char *ImageData, int HW, int VW );

void init_MTU2_RotaryEncoder( void );

//------------------------------------------------------------------//
//Global variable (NTSC-video)
//------------------------------------------------------------------//
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(16)));  //16 bytes aligned!;
static uint8_t FrameBuffer_Video_B[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(16)));  //16 bytes aligned!;
uint8_t * write_buff_addr = FrameBuffer_Video_A;
uint8_t * save_buff_addr  = FrameBuffer_Video_B;
static volatile int32_t vsync_count;
static volatile int32_t vfield_count;
static volatile int32_t vfield_count2 = 1;
static volatile int32_t vfield_count2_buff;

//------------------------------------------------------------------//
//Global variable for Image process
//------------------------------------------------------------------//
unsigned char   ImageData_A[ ( ( PIXEL_HW * 2) * PIXEL_VW ) ];
unsigned char   ImageData_B[ ( PIXEL_HW * PIXEL_VW ) ];
unsigned char   ImageComp_B[ ( PIXEL_HW * PIXEL_VW ) ];

double          Rate = 0.25;       /* Reduction ratio              */

//------------------------------------------------------------------//
//Global variable for Digital sensor function
//------------------------------------------------------------------//
int             SenError;
int             Sen1Px[5];
//------------------------------------------------------------------//
//Global variable for Mark detection function
//------------------------------------------------------------------//

//------------------------------------------------------------------//
//Global variable for Trace program
//------------------------------------------------------------------//
volatile unsigned long  cnt0;           /* Used by timer function   */
volatile unsigned long  cnt1;           /* Used within main         */
volatile unsigned long  cntGate;           /* Used within main         */
volatile unsigned long  cntFaling;           /* Used within main         */
volatile long  cnt_curve;           /* Used curve         */
volatile long  cnt_dammy;           /* Used within main         */
volatile long  cntUnder;           /* Used within main         */
volatile int            pattern;        /* Pattern numbers          */

volatile int            led_pattern;    /* led_m_process function only */
volatile int            initFlag;       /* Initialize flag          */
volatile int            threshold_buff; /* Binarization function only */

int                     memory[10000][5];
int                     m_number;
int                     cr = 0;
int                     bar;
int 					pidValue;
int						handle_value;
int                     crank,crank2,crank_turn,lane_half,lane_Black;
char                    LR,curve_LR;             //CLANK,Lenchang dircection
char                    mem_lr[] = {'R','R','R','L','R','e'};     //Crank and Lenchange memory
int                     mem_crk[] = {0,300,290,0,260,-1};           //Crank brake
int                     n_lr = 0;           //Crank and Lenchange position
char					fall_flag;			//Detects fall from the course [1]->non [0]->on
int Threshold_value[50];
int Threshold_Ave,k=0,underPass = 0;
int LeneChange_time;

//Rotary Encoder
volatile long           lEncoderTotal;  /* Integrated value         */
volatile int            iEncoder;       /* current value            */
volatile unsigned int   uEncoderBuff;   /* last count               */
volatile long           lEncoderLine;   /* Integrated value         */

//******************************************************************//
// Main function
//*******************************************************************/
int main( void )
{
    volatile int    Number;             /* Serial Debug Mode only   */

    initFlag = 1;                       /* Initialization start     */
    int sp;
    int hd;
    fall_flag = 0;
    int LeneChange_state,LeneChange_count;

    /* Camera start */
    init_Camera();
    /* wait to stabilize NTSC signal (about 170ms) */
    wait(0.2);

    /* Initialize MCU functions */

    interrput.attach(&intTimer, 0.001);
    pc.baud(230400);
    init_MTU2_RotaryEncoder();
    /* Initialize Micon Car state */
    m.handle( 0 );
    m.motor( 0, 0 );
    led_out( 0x0 );
    led_m_set( STOP );

	cntFaling = 0;

//    threshold_buff = Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
    threshold_buff = THRESHOLD;

    wait(0.5);

     /* Initialize Pattern Matching */
    //RightCrank.sdevi = Standard_Deviation( RightCrank.binary, RightCrank.devi, RightCrank.w, RightCrank.h );
    //RightLaneChange.sdevi = Standard_Deviation( RightLaneChange.binary, RightLaneChange.devi, RightLaneChange.w, RightLaneChange.h );

    //LeftCrank.sdevi = Standard_Deviation(  LeftCrank.binary,  LeftCrank.devi,  LeftCrank.w,  LeftCrank.h );
    //LeftLaneChange.sdevi = Standard_Deviation(  LeftLaneChange.binary,  LeftLaneChange.devi,  LeftLaneChange.w,  LeftLaneChange.h );

    initFlag = 0;                       /* Initialization end       */
    c.y_center = 20;
    /* Debug Program */
    if( user_button_get() ) {
        led_m_set( DEBUG );
        wait(0.1);//ON Time
        while( user_button_get() );
        wait(0.5);//OFF Time
        while( 1 ) {
            pc.printf( "Serial Debug Mode Ver1.0 (2018.10.30)\n\r" );
            pc.printf( "\n\r" );
            pc.printf( "0:TeraTram Real-time display 20* 15pixel (Binary)\n\r" );
            pc.printf( "1:Excel(csv) 160*120pixel\n\r" );
            pc.printf( "2:Excel(csv)  20* 15pixel\n\r" );
            pc.printf( "3:Excel(csv) 160*120pixel -> csv_jpg_convert.bat (color)\n\r" );
            pc.printf( "4:Excel(csv) 160*120pixel -> csv_jpg_convert.bat (grey scale)\n\r" );
            pc.printf( "5:Excel(csv)  20* 15pixel -> csv_jpg_convert.bat (binary)\n\r" );
            pc.printf( "6:TeraTram Real-time display 20* 15pixel (Raw)\n\r" );
            pc.printf( "\n\r" );
            pc.printf( "Please Number\n\r" );
            pc.printf( "No = " );
            pc.scanf( "%d", &Number );
            pc.printf( "\n\r" );
            pc.printf( "Please push the SW ( on the Motor drive board )\n\r" );
            pc.printf( "\n\r" );
            while( !pushsw_get() );

            switch( Number ) {
                case 0:
                    /* for TeraTerm(Real-time display) */
                    while( 1 ) {
                        ImageData_Serial_Out2( c.ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) );
                     }
 //                   break;
                case 1:
                    /* for the Excel(csv) 160*120pixel */
                    ImageData_Serial_Out1( ImageData_B, PIXEL_HW, PIXEL_VW );
                    break;
                case 2:
                    /* for the Excel(csv)  20* 15pixel */
                    ImageData_Serial_Out1( ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate) );
                    break;
                case 3:
                    /* for the Excel(csv) -> csv_jpg_convert.bat */
                    ChangeFrameBuffer();
                    ImageData_Serial_Out3( save_buff_addr, PIXEL_HW, PIXEL_VW, COLOR );
                    break;
                case 4:
                    /* for the Excel(csv) -> csv_jpg_convert.bat */
                    ImageData_Serial_Out3( ImageData_B, PIXEL_HW, PIXEL_VW, GREY_SCALE );
                    break;
                case 5:
                    /* for the Excel(csv) -> csv_jpg_convert.bat */
                    ImageData_Serial_Out3( c.ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), BINARY );
                    break;
                case 6:
                    /* for TeraTerm(Real-time display) */
                	while( 1 ) {
                        ImageData_Serial_Out4( ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate) );
//                        SenVal8	= sensor_process8( c.ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 12 );
//                        pc.printf( "threshold %3d \n\r", Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate)) );
                        pc.printf( "sensor_process8 %x \n\r", c.SenVal8 );

                	}
                   break;
                default:
                    break;
            }
            led_m_set( STOP );
            while(1);
        }
    }

    /* Trace program */
    led_m_set( RUN );

    while( 1 ) {
        if(user_button_get()) pattern = 1000;
//        if(pattern > 9 && pattern < 1000 && pushsw_get()) pattern = 1000;
//       pc.printf( "pattern = %d\n\r",pattern );
        switch( pattern ) {
            case 0:
                /* wait for switch input */
                if( pushsw_get() ) {
                    led_out( 0x0 );
                    led_m_set( RUN );
//                    wait(2.0);
                    pattern = 10;
                    cnt1 = 0;
                    cntGate = 0;
                    cnt_curve = 0;
                    handle_value = 0;
                    LeneChange_state = 0;
                    LeneChange_count = 0;
                    LeneChange_time = 0;
                    break;
                }
                if(bar){
					if( cnt1 < 100 ) {
						led_out( 0x1 );
					} else if( cnt1 < 200 ) {
						led_out( 0x2 );
					} else {
						cnt1 = 0;
					}
                }else{
					if( cnt1 < 1000 ) {
						led_out( 0x1 );
					} else if( cnt1 < 2000 ) {
						led_out( 0x2 );
					} else {
						cnt1 = 0;
					}
                }
                break;
            case 1:/* Camera check */
         		if(lane_Black != -1)pattern = 6;
                if( cnt1 < 50 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 100 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
            break;
            case 2:
            	m.motor(0,0);
                if(cnt1 > 1000){
                    pattern = 3;
                }
            break;
            case 3:
            	m.motor(0,0);
                if( bar == 0 || pushsw_get()) {
                    led_out( 0x0 );
                    led_m_set( RUN );
                    pattern = 21;
                    cnt1 = 0;
                    cntGate = 0;
                    break;
                }
                if( cnt1 < 50 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 100 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
                break;
            case 5:
                led_out( 0x3 );
                if( !pushsw_get() ) {
                    pattern = 6;
                    wait(1.0);
                }
            break;
            case 6:
                if( pushsw_get() ) {
                    led_out( 0x0 );
                    led_m_set( RUN );
                    pattern = 11;
                    cnt1 = 0;
                    break;
                }
                if( cnt1 < 200 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 400 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
                break;

            case 21: /* 8bit trace */
            	if( crank && cntGate > 2000){
            		pattern = 30;
                        break;
            	}
            	if( lane_half != -1 ){
            		pattern = 50;
            		break;
            	}
            	switch( c.sensor_inp8(MASK3_3) ) {
        		/* 1 */
            		case 0x00: /* xxx_ _xxx */
            			m.handle( 0 );
            			m.motor( 100, 100 );
                    break;
               /* 2 */
                   case 0x04: /* xxx_ _Oxx */
                	   m.handle( 3 );
                	   m.motor( 100, m.diff(100) );
                   break;
                   case 0x20: /* xxO_ _xxx */
                	   m.handle( -3 );
                	   m.motor( m.diff(100), 100 );
       				break;
       			/* 3�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｿ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｽ�ｽ｢ */
                   case 0x06: /* xxx_ _OOx */
                	   m.handle( 5 );
                	   m.motor( 80, m.diff(80) );
         		   break;
                   case 0x60: /* xOO_ _xxx */
                	   m.handle( -5 );
                	   m.motor( m.diff(80), 80 );
        		   break;
        		/* 4�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｿ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｿ�ｽｽ�ｿｽ�ｽｽ�ｽｽ�ｿｽ�ｽｽ�ｽ｣ */
                   case 0x02: /* xxx_ _xOx */
                	   m.handle( 10 );
                	   m.motor( 60, m.diff(60) );
                   break;
                   case 0x40: /* xOx_ _xxx */
                	   m.handle( -10 );
                	   m.motor( m.diff(60), 60 );
                    break;
               /* 5 */
                   case 0x07: /* xxx_ _OOO */
                	   m.handle( 15 );
                	   m.motor( 60, m.diff(60) );
       				break;
       			   case 0xe0: /* OOO_ _xxx */
       				m.handle( -15 );
       				m.motor( m.diff(60), 60 );
       				break;
       			/* 6 */
       			 case 0x03: /* xxx_ _xOO */
                 case 0x01: /* xxx_ _xxO */
                	 m.handle( 20 );
                	 m.motor( 50, m.diff(50) );
       				 cnt1 = 0;
       				 pattern = 20;
       				 LR = 'R';
       			break;
                case 0xc0: /* OOx_ _xxx */
                case 0x80: /* Oxx_ _xxx */
                	m.handle( -20 );
                	m.motor( m.diff(50), 50 );
       				cnt1 = 0;
       				pattern = 20;
       				LR = 'L';
                break;
                default:
                break;
                }
            break;
         	case 20:  /* Big curve */
            	if( crank && cntGate > 2000){
            		pattern = 30;
            	}
            	if( lane_half != -1 ){
            		pattern = 50;
            		break;
            	}
       		if( LR == 'L' ){
         	        if( c.sensor_inp8(MASK3_3) == 0x60 ) {
         	            pattern = 11;
         	        }
         		}
         		if( LR == 'R' ){
         	        if( c.sensor_inp8(MASK3_3) == 0x06 ) {
         	            pattern = 11;
         	        }
         		}
         		break;
         	case 10:
 /*        		if( cnt1 > 500){
         			pattern =11;
         			cnt1=0;
         			cntGate = 0;
         		}
         		m.handle(0);
         		m.run( 50 );
 */
         		if(bar == 0){
         			wait(0.25);
         			cntUnder = 2000;
         			pattern = 11;
         		}
                if( cnt1 < 100 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 1000 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
         		break;

         	case 11:
            	if( crank && cntGate > 500 && underPass == 0){
            		c.y_center = 20;
            		pattern = 30;
            		cnt1 = 0;
                    break;
            	}
            	if( lane_half != -1 && cntGate > 1000  && cntUnder > 1700  /*&& LeneChange_state == 0*/){
            		LeneChange_count++;
            		if( LeneChange_count == 5){
            			c.y_center = 20;
            			pattern = 50;
            			break;
            		}
            	}

            	if(c.SenVal_Center < 0) sp = -c.SenVal_Center;
            	else				  sp = c.SenVal_Center;

            	if( sp < 5){
            		c.y_center = 20;
            		sp = 0;
             	}else if(sp <10){
            		sp = sp /2;
            		c.y_center = 20;
             	}else if(sp < 12){
                 	sp = sp*3/2;
                 	c.y_center = 20;
             	}else if(sp < 15){
                 	sp = sp*2;
                 	c.y_center = 20;
             	}else{
              		c.y_center = 25;
              		sp = sp*3;
            		cnt1 = 0;
//                	if(c.SenVal_Center < 0)  m.handle(-40);
//                	else				  m.handle(40);;

                	if(cntGate>2000)
            		pattern = 22;
            		cnt_curve = 0;
//            		break;
             	}

            	m.handle(pidValue -1);
            	m.run( 100 - sp );
         		break;

         	case 22:  /* Big curve */

         		if(cnt_curve > 10)m.run( 70 );
         		else m.run( -50 );

            	if( crank && cntGate > 500 && underPass == 0){
            		pattern = 30;
            	}
            	if( lane_half != -1 && LeneChange_state == 0){
//            		pattern = 50;
            		break;
            	}
//            	if(cnt1 > 100)m.run(100);

         	   if(c.SenVal_Center < 5 && c.SenVal_Center > -5 && c.SenVal_Center != 0 ) {
         	            pattern = 11;
         	   }
           		break;
            case 30:
                led_m_set( CRANK );
                led_out( 0x1 );
                m.handle(c.SenVal_Center);
                m.motor(-100,-100);
//                if( cnt1 > mem_crk[n_lr] ) {
                if( cnt1 > 150 ) {
                    pattern = 31;
                    cnt1 = 0;
                }
                break;
            case 31:
                if( crank_turn == 1){
                        LR = 'L';
                        pattern = 33;
                        cnt1 = 0;
                        cnt_dammy = 0;
                        fall_flag = 1;
                        break;
               }
               if( crank_turn == 2){
                        LR = 'R';
                        pattern = 33;
                        cnt1 = 0;
                        cnt_dammy = 0;
                        fall_flag = 1;
                        break;
               }
               m.handle(c.SenVal_Center);
               m.motor2(30,30);
            break;
            case 32:
                if( crank_turn == -1 || cnt_dammy > 2000){
                        pattern = 33;
                        cnt1 = 0;
                        fall_flag = 1;
                        break;
               }
                m.handle(c.SenVal_Center);
                m.motor2(25,25);
            break;

            case 33:
                if(LR == 'L'){
                    /* Left crank */
                	m.handle( -45 );
                	m.motor2( 0,50 );
                    if( c.sensor_inp8(MASK0_2) && cnt1 > 500){
                    	pattern = 11;
                    	cnt1 = 0;
                    }
                }else{
                    /* Right crank */
                	m.handle( 45 );
                	m.motor2( 60,0 );
                    if( c.sensor_inp8(MASK2_0) && cnt1 > 500){
                    	pattern = 11;
                    	cnt1 = 0;
                    	fall_flag = 0;
                    }
                }
            break;


 /* Lane change processing at 1st process */
            case 50:
            	m.handle( 0 );
            	m.motor(-30,-30);
//            	m.run( 0 );
//                LR = mem_lr[n_lr];
                if( lane_half == 1){
                        LR = 'L';
               }
               if( lane_half == 2){
                        LR = 'R';
               }
               pattern = 51;
               fall_flag = 1;
            break;
            case 51:
                led_m_set( LCHANGE );
                led_out( 0x01 );
                if( LR == 'L' ) m.handle(c.SenVal_Center -10);
                else            m.handle(c.SenVal_Center +10);
                m.run2(30);
/*
            	if( crank ){
            		pattern = 30;
            		cnt1 = 0;
                    break;
            	}
*/
            	if( lane_Black ){
					if(LR == 'L'){
						/* Right lane change */
						m.handle( -25 );
					}else{
						/* Left lane change */
						m.handle( 25 );
					}
					pattern = 52;
					cnt1 = 0;
                }
                break;

            case 52:
                /* Lane change processing at 2nd process */
                if( cnt1 > 350 ) {
                    if(LR == 'L'){
                        /* Left lane change */
                    	m.handle( -8 );
                    	m.run2( 15 );
//                    	m.motor2( 30,20 );
                    }else{
                        /* Right lane change */
                    	m.handle( 8);
                    	m.run2( 15 );
//                    	m.motor2( 20,30 );
                    }
                    pattern = 525;
                    cnt1 = 0;
                }
                break;

            case 525:
            	if( LR == 'L'){
            		if(c.SenVal_Center < -10 && c.SenVal_Center > -20 && c.SenVal_Center != 0){
            			m.handle( 0 );
            			m.run2( 35 );
            		}
            		if(c.SenVal_Center < 5 && c.SenVal_Center > -10 && c.SenVal_Center != 0){
            			pattern = 53;
            		}

            	}
            	if( LR == 'R'){
            		if(c.SenVal_Center < 20 && c.SenVal_Center > 10 && c.SenVal_Center != 0){
                    	m.handle( 0 );
                    	m.run2( 35);
            		}
               		if(c.SenVal_Center < 10 && c.SenVal_Center > 5 && c.SenVal_Center != 0){
               			pattern = 53;
               		}
            	}
            break;

            case 53:
                /* lane change end check */
            	if(c.SenVal_Center < 5 && c.SenVal_Center > -5 && c.SenVal_Center != 0){
            		led_m_set( RUN );
            		led_out( 0x0 );
            		pattern = 54;
            		n_lr++;
            		if(mem_lr[n_lr] == 'e') n_lr = 0;
            		cnt1 = 0;
		            fall_flag = 0;
            	}

                break;

            case 54:
            	m.handle(c.SenVal_Center *2);
            	m.motor2( 50 , 50 );
            	if(cnt1 > 1200){
            		LeneChange_state = 1;
            		LeneChange_count = 0;
            		LeneChange_time++;
            		pattern = 11;
            	}
            break;

            case 200:
            	m.motor(0,0);
            	m.handle( 0 );
                if( cnt1 < 30 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 60 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
            	break;

            case 1000: //Out of log data
            	fall_flag = 1;
                led_out( 0x3 );
                m.motor(0,0);
                m.handle( 0 );
                wait(0.5);
                led_out( 0x0 );
                wait(2.0);
                pattern = 1010;
                break;
            case 1010:
                if( pushsw_get() ) {
                    pattern = 1020;
                    m_number = 0;
                    led_out( 0x3 );
                    break;
                }
                if( cnt1 < 300 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 300*2 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
                break;
            case 1020:
                pc.printf("%d,%4d,  %d,%4d,%6d,%4d\r\n",m_number,memory[m_number][0],memory[m_number][1],memory[m_number][2],memory[m_number][3],memory[m_number][4]);
                m_number++;
                if(m_number > 10000) pattern = 1030;
                break;
            case 1030:
                led_out(0x00);
                break;

            default:
                break;
        }
    }
}

//------------------------------------------------------------------//
//Initialize Camera function
//------------------------------------------------------------------//
void init_Camera( void )
{
    /* NTSC-Video */
    DisplayBase::graphics_error_t error;

    /* Graphics initialization process */
    error = Display.Graphics_init(NULL);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_VDEC, NULL);
    if( error != DisplayBase::GRAPHICS_OK ) {
        while(1);
    }

    /* Interrupt callback function setting (Vsync signal input to scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0, IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video capture setting (progressive form fixed) */
    error = Display.Video_Write_Setting(
                VIDEO_INPUT_CH,
                DisplayBase::COL_SYS_NTSC_358,
                write_buff_addr,
                VIDEO_BUFFER_STRIDE,
                DisplayBase::VIDEO_FORMAT_YCBCR422,
                DisplayBase::WR_RD_WRSWA_32_16BIT,
                PIXEL_VW,
                PIXEL_HW
            );
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Interrupt callback function setting (Field end signal for recording function in scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video write process start */
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video write process stop */
    error = Display.Video_Stop (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video write process start */
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Wait vsync to update resister */
    WaitVsync(1);

    /* Wait 2 Vfield(Top or bottom field) */
    WaitVfield(2);
}

//------------------------------------------------------------------//
//ChangeFrameBuffer function
//------------------------------------------------------------------//
void ChangeFrameBuffer( void )
{
    /* NTSC-Video */
    DisplayBase::graphics_error_t error;

    /* Change write buffer */
    if (write_buff_addr == FrameBuffer_Video_A) {
        write_buff_addr = FrameBuffer_Video_B;
        save_buff_addr  = FrameBuffer_Video_A;
    } else {
        write_buff_addr = FrameBuffer_Video_A;
        save_buff_addr  = FrameBuffer_Video_B;
    }
    error = Display.Video_Write_Change(
                VIDEO_INPUT_CH,
                write_buff_addr,
                VIDEO_BUFFER_STRIDE);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type)
{
    if (vfield_count > 0) {
        vfield_count--;
    }
    /* top or bottom (Change) */
    if( vfield_count2 == 0 )  vfield_count2 = 1;
    else                      vfield_count2 = 0;
    led_m_user( vfield_count2 );
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVfield(const int32_t wait_count)
{
    vfield_count = wait_count;
    while (vfield_count > 0) {
        /* Do nothing */
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function for Vsync interruption
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type)
{
    if (vsync_count > 0) {
        vsync_count--;
    }
}

static void WaitVsync(const int32_t wait_count)
{
    vsync_count = wait_count;
    while (vsync_count > 0) {
        /* Do nothing */
    }
}

//------------------------------------------------------------------//
// Interrupt function( intTimer )
//------------------------------------------------------------------//
void intTimer( void )
{
    static int      counter = 0;    /* Only variable for image process */
    int i;

    cnt0++;
    cnt1++;
    cntGate++;
    cnt_curve++;
    cnt_dammy++;
    cntFaling++;
    cntUnder++;
    if(cntUnder > 2000)cntUnder = 2000;

    /* field check */
    if( vfield_count2 != vfield_count2_buff ) {
        vfield_count2_buff = vfield_count2;
        counter = 0;
        // Servo PWM counter clear
//        MTU2TCNT_0 = 0;
    }

    /* Top field / bottom field */
    switch( counter++ ) {
        case 0:
            ImageCopy( write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2 );
            break;
        case 1:
            ImageCopy( write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2 );
            break;
        case 2:
            Extraction_Brightness( ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2 );
            break;
        case 3:
            Extraction_Brightness( ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2 );
            break;
        case 4:
            ImageReduction( ImageData_B, PIXEL_HW, PIXEL_VW, ImageComp_B, Rate );
            break;
        case 5:
            ImageReduction( ImageData_B, PIXEL_HW, PIXEL_VW, ImageComp_B, Rate );
            break;
        case 6:
            Binarization( ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate), c.ImageBinary, threshold_buff );
            bar = c.StartBarCheck();
//            pc.printf( "bar = %d\n\r\n\r",bar);
            break;
        case 7:
            if( !initFlag ) c.SenVal8	= c.sensor_process8();
            if( !initFlag ) c.SenVal_Center	= c.sensor_process_Center();
            if( !initFlag ) pidValue	= c.PID_process();
            if( !initFlag ) handle_value +=pidValue;
          break;
        case 8:
            crank_turn = c.Crank_Turn_Point();
            crank = c.Crank_Mark_Check();
            break;
        case 9:
            lane_half = c.LaneChangeHalf();
 //           for( i=15;i<30;i++)pc.printf( "width[%d] = %d Start=%d Stop=%d\n\r", i,c.width[i],c.Start[i],c.Stop[i] );
 //          for( i=10;i<30;i++)pc.printf( "width[%d] = %d \n\r", i,c.width[i] );
 //          pc.printf( "lane_half = %d  width_Max = %d \n\r\n\r",lane_half,c.width_Max);
            lane_Black = c.LaneChangeBlack();
           break;
        case 10:
            /* Rotary encoder process */
            i = MTU2TCNT_1;
            if( ( MTU2TSR_1 & 0x90 ) == 0x90 ) {
                // Overflow ( TCFD:1 TCFV:1 )
                // Flag clear
                MTU2TSR_1 &= 0xef;
                iEncoder   = ( i - uEncoderBuff ) + 65535;
            } else if( ( MTU2TSR_1 & 0x20 ) == 0x20 ) {
                // Underflow ( TCFD:0 TCFU:1 )
                // Flag clear
                MTU2TSR_1 &= 0xdf;
                iEncoder   = ( i - uEncoderBuff ) - 65535;
            } else {
                iEncoder   = i - uEncoderBuff;
            }
            lEncoderTotal += iEncoder;
            uEncoderBuff   = i;

            break;
        case 11:
            break;
        case 12:
           if(pattern > 10 && pattern < 1000) {
                memory[m_number][0] = pattern;
//                memory[m_number][1] = c.sensor_inp8(MASK4_4);
                memory[m_number][1] = c.SenVal_Center;
//                memory[m_number][2] = c.wide;
//                memory[m_number][2] = underPass;cntGate
                memory[m_number][2] = cntGate;
//                memory[m_number][2] = LeneChange_count;
                memory[m_number][3] = cntUnder;
//                memory[m_number][3] = c.width_Max;
                memory[m_number][4] =lane_half;
                m_number++;
                if(m_number > 10000)m_number = 10000;
            }
            break;
        case 13:
            threshold_buff = Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
//	          pc.printf( "threshold = %d Threshold_Ave = %d \n\r\n\r",threshold_buff,Threshold_Ave);

           if(threshold_buff - Threshold_Ave > -15 ){
        	   Threshold_value[k] = threshold_buff;
        	   k++;
        	   if(k > 50)k = 0;

        	   Threshold_Ave = 0;
        	   for(i=0;i < 50;i++){
//		       		pc.printf( "Threshold_value[%d] = %d \n\r",i,Threshold_value[i]);
        		   Threshold_Ave +=Threshold_value[i];
            	}
            	Threshold_Ave /= 50;
//	          	pc.printf( "clc \n\r\n\r");
            		underPass = 0;
           }else{
        	   if(LeneChange_time == 1){
        		   cntUnder = 0;
        		   underPass = 1;
        	   }
           }

            break;
        case 14:
        	if( fall_flag == 0 && c.wide == 99 && lane_Black == 1){
        		cntFaling++;

        	}else{
        		cntFaling = 0;
        	}
        	if(cntFaling > 1000 && pattern > 10)pattern = 200;

        	// Servo PWM counter clear
//            MTU2TCNT_0 = 0;

        	break;
        case 15:

        	break;
        default:
            break;
    }

    /* LED(rgb) on the GR-peach board */
    led_m_process();
}

//******************************************************************//
// functions ( on GR-PEACH board )
//*******************************************************************/
//------------------------------------------------------------------//
//led_rgb Function
//------------------------------------------------------------------//
void led_rgb(int led)
{
    LED_R = led & 0x1;
    LED_G = (led >> 1 ) & 0x1;
    LED_B = (led >> 2 ) & 0x1;
}

//------------------------------------------------------------------//
//user_button_get Function
//------------------------------------------------------------------//
unsigned char user_button_get( void )
{
    return (~user_botton) & 0x1;        /* Read ports with switches */
}

//------------------------------------------------------------------//
//led_m_user Function
//------------------------------------------------------------------//
void led_m_user( int led )
{
    USER_LED = led & 0x01;
}

//------------------------------------------------------------------//
//led_m_set Function
//------------------------------------------------------------------//
void led_m_set( int set )
{
    led_pattern = set;
}

//------------------------------------------------------------------//
//led_m_process Function for only interrupt
//------------------------------------------------------------------//
void led_m_process( void )
{
    int                     led_color;
    int                     onTime;
    int                     offTime;
    static unsigned long    cnt_led_m;

    switch( led_pattern ) {
        case RUN:
            led_color = LED_GREEN;
            onTime    = 500;
            offTime   = 500;
            break;
        case STOP:
            led_color = LED_RED;
            onTime    = 500;
            offTime   = 0;
            break;
        case ERROR:
            led_color = LED_RED;
            onTime    = 100;
            offTime   = 100;
            break;
        case DEBUG:
            led_color = LED_BLUE;
            onTime    = 50;
            offTime   = 50;
            break;
        case CRANK:
            led_color = LED_YELLOW;
            onTime    = 500;
            offTime   = 500;
            break;
        case LCHANGE:
            led_color = LED_BLUE;
            onTime    = 500;
            offTime   = 500;
            break;
        default:
            led_color = LED_OFF;
            onTime    = 500;
            offTime   = 500;
            break;
    }

    cnt_led_m++;

    /* Display */
    if( cnt_led_m < onTime ) led_rgb( led_color );
    else if( cnt_led_m < ( onTime + offTime ) ) led_rgb( LED_OFF );
    else cnt_led_m = 0;
}

//******************************************************************//
// functions ( on Motor drive board )
//*******************************************************************/
//------------------------------------------------------------------//
//led_out Function
//------------------------------------------------------------------//
void led_out(int led)
{
    led = ~led;
    LED_3 = led & 0x1;
    LED_2 = ( led >> 1 ) & 0x1;
}

//------------------------------------------------------------------//
//pushsw_get Function
//------------------------------------------------------------------//
unsigned char pushsw_get( void )
{
    return (~push_sw) & 0x1;            /* Read ports with switches */
}

//******************************************************************//
// Debug functions
//*******************************************************************/
//------------------------------------------------------------------//
//Image Data Output( for the Excel )
//------------------------------------------------------------------//
void ImageData_Serial_Out1( unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp, inc;

    for( Yp = 0, inc = 0; Yp < VW; Yp++ ) {
        for( Xp = 0; Xp < HW; Xp++, inc++ ) {
            pc.printf( "%d,", ImageData[ inc ] );
        }
        pc.printf("\n\r");
    }
}

//------------------------------------------------------------------//
//Image Data Output2( for TeraTerm )
//------------------------------------------------------------------//
void ImageData_Serial_Out2( unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     i;

    for( Yp = 0; Yp < VW; Yp +=2 ) {
        for( Xp = 0; Xp < HW; Xp +=2 ) {
            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
        }
        pc.printf( "\n\r" );
    }

    //Add display
    pc.printf( "\n\r" );
//    pc.printf( "sensor_inp = 0x%2x\n\r", c.sensor_inp8(MASK4_4) );
//      pc.printf( "wide = %3d\n\r",c.wide );
      pc.printf( "lEnc = %5d %5d\n\r",iEncoder,lEncoderTotal);

      //      pc.printf( "dipsw = %3d\n\r", m.sw_data );
//      pc.printf( "bar = %3d\n\r",bar );
    pc.printf( "pidValue = %3d\n\r", pidValue );
//    pc.printf( "handle_value %3d\n\r",handle_value);
    pc.printf( "Center = %3d\n\r", c.SenVal_Center );
//    pc.printf( "LaneChangeHalf %3d\n\r",c.LaneChangeHalf());
//    pc.printf( "LaneChangeBlack %d\n\r",c.LaneChangeBlack());
//    pc.printf( "center_inp = 0x%02x\n\r", center_inp() );
//    pc.printf( "threshold= %d\n\r", Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate)));
//      pc.printf( "RightCrank      = %01d, = %3d%%, X = %2d, Y = %2d\n\r", RightCrankCheck(80), RightCrank.p, RightCrank.x, RightCrank.y );
    pc.printf( "lane_black = %3d\n\r",c.LaneChangeBlack());
    pc.printf( "lane_Half = %2d\n\r",c.LaneChangeHalf());

//    pc.printf( "RightLaneChange = %01d, = %3d%%, X = %2d, Y = %2d\n\r", RightLaneChangeCheck(80), RightLaneChange.p, RightLaneChange.x, RightLaneChange.y );
//    pc.printf( "LeftCrank      = %01d, = %3d%%, X = %2d, Y = %2d\n\r", LeftCrankCheck(80), LeftCrank.p, LeftCrank.x, LeftCrank.y );
//    pc.printf( "LeftLaneChange = %01d, = %3d%%, X = %2d, Y = %2d\n\r", LeftLaneChangeCheck(80), LeftLaneChange.p, LeftLaneChange.x, LeftLaneChange.y );
//    pc.printf( "Crank_Mark_Check %d\n\r",c.Crank_Mark_Check());
//        pc.printf( "CrankturnPoint %d\n\r",c.Crank_Turn_Point());
    pc.printf( "\n\r" );
    VW += 6;

//    pc.printf( "Threshold Check ( please user button on GR-peach board )\n\r" );
    pc.printf( "\n\r" );
    VW += 2;

    if( user_button_get() ) {
        pc.printf( "Please Threshold\n\r" );
        pc.printf( " = " );
        pc.scanf( "%d", &i );
        pc.printf( "\n\r" );
        threshold_buff = i;
        VW += 2;
    }

    pc.printf( "\033[%dA", VW );

    if(handle_value>40*15)handle_value=40*15;
    if(handle_value<-40*15)handle_value=-40*15;
    m.handle(handle_value/15);
}

//------------------------------------------------------------------//
//Image Data Output3( for TeraTerm )
//------------------------------------------------------------------//
void ImageData_Serial_Out3( unsigned char *ImageData, int HW, int VW, int color_pattern )
{
    int X, Y;
    int Px, Py;
    int HW_T;//HW Twice
    int value;

    HW_T = HW + HW;

    /* Camera module test process 2 */
    pc.printf( "//,X-Size,Y-Size" );
    pc.printf( "\n\r" );
    pc.printf( "#SIZE,%3d,%3d", HW, VW );
    pc.printf( "\n\r" );
    pc.printf( "//,X-Point,Y-Point" );
    pc.printf( "\n\r" );

    switch( color_pattern ) {
        case COLOR:
            //YCBCR_422 Color
            for( Py = 0, Y = 0; Py < VW; Py+=1, Y++ ) {
                for( Px = 0, X = 0; Px < HW_T; Px+=4, X+=2 ) {
                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X);
                    /*Yp*/pc.printf( "%d,", Y);
                    /*Y0*/pc.printf( "%d,", ImageData[ ( Px + 0 ) + ( HW_T * Py ) ] );//6
                    /*Cb*/pc.printf( "%d,", ImageData[ ( Px + 1 ) + ( HW_T * Py ) ] );//5
                    /*Cr*/pc.printf( "%d,", ImageData[ ( Px + 3 ) + ( HW_T * Py ) ] );//7
                    pc.printf( "\n\r" );

                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X+1);
                    /*Yp*/pc.printf( "%d,", Y);
                    /*Y1*/pc.printf( "%d,", ImageData[ ( Px + 2 ) + ( HW_T * Py ) ] );//4
                    /*Cb*/pc.printf( "%d,", ImageData[ ( Px + 1 ) + ( HW_T * Py ) ] );//5
                    /*Cr*/pc.printf( "%d,", ImageData[ ( Px + 3 ) + ( HW_T * Py ) ] );//7
                    pc.printf( "\n\r" );
                }
            }
            break;
        case GREY_SCALE:
            //YCBCR_422 GreyScale
            for( Y = 0; Y < VW; Y++ ) {
                for( X = 0; X < HW; X+=2 ) {
                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X);
                    /*Yp*/pc.printf( "%d,", Y);
                    /*Y0*/pc.printf( "%d,", ImageData[ ( Y * HW ) + ( X + 0 ) ] );//6
                    /*Cb*/pc.printf( "%d,", 128);//5
                    /*Cr*/pc.printf( "%d,", 128);//7
                    pc.printf( "\n\r" );

                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X+1);
                    /*Yp*/pc.printf( "%d,", Y);
                    /*Y1*/pc.printf( "%d,", ImageData[ ( Y * HW ) + ( X + 1 ) ] );//4
                    /*Cb*/pc.printf( "%d,", 128);//5
                    /*Cr*/pc.printf( "%d,", 128);//7
                    pc.printf( "\n\r" );
                }
            }
            break;
        case BINARY:
            //YCBCR_422 Binary
            for( Y = 0; Y < VW; Y++ ) {
                for( X = 0; X < HW; X+=2 ) {
                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X);
                    /*Yp*/pc.printf( "%d,", Y);
                    value = ImageData[ ( Y * HW ) + ( X + 0 ) ];
                    if( value ) value = 255;
                    else        value = 0;
                    /*Y0*/pc.printf( "%d,", value );//6
                    /*Cb*/pc.printf( "%d,", 128);//5
                    /*Cr*/pc.printf( "%d,", 128);//7
                    pc.printf( "\n\r" );

                    pc.printf( "#YCbCr," );
                    /*Xp*/pc.printf( "%d,", X+1);
                    /*Yp*/pc.printf( "%d,", Y);
                    value = ImageData[ ( Y * HW ) + ( X + 1 ) ];
                    if( value ) value = 255;
                    else        value = 0;
                    /*Y1*/pc.printf( "%d,", value );//4
                    /*Cb*/pc.printf( "%d,", 128);//5
                    /*Cr*/pc.printf( "%d,", 128);//7
                    pc.printf( "\n\r" );
                }
            }
            break;
        default:
            break;
    }
    pc.printf( "End\n\r" );
}
//------------------------------------------------------------------//
//Image Data Output2( for TeraTerm )
//------------------------------------------------------------------//
void ImageData_Serial_Out4( unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;

    for( Yp = 0; Yp < VW; Yp++ ) {
        pc.printf( "%2d: ", Yp );
        for( Xp = 0; Xp < HW; Xp++ ) {
            pc.printf( "%3d ", ImageData[Xp + (Yp * HW)] );
        }
        pc.printf( "\n\r" );
    }
    pc.printf( "\033[%dA", VW );
}
//----------------------------------------------------------------------//
//MTU2_1
//Phase coefficient mode
//TCLKA(P1_0 ) :Pulse A
//TCLKB(P1_10) :Pulse B
//----------------------------------------------------------------------//
void init_MTU2_RotaryEncoder( void )
{
    /* Port setting for S/W I/O Contorol */
    GPIOPIBC1  &= 0xfbfe;               /* Input buffer prohibition     */
    GPIOPBDC1  &= 0xfbfe;               /* Interactive mode prohibition */
    GPIOPM1    |= 0x0401;               /* P1_0,P1_10:Input mode        */
    GPIOPMC1   &= 0xfbfe;               /* P1_0,P1_10:Port mode         */
    GPIOPIPC1  &= 0xfbfe;               /* S/W I/O control mode         */

    GPIOPFC1   &= 0xfffe;               /* Choice of the pin function   */
    GPIOPFC1   |= 0x0400;               /* Choice of the pin function   */
    GPIOPFCE1  |= 0x0401;               /* Choice of the pin function   */
    GPIOPFCAE1 &= 0xfbfe;               /* Choice of the pin function   */

    GPIOPIPC1  |= 0x0401;               /* Direct I/O control mode      */
    GPIOPMC1   |= 0x0401;               /* P1_0,P1_10:Double mode       */
                                        /* TCLKA(P1_0), TCLKB(P1_10)    */

    /* Mosule stop 33(MTU2) canceling */
    CPGSTBCR3  &= 0xf7;

    /* Rotary Encoder          */
    /* MTU2_1 A(P1_0) B(P1_10) */
    MTU2TCR_1   = 0x14;                 /* TCLKA(P1_0), Both edge count */
                                        /* Ignored, when phase coefficient mode. */
    MTU2TMDR_1  = 0x05;                 /* 0x02:PWM mode 1              */
                                        /* 0x04:Phase coefficient mode 1*/
    MTU2TSTR   |= 0x02;                 /* TCNT_1 Start                 */
}
//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
