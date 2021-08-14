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

//------------------------------------------------------------------//
//Define
//------------------------------------------------------------------//
//Motor PWM cycle
#define     MOTOR_PWM_CYCLE     33332   /* Motor PWM period         */
/* 1ms    P0ﾏ�/1  = 0.03us   */
//Servo PWM cycle
#define     SERVO_PWM_CYCLE     33332   /* SERVO PWM period         */
/* 16ms   P0ﾏ�/16 = 0.48us   */
#define     SERVO_CENTER        3100    /* 1.5ms / 0.48us - 1 = 3124*/
#define     HANDLE_STEP         18      /* 1 degree value           */

#define     THRESHOLD           180     /* Binarization function only */
//#define     CLK_SP              25

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
#define PIXEL_HW               (160u)  /* QVGA */
#define PIXEL_VW               (120u)  /* QVGA */
#define VIDEO_BUFFER_STRIDE    (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT    (PIXEL_VW)

/* 繝槭せ繧ｯ蛟､險ｭ螳� ﾃ暦ｼ壹�槭せ繧ｯ縺ゅｊ(辟｡蜉ｹ)縲�笳具ｼ壹�槭せ繧ｯ辟｡縺�(譛牙柑) */
#define MASK2_2         0x66            /* ﾃ冷雷笳凝療冷雷笳凝�             */
#define MASK4_3         0xfe            /* 笳銀雷笳銀雷笳銀雷笳凝�             */
#define MASK3_4         0x7f            /* ﾃ冷雷笳銀雷笳銀雷笳銀雷             */
#define MASK1_0         0x80            /* 笳凝療療療療療療�             */
#define MASK0_1         0x01            /* ﾃ療療療療療療冷雷             */
#define MASK1_1         0x81            /* 笳凝療療療療療冷雷             */
#define MASK2_0         0x60            /* ﾃ冷雷笳凝療療療療�             */
#define MASK0_2         0x06            /* ﾃ療療療療冷雷笳凝�             */
#define MASK3_3         0xe7            /* 笳銀雷笳凝療冷雷笳銀雷             */
#define MASK0_3         0x07            /* ﾃ療療療療冷雷笳銀雷             */
#define MASK3_0         0xe0            /* 笳銀雷笳凝療療療療�             */
#define MASK4_0         0xf0            /* 笳銀雷笳銀雷ﾃ療療療�             */
#define MASK0_4         0x0f            /* ﾃ療療療冷雷笳銀雷笳�             */
#define MASK4_4         0xff            /* 笳銀雷笳銀雷笳銀雷笳銀雷             */
#define MASK_5_			0x7e			/* ﾃ冷雷笳銀雷笳銀雷笳凝�             */
#define MASK_2_			0x18			/* ﾃ療療冷雷笳凝療療�             */

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

//DigitalOut  Left_motor_signal(P4_6);    /* Used by motor function   */
//DigitalOut  Right_motor_signal(P4_7);   /* Used by motor function   */
DigitalIn   push_sw(P2_13);             /* SW1 on the Motor Drive board */
DigitalOut  LED_3(P2_14);               /* LED3 on the Motor Drive board */
DigitalOut  LED_2(P2_15);               /* LED2 on the Motor Drive board */

Drive m;

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
void init_MTU2_PWM_Motor( void );       /* Initialize PWM functions */
void init_MTU2_PWM_Servo( void );       /* Initialize PWM functions */

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
//void motor( int accele_l, int accele_r );
//void motor2( int accele_l, int accele_r );
//void handle( int angle );
int diff( int pwm );

//Shield board
unsigned char dipsw_get( void );

//------------------------------------------------------------------//
//Prototype( Digital sensor process )
//------------------------------------------------------------------//
int init_sensor( unsigned char *BuffAddrIn, int HW, int VW, int Cx, int *SENPx, int Y );
unsigned char sensor_process( unsigned char *BuffAddrIn, int HW, int VW, int *SENPx, int Y ); /* Only function for interrupt */
unsigned char sensor_process8( unsigned char *BuffAddrIn, int HW, int VW, int *SENPx, int Y ); /* Only function for interrupt */
int sensor_process_Center( unsigned char *ImageData, int HW, int VW, int *SENPx, int Y );
unsigned char sensor_inp( void );
unsigned char sensor_inp8( unsigned char mask );
unsigned char center_inp( void );

//------------------------------------------------------------------//
//Prototype( Mark detect functions )
//------------------------------------------------------------------//
int StartBarCheck(unsigned char *ImageData, int HW, int VW);
int Crank_Turn_Point(unsigned char *ImageData, int HW, int VW );
int Crank_Mark_Check( unsigned char *ImageData, int HW, int VW);
int CrankCheck(unsigned char *ImageData, int HW, int VW );
int LaneChangeBlack(unsigned char *ImageData, int HW, int VW);
int LaneChangeHalf(unsigned char *ImageData, int HW, int VW);
int RightCrankCheck( int percentage );
int RightLaneChangeCheck( int percentage );
int LeftCrankCheck( int percentage );
int LeftLaneChangeCheck( int percentage );
//------------------------------------------------------------------//
//Prototype( Debug functions )
//------------------------------------------------------------------//
void ImageData_Serial_Out1( unsigned char *ImageData, int HW, int VW );
void ImageData_Serial_Out2( unsigned char *ImageData, int HW, int VW );
void ImageData_Serial_Out3( unsigned char *ImageData, int HW, int VW, int color_pattern );
void ImageData_Serial_Out4( unsigned char *ImageData, int HW, int VW );

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
unsigned char   ImageBinary[ ( PIXEL_HW * PIXEL_VW ) ];
//unsigned char   ImageBinary_all[ ( PIXEL_HW * PIXEL_VW ) ];

//double          Rate = 0.125;       /* Reduction ratio              */
double          Rate = 0.25;       /* Reduction ratio              */

//------------------------------------------------------------------//
//Global variable for Digital sensor function
//------------------------------------------------------------------//
int             SenError;
int             Sen1Px[5];
unsigned char   SenVal1,SenVal8;
int				SenVal_Center;
int wide;
//------------------------------------------------------------------//
//Global variable for Mark detection function
//------------------------------------------------------------------//
ImagePartPattern RightCrank = {0,0,0,0,{0},   //percent, Point X, Point Y, Standard_Deviation, Deviation
    {
        0,0,0,0,0,0,0, //Binary image
        0,0,1,1,1,1,1, //Binary image
        0,0,1,1,0,0,0, //Binary image
        0,0,1,1,0,0,0
    },//Binary image
    7, 4
};         //Binary Width pixel, Binary Height pixel

ImagePartPattern RightLaneChange = {0,0,0,0,{0},//percent, Point X, Point Y, Standard_Deviation, Deviation
    {
        1,1,0,0,0,0,0,0,0,0,0,0,0,0, //Binary image
        1,1,0,0,0,0,0,0,0,0,0,0,0,0, //Binary image
        1,1,0,0,0,0,0,0,0,0,0,0,0,0
    },//Binary image
    14, 3
};   //Binary Width pixel, Binary Height pixel

ImagePartPattern LeftCrank = {0,0,0,0,{0},   //percent, Point X, Point Y, Standard_Deviation, Deviation
    {
        0,0,0,0,0,0,0, //Binary image
        1,1,1,1,1,0,0, //Binary image
        0,0,0,1,1,0,0, //Binary image
        0,0,0,1,1,0,0
    },//Binary image
    7, 4
};         //Binary Width pixel, Binary Height pixel

ImagePartPattern LeftLaneChange = {0,0,0,0,{0},//percent, Point X, Point Y, Standard_Deviation, Deviation
    {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0, //Binary image
        0,0,0,0,0,0,0,0,0,0,0,1,1,0, //Binary image
        0,0,0,0,0,0,0,0,0,0,0,1,1,0
    },//Binary image
    14, 3
};   //Binary Width pixel, Binary Height pixel

//------------------------------------------------------------------//
//Global variable for Trace program
//------------------------------------------------------------------//
volatile unsigned long  cnt0;           /* Used by timer function   */
volatile unsigned long  cnt1;           /* Used within main         */
volatile unsigned long  cntGate;           /* Used within main         */
volatile long  cnt_curve;           /* Used within main         */
volatile long  cnt_dammy;           /* Used within main         */
volatile int            pattern;        /* Pattern numbers          */

volatile int            led_pattern;    /* led_m_process function only */
volatile int            initFlag;       /* Initialize flag          */
volatile int            threshold_buff; /* Binarization function only */
//volatile int            handle_buff;    /* diff function only       */

int                     memory[10000][5];
int                     m_number;
int                     cr = 0;
int                     bar;
int                     crank,crank2,crank_turn,lane_half,lane_Black;
char                    LR;             //繧ｯ繝ｩ繝ｳ繧ｯ縲√Ξ繝ｼ繝ｳ繝√ぉ繝ｳ繧ｸ縺後←縺｡繧峨°
char                    mem_lr[] = {'R','R','R','L','R','e'};     //繧ｯ繝ｩ繝ｳ繧ｯ縲√Ξ繝ｼ繝ｳ繝√ぉ繝ｳ繧ｸ縺ｮ險俶�ｶ
int                     mem_crk[] = {0,300,290,0,260,-1};           //繧ｯ繝ｩ繝ｳ繧ｯ縺ｮ繝悶Ξ繝ｼ繧ｭ蜉�
int                     n_lr = 0;           //繧ｯ繝ｩ繝ｳ繧ｯ縲√Ξ繝ｼ繝ｳ繝√ぉ繝ｳ繧ｸ縺ｮ繝昴ず繧ｷ繝ｧ繝ｳ
char					fall_flag;			//繧ｳ繝ｼ繧ｹ縺九ｉ閼ｱ霈ｪ繧呈､懃衍縺励↑縺�蝣ｴ蜷医��1縲阪��讀懃衍縺吶ｋ蝣ｴ蜷医��0縲�
const int revolution_difference[] = {   /* diff function only       */
    100, 98, 97, 95, 93,
    92, 90, 88, 87, 85,
    84, 82, 81, 79, 78,
    76, 75, 73, 72, 71,
    69, 68, 66, 65, 64,
    62, 61, 59, 58, 57,
    55, 54, 52, 51, 50,
    48, 47, 45, 44, 42,
    41, 39, 38, 36, 35,
    33
};

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

    /* Camera start */
    init_Camera();
    /* wait to stabilize NTSC signal (about 170ms) */
    wait(0.2);

    /* Initialize MCU functions */
    init_MTU2_PWM_Motor();
    init_MTU2_PWM_Servo();
    interrput.attach(&intTimer, 0.001);
    pc.baud(230400);

    /* Initialize Micon Car state */
    m.handle( 0 );
    m.motor( 0, 0 );
    led_out( 0x0 );
    led_m_set( STOP );

//    threshold_buff = Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
    threshold_buff = THRESHOLD;

    wait(0.5);

    /* Initialize Digital sensor */
//    SenError = init_sensor( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), (PIXEL_HW * Rate) / 2, Sen1Px, 12 );
/*
    if( SenError !=0 ) {
        led_m_set( ERROR );
        cnt1 = 0;
        while( cnt1 < 3000 ) {
            if( cnt1 % 200 < 100 ) {
                led_out( 0x3 );
            } else {
                led_out( 0x0 );
            }
        }
    }
*/
    /* Initialize Pattern Matching */
    RightCrank.sdevi = Standard_Deviation( RightCrank.binary, RightCrank.devi, RightCrank.w, RightCrank.h );
    RightLaneChange.sdevi = Standard_Deviation( RightLaneChange.binary, RightLaneChange.devi, RightLaneChange.w, RightLaneChange.h );

    LeftCrank.sdevi = Standard_Deviation(  LeftCrank.binary,  LeftCrank.devi,  LeftCrank.w,  LeftCrank.h );
    LeftLaneChange.sdevi = Standard_Deviation(  LeftLaneChange.binary,  LeftLaneChange.devi,  LeftLaneChange.w,  LeftLaneChange.h );

    initFlag = 0;                       /* Initialization end       */

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
                        ImageData_Serial_Out2( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) );
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
                    ImageData_Serial_Out3( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), BINARY );
                    break;
                case 6:
                    /* for TeraTerm(Real-time display) */
                	while( 1 ) {
                        ImageData_Serial_Out4( ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate) );
//                        SenVal8	= sensor_process8( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 12 );
//                        pc.printf( "threshold %3d \n\r", Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate)) );
                        pc.printf( "sensor_process8 %x \n\r", SenVal8 );

                	}
//                    break;
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
//        pc.printf( "pattern = %d\n\r",pattern );
        switch( pattern ) {
            case 0:
                /* wait for switch input */
                if( pushsw_get() ) {
                    led_out( 0x0 );
                    led_m_set( RUN );
                    pattern = 11;
                    cnt1 = 0;
                    cntGate = 0;
                    break;
                }
                if( cnt1 < 100 ) {
                    led_out( 0x1 );
                } else if( cnt1 < 200 ) {
                    led_out( 0x2 );
                } else {
                    cnt1 = 0;
                }
                break;
            case 1:/* 繧ｹ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｾ縺ｧ陦後￥ */
                if( bar == 1){
                    cnt1 = 0;
                    pattern = 2;
                }
                m.motor2( 20, 20 );
                switch( (sensor_inp()&0x0f) ) {
                    case 0x00:
                    	m.handle( 0 );
                        break;
                    case 0x02:
                    case 0x03:
                    	m.handle( 3 );
                        break;
                    case 0x04:
                    case 0x0c:
                    	m.handle( -3 );
                        break;
                    default:
                        break;
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
                    pattern = 3;
                    cnt1 = 0;
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

            case 21: /* 8bit trace */
            	if( crank && cntGate > 2000){
            		pattern = 30;
                        break;
            	}
            	if( lane_half != -1 ){
            		pattern = 50;
            		break;
            	}
            	switch( sensor_inp8(MASK3_3) ) {
        		/* 竭� */
            		case 0x00: /*ﾃ療療冷夢  笆ｲﾃ療療� */
            			m.handle( 0 );
            			m.motor( 100, 100 );
                    break;
               /* 竭｡ */
                   case 0x04: /*ﾃ療療冷夢  笆ｲ笳凝療� */
                	   m.handle( 3 );
                	   m.motor( 100, m.diff(100) );
                   break;
                   case 0x20: /*ﾃ療冷雷笆ｲ  笆ｲﾃ療療� */
                	   m.handle( -3 );
                	   m.motor( m.diff(100), 100 );
       				break;
       			/* 竭｢ */
                   case 0x06: /*ﾃ療療冷夢  笆ｲ笳銀雷ﾃ� */
                	   m.handle( 5 );
                	   m.motor( 80, m.diff(80) );
         		   break;
                   case 0x60: /*ﾃ冷雷笳銀夢  笆ｲﾃ療療� */
                	   m.handle( -5 );
                	   m.motor( m.diff(80), 80 );
        		   break;
        		/* 竭｣ */
                   case 0x02: /*ﾃ療療冷夢  笆ｲﾃ冷雷ﾃ� */
                	   m.handle( 10 );
                	   m.motor( 60, m.diff(60) );
                   break;
                   case 0x40: /*ﾃ冷雷ﾃ冷夢  笆ｲﾃ療療� */
                	   m.handle( -10 );
                	   m.motor( m.diff(60), 60 );
                    break;
               /* 竭､ */
                   case 0x07: /*ﾃ療療冷夢  笆ｲ笳銀雷笳� */
                	   m.handle( 15 );
                	   m.motor( 60, m.diff(60) );
       				break;
       			   case 0xe0: /*笳銀雷笳銀夢  笆ｲﾃ療療� */
       				m.handle( -15 );
       				m.motor( m.diff(60), 60 );
       			break;
       			/* 竭･ */
       			 case 0x03: /*ﾃ療療冷夢  笆ｲﾃ冷雷笳� */
                 case 0x01: /*ﾃ療療冷夢  笆ｲﾃ療冷雷 */
                	 m.handle( 20 );
                	 m.motor( 50, m.diff(50) );
       				 cnt1 = 0;
       				 pattern = 20;
       				 LR = 'R';
       			break;
                case 0xc0: /*笳銀雷ﾃ冷夢  笆ｲﾃ療療� */
                case 0x80: /*笳凝療冷夢  笆ｲﾃ療療� */
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
         	case 20:  /* 螟ｧ繧ｫ繝ｼ繝� */
            	if( crank && cntGate > 2000){
            		pattern = 30;
    //                    break;
            	}
            	if( lane_half != -1 ){
            		pattern = 50;
            		break;
            	}
       		if( LR == 'L' ){
         	        if( sensor_inp8(MASK3_3) == 0x60 ) {
         	            pattern = 11;
         	        }
         		}
         		if( LR == 'R' ){
         	        if( sensor_inp8(MASK3_3) == 0x06 ) {
         	            pattern = 11;
         	        }
         		}
         		break;

         	case 11:
            	if( crank && cntGate > 500){
            		pattern = 30;
            		cnt1 = 0;
                    break;
            	}
            	if( lane_half != -1 && cntGate > 2000){
            		pattern = 50;
            		break;
            	}

            	if(SenVal_Center < 0) sp = -SenVal_Center;
            	else				  sp = SenVal_Center;

            	if( sp < 5){
            		sp = 0;
            		hd = SenVal_Center * 15/10;
            	}else if(sp < 10){
            		sp = sp*3;
            		hd = SenVal_Center * 15/10;
              	}else if(sp < 15){
                    		sp = sp*3;
                    		hd = SenVal_Center * 15/10;
              	}else{
            		sp = sp*5;
            		hd = SenVal_Center * 18/10;
            		pattern = 22;
            	}
            	m.handle(hd);
            	m.motor( 100 - sp , 100 - sp );
         		break;

         	case 22:  /* 螟ｧ繧ｫ繝ｼ繝� */
            	if( crank && cntGate > 2000){
            		pattern = 30;
    //                    break;
            	}
            	if( lane_half != -1 ){
            		pattern = 50;
            		break;
            	}
         	   if(SenVal_Center < 5 && SenVal_Center > -5 && SenVal_Center != 0 ) {
         	            pattern = 11;
         	   }
           		break;
            case 30:
                led_m_set( CRANK );
                led_out( 0x1 );
                m.handle(SenVal_Center);
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
               m.handle(SenVal_Center);
               m.motor2(30,30);
            break;
            case 32:
                if( crank_turn == -1 || cnt_dammy > 2000){
                        pattern = 33;
                        cnt1 = 0;
                        fall_flag = 1;
                        break;
               }
                m.handle(SenVal_Center);
                m.motor2(30,30);
            break;

            case 33:
                if(LR == 'L'){
                    /* Left crank */
                	m.handle( -40 );
                	m.motor2( 0,60 );
                    if( sensor_inp8(MASK2_0) && cnt1 > 500){
                    	pattern = 11;
                    	cnt1 = 0;
                    }
                }else{
                    /* Right crank */
                	m.handle( 40 );
                	m.motor2( 60,0 );
                    if( sensor_inp8(MASK0_2) && cnt1 > 500){
                    	pattern = 11;
                    	cnt1 = 0;
                    	fall_flag = 0;
                    }
                }
            break;


 /* Lane change processing at 1st process */
            case 50:
            	m.handle( 0 );
            	m.motor2( 0,0 );
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
                m.handle(SenVal_Center);
                m.motor(50,50);
            	if( crank ){
            		pattern = 30;
            		cnt1 = 0;
                    break;
            	}
            	if( lane_Black ){
					if(LR == 'L'){
						/* Right lane change */
						m.handle( -20 );
					}else{
						/* Left lane change */
						m.handle( 20 );
					}
					pattern = 52;
					cnt1 = 0;
                }
                break;

            case 52:
                /* Lane change processing at 2nd process */
                if( cnt1 > 250 ) {
                    if(LR == 'L'){
                        /* Left lane change */
                    	m.handle( -12 );
                    	m.motor2( 30,20 );
                    }else{
                        /* Right lane change */
                    	m.handle( 12 );
                    	m.motor2( 20,30 );
                    }
                    pattern = 53;
                    cnt1 = 0;
                }
                break;

            case 53:
                /* lane change end check */
            	if(SenVal_Center < 5 && SenVal_Center > -5 && SenVal_Center != 0){
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
            	m.handle(SenVal_Center *2);
            	m.motor( 50 , 50 );
            	if(cnt1 > 1200){
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

            case 1000: //繝ｭ繧ｰ縺ｮ蜃ｺ蜉�
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
                pc.printf("%d,%4d,  0x%2x,%4d,%4d,%4d\r\n",m_number,memory[m_number][0],memory[m_number][1],memory[m_number][2],memory[m_number][3],memory[m_number][4]);
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

//******************************************************************//
// Initialize functions
//*******************************************************************/
//------------------------------------------------------------------//
//Initialize MTU2 PWM functions
//------------------------------------------------------------------//
//MTU2_3, MTU2_4
//Reset-Synchronized PWM mode
//TIOC4A(P4_4) :Left-motor
//TIOC4B(P4_5) :Right-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Motor( void )
{
    /* Port setting for S/W I/O Control */
    /* alternative mode     */

    /* MTU2_4 (P4_4)(P4_5)  */
    GPIOPBDC4   = 0x0000;               /* Bidirection mode disabled*/
    GPIOPFCAE4 &= 0xffcf;               /* The alternative function of a pin */
    GPIOPFCE4  |= 0x0030;               /* The alternative function of a pin */
    GPIOPFC4   &= 0xffcf;               /* The alternative function of a pin */
    /* 2nd altemative function/output    */
    GPIOP4     &= 0xffcf;               /*                          */
    GPIOPM4    &= 0xffcf;               /* p4_4,P4_5:output         */
    GPIOPMC4   |= 0x0030;               /* P4_4,P4_5:double         */

    /* Module stop 33(MTU2) canceling */
    CPGSTBCR3  &= 0xf7;

    /* MTU2_3 and MTU2_4 (Motor PWM) */
    MTU2TCR_3   = 0x20;                 /* TCNT Clear(TGRA), P0ﾏ�/1  */
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

//------------------------------------------------------------------//
//Initialize MTU2 PWM functions
//------------------------------------------------------------------//
//MTU2_0
//PWM mode 1
//TIOC0A(P4_0) :Servo-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Servo( void )
{
    /* Port setting for S/W I/O Control */
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

    /* Module stop 33(MTU2) canceling */
    CPGSTBCR3  &= 0xf7;

    /* MTU2_0 (Motor PWM) */
    MTU2TCR_0   = 0x22;                 /* TCNT Clear(TGRA), P0ﾏ�/16 */
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

    cnt0++;
    cnt1++;
    cntGate++;
    cnt_curve+=3;
    cnt_dammy++;

    /* field check */
    if( vfield_count2 != vfield_count2_buff ) {
        vfield_count2_buff = vfield_count2;
        counter = 0;
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
            Binarization( ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate), ImageBinary, threshold_buff );
//            if( !initFlag ) SenVal1 = sensor_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 12 );
            crank_turn = Crank_Turn_Point( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
//            crank2 = CrankCheck( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
            crank = Crank_Mark_Check( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
//            lane = LaneChangeHalf( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
            bar = StartBarCheck( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
            break;
        case 7:
            if( !initFlag ) SenVal1 = sensor_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 12 );
            if( !initFlag ) SenVal8	= sensor_process8( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 24 );
            if( !initFlag ) SenVal_Center	= sensor_process_Center( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), Sen1Px, 24 );
          break;
        case 8:
            if( !initFlag ) PatternMatching_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), &RightCrank, 2, 10, 2, 8 );
            lane_half = LaneChangeHalf( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
            lane_Black = LaneChangeBlack( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
             break;
        case 9:
            if( !initFlag ) PatternMatching_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), &RightLaneChange, 0, 2, 1, 3 );
            break;
        case 10:
            if( !initFlag ) PatternMatching_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), &LeftCrank, 4, 12, 2, 8 );
            break;
        case 11:
            if( !initFlag ) PatternMatching_process( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate), &LeftLaneChange, 1, 3, 1, 3 );
            break;
        case 12:
           if(pattern > 9 && pattern < 1000) {
                memory[m_number][0] = pattern;
                memory[m_number][1] = sensor_inp8(MASK4_4);
                memory[m_number][2] = SenVal_Center;
                memory[m_number][3] = wide;
                memory[m_number][4] = 0;
                m_number++;
                if(m_number > 10000)m_number = 10000;
            }
            break;
        case 13:
            threshold_buff = Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate));
//            threshold_buff = THRESHOLD;
            break;
        case 14:
        	if( fall_flag == 0 && wide == 99 && lane_Black == 1) pattern = 200;
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
// functions ( on Shield board )
//*******************************************************************/
//------------------------------------------------------------------//
//Dipsw get Function
//------------------------------------------------------------------//
unsigned char dipsw_get( void )
{
//    return( dipsw.read() & 0x0f );
    return 7;
}

//******************************************************************//
// digital sensor functions
//*******************************************************************/
//------------------------------------------------------------------//
// init sensor Function
//------------------------------------------------------------------//
int init_sensor( unsigned char *BuffAddrIn, int HW, int VW, int Cx, int *SENPx, int Y )
{
    int X;
    int L1x, R1x;
    int i, error_cnt;

    //Left side
    for( X = Cx; X > 1; X-- ) {
        if( BuffAddrIn[ ( Y * HW ) + ( X - 0 ) ] == 0 && BuffAddrIn[ ( Y * HW ) + ( X - 1 ) ] == 0 ) {
            L1x = X;
            break;
        }
    }
    //Right side
    for( X = Cx; X < ( HW - 1 ); X++ ) {
        if( BuffAddrIn[ ( Y * HW ) + ( X + 0 ) ] == 0 && BuffAddrIn[ ( Y * HW ) + ( X + 1 ) ] == 0 ) {
            R1x = X;
            break;
        }
    }

    SENPx[ 4 ] = L1x + ( ( R1x - L1x ) / 2 );    // Center
    SENPx[ 2 ] = L1x;   // L1
    SENPx[ 1 ] = R1x;   // R1
    SENPx[ 3 ] = SENPx[ 2 ] - ( SENPx[ 4 ] - SENPx[ 2 ] );  //L2
    SENPx[ 0 ] = SENPx[ 1 ] + ( SENPx[ 1 ] - SENPx[ 4 ] );  //R2

    /* error check */
    error_cnt = 0;
    for( i = 0; i < 4; i++ ) {
        if( SENPx[4] == SENPx[i] ) error_cnt += 1;
    }

//#define DEBUG_MODE
#ifdef DEBUG_MODE
    pc.printf( "L2=%2d, L1=%2d, Cx=%2d, R1=%2d, R2=%2d\n\r", SENPx[ 3 ], SENPx[ 2 ], SENPx[ 4 ], SENPx[ 1 ], SENPx[ 0 ] );
    if( error_cnt != 0 ) pc.printf( "init_sensor function Error %1d\n\r", error_cnt );
    else                 pc.printf( "init_sensor function complete\n\r" );
    pc.printf( "\n\r" );
#endif

    return error_cnt;
}

//------------------------------------------------------------------//
//sensor_process Function(Interrupt function)
//------------------------------------------------------------------//
unsigned char sensor_process( unsigned char *BuffAddrIn, int HW, int VW, int *SENPx, int Y )
{
    int sensor;
    int data;

    sensor  = 0x00;
    if( VW < Y ) {
        pc.printf( "sensor process function error\n\r" );
        return sensor;
    }
//    pc.printf( "SENPx[ 4 ]%d SENPx[ 3 ]%d SENPx[ 2 ]%d SENPx[ 1 ]%d SENPx[ 0 ]%d\n\r",SENPx[ 4 ],SENPx[ 3 ],SENPx[ 2 ],SENPx[ 1 ],SENPx[ 0 ] );

    data    = BuffAddrIn[ ( Y * HW ) + SENPx[ 4 ] ] & 0x01;
    sensor |= ( data << 4 ) & 0x10;
    data    = BuffAddrIn[ ( Y * HW ) + SENPx[ 3 ] ] & 0x01;
    sensor |= ( data << 3 ) & 0x08;
    data    = BuffAddrIn[ ( Y * HW ) + SENPx[ 2 ] ] & 0x01;
    sensor |= ( data << 2 ) & 0x04;
    data    = BuffAddrIn[ ( Y * HW ) + SENPx[ 1 ] ] & 0x01;
    sensor |= ( data << 1 ) & 0x02;
    sensor |= BuffAddrIn[ ( Y * HW ) + SENPx[ 0 ] ] & 0x01;

    sensor &= 0x1f;

    return sensor;
}

unsigned char sensor_process8( unsigned char *ImageData, int HW, int VW, int *SENPx, int Y )
{
    int     Xp, Yp;
    unsigned int sensor;
    sensor = 0;

        for( Xp = 5; Xp < 40; Xp += 4 ) {
        	sensor += ImageData[Xp + (Y * HW)];
        	sensor = sensor << 1;
//            pc.printf( "%3d ", ImageData[Xp + (Y * HW)] );
        }
        sensor = sensor >> 1;
 //       sensor = s;
 //       pc.printf( "    sensor = %x \n\r",sensor );
        return sensor;
//    pc.printf( "\033[%dA", VW );

}

int sensor_process_Center( unsigned char *ImageData, int HW, int VW, int *SENPx, int Y )
{
    int     Xp, Yp;
    int white;
    int t;

    int center;
	int Lsensor,Rsensor;
    white = 0;
    t = 0;
    wide = 0;


    //    pc.printf( "\n\r sensor_process8 \n\r");
 //  for( Yp = 0; Yp < VW; Yp++ ) {
 //       pc.printf( "%2d: ", Yp );
//        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%3d ", ImageData[Xp + (Y * HW)] );
//        }
//        pc.printf( "\n\r" );
//    }
        for( Xp = 0; Xp < HW; Xp++ ) {
    		if(t==0){	/* �ｼ第悽逶ｮ縺ｮ逋ｽ邱� */
    			if( ImageData[Xp + (Y * HW)] ){		/* 蟾ｦ縺九ｉ譛�蛻昴�ｮ逋ｽ */
    				white++;
    				Lsensor = Xp;
    				t = 1;
    			}
    		}else if(t==1){
    			if( ImageData[Xp + (Y * HW)] ){		/* 蟾ｦ縺九ｉ譛�蛻昴�ｮ逋ｽ */
    				white++;
    				Lsensor = Xp;
    				t = 2;
    			}
    		}else if(t==2){	/* �ｼ第悽逶ｮ縺ｮ鮟堤ｷ� */
    			if( !ImageData[Xp + (Y * HW)] ){		/* 蟾ｦ縺九ｉ譛�蛻昴�ｮ鮟� */
    				Rsensor = Xp;
    				t = 3;
    			}
    		}else if(t==3){
    			if( !ImageData[Xp + (Y * HW)] ){		/* 蟾ｦ縺九ｉ譛�蛻昴�ｮ鮟� */
    				Rsensor = Xp;
    				t = 4;
    		}
    		}
        }

 //      if( white >= 5){
    		wide = Rsensor - Lsensor;
    	if(wide > 0 && wide < 40){
    		if(wide > 5){
    			center = (Rsensor + Lsensor)/2 - 20;
    		}else{
    			center = 0;
    		}
       }else{
    	   center = 0;
    	   wide = 99;
       }
//            pc.printf( "%3d ", ImageData[Xp + (Y * HW)] );
  //       sensor = s;
 //       pc.printf( "    sensor = %x \n\r",sensor );
        return center;
//    pc.printf( "\033[%dA", VW );

}


//------------------------------------------------------------------//
//sensor Function
//------------------------------------------------------------------//
unsigned char sensor_inp( void )
{
    return SenVal1 & 0x0f;
}

unsigned char sensor_inp8( unsigned char mask )
{
    return SenVal8 & mask;
}
//------------------------------------------------------------------//
//sensor Function
//------------------------------------------------------------------//
unsigned char center_inp( void )
{
    return ( SenVal1 >> 4 ) & 0x01;
}
//------------------------------------------------------------------//
// Start Bar detctive
// Return values: 0: no Start Bar, 1: Start Bar
//------------------------------------------------------------------//
int StartBarCheck(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     s,r;
    int width;
    int w[VW];
    int n;

    s = 0;
     width = 0;

    for( Yp = 0; Yp < 15; Yp++ ) {
        w[Yp] = 0;
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageData[Xp + (Yp * HW)] == 1){
                s++;
                w[Yp]++;
            }
            if( ImageData[Xp + (Yp * HW)] == 0){
                if(s > width) {
                    width = s;
                }
                s = 0;
            }
        }
    }
    r = -1;
    //繧ｹ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｮ讀懷�ｺ
    for( Yp = 0; Yp < 10; Yp++ ) {
//        if(w[Yp] > 15){ //繧ｹ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｮ髟ｷ縺輔′10莉･荳�
        if(width > 12){ //繧ｹ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｮ髟ｷ縺輔′10莉･荳�
        	r = 1;
        }
    }
    //繧ｹ繧ｿ繝ｼ繝医ヰ繝ｼ縺檎┌縺上↑縺｣縺溘％縺ｨ繧呈､懷�ｺ
    n = 0;
    for( Yp = 0; Yp < 15; Yp++ ) {
//        if(w[Yp] < 6){ //縺吶せ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｮ髟ｷ縺輔′10莉･荳�
        if(width < 6){ //縺吶せ繧ｿ繝ｼ繝医ヰ繝ｼ縺ｮ髟ｷ縺輔′10莉･荳�
//        	r = 0;
        	n++;
            }
    }
    if(n == 15)r = 0;
     /*
    n = 0;
    for( Yp = 0; Yp < 15; Yp++ ) {
        if(w[Yp] < 6){
            n++;
        }
    }
    if(n == 15)r = -1;
    */
    return r;
}
//------------------------------------------------------------------//
// Crank Check
// Return values: 0: no crank 1: Left crank 2: Right crank
//------------------------------------------------------------------//
int Crank_Turn_Point(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     r;
    int     l_START,l_STOP;
    int     w;
    int     Center;
    int     state;

    w = 0;
    l_START = 0; l_STOP = 19;
    state = 0;

    Yp = 20;
    for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
        if( ImageData[Xp + (Yp * HW)] == 1){
            if(state == 0) l_START = Xp;
            w++;
            state = 1;
        }else{
            if( state == 1){
                l_STOP = Xp;
                state = 2;
            }
        }
    }
    if(!(l_START == 0 && l_STOP == 39)){
        Center = (l_START + l_STOP ) /2;
    }
    //pc.printf( "%d %d %d \n\r",l_START,l_STOP,Center );
    r = 0;
    if(w >= 15){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ髟ｷ縺輔′7莉･荳�
        if( Center <= 19) return 1;
        else return 2;
//        r = 1;
    }
    if(w == 0){
        r = -1;
    }
    return r;
}
//------------------------------------------------------------------//
// Crank Check
// Return values: 0: no Start Bar, 1: Start Bar
//------------------------------------------------------------------//
int Crank_Mark_Check(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     s,r;
    int width;
    int w[VW];
//    int n;

    s = 0;
     width = 0;

    for( Yp = 15; Yp < 30; Yp++ ) {
        w[Yp] = 0;
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageData[Xp + (Yp * HW)] == 1){
                s++;
                w[Yp]++;
            }
            if( ImageData[Xp + (Yp * HW)] == 0){
                if(s > width) {
                    width = s;
                }
                s = 0;
            }
        }
    }
    r = 0;
    for( Yp = 15; Yp < 30; Yp++ ) {
        if(w[Yp] > 30){
            r = 1;
        }
    }
     /*
    n = 0;
    for( Yp = 0; Yp < 15; Yp++ ) {
        if(w[Yp] < 6){
            n++;
        }
    }
    if(n == 15)r = -1;
    */
    return r;
}

//------------------------------------------------------------------//
// RightCrank Check
// Return values: 0: no triangle mark, 1: Triangle mark
//------------------------------------------------------------------//
int CrankCheck(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     s,r;
    int width;
    int w[VW];
    int y;

    s = 0;
     width = 0;

    for( Yp = 14; Yp < 29; Yp++ ) {
        w[Yp] = 0;
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageData[Xp + (Yp * HW)] == 1){
                s++;
                w[Yp]++;
            }
            if( ImageData[Xp + (Yp * HW)] == 0){
                if(s > width) {
                    width = s;
                    y = Yp;
                }
                s = 0;
            }
        }
        if(width > 20)break;
    }
//    r = 0;
    if(width >= 20){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ髟ｷ縺輔′10莉･荳�
 //        if(y > 0 && w[y - 1] < 15){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ蜑阪�ｯ縺ｻ縺ｨ繧薙←鮟偵°
 //           if(w[y + 3] < 15){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ谺｡縺ｯ繝医Ξ繝ｼ繧ｹ繝ｩ繧､繝ｳ
                r = 1;
 //           }
 //       }
    }
 //   return r;
    return width;
}
int RightCrankCheck( int percentage )
{
    int ret;

    ret = 0;
    if( RightCrank.p >= percentage ) {
        ret = 1;
    }

    return ret;
}

int LeftCrankCheck( int percentage )
{
    int ret;

    ret = 0;
    if( LeftCrank.p >= percentage ) {
        ret = 1;
    }

    return ret;
}
//------------------------------------------------------------------//
// RightLaneChange Check
// Return values: 0: no triangle mark, 1: Triangle mark
//------------------------------------------------------------------//
//繝ｬ繝ｼ繝ｳ繝√ぉ繝ｳ繧ｸ縺ｮ蜈ｨ縺ｦ鮟偵ｒ隕九▽縺代ｋ
int LaneChangeBlack(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     s = 0;

    for( Yp = 15; Yp < 30; Yp++ ) {
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageData[Xp + (Yp * HW)] == 1) s++;
        }
//        pc.printf( "\n\r" );
    }
    if(s == 0) return 1;
    else return 0;

}
//繝ｬ繝ｼ繝ｳ繝√ぉ繝�繧ｯ縺ｮ繝上�ｼ繝輔Λ繧､繝ｳ繧定ｦ九▽縺代ｋ
int LaneChangeHalf(unsigned char *ImageData, int HW, int VW )
{
    int     Xp, Yp;
    int     r;
    int     l_START,l_STOP;
    int     w;
    int     Center;
    int     state;

    w = 0;
    l_START = 0; l_STOP = 19;
    state = 0;

//    Yp = 7;
    for( Yp = 15; Yp < 30; Yp++){
		for( Xp = 0; Xp < HW; Xp++ ) {
	//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
			if( ImageData[Xp + (Yp * HW)] == 1){
				if(state == 0) l_START = Xp;
				w++;
				state = 1;
			}else{
				if( state == 1){
					l_STOP = Xp;
					state = 2;
				}
			}
		}
		if(w < 20){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ髟ｷ縺輔′15莉･荳�
		    w = 0;
		    l_START = 0; l_STOP = 39;
		    state = 0;
		}else{
			break;
		}
    }
    if(!(l_START == 0 && l_STOP == 39)){
        Center = (l_START + l_STOP ) /2;
    }
    //pc.printf( "%d %d %d \n\r",l_START,l_STOP,Center );
    r = 0;
    if(w >= 20){ //繧ｯ繝ｭ繧ｹ繝ｩ繧､繝ｳ縺ｮ髟ｷ縺輔′7莉･荳�
        if( Center <= 20) return 1;
        else return 2;
//        r = 1;
    }
    if(w == 0){
        r = -1;
    }
    return r;
}

int RightLaneChangeCheck( int percentage )
{
    int ret;

    ret = 0;
    if( RightLaneChange.p >= percentage ) {
        ret = 1;
    }

    return ret;
}

int LeftLaneChangeCheck( int percentage )
{
    int ret;

    ret = 0;
    if( LeftLaneChange.p >= percentage ) {
        ret = 1;
    }

    return ret;
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
//    pc.printf( "sensor_inp = 0x%2x\n\r", sensor_inp8(MASK4_4) );
      pc.printf( "wide = %3d\n\r", wide );
    pc.printf( "Center = %3d\n\r", SenVal_Center );
//    pc.printf( "LaneChangeHalf %d\n\r",LaneChangeHalf( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) ));
    pc.printf( "LaneChangeBlack %d\n\r",LaneChangeBlack( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) ));
//    pc.printf( "center_inp = 0x%02x\n\r", center_inp() );
//    pc.printf( "threshold= %d\n\r", Threshold_process(ImageComp_B, (PIXEL_HW * Rate), (PIXEL_VW * Rate)));
//      pc.printf( "RightCrank      = %01d, = %3d%%, X = %2d, Y = %2d\n\r", RightCrankCheck(80), RightCrank.p, RightCrank.x, RightCrank.y );
//    pc.printf( "lane_black = %2d\n\r",LaneChangeBlack( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate)));
//    pc.printf( "lane_Half = %2d\n\r",LaneChangeHalf( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate)));

//    pc.printf( "RightLaneChange = %01d, = %3d%%, X = %2d, Y = %2d\n\r", RightLaneChangeCheck(80), RightLaneChange.p, RightLaneChange.x, RightLaneChange.y );
//    pc.printf( "LeftCrank      = %01d, = %3d%%, X = %2d, Y = %2d\n\r", LeftCrankCheck(80), LeftCrank.p, LeftCrank.x, LeftCrank.y );
//    pc.printf( "LeftLaneChange = %01d, = %3d%%, X = %2d, Y = %2d\n\r", LeftLaneChangeCheck(80), LeftLaneChange.p, LeftLaneChange.x, LeftLaneChange.y );
    pc.printf( "Crank_Mark_Check %d\n\r",Crank_Mark_Check( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) ));
        pc.printf( "CrankturnPoint %d\n\r",Crank_Turn_Point( ImageBinary, (PIXEL_HW * Rate), (PIXEL_VW * Rate) ));
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

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
