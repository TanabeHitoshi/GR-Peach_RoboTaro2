#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "image_process.h"
#include "isCamera.h"

isCamera::isCamera()
{

}

/***************************************************************************************************/
// Convert sensor input to 8bit
/***************************************************************************************************/
unsigned char isCamera::sensor_process8(void)
{
    int Y_axis = 24;		/* y axis */
	int     Xp, Yp;
    unsigned int sensor;
    int HW,VW;

    sensor = 0;

    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;
    for( Xp = 5; Xp < 40; Xp += 4 ) {
      	sensor += ImageBinary[Xp + (Y_axis * HW)];
//        	sensor += ImageData[Xp + (Y * HW)];
        	sensor = sensor << 1;
    }
    sensor = sensor >> 1;

    return sensor;

}

unsigned char isCamera::sensor_inp8( unsigned char mask )
{
    return SenVal8 & mask;
}

/***************************************************************************************************/
// Calculate the center of the line
/***************************************************************************************************/
int isCamera::sensor_process_Center(void)
{
    int     Xp, Yp;
    int white;
    int t;
    int HW,VW;
    int Y_axis = 24;		/* y axis */

    int center;
	int Lsensor,Rsensor;
    white = 0;
    t = 0;
    wide = 0;

    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;

    //    pc.printf( "\n\r sensor_process8 \n\r");
 //  for( Yp = 0; Yp < VW; Yp++ ) {
 //       pc.printf( "%2d: ", Yp );
//        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%3d ", ImageData[Xp + (Y * HW)] );
//        }
//        pc.printf( "\n\r" );
//    }
        for( Xp = 0; Xp < HW; Xp++ ) {
    		if(t==0){	/* �ｿｽ�ｽｼ隨ｬ謔ｽ騾ｶ�ｽｮ邵ｺ�ｽｮ騾具ｽｽ驍ｱ�ｿｽ */
    			if( ImageBinary[Xp + (Y_axis * HW)] ){		/* 陝ｾ�ｽｦ邵ｺ荵晢ｽ芽ｭ幢ｿｽ陋ｻ譏ｴ�ｿｽ�ｽｮ騾具ｽｽ */
    				white++;
    				Lsensor = Xp;
    				t = 1;
    			}
    		}else if(t==1){
    			if( ImageBinary[Xp + (Y_axis * HW)] ){		/* 陝ｾ�ｽｦ邵ｺ荵晢ｽ芽ｭ幢ｿｽ陋ｻ譏ｴ�ｿｽ�ｽｮ騾具ｽｽ */
    				white++;
    				Lsensor = Xp;
    				t = 2;
    			}
    		}else if(t==2){	/* �ｿｽ�ｽｼ隨ｬ謔ｽ騾ｶ�ｽｮ邵ｺ�ｽｮ魄溷�､�ｽｷ�ｿｽ */
    			if( !ImageBinary[Xp + (Y_axis * HW)] ){		/* 陝ｾ�ｽｦ邵ｺ荵晢ｽ芽ｭ幢ｿｽ陋ｻ譏ｴ�ｿｽ�ｽｮ魄滂ｿｽ */
    				Rsensor = Xp;
    				t = 3;
    			}
    		}else if(t==3){
    			if( !ImageBinary[Xp + (Y_axis * HW)] ){		/* 陝ｾ�ｽｦ邵ｺ荵晢ｽ芽ｭ幢ｿｽ陋ｻ譏ｴ�ｿｽ�ｽｮ魄滂ｿｽ */
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

// Calculate the pid value
int isCamera::PID_process(void)
{
	static int iCenter;
	static int preCenter;
	int pid_value;

	iCenter += SenVal_Center - preCenter;

	if(SenVal_Center> 15 || SenVal_Center < -15){
		pid_value = SenVal_Center*18/10 + iCenter*2/10 + (SenVal_Center -preCenter)*8/10;
	}else if(SenVal_Center> 5 || SenVal_Center < -5){
		pid_value = SenVal_Center* 14/10 + iCenter*2/10 + (SenVal_Center -preCenter)*4/10;
	}else{
		pid_value = SenVal_Center* 8/10 + iCenter*3/10 + (SenVal_Center -preCenter)*2/10;
	}

	preCenter = SenVal_Center;

	return pid_value;

}

/***************************************************************************************************/
// detect Lane change  half Line
/***************************************************************************************************/
int isCamera::LaneChangeHalf(void)
{
    int     Xp, Yp;
    int     r;
    int     l_START,l_STOP;
    int     w;
    int     Center;
    int     state;
    int HW,VW;

    w = 0;
    l_START = 0; l_STOP = 19;
    state = 0;
    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;

//    Yp = 7;
    for( Yp = 15; Yp < 30; Yp++){
		for( Xp = 0; Xp < HW; Xp++ ) {
	//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
			if( ImageBinary[Xp + (Yp * HW)] == 1){
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
		if(w < 20){ //郢ｧ�ｽｯ郢晢ｽｭ郢ｧ�ｽｹ郢晢ｽｩ郢ｧ�ｽ､郢晢ｽｳ邵ｺ�ｽｮ鬮滂ｽｷ邵ｺ霈披�ｲ15闔会ｽ･闕ｳ�ｿｽ
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
    if(w >= 20){ //郢ｧ�ｽｯ郢晢ｽｭ郢ｧ�ｽｹ郢晢ｽｩ郢ｧ�ｽ､郢晢ｽｳ邵ｺ�ｽｮ鬮滂ｽｷ邵ｺ霈披�ｲ7闔会ｽ･闕ｳ�ｿｽ
        if( Center <= 20) return 1;
        else return 2;
//        r = 1;
    }
    if(w == 0){
        r = -1;
    }
    return r;
}

/***************************************************************************************************/
// detect Lane change  half Line
/***************************************************************************************************/
int isCamera::LaneChangeBlack(void)
{
    int     Xp, Yp;
    int     s = 0;
    int HW,VW;

    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;

    for( Yp = 15; Yp < 30; Yp++ ) {
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageBinary[Xp + (Yp * HW)] == 1) s++;
        }
//        pc.printf( "\n\r" );
    }
    if(s == 0) return 1;
    else return 0;

}

/***************************************************************************************************/
// Crank Check
// Return values: 0: no Start Bar, 1: Start Bar
/***************************************************************************************************/
int isCamera::Crank_Mark_Check(void)
{
    int     Xp, Yp;
    int     s,r;
    int width;
    int w[40];

    int HW,VW;
    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;


    s = 0;
     width = 0;

    for( Yp = 15; Yp < 30; Yp++ ) {
        w[Yp] = 0;
        for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
            if( ImageBinary[Xp + (Yp * HW)] == 1){
                s++;
                w[Yp]++;
            }
            if( ImageBinary[Xp + (Yp * HW)] == 0){
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

/***************************************************************************************************/
// Crank turn check
// Return values: 0: no crank 1: Left crank 2: Right crank
/***************************************************************************************************/
int isCamera::Crank_Turn_Point(void)
{
    int     Xp, Yp;
    int     r;
    int     l_START,l_STOP;
    int     w;
    int     Center;
    int     state;
    int HW,VW;

    HW = PIXEL_HW * rate;
    VW = PIXEL_VW * rate;
    w = 0;
    l_START = 0; l_STOP = 19;
    state = 0;

    Yp = 20;
    for( Xp = 0; Xp < HW; Xp++ ) {
//            pc.printf( "%d ", ImageData[Xp + (Yp * HW)] );
        if( ImageBinary[Xp + (Yp * HW)] == 1){
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
    if(w >= 15){ //Cross line length is 15 or more
        if( Center <= 19) return 1;
        else return 2;
//        r = 1;
    }
    if(w == 0){
        r = -1;
    }
    return r;
}
