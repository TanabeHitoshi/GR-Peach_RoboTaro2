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
unsigned char isCamera::sensor_process8( unsigned char *ImageData, int HW, int VW, int *SENPx, int Y )
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
