#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "image_process.h"
#include "isEncoder.h"



isEncoder::isEncoder()
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

void isEncoder::measure(void )
{
	int i;
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
}
