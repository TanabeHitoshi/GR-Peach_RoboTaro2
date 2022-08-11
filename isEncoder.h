#ifndef ISENCODER_H
#define ISENCODER_H

class isEncoder {
public:
    isEncoder(void);
    void measure(void);
    //Rotary Encoder
    volatile long           lEncoderTotal;  /* Integrated value         */
    volatile int            iEncoder;       /* current value            */
    volatile unsigned int   uEncoderBuff;   /* last count               */
    volatile long           lEncoderLine;   /* Integrated value         */


private:

};
 
#endif
