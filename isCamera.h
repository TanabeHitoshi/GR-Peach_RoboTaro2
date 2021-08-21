#ifndef ISCAMERA_H
#define ISCAMERA_H

#define PIXEL_HW               (160u)  /* QVGA */
#define PIXEL_VW               (120u)  /* QVGA */
#define rate					(0.25)

/* Mask set X:Mask on O:Mask off */
#define MASK2_2         0x66            /* XOOX XOOX */
#define MASK4_3         0xfe            /* OOOO OOOX */
#define MASK3_4         0x7f            /* XOOO OOOO */
#define MASK1_0         0x80            /* OXXX XXXX */
#define MASK0_1         0x01            /* XXXX XXXO */
#define MASK1_1         0x81            /* OXXX XXXO */
#define MASK2_0         0x60            /* XOOX XXXX */
#define MASK0_2         0x06            /* XXXX XOOX */
#define MASK3_3         0xe7            /* OOOX XOOO */
#define MASK0_3         0x07            /* XXXX XOOO */
#define MASK3_0         0xe0            /* OOOX XXXX */
#define MASK4_0         0xf0            /* OOOO XXXX */
#define MASK0_4         0x0f            /* XXXX OOOO */
#define MASK4_4         0xff            /* OOOO OOOO */
#define MASK_5_			0x7e			/* XOOO OOOX */
#define MASK_2_			0x18			/* XXXO OXXX */

class isCamera{
    public:
		isCamera(void);
		unsigned char sensor_process8(void);
		unsigned char sensor_inp8( unsigned char mask );
		int sensor_process_Center( void );
		int PID_process( void );
		int LaneChangeHalf(void);
		int LaneChangeBlack(void);
		int Crank_Mark_Check(void);
		int Crank_Turn_Point(void);

		unsigned char   ImageBinary[ ( PIXEL_HW * PIXEL_VW ) ];
		int wide;				//Line wide
		unsigned char   SenVal1,SenVal8;
		int				SenVal_Center;
    private:

};
 
#endif
