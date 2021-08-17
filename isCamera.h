#ifndef ISCAMERA_H
#define ISCAMERA_H

#define PIXEL_HW               (160u)  /* QVGA */
#define PIXEL_VW               (120u)  /* QVGA */
#define rate					(0.25)

class isCamera{
    public:
		isCamera(void);
		unsigned char sensor_process8( unsigned char *ImageData, int HW, int VW, int *SENPx, int Y );

		unsigned char   ImageBinary[ ( PIXEL_HW * PIXEL_VW ) ];

    private:

};
 
#endif
