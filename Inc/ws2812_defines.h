#ifndef SW2812_DEFINES
#define SW2812_DEFINES

#include "stdint.h"

#define MAX_LED_COUNT_PER_CHN       144
#define RGB_BUFFER_SIZE_BYTES       (RGB_BUFFER_SIZE * (8 + 8 + 8) )

#define RGB_BUFFER_SIZE             20										//temorary buffer in memory for led strip transmission
#define RGB_BUFFER_HALF_SIZE        (RGB_BUFFER_SIZE/2)		

#define LEDS_IN_ONE_LED_PACKAGE     sizeof(RGB_t)

typedef struct _RGB_t
{
    uint8_t r, g, b;
} RGB_t;

typedef struct _HSV_t
{
    int16_t h;
    uint8_t s, v;
} HSV_t;

#pragma pack(push, 1)
typedef struct 
{
	uint8_t address;
	uint8_t mode;
	uint8_t ch0_size;
	uint8_t ch1_size;
	uint8_t ch0_data[MAX_LED_COUNT_PER_CHN * LEDS_IN_ONE_LED_PACKAGE];
	uint8_t ch1_data[MAX_LED_COUNT_PER_CHN * LEDS_IN_ONE_LED_PACKAGE];
} LedStripRxPacket;
#pragma pack(pop)


//structure for ws2812 throuth timer pwm transmission
#pragma pack(push, 1)
typedef struct
{
    uint8_t g[8], r[8], b[8];
} PWM_t;
#pragma pack(pop)

void RGB2PWM(RGB_t *rgb, PWM_t *pwm);

typedef struct
{
	PWM_t dmaBuffer[RGB_BUFFER_SIZE];

	uint8_t stripLedCount;
	uint8_t conversionRemain;
	uint8_t transmissionRemain;
	
	PWM_t *firstPartPwmBuf;
	PWM_t *secondPartPwmBuf;
	RGB_t *rgbBufferPtr;
	
} WsOperationsStatus;


#endif
