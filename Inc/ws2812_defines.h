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
    uint16_t g[8], r[8], b[8];
} PWM_t;
#pragma pack(pop)

extern PWM_t DMABuffer[RGB_BUFFER_SIZE];

void RGB2PWM(RGB_t *rgb, PWM_t *pwm);

typedef struct
{
	uint8_t ch0Size;
	uint8_t ch1Size;
	
	uint8_t ch0ConversionRemain;
	uint8_t ch1ConversionRemain;
	uint8_t ch0TransmissionRemain;
	uint8_t ch1TransmissionRemain;
	
	PWM_t *FirstPartPwmBuf;
	PWM_t *SecondPartPwmBuf;
	RGB_t *RgbBufferPtr;
	
	uint8_t endTransactionFlag;
} WsOperationsStatus;


#endif
