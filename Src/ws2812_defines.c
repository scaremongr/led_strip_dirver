
#include "ws2812_defines.h"
#include "ws2812b.h"

#include <stdint.h>
 

void RGB2PWM(RGB_t *rgb, PWM_t *pwm)
{
    uint8_t mask = 0x80;

    uint32_t i;
    for (i = 0; i < 8; i++)
    {
        pwm->r[i] = rgb->r & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        pwm->g[i] = rgb->g & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        pwm->b[i] = rgb->b & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;

        mask >>= 1;
    }
}


