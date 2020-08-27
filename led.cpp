/**
 * File:   Clase que controla los leds que indican los estados.
 *
 * - Compiler:           Arduino 1.8.13
 * - Supported devices:  Nano
 *
 * \author               JS: jschiavoni@intelektron.com
 *
 * Date:  27-08-2020
 *
 *      Intelektron SA Argentina.
 */
#include "led.h"

// Array de Leds.
CRGB leds[ NUM_LEDS ];

CLed::CLed()
{
    color = CRGB::Black;
}

void CLed::init( void )
{
    FastLED.addLeds<WS2812B, PIN_WS2812B_CTRL, GRB>(leds, NUM_LEDS);
}

// Actualiza el display de leds inteligentes, cuando el
// color actual es diferente del anterior.
void CLed::set( CRGB color )
{
    if( this->color != color ){
        for (uint8_t dot = 0; dot < NUM_LEDS; dot++) {
            leds[ dot ] = color;
        }

        FastLED.show();

        this->color = color;
    }
}
