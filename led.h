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
#ifndef LEDS_H
#define LEDS_H

#include "Arduino.h"
#include <FastLED.h>

#define NUM_LEDS                        16      // Cantidad de led
#define PIN_WS2812B_CTRL                8       // Pin de control de los leds inteligentes.

class CLed
{
  public:
    CLed();
    void init( void );
    void set( CRGB  );

  private:
      CRGB color;
};

#endif // LEDS_H
