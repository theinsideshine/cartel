/**
 * File:   Clase que controla los leds que indican los estados.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#ifndef LEDS_H
#define LEDS_H

#include "Arduino.h"
#include <FastLED.h>

#define NUM_LEDS                        8      // Cantidad de led
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
