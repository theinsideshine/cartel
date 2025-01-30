/**
 * File:   Clase para controlar el buzzer.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"

#define PIN_BUZZER                      6       // Pin del buzzer.

class CBuzzer
{
  public:
    CBuzzer();
    void init( void );
    void on();
    void off();
    bool tgl();

  private:
};

#endif // BUZZER_H
