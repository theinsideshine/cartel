/**
 * File:   Clase para controlar el buzzer.
 *
 * - Compiler:           Arduino 1.8.13
 * - Supported devices:  Nano
 *
 * \author               JS: jschiavoni@intelektron.com
 *
 * Date:  26-08-2020
 *
 *      Intelektron SA Argentina.
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
