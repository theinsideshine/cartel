/**
 * File:   Clase de timers por software basados en la funcion millis()
 *         de arduino que al estar basada en un timer de la cpu por irq,
           no bloquea la ejecucion. 
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
#ifndef TIMER_H
#define TIMER_H

#include "Arduino.h"

class CTimer
{
  public:
    CTimer();
    void start();
    bool expired(uint32_t);

  private:
      uint32_t timer;
};

#endif // TIMER_H
