/**
 * File:   Clase de timers por software basados en la funcion millis()
 *         de arduino que al estar basada en un timer de la cpu por irq,
           no bloquea la ejecucion. 
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
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
