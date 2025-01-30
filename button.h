/**
 * File:   Clase que controla el boton de configuracion y operacion
 *         del cartel.
 *
  * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#ifndef BUTTON_H
#define BUTTON_H

#include "Arduino.h"
#include "timer.h"

#define PIN_CFG_BUTTON                  2       // Pin del pulsador de configuracion.

class CButton
{
  public:
    CButton();
    void init( void );
    bool is_pressed( void );
    void debounce( void );

  private:
      CTimer Timer;
      bool state;                   // TRUE presionado.
};

#endif // BUTTON_H
