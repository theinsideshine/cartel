/**
 * File:   Clase que controla el boton de configuracion y operacion
 *         del cartel.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 
 */
#include "button.h"

CButton::CButton()
{

}

void CButton::init( void )
{
    pinMode( PIN_CFG_BUTTON, INPUT_PULLUP );
}

// Resetea el evento de click para evitar falsos disparos.
bool CButton::is_pressed( void )
{
bool ret_val = state;

    state = false;

    return ret_val;
}

// Retorna true cuando el operador presiono el pulsador de programacion.
// Aplica un mecanismo de antirebote.
void CButton::debounce( void )
{
    state = (digitalRead( PIN_CFG_BUTTON ) == LOW);

    // Despues que se presiona el pulsador debe permanecer 500 mS liberado.
    if( state ) {
        if( !Timer.expired( 500 ) ){
          state = false;
        }

        Timer.start();
    }
}
