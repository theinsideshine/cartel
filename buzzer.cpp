/**
 * File:   Clase para controlar el buzzer.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#include "buzzer.h"

CBuzzer::CBuzzer()
{
}

void CBuzzer::init( void )
{
    pinMode( PIN_BUZZER, OUTPUT );
    off();
}

void CBuzzer::on()
{
    digitalWrite( PIN_BUZZER, LOW );
}

void CBuzzer::off()
{
    digitalWrite( PIN_BUZZER, HIGH );
}

// Cambia de estado el buzzer.
// Retorna el estado anterior.
bool CBuzzer::tgl()
{
bool state = (digitalRead( PIN_BUZZER ) == LOW);

    if (state){
        off();
    }else{
        on();
    }

    return !state;
}
