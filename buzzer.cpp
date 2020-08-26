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
