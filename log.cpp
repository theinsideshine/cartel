/**
 * File:   Clase para loguear mensajes formateados en arduino.
 *
 * - Compiler:           Arduino 1.8.13
 * - Supported devices:  Nano
 *
 * \author               JS: jschiavoni@intelektron.com
 *
 * Date:  25-08-2020
 *
 *      Intelektron SA Argentina.
 */
#include "log.h"

Clog::Clog()
{
}

void Clog::init( void )
{
    Serial.begin( LOG_SERIAL_SPEED );
}

// Muestra informacion de logueo por el puerto serie, precedidos
// por los milisegundos desde el reset.
// Implementa un wrapper de la funcion print de C para usar string
// formateados, ejemplo: ("distancia %d", var)
// NOTA: para ahorrar memoria RAM usa la version vsnprintf_P para que los
//       string se almacenen en la flash. Hay que anteponer el modificador
//       F(), ejemplo: log_msg( F("valor = %d"), var );
void Clog::msg( const __FlashStringHelper *fmt, ... )
{
char buf[ 128 ];
va_list args;

    va_start(args, fmt);
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
    va_end(args);

    Serial.print( millis() );
    Serial.print( " " );
    Serial.println( buf );
}

void Clog::ctrl( uint8_t enable, uint16_t raw, uint16_t filtered, uint8_t state, uint16_t danger_point )
{
    if( enable == LOG_STANDAR )
    {
        msg( F("raw = %d distancia = %d estado = %d peligro = %d"),
             raw, filtered, state, danger_point );
    }
}
