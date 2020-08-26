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
    type = LOG_DISABLED;
}

void Clog::init( uint8_t enable )
{
    Serial.begin( LOG_SERIAL_SPEED );
    type = enable;
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

// Lee por el puerto serie por comandos log.
// Por ahora se acepta uno solo y tiene 3 valores.
// log:0 = descativa el log.
// log:1 = activa el log.
// log:3 = activa el log en formato arduino serial plotter.
void Clog::process_cmd( void )
{

}
