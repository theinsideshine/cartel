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
    level = LOG_DISABLED;
}

void Clog::init( uint8_t level )
{
    Serial.begin( LOG_SERIAL_SPEED );
    set_level( level );
}

void Clog::set_level( uint8_t level )
{
    this->level = level;
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
    if( level == LOG_MSG ){
        char buf[ 128 ];
        va_list args;

        va_start(args, fmt);
        vsnprintf_P( buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
        va_end(args);

        Serial.print( millis() );
        Serial.print( " " );
        Serial.println( buf );
    }
}

// Muestra informacion de logueo por el puerto serie, precedidos
// por los milisegundos desde el reset.
// Implementa un wrapper de la funcion print de C para usar string
// formateados, ejemplo: ("distancia %d", var)
// NOTA: para ahorrar memoria RAM usa la version vsnprintf_P para que los
//       string se almacenen en la flash. Hay que anteponer el modificador
//       F(), ejemplo: log_msg( F("valor = %d"), var );
void Clog::msg_ctrl( bool send_mills, const __FlashStringHelper *fmt, ... )
{
char buf[ 128 ];
va_list args;

    va_start(args, fmt);
    vsnprintf_P( buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
    va_end(args);

    if( send_mills ) {
        Serial.print( millis() );
        Serial.print( " " );
    }

    Serial.println( buf );
}

// Loguea la informacion de control. Hay dos opciones, la primera es para logeos
// para pos-procesar con excel. Y la segunda es para procesarla en tiempo real con
// la utilidad plotter de arduino.
void Clog::ctrl( uint16_t raw, uint16_t filtered, uint8_t state, uint16_t danger_point )
{
    if( level == LOG_CTRL_STANDARD ){
        msg_ctrl( true, F("raw = %d filtered = %d state = %d danger = %d"),
                  raw, filtered, state, danger_point );
    }else if( level == LOG_CTRL_ARDUINO_PLOTTER ) {
        // Escala el estado para mejorar la visualizacion.
        uint32_t scale = map( state, 0, 3, 0, 5000 );

        msg_ctrl( false, F("Min:0, raw:%d, filtered:%d, state:%d, danger:%d, Max:8190"),
                  raw, filtered, scale, danger_point );
    }
}
