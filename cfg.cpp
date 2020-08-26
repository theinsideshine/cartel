/**
 * File:   Clase para controlar la confirguracion en la EEPROM.
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
#include "cfg.h"
#include "log.h"

#include <EEPROM.h>
#include <ArduinoJson.h>

CConfig::CConfig()
{
uint8_t magic_number;

    EEPROM.get( EEPROM_ADDRESS_MAGIC_NUMBER, magic_number );

    if( magic_number != MAGIC_NUMBER ){
        magic_number = MAGIC_NUMBER;
        EEPROM.put( EEPROM_ADDRESS_MAGIC_NUMBER, magic_number );

        set_safe( DANGER_DEFAULT + (DISTANCE_BAND * 2) );
        set_warning( DANGER_DEFAULT + DISTANCE_BAND );
        set_danger( DANGER_DEFAULT );
        set_buzzer( BUZZER_DEFAULT );
        set_log_control( LOG_DISABLED );

        set_color_danger( COLOR_DANGER_DEFAULT );
        set_color_warning( COLOR_WARNING_DEFAULT );
        set_color_safe( COLOR_SAFE_DEFAULT );
        set_ewma_alpha( EWMA_ALPHA );
        set_buzzer_ton( TIME_BUZZER_ON );
        set_buzzer_toff( TIME_BUZZER_OFF );
    }else {
        EEPROM.get( EEPROM_ADDRESS_SAFE, safe );
        EEPROM.get( EEPROM_ADDRESS_WARNING, warning );
        EEPROM.get( EEPROM_ADDRESS_DANGER, danger );

        EEPROM.get( EEPROM_ADDRESS_BUZZER, buzzer_on );
        EEPROM.get( EEPROM_ADDRESS_LOG_CONTROL, log_control );

        EEPROM.get( EEPROM_ADDRESS_COLOR_DANGER, color_danger );
        EEPROM.get( EEPROM_ADDRESS_COLOR_WARNING, color_warning );
        EEPROM.get( EEPROM_ADDRESS_COLOR_SAFE, color_safe);

        EEPROM.get( EEPROM_ADDRESS_EWMA_ALPHA, ewma_alpha );
        EEPROM.get( EEPROM_ADDRESS_BUZZER_TON, buzzer_ton );
        EEPROM.get( EEPROM_ADDRESS_BUZZER_TOFF, buzzer_toff );
    }
}

uint16_t CConfig::get_safe( void )
{
    return safe;
}

void CConfig::set_safe( uint16_t val )
{
    safe = val;
    EEPROM.put( EEPROM_ADDRESS_SAFE, val );
}

uint16_t CConfig::get_warning( void )
{
    return warning;
}

void CConfig::set_warning( uint16_t val )
{
    warning = val;
    EEPROM.put( EEPROM_ADDRESS_WARNING, val );
}

uint16_t CConfig::get_danger( void )
{
    return danger;
}

void CConfig::set_danger( uint16_t val )
{
    danger = val;
    EEPROM.put( EEPROM_ADDRESS_DANGER, val );
}

bool CConfig::get_buzzer( void )
{
    return buzzer_on;
}

void CConfig::set_buzzer( bool enable )
{
    buzzer_on = enable;
    EEPROM.put( EEPROM_ADDRESS_BUZZER, buzzer_on );
}

uint32_t CConfig::get_color_danger( void )
{
    return color_danger;
}

void CConfig::set_color_danger( uint32_t color )
{
    color_danger = color;
    EEPROM.put( EEPROM_ADDRESS_COLOR_DANGER, color_danger );
}

uint32_t CConfig::get_color_warning( void )
{
    return color_warning;
}

void CConfig::set_color_warning( uint32_t color )
{
    color_warning = color;
    EEPROM.put( EEPROM_ADDRESS_COLOR_WARNING, color_warning );
}

uint32_t CConfig::get_color_safe( void )
{
    return color_safe;
}

void CConfig::set_color_safe( uint32_t color )
{
    color_safe = color;
    EEPROM.put( EEPROM_ADDRESS_COLOR_SAFE, color_safe );
}

uint8_t CConfig::get_log_control( void )
{
    return log_control;
}

void CConfig::set_log_control( uint8_t enable )
{
    log_control = enable;
    EEPROM.put( EEPROM_ADDRESS_LOG_CONTROL, log_control );
}

double CConfig::get_ewma_alpha( void )
{
    return ewma_alpha;
}

void CConfig::set_ewma_alpha( double val )
{
    ewma_alpha = val;
    EEPROM.put( EEPROM_ADDRESS_EWMA_ALPHA, ewma_alpha );
}

uint32_t CConfig::get_buzzer_ton( void )
{
    return buzzer_ton;
}

void CConfig::set_buzzer_ton( uint32_t val )
{
    buzzer_ton = val;
    EEPROM.put( EEPROM_ADDRESS_BUZZER_TON, buzzer_ton );
}

uint32_t CConfig::get_buzzer_toff( void )
{
    return buzzer_toff;
}

void CConfig::set_buzzer_toff( uint32_t val )
{
    buzzer_toff = val;
    EEPROM.put( EEPROM_ADDRESS_BUZZER_TOFF, buzzer_toff );
}

// Lee por el puerto serie parametros de configuracion en formato json.
// log_control:0=desactivado,1=estandar,2=arduino plotter
// buzzer:false/true.           activa el buzzer
// point_danger:0 a 65535       configura el punto de peligro
// point_warning:0 a 65535      configura el punto de precaucion.
// point_safe:0 a 65535         configura el punto de seguridad.
// color_danger:0 a 0xFFFFFFFF  configura el color para peligro.
// color_warning:0 a 0xFFFFFFF  configura el color para precaucion.
// color_safe: 0 a 0xFFFFFFFF   configura el color para seguro.
// ewma_alpha: 0 a 1            configura la constante alpha del filtro exponencial.
// buzzer_ton: 0 a 0xFFFFFFFF   configura el tiempo que emite sonido en mS.
// buzzer_toff: 0 a 0xFFFFFFFF   configura el tiempo que permanece apagado en mS.
void CConfig::host_cmd( void )
{
    if ( Serial.available() ){
        StaticJsonDocument<256> doc;
        auto error = deserializeJson( doc, Serial );
        if ( !error ) {
            if ( doc.containsKey("buzzer") ) {
                set_buzzer( doc["buzzer"] );
                Serial.println( get_buzzer() );
            }

            if ( doc.containsKey("point_danger") ) {
                set_danger( doc["point_danger"] );
                Serial.println( get_danger() );
            }

            if ( doc.containsKey("point_warning") ) {
                set_warning( doc["point_warning"] );
                Serial.println( get_warning() );
            }

            if ( doc.containsKey("point_safe") ) {
                set_safe( doc["point_safe"] );
                Serial.println( get_safe() );
            }

            if ( doc.containsKey("color_danger") ) {
                set_color_danger( doc["color_danger"] );
                Serial.println( get_color_danger() );
            }

            if ( doc.containsKey("color_safe") ) {
                set_color_safe( doc["color_safe"] );
                Serial.println( get_color_safe() );
            }

            if ( doc.containsKey("color_warning") ) {
                set_color_warning( doc["color_warning"] );
                Serial.println( get_color_warning() );
            }

            if ( doc.containsKey("log_control") ) {
                set_log_control( doc["log_control"] );
                Serial.println( get_log_control() );
            }

            if ( doc.containsKey("ewma_alpha") ) {
                set_ewma_alpha( doc["ewma_alpha"] );
                Serial.println( get_ewma_alpha() );
            }

            if ( doc.containsKey("buzzer_ton") ) {
                set_buzzer_ton( doc["buzzer_ton"] );
                Serial.println( get_buzzer_ton() );
            }

            if ( doc.containsKey("buzzer_toff") ) {
                set_buzzer_toff( doc["buzzer_toff"] );
                Serial.println( get_buzzer_toff() );
            }
        }
    }
}
