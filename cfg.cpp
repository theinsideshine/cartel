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
        set_log_level( LOG_DISABLED );

        set_color_danger( COLOR_DANGER_DEFAULT );
        set_color_warning( COLOR_WARNING_DEFAULT );
        set_color_safe( COLOR_SAFE_DEFAULT );
        set_ewma_alpha( EWMA_ALPHA );
        set_buzzer_ton( TIME_BUZZER_ON );
        set_buzzer_toff( TIME_BUZZER_OFF );
        set_time_state( TIME_STATE_DEFAULT );
        set_hysterisis( HYSTERISIS_DEFAULT );
    }else {
        EEPROM.get( EEPROM_ADDRESS_SAFE, safe );
        EEPROM.get( EEPROM_ADDRESS_WARNING, warning );
        EEPROM.get( EEPROM_ADDRESS_DANGER, danger );

        EEPROM.get( EEPROM_ADDRESS_BUZZER, buzzer_on );
        EEPROM.get( EEPROM_ADDRESS_LOG_LEVEL, log_level );

        EEPROM.get( EEPROM_ADDRESS_COLOR_DANGER, color_danger );
        EEPROM.get( EEPROM_ADDRESS_COLOR_WARNING, color_warning );
        EEPROM.get( EEPROM_ADDRESS_COLOR_SAFE, color_safe);

        EEPROM.get( EEPROM_ADDRESS_EWMA_ALPHA, ewma_alpha );
        EEPROM.get( EEPROM_ADDRESS_BUZZER_TON, buzzer_ton );
        EEPROM.get( EEPROM_ADDRESS_BUZZER_TOFF, buzzer_toff );
        EEPROM.get( EEPROM_ADDRESS_TIME_STATE, time_state );
        EEPROM.get( EEPROM_ADDRESS_HYSTERESIS, hysterisis );
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

uint8_t CConfig::get_log_level( void )
{
    return log_level;
}

void CConfig::set_log_level( uint8_t enable )
{
    log_level = enable;
    EEPROM.put( EEPROM_ADDRESS_LOG_LEVEL, log_level );
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

uint32_t CConfig::get_time_state( void )
{
    return time_state;
}

void CConfig::set_time_state( uint32_t val )
{
    time_state = val;
    EEPROM.put( EEPROM_ADDRESS_TIME_STATE, time_state );
}

uint16_t CConfig::get_hysterisis( void )
{
    return hysterisis;
}

void CConfig::set_hysterisis( uint16_t val )
{
    hysterisis = val;
    EEPROM.put( EEPROM_ADDRESS_HYSTERESIS, hysterisis );
}

// Lee por el puerto serie parametros de configuracion en formato json.
// read:'all' or 'version'      envia todos los parametros en formato json, o la version del firmware.
// log_level:0=desactivado,1=mensajes,2=info control estandar,3=info control arduino plotter
// buzzer:false/true.           activa el buzzer
// point_danger:0 a 65535       configura el punto de peligro
// point_warning:0 a 65535      configura el punto de precaucion.
// point_safe:0 a 65535         configura el punto de seguridad.
// color_danger:0 a 0xFFFFFFFF  configura el color para peligro.
// color_warning:0 a 0xFFFFFFF  configura el color para precaucion.
// color_safe: 0 a 0xFFFFFFFF   configura el color para seguro.
// ewma_alpha: 0 a 1            configura la constante alpha del filtro exponencial.
// buzzer_ton: 0 a 0xFFFFFFFF   configura el tiempo que emite sonido en mS.
// buzzer_toff: 0 a 0xFFFFFFFF  configura el tiempo que permanece apagado en mS.
// time_state 0 a 0xFFFFFFFF    configura el tiempo que dura la indicacion de estado.
// hysterisis 0 a 65535         configura la histerisis para salir del estado (0 desactivado).
void CConfig::host_cmd( void )
{
bool known_key = false;

    if ( Serial.available() ){
        StaticJsonDocument<512> doc;
        auto error = deserializeJson( doc, Serial );
        if ( !error ) {
            if ( doc.containsKey("buzzer") ) {
                set_buzzer( doc["buzzer"] );
                known_key = true;
            }

            if ( doc.containsKey("point_danger") ) {
                set_danger( doc["point_danger"] );
                known_key = true;
            }

            if ( doc.containsKey("point_warning") ) {
                set_warning( doc["point_warning"] );
                known_key = true;
            }

            if ( doc.containsKey("point_safe") ) {
                set_safe( doc["point_safe"] );
                known_key = true;
            }

            if ( doc.containsKey("color_danger") ) {
                set_color_danger( doc["color_danger"] );
                known_key = true;
            }

            if ( doc.containsKey("color_safe") ) {
                set_color_safe( doc["color_safe"] );
                known_key = true;
            }

            if ( doc.containsKey("color_warning") ) {
                set_color_warning( doc["color_warning"] );
                known_key = true;
            }

            if ( doc.containsKey("log_level") ) {
                set_log_level( doc["log_level"] );
                known_key = true;
            }

            if ( doc.containsKey("ewma_alpha") ) {
                set_ewma_alpha( doc["ewma_alpha"] );
                known_key = true;
            }

            if ( doc.containsKey("buzzer_ton") ) {
                set_buzzer_ton( doc["buzzer_ton"] );
                known_key = true;
            }

            if ( doc.containsKey("buzzer_toff") ) {
                set_buzzer_toff( doc["buzzer_toff"] );
                known_key = true;
            }

            if ( doc.containsKey("time_state") ) {
                set_time_state( doc["time_state"] );
                known_key = true;
            }

            if ( doc.containsKey("hysterisis") ) {
                set_hysterisis( doc["hysterisis"] );
                known_key = true;
            }

            if ( doc.containsKey("info") ) {
                String key = doc["info"];

                if( key == "all-params" ) {
                    send_all_params( doc );
                }else if( key == "version" ) {
                    send_version( doc );
                }
            } else if( known_key == true ) {
                send_ok( doc );
            }
        }
    }
}

// Envia todos los parametros de configuracion en formato json.
// Mas adelante, se podria usar el parametro del comando rea
// para leer un parametro en especial.
void CConfig::send_all_params( JsonDocument& doc )
{
    doc["buzzer"] = get_buzzer();
    doc["point_danger"] =  get_danger();
    doc["point_warning"] = get_warning();
    doc["point_safe"] =  get_safe();
    doc["color_danger"] = get_color_danger();
    doc["color_safe"] = get_color_safe();
    doc["color_warning"] = get_color_warning();
    doc["log_level"] = get_log_level();
    doc["ewma_alpha"] = get_ewma_alpha();
    doc["buzzer_ton"] = get_buzzer_ton();
    doc["buzzer_toff"] = get_buzzer_toff();
    doc["time_state"] = get_time_state();
    doc["hysterisis"] = get_hysterisis();

    serializeJsonPretty( doc, Serial );
}

// Envia la version del firmware.
void CConfig::send_version( JsonDocument& doc )
{
    doc["version"] = FIRMWARE_VERSION;

    serializeJsonPretty( doc, Serial );
}

// Envia el resultado en formato json
void CConfig::send_ok( JsonDocument& doc )
{
    doc[ "result" ] = "ok";

    serializeJsonPretty( doc, Serial );
}
