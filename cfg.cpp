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
        magic_number     = MAGIC_NUMBER;
        EEPROM.put( EEPROM_ADDRESS_MAGIC_NUMBER, magic_number );

        set_safe( DANGER_DEFAULT + (DISTANCE_BAND * 2) );
        set_warning( DANGER_DEFAULT + DISTANCE_BAND );
        set_danger( DANGER_DEFAULT );
        set_buzzer( BUZZER_DEFAULT );
        set_log_enable( LOG_DISABLED );
    }else {
        EEPROM.get( EEPROM_ADDRESS_SAFE, safe );
        EEPROM.get( EEPROM_ADDRESS_WARNING, warning );
        EEPROM.get( EEPROM_ADDRESS_DANGER, danger );

        EEPROM.get( EEPROM_ADDRESS_BUZZER, buzzer_on );
        EEPROM.get( EEPROM_ADDRESS_LOG_ENABLE, log_enable );
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

uint8_t CConfig::get_log_enable( void )
{
    return log_enable;
}

void CConfig::set_log_enable( uint8_t enable )
{
    log_enable = enable;
    EEPROM.put( EEPROM_ADDRESS_LOG_ENABLE, log_enable );
}
