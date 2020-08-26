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
#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

#define EEPRON_ADDRESS_CONFIG           4       // Direccion en la epprom donde se almacena la configuracion.
#define MAGIC_NUMBER                    23      // Numero magico para detectar memoria desinicializada.

#define MIN_SENSOR_DISTANCE             600     // Minima distancia de lectura en mm.
#define MAX_SENSOR_DISTANCE             2000    // Distancia maxima a la que puede leer el sensor.
#define DISTANCE_BAND                   200     // Tamaño de la franja en mm.
#define DANGER_DEFAULT                  800     // Distancia de peligro por defecto en mm.
#define BUZZER_DEFAULT                  1       // El buzzer esta activado.

// Mapa de direcciones de los campos de configuracion en la EEPROM.
#define EEPROM_ADDRESS_MAGIC_NUMBER     0
#define EEPROM_ADDRESS_SAFE             (EEPROM_ADDRESS_MAGIC_NUMBER + sizeof(uint8_t))
#define EEPROM_ADDRESS_WARNING          (EEPROM_ADDRESS_SAFE + sizeof(uint16_t))
#define EEPROM_ADDRESS_DANGER           (EEPROM_ADDRESS_WARNING + sizeof(uint16_t))
#define EEPROM_ADDRESS_BUZZER           (EEPROM_ADDRESS_DANGER + sizeof(uint16_t))
#define EEPROM_ADDRESS_LOG_ENABLE       (EEPROM_ADDRESS_BUZZER + sizeof(bool))

class CConfig
{
  public:
    CConfig();
    uint16_t get_safe( void );
    void set_safe( uint16_t );
    uint16_t get_warning( void );
    void set_warning( uint16_t );
    uint16_t get_danger( void );
    void set_danger( uint16_t );

    bool get_buzzer( void );
    void set_buzzer( bool enable );

    uint8_t get_log_enable( void );
    void set_log_enable( uint8_t enable );
  private:
    uint8_t         log_enable;       // 0 = log desactivado.
    bool            buzzer_on;

    uint16_t danger;
    uint16_t warning;
    uint16_t safe;
};

#endif // CONFIG_H
