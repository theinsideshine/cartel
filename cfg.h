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

#define COLOR_DANGER_DEFAULT            0x00FF0000  // Color rojo por defecto para la zona de peligro.
#define COLOR_WARNING_DEFAULT           0x00FFFF00  // Color amarillo por defecto para la zona de precaucion.
#define COLOR_SAFE_DEFAULT              0x0000FF00  // Color verde por defecto para la zona segura.

// Mapa de direcciones de los campos de configuracion en la EEPROM.
#define EEPROM_ADDRESS_MAGIC_NUMBER     0
#define EEPROM_ADDRESS_SAFE             (EEPROM_ADDRESS_MAGIC_NUMBER + sizeof(uint8_t))
#define EEPROM_ADDRESS_WARNING          (EEPROM_ADDRESS_SAFE + sizeof(uint16_t))
#define EEPROM_ADDRESS_DANGER           (EEPROM_ADDRESS_WARNING + sizeof(uint16_t))
#define EEPROM_ADDRESS_BUZZER           (EEPROM_ADDRESS_DANGER + sizeof(uint16_t))
#define EEPROM_ADDRESS_LOG_CONTROL      (EEPROM_ADDRESS_BUZZER + sizeof(bool))
#define EEPROM_ADDRESS_COLOR_DANGER     (EEPROM_ADDRESS_LOG_CONTROL + sizeof(uint8_t) )
#define EEPROM_ADDRESS_COLOR_WARNING    (EEPROM_ADDRESS_COLOR_DANGER + sizeof(uint32_t) )
#define EEPROM_ADDRESS_COLOR_SAFE       (EEPROM_ADDRESS_WARNING + sizeof(uint32_t) )

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

    uint8_t get_log_control( void );
    void set_log_control( uint8_t enable );

    uint32_t get_color_danger( void );
    void set_color_danger( uint32_t );
    uint32_t get_color_warning( void );
    void set_color_warning( uint32_t );
    uint32_t get_color_safe( void );
    void set_color_safe( uint32_t );

    void host_cmd( void );
  private:
    uint8_t         log_control;      // 0 = log de informacion de control desactivada.
    bool            buzzer_on;

    uint16_t danger;                  // Punto de peligro.
    uint16_t warning;                 // Punto de precaucion.
    uint16_t safe;                    // Punto de seguridad.

    uint32_t color_danger;
    uint32_t color_warning;
    uint32_t color_safe;
};

#endif // CONFIG_H
