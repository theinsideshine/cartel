/**
 * File:   Clase para controlar la confirguracion en la EEPROM.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include <ArduinoJson.h>

#define FIRMWARE_VERSION                "1.1.00"

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

#define EWMA_ALPHA                      0.05    // Factor ALFA, tiene que ser mayor que 0 y menor que 1.
                                                // A medida que es menor, mejora el filtrado pero demora
                                                // la salida.
#define TIME_BUZZER_ON                  200     // Tiempo en mS que activa el buzzer cuando esta en la zona de peligro.
#define TIME_BUZZER_OFF                 2000    // Tiempo en mS que permanece apagado el buzzer.

#define TIME_STATE_DEFAULT              1000    // Tiempo por defecto que dura la indicacion del estado.

#define HYSTERISIS_DEFAULT              (DISTANCE_BAND/2) // Por defecto la hysterisis esta activada a la mitad de la banda.

// Mapa de direcciones de los campos de configuracion en la EEPROM.
#define EEPROM_ADDRESS_MAGIC_NUMBER     0
#define EEPROM_ADDRESS_SAFE             (EEPROM_ADDRESS_MAGIC_NUMBER + sizeof(uint8_t))
#define EEPROM_ADDRESS_WARNING          (EEPROM_ADDRESS_SAFE + sizeof(uint16_t))
#define EEPROM_ADDRESS_DANGER           (EEPROM_ADDRESS_WARNING + sizeof(uint16_t))
#define EEPROM_ADDRESS_BUZZER           (EEPROM_ADDRESS_DANGER + sizeof(uint16_t))
#define EEPROM_ADDRESS_LOG_LEVEL        (EEPROM_ADDRESS_BUZZER + sizeof(bool))
#define EEPROM_ADDRESS_COLOR_DANGER     (EEPROM_ADDRESS_LOG_LEVEL + sizeof(uint8_t) )
#define EEPROM_ADDRESS_COLOR_WARNING    (EEPROM_ADDRESS_COLOR_DANGER + sizeof(uint32_t) )
#define EEPROM_ADDRESS_COLOR_SAFE       (EEPROM_ADDRESS_COLOR_WARNING + sizeof(uint32_t) )
#define EEPROM_ADDRESS_EWMA_ALPHA       (EEPROM_ADDRESS_COLOR_SAFE + sizeof(uint32_t))
#define EEPROM_ADDRESS_BUZZER_TON       (EEPROM_ADDRESS_EWMA_ALPHA + sizeof(uint32_t))
#define EEPROM_ADDRESS_BUZZER_TOFF      (EEPROM_ADDRESS_BUZZER_TON + sizeof(uint32_t))
#define EEPROM_ADDRESS_TIME_STATE       (EEPROM_ADDRESS_BUZZER_TOFF + sizeof(uint32_t))
#define EEPROM_ADDRESS_HYSTERESIS       (EEPROM_ADDRESS_TIME_STATE + sizeof(uint32_t))

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

    uint8_t get_log_level( void );
    void set_log_level( uint8_t enable );

    uint32_t get_color_danger( void );
    void set_color_danger( uint32_t );
    uint32_t get_color_warning( void );
    void set_color_warning( uint32_t );
    uint32_t get_color_safe( void );
    void set_color_safe( uint32_t );

    double get_ewma_alpha( void );
    void set_ewma_alpha( double );

    uint32_t get_buzzer_ton( void );
    void set_buzzer_ton( uint32_t );
    uint32_t get_buzzer_toff( void );
    void set_buzzer_toff( uint32_t );

    uint32_t get_time_state( void );
    void set_time_state( uint32_t );

    uint16_t get_hysterisis( void );
    void set_hysterisis( uint16_t );

    void host_cmd( void );

  private:
    uint8_t log_level;          // 0 = log de informacion de control desactivada.
    bool    buzzer_on;

    uint16_t danger;            // Punto de peligro.
    uint16_t warning;           // Punto de precaucion.
    uint16_t safe;              // Punto de seguridad.

    uint32_t color_danger;
    uint32_t color_warning;
    uint32_t color_safe;

    double ewma_alpha;          // Constante alpha del filtro exponencial.

    uint32_t buzzer_ton;        // Tiempo en mS que emite.
    uint32_t buzzer_toff;       // Tiempo en mS que esta apagado.

    uint32_t time_state;        // Tiempo que dura la indicacion del estado en mS.

    uint16_t hysterisis;        // Histerisis para salir de los estados.

    void send_all_params( JsonDocument& );
    void send_version( JsonDocument& );
    void send_ok( JsonDocument& );
};

#endif // CONFIG_H
