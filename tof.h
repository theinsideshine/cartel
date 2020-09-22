/**
 * File:   Encapsula el control del sensor de distancia con tecnologia TOF.
 *
 * - Compiler:           Arduino 1.8.13
 * - Supported devices:  Nano
 *
 * \author               JS: jschiavoni@intelektron.com
 *
 * Date:  27-08-2020
 *
 *      Intelektron SA Argentina.
 */
#ifndef TOF_H
#define TOF_H

#include "Arduino.h"

#define XSHUT_PIN                       17      // Pin de control de modo bajo consumo.

#define SAMPLES_BUFFER_SIZE             4       // Tamaño del buffer de muestras de distancia.
#define FILTER_EWMA                             // Selecciona el tipo de filtro que se va a usar para
                                                // posprocesar la distancia del sensor.
class CTof
{
  public:
    CTof();
    bool init( void );
    bool read( double alpha );
    bool is_sample( void );
    uint16_t get_raw( void );
    uint16_t get_filtered( void );

  private:
      bool new_sample;                  // TRUE nueva muestra.
      uint16_t raw;                     // Distancia en mm en crudo.
      uint16_t filtered;                // Distancia en mm filtrada.

#if defined( FILTER_CUSTOM )
      uint8_t    custom_index = 0;
      uint16_t   custom_last_valid_val = 0;
      uint16_t   custom_buff[ SAMPLES_BUFFER_SIZE ] = {0};

      void filter_custom( void );
#elif defined( FILTER_EWMA )
      double ewma_output;

      void filter_ewma( double alpha );
#endif
};

#endif // TOF_H
