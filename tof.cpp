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
#include "tof.h"
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X sensor;

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY

CTof::CTof()
{
    new_sample = false;
    raw = 0;

#if defined( FILTER_CUSTOM )
    custom_index = 0;
    custom_last_valid_val = 0;
    for(uint8_t x=0; x<SAMPLES_BUFFER_SIZE; x++) {
        custom_buff[ x ] = 0;
    }
#elif defined( FILTER_EWMA )
      ewma_output = 0;
#endif
}

bool CTof::init( void )
{
    Wire.begin();
    Wire.setClock(400000);
    sensor.setTimeout(500);

    bool result = sensor.init();

    if( result ) {
#if defined LONG_RANGE
        // lower the return signal rate limit (default is 0.25 MCPS)
        sensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
        // reduce timing budget to 20 ms (default is about 33 ms)
        sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
        // increase timing budget to 200 ms
        sensor.setMeasurementTimingBudget(200000);
#endif
    }

    return result;
}

bool CTof::is_sample( void )
{
bool ret_val = new_sample;

    new_sample = false;

    return ret_val;
}

uint16_t CTof::get_raw( void )
{
    return raw;
}

uint16_t CTof::get_filtered( void )
{
    return filtered;
}

#if defined( FILTER_CUSTOM )
void CTof::filter_custom( void )
{
bool over_range = false;

    if (custom_index++ >= SAMPLES_BUFFER_SIZE) {
      custom_index = 0;
    }

    custom_buff[ custom_index ] = raw;

    // Para validar la muestra, los valores almacenados
    // en el buffer no tienen que estar separados mas que 20 cm.
    for (uint8_t x=0; x<SAMPLES_BUFFER_SIZE; x++) {
      over_range = (abs(custom_buff[ x ] - val) > 200);

      if (over_range) {
        break;
      }
    }

    // Cuando el valor cumple con la condicion, valida la muestra.
    if (!over_range){
      custom_last_valid_val = raw;
    }

    filtered = custom_last_valid_val;
}
#elif defined( FILTER_EWMA )
// Obtiene una muestra del sensor de distancia y le aplica los filtros correspondientes.
// EWMA Filter - Exponentially Weighted Moving Average filter used for smoothing
// data series readings.
void CTof::filter_ewma( double alpha )
{
    // A medida que el valor es mayor a 0 el filtro tiene menos retardo, pero es
    // mas propenso al ruido.
    // Un alfa de 0.1, significa que el resultado será aproximadamente el
    // promedio de las últimas 10 lecturas.
    ewma_output = (alpha * ( ((double)raw) - ewma_output )) + ewma_output;

    filtered = ((uint16_t) ewma_output);
}
#endif

// Obtiene una muestra del sensor de distancia y le aplica los filtros correspondientes.
// La funcion retorna el valor en crudo para propositos de estudio del filtro.
bool CTof::read( double alpha )
{
bool ret_val = true;

    // La funcion comienza la lectura de un rango en forma no bloqueante.
    new_sample = sensor.readRangeNoBlocking( raw );

    // Descarta los valores validos en cero.
    if( new_sample && raw == 0 ){
        new_sample = false;
    }

    // La funcion comienza la lectura de un rango en forma no bloqueante.
    if ( new_sample ) {
        // Procesa el valor cuando la distancia se obtuvo sin problemas.
        ret_val = !sensor.timeoutOccurred();
        if ( ret_val ) {
#if defined( FILTER_CUSTOM )
            filter_custom();
#elif defined( FILTER_EWMA )
            filter_ewma( alpha );
#endif
        }
    }

    return ret_val;
}
