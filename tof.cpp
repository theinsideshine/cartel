/**
 * File:   Encapsula el control del sensor de distancia con tecnologia TOF.
 *
 * - Compiler:           Arduino 2.3.4  
 * - Supported devices:  Nano/Mega2560
 *
 * \author               JS: juanschiavoni@gmail.com 
 */
#include "tof.h"
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X sensor;

 bool primer_tiempo = true; // Indica si es la primera vez que se mide
unsigned long tiempo_anterior = 0; // Guarda el tiempo de la última muestra válida


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
bool result;

    // Precaucion
    // El pin Shutdown del VL53L0X es activo bajo y no tolera 5V. Se puede quemar.
    pinMode( XSHUT_PIN, OUTPUT );

    delay( 5 );

    pinMode( XSHUT_PIN, INPUT );

    //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
    delay( 10 );


    Wire.begin();
    Wire.setClock(400000);
    sensor.setTimeout(500);

    // Intenta inicializar el sensor, si no puede espera 100 mS
    // y reintenta hasta que el contador llega a cero.
    for ( uint8_t x = 0; x < 10; x++ ) {
        if( (result = sensor.init()) ) {
            break;
        }

        delay( 100 );
    }

    if ( result ) {
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
      over_range = (abs(custom_buff[ x ] - custom_last_valid_val) > 200);

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
// A la lectura en crudo le aplica un filtro exponencial.
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

// Obtiene una muestra del sensor de distancia y le aplica un filtro.
// Retorna false cuando demora mucho tiempos en obtener la muestra.
bool CTof::read( double alpha )
{
bool ret_val = true;
unsigned long tiempo_actual = 0; // Tiempo actual calculado con millis()

     
    // La funcion comienza la lectura de un rango en forma no bloqueante.
    new_sample = sensor.readRangeNoBlocking( raw );

    // Descarta los valores validos en cero.
    if( new_sample && raw == 0 ){
        new_sample = false;
    }

    if ( new_sample ) {       
        
        ret_val = !sensor.timeoutOccurred();
        if ( ret_val ) {
         
#if defined( FILTER_CUSTOM )
            filter_custom();
#elif defined( FILTER_EWMA )
            filter_ewma( alpha );
#endif

          /*
          // Obtiene el tiempo actual
            tiempo_actual = millis();

            if (primer_tiempo) {
                // Si es la primera vez, guarda el tiempo actual
                tiempo_anterior = tiempo_actual;
                primer_tiempo = false;
            } else {
                // Si no es la primera vez, calcula la diferencia de tiempo
                unsigned long diferencia_tiempo = tiempo_actual - tiempo_anterior;
                tiempo_anterior = tiempo_actual; // Actualiza el tiempo anterior

                // Imprime la diferencia de tiempo
                Serial.print("Diferencia de tiempo entre muestras válidas: ");
                Serial.print(diferencia_tiempo);
                Serial.println(" ms");
            }
            */

        }
    }
    
    
    return ret_val;
}


