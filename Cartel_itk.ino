/**
 * File:   Cartel para distanciamiento social (Covid-19).
 *         Un sensor TOF (VL53L0x) lee la distancia del usuario,
 *         y modifica el color de leds inteligentes dependiendo
 *         de la zonas en la que se encuentre (peligro/precaucion/seguro).
 *
 *         Despues de una condicion de reset, el operador puede calibrar la
 *         distancia del punto de peligro, que ademas se usa para definir las
 *         dos zonas restantes que estan separadas por 20 cm.
 *
 *         Mientras el equipo esta controlando, se puede usar el pulsador
 *         para activar/desactivar el buzzer cuando el usuario esta en la zona
 *         de peligro.
 *
 *         Para que la indicacion de peligro desaparezca, el usuario
 *         tiene que salir de la zona por mas de 1 segundo.
 *
 * - Compiler:           Arduino 1.8.13
 * - Supported devices:  Nano
 *
 * \author               MM: mmartinez@intelektron.com
 *                       JS: jschiavoni@intelektron.com
 *
 * Date:  22-08-2020
 *
 *      Intelektron SA Argentina.
 */
#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>

#include "log.h"
#include "cfg.h"
#include "buzzer.h"
#include "timer.h"
#include "button.h"

#define NUM_LEDS                        16      // Cantidad de led

#define ST_UNKNOW                       0       // Estado desconocido.
#define ST_DANGER                       1       // El usuario esta en peligro.
#define ST_WARNING                      2       // El usuario tiene que tener precaucion.
#define ST_SAFE                         3       // El usuario esta seguro.

#define SAMPLES_BUFFER_SIZE             4       // Tamaño del buffer de muestras de distancia.

#define ST_LOOP_INIT                    0       // Inicializa el programa (carga la configuracion).
#define ST_LOOP_TIMER_CFG               1       // Temporizador de configuracion.
#define ST_LOOP_WAIT_CFG_BUTTON_RELEACE 2       // Espera que el usuario libere el pulsador.
#define ST_LOOP_CONFIG                  3       // Configura la distancia de peligro del cartel.
#define ST_LOOP_RUN                     4       // Verifica la distancia del usuario.
#define ST_INIT_TIMER_CHANGE_BUZZER     5       // Inicializa el timer para modificar el buzzer.

#define LOG_CTRL_INFO_TIMEOUT           100     // Logea la informacion de control cada 100 mS.

#define TIME_CFG_BLUE                   500     // Tiempo de parpadeo en calibracion cuando el valor es valido.
#define TIME_CFG_RED                    300     // Tiempo de parpadeo en calibracion cuando el valor es invalido.

#define FILTER_EWMA                             // Selecciona el tipo de filtro que se va a usar para
                                                // posprocesar la distancia del sensor.

#define ENABLE_HYSTERESIS                       // Habilita la histerisis para salir del estado anterior.

// Informacion de la muestra necesaria para implementar
// un sistema de log con el proposito de estudiar el
// comportamiento del filtro.
typedef struct tag_SAMPLE_INFO {
  bool      result;                 // TRUE se obtuvo muestra valida.
  uint16_t  raw;                    // Valor en crudo.
  uint16_t  filtered;               // Valor filtrado.
} SAMPLE_INFO;

// WS2812B -> Pin de Control.
#define DATA_PIN 8
// Array de Leds.
CRGB leds[NUM_LEDS];

VL53L0X sensor;

Clog Log;
CConfig Config;
CBuzzer Buzzer;
CButton Button;

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY

// Retorna true cuando el operador presiono el pulsador de programacion.
// Aplica un mecanismo de antirebote.
static bool button_debounced( void )
{
static CTimer Debounce;

  bool val = (digitalRead( PIN_CFG_BUTTON ) == LOW);

  // Despues que se presiona el pulsador debe permanecer 500 mS liberado.
  if( val ) {
    if( Debounce.expired( 500 ) ){
      val = false;
    }

    Debounce.start();
  }

  return val;
}

#if defined( FILTER_CUSTOM )
// Obtiene una muestra del sensor de distancia y le aplica los filtros correspondientes.
// La funcion retorna el valor en crudo para propositos de estudio del filtro.
static void get_filtered_distance( SAMPLE_INFO& sample )
{
bool              over_range = false;
static uint8_t    index = 0;
static uint16_t   last_valid_val = 0;
static uint16_t   sensor_buff[ SAMPLES_BUFFER_SIZE ] = {0};

  // La funcion comienza la lectura de un rango en forma no bloqueante.
  sample.result = sensor.readRangeNoBlocking( sample.raw );

  // Descarta los valores validos en cero.
  if( sample.result && sample.raw == 0 ){
      sample.result = false;
  }

  // La funcion comienza la lectura de un rango en forma no bloqueante.
  if ( sample.result ) ) {
    // Procesa el valor cuando distancia se obtuvo sin problemas.
    if (!sensor.timeoutOccurred()) {
        if (index++ >= SAMPLES_BUFFER_SIZE) {
          index = 0;
        }

        sensor_buff[ index ] = sample.raw;

        // Para validar la muestra, los valores almacenados
        // en el buffer no tienen que estar separados mas que 20 cm.
        for (uint8_t x=0; x<SAMPLES_BUFFER_SIZE; x++) {
          over_range = (abs(sensor_buff[ x ] - val) > 200);

          if (over_range) {
            break;
          }
        }

        // Cuando el valor cumple con la condicion, valida la muestra.
        if (!over_range){
          last_valid_val = sample.raw;
        }
    } else {
        Log.msg( F("Sensor error TIMEOUT") );
    }
  }

  sample.filtered = last_valid_val;
}
#elif defined( FILTER_EWMA )
// Obtiene una muestra del sensor de distancia y le aplica los filtros correspondientes.
// EWMA Filter - Exponentially Weighted Moving Average filter used for smoothing
// data series readings.
static void get_filtered_distance( SAMPLE_INFO& sample )
{
static double output;

  // La funcion comienza la lectura de un rango en forma no bloqueante.
  sample.result = sensor.readRangeNoBlocking( sample.raw );

  // Descarta los valores validos en cero.
  if( sample.result && sample.raw == 0 ){
      sample.result = false;
  }

  if ( sample.result ) {
    // Procesa el valor cuando la distancia se obtuvo sin problemas.
    if (!sensor.timeoutOccurred()) {

        // A medida que el valor es mayor a 0 el filtro tiene menos retardo, pero es
        // mas propenso al ruido.
        // Un alfa de 0.1, significa que el resultado será aproximadamente el
        // promedio de las últimas 10 lecturas.
        output = (Config.get_ewma_alpha() * ( ((double)sample.raw) - output )) + output;
    } else {
        Log.msg( F("Sensor error TIMEOUT") );
    }
  }

  sample.filtered = ((uint16_t) output);
}
#endif

// La distancia de calibracion, tiene que ser menor al maximo alcance
// del sensor (2 metros) menos las distancias de las dos franjas.
// Retorna true cuando es valida.
bool check_max_calibration_distance( uint16_t val )
{
  return( (val < ( MAX_SENSOR_DISTANCE - ( 2 * DISTANCE_BAND) )) &&
          (val > MIN_SENSOR_DISTANCE) );
}

// El operador debe selecionar el punto de peligro para que
// el sistema calcule el resto de las franjas.
bool calibracion( uint16_t new_distance )
{
static bool     blink_led = false;
static bool     distance_ok = true;
static CTimer   Timer;
static uint32_t blink_time = TIME_CFG_BLUE;

  // Presionando el pulsador graba la nueva franja en la eeprom
  // y pasa al estado de control.
  if ( Button.is_pressed() ) {
    // No acepta valores fuera de rango.
    distance_ok = check_max_calibration_distance( new_distance );
    if ( !distance_ok ) {
      Log.msg( F("La maxima distancia permitida es %d"), (MAX_SENSOR_DISTANCE -( 2 * DISTANCE_BAND) ));

      // Fuerza un tick en estado de alarma.
      Timer.start();
      blink_led = true;
    } else {
      // La distancia medida queda en el medio de la franja de peligro.
      new_distance += (DISTANCE_BAND / 2);
      Config.set_danger( new_distance );
      Config.set_warning( new_distance + DISTANCE_BAND );
      Config.set_safe( new_distance + ( 2 * DISTANCE_BAND ) );

      Log.msg( F("Nuevos puntos -> seguro = %d, precaucion = %d, peligro = %d"),
              Config.get_safe(), Config.get_warning(), Config.get_danger());

      return true; // Calibracion exitosa.
    }
  } else {
    // Indica el modo de calibracion parpadeando en azul.
    if( Timer.expired( blink_time ) ) {
        Timer.start();

        blink_led = !blink_led;
        distance_ok = true;
    }

    if( !blink_led ){
      set_led( CRGB::Black );
      Buzzer.off();
    // Si la distancia execede el rango permitido muestra el
    }else if( !distance_ok ) {
      set_led( CRGB::Red );
      Buzzer.on();
      blink_time = TIME_CFG_RED;
    } else {
        blink_time = TIME_CFG_BLUE;
        set_led( CRGB::Blue );
        Log.msg( F(" Presione para configurar el punto de peligro en %d mm"),
                 new_distance + (DISTANCE_BAND / 2) );
    }
  }

  return false;  // Continuar con la calibracion.
}

// Controla la distancia del usuario con respecto al sensor.
void control( SAMPLE_INFO& sample )
{
uint8_t state;
static uint8_t  last_state = 0;
static CTimer   Timer_blink;
static CTimer   Timer_buzzer;
static uint32_t buzzer_time = Config.get_buzzer_ton();         // Variable para controlar el ton/toff del buzzer.

    state = get_state( sample.filtered, last_state );

    // Mantiene el estado anterior cuando el actual es menos peligroso que el
    // actual y no expiro el temporizando.
    if ( (state > last_state) && !Timer_blink.expired( 1000 ) ) {
        state = last_state;
    } else {
        // Cuando el estado anterior es igual al actual,
        // resetea el temporizador de iluminacion.
        if( state == last_state ) {
            Timer_blink.start();
        }
    }

    // Apaga el buzzer cuando el estado no es peligro.
    if (state != ST_DANGER) {
        Buzzer.off();
        buzzer_time = Config.get_buzzer_ton();
        Timer_buzzer.start();
    }

    switch( state ) {
        case ST_WARNING:
            set_led( Config.get_color_warning() );
        break;

        case ST_DANGER:
            set_led( Config.get_color_danger() );

            // Si el buzzer esta activado, lo prende y apaga en forma intermitente.
            if ( Config.get_buzzer() ) {
              if( Timer_buzzer.expired( buzzer_time ) ) {
                  // La pausa es el doble del sonido.
                  buzzer_time = (Buzzer.tgl() ? Config.get_buzzer_ton() : Config.get_buzzer_toff());
                  Timer_buzzer.start();
              }
            }else{
              Buzzer.off();
            }
        break;

        default:
        case ST_SAFE:
            set_led( Config.get_color_safe() );
        break;
    }

    last_state = state;

    // Para evitar sobrecargar la unidad serie, Logea la informacion
    // de control cada vez que hay una muestra nueva.
    if( sample.result ) {
        Log.ctrl( sample.raw, sample.filtered, state, Config.get_danger() );
    }
}

// Para salir de las franjas mas bajas, aplica una histerisis de media banda.
bool hysteresis_off( uint16_t val, uint16_t next_point )
{
#ifdef ENABLE_HYSTERESIS
  return ( val < (next_point - (DISTANCE_BAND / 2)) );
#else
  return false;
#endif
}

// Obtiene el estado comparando la distancia con las franjas configuradas.
uint8_t get_state( uint16_t val, uint8_t last_state )
{
uint8_t state;

  if ( val < Config.get_danger() ) {
    state = ST_DANGER;
  } else if ( val < Config.get_warning() ) {
    // Para salir de los estados aplica una histerisis de media franja.
    if( (last_state == ST_DANGER) && hysteresis_off( val, Config.get_warning() ) ){
      state = last_state;
    }else {
      state = ST_WARNING;
    }
  } else {
      // Para salir de los estados aplica una histerisis de media franja.
      if( (last_state == ST_WARNING) && hysteresis_off( val, Config.get_safe() ) ){
          state = last_state;
      } else {
          state = ST_SAFE;
      }
  }

  return state;
}

// Invierte la habilitacion del buzzer, y lo almacena en la eeprom.
void config_buzzer_on_tgl( void )
{
  Config.set_buzzer( !Config.get_buzzer() );

  Log.msg( F("Habilitacion buzzer = %d"), Config.get_buzzer() );
}

// Actualiza el display de leds inteligentes, cuando el
// color actual es diferente del anterior.
void set_led( CRGB new_color )
{
static CRGB last_color = CRGB::Black;

  if( last_color != new_color ){
    for(uint8_t dot = 0; dot < NUM_LEDS; dot++) {
      leds[dot] = new_color;
    }

    FastLED.show();

    last_color = new_color;
  }
}

// Inicializa los perfericos del cartel.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);

  Log.init( Config.get_log_level() );

  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);

  Buzzer.init();
  Button.init();

  Log.msg( F("Cartel distanciamiento Covid-19 - version 1.0.2") );
  Log.msg( F("Intelektron SA - 2020") );

  if ( !sensor.init() ){
    Log.msg( F("Fallo al Inicializar el Sensor VL53L0X") );
    while (1) {
      Buzzer.tgl();

      set_led( CRGB::Blue );
      delay(1000);
      set_led( CRGB::Green );
      delay(2000);
      set_led( CRGB::Red );
      delay(500);
    }
  }
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

  Log.msg( F("Sistema inicializado correctamente") );
}

// Verifica que las bandas sean validas.
bool check_bands( void )
{
    if( (Config.get_danger() > MIN_SENSOR_DISTANCE) && (Config.get_danger() < MAX_SENSOR_DISTANCE) ){
        return ( ((Config.get_warning() - DISTANCE_BAND) == Config.get_danger()) &&
                 ((Config.get_safe() - DISTANCE_BAND) == Config.get_warning()) );
    }

    return false;
}

// Loop de control del cartel de distanciamiento.
void loop()
{
static CTimer   Timer;
static uint8_t  st_loop = ST_LOOP_INIT;
static bool     bands_ok = false;
SAMPLE_INFO     sample;

    get_filtered_distance( sample );

    Button.debounce();

    // Verifica si el host envio un JSON con parametros a procesar.
    Config.host_cmd();

    // Actualiza el nivel de log para detener en tiempo real el envio de parametros.
    Log.set_level( Config.get_log_level() );

    switch( st_loop ) {
        // Carga los valores de la configuracion y pasa al estado temporizado
        // donde espera que el usuario configure la distancia de peligro.
        case ST_LOOP_INIT:
            st_loop = ST_LOOP_TIMER_CFG;
            Timer.start();

            // Si hay un error en muestra el color rosado.
            if( (bands_ok = check_bands()) ) {
                set_led( CRGB::Blue );
            }else {
                set_led( CRGB::Pink );
            }
        break;

        case ST_LOOP_TIMER_CFG:
            // Si el operador presiona el pulsador activa la secuencia de configuracion.
            // Si expira el temporizador pasa al estado de control de distancia.
            if ( Button.is_pressed() ) {
              set_led( CRGB::Pink );

              Log.msg( F("Suelte el pulsador para continuar con la calibracion.") );

              st_loop = ST_LOOP_CONFIG;
            // Si las bandas son invalidas se queda en configuracion.
            } else if( bands_ok && Timer.expired( 5000 ) ){
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }
        break;

        case ST_LOOP_CONFIG:
            // De la rutina de configuracion, sale cuando el usuario presiona el pulsador
            // y el valor que lee del sensor esta dentro del rango permitido.
            if( calibracion( sample.filtered ) ) {
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }
        break;

        case ST_INIT_TIMER_CHANGE_BUZZER:
          Timer.start();
          st_loop = ST_LOOP_RUN;
        break;

        default:
          st_loop = ST_LOOP_RUN;
        case ST_LOOP_RUN:
            // Si el tiempo expiro, cada vez que se presiona el pulsado, invierte la
            // habilitacion del buzzer.
            if( Timer.expired( 1000 ) && Button.is_pressed() ){
                config_buzzer_on_tgl();
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }

            control( sample );
    }
}
