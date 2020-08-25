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
#include <EEPROM.h>

#define EEPRON_ADDRESS_CONFIG           4       // Direccion en la epprom donde se almacena la configuracion.
#define NUM_LEDS                        16      // Cantidad de led

#define MIN_SENSOR_DISTANCE             600     // Minima distancia de lectura en mm.
#define MAX_SENSOR_DISTANCE             2000    // Distancia maxima a la que puede leer el sensor.
#define DISTANCE_BAND                   200     // Tamaño de la franja en mm.
#define DANGER_DEFAULT                  800     // Distancia de peligro por defecto en mm.
#define BUZZER_DEFAULT                  1       // El buzzer esta activado.

#define ST_UNKNOW                       0       // Estado desconocido.
#define ST_DANGER                       1       // El usuario esta en peligro.
#define ST_WARNING                      2       // El usuario tiene que tener precaucion.
#define ST_SAFE                         3       // El usuario esta seguro.

#define EWMA_ALPHA                      0.05     // Factor ALFA, tiene que ser mayor que 0 y menor que 1.
                                                // A medida que es menor, mejora el filtrado pero demora
                                                // la salida.
#define SAMPLES_BUFFER_SIZE             4       // Tamaño del buffer de muestras de distancia.

#define PIN_CFG_BUTTON                  2       // Pin del pulsador de configuracion.
#define PIN_BUZZER                      6       // Pin del buzzer.

#define ST_LOOP_INIT                    0       // Inicializa el programa (carga la configuracion).
#define ST_LOOP_TIMER_CFG               1       // Temporizador de configuracion.
#define ST_LOOP_WAIT_CFG_BUTTON_RELEACE 2       // Espera que el usuario libere el pulsador.
#define ST_LOOP_CONFIG                  3       // Configura la distancia de peligro del cartel.
#define ST_LOOP_RUN                     4       // Verifica la distancia del usuario.
#define ST_INIT_TIMER_CHANGE_BUZZER     5       // Inicializa el timer para modificar el buzzer.

#define LOG_CTRL_INFO_TIMEOUT           100     // Logea la informacion de control cada 100 mS.

#define TIME_DANGER_ON                  200     // Tiempo en mS que activa el buzzer cuando esta en la zona de peligro.
#define TIME_DANGER_OFF                 2000    // Tiempo en mS que permanece apagado el buzzer.

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
  uint16_t filtered;                // Valor filtrado.
} SAMPLE_INFO;

// Contiene la informacion de los puntos para definir,
// la franja de precaucion, peligro y seguro.
typedef struct tag_DISTANCE_POINT {
  uint16_t danger;
  uint16_t warning;
  uint16_t safe;
} DISTANCE_POINT;

// Contiene la informacion de configuracion del dispositivo.
typedef struct tag_DEVICE_CONFIG {
  bool              factory_reset;
  DISTANCE_POINT    points;
  bool              buzzer_on;
} DEVICE_CONFIG;

// WS2812B -> Pin de Control.
#define DATA_PIN 8
// Array de Leds.
CRGB leds[NUM_LEDS];

VL53L0X sensor;

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY

// Funciones para usar el temporizador no bloqueante
// del arduino en milisegundos.
#define TIMER_IS_EXPIRED_MS( tmr, us ) ((millis() - tmr) > us)
#define TIMER_START_MS( tmr ) (tmr = millis())

// Muestra informacion de logueo por el puerto serie, precedidos
// por los milisegundos desde el reset.
// Implementa un wrapper de la funcion print de C para usar string
// formateados, ejemplo: ("distancia %d", var)
// NOTA: para ahorrar memoria RAM usa la version vsnprintf_P para que los
//       string se almacenen en la flash. Hay que anteponer el modificador
//       F(), ejemplo: log_msg( F("valor = %d"), var );
static void log_msg( const __FlashStringHelper *fmt, ... )
{
char buf[ 128 ];
va_list args;

    va_start(args, fmt);
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
    va_end(args);

    Serial.print( millis() );
    Serial.print( " " );
    Serial.println( buf );
}

// Retorna true cuando el operador presiono el pulsador de programacion.
// Aplica un mecanismo de antirebote.
static bool button_debounced( void )
{
static uint32_t timer_debounce = 0;

  bool val = (digitalRead( PIN_CFG_BUTTON ) == LOW);

  // Despues que se presiona el pulsador debe permanecer 500 mS liberado.
  if( val ) {
    if( !TIMER_IS_EXPIRED_MS(timer_debounce, 500) ){
      val = false;
    }

    TIMER_START_MS( timer_debounce );
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
        log_msg( F("Sensor error TIMEOUT") );
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
        output = EWMA_ALPHA * ( ((double)sample.raw) - output ) + output;
    } else {
        log_msg( F("Sensor error TIMEOUT") );
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
bool calibracion( bool button, DEVICE_CONFIG* dev_cfg, uint16_t new_distance )
{
static bool     blink_led = false;
static bool     distance_ok = true;
static uint32_t blink_timer = 0;
static uint32_t blink_time = TIME_CFG_BLUE;

  // Presionando el pulsador graba la nueva franja en la eeprom
  // y pasa al estado de control.
  if ( button ) {
    // No acepta valores fuera de rango.
    distance_ok = check_max_calibration_distance( new_distance );
    if ( !distance_ok ) {
      log_msg( F("La maxima distancia permitida es %d"), (MAX_SENSOR_DISTANCE -( 2 * DISTANCE_BAND) ));

      // Fuerza un tick en estado de alarma.
      TIMER_START_MS( blink_timer );
      blink_led = true;
      button = false;
    } else {
      // La distancia medida queda en el medio de la franja de peligro.
      dev_cfg->points.danger = new_distance + (DISTANCE_BAND / 2);
      dev_cfg->points.warning = dev_cfg->points.danger + DISTANCE_BAND;
      dev_cfg->points.safe = dev_cfg->points.danger + ( 2 * DISTANCE_BAND );

      log_msg( F("Puntos seguro = %d, precaucion = %d, peligro = %d"),
              dev_cfg->points.safe,
              dev_cfg->points.warning,
              dev_cfg->points.danger);

      config_write( dev_cfg );
    }
  } else {
    // Indica el modo de calibracion parpadeando en azul.
    if( TIMER_IS_EXPIRED_MS( blink_timer, blink_time ) ) {
        TIMER_START_MS( blink_timer );

        blink_led = !blink_led;
        distance_ok = true;
    }

    if( !blink_led ){
      set_led( CRGB::Black );
      buzzer_off();
    // Si la distancia execede el rango permitido muestra el
    }else if( !distance_ok ) {
      set_led( CRGB::Red );
      buzzer_on();
      blink_time = TIME_CFG_RED;
    } else {
        blink_time = TIME_CFG_BLUE;
        set_led( CRGB::Blue );
        log_msg( F(" Presione para configurar el punto de peligro en %d mm"),
                 new_distance + (DISTANCE_BAND / 2) );
    }
  }

  return button;
}

// Controla la distancia del usuario con respecto al sensor.
void control( DEVICE_CONFIG* dev_cfg, SAMPLE_INFO& sample )
{
uint8_t state;
static uint8_t  last_state = 0;
static uint32_t blink_timer = 0;
static uint32_t buzzer_timer = 0;
static uint32_t buzzer_time = TIME_DANGER_ON;         // Variable para controlar el ton/toff del buzzer.

    state = get_state( &dev_cfg->points, sample.filtered, last_state );

    // Mantiene el estado anterior cuando el actual es menos peligroso que el
    // actual y no expiro el temporizando.
    if ( (state > last_state) && !TIMER_IS_EXPIRED_MS( blink_timer, 1000 ) ) {
        state = last_state;
    } else {
        // Cuando el estado anterior es igual al actual,
        // resetea el temporizador de iluminacion.
        if( state == last_state ) {
            TIMER_START_MS( blink_timer );
        }
    }

    // Apaga el buzzer cuando el estado no es peligro.
    if (state != ST_DANGER) {
        buzzer_off();
        buzzer_time = TIME_DANGER_ON;
        TIMER_START_MS( buzzer_timer );
    }

    switch( state ) {
        case ST_WARNING:
            set_led( CRGB::Yellow );
        break;

        case ST_DANGER:
            set_led( CRGB::Red );

            // Si el buzzer esta activado, lo prende y apaga en forma intermitente.
            if(dev_cfg->buzzer_on) {
              if( TIMER_IS_EXPIRED_MS( buzzer_timer, buzzer_time ) ) {
                  // La pausa es el doble del sonido.
                  buzzer_time = (buzzer_toggle() ? TIME_DANGER_ON : TIME_DANGER_OFF);
                  TIMER_START_MS( buzzer_timer );
              }
            }else{
              buzzer_off();
            }
        break;

        default:
        case ST_SAFE:
            set_led( CRGB::Green );
        break;
    }

    last_state = state;

    // Para evitar sobrecargar la unidad serie, logea la informacion
    // de control cada vez que hay una muestra nueva.
    if( sample.result ) {
        log_msg( F("raw = %d distancia = %d estado = %d peligro = %d"),
                 sample.raw, sample.filtered, state, dev_cfg->points.danger );
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
uint8_t get_state( DISTANCE_POINT* points, uint16_t val, uint8_t last_state )
{
uint8_t state;

  if ( val < points->danger ) {
    state = ST_DANGER;
  } else if ( val < points->warning ) {
    // Para salir de los estados aplica una histerisis de media franja.
    if( (last_state == ST_DANGER) && hysteresis_off( val, points->warning ) ){
      state = last_state;
    }else {
      state = ST_WARNING;
    }
  } else {
      // Para salir de los estados aplica una histerisis de media franja.
      if( (last_state == ST_WARNING) && hysteresis_off( val, points->safe ) ){
          state = last_state;
      } else {
          state = ST_SAFE;
      }
  }

  return state;
}

// Lee los parametros de configuracion de la eeprom.
void config_read( DEVICE_CONFIG* dev_cfg )
{
DEVICE_CONFIG local;

  EEPROM.get( EEPRON_ADDRESS_CONFIG, local );

  dev_cfg->factory_reset = local.factory_reset;
  dev_cfg->points.safe = local.points.safe;
  dev_cfg->points.warning = local.points.warning;
  dev_cfg->points.danger = local.points.danger;
  dev_cfg->buzzer_on = local.buzzer_on;

  // Carga los parametros de fabrica, cuando detecta que la eeprom
  // no fue inicializada.
  if( dev_cfg->factory_reset != false ) {
    config_load_default_values( dev_cfg );

    config_write( dev_cfg );
  }

  log_msg( F("Leer config: seguro = %d, precaucion = %d, peligro = %d, buzzer = %d"),
           dev_cfg->points.safe, dev_cfg->points.warning,
           dev_cfg->points.danger, dev_cfg->buzzer_on );
}

// Carga los parametros de configuracion por defecto.
void config_load_default_values( DEVICE_CONFIG* dev_cfg )
{
  dev_cfg->factory_reset    = false;
  dev_cfg->points.safe      = DANGER_DEFAULT + (DISTANCE_BAND * 2);
  dev_cfg->points.warning   = DANGER_DEFAULT + DISTANCE_BAND;
  dev_cfg->points.danger    = DANGER_DEFAULT;
  dev_cfg->buzzer_on        = BUZZER_DEFAULT;

  log_msg( F("Valores por defecto config: seguro = %d, precaucion = %d, peligro = %d, buzzer = %d"),
           dev_cfg->points.safe, dev_cfg->points.warning,
           dev_cfg->points.danger, dev_cfg->buzzer_on );
}

// Invierte la habilitacion del buzzer, y lo almacena en la eeprom.
void config_buzzer_on_tgl( DEVICE_CONFIG* dev_cfg )
{
  dev_cfg->buzzer_on = !dev_cfg->buzzer_on;
  config_write( dev_cfg );

  log_msg( F("Habilitacion buzzer = %d"), dev_cfg->buzzer_on );
}

// Graba los parametros de configuracion en la eeprom.
void config_write( DEVICE_CONFIG* dev_cfg )
{
DEVICE_CONFIG local;

  local.factory_reset = dev_cfg->factory_reset;
  local.points.safe = dev_cfg->points.safe;
  local.points.warning = dev_cfg->points.warning;
  local.points.danger = dev_cfg->points.danger;
  local.buzzer_on = dev_cfg->buzzer_on;

  EEPROM.put( EEPRON_ADDRESS_CONFIG, local );

  log_msg( F("Grabando configuracion"), dev_cfg->buzzer_on );
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

// Activa el buzzer.
void buzzer_on( void )
{
    digitalWrite(PIN_BUZZER, LOW);
}

// Apaga el buzzer.
void buzzer_off( void )
{
    digitalWrite(PIN_BUZZER, HIGH);
}

// Cambia de estado el buzzer.
// Retorna el estado anterior.
bool buzzer_toggle( void )
{
bool state = (digitalRead( PIN_BUZZER ) == LOW);

    if (state){
        buzzer_off();
    }else{
        buzzer_on();
    }

    return !state;
}

// Inicializa los perfericos del cartel.
void setup() {
  pinMode(PIN_CFG_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);

  log_msg( F("Cartel distanciamiento Covid-19 - version 1.0.1") );
  log_msg( F("Intelektron SA - 2020") );

  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);

  if ( !sensor.init() ){
    log_msg( F("Fallo al Inicializar el Sensor VL53L0X") );
    while (1) {
      buzzer_toggle();

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

  log_msg( F("Sistema inicializado correctamente") );
}

// Verifica que las bandas sean validas.
bool check_bands( DISTANCE_POINT* points )
{
    if( (points->danger > MIN_SENSOR_DISTANCE) && (points->danger < MAX_SENSOR_DISTANCE) ){
        return ( ((points->warning - DISTANCE_BAND) == points->danger) &&
                 ((points->safe - DISTANCE_BAND) == points->warning) );
    }

    return false;
}

// Loop de control del cartel de distanciamiento.
void loop()
{
bool                  button;

static DEVICE_CONFIG  dev_cfg;
static uint32_t       timer = 0;
static uint8_t        st_loop = ST_LOOP_INIT;
static bool           bands_ok = false;
SAMPLE_INFO           sample;

    get_filtered_distance( sample );

    button = button_debounced();

    switch( st_loop ) {
        // Carga los valores de la configuracion y pasa al estado temporizado
        // donde espera que el usuario configure la distancia de peligro.
        case ST_LOOP_INIT:
            st_loop = ST_LOOP_TIMER_CFG;
            TIMER_START_MS( timer );

            config_read( &dev_cfg );

            // Si hay un error en muestra el color rosado.
            if( (bands_ok = check_bands( &dev_cfg.points )) ) {
                set_led( CRGB::Blue );
            }else {
                set_led( CRGB::Pink );
            }
        break;

        case ST_LOOP_TIMER_CFG:
            // Si el operador presiona el pulsador activa la secuencia de configuracion.
            // Si expira el temporizador pasa al estado de control de distancia.
            if ( button ) {
              set_led( CRGB::Pink );

              log_msg( F("Suelte el pulsador para continuar con la calibracion.") );

              st_loop = ST_LOOP_CONFIG;
            // Si las bandas son invalidas se queda en configuracion.
            } else if( bands_ok && TIMER_IS_EXPIRED_MS( timer, 5000 ) ){
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }
        break;

        case ST_LOOP_CONFIG:
            // De la rutina de configuracion, sale cuando el usuario presiona el pulsador
            // y el valor que lee del sensor esta dentro del rango permitido.
            if( calibracion( button, &dev_cfg, sample.filtered ) ) {
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }
        break;

        case ST_INIT_TIMER_CHANGE_BUZZER:
          TIMER_START_MS( timer );
          st_loop = ST_LOOP_RUN;
        break;

        default:
          st_loop = ST_LOOP_RUN;
        case ST_LOOP_RUN:
            // Si el tiempo expiro, cada vez que se presiona el pulsado, invierte la
            // habilitacion del buzzer.
            if( TIMER_IS_EXPIRED_MS( timer, 1000 ) && button ){
                config_buzzer_on_tgl( &dev_cfg );
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }

            control( &dev_cfg, sample );
    }
}
