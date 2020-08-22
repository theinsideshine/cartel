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

#define MAX_SENSOR_DISTANCE             2000    // Distancia maxima a la que puede leer el sensor.
#define DISTANCE_BAND                   200     // Tamaño de la franja en mm.
#define DANGER_DEFAULT                  800     // Distancia de peligro por defecto en mm.
#define BUZZER_DEFAULT                  1       // El buzzer esta activado.

#define ST_UNKNOW                       0       // Estado desconocido.
#define ST_DANGER                       1       // El usuario esta en peligro.
#define ST_WARNING                      2       // El usuario tiene que tener precaucion.
#define ST_SAFE                         3       // El usuario esta seguro.

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

// Retorna true cuando el pulsador de programacion esta presionado.
// Aplica un mecanizmo de antirebote.
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

// Obtiene una muestra del sensor de distancia y le aplica los filtros correspondientes.
static uint16_t get_filtered_distance( void )
{
  uint16_t          val;
  bool              over_range = false;
  static uint8_t    index = 0;
  static uint16_t   last_valid_val = 0;
  static uint16_t   sensor_buff[ SAMPLES_BUFFER_SIZE ] = {0};

  if ( sensor.readRangeNoBlocking( val ) ) {
    // Verifica que la distancia se obtenga sin problemas.
    if (!sensor.timeoutOccurred()) {
        if (index++ >= SAMPLES_BUFFER_SIZE) {
          index = 0;
        }

        sensor_buff[ index ] = val;

        // Para validar la muestra, los valores almacenados
        // en el buffer no tienen que estar separados mas que 20 cm.
        for (uint8_t x=0; x<SAMPLES_BUFFER_SIZE; x++){
          over_range = (abs(sensor_buff[ x ] - val) > 200);

          if (over_range) {
            break;
          }
        }

        // Si las muestras no respetan la distancia, usa la ultima valida.
        if (over_range){
          val = last_valid_val;
        }else{
          last_valid_val = val;
        }
    } else {
        log_msg( F("Sensor error TIMEOUT") );
    }
  }

  return last_valid_val;
}

// La distancia de calibracion, tiene que ser menor al maximo alcance del sensor (2 metros)
// menos las distancias de las dos franjas.
// Retorna true cuando es valida.
bool check_max_calibration_distance( uint16_t val )
{
    return ( val < ( MAX_SENSOR_DISTANCE - ( 2 * DISTANCE_BAND) ) );
}

// Cuando entra a Modo de Calibracion se prende Azul.
// Mientras va midiendo la distancia.
bool calibracion( bool button, DEVICE_CONFIG* dev_cfg, uint16_t new_distance )
{
static uint8_t  blink_led = 0;
static uint32_t blink_timer = 0;

  // Cuando seleccionamos la distancia deseada
  // pulsamos el boton y graba los parametros en la eeprom
  // y pasa al estado de control.
  if ( button ) {
    // No acepta valores fuera de rango.
    if ( !check_max_calibration_distance( new_distance ) ) {
      log_msg( F("La maxima distancia permitida es %d"), (MAX_SENSOR_DISTANCE -( 2 * DISTANCE_BAND) ));
      set_led( CRGB::Red );
      buzzer_on();

      // Fuerza un tick en estado de alarma.
      TIMER_START_MS( blink_timer );
      blink_led = false;
      button = false;
    } else {
      dev_cfg->points.danger = new_distance;
      dev_cfg->points.warning = dev_cfg->points.danger + DISTANCE_BAND;
      dev_cfg->points.safe = dev_cfg->points.danger + ( 2 * DISTANCE_BAND );

      log_msg( F("Puntos seguro = %d, precaucion = %d, peligro = %d"),
              dev_cfg->points.safe,
              dev_cfg->points.warning,
              dev_cfg->points.danger);

      config_write( dev_cfg );
    }
  } else {
    // Va a parpadear para indicar el Modo de Calibracion.
    // Azul -> Rango Valido, Rojo -> Rango Excedido.
    if( TIMER_IS_EXPIRED_MS( blink_timer, 500 ) ) {
        TIMER_START_MS( blink_timer );

        blink_led = !blink_led;
    }

    if( !blink_led ){
      set_led( CRGB::Black );
      buzzer_off();
    }else{
        set_led( CRGB::Blue );
        buzzer_off();
        log_msg( F(" Presione para configurar el punto de peligro en %d mm"), new_distance );
    }
  }

  return button;
}

// Controla la distancia del usuario con respecto al sensor.
void control( DEVICE_CONFIG* dev_cfg, uint16_t new_distance )
{
static uint8_t  last_state = 0;
static uint32_t danger_timer = 0;
static uint32_t buzzer_timer = 0;
static uint32_t log_timer = 0;
uint8_t state;

    state = get_state( &dev_cfg->points, new_distance );

    // Si el estado actual es peligro, resetea el timer de presentacion.
    if( state == ST_DANGER ) {
        TIMER_START_MS( danger_timer );
    // Si el anterior fue peligro, pero el actual no, y el timer no expiro
    // entonces continua en estado de peligro.
    } else if( (last_state == ST_DANGER) && !TIMER_IS_EXPIRED_MS( danger_timer, 1000 ) ) {
        state = last_state;
    // Cuando sale de las condiciones de peligro, resetea el buzzer y el temporizador.
    } else {
        buzzer_off();
        TIMER_START_MS( buzzer_timer );
        TIMER_START_MS( danger_timer );
    }

    switch( state ) {
        case ST_WARNING:
            set_led( CRGB::Yellow );
        break;

        case ST_DANGER:
            set_led( CRGB::Red );

            // Si el buzzer esta actibado, lo prende en forma intermitente cada 400 mS
            if(dev_cfg->buzzer_on) {
              if( TIMER_IS_EXPIRED_MS( buzzer_timer, 300 ) ) {
                  buzzer_toggle();
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

    // Para evitar sobrecargar la unidad serie, el log de la
    // informacion se realiza cada 100 mS.
    if( TIMER_IS_EXPIRED_MS( log_timer, LOG_CTRL_INFO_TIMEOUT ) ) {
        log_msg( F("distancia = %d estado = %d peligro = %d"),
                 new_distance, state, dev_cfg->points.danger );
        TIMER_START_MS( log_timer );
    }
}

// Obtiene el estado comparando la distancia con las franjas configuradas.
uint8_t get_state( DISTANCE_POINT* points, uint16_t val )
{
uint8_t state;

  if ( val < points->danger ) {
    state = ST_DANGER;
  } else if ( (val > points->danger) && (val < points->warning) ) {
    state = ST_WARNING;
  } else {
    state = ST_SAFE;
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
void buzzer_toggle( void )
{
    if (digitalRead( PIN_BUZZER ) == LOW){
        buzzer_off();
    }else{
        buzzer_on();
    }
}

// Inicializa los perfericos del cartel.
void setup() {
  pinMode(PIN_CFG_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);

  log_msg( F("Cartel distanciamiento Covid-19 - version 1.0.0") );
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

// Loop de control del cartel de distanciamiento.
void loop()
{
bool                  button;
uint16_t              val;
static DEVICE_CONFIG  dev_cfg;
static uint32_t       timer = 0;
static uint8_t        st_loop = ST_LOOP_INIT;

    val = get_filtered_distance();

    button = button_debounced();

    switch( st_loop ) {
        // Carga los valores de la configuracion y pasa al estado temporizado
        // donde espera que el usuario configure la distancia de peligro.
        case ST_LOOP_INIT:
            st_loop = ST_LOOP_TIMER_CFG;
            TIMER_START_MS( timer );

            config_read( &dev_cfg );
            set_led( CRGB::Blue );
        break;

        case ST_LOOP_TIMER_CFG:
            // Si el operador presiona el pulsador activa la secuencia de configuracion.
            // Si expira el temporizador pasa al estado de control de distancia.
            if ( button ) {
              set_led( CRGB::Pink );

              log_msg( F("Suelte el pulsador para continuar con la calibracion.") );

              st_loop = ST_LOOP_CONFIG;
            } else if( TIMER_IS_EXPIRED_MS( timer, 5000 ) ){
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }
        break;

        case ST_LOOP_CONFIG:
            // De la rutina de configuracion, sale cuando el usuario presiona el pulsador
            // y el valor que lee del sensor esta dentro del rango permitido.
            if( calibracion( button, &dev_cfg, val ) ) {
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

            control( &dev_cfg, val );
    }
}
