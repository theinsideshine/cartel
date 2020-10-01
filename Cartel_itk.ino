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
#include <avr/wdt.h>

#include "log.h"
#include "cfg.h"
#include "buzzer.h"
#include "timer.h"
#include "button.h"
#include "tof.h"
#include "led.h"

#define ST_UNKNOW                       0       // Estado desconocido.
#define ST_DANGER                       1       // El usuario esta en peligro.
#define ST_WARNING                      2       // El usuario tiene que tener precaucion.
#define ST_SAFE                         3       // El usuario esta seguro.

#define ST_LOOP_INIT                    0       // Inicializa el programa (carga la configuracion).
#define ST_LOOP_TIMER_CFG               1       // Temporizador de configuracion.
#define ST_LOOP_WAIT_CFG_BUTTON_RELEACE 2       // Espera que el usuario libere el pulsador.
#define ST_LOOP_CONFIG                  3       // Configura la distancia de peligro del cartel.
#define ST_LOOP_RUN                     4       // Verifica la distancia del usuario.
#define ST_INIT_TIMER_CHANGE_BUZZER     5       // Inicializa el timer para modificar el buzzer.

#define LOG_CTRL_INFO_TIMEOUT           100     // Logea la informacion de control cada 100 mS.

#define TIME_CFG_BLUE                   500     // Tiempo de parpadeo en calibracion cuando el valor es valido.
#define TIME_CFG_RED                    300     // Tiempo de parpadeo en calibracion cuando el valor es invalido.

Clog    Log;
CConfig Config;
CBuzzer Buzzer;
CButton Button;
CTof    Tof;
CLed    Leds;

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
bool calibracion( void )
{
static bool     blink_led = false;
static bool     distance_ok = true;
static CTimer   Timer;
static uint32_t blink_time = TIME_CFG_BLUE;
uint16_t        distance;

  distance = Tof.get_filtered();

  // Presionando el pulsador graba la nueva franja en la eeprom
  // y pasa al estado de control.
  if ( Button.is_pressed() ) {
    // No acepta valores fuera de rango.
    distance_ok = check_max_calibration_distance( distance );
    if ( !distance_ok ) {
      Log.msg( F("La maxima distancia permitida es %d"), (MAX_SENSOR_DISTANCE -( 2 * DISTANCE_BAND) ));

      // Fuerza un tick en estado de alarma.
      Timer.start();
      blink_led = true;
    } else {
      // La distancia medida queda en el medio de la franja de peligro.
      distance += (DISTANCE_BAND / 2);
      Config.set_danger( distance );
      Config.set_warning( distance + DISTANCE_BAND );
      Config.set_safe( distance + ( 2 * DISTANCE_BAND ) );

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
      Leds.set( CRGB::Black );
      Buzzer.off();
    // Si la distancia execede el rango permitido muestra el
    }else if( !distance_ok ) {
      Leds.set( CRGB::Red );
      Buzzer.on();
      blink_time = TIME_CFG_RED;
    } else {
        blink_time = TIME_CFG_BLUE;
        Leds.set( CRGB::Blue );
        Log.msg( F(" Presione para configurar el punto de peligro en %d mm"),
                 distance + (DISTANCE_BAND / 2) );
    }
  }

  return false;  // Continuar con la calibracion.
}

// Controla la distancia del usuario con respecto al sensor.
void control( void )
{
uint8_t state;
static uint8_t  last_state = 0;
static CTimer   Timer_led;
static CTimer   Timer_buzzer;
static uint32_t buzzer_time = Config.get_buzzer_ton();         // Variable para controlar el ton/toff del buzzer.

    state = get_state( Tof.get_filtered(), last_state );

    // Mantiene el estado anterior cuando el actual es menos peligroso
    // y el temporizando de estados no expiro.
    if ( (state > last_state) && !Timer_led.expired( Config.get_time_state() ) ) {
        state = last_state;
    } else {
        // Cuando el estado anterior es igual al actual,
        // resetea el temporizador de iluminacion.
        if( state == last_state ) {
            Timer_led.start();
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
            Leds.set( Config.get_color_warning() );
        break;

        case ST_DANGER:
            Leds.set( Config.get_color_danger() );

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
            Leds.set( Config.get_color_safe() );
        break;
    }

    last_state = state;

    // Para evitar sobrecargar la unidad serie, logea la informacion
    // de control cada vez que hay una muestra nueva.
    if( Tof.is_sample() ) {
        Log.ctrl( Tof.get_raw(), Tof.get_filtered(), state, Config.get_danger() );
    }
}

// Para salir de las franjas mas bajas, aplica una histerisis de media banda.
bool hysteresis_off( uint16_t val, uint16_t next_point )
{
    if( Config.get_hysterisis() > 0 ) {
        return ( val < (next_point - Config.get_hysterisis()) );
    }

    return false;
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

// Inicializa los perfericos del cartel.
void setup()
{
    pinMode( LED_BUILTIN, OUTPUT );

    Leds.init();

    Log.init( Config.get_log_level() );

    Buzzer.init();
    Button.init();

    Log.msg( F("Cartel distanciamiento Covid-19 - %s"), FIRMWARE_VERSION );
    Log.msg( F("Intelektron SA - 2020") );

    // Activamos antes de la inicializacion del TOF, para que si falla, genere un
    // powerup del micro.
    wdt_enable( WDTO_4S );

    if ( !Tof.init() ){
        Log.msg( F("Fallo al Inicializar el Sensor VL53L0X") );
        while (1) {
            Buzzer.tgl();

            Leds.set( CRGB::Blue );
            delay(1000);
            Leds.set( CRGB::Green );
            delay(2000);
            Leds.set( CRGB::Red );
            delay(500);
        }
    }

    Log.msg( F("Sistema inicializado correctamente") );
}

// Verifica que las bandas sean validas.
bool check_bands( void )
{
    // Danger tiene que estar dentro de los limites de operacion del sensor
    // y ser menor que warning, y a su vez warning menor que safe.
    // y safe menor que el limite superior del sensor.
    if( Config.get_danger() > MIN_SENSOR_DISTANCE ){
        if ( (Config.get_danger() < Config.get_warning()) &&
             (Config.get_warning() < Config.get_safe()) ) {
                 return ( Config.get_safe() < MAX_SENSOR_DISTANCE );
             }
    }

    return false;
}

// Loop de control del cartel de distanciamiento.
void loop()
{
static CTimer   Timer;
static uint8_t  st_loop = ST_LOOP_INIT;
static bool     bands_ok = false;

    // TODO: experimentar si es suficiente para sacar la placa del estado de cuelgue
    //       cuando la fuente principal tiene poco filtro, usando el reset del wdt en
    //       el loop principal, o tiene que ser mas condicional. 
    wdt_reset();

    // Toma una muestra del sensor de distancia.
    if( !Tof.read( Config.get_ewma_alpha() ) ) {
      Log.msg( F("Sensor error TIMEOUT") );
    }

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
                Leds.set( CRGB::Blue );
            }else {
                Leds.set( CRGB::Pink );
            }
        break;

        case ST_LOOP_TIMER_CFG:
            // Si el operador presiona el pulsador activa la secuencia de configuracion.
            // Si expira el temporizador pasa al estado de control de distancia.
            if ( Button.is_pressed() ) {
              Leds.set( CRGB::Pink );

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
            if( calibracion() ) {
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
            // Si el tiempo expiro, cada vez que se presiona el pulsador, invierte la
            // habilitacion del buzzer.
            if( Timer.expired( 1000 ) && Button.is_pressed() ){
                config_buzzer_on_tgl();
                st_loop = ST_INIT_TIMER_CHANGE_BUZZER;
            }

            control();
    }
}
