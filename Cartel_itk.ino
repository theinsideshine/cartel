/*  Lee el Sensor Tof y cambian de color
 *  los Leds cuando nos hacercamos a las
 *  distancias seleccionadas que guarda la eeprom.
 *  La calibracion se realiza x Auto Aprendizaje
 *  apuntando el equipo contra un objeto (pared).
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>
#include <EEPROM.h>

#define NUM_LEDS 16
#define DISTANCIA_MAXIMA 2000
#define DISTANCIA_FRANJA 200
#define DISTANCIA_DEFECTO 800
#define COUNTER_VACIO 5

#define ESTADO_ROJO 0
#define ESTADO_AMARILLO 1
#define ESTADO_VERDE 2
#define ESTADO_AZUL 3
#define ESTADO_BLANCO 4
#define ESTADO_DESCONOCIDO 5

struct MisDistancias {
  byte iniciado;
  float rojo;
  float amarillo;
  float verde; 
};
       
int PULSADOR = 2;
int BUZZER = 6;
bool autoaprendizaje = false;
bool semaforo = false;

float DISTANCIA = 0;  
float lastDISTANCIA = 0;   
float lastmm = 0;
MisDistancias punto = {
 0,0,0,0
};

int counter = 0;
int estado = ESTADO_VERDE;
int lastEstado = ESTADO_VERDE;
int tiempo = 0;

// WS2812B -> Pin de Control.
#define DATA_PIN 8
// Array de Leds.
CRGB leds[NUM_LEDS];

VL53L0X sensor;

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY

void setup() {
  pinMode(PULSADOR, INPUT_PULLUP); 
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(BUZZER, OUTPUT);  
  digitalWrite(BUZZER, HIGH);  
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  Serial.begin(115200);  
  Serial.println("Inicializacion del Sistema en Curso...");
  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Fallo al Inicializar el Sensor VL53L0X!?.");
    while (1) 
    {
      digitalWrite(BUZZER, LOW);
      led_blue();
      delay(1000);
      led_green();
      delay(2000);
      led_red();
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
  
  Serial.println("Sistema Inicializado Correctamente Sensor -> VL53L0X Preparado!."); 

  // Espera un tiempo para entrar a modo de calibracion.  
  for (int i=0; i<10; i++)
  {
    if (digitalRead(PULSADOR) == LOW) 
    { 
      led_pink();
      delay(100);
      estado = ESTADO_BLANCO;
      Serial.println("Suelte el Pulsador para poder Calibrar la Distancia del Punto Minimo (Rojo).");
      while ( digitalRead(PULSADOR) == LOW );
      led_blue();
      Serial.println("Modo Autoaprendizaje!!"); 
      autoaprendizaje = true;
      estado = ESTADO_AZUL;
    }   
    delay(100);
  }
  readConfig(); 
  counter = 0;
  lastmm = DISTANCIA_MAXIMA;
  tiempo = 0;
}

void loop() 
{  
  if ( autoaprendizaje )
  { 
    calibracion();
  }
  else
  {
    control();   
  }
}

// Cuando entra a Modo de Calibracion se prende Azul.
// Mientras va midiendo la distancia.
void calibracion()
{  
  // Cuando seleccionamos la distancia deseada
  // pulsamos el boton y graba los parametros en la eeprom
  // y pasa al estado de control.
  if (digitalRead(PULSADOR) == LOW) 
  { 
    autoaprendizaje = false;

    // Si al calibrar no encuentra objeto enfrente y pulsamos x error
    // graba los parameros x defecto (xxx mm).
    if ( DISTANCIA > ( DISTANCIA_MAXIMA -( 2 * DISTANCIA_FRANJA) ) )
    {
      DISTANCIA = DISTANCIA_DEFECTO;
    }      
    punto.rojo = DISTANCIA;
    punto.amarillo = punto.rojo + DISTANCIA_FRANJA; 
    punto.verde = punto.rojo + ( 2 * DISTANCIA_FRANJA );     
    delay(100);     
    Serial.print("Punto Maximo -> PUNTO VERDE = ");Serial.println(punto.verde);
    Serial.print("Punto Medio  -> PUNTO AMARILLO = ");Serial.println(punto.amarillo);
    Serial.print("Punto Minimo -> PUNTO ROJO =");Serial.println(punto.rojo);   
    writeConfig();
    readConfig();
  }
  else
  {
    DISTANCIA = (sensor.readRangeSingleMillimeters()); 
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print("Midiendo Distancia Minima -> COVID : " );Serial.print(DISTANCIA);Serial.println(" mm."); 
    
    if ( DISTANCIA > ( DISTANCIA_MAXIMA -( 2 * DISTANCIA_FRANJA) ) )
    {
      Serial.print("La Maxima Distancia Permitida es:");Serial.print(( DISTANCIA_MAXIMA -( 2 * DISTANCIA_FRANJA) ));Serial.println(" mm.");
      estado = ESTADO_ROJO;
    }
    else
    {
       estado = ESTADO_AZUL;
    }

    // Va a parpadear para indicar el Modo de Calibracion.
    // Azul -> Rango Valido, Rojo -> Rango Exedido.
    if ( ! semaforo )
    {
      led_off();  
      semaforo = true;
    }
    else
    {
      if (estado == ESTADO_AZUL)
      {
        led_blue();
        digitalWrite(BUZZER, HIGH);
      }
      else
      {
        led_red();
        if (tiempo < 3)
        {
          digitalWrite(BUZZER, HIGH);
          tiempo++;
        }
        else
        {
          digitalWrite(BUZZER, LOW);
          tiempo = 0;            
        }
      }
      semaforo = false;
    }   
  }
}

void control()
{
  float mm = 0;
  mm = (sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("Distancia -COVID-: ");Serial.print(punto.rojo);Serial.println(" mm.");    
  Serial.print("Distancia Actual : ");Serial.print(mm);Serial.print(" mm.");   

  if ( mm == 0 )
  {
    mm = lastmm;
  }
  
  // Si mm esta dentro del rango valido
  // resetea en contador, y lo guarda.
  if( (mm > 0) && (mm < 3000) )
  {
    lastmm = mm;
    counter = 0;
  }
  // Pero si no detecta objeto enfrente
  // lo pone para que sea estado verde
  // cuando el contador es 5.
  else if ( mm > 8100 )
  {
      if( counter < COUNTER_VACIO )
      {
        counter++;
      }
      else
      {
        mm = 2001;
        counter = 0;
      } 
  }
  // Si es cero lo filtra,
  else if ( mm = 0 )
  {
      mm = lastmm;
  }
  else
  {
    //digitalWrite(BUZZER, HIGH); 
  }
  stateColor(mm);
  showColors(estado);
}

void stateColor(float milis)
{  
  // En funcion de la distancia halla el estado del color.
  if ( milis  > punto.verde) 
  {     
    estado = ESTADO_VERDE;
  }
  else if ( (milis  < punto.verde) && (milis  > punto.rojo) ) 
  { 
    estado = ESTADO_AMARILLO;
  }
  else if (milis  < punto.rojo)
  {     
    estado = ESTADO_ROJO;   
  } 
  else
  {
    Serial.println(" --Estado Desconocido.");
    estado = ESTADO_DESCONOCIDO;
  }
}

void showColors(int state)
{
  // Implementamos una pequeña maquinita de estados para controlar los 
  // colores y el temporizador del color rojo y el buzzer.
  switch (state)
  {
    case ESTADO_ROJO:
        led_red();
        digitalWrite(BUZZER, LOW); 
        tiempo = 0;
        Serial.println(" --Estado Rojo--");
    break;
    
    case ESTADO_AMARILLO:
        if( (lastEstado == ESTADO_ROJO) && (tiempo <= 8) )
        {
          led_red();
          digitalWrite(BUZZER, LOW); 
          tiempo++;
          estado = ESTADO_ROJO;
          Serial.println(" --Estado Rojo--");
        }
        else
        {
          led_yellow();
          digitalWrite(BUZZER, HIGH); 
          Serial.println(" --Estado Amarillo--");
        }           
    break;

    case ESTADO_VERDE:
         if( (lastEstado == ESTADO_ROJO) && (tiempo <= 8) )
        {
          led_red();
          digitalWrite(BUZZER, LOW); 
          tiempo++;
          estado = ESTADO_ROJO;
          Serial.println(" --Estado Rojo--");
        }
        else
        {
          led_green();
          digitalWrite(BUZZER, HIGH); 
          Serial.println(" --Estado Verde--");
        }          
    break;
  }
  lastEstado = estado; 
}

//.....................Funciones de Control de la Eeprom.............................
void readConfig(){
  int eeAddress = sizeof(float);

  EEPROM.get(eeAddress, punto);
  Serial.println(" Leyendo las Distancias almacenadas en la Eeprom.");
  if( punto.iniciado != 0 )
  {
    writeConfigDefecto();
    EEPROM.get(eeAddress, punto);
  } 
  Serial.print("Punto Minimo (---ROJO---)= ");Serial.print(punto.rojo);Serial.println(" mm.");
  Serial.print("Punto Medio  (-AMARILLO-)= ");Serial.print(punto.amarillo);Serial.println(" mm.");
  Serial.print("Punto Maximo (---VERDE--)= ");Serial.print(punto.verde);Serial.println(" mm."); 
}
void writeConfigDefecto(){
  int eeAddress = sizeof(float);
  MisDistancias puntosInit = {
  0,1000,1500,2000
  }; 
  Serial.println("Sistema Iniciado x Primera Vez!!.");
  EEPROM.put(eeAddress, puntosInit);
  Serial.println("Grabando Configuracion x Defecto!1.....");  
}
void writeConfig(){
  int eeAddress = sizeof(float);
  EEPROM.put(eeAddress, punto);
  Serial.println("Grabando Configuracion de Autoaprendizaje!!........");  
}
//..................................................................................


//.....................Funciones de Control del Led Inteligente.....................
void led_red()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Green;
  }
  FastLED.show(); 
}
void led_green()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Red;
  }
  FastLED.show();  
}
void led_yellow()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Yellow;
  }
  FastLED.show();
}
void led_blue()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Blue;
  }
  FastLED.show();
}
void led_violet()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Violet;
  }
  FastLED.show();
}
void led_pink()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Pink;
  }
  FastLED.show();
}
void led_off()
{
  for(int dot = 0; dot < NUM_LEDS; dot++) 
  { 
    leds[dot] = CRGB::Black;
  }
  FastLED.show();
}
//----------------------------------------------------------------------------------
