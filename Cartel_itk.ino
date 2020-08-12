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
#define COUNTER_VACIO 5

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
  
  Serial.println("Sistema Inicializado Correctamente Sensor Listo!."); 

  // Espera un tiempo para entrar a modo de calibracion.  
  for (int i=0; i<10; i++)
  {
    if (digitalRead(PULSADOR) == LOW) 
    { 
      led_pink();
      delay(100);
      Serial.println("Suelte el Pulsador para poder Calibrar la Distancia del Punto Minimo (Rojo).");
      while ( digitalRead(PULSADOR) == LOW );
      led_blue();
      Serial.println("Modo Autoaprendizaje!!"); 
      autoaprendizaje = true;
    }   
    delay(100);
  }
  readConfig(); 
  counter = 0;
  lastmm = DISTANCIA_MAXIMA;
}

void loop() 
{  
  if ( autoaprendizaje )
  { 
    meter();
  }
  else
  {
    control();   
  }
}

// Cuando entra a Modo de Calibracion se prende Azul.
// Mientras va midiendo la distancia.
void meter(){  
  led_blue();

  // Cuando seleccionamos la distancia deseada
  // pulsamos el boton y graba los parametros en la eeprom
  // y pasa al estado de control.
  if (digitalRead(PULSADOR) == LOW) 
  { 
    autoaprendizaje = false;

    // Si al calibrar no encuentra objeto enfrente y pulsamos x error
    // graba los parameros x defecto (500, 1000, 2000 mm).
    if (DISTANCIA > 8000)
    {
      DISTANCIA = DISTANCIA_MAXIMA / 2;
    }
    
    punto.verde = DISTANCIA_MAXIMA; 
    punto.rojo = DISTANCIA;
    punto.amarillo = (punto.verde +  punto.rojo) / 2;  
    delay(100);  
    Serial.print("Punto Maximo -> PUNTO VERDE = ");
    Serial.println(punto.verde);
    Serial.print("Punto Medio  -> PUNTO AMARILLO = ");
    Serial.println(punto.amarillo);
    Serial.print("Punto Minimo -> PUNTO ROJO =");
    Serial.println(punto.rojo);
    writeConfig();
    readConfig();
  }
  else
  {
    DISTANCIA = (sensor.readRangeSingleMillimeters()); 
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print("Midiendo Distancia Minima -> COVID : " ); 
    Serial.print(DISTANCIA);
    Serial.println(" mm.");
 
    // Va a parpadear en azul para indicar el Modo de Calibracion.
    if ( ! semaforo )
    {
       led_off();
       semaforo = true;
    }
    else
    {
       led_blue();
       semaforo = false;
    }
  }
  delay(50);
}

void control()
{
  float mm = 0;
  mm = (sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("Distancia -COVID-: ");    
  Serial.print(punto.rojo);
  Serial.println("mm.");
  Serial.print("Distancia Actual : ");    
  Serial.print(mm);
  Serial.print("mm.");
  
  // Si mm esta dentro del rango valido
  // resetea en contador, y lo guarda.
  if( (mm > 0) && (mm < 3000) )
  {
    lastmm = mm;
    counter = 0;
  }
  // Pero si no detecta objeto enfrente
  // lo pone para que sea estado verde
  // cuando el contador es 10.
  else if ( mm > 8100 )
  {
      if( counter < COUNTER_VACIO )
      {
        counter++;
      }
      else
      {
        mm = 2001;
        digitalWrite(BUZZER, HIGH); 
        counter = 0;
      } 
  }
  // Si es cero lo filtra,
  else if ( mm = 0 )
  {
      mm = lastmm;
      digitalWrite(BUZZER, HIGH);
  }
  else
  {
    digitalWrite(BUZZER, HIGH); 
  }
  stateColor(mm);
}

void stateColor(float milis)
{  
  if ( milis  < punto.rojo) 
  {
    digitalWrite(LED_BUILTIN, HIGH);    
    digitalWrite(BUZZER, LOW);   
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(BUZZER, HIGH);    
  }
  
  if ( milis  > punto.verde) 
  {     
    led_green();
    Serial.println(" --Estado Verde--");
  }
  else if ( (milis  < punto.verde) && (milis  > punto.amarillo) ) 
  { 
    led_yellow();
    Serial.println(" --Estado Amarillo--");
  }
  else if ( (milis  < punto.amarillo) && (milis  > punto.rojo) ) 
  { 
    if ( ! semaforo )
    {
      led_off();
      semaforo = true;
    }
    else
    {
      led_yellow();
      semaforo = false;
    }
    Serial.println(" --Estado Amarillo Parpadeando--");
  }
  else if (milis  < punto.rojo)
  {     
    led_red();
    Serial.println(" --Estado Rojo--");
  } 
  else
  {
     Serial.println(" --Estado Desconocido.");
    digitalWrite(BUZZER, HIGH); 
  }
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
