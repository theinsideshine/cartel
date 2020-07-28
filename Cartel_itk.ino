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

#define NUM_LEDS 8

struct MisDistancias {
  byte iniciado;
  float minimo;
  float medio;
  float largo;
  float maximo; 
};
       
int pulsador = 2;
int BUZZER = 6;
bool autoaprendizaje = false;
bool semaforo = false;

float DISTANCIA = 0;   
float lastmm = 0;
MisDistancias punto = {
 0,0,0,0,0
};

// WS2812B -> Pin de Control.
#define DATA_PIN 10
// Array de Leds.
CRGB leds[NUM_LEDS];

VL53L0X sensor;

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

void setup() {
  pinMode(pulsador, INPUT_PULLUP); 
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
    Serial.println("Fallo al Inicializar el Sensor?");
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
    if (digitalRead(pulsador) == LOW) 
    { 
      led_pink();
      delay(100);
      while ( digitalRead(pulsador) == LOW );
      led_blue();
      Serial.println("Modo Autoaprendizaje!!"); 
      autoaprendizaje = true;
    }   
    delay(100);
  }
  readConfig(); 
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

  // Cuando seleccionamos la distancia deseado
  // pulsamos el boton y graba los parametros en la eeprom
  // y pasa al estado de control.
  if (digitalRead(pulsador) == LOW) 
  { 
    autoaprendizaje = false;
    punto.maximo = DISTANCIA;
    punto.minimo = DISTANCIA / 4;
    punto.medio = DISTANCIA / 2;
    punto.largo = punto.maximo - punto.minimo;   
    delay(100);  
    Serial.print("Punto Minimo=");
    Serial.println(punto.minimo);
    Serial.print("Punto Medio=");
    Serial.println(punto.medio);
    Serial.print("Punto Largo=");
    Serial.println(punto.largo);
    Serial.print("Punto Maximo=");
    Serial.println(punto.maximo);
    writeConfig();
    readConfig();
  }
  else
  {
    DISTANCIA = (sensor.readRangeSingleMillimeters()); 
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print("Midiendo Distancia Maxima: " ); 
    Serial.print(DISTANCIA);
    Serial.println(" mm.");     
  }
  delay(200);
}

void control(){
  float mm = 0;
  mm = (sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  if (mm != lastmm)
  {
    Serial.print("Distancia Maxima -COVID-: ");    
    Serial.print(punto.maximo);
    Serial.println("mm.");
    Serial.print("Distancia Medida Actual : ");    
    Serial.print(mm);
    Serial.println("mm.");
    lastmm = mm;
  }
  
  if ( mm  < punto.medio) 
  {
     digitalWrite(LED_BUILTIN, HIGH);    
     digitalWrite(BUZZER, LOW);   
  }
  else
  {
     digitalWrite(LED_BUILTIN, LOW);
     digitalWrite(BUZZER, HIGH);    
  }

  if ( mm  > punto.maximo) 
  {     
     led_green();
  }
  else if ( (mm  < punto.maximo) && (mm  > punto.largo) ) 
  { 
     led_yellow();
  }
  else if ( (mm  < punto.largo) && (mm  > punto.medio) ) 
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
  }
  else
  { 
     led_red();
  } 
  delay(10);
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
  Serial.print("Punto Minimo = ");Serial.print(punto.minimo);Serial.println(" mm.");
  Serial.print("Punto Medio = ");Serial.print(punto.medio);Serial.println(" mm.");
  Serial.print("Punto Largo = ");Serial.print(punto.largo);Serial.println(" mm.");
  Serial.print("Punto Maximo = ");Serial.print(punto.maximo);Serial.println(" mm."); 
}
void writeConfigDefecto(){
  int eeAddress = sizeof(float);
  MisDistancias puntosInit = {
  0,500,1000,1500,2000
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


