# Cartel para Distanciamiento Social (Covid-19)

Este proyecto utiliza un sensor TOF (VL53L0x) para medir la distancia de un usuario y modificar el color de un conjunto de LEDs inteligentes en función de la zona en la que se encuentre (peligro, precaución o seguro). También incluye un buzzer que puede activarse o desactivarse mediante un pulsador. El sistema permite calibrar las distancias de las zonas a través de un proceso de configuración inicial.

## Descripción del Proyecto

El sistema está diseñado para ser utilizado como un cartel informativo de distanciamiento social, donde:

- **Zona de Peligro**: El usuario está demasiado cerca y se activa el buzzer y un LED rojo.
- **Zona de Precaución**: El usuario está a una distancia moderada, se activa un LED amarillo o de precaución.
- **Zona Segura**: El usuario está en una distancia segura, se activa un LED verde.

### Características:

- **Calibración**: Permite configurar la distancia de peligro mediante un proceso de calibración.
- **Zonas Definidas**: Tres zonas basadas en distancias configurables.
- **Buzzer Activable**: El buzzer se activa cuando el usuario entra en la zona de peligro. Puede ser encendido o apagado a través de un pulsador.
- **Histeresis**: Se utiliza para evitar cambios rápidos entre las zonas cercanas, aplicando una pequeña "zona de transición".
- **Log de Control**: El sistema mantiene un log con el estado del sensor y la distancia medida.

## Componentes

- **Microcontrolador**: Compatible con Arduino Nano/Mega2560.
- **Sensor TOF**: VL53L0x para medir la distancia.
- **LEDs**: LEDs RGB para indicar las zonas de peligro.
- **Buzzer**: Para emitir sonidos cuando el usuario está en la zona de peligro.
- **Pulsador**: Para activar y desactivar el buzzer, así como configurar las distancias.

## Conexiones

- **Sensor TOF (VL53L0x)**: Conectado a los pines I2C del microcontrolador.
- **LEDs**: Utiliza un controlador de LEDs compatible con RGB.
- **Buzzer y Pulsador**: Conectados a los pines digitales del microcontrolador.



Esquematico
[Esquematico](doc/time-of-flight.pdf)

## Funcionalidad

1. **Calibración**: El operador debe presionar el pulsador para calibrar la distancia de peligro. La zona de precaución y la zona segura se definen en función de la distancia calibrada, separadas por 20 cm.
2. **Control del Usuario**: El sistema verifica la distancia del usuario constantemente. Dependiendo de la distancia medida, cambia el color del LED y activa el buzzer si es necesario.
3. **Pulsador**: Al presionar el pulsador, se activa o desactiva el buzzer cuando el usuario se encuentra en la zona de peligro.

## Estados

El sistema tiene cuatro posibles estados:

- `ST_UNKNOW`: Estado desconocido.
- `ST_DANGER`: El usuario está en la zona de peligro.
- `ST_WARNING`: El usuario está en la zona de precaución.
- `ST_SAFE`: El usuario está en la zona segura.

## Ciclo de Operación

1. **Inicialización**: Se cargan los valores de configuración y se realiza la calibración.
2. **Configuración**: El operador puede configurar el punto de peligro presionando el pulsador.
3. **Control**: El sistema mide la distancia constantemente y ajusta el estado según la proximidad del usuario.
4. **Buzzer**: El buzzer puede ser activado o desactivado mediante el pulsador.

## Requisitos

- **Arduino IDE**: Utilizar la versión 2.3.4 o superior.
- **Dispositivos Soportados**: Este proyecto está diseñado para trabajar con los microcontroladores Arduino Nano y Mega2560.

## Configuración

- **Distancia de Calibración**: El valor de calibración debe ser menor al alcance máximo del sensor (2 metros), menos las distancias de las franjas.
- **Bandas de Peligro**: Las zonas de peligro, precaución y segura se definen mediante la distancia calibrada.
- **Buzzer**: Se puede habilitar o deshabilitar el buzzer mediante el pulsador. Si está habilitado, el buzzer emite un sonido intermitente cuando el usuario está en la zona de peligro.

## Funciones Principales

- `check_max_calibration_distance(val)`: Verifica si la distancia de calibración está dentro de un rango permitido.
- `calibracion()`: Proceso de calibración donde el operador define la distancia de peligro.
- `control()`: Controla el estado del sistema según la distancia medida por el sensor.
- `hysteresis_off(val, next_point)`: Aplica histeresis para evitar cambios rápidos de estado.
- `get_state(val, last_state)`: Obtiene el estado actual basado en la distancia medida.
- `config_buzzer_on_tgl()`: Habilita o deshabilita el buzzer.
- `setup()`: Inicializa los periféricos y configura el sistema.
- `loop()`: El ciclo principal del sistema que controla el estado y la configuración.

## Autor

- JS (Juan Schiavoni): juanschiavoni@gmail.com

## Licencia

Este proyecto está bajo una licencia MIT.

## Licencia

Este proyecto está bajo una licencia MIT.




Arquitectura no bloqueante para el embebido en un proyecto real - C++ Parte1
Se evaluan los ventajas y desventajas de la arquitectura en un proyecto real
[![Introduccion al proyecto.](images/C5.png)](https://youtu.be/cBQaSfbyPL8)

Repositorio
https://github.com/jjsch-dev/vl53l0x-arduino 




