// incluir las liberias en el dominio de la libreria
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "uart.h"

// a led verde y rojo  -  //b led amarillo y rojo  -  //c led rojo
//  puerto del arduino A0-A4
#define LED_VERDE_PIN_A0 PC5
#define LED_ROJO_PIN_A1 PC4
#define LED_AMARILLO_PIN_A2 PC3
#define LED_ROJO_PIN_A3 PC2
#define LED_ROJO_PIN_A4 PC1
//#define LED_ROJO_PIN_A5 PC0
#define LED_PORT PORTC
#define LED_DDR DDRC
#define LED_PIN PINC

// seral port puertos 0 y 1
#define SERIAL_RX PD0
#define SERIAL_TX PD1
#define SERIAL_PORT PORTD
#define SERIAL_DDR DDRD
#define SERIAL_PIN PIND

// pwm para salida del arduino audio puerto 11
#define PWM_OUT PB3
#define PWM_PORT PORTB
#define PWM_DDR DDRB
#define PWM_PIN PINB
#define PWM_TCCR0A TCCR0A
#define PWM_TCCR0B TCCR0B
#define PWM_OCR0A OCR0A
#define PWM_OCR0B OCR0B
#define PWM_COMPARE_A 0x00
#define PWM_COMPARE_B 0x01
#define PWM_WGM_A1 WGM00
#define PWM_WGM_A0 WGM01
#define PWM_OCR0A OCR0A
#define PWM_OCR0B OCR0B
#define PWM_ENABLE_A 0x00
#define PWM_ENABLE_B 0x02

// ultrasonico puertos del arduino 2 y 3
#define ULTRASONIC_TRIG PD2
#define ULTRASONIC_ECHO PD3
#define ULTRASONIC_PORT PORTD
#define ULTRASONIC_DDR DDRD
#define ULTRASONIC_PIN PIND

// declaracion de variables globales
// variables para el usuario y contraseña
// definimos un usuario y contraseña
char usuario_definido[11] = "Supervisor";
char contrasena_definido[10] = "3mb3b1D0s";
char usuario_recibido[10];
char contrasena_recibida[10];
char mensaje[20];

void setup();
int ultrasonico_distancia();
char *ultrasonico_distancia_str();
void sonido_sonar(int distancia);
void sonido_parar();
void parquear();

int main()
{
  setup();

  while (1)
  {

    parquear();

    if (isconected)
    {
      serial_print_str("Usuario: ");
      while (compare_str(usuario_recibido, usuario_definido) == false)
      {
        if (is_data_ready())
        {
          serial_println_str(get_RX_buffer());
          strcpy(usuario_recibido, get_RX_buffer());
        }
        // esperamos
        _delay_ms(1000);
        parquear();
      }
      serial_print_str("Contraseña: ");
      while (compare_str(contrasena_recibida, contrasena_definido) == false)
      {
        if (is_data_ready())
        {
          serial_println_str(get_RX_buffer());
          strcpy(contrasena_recibida, get_RX_buffer());
        }
        // esperamos
        _delay_ms(1000);
        parquear();
      }
      // mostramos la distancia del ultrasonico
      while (true)
      {
        serial_println_str(ultrasonico_distancia_str());
        _delay_ms(1000);
        parquear();
      }
    }
  }

  return 0;
}

// setup
void setup()
{
  // config de leds
  LED_DDR |= (1 << LED_VERDE_PIN_A0);
  LED_DDR |= (1 << LED_ROJO_PIN_A1);
  LED_DDR |= (1 << LED_AMARILLO_PIN_A2);
  LED_DDR |= (1 << LED_ROJO_PIN_A3);
  LED_DDR |= (1 << LED_ROJO_PIN_A4);

  // config serial a 9600 baudios
  serial_begin(9600);

  // config pwm para salida del arduino audio
  PWM_DDR |= (1 << PWM_OUT);

  // config ultrasonico
  ULTRASONIC_DDR &= ~(1 << ULTRASONIC_TRIG);
  ULTRASONIC_DDR |= (1 << ULTRASONIC_ECHO);
}

// funcion ultrasonico_distancia() - devuelve la distancia en cm
int ultrasonico_distancia()
{
  // configuracion de la salida del ultrasonico
  ULTRASONIC_PORT &= ~(1 << ULTRASONIC_TRIG);
  _delay_ms(2);
  ULTRASONIC_PORT |= (1 << ULTRASONIC_TRIG);
  _delay_ms(10);
  ULTRASONIC_PORT &= ~(1 << ULTRASONIC_TRIG);
  // esperamos a que el echo llegue
  while (!(ULTRASONIC_PIN & (1 << ULTRASONIC_ECHO)))
  {
    _delay_ms(1);
  }
  // calculamos la distancia
  int distancia = 0;
  while (ULTRASONIC_PIN & (1 << ULTRASONIC_ECHO))
  {
    distancia++;
    //_delay_ms(1);
  }
  return distancia;
}

// funcion debuelve un string de la distancia en cm
char *ultrasonico_distancia_str()
{
  // configuracion de la salida del ultrasonico
  ULTRASONIC_PORT &= ~(1 << ULTRASONIC_TRIG);
  _delay_ms(2);
  ULTRASONIC_PORT |= (1 << ULTRASONIC_TRIG);
  _delay_ms(10);
  ULTRASONIC_PORT &= ~(1 << ULTRASONIC_TRIG);
  // esperamos a que el echo llegue
  while (!(ULTRASONIC_PIN & (1 << ULTRASONIC_ECHO)))
  {
    _delay_ms(1);
  }
  // calculamos la distancia
  int distancia = 0;
  while (ULTRASONIC_PIN & (1 << ULTRASONIC_ECHO))
  {
    distancia++;
    //_delay_ms(1);
  }
  // convertimos la distancia a string
  sprintf(mensaje, "%d", distancia);
  return mensaje;
}

// funcion sonido_sonar(distancia) - hace sonar el arduino audio
// la intensidad del sonido depende de la distancia
void sonido_sonar(int distancia)
{
  // calculamos la intensidad del sonido
  int intensidad = (distancia * 100) / 1000;
  // configuracion de la salida del arduino audio
  PWM_PORT &= ~(1 << PWM_OUT);
  PWM_DDR |= (1 << PWM_OUT);
  // configuracion de la salida del arduino audio
  PWM_TCCR0A = (1 << PWM_COMPARE_A) | (1 << PWM_WGM_A1) | (1 << PWM_WGM_A0);
  PWM_OCR0A = intensidad;
}

void sonido_parar()
{
  // configuracion de la salida del arduino audio
  PWM_PORT &= ~(1 << PWM_OUT);
  PWM_DDR |= (1 << PWM_OUT);
  // configuracion de la salida del arduino audio
  PWM_TCCR0A = (1 << PWM_COMPARE_A) | (1 << PWM_WGM_A1) | (1 << PWM_WGM_A0);
  PWM_OCR0A = 0;
}

// funcion de control de leds
void parquear()
{
  // preguntamos el valor de la distancia
  int distancia = ultrasonico_distancia();
  // si la distancia es superior a 5265 mm se enciende el led verde
  if (distancia > 5265)
  {
    LED_PORT |= (1 << LED_VERDE_PIN_A0);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A1);
    LED_PORT &= ~(1 << LED_AMARILLO_PIN_A2);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A3);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A4);
    // LED_PORT &= ~(1 << LED_ROJO_PIN_A5);
    sonido_parar();
  }
  // si la sistancia es inferior a 5265 mm se enciende el led amarillo
  else if (distancia < 5265 && distancia > 3030)
  {
    LED_PORT |= (1 << LED_AMARILLO_PIN_A2);
    LED_PORT &= ~(1 << LED_VERDE_PIN_A0);
    LED_PORT |= (1 << LED_ROJO_PIN_A1);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A3);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A4);
    // LED_PORT &= ~(1 << LED_ROJO_PIN_A5);
    sonido_parar();
  }
  // si la distancia es inferior a 2030 mm se enciende el led rojo
  else if (distancia < 3030 && distancia > 1000)
  {
    LED_PORT |= (1 << LED_ROJO_PIN_A3);
    LED_PORT &= ~(1 << LED_VERDE_PIN_A0);
    LED_PORT |= (1 << LED_ROJO_PIN_A1);
    LED_PORT &= ~(1 << LED_AMARILLO_PIN_A2);
    LED_PORT &= ~(1 << LED_ROJO_PIN_A4);
    // LED_PORT &= ~(1 << LED_ROJO_PIN_A5);
    sonido_parar();
  }
  // si la distancia es inferior a 1000 mm se enciende el led rojo
  else if (distancia < 1000)
  {
    LED_PORT |= (1 << LED_ROJO_PIN_A4);
    LED_PORT &= ~(1 << LED_VERDE_PIN_A0);
    LED_PORT |= (1 << LED_ROJO_PIN_A1);
    LED_PORT &= ~(1 << LED_AMARILLO_PIN_A2);
    LED_PORT |= (1 << LED_ROJO_PIN_A3);
    // LED_PORT &= ~(1 << LED_ROJO_PIN_A5);
    sonido_sonar(distancia);
  }
}
