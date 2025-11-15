#include <Controllino.h>  // Manejo de pines del Controllino

// Arreglo con los pines de los 9 LEDs
char leds[9] = { 
  CONTROLLINO_D0, CONTROLLINO_D6, CONTROLLINO_D12,
  CONTROLLINO_D13, CONTROLLINO_D14, CONTROLLINO_D8,
  CONTROLLINO_D2, CONTROLLINO_D1, CONTROLLINO_D7 
};

// Puntero al inicio del arreglo (acceso por desplazamiento)
char* index = leds;

// Variables de temporización y control de secuencia
int t_previo = 0;        // Marca del último cambio de LED
int iteracion = 0;       // LED actualmente encendido
int t_actual = 0;        // Tiempo actual leído con millis()
int intervalo = 1000;    // Tiempo entre cambios
int off = 0;             // LED que se apagará en la transición
boolean cambio = true;   // Habilita transición de estado
int izq = 0, der = 0;    // Dirección de desplazamiento

// Entradas
const int botonizq = CONTROLLINO_I16;
const int botonParo = CONTROLLINO_I17;
const int botonder = CONTROLLINO_I18;

void setup() {
  // Configuración de pines de salida para LEDs
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT);
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);
  pinMode(CONTROLLINO_D8, OUTPUT);
  pinMode(CONTROLLINO_D12, OUTPUT);
  pinMode(CONTROLLINO_D13, OUTPUT);
  pinMode(CONTROLLINO_D14, OUTPUT);

  // Botones
  pinMode(botonder, INPUT);
  pinMode(botonParo, INPUT);
  pinMode(botonizq, INPUT);
}

void loop() {

  t_actual = millis();   // Tiempo actual

  // Dirección hacia la derecha
  if (digitalRead(botonder) == HIGH){
    der = 1;
    izq = 0;
    cambio = true;
  
  // Dirección hacia la izquierda
  } else if (digitalRead(botonizq) == HIGH){
    iteracion = 9;
    der = 0;
    izq = 1;
    cambio = true;
  }

  // Temporizador no bloqueante
  if (t_actual - t_previo >= intervalo) {
    t_previo = t_actual;

    // Avance hacia la derecha
    if (der == 1){
      iteracion++;
      off = iteracion - 1;

    // Avance hacia la izquierda
    } else if (izq == 1){
      iteracion--;
      off = iteracion + 1;
    }
  }

  // Control del encendido/apagado de LEDs mientras no se presione PARO
  if (cambio == true && digitalRead(botonParo) == LOW) {
    digitalWrite(*(index + iteracion), HIGH);  // LED ON
    digitalWrite(*(index + off), LOW);         // LED OFF

    // Reinicio de secuencia derecha
    if (iteracion == 9 && der == 1) {
      iteracion = 0;

    // Reinicio de secuencia izquierda
    } else if (iteracion == 0 && izq == 1) {
      iteracion = 9;
    }

  } else if (digitalRead(botonParo) == HIGH) {
    // Apagar LED actual al presionar PARO
    digitalWrite(*(index + iteracion), LOW);
    cambio = false;
    iteracion = 0;
  }
}
