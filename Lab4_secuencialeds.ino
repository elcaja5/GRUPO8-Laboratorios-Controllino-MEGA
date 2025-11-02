// Práctica 2: Encender todos los LEDs del tablero secuencialmente

#include <Controllino.h>  // Librería necesaria para manejar los pines del módulo Controllino

// Arreglo con los pines digitales donde están conectados los LEDs
char leds[9] = { 
  CONTROLLINO_D0, CONTROLLINO_D6, CONTROLLINO_D12,
  CONTROLLINO_D13, CONTROLLINO_D14, CONTROLLINO_D8,
  CONTROLLINO_D2, CONTROLLINO_D1, CONTROLLINO_D7 
};

// Puntero que apunta al primer elemento del arreglo de LEDs
char* index = leds;

// Variables para control de tiempo y secuencia
int t_previo = 0;       // Guarda el último instante en que se cambió de LED
int iteracion = 0;      // Índice actual del LED encendido
int t_actual = 0;       // Tiempo actual leído con millis()
int intervalo = 1000;   // Intervalo de tiempo entre cada cambio (en milisegundos)
int off = 0;            // Índice del LED que se apaga
boolean cambio = true;  // Bandera para habilitar el cambio de estado

void setup() {
  pinMode(CONTROLLINO_D0, OUTPUT);   // Salida digital D0
  pinMode(CONTROLLINO_D1, OUTPUT);   // Salida digital D1
  pinMode(CONTROLLINO_D2, OUTPUT);   // Salida digital D2
  pinMode(CONTROLLINO_D6, OUTPUT);   // Salida digital D6
  pinMode(CONTROLLINO_D7, OUTPUT);   // Salida digital D7
  pinMode(CONTROLLINO_D8, OUTPUT);   // Salida digital D8
  pinMode(CONTROLLINO_D12, OUTPUT);  // Salida digital D12
  pinMode(CONTROLLINO_D13, OUTPUT);  // Salida digital D13
  pinMode(CONTROLLINO_D14, OUTPUT);  // Salida digital D14
}

void loop() {
  // Se obtiene el tiempo actual desde que se inició el programa
  t_actual = millis();

  // Si ha pasado el intervalo de tiempo definido, se avanza a la siguiente iteración
  if (t_actual - t_previo >= intervalo) {
    t_previo = t_actual;  // Se actualiza el tiempo de referencia
    iteracion++;          // Se incrementa el índice del LED activo
    off = iteracion - 1;  // Se define cuál LED debe apagarse (el anterior)
  }

  // Si el cambio está habilitado, se enciende el LED actual y se apaga el anterior
  if (cambio == true) {
    digitalWrite(*(index + iteracion), HIGH);  // Enciende el LED actual
    digitalWrite(*(index + off), LOW);         // Apaga el LED anterior
  }

  // Cuando se llega al último LED, se reinicia la secuencia
  if (iteracion == 9) {
    iteracion = 0;
  }
}
