#include <Controllino.h>

// -------------------- Definiciones --------------------
// Estado para la máquina de estados del semáforo
enum EstadoSemaforo { ROJO, AMARILLO, VERDE };

struct Semaforo {
  int ledRojo;
  int ledAmarillo;
  int ledVerde;
  EstadoSemaforo estado;   // Estado actual dentro de la FSM
};

// -------------------- Pines --------------------
Semaforo semA = {CONTROLLINO_D0, CONTROLLINO_D1, CONTROLLINO_D2, ROJO};
Semaforo semB = {CONTROLLINO_D12, CONTROLLINO_D13, CONTROLLINO_D14, VERDE};

const int botonInicio = CONTROLLINO_I16;
const int botonParo   = CONTROLLINO_I17;

// -------------------- Variables de tiempo --------------------
// Variables necesarias para el temporizador no bloqueante, parte clave del control de la FSM
unsigned long t_previo = 0;
unsigned long t_actual = 0;
unsigned long intervalo = 0;

// -------------------- Tiempos de luz (ms) --------------------
const unsigned long tVerde = 5000;
const unsigned long tAmarillo = 2000;
const unsigned long tRojo = 5000;

// -------------------- Control --------------------
bool sistemaActivo = false;
bool cambio = true;

// -------------------- Setup --------------------
void setup() {
  pinMode(semA.ledRojo, OUTPUT);
  pinMode(semA.ledAmarillo, OUTPUT);
  pinMode(semA.ledVerde, OUTPUT);

  pinMode(semB.ledRojo, OUTPUT);
  pinMode(semB.ledAmarillo, OUTPUT);
  pinMode(semB.ledVerde, OUTPUT);

  pinMode(botonInicio, INPUT);
  pinMode(botonParo, INPUT);

  actualizarSemaforos();
}

// -------------------- Loop --------------------
void loop() {

  // Lectura de eventos externos -> entrada para la FSM
  if (digitalRead(botonInicio) == HIGH) sistemaActivo = true;
  if (digitalRead(botonParo) == HIGH)   sistemaActivo = false;

  // Estado "apagado" del sistema
  if (!sistemaActivo) {
    apagarSemaforos();
    return;
  }

  // Temporizador -> condición para transición de estado
  t_actual = millis();
  if (t_actual - t_previo >= intervalo) {
    t_previo = t_actual;
    cambio = true;
  }

  // Si toca cambiar de estado, ejecutar transición en la FSM
  if (cambio) {
    cambio = false;
    avanzarSemaforos();
  }
}

// -------------------- Funciones --------------------
// Máquina de estados: define las transiciones entre ROJO -> VERDE -> AMARILLO
void avanzarSemaforos() {
  switch (semA.estado) {   // Evaluación del estado actual
    case VERDE:            // Transición desde VERDE
      semA.estado = AMARILLO;
      semB.estado = ROJO;
      intervalo = tAmarillo;
      break;

    case AMARILLO:         // Transición desde AMARILLO
      semA.estado = ROJO;
      semB.estado = VERDE;
      intervalo = tRojo;
      break;

    case ROJO:             // Transición desde ROJO
      semA.estado = VERDE;
      semB.estado = AMARILLO;
      intervalo = tVerde;
      break;
  }
  
  actualizarSemaforos();   // Salida de la FSM -> actualizar LEDs
}

// Actualiza las salidas según el estado actual de la FSM
void actualizarSemaforos() {
  // Semáforo A
  digitalWrite(semA.ledRojo, semA.estado == ROJO);
  digitalWrite(semA.ledAmarillo, semA.estado == AMARILLO);
  digitalWrite(semA.ledVerde, semA.estado == VERDE);

  // Semáforo B
  digitalWrite(semB.ledRojo, semB.estado == ROJO);
  digitalWrite(semB.ledAmarillo, semB.estado == AMARILLO);
  digitalWrite(semB.ledVerde, semB.estado == VERDE);
}

// Estado "apagado" para cuando el sistema no está activo
void apagarSemaforos() {
  digitalWrite(semA.ledRojo, LOW);
  digitalWrite(semA.ledAmarillo, LOW);
  digitalWrite(semA.ledVerde, LOW);
  digitalWrite(semB.ledRojo, LOW);
  digitalWrite(semB.ledAmarillo, LOW);
  digitalWrite(semB.ledVerde, LOW);
}
