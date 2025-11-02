#include <Controllino.h>          // Librería para manejar las entradas/salidas del Controllino
#include "Stone_HMI_Define.h"     // Definiciones para la comunicación con la pantalla HMI STONE
#include "Procesar_HMI.h"         // Funciones personalizadas para procesar datos del HMI

// ==========================
// Definición de pines
// ==========================
const int led    = CONTROLLINO_D0;   // LED 1 conectado a la salida digital D0
const int led2   = CONTROLLINO_D2;   // LED 2 conectado a la salida digital D2
const int led3   = CONTROLLINO_D12;  // LED 3 conectado a la salida digital D12 (indicador)
const int led4   = CONTROLLINO_D14;  // LED 4 conectado a la salida digital D14 (indicador)

const int boton1 = CONTROLLINO_I16;  // Botón 1 conectado a la entrada digital I16
const int boton2 = CONTROLLINO_I18;  // Botón 2 conectado a la entrada digital I18

// ==========================
// Variables de control
// ==========================
int pulsante1 = 0, pulsante2 = 0;     // Lecturas de los botones
int onoff1 = 0, onoff2 = 0;           // Contadores de estados ON/OFF
int pwmValue = 0, pwmValue2 = 0;      // Valores PWM (0–255)
float dutyCyclePercent = 0, duty2 = 0;// Duty cycle en porcentaje (0–100)

// ==========================
// Configuración inicial
// ==========================
void setup() {
  Serial.begin(115200);    // Comunicación serial con el PC (para monitoreo)
  Serial2.begin(115200);   // Comunicación serial con la pantalla HMI STONE

  // Configuración de pines de salida
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  // Configuración de pines de entrada
  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);

  HMI_init();  // Inicializa la comunicación y colas del sistema HMI

  // Inicializa el valor del componente “spin_box1” en 0 al iniciar
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 0);
}

// ==========================
// Bucle principal
// ==========================
void loop() {
  // Lee desde el HMI los valores de los controles de tipo spin box
  dutyCyclePercent = HMI_get_value("spin_box", "spin_box1"); // Valor del primer control
  duty2 = HMI_get_value("spin_box", "spin_box2");             // Valor del segundo control

  // Lectura de botones físicos
  pulsante1 = digitalRead(boton1);
  if (pulsante1 == HIGH) {
    onoff1++; // Cada pulsación incrementa el contador
  }

  pulsante2 = digitalRead(boton2);
  if (pulsante2 == HIGH) {
    onoff2++;
  }

  // ==========================
  // Control PWM para LED2
  // ==========================
  if ((duty2 >= 0 && duty2 <= 100) && (onoff2 != 1)) {
    pwmValue2 = map(duty2, 0, 100, 0, 255); // Conversión de 0–100% a 0–255
    analogWrite(led2, pwmValue2);           // Salida PWM
    Serial.print("Duty cycle 2(%): ");
    Serial.print(duty2);
    Serial.print(" -> PWM value: ");
    Serial.println(pwmValue2);
  } else {
    Serial.println("Ingresa un valor entre 0 y 100.");
  }

  // ==========================
  // Control PWM para LED1
  // ==========================
  if ((dutyCyclePercent >= 0 && dutyCyclePercent <= 100) && (onoff1 != 1)) {
    pwmValue = map(dutyCyclePercent, 0, 100, 0, 255); // Conversión de 0–100% a 0–255
    analogWrite(led, pwmValue);
    Serial.print("Duty cycle (%): ");
    Serial.print(dutyCyclePercent);
    Serial.print(" -> PWM value: ");
    Serial.println(pwmValue);
  } else {
    Serial.println("Ingresa un valor entre 0 y 100.");
  }

  // ==========================
  // Control ON/OFF LED3 asociado al LED1
  // ==========================
  if (onoff1 == 1) {
    digitalWrite(led3, HIGH); // Enciende LED indicador
    analogWrite(led, 0);      // Apaga la salida PWM
    Serial.println("LED1 apagado (modo OFF)");
  } else if (onoff1 == 2) {
    digitalWrite(led3, LOW);  // Apaga indicador y reinicia estado
  } else {
    onoff1 = 0;
  }

  // ==========================
  // Control ON/OFF LED4 asociado al LED2
  // ==========================
  if (onoff2 == 1) {
    digitalWrite(led4, HIGH); // Enciende LED indicador
    analogWrite(led2, 0);     // Apaga la salida PWM
    Serial.println("LED2 apagado (modo OFF)");
  } else if (onoff2 == 2) {
    digitalWrite(led4, LOW);  // Apaga indicador y reinicia estado
  } else {
    onoff2 = 0;
  }
}
