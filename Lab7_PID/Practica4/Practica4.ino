// LIBRERÍAS
#include <Controllino.h>
#include "Stone_HMI_Define.h" // Librería oficial de HMI Stone
#include "Procesar_HMI.h"     // Librería implementada para procesar respuestas del HMI

// =======================================================
// === VARIABLES PARA PWM DEL MOTOR ======================
// =======================================================
const int pin_motor = CONTROLLINO_D0;
int8_t slider_DutyCycle = 0;
char label2_text[10];

// =======================================================
// === VARIABLES PARA CONTEO DE PULSOS y RPM =============
// =======================================================
const int entrada = CONTROLLINO_IN1;
volatile unsigned long conteo_pulsos = 0;
char label4_text[10];
float rpm = 0;
const uint16_t PULSOS_POR_REV = 36;
const float fs = 20.0;    // frecuencia de muestreo en Hz
const float T = 1.0 / fs; // tiempo de muestreo (s)

// =======================================================
// === VARIABLES DE CONTROL ==============================
// =======================================================
float uk = 0, uk1 = 0;      // Salida del controlador (PWM actual y previa)
float ek = 0, ek1 = 0, ek2 = 0; // Errores (actual, previo y dos pasos atrás)


//DE 0 A 255
float Kp = 0.1;              // Ganancia proporcional
float Ti = 0.075;             // Tiempo integral (ajustar)
float Td = 0.02;            // Tiempo derivativo (ajustar)

char labelP_text[10];
char labelI_text[10];
char labelD_text[10];

// // LabView Valores sintonizados para estos es necesario usar el map y la restriccion
// que esta comentado abajo en la parte de la interrupcion
// float Kp = 0.6;              // Ganancia proporcional
// float Ti = 0.15;             // Tiempo integral (ajustar)
// float Td = 0.04;            // Tiempo derivativo (ajustar)

//valores iniciales calculados de labview
// float Kp = 0.357;              // Ganancia proporcional
// float Ti = 0.325;             // Tiempo integral (ajustar)
// float Td = 0.081;            // Tiempo derivativo (ajustar)



float rpmsetpoint = 0;

const int botonemergencia = CONTROLLINO_I16;
const int botcontinuar = CONTROLLINO_I18;
const int ledemerg = CONTROLLINO_D12;
const int ledcont = CONTROLLINO_D14;
int apagar = 0;

// =======================================================
// === VARIABLES DE TIEMPO Y OTROS ========================
// =======================================================
unsigned long t_previo = 0;
unsigned long t_previo1 = 0;

// =======================================================
// === VARIABLES PARA MEDIR TU (PERÍODO DE OSCILACIÓN) ===
// =======================================================
float rpm_anterior = 0;
unsigned long tiempo_cruce_anterior = 0;
unsigned long tiempo_cruce_actual = 0;
float Tu = 0;
bool cruce_detectado = false;
float rpm_media = 0;
float Tu_promedio = 0;
int conteo_cruces = 0;
const int N_CRUCES = 5;
char label_Tu[10];

// =======================================================
// === PROTOTIPO DE FUNCIÓN ==============================
// =======================================================
void contarPulso();

// =======================================================
// === CONFIGURACIÓN INICIAL =============================
// =======================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  STONE_push_series("line_series", "line_series1", 0);
  STONE_push_series("line_series", "line_series2", 0);
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0);

  pinMode(entrada, INPUT);
  pinMode(botonemergencia, INPUT);
  pinMode(botcontinuar, INPUT);
  pinMode(pin_motor, OUTPUT);
  pinMode(ledemerg, OUTPUT);
  pinMode(ledcont, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(entrada), contarPulso, FALLING);

  // --- Configuración del Timer1 para muestreo ---
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0b00000100;  // preescaler 256
  TIMSK1 |= B00000010;  // interrupción por comparación
  OCR1A = 62500 / fs;   // cada 1/fs segundos
  interrupts();

  HMI_init();

  Serial.println("Sistema iniciado con PID discreto.");
}

// =======================================================
// === BUCLE PRINCIPAL ===================================
// =======================================================
void loop() {
  // ------------------- CONTROL DE EMERGENCIA -------------------
  if ((digitalRead(botonemergencia) == HIGH) && (digitalRead(botcontinuar) == LOW)) {
    
    digitalWrite(ledemerg, HIGH);
    digitalWrite(ledcont, LOW);
    apagar = 1;
  } else if ((digitalRead(botcontinuar) == HIGH) && (digitalRead(botonemergencia) == LOW)) {
    digitalWrite(ledemerg, LOW);
    digitalWrite(ledcont, HIGH);
    apagar = 0;
  }

  // ------------------- ACTUALIZACIÓN DEL SETPOINT -------------------
  if (millis() - t_previo1 >= 10) {
    slider_DutyCycle = HMI_get_value("slider", "slider1");
    rpmsetpoint = map(slider_DutyCycle, 0, 100, 0, 6000);
    t_previo1 = millis();
  }

  // ------------------- MONITOREO EN SERIE -------------------
  Serial.print("Setpoint: "); Serial.print(rpmsetpoint);
  Serial.print(" | RPM: "); Serial.print(rpm);
  Serial.print(" | e(k): "); Serial.print(ek);
  Serial.print(" | PID: "); Serial.print(uk);
  Serial.println();

  // =======================================================
  // === DETECCIÓN DE CRUCES PARA MEDIR TU ================
  // =======================================================
  if (rpm > 100) {
    rpm_media = 0.95 * rpm_media + 0.05 * rpm;
    if (rpm_anterior < rpm_media && rpm >= rpm_media) {
      tiempo_cruce_actual = millis();
      if (cruce_detectado) {
        unsigned long periodo = tiempo_cruce_actual - tiempo_cruce_anterior;
        Tu = (float)periodo / 1000.0;
        conteo_cruces++;
        Tu_promedio = ((Tu_promedio * (conteo_cruces - 1)) + Tu) / conteo_cruces;
        if (conteo_cruces > N_CRUCES) conteo_cruces = N_CRUCES;
        Serial.print("Tu actual: "); Serial.print(Tu);
        Serial.print(" s | Tu promedio: "); Serial.print(Tu_promedio);
        Serial.println(" s");
      }
      tiempo_cruce_anterior = tiempo_cruce_actual;
      cruce_detectado = true;
    }
    rpm_anterior = rpm;
  }

  // ------------------- ACTUALIZAR DATOS EN EL HMI -------------------
  if (millis() - t_previo >= 100) {
    t_previo = millis();
    dtostrf(rpmsetpoint, 7, 2, label2_text);
    dtostrf(rpm, 7, 2, label4_text);
    
    dtostrf(Kp, 7, 2, labelP_text);
    dtostrf(Ti, 7, 2, labelI_text);
    dtostrf(Td, 7, 2, labelD_text);
    
    Stone_HMI_Set_Text("label", "label2", label2_text);
    Stone_HMI_Set_Text("label", "label4", label4_text);
    Stone_HMI_Set_Text("label", "P", labelP_text);
    Stone_HMI_Set_Text("label", "I", labelI_text);
    Stone_HMI_Set_Text("label", "D", labelD_text);


    STONE_push_series("line_series", "line_series1", rpmsetpoint);
    STONE_push_series("line_series", "line_series2", rpm);
    STONE_push_series("line_series", "comparativa", uk);
  }
}

// =======================================================
// === ISR PARA CÁLCULO DE RPM ============================
// =======================================================
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  rpm = (float(conteo_pulsos) * 60.0 * fs) / PULSOS_POR_REV;
  conteo_pulsos = 0;


    // === Cálculo del error actual ===
    ek = rpmsetpoint - rpm;

    // === PID discreto por ecuación de recurrencia ===
    uk = uk1 + Kp * (
      (ek - ek1)
      + (T / Ti) * ek
      + (Td / T) * (ek - 2 * ek1 + ek2)
    );

    // // Limita la salida del PID
    // uk = constrain(uk, 0, 255);

    //intento de mapeo para usar los valores de labview
    uk = constrain(uk, 0, 5);
    uk = map(uk, 0, 5, 0,255);

    if (apagar == 0){
    // Aplica al motor
      analogWrite(pin_motor, uk);
    } else {
      analogWrite(pin_motor, 0);
      uk=0;
    }
    // === Actualiza variables para el siguiente ciclo ===
    uk1 = uk;
    ek2 = ek1;
    ek1 = ek;
}

// =======================================================
// === ISR DE PULSOS =====================================
// =======================================================
void contarPulso() {
  conteo_pulsos++;
}
