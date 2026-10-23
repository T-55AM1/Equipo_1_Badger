// Arduino UNO (ATmega328P), L298N, encoder A/B, ACS712-20A.
// Leer README.md ANTES de conectar. No requiere libreria Teleplot.
#include <Arduino.h>
#include <math.h>
#include <util/atomic.h>

// ---------------- CONFIGURACION DEL USUARIO ----------------
constexpr uint8_t ENA = 9, IN1 = 7, IN2 = 8;
constexpr uint8_t ENC_A = 2, ENC_B = 3, START = 4, STOP = 5;
constexpr uint8_t POT = A0, CURRENT = A1;
// OBLIGATORIO: cuentas por vuelta del EJE DE SALIDA, contando SOLO
// flancos ascendentes de A (x1). 0 bloquea el arranque hasta calibrar.
constexpr float COUNTS_PER_OUTPUT_REV = 500.0f; //330 predet..##################################
constexpr int8_t ENCODER_SIGN = 1;  // Cambiar a -1 si RPM sale negativa.
constexpr uint32_t SIGNAL_MS = 15000; // 10 s de operacion por senal. 10k predet...################
constexpr uint32_t GAP_MS = 3000; // Opcional: 10000 = 10 s APAGADO entre senales.
constexpr uint32_t CONTROL_MS = 50, PLOT_MS = 100;
constexpr bool USE_POT = true; // Se lee una vez al pulsar INICIO.
constexpr float FIXED_SCALE = 0.50f; // Si USE_POT=false, escala 0..1.
constexpr float RPM_LIMIT = 500.0f; // Limite de referencia, no velocidad garantizada. 170 predet..##################
constexpr float NOMINAL_RPM = 200.0f; // Solo estimacion para prealimentacion. 170 predet...###################
//-------------------------------- Linea modificada -------------------------
constexpr int PWM_MAX = 130; // Limite inicial conservador: 0..255. 255 predet..#############################
//--------------------------------- Linea modificada ------------------------------------------
constexpr float KP = 200.0f, KI = 100.0f; // Valores INICIALES, requieren ajuste real.
//-----------------------------------------------------------------------------------
constexpr float RPM_FILTER = 0.75f; // 0..1; 1 elimina el filtrado.
constexpr float ADC_VCC = 5.0f, ACS_V_PER_A = 0.100f;
// ---------------- Linea modificada ----------------
constexpr float CURRENT_TRIP_A = 2.0f; // Umbral inicial, NO dato del motor.
//----------------------------------------------------
constexpr uint32_t OVERCURRENT_MS = 100;
constexpr int NO_ENCODER_PWM = 80;
constexpr uint32_t NO_ENCODER_MS = 800;

constexpr float CONSTANT_RPM = 80.0f;
constexpr float RAMP_START = 0.0f, RAMP_END = 110.0f;
constexpr float TRAP_LOW = 0.0f, TRAP_HIGH = 110.0f;
constexpr float TRAP_ACCEL_FRACTION = 0.25f, TRAP_DECEL_START = 0.75f;
constexpr float SINE_OFFSET = 65.0f, SINE_AMPLITUDE = 35.0f;
constexpr float SINE_HZ = 0.2f, SINE_PHASE_RAD = 0.0f;
constexpr float SQUARE_LOW = 30.0f, SQUARE_HIGH = 100.0f;
constexpr float SQUARE_PERIOD_S = 4.0f;
// La imagen llama "exponencial" a una ley de POTENCIA; se conserva su formula.
constexpr float POWER_P0 = 20.0f, POWER_ALPHA = 0.25f, POWER_K = 1.3f;
constexpr float GAUSS_PEAK = 100.0f, GAUSS_CENTER_S = 5.0f, GAUSS_SIGMA_S = 1.7f;
constexpr float SAW_PEAK = 100.0f, SAW_PERIOD_S = 3.0f;
constexpr float NOISE_OFFSET = 65.0f, NOISE_AMPLITUDE = 25.0f;
constexpr uint32_t NOISE_HOLD_MS = 100;
// -----------------------------------------------------------
static_assert(SIGNAL_MS > 0 && CONTROL_MS > 0, "Tiempos positivos");
static_assert(PWM_MAX > 0 && PWM_MAX <= 255, "PWM fuera de rango");
static_assert(SQUARE_PERIOD_S > 0 && SAW_PERIOD_S > 0 && GAUSS_SIGMA_S > 0,
              "Periodos y sigma deben ser positivos");
static_assert(TRAP_ACCEL_FRACTION > 0 && TRAP_ACCEL_FRACTION < TRAP_DECEL_START
              && TRAP_DECEL_START < 1, "Tramos trapezoidales invalidos");
static_assert(NOISE_HOLD_MS > 0 && POWER_ALPHA >= 0 && POWER_K > 0,
              "Parametros de potencia/ruido invalidos");

volatile int32_t encoderCount = 0;
void encoderISR() { encoderCount += digitalRead(ENC_B) ? -1 : 1; }
int32_t readCount() {
  int32_t n;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { n = encoderCount; }
  return n;
}
float clip(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

enum State : uint8_t { IDLE = 0, RUN = 1, GAP = 2, DONE = 3, FAULT = 4 };
State state = IDLE;
uint8_t signalIndex = 0, faultCode = 0;
uint32_t phaseStart = 0, lastControl = 0, lastPlot = 0, lastSample = 0;
uint32_t overSince = 0, noEncoderSince = 0;
bool overActive = false, noEncoderActive = false;
int32_t lastCount = 0;
float referenceRPM = 0, measuredRPM = 0, integral = 0, amplitudeScale = 0;
float zeroADC = 512, currentA = 0, currentAbsA = 0;
float currentSum = 0, currentAbsSum = 0;
uint8_t currentSamples = 0;
int appliedPWM = 0;
uint32_t noiseBucket = 0xFFFFFFFFUL;
float noiseValue = 0;

void motorOff() {
  analogWrite(ENA, 0); digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  appliedPWM = 0; referenceRPM = 0; integral = 0;
  overActive = false; noEncoderActive = false;
}
void fail(uint8_t code) { motorOff(); faultCode = code; state = FAULT; }

// Devuelve referencia de VELOCIDAD, no posicion ni voltaje analogico.
float signalRPM(uint8_t index, uint32_t elapsed) {
  const float t = elapsed * 0.001f;
  const float x = clip(static_cast<float>(elapsed) / SIGNAL_MS, 0, 1);
  switch (index) {
    case 0: return CONSTANT_RPM;
    case 1: return RAMP_START + (RAMP_END - RAMP_START) * x;
    case 2:
      if (x < TRAP_ACCEL_FRACTION)
        return TRAP_LOW + (TRAP_HIGH - TRAP_LOW) * x / TRAP_ACCEL_FRACTION;
      if (x < TRAP_DECEL_START) return TRAP_HIGH;
      return TRAP_HIGH - (TRAP_HIGH - TRAP_LOW) *
             (x - TRAP_DECEL_START) / (1.0f - TRAP_DECEL_START);
    case 3: return SINE_OFFSET + SINE_AMPLITUDE *
                   sinf(2.0f * PI * SINE_HZ * t + SINE_PHASE_RAD);
    case 4: return fmodf(t, SQUARE_PERIOD_S) < SQUARE_PERIOD_S / 2 ?
                   SQUARE_LOW : SQUARE_HIGH;
    case 5: return POWER_P0 * powf(1.0f + POWER_ALPHA * t, POWER_K);
    case 6: {
      const float z = (t - GAUSS_CENTER_S) / GAUSS_SIGMA_S;
      return GAUSS_PEAK * expf(-z * z);
    }
    case 7: return SAW_PEAK * fmodf(t, SAW_PERIOD_S) / SAW_PERIOD_S;
    case 8: {
      const uint32_t bucket = elapsed / NOISE_HOLD_MS;
      if (bucket != noiseBucket) {
        noiseBucket = bucket;
        noiseValue = random(-10000L, 10001L) / 10000.0f;
      }
      return NOISE_OFFSET + NOISE_AMPLITUDE * noiseValue;
    }
    default: return 0;
  }
}

void startSequence(uint32_t now) {
  if (COUNTS_PER_OUTPUT_REV <= 0) { fail(1); return; }
  motorOff(); signalIndex = 0; faultCode = 0;
  amplitudeScale = clip(USE_POT ? analogRead(POT) / 1023.0f : FIXED_SCALE, 0, 1);
  noiseBucket = 0xFFFFFFFFUL;
  randomSeed(1); // Misma secuencia pseudoaleatoria para comparar ensayos.
  lastCount = readCount(); measuredRPM = 0;
  lastControl = phaseStart = now;
  state = RUN;
}

void sampleCurrent(uint32_t now) {
  if (now - lastSample < 1) return;
  lastSample = now;
  const float amps = (analogRead(CURRENT) - zeroADC) * ADC_VCC / 1023.0f / ACS_V_PER_A;
  currentSum += amps; currentAbsSum += fabsf(amps);
  if (++currentSamples >= 8) {
    currentA = currentSum / currentSamples;
    currentAbsA = currentAbsSum / currentSamples;
    currentSum = currentAbsSum = 0; currentSamples = 0;
  }
  if (state != RUN) { overActive = false; return; }
  if (currentAbsA > CURRENT_TRIP_A) {
    if (!overActive) { overSince = now; overActive = true; }
    if (now - overSince >= OVERCURRENT_MS) fail(2);
  } else overActive = false;
}

void updateSequence(uint32_t now) {
  if (state == GAP && now - phaseStart >= GAP_MS) {
    phaseStart = now; state = RUN;
  }
  if (state != RUN || now - phaseStart < SIGNAL_MS) return;
  // Con GAP_MS=0 la transicion conserva el PI y no impone una parada.
  if (++signalIndex >= 9) { motorOff(); state = DONE; return; }
  phaseStart = now;
  if (GAP_MS > 0) { motorOff(); state = GAP; }
}
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void control(uint32_t now) {
  if (now - lastControl < CONTROL_MS) return;
  const float dt = (now - lastControl) * 0.001f;
  lastControl = now;

  // 1. Leer potenciómetro en tiempo real
  if (USE_POT) {
    amplitudeScale = clip(analogRead(POT) / 1023.0f, 0.05f, 1.0f);
  } else {
    amplitudeScale = FIXED_SCALE;
  }

  // 2. Calcular RPM medidas desde el encoder
  const int32_t count = readCount();
  const int32_t delta = count - lastCount;
  lastCount = count;
  
  const float rawRPM = COUNTS_PER_OUTPUT_REV > 0 ?
    ENCODER_SIGN * delta * 60.0f / (COUNTS_PER_OUTPUT_REV * dt) : 0;
  
  // Ajustado el filtro para reducir el retraso en la lectura (antes 0.4f)
  measuredRPM += 0.75f * (rawRPM - measuredRPM);
  
  if (state != RUN) return;
  
  // 3. Obtener la velocidad deseada de la gráfica actual
  referenceRPM = clip(signalRPM(signalIndex, now - phaseStart) * amplitudeScale, 0, RPM_LIMIT);
  
  if (referenceRPM < 1.0f) { 
    motorOff(); 
    return; 
  }
  
  // 4. CONTROLADOR PI SIN PREALIMENTACIÓN FIJA
  const float error = referenceRPM - measuredRPM;
  
  // Acumular la integral de forma limpia
  integral += KI * error * dt;
  integral = clip(integral, -PWM_MAX, PWM_MAX);
  
  // Calcular el PWM total combinando Proporcional + Integral
  float trial = (KP * error) + integral;
  
  // Forzar un mínimo de PWM para que venza la fricción del reductor
  if (trial > 0 && trial < 35) trial = 35; 
  
  // Acotar al rango permitido de PWM (0 a PWM_MAX)
  appliedPWM = static_cast<int>(clip(trial, 0, PWM_MAX));
  
  // 5. Aplicar potencia física al puente H L298N
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, appliedPWM);
}

// Función encargada de imprimir los datos al Monitor / Plotter Serie
void plot(uint32_t now) {
  if (now - lastPlot >= PLOT_MS) {
    lastPlot = now;
    Serial.print(">Ref_RPM:"); Serial.println(referenceRPM);
    Serial.print(">Med_RPM:"); Serial.println(measuredRPM);
    Serial.print(">PWM_Enviado:"); Serial.println(appliedPWM);
    Serial.print(">Corriente_A:"); Serial.println(currentAbsA);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(START, INPUT_PULLUP);
  pinMode(STOP, INPUT_PULLUP);
  
  // Interrupción del Encoder en flanco de subida (RISING)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  
  // Calibración inicial del sensor de corriente en vacío
  long adcSum = 0;
  for(int i=0; i<100; i++) { adcSum += analogRead(CURRENT); delay(2); }
  zeroADC = adcSum / 100.0f;
}

void loop() {
  uint32_t now = millis();
  
  // Máquina de estados y lectura de botones
  if (digitalRead(START) == LOW && state == IDLE) {
    startSequence(now);
    delay(200); // Anti-rebote
  }
  if (digitalRead(STOP) == LOW && state != IDLE) {
    motorOff();
    state = IDLE;
    delay(200);
  }
  
  // Hilos de ejecución por tiempo
  sampleCurrent(now);
  updateSequence(now);
  control(now);
  plot(now); 
}
