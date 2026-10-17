
//  AERSP 304 - Assignment 1: DC Motor System Identification
//  Motor: RK-370SD-2470 DC 6V with quadrature encoder
//  Hardware: Arduino Mega 2560 + L298N H-Bridge
//  Wiring:
//    L298N ENA        -> Pin 9  (PWM)
//    L298N IN1        -> Pin 7
//    L298N IN2        -> Pin 8
//    L298N GND        -> Arduino GND (shared ground!)
//    L298N 12V        -> 12V supply
//    Encoder A (Yellow) -> Pin 2  (interrupt 0)
//    Encoder B (White)  -> Pin 3  (interrupt 1)
//    Encoder VCC (Blue) -> 5V
//    Encoder GND (Green)-> GND


// Pin Definitions 
#define ENA_PIN   9
#define IN1_PIN   7
#define IN2_PIN   8
#define ENC_A_PIN 2
#define ENC_B_PIN 3

// Encoder / Motor Constants 
const float COUNTS_PER_REV = 960;

//  Motor Supply Voltage
const float SUPPLY_VOLTAGE = 12.0;

// Timing 
const unsigned long SAMPLE_MS          = 100;
const unsigned long RECORD_DURATION_MS = 12000;
const unsigned long SETTLE_DELAY_MS    = 3000;

//  PWM / Voltage Levels 
const int NUM_TRIALS = 5;
const int VOLTAGE_LEVELS[NUM_TRIALS] = {204, 178, 153, 128, 102};

// Chirp Parameters 
 #define RUN_CHIRP
const float CHIRP_AMPLITUDE_PWM = 153.0;
const float CHIRP_F0 = 0.1;
const float CHIRP_F1 = 2.0;
const float CHIRP_T  = 20.0;

//  Global Encoder State 
volatile long encoderCount = 0;


void encoderISR_A() {
  encoderCount++;
}

void encoderISR_B() {
  encoderCount++;
}


//  HELPER FUNCTIONS
void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, pwmValue);
}

void stopMotor() {
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

float computeRPM(long deltaCounts, unsigned long deltaTimeMs) {
  if (deltaTimeMs == 0) return 0.0;
  float revolutions = (float)deltaCounts / COUNTS_PER_REV;
  float minutes     = (float)deltaTimeMs / 60000.0;
  return revolutions / minutes;
}

float pwmToVoltage(int pwm) {
  return (pwm / 255.0) * SUPPLY_VOLTAGE;
}


//  RUN ONE STEP TRIAL
void runStepTrial(int pwmValue, int trialNum) {
  Serial.print("# TRIAL ");
  Serial.print(trialNum);
  Serial.print("  PWM=");
  Serial.print(pwmValue);
  Serial.print("  Voltage=");
  Serial.print(pwmToVoltage(pwmValue), 3);
  Serial.println("V");
  Serial.println("# time_s,voltage_V,rpm");

  noInterrupts();
  encoderCount = 0;
  interrupts();

  long          lastCount  = 0;
  unsigned long trialStart = millis();
  unsigned long lastSample = trialStart;

  setMotorPWM(pwmValue);

  while (millis() - trialStart < RECORD_DURATION_MS) {
    unsigned long now = millis();
    if (now - lastSample >= SAMPLE_MS) {
      unsigned long dt = now - lastSample;
      lastSample = now;

      noInterrupts();
      long currentCount = encoderCount;
      interrupts();

      long delta    = currentCount - lastCount;
      lastCount     = currentCount;

      float rpm     = computeRPM(abs(delta), dt);
      float timeSec = (now - trialStart) / 1000.0;
      float voltage = pwmToVoltage(pwmValue);

      Serial.print(timeSec, 3);
      Serial.print(",");
      Serial.print(voltage, 3);
      Serial.print(",");
      Serial.println(rpm, 2);
    }
  }

  stopMotor();
  Serial.print("# Trial ");
  Serial.print(trialNum);
  Serial.println(" complete. Cooling...");
  delay(SETTLE_DELAY_MS);
}


//  RUN CHIRP TRIAL
void runChirpTrial() {
  Serial.println("# CHIRP MODE");
  Serial.print("# f0="); Serial.print(CHIRP_F0);
  Serial.print("Hz  f1="); Serial.print(CHIRP_F1);
  Serial.print("Hz  T="); Serial.print(CHIRP_T);
  Serial.println("s");
  Serial.print("# Amplitude PWM="); Serial.print(CHIRP_AMPLITUDE_PWM);
  Serial.print("  (~"); Serial.print((CHIRP_AMPLITUDE_PWM/255.0)*SUPPLY_VOLTAGE, 2);
  Serial.println("V)");
  Serial.println("# time_s,voltage_V,rpm");

  noInterrupts();
  encoderCount = 0;
  interrupts();

  long          lastCount  = 0;
  unsigned long start      = millis();
  unsigned long lastSample = start;

  float c = (CHIRP_F1 - CHIRP_F0) / CHIRP_T;

  while (true) {
    unsigned long now = millis();
    float t = (now - start) / 1000.0;
    if (t > CHIRP_T) break;

    float phase  = 2.0 * PI * (0.5 * c * t * t + CHIRP_F0 * t);
    float sinVal = sin(phase);
    int pwm = (int)(CHIRP_AMPLITUDE_PWM * 0.5 + CHIRP_AMPLITUDE_PWM * 0.5 * sinVal);
    pwm = constrain(pwm, 0, 255);
    setMotorPWM(pwm);

    if (now - lastSample >= SAMPLE_MS) {
      unsigned long dt = now - lastSample;
      lastSample = now;

      noInterrupts();
      long currentCount = encoderCount;
      interrupts();

      long delta = currentCount - lastCount;
      lastCount  = currentCount;

      float rpm     = computeRPM(abs(delta), dt);
      float voltage = pwmToVoltage(pwm);

      Serial.print(t, 3);
      Serial.print(",");
      Serial.print(voltage, 3);
      Serial.print(",");
      Serial.println(rpm, 2);
    }
  }

  stopMotor();
  Serial.println("# Chirp complete.");
}


//  SETUP
void setup() {
  Serial.begin(115200);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  stopMotor();

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR_B, CHANGE);

  Serial.println("# ==========================================");
  Serial.println("# AERSP 304 - Assignment 1");
  Serial.println("# Motor: RK-370SD-2470  Gear ratio: 9.68:1");
  Serial.println("# CPR: 240  ISR: simplified (speed only)");
  Serial.print  ("# Sample interval: "); Serial.print(SAMPLE_MS); Serial.println("ms");
  Serial.println("# Output: time_s, voltage_V, rpm  (CSV)");
  Serial.println("# ==========================================");
  Serial.println("# Starting in 3s...");
  delay(3000);
}


//  LOOP
void loop() {

#ifdef RUN_CHIRP
  runChirpTrial();
  Serial.println("# Done. Reset Arduino to run again.");
  while (true);

#else
  for (int i = 0; i < NUM_TRIALS; i++) {
    runStepTrial(VOLTAGE_LEVELS[i], i + 1);
  }
  Serial.println("# ==========================================");
  Serial.println("# ALL TRIALS COMPLETE");
  Serial.println("# Copy serial output to .csv and open in MATLAB");
  Serial.println("# ==========================================");
  while (true);
#endif
}