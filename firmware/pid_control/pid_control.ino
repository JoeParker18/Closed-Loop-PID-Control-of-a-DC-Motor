#define ENA   9
#define IN1   7
#define IN2   8
#define ENC_A 2
#define ENC_B 3
  
const int Count_Per_Rev = 960;
volatile long pulseCount = 0;
unsigned long prevTime = 0;

const float SUPPLY_VOLTAGE = 12.0;

// Timing 
const unsigned long SAMPLE_MS = 100;
const float Ts = 0.1;

float Kp = 2;
float Ki = 15;
float Kd = 0.0;

const float targetRPM = 240;
 
float err_over_time = 0.0;
float prev_err = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), countPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), countPulse, CHANGE);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  prevTime = millis();
}

void countPulse() {
  pulseCount++;
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= SAMPLE_MS) {
    float rpm = (float)pulseCount / Count_Per_Rev * 60000.0 / (currentTime - prevTime);

    float error = targetRPM - rpm;
    err_over_time += error * Ts;
    float deriv = (error - prev_err) / Ts;
    float output = Kp * error + Ki * err_over_time + Kd * deriv;

    float pwmOutput = output * (255.0 / SUPPLY_VOLTAGE);

    prev_err = error;
    int pwmVal = constrain((int)pwmOutput, 0, 255);
    analogWrite(ENA, pwmVal);

    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(targetRPM);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.println(pwmVal);

    pulseCount = 0;
    prevTime = currentTime;
  }
}