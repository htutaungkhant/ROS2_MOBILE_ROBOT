// ==================== UART pins to Raspberry Pi ====================
// Pi side:  ser = serial.Serial("/dev/ttyAMA0", 115200)
// Pi TXD (GPIO14) -----> ESP32 UART_RX_PIN
// Pi RXD (GPIO15) <----- ESP32 UART_TX_PIN
// Pi GND           <---> ESP32 GND   (MUST be shared)
//
// ⚠️ Pins 0,1,3,4,5,6,7,10 are already used by motors/encoders below,
//    so pick two DIFFERENT free GPIOs on your board for these two.
//    (e.g. ESP32-C3 SuperMini -> 20/21, classic ESP32 dev board -> 16/17)
#define UART_RX_PIN 20   // ESP32 pin that receives from Pi TXD
#define UART_TX_PIN 21   // ESP32 pin that transmits to Pi RXD

#define IN1 1   // Left  motor - forward PWM
#define IN2 0   // Left  motor - reverse PWM
#define IN3 3   // Right motor - forward PWM
#define IN4 4   // Right motor - reverse PWM

// Encoder pins
#define ENC_L_A 5
#define ENC_L_B 6
#define ENC_R_A 7
#define ENC_R_B 10

volatile long countL = 0;
volatile long countR = 0;
const float CPR = 8344.0;  // GA12-N20, 7PPR x4 quadrature x 298:1 gear ratio (50RPM @6V)
                            // ⚠️ actual test လုပ်ပြီး confirm ဖြစ်ရင် ဒီ value ကို ပြင်ပါ

// ==================== Odometry parameters ====================
const float WHEEL_RADIUS = 0.022;   // 44mm diameter wheel -> radius = 0.022m
const float WHEEL_BASE   = 0.134;    // metres - ⚠️ actual ruler တိုင်းပြီး ပြင်ပါ
const float WHEEL_CIRCUM = 2.0 * PI * WHEEL_RADIUS;
const float DIST_PER_TICK = WHEEL_CIRCUM / CPR;

double x = 0.0, y = 0.0, theta = 0.0;
float vx = 0.0, omega = 0.0;
float vL = 0.0, vR = 0.0;              // actual measured wheel speed (m/s)

unsigned long lastTime = 0;
float targetVL = 0, targetVR = 0;      // desired wheel speed from ROS2 (m/s)

// PID constants (m/s error -> PWM output, tuning လိုအပ်ပါလိမ့်မယ်)
float Kp = 800.0, Ki = 300.0, Kd = 5.0;
float errL_prev = 0, errR_prev = 0;
float integralL = 0, integralR = 0;

// UART to Pi (hardware Serial1) - keeps USB Serial free for debugging
HardwareSerial PiSerial(1);

void IRAM_ATTR readLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countL++;
  else countL--;
}
void IRAM_ATTR readRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countR++;
  else countR--;
  // ⚠️ Odom test လုပ်ပြီး x မတက်ဘဲ theta ချည်းတက်နေရင်
  // ဒီ ISR ထဲက ++ / -- ကို ပြောင်းပြန်လှန်ပါ
}

void setup() {
  Serial.begin(115200);                 // USB - debugging only
  PiSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // UART to Pi

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readRight, CHANGE);

  ledcAttach(IN1, 1000, 8);
  ledcAttach(IN2, 1000, 8);
  ledcAttach(IN3, 1000, 8);
  ledcAttach(IN4, 1000, 8);

  lastTime = millis();
}

void setMotorL(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { ledcWrite(IN1, pwm); ledcWrite(IN2, 0); }
  else          { ledcWrite(IN1, 0);   ledcWrite(IN2, -pwm); }
}

void setMotorR(int pwm) {
  pwm = constrain(pwm, -255, 255);
  //pwm = -pwm;   // ⭐ Right motor invert
  if (pwm >= 0) { ledcWrite(IN3, pwm); ledcWrite(IN4, 0); }
  else          { ledcWrite(IN3, 0);   ledcWrite(IN4, -pwm); }
}

void loop() {
  // ROS2 (Pi) ကနေ "vL,vR\n" ဖြင့် desired wheel speed (m/s) ရယူမယ် (ဥပမာ "0.08,-0.08")
  if (PiSerial.available()) {
    String cmd = PiSerial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      targetVL = cmd.substring(0, comma).toFloat();
      targetVR = cmd.substring(comma + 1).toFloat();
    }
  }

  if (millis() - lastTime >= 50) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    noInterrupts();
    long cL = countL;
    long cR = countR;
    countL = 0;
    countR = 0;
    interrupts();

    // ==================== ODOMETRY ====================
    float distL = cL * DIST_PER_TICK;
    float distR = cR * DIST_PER_TICK;

    vL = distL / dt;   // actual measured left wheel speed (m/s)
    vR = distR / dt;   // actual measured right wheel speed (m/s)

    float dCenter = (distL + distR) / 2.0;
    float dTheta  = (distR - distL) / WHEEL_BASE;

    vx    = (vL + vR) / 2.0;
    omega = (vR - vL) / WHEEL_BASE;

    theta += dTheta;
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;

    x += dCenter * cos(theta);
    y += dCenter * sin(theta);

    // Send odometry to Pi over UART
    PiSerial.print("ODOM,");
    PiSerial.print(x, 4); PiSerial.print(",");
    PiSerial.print(y, 4); PiSerial.print(",");
    PiSerial.print(theta, 4); PiSerial.print(",");
    PiSerial.print(vx, 4); PiSerial.print(",");
    PiSerial.println(omega, 4);

    // Optional: mirror to USB for live debugging on your laptop
    // Serial.print("ODOM,"); Serial.print(x,4); ...

    // ==================== PID Left (m/s error -> PWM) ====================
    float errL = targetVL - vL;
    integralL += errL * dt;
    integralL = constrain(integralL, -100, 100);
    float derivativeL = (errL - errL_prev) / dt;
    int outL = Kp * errL + Ki * integralL + Kd * derivativeL;
    outL = constrain(outL, -255, 255);
    errL_prev = errL;

    // ==================== PID Right ====================
    float errR = targetVR - vR;
    integralR += errR * dt;
    integralR = constrain(integralR, -100, 100);
    float derivativeR = (errR - errR_prev) / dt;
    int outR = Kp * errR + Ki * integralR + Kd * derivativeR;
    outR = constrain(outR, -255, 255);
    errR_prev = errR;

    if (targetVL == 0 && targetVR == 0) {
      integralL = 0; integralR = 0;
      outL = 0; outR = 0;
    }

    setMotorL(outL);
    setMotorR(outR);
  }
}
