#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----- Pins -----
// Motors
int ENA = 6;   // RIGHT motor enable (PWM)
int IN1 = 7;   // RIGHT motor IN1
int IN2 = 8;   // RIGHT motor IN2

int IN3 = 4;   // LEFT motor IN3
int IN4 = 2;   // LEFT motor IN4
int ENB = 3;   // LEFT motor enable (PWM)

// Sensors
const int IR_LEFT  = 10;   // left IR sensor
const int IR_RIGHT = 9;    // right IR sensor

// LEDs + Buzzer
const int LED_LEFT  = 12;  // left LED
const int LED_RIGHT = 13;  // right LED
const int BUZZER    = 11;  // buzzer on D11

// LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // 0x27 or 0x3F usually

// ----- Motor speeds -----
const int BASE_SPEED = 90;    // normal speed
const int SLOW_SPEED = 80;    // speed after 2nd horizontal line
const int MAX_SPEED  = 125;   // cap
const int MIN_SPEED  = 80;    // minimum PWM where motors actually move
const int TURN_SPEED = 90;    // speed for the single-wheel pivot turns

// Extra push during turns (for dual-sensor PID turns only)
const int TURN_BOOST_LEFT  = 20; // more help when LEFT sensor is on line
const int TURN_BOOST_RIGHT = 15; // right turns stay gentler

// Buzzer intensity (0–255)
const int BUZZER_LEVEL = 200;

// ===== PID GAINS (TUNE THESE) =====
float Kp = 60.0;   // strong steering
float Ki = 0.0;    // keep 0 for now
float Kd = 0.0;    // D off for now

// PID state
float error      = 0;
float lastError  = 0;
float integral   = 0;
unsigned long lastTime = 0;

// Integral anti-windup limit
const float INTEGRAL_LIMIT = 50.0;

// ---- Horizontal line detection ----
int  checkpointCount = 0;   // how many times we've seen BOTH BLACK
bool lastBothBlack   = false;

// --------- HELPERS ---------

void setMotorSpeeds(int rightSpeed, int leftSpeed) {
  // constrain to [0, MAX_SPEED]
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);

  // enforce minimum speed if wheel is supposed to move
  if (rightSpeed > 0 && rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;
  if (leftSpeed  > 0 && leftSpeed  < MIN_SPEED) leftSpeed  = MIN_SPEED;

  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);

  // both wheels forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void buzzerOn() {
  analogWrite(BUZZER, BUZZER_LEVEL);
}

void buzzerOff() {
  analogWrite(BUZZER, 0);
}

void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor pins
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // LEDs + buzzer
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Line Follower");
  lcd.setCursor(0, 1);
  lcd.print("PID + Checkpts");
  delay(1000);
  lcd.clear();

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // seconds
  if (dt <= 0) dt = 0.01;
  lastTime = now;

  // Read IR sensors (LOW = black, HIGH = white)
  bool leftOnLine  = (digitalRead(IR_LEFT)  == LOW);
  bool rightOnLine = (digitalRead(IR_RIGHT) == LOW);
  bool bothBlack   = leftOnLine && rightOnLine;

  // LCD top row — sensor states
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(leftOnLine  ? "B" : "W");
  lcd.print(" R:");
  lcd.print(rightOnLine ? "B" : "W");
  lcd.print("   ");

  // LEDs show which sensor sees black
  digitalWrite(LED_LEFT,  leftOnLine  ? HIGH : LOW);
  digitalWrite(LED_RIGHT, rightOnLine ? HIGH : LOW);

  // ---- Detect NEW horizontal line (rising edge of both-black) ----
  if (bothBlack && !lastBothBlack) {
    checkpointCount++;   // we just hit a new horizontal stripe
  }
  lastBothBlack = bothBlack;

  // -------- SPECIAL CASE: BOTH BLACK → STRAIGHT FORWARD --------
  if (bothBlack) {
    // reset PID memory so it doesn't accumulate weird stuff
    error = 0;
    lastError = 0;
    integral = 0;

    int speedToUse;

    // 1st time: go forward at BASE_SPEED
    // 2nd time and later: go slower (finish / tricky zone)
    if (checkpointCount >= 2) {
      speedToUse = SLOW_SPEED;
      lcd.setCursor(0, 1);
      lcd.print("CHK ");
      lcd.print(checkpointCount);
      lcd.print(" SLOW FWD  ");
    } else {
      speedToUse = BASE_SPEED;
      lcd.setCursor(0, 1);
      lcd.print("CHK ");
      lcd.print(checkpointCount);
      lcd.print(" FAST FWD  ");
    }

    setMotorSpeeds(speedToUse, speedToUse);
    //buzzerOff();

    delay(10);
    return;   // skip rest of PID logic
  }

  // ---------- ERROR CALCULATION ----------
  if (leftOnLine && !rightOnLine) {
    error = -1;      // line on left
  } else if (rightOnLine && !leftOnLine) {
    error = 1;       // line on right
  } else {
    // BOTH WHITE: lost line → use last direction
    error = lastError;
  }

  // ---------- PID COMPUTATION (still update internal state) ----------
  float Pout = Kp * error;

  integral += error * dt;
  if (integral > INTEGRAL_LIMIT)  integral = INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  float Iout = Ki * integral;

  float derivative = (error - lastError) / dt;
  float Dout = Kd * derivative;

  float output = Pout + Iout + Dout;
  lastError = error;

  // ---------- MOTOR SPEEDS ----------
  int rightSpeed = 0;
  int leftSpeed  = 0;

  // 1) PURE PIVOT MODES: SINGLE-SENSOR CASES
  //
  // Left sensor only: pivot LEFT → only RIGHT wheel spins
  if (leftOnLine && !rightOnLine) {
    rightSpeed = TURN_SPEED;  // ≥ MIN_SPEED, will be clamped
    leftSpeed  = 0;
    lcd.setCursor(0, 1);
    lcd.print("PIVOT LEFT      ");
  }
  // Right sensor only: pivot RIGHT → only LEFT wheel spins
  else if (rightOnLine && !leftOnLine) {
    leftSpeed  = TURN_SPEED;
    rightSpeed = 0;
    lcd.setCursor(0, 1);
    lcd.print("PIVOT RIGHT     ");
  }
  // 2) NORMAL PID FOLLOW (no sensor or both white handled later)
  else {
    int base = BASE_SPEED;

    // Stronger slow-down on curves so it doesn’t fling off on sharp turns
    if (error != 0) {
      base -= 15;
      if (base < MIN_SPEED) base = MIN_SPEED;
    }

    rightSpeed = base - (int)output;
    leftSpeed  = base + (int)output;

    // Extra push for outer wheel when turning (for dual-sensor / transitional)
    if (error == -1) {
      rightSpeed += TURN_BOOST_LEFT;
      leftSpeed  -= TURN_BOOST_LEFT;
    } else if (error == 1) {
      leftSpeed  += TURN_BOOST_RIGHT;
      rightSpeed -= TURN_BOOST_RIGHT;
    }

    lcd.setCursor(0, 1);
    if (error < 0) {
      lcd.print("TURN LEFT PID   ");
    } else if (error > 0) {
      lcd.print("TURN RIGHT PID  ");
    } else {
      lcd.print("CENTER PID      ");
    }
  }

  // 3) LOST LINE OVERRIDE: both white → search with safe speeds
  if (!leftOnLine && !rightOnLine) {
    const int SEARCH_FAST = 70;  // ≥ MIN_SPEED
    const int SEARCH_SLOW = 60;  // <= MIN_SPEED, will clamp to 65

    if (lastError < 0) {
      // last seen on LEFT → keep turning left a bit
      leftSpeed  = SEARCH_SLOW;
      rightSpeed = SEARCH_FAST;
    } else if (lastError > 0) {
      // last seen on RIGHT → keep turning right a bit
      leftSpeed  = SEARCH_FAST;
      rightSpeed = SEARCH_SLOW;
    } else {
      // no idea → go straight at a safe search speed
      leftSpeed  = SEARCH_FAST;
      rightSpeed = SEARCH_FAST;
    }

    lcd.setCursor(0, 1);
    lcd.print("LOST - SEARCH   ");
  }

  setMotorSpeeds(rightSpeed, leftSpeed);

  delay(10);
}
