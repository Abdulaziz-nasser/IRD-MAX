#include <Wire.h>
#include "DFRobot_BNO055.h"
#include <Servo.h>

/* ================= Pins ================= */
#define BUZZER_PIN A0
#define START_BTN  A1
#define START_ACTIVE_LOW 1    // 1 = button pulls pin LOW when pressed (with INPUT_PULLUP); 0 = active HIGH

// BTS7960 pins
#define RPWM 5
#define LPWM 6
#define R_EN 7
#define L_EN 8

// Servo
#define SERVO_PIN 9

// Ultrasonic (single-pin trig+echo)
#define US_R_PIN  2
#define US_L_PIN  3
#define US_F_PIN  4

/* ===== Buzzer Master Switch ===== */
#define BUZZ_ENABLE 1   // 0=OFF, 1=ON

/* ===== Turn timeout (default; can be set from Jetson) ===== */
static unsigned long TURN_TIMEOUT_MS = 1200;   // <- Jetson can change via: SET_TURN_TIMEOUT,<ms>

/* ============ Servo geometry (LIVE-TRIMMABLE) ============ */
int   SERVO_CENTER = 90;   // can be changed via SET_CENTER,<deg>
#define SERVO_MIN    65
#define SERVO_MAX    112  // LEFT 
float SPAN_LEFT  = 90 - 60;
float SPAN_RIGHT = 110 - 90;
float STEER_TRIM_NORM = 0.0f;  // + = steer RIGHT

static void syncCenterSpans(){
  SPAN_LEFT  = (float)(SERVO_CENTER - SERVO_MIN);
  SPAN_RIGHT = (float)(SERVO_MAX    - SERVO_CENTER);
}

/* ===== Ultrasonic speed (temp-corrected) ===== */
#define VELOCITY_TEMP(tempC) ((331.5 + 0.6 * (float)(tempC)) * 100 / 1000000.0)  // cm/us
int16_t us_tempC = 20;

/* ================= IMU ================= */
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);

/* ---- IMU watchdog / recovery ---- */
unsigned long lastImuOkMs = 0;
int imuBadBurst = 0;
const unsigned long IMU_STALE_REINIT_MS = 400;
const int IMU_BAD_LIMIT = 3;

/* =============== State =============== */
enum State { IDLE=0, CENTERING=1, TURNING=2 };
State state = IDLE;
bool armed = false;

/* ===== Controls & smoothing ===== */
Servo myservo;
float current_steer_norm = 0.0f;
int   current_speed      = 0;
int   current_dir        = 0;

float steer_f = 0.0f;
float speed_f = 0.0f;

const float ALPHA_STEER = 0.18f;
const float ALPHA_SPEED = 0.35f;

/* ============ Yaw / Turn control ============ */
static float wrap180(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }
static float shortestErr(float target, float now){ return wrap180(target - now); }
float yaw_deg = 0.0f;

/* PID for turning */
float Kp = 3.0f, Ki = 0.0f, Kd = 0.26f;
float pid_i = 0, prev_err = 0, d_filt = 0;
const float D_ALPHA = 0.35f;

unsigned long lastPIDms = 0;
const float TURN_DONE_TOL_DEG = 2.0f;
const uint16_t TURN_HOLD_MS = 100;
float target_yaw_deg = 0.0f;
bool turnDoneLatched = false;
unsigned long turnWithinTolStart = 0;
unsigned long turnStartMs = 0;              // when TURNING started

/* ======= Speed clamps ======= */
const int SPEED_MIN_CLAMP = 0;
const int SPEED_MAX_CLAMP = 255;

/* ======= Turn speed (default + per-turn override) ======= */
int TURN_PWM_DEFAULT = 120;
int turn_pwm_override = -1;

/* ============ Buzzer helpers ============ */
inline void beep(uint16_t ms=80){
#if BUZZ_ENABLE
  tone(BUZZER_PIN, 3000, ms);
#endif
}
bool buzzerRapid = false;
unsigned long buzzLastMs = 0;
const uint16_t BUZZ_PERIOD_MS = 80;
const uint16_t BUZZ_PULSE_MS  = 40;
void chirpRapidTick(){
#if BUZZ_ENABLE
  if(!buzzerRapid) return;
  unsigned long now = millis();
  if(now - buzzLastMs >= BUZZ_PERIOD_MS){
    buzzLastMs = now;
    tone(BUZZER_PIN, 3500, BUZZ_PULSE_MS);
  }
#endif
}

/* ============ Motor / Servo helpers (BTS7960) ============ */
void bts_init(){
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void setDrive(int pwm, int dir){
  pwm = constrain(pwm, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
  if(dir > 0){
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  }else if(dir < 0){
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  }else{
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}
void stopMotor(){
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

/* ---------- Servo APIs ---------- */
void steerTo(int deg){
  deg = constrain(deg, SERVO_MIN, SERVO_MAX);
  myservo.write(deg);
}

void setSteerNorm(float u_in){
  float u = u_in + STEER_TRIM_NORM;
  if (fabs(u) < 0.02f) u = 0.0f;
  u = constrain(u, -1.0f, 1.0f);
  float angle = (u >= 0.0f) ? (SERVO_CENTER + u*SPAN_RIGHT) : (SERVO_CENTER + u*SPAN_LEFT);
  angle = constrain(angle, (float)SERVO_MIN, (float)SERVO_MAX);
  myservo.write((int)angle);
}

/* ============ IMU ============ */
bool initIMU(){
  bno.setOprMode(BNO::eOprModeConfig);
  delay(25);
  if(bno.begin() != BNO::eStatusOK) return false;
  bno.setOprMode(BNO::eOprModeNdof);
  delay(30);
  lastImuOkMs = millis();
  imuBadBurst = 0;
  return true;
}

bool reinitIMU(){
  bno.setOprMode(BNO::eOprModeConfig);
  delay(25);
  bool ok = (bno.begin() == BNO::eStatusOK);
  if(ok){
    bno.setOprMode(BNO::eOprModeNdof);
    delay(30);
    lastImuOkMs = millis();
    imuBadBurst = 0;
    Serial.println("INFO,IMU_REINIT_OK");
  }else{
    Serial.println("WARN,IMU_REINIT_FAIL");
  }
  return ok;
}

void readYaw(){
  BNO::sEulAnalog_t e = bno.getEul();
  unsigned long now = millis();

  bool ok = (bno.lastOperateStatus == BNO::eStatusOK);
  bool looksZeroFrame = (fabs(e.head) < 0.001f && fabs(e.roll) < 0.001f && fabs(e.pitch) < 0.001f);

  if(ok && !looksZeroFrame){
    float y = wrap180(e.head);
    if(!isnan(y)){
      yaw_deg = y;
      lastImuOkMs = now;
      imuBadBurst = 0;
    }else{
      imuBadBurst++;
    }
  }else{
    imuBadBurst++;
  }

  if(imuBadBurst >= IMU_BAD_LIMIT || (now - lastImuOkMs) > IMU_STALE_REINIT_MS){
    reinitIMU();
  }
}

/* ============ Ultrasonic ============ */
#define US_MAX_RANGE_CM 300
long distF = -1, distL = -1, distR = -1;

static inline unsigned long usTimeoutFromRangeCm(int cm){
  float v = VELOCITY_TEMP(us_tempC);
  return (unsigned long)((2.0f * cm) / v * 1.2f);
}

inline long usPingOnePin(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  unsigned long timeout = usTimeoutFromRangeCm(US_MAX_RANGE_CM);
  unsigned long pw = pulseIn(pin, HIGH, timeout);
  if(pw == 0) return -1;
  long cm = (long)(pw * VELOCITY_TEMP(us_tempC) / 2.0f);
  if (cm > US_MAX_RANGE_CM) cm = US_MAX_RANGE_CM;
  return cm;
}

inline void readUSAllFast(){
  distF = usPingOnePin(US_F_PIN);
  distL = usPingOnePin(US_L_PIN);
  distR = usPingOnePin(US_R_PIN);
}

/* ============ Start button helpers ============ */
static inline bool startPressedRaw(){
  int v = digitalRead(START_BTN);
  return START_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}
static bool startPressedDebounced(uint16_t ms=20){
  if(!startPressedRaw()) return false;
  delay(ms);
  return startPressedRaw();
}

/* ===== Helpers to finish a turn ===== */
// === NO-BRAKE ===
// Instead of stopping and going IDLE, immediately transition to CENTERING
// while continuing to drive forward (dir=+1) using current_speed.
static void finishTurnAndCenter(){
  // center wheels as we hand back to centering, but DO NOT stop the motor
  setSteerNorm(0.0f);
  if(current_speed <= 0){
    // if no prior speed cached, fall back to TURN default to avoid a blip
    current_speed = (turn_pwm_override >= 0) ? turn_pwm_override : TURN_PWM_DEFAULT;
  }
  current_dir = +1;
  setDrive(current_speed, current_dir);
  state = CENTERING;

  pid_i = 0; prev_err = 0; d_filt = 0; lastPIDms = 0; turnDoneLatched = false;
  turn_pwm_override = -1;
#if BUZZ_ENABLE
  buzzerRapid = false; noTone(BUZZER_PIN);
#endif
  beep(40);
}
// === /NO-BRAKE ===

/* ============ Turn controller ============ */
void runTurnController(){
  int turn_pwm = (turn_pwm_override >= 0) ? turn_pwm_override : TURN_PWM_DEFAULT;
  turn_pwm = constrain(turn_pwm, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);

  float err = shortestErr(target_yaw_deg, yaw_deg);
  if (fabs(err) < 1.0f) err = 0.0f;

  unsigned long now = millis();
  float dt = (lastPIDms==0)? 0.02f : (now - lastPIDms)/1000.0f;
  lastPIDms = now;

  pid_i += err * dt;

  float d = (err - prev_err) / (dt > 1e-3f ? dt : 1e-3f);
  d_filt = (D_ALPHA * d) + (1.0f - D_ALPHA) * d_filt;
  prev_err = err;

  float u_deg = Kp*err + Ki*pid_i + Kd*d_filt;
  float u_norm = constrain(u_deg/45.0f, -1.2f, 1.2f);
  setSteerNorm(-u_norm);
  setDrive(turn_pwm, +1);

#if BUZZ_ENABLE
  buzzerRapid = true;
#endif

  // Normal completion by angular tolerance and hold
  if(fabs(err) <= TURN_DONE_TOL_DEG){
    if(!turnDoneLatched){ turnWithinTolStart = now; turnDoneLatched = true; }
    else if(now - turnWithinTolStart >= TURN_HOLD_MS){
      // === NO-BRAKE ===
      finishTurnAndCenter();
      return;
      // === /NO-BRAKE ===
    }
  }else{
    turnDoneLatched = false;
  }

  // Timeout safeguard
  if(now - turnStartMs >= TURN_TIMEOUT_MS){
    Serial.println("INFO,TURN_FINISH,TIMEOUT");
    // === NO-BRAKE ===
    finishTurnAndCenter();
    return;
    // === /NO-BRAKE ===
  }
}

/* ============ Serial protocol ============
 * START
 * CENTER,<norm>,<speed>
 * BACK,<speed>
 * TURN_ABS,<target_yaw>[,<turn_pwm>]
 * TRIM_NORM,<value>
 * SET_CENTER,<deg>
 * SET_TURN_PWM,<pwm>
 * CLEAR_TURN_PWM
 * SET_TURN_TIMEOUT,<ms>
 * STOP
 * PING  -> respond with "PONG"
 */
String rx;

void setArmed(bool on){
  if(on && !armed){
    armed = true;
    beep(100);
    Serial.println("ARMED");
  }else if(!on && armed){
    armed = false;
  }
}

void handleLine(const String &line){
  if(line.equalsIgnoreCase("PING")){
    Serial.println("PONG");
    return;
  }

  if(line.equalsIgnoreCase("START") || line.equalsIgnoreCase("S")){
    setArmed(true);
    return;
  }

  if(!armed) return;

  if(line.startsWith("CENTER")){
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1+1);
    if(c1>0 && c2>c1){
      float u = line.substring(c1+1, c2).toFloat();
      int spd = line.substring(c2+1).toInt();
      current_steer_norm = constrain(u, -1.0f, 1.0f);
      current_speed = constrain(spd, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
      current_dir = (current_speed > 0) ? +1 : 0;
      if(state != TURNING) state = CENTERING;
#if BUZZ_ENABLE
      buzzerRapid = false; noTone(BUZZER_PIN);
#endif
    }
  }else if(line.startsWith("BACK")){
    int c1 = line.indexOf(',');
    int spd = 0;
    if(c1>0){
      spd = line.substring(c1+1).toInt();
      spd = constrain(spd, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
    }
    current_speed = spd;
    current_dir   = (spd > 0) ? -1 : 0;
    if(state != TURNING) state = CENTERING;
#if BUZZ_ENABLE
    buzzerRapid = false; noTone(BUZZER_PIN);
#endif
  }else if(line.startsWith("TURN_ABS")){
    int c1 = line.indexOf(',');
    if(c1>0){
      int c2 = line.indexOf(',', c1+1);
      target_yaw_deg = wrap180(line.substring(c1+1, (c2>0? c2 : line.length())).toFloat());
      if(c2>0){
        turn_pwm_override = line.substring(c2+1).toInt();
      }else{
        turn_pwm_override = -1;
      }

      // === NO-BRAKE ===
      // Preload current_speed/dir with the intended turn speed so we keep rolling
      // after the controller exits TURNING.
      int turn_pwm = (turn_pwm_override >= 0) ? turn_pwm_override : TURN_PWM_DEFAULT;
      current_speed = constrain(turn_pwm, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
      current_dir   = +1;
      // === /NO-BRAKE ===

      state = TURNING;
      pid_i = 0; prev_err = 0; d_filt = 0; lastPIDms = 0; turnDoneLatched=false;
      turnStartMs = millis();
#if BUZZ_ENABLE
      buzzerRapid = true; buzzLastMs = 0;
      beep(120);
#endif
    }
  }else if(line.startsWith("TRIM_NORM")){
    int c1 = line.indexOf(',');
    if(c1>0){
      STEER_TRIM_NORM = line.substring(c1+1).toFloat();
      STEER_TRIM_NORM = constrain(STEER_TRIM_NORM, -0.3f, 0.3f);
      beep(40);
    }
  }else if(line.startsWith("SET_CENTER")){
    int c1 = line.indexOf(',');
    if(c1>0){
      SERVO_CENTER = constrain(line.substring(c1+1).toInt(), SERVO_MIN, SERVO_MAX);
      syncCenterSpans();
      steerTo(SERVO_CENTER);
      beep(80);
    }
  }else if(line.startsWith("SET_TURN_PWM")){
    int c1 = line.indexOf(',');
    if(c1>0){
      TURN_PWM_DEFAULT = constrain(line.substring(c1+1).toInt(), SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
      beep(60);
    }
  }else if(line.startsWith("CLEAR_TURN_PWM")){
    turn_pwm_override = -1; beep(40);
  }else if(line.startsWith("SET_TURN_TIMEOUT")){
    int c1 = line.indexOf(',');
    if(c1>0){
      unsigned long v = (unsigned long)line.substring(c1+1).toInt();
      if(v < 200) v = 200;
      if(v > 5000) v = 5000;
      TURN_TIMEOUT_MS = v;
      beep(40);
    }
  }else if(line.startsWith("STOP")){
    state = IDLE;
    current_speed = 0;
    current_dir = 0;
    setDrive(0,0);
    setSteerNorm(0.0f);
#if BUZZ_ENABLE
    buzzerRapid = false; noTone(BUZZER_PIN);
#endif
  }
}

/* ============ Telemetry (~40 Hz) ============ */
void sendTelemetry(){
  static unsigned long last = 0;
  unsigned long now = millis();
  if(now - last >= 25){
    last = now;
    readUSAllFast();
    Serial.print("TLM,yaw=");   Serial.print(yaw_deg, 1);
    Serial.print(",state=");    Serial.print((int)state);
    Serial.print(",steer=");    Serial.print(current_steer_norm, 2);
    Serial.print(",speed=");    Serial.print(current_speed);
    Serial.print(",dF=");       Serial.print(distF);
    Serial.print(",dL=");       Serial.print(distL);
    Serial.print(",dR=");       Serial.println(distR);
  }
}

/* ============ Start latch (button OR serial) ============ */
void waitForStart(){
  stopMotor();
  steerTo(SERVO_CENTER);
  String localRx = "";
  while(!armed){
    if(startPressedDebounced()){
      setArmed(true);                         // prints "ARMED"
      while(startPressedRaw()) { delay(5); }  // wait for release
      break;
    }
    while(Serial.available()){
      char ch = Serial.read();
      if(ch=='\n' || ch=='\r'){
        if(localRx.equalsIgnoreCase("S") || localRx.equalsIgnoreCase("START")){
          setArmed(true);
          localRx = "";
          break;
        }
        localRx = "";
      } else {
        if(localRx.length() < 32) localRx += ch;
      }
    }
    delay(2);
  }
}

/* ================= Setup / Loop ================= */
void setup(){
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);

  bts_init();
  stopMotor();

  myservo.attach(SERVO_PIN);
  syncCenterSpans();
  steerTo(SERVO_CENTER);

  Wire.begin();
  Wire.setClock(100000);
  initIMU();

  Serial.println("READY");
  digitalWrite(A0,1);
  delay(1000);
  digitalWrite(A0,0);

  state = IDLE;
}

void loop(){
  chirpRapidTick();

  // Read serial lines
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){
      if(rx.length()>0){ handleLine(rx); rx=""; }
    } else {
      rx += ch; if(rx.length()>128) rx.remove(0);
    }
  }

  // Allow button to arm anytime (e.g., after STOP)
  if(!armed && startPressedDebounced()){
    setArmed(true);
    while(startPressedRaw()) { delay(5); }  // wait for release
  }

  readYaw();

  // If we're turning but IMU hasn't produced a good update recently, stop safely.
  if(state == TURNING && (millis() - lastImuOkMs) > IMU_STALE_REINIT_MS){
    stopMotor();
    steerTo(SERVO_CENTER);
    state = IDLE;
#if BUZZ_ENABLE
    buzzerRapid = false; noTone(BUZZER_PIN);
#endif
    Serial.println("WARN,IMU_STALE_STOP");
  }

  if (state != TURNING) {
#if BUZZ_ENABLE
    buzzerRapid = false; noTone(BUZZER_PIN);
#endif
  }

  switch(state){
    case IDLE:
      setDrive(0,0);
      setSteerNorm(0.0f);
      break;

    case CENTERING: {
      steer_f += ALPHA_STEER * (current_steer_norm - steer_f);
      speed_f += ALPHA_SPEED * ((float)current_speed - speed_f);
      setSteerNorm(steer_f);
      setDrive((int)speed_f, current_dir);
      break;
    }

    case TURNING:
      runTurnController();
      break;
  }

  sendTelemetry();
}
