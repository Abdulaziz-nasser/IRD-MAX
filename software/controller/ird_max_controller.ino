#include <Wire.h>
#include "DFRobot_BNO055.h"
#include <Servo.h>

/* ================= Pins ================= */
#define BUZZER_PIN A0
#define START_BTN  A1
#define START_ACTIVE_LOW 1

// BTS7960
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

/* ===== Buzzer ===== */
#define BUZZ_ENABLE 1

/* ===== Turn timeout (can be set by Jetson) ===== */
static unsigned long TURN_TIMEOUT_MS = 1200;

/* ============ Servo geometry ============ */
int   SERVO_CENTER = 90;
#define SERVO_MIN    60
#define SERVO_MAX    115
const int SERVO_DIR = -1;

float SPAN_LEFT  = 90 - 60;
float SPAN_RIGHT = 110 - 90;
float STEER_TRIM_NORM = 0.0f;

static void syncCenterSpans(){
  SPAN_LEFT  = (float)(SERVO_CENTER - SERVO_MIN);
  SPAN_RIGHT = (float)(SERVO_MAX    - SERVO_CENTER);
}

/* ===== Ultrasonic timing ===== */
#define VELOCITY_TEMP(tempC) ((331.5 + 0.6 * (float)(tempC)) * 100 / 1000000.0)  // cm/us
int16_t us_tempC = 20;
unsigned long US_TIMEOUT = 120000UL;  // µs
long US_FAR_CM = 300;

/* ================= IMU (BNO055) ================= */
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);

unsigned long lastImuOkMs = 0;
int imuBadBurst = 0;
const unsigned long IMU_STALE_REINIT_MS = 400;
const int IMU_BAD_LIMIT = 3;

static float wrap180(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }
static float shortestErr(float target, float now){ return wrap180(target - now); }

/* Use explicit opmode codes (portable across library versions) */
const DFRobot_BNO055::eOprMode_t MODE_IMU_NO_MAG = (DFRobot_BNO055::eOprMode_t)0x08; // IMU+
const DFRobot_BNO055::eOprMode_t MODE_NDOF       = (DFRobot_BNO055::eOprMode_t)0x0C; // NDOF

DFRobot_BNO055::eOprMode_t imuMode = MODE_NDOF;
unsigned long lastModeSwitchMs = 0;
const unsigned long MODE_SWITCH_COOLDOWN_MS = 150;

bool imuSetMode(DFRobot_BNO055::eOprMode_t m){
  unsigned long now = millis();
  if (now - lastModeSwitchMs < MODE_SWITCH_COOLDOWN_MS) return false;
  if (imuMode == m) return true;
  bno.setOprMode(DFRobot_BNO055::eOprModeConfig);
  delay(20);
  bno.setOprMode(m);
  delay(20);
  imuMode = m;
  lastModeSwitchMs = now;
  return true;
}

bool initIMU(){
  bno.setOprMode(DFRobot_BNO055::eOprModeConfig);
  delay(25);
  if(bno.begin() != DFRobot_BNO055::eStatusOK) return false;
  imuSetMode(MODE_NDOF);                 // start locked to absolute yaw
  lastImuOkMs = millis();
  imuBadBurst = 0;
  return true;
}

bool reinitIMU(){
  bno.setOprMode(DFRobot_BNO055::eOprModeConfig);
  delay(25);
  bool ok = (bno.begin() == DFRobot_BNO055::eStatusOK);
  if(ok){
    imuSetMode(MODE_NDOF);
    lastImuOkMs = millis();
    imuBadBurst = 0;
    Serial.println("INFO,IMU_REINIT_OK");
  }else{
    Serial.println("WARN,IMU_REINIT_FAIL");
  }
  return ok;
}

/* ====== Fused yaw (from BNO055 fusion Euler) ====== */
float fused_yaw_deg = 0.0f;     // what we use everywhere

void updateFusedYaw(){
  // Always read Euler; in IMU+ it's gyro/acc fusion (no mag), in NDOF it's absolute.
  DFRobot_BNO055::sEulAnalog_t e = bno.getEul();
  bool ok = (bno.lastOperateStatus == DFRobot_BNO055::eStatusOK);
  if (ok){
    float y = wrap180(e.head);
    if (!isnan(y)){
      fused_yaw_deg = y;
      lastImuOkMs = millis();
      imuBadBurst = 0;
    } else {
      imuBadBurst++;
    }
  } else {
    imuBadBurst++;
  }

  if(imuBadBurst >= IMU_BAD_LIMIT || (millis() - lastImuOkMs) > IMU_STALE_REINIT_MS){
    reinitIMU();
  }
}

/* =============== Encoder (A2/A3) =============== */
#define COUNTS_PER_10CM 624
#define INVERT_DIR false
const float CM_PER_COUNT = 10.0f / (float)COUNTS_PER_10CM;

#if defined(ARDUINO_ARCH_ESP32)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

const uint8_t PIN_ENC_A = A2;    // CLK
const uint8_t PIN_ENC_B = A3;    // DT

volatile long   encCount   = 0;  // signed 4x counts
volatile uint8_t prevState = 0;  // (A<<1)|B

const int8_t QDEC_LUT[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define ENC_PINREG   PINC
  #define ENC_A_BIT    PC2
  #define ENC_B_BIT    PC3
  static inline uint8_t readABFast() {
    uint8_t p = ENC_PINREG;
    return (((p >> ENC_A_BIT) & 1) << 1) | ((p >> ENC_B_BIT) & 1);
  }
#else
  static inline uint8_t readABFast() {
    uint8_t a = digitalRead(PIN_ENC_A);
    uint8_t b = digitalRead(PIN_ENC_B);
    return (a << 1) | b;
  }
#endif

static inline void qdecUpdate(uint8_t curr) {
  uint8_t idx  = (prevState << 2) | curr;
  int8_t  step = QDEC_LUT[idx];
  if (INVERT_DIR) step = -step;
  if (step != 0) encCount += step;
  prevState = curr;
}
void ISR_ATTR handleEncoderAttach() { qdecUpdate(readABFast()); }
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
ISR(PCINT1_vect) { qdecUpdate(readABFast()); }
#endif

static inline float countsToCm(long c){ return c * CM_PER_COUNT; }

/* =============== State =============== */
enum State { IDLE=0, CENTERING=1, TURNING=2 };
State state = IDLE;
bool armed = false;

/* ===== Controls & smoothing ===== */
Servo myservo;
float current_steer_norm = 0.0f;
int   current_speed      = 0;      // 0..255
int   current_dir        = 0;      // +1 FWD, -1 REV, 0 stop
float steer_f = 0.0f, speed_f = 0.0f;
const float ALPHA_STEER = 0.18f, ALPHA_SPEED = 0.35f;

/* ============ Turn control (uses fused_yaw_deg) ============ */
float Kp = 3.0f, Ki = 0.0f, Kd = 0.26f;
float pid_i = 0, prev_err = 0, d_filt = 0;
const float D_ALPHA = 0.35f;

unsigned long lastPIDms = 0;
const float TURN_DONE_TOL_DEG = 2.0f;
const uint16_t TURN_HOLD_MS = 100;
float target_yaw_deg = 0.0f;
unsigned long turnStartMs = 0;

const int SPEED_MIN_CLAMP = 0;
const int SPEED_MAX_CLAMP = 255;

int TURN_PWM_DEFAULT = 120;
int turn_pwm_override = -1;
int turn_drive_dir    = +1;

/* ============ Buzzer helpers ============ */
inline void beep(uint16_t ms=80){
#if BUZZ_ENABLE
  tone(BUZZER_PIN, 3000, ms);
#else
  (void)ms;
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

/* ============ Motor / Servo helpers ============ */
void bts_init(){
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
  analogWrite(RPWM, 0); analogWrite(LPWM, 0);
}
void setDrive(int pwm, int dir){
  pwm = constrain(pwm, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
  if(dir > 0){ analogWrite(RPWM, pwm); analogWrite(LPWM, 0); }
  else if(dir < 0){ analogWrite(RPWM, 0); analogWrite(LPWM, pwm); }
  else { analogWrite(RPWM, 0); analogWrite(LPWM, 0); }
}
void stopMotor(){ analogWrite(RPWM,0); analogWrite(LPWM,0); }

void setSteerNorm(float u_in){
  float u = SERVO_DIR * (u_in + STEER_TRIM_NORM);
  if (fabs(u) < 0.02f) u = 0.0f;
  u = constrain(u, -1.0f, 1.0f);
  float angle = (u >= 0.0f) ? (SERVO_CENTER + u*SPAN_RIGHT) : (SERVO_CENTER + u*SPAN_LEFT);
  angle = constrain(angle, (float)SERVO_MIN, (float)SERVO_MAX);
  myservo.write((int)angle);
}
void steerTo(int deg){ deg = constrain(deg, SERVO_MIN, SERVO_MAX); myservo.write(deg); }

/* ============ Ultrasonic ============ */
volatile long distF=-1, distL=-1, distR=-1;

inline long usPingOnePin(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  unsigned long pw = pulseIn(pin, HIGH, US_TIMEOUT);
  if(pw == 0) return -1;
  float cm = pw * VELOCITY_TEMP(us_tempC) / 2.0f;
  if(cm > US_FAR_CM) cm = US_FAR_CM;
  return (long)cm;
}
inline long readFrontMin2(){
  long a = usPingOnePin(US_F_PIN);
  long b = usPingOnePin(US_F_PIN);
  if(a<0) return b; if(b<0) return a; return (a<b)?a:b;
}
uint8_t us_phase = 0;
void readUSStaggered(){
  switch(us_phase){
    case 0: distF = readFrontMin2(); break;
    case 1: distL = usPingOnePin(US_L_PIN); break;
    case 2: distR = usPingOnePin(US_R_PIN); break;
    default: break;
  }
  us_phase = (us_phase + 1) & 3;
}

/* ===== Helpers to finish a turn ===== */
static void finishTurnAndIdle(){
  stopMotor();
  steerTo(SERVO_CENTER);
  state = IDLE;
  pid_i = 0; prev_err = 0; d_filt = 0; lastPIDms = 0;
  turn_pwm_override = -1;
  turn_drive_dir = +1;
#if BUZZ_ENABLE
  buzzerRapid = false; noTone(BUZZER_PIN);
#endif
  beep(60);
}

/* ============ Turn controller (uses fused_yaw_deg) ============ */
void runTurnController(){
  int turn_pwm = (turn_pwm_override >= 0) ? turn_pwm_override : TURN_PWM_DEFAULT;
  float err = shortestErr(target_yaw_deg, fused_yaw_deg);
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

  float steer_cmd = (turn_drive_dir >= 0) ? u_norm : -u_norm;
  setSteerNorm(steer_cmd);
  setDrive(turn_pwm, turn_drive_dir);

#if BUZZ_ENABLE
  buzzerRapid = true;
#endif

  static unsigned long tolStart=0;
  if(fabs(err) <= TURN_DONE_TOL_DEG){
    if(!tolStart) tolStart = now;
    else if(now - tolStart >= TURN_HOLD_MS){ finishTurnAndIdle(); tolStart=0; return; }
  }else tolStart=0;

  if(now - turnStartMs >= TURN_TIMEOUT_MS){
    Serial.println("INFO,TURN_FINISH,TIMEOUT");
    finishTurnAndIdle();
    return;
  }
}

/* ============ Serial protocol ============

START
CENTER,<norm>,<speed>
BACK,<speed>
BACKC,<norm>,<speed>
TURN_ABS,<target_yaw>[,<pwm>[,REV]]
STEER_DEG,<deg>
TRIM_NORM,<value>
SET_CENTER,<deg>
SET_TURN_PWM,<pwm>
CLEAR_TURN_PWM
SET_TURN_TIMEOUT,<ms>
SET_US_TIMEOUT,<micros>
SET_US_FAR_CM,<cm>
STOP
PING -> PONG

ENC_ZERO
ENC_GET
*/
String rx;

void setArmed(bool on){
  if(on && !armed){ armed = true; beep(100); Serial.println("ARMED"); }
  else if(!on && armed){ armed = false; }
}

static inline bool startPressedRaw(){
  int v = digitalRead(START_BTN);
  return START_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}
static bool startPressedDebounced(uint16_t ms=20){
  if(!startPressedRaw()) return false;
  delay(ms);
  return startPressedRaw();
}

void handleLine(const String &line){
  if(line.equalsIgnoreCase("PING")){ Serial.println("PONG"); return; }
  if(line.equalsIgnoreCase("START") || line.equalsIgnoreCase("S")){ setArmed(true); return; }

  if(!armed){
    if(line.startsWith("SET_US_TIMEOUT")){
      int c1=line.indexOf(','); if(c1>0){ US_TIMEOUT=(unsigned long)line.substring(c1+1).toInt(); beep(40); }
    } else if(line.startsWith("SET_US_FAR_CM")){
      int c1=line.indexOf(','); if(c1>0){ US_FAR_CM=line.substring(c1+1).toInt(); beep(40); }
    } else if(line.startsWith("ENC_ZERO")){
      noInterrupts(); encCount = 0; interrupts(); beep(40); Serial.println("OK,ENC_ZERO");
    } else if(line.startsWith("ENC_GET")){
      long c; noInterrupts(); c = encCount; interrupts();
      Serial.print("ENC_CM,"); Serial.println(countsToCm(c), 2);
    }
    return;
  }

  if(line.startsWith("STOP")){
    state = IDLE; current_speed = 0; current_dir = 0; setDrive(0,0); setSteerNorm(0.0f);
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    return;
  }

  if(line.startsWith("ENC_ZERO")){
    noInterrupts(); encCount = 0; interrupts(); beep(40); Serial.println("OK,ENC_ZERO"); return;
  }
  if(line.startsWith("ENC_GET")){
    long c; noInterrupts(); c = encCount; interrupts();
    Serial.print("ENC_CM,"); Serial.println(countsToCm(c), 2); return;
  }

  if(line.startsWith("CENTER")){
    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    if(c1>0 && c2>c1){
      float u=line.substring(c1+1,c2).toFloat();
      int spd=line.substring(c2+1).toInt();
      current_steer_norm = constrain(u,-1.0f,1.0f);
      current_speed = constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP);
      current_dir = (current_speed>0)? +1:0;
      if(state != TURNING) state = CENTERING;
#if BUZZ_ENABLE
      buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    }
  }else if(line.startsWith("BACKC")){
    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    if(c1>0 && c2>c1){
      float u=line.substring(c1+1,c2).toFloat();
      int spd=line.substring(c2+1).toInt();
      current_steer_norm = constrain(u,-1.0f,1.0f);
      current_speed = constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP);
      current_dir = (current_speed>0)? -1:0;
      if(state != TURNING) state = CENTERING;
#if BUZZ_ENABLE
      buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    }
  }else if(line.startsWith("BACK")){
    int c1=line.indexOf(','); int spd=0;
    if(c1>0){ spd=line.substring(c1+1).toInt(); spd=constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP); }
    current_speed = spd; current_dir = (spd>0)? -1:0;
    if(state != TURNING) state = CENTERING;
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
  }else if(line.startsWith("TURN_ABS")){
    int c1=line.indexOf(',');
    if(c1>0){
      int c2=line.indexOf(',', c1+1);
      target_yaw_deg = wrap180(line.substring(c1+1, (c2>0? c2 : line.length())).toFloat());
      turn_pwm_override = -1; turn_drive_dir = +1;
      if(c2>0){
        String tail=line.substring(c2+1); tail.trim();
        int ct=tail.indexOf(',');
        if(ct<0){
          if(tail.equalsIgnoreCase("REV")) turn_drive_dir=-1;
          else turn_pwm_override=tail.toInt();
        }else{
          String p1=tail.substring(0,ct), p2=tail.substring(ct+1);
          turn_pwm_override=p1.toInt();
          if(p2.equalsIgnoreCase("REV")) turn_drive_dir=-1;
        }
      }
      state = TURNING; pid_i=0; prev_err=0; d_filt=0; lastPIDms=0; turnStartMs=millis();
#if BUZZ_ENABLE
      buzzerRapid = true; buzzLastMs = 0; beep(120);
#endif
    }
  }else if(line.startsWith("STEER_DEG")){
    int c1=line.indexOf(','); if(c1>0){ steerTo(line.substring(c1+1).toInt()); }
  }else if(line.startsWith("TRIM_NORM")){
    int c1=line.indexOf(','); if(c1>0){ STEER_TRIM_NORM=constrain(line.substring(c1+1).toFloat(),-0.3f,0.3f); beep(40); }
  }else if(line.startsWith("SET_CENTER")){
    int c1=line.indexOf(','); if(c1>0){ SERVO_CENTER=constrain(line.substring(c1+1).toInt(),SERVO_MIN,SERVO_MAX); syncCenterSpans(); steerTo(SERVO_CENTER); beep(80); }
  }else if(line.startsWith("SET_TURN_PWM")){
    int c1=line.indexOf(','); if(c1>0){ TURN_PWM_DEFAULT=constrain(line.substring(c1+1).toInt(),SPEED_MIN_CLAMP,SPEED_MAX_CLAMP); beep(60); }
  }else if(line.startsWith("CLEAR_TURN_PWM")){
    turn_pwm_override=-1; beep(40);
  }else if(line.startsWith("SET_TURN_TIMEOUT")){
    int c1=line.indexOf(','); if(c1>0){ unsigned long v=(unsigned long)line.substring(c1+1).toInt(); v = max(200UL,min(5000UL,v)); TURN_TIMEOUT_MS=v; beep(40); }
  }else if(line.startsWith("SET_US_TIMEOUT")){
    int c1=line.indexOf(','); if(c1>0){ US_TIMEOUT=(unsigned long)line.substring(c1+1).toInt(); beep(40); }
  }else if(line.startsWith("SET_US_FAR_CM")){
    int c1=line.indexOf(','); if(c1>0){ US_FAR_CM=line.substring(c1+1).toInt(); beep(40); }
  }
}

/* ============ Telemetry (~60 Hz) ============ */
void sendTelemetry(){
  static unsigned long last = 0;
  unsigned long now = millis();
  if(now - last >= 16){
    last = now;
    Serial.print("TLM,yaw=");   Serial.print(fused_yaw_deg, 1);
    Serial.print(",state=");    Serial.print((int)state);
    Serial.print(",steer=");    Serial.print(current_steer_norm, 2);
    Serial.print(",speed=");    Serial.print(current_speed);
    Serial.print(",dF=");       Serial.print(distF);
    Serial.print(",dL=");       Serial.print(distL);
    Serial.print(",dR=");       Serial.print(distR);
    long c; noInterrupts(); c = encCount; interrupts();
    Serial.print(",enc_cm=");   Serial.println(countsToCm(c), 2);
  }
}

/* ================= Setup / Loop ================= */
unsigned long lastStationaryMs = 0;

void setup(){
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);

  bts_init(); stopMotor();
  myservo.attach(SERVO_PIN); syncCenterSpans(); steerTo(SERVO_CENTER);

  // Encoder
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  prevState = readABFast();
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);
#else
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoderAttach, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), handleEncoderAttach, CHANGE);
#endif

  Wire.begin(); Wire.setClock(100000);
  initIMU();

  Serial.println("READY");
  digitalWrite(A0,1); delay(200); digitalWrite(A0,0);
}

void loop(){
  chirpRapidTick();
  readUSStaggered();

  // Serial lines
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){ if(rx.length()>0){ handleLine(rx); rx=""; } }
    else { rx += ch; if(rx.length()>128) rx.remove(0); }
  }

  // Local button to arm
  if(!armed && startPressedDebounced()){
    setArmed(true);
    while(startPressedRaw()) { delay(5); }
  }

  // Motion detection (motor command OR encoder change)
  static long lastEnc = 0;
  long nowEnc; noInterrupts(); nowEnc = encCount; interrupts();
  bool encoder_moved = (nowEnc != lastEnc); lastEnc = nowEnc;
  bool motor_driving = ((state==CENTERING) && (current_dir!=0) && (current_speed>0)) || (state==TURNING);
  bool moving = motor_driving || encoder_moved;

  // Switch IMU modes to avoid magnetometer during motion
  if (moving){
    imuSetMode(MODE_IMU_NO_MAG);   // no magnetometer → wheels won't swing yaw
    lastStationaryMs=0;
  } else {
    if(lastStationaryMs==0) lastStationaryMs=millis();
    if(millis()-lastStationaryMs > 400) imuSetMode(MODE_NDOF);   // re-lock absolute yaw
  }

  // Update yaw (Euler from fusion)
  updateFusedYaw();

  // Watchdog
  if(state == TURNING && (millis() - lastImuOkMs) > IMU_STALE_REINIT_MS){
    stopMotor(); steerTo(SERVO_CENTER); state = IDLE;
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    Serial.println("WARN,IMU_STALE_STOP");
  }

  switch(state){
    case IDLE:
      setDrive(0,0); setSteerNorm(0.0f);
#if BUZZ_ENABLE
      buzzerRapid=false; noTone(BUZZER_PIN);
#endif
      break;

    case CENTERING:{
      steer_f += ALPHA_STEER * (current_steer_norm - steer_f);
      speed_f += ALPHA_SPEED * ((float)current_speed - speed_f);
      float u_eff = (current_dir < 0) ? -steer_f : steer_f; // flip in reverse so nose turns correctly
      setSteerNorm(u_eff);
      setDrive((int)speed_f, current_dir);
      break;
    }

    case TURNING:
      runTurnController();
      break;
  }

  sendTelemetry();
}

1