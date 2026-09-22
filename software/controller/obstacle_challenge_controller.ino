#include <Wire.h>
#include "DFRobot_BNO055.h"
#include <Servo.h>

/* ================= Pins ================= */
#define BUZZER_PIN A0
#define START_BTN  A1

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
#define US_B_PIN  10   // rear ultrasonic

/* ===== Buzzer Master Switch ===== */
#define BUZZ_ENABLE 1   // 0=OFF, 1=ON

/* ============ Servo geometry ============
 * Servo: center=90, left max=120, right max=65
 * Forward: 120 = LEFT, 65 = RIGHT
 * Backward: driving direction flips the steering effect (handled here)
 */
int   SERVO_CENTER = 90;   // change via SET_CENTER,<deg>
#define SERVO_MIN    60
#define SERVO_MAX    115
const int SERVO_DIR = -1;  // +1=normal, -1=reversed (keep -1 for your servo)

float SPAN_LEFT  = 90 - 65;    // updated in syncCenterSpans()
float SPAN_RIGHT = 120 - 90;
float STEER_TRIM_NORM = 0.0f;  // + = steer RIGHT

static void syncCenterSpans(){
  SPAN_LEFT  = (float)(SERVO_CENTER - SERVO_MIN);
  SPAN_RIGHT = (float)(SERVO_MAX    - SERVO_CENTER);
}

/* ===== Ultrasonic timing ===== */
#define VELOCITY_TEMP(tempC) ((331.5 + 0.6 * (float)(tempC)) * 100 / 1000000.0)  // cm/us
int16_t us_tempC = 20;
unsigned long US_TIMEOUT = 120000UL;  // µs
long US_FAR_CM = 999;

/* ================= IMU ================= */
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);
unsigned long lastImuOkMs = 0;
int imuBadBurst = 0;
const unsigned long IMU_STALE_REINIT_MS = 400;
const int IMU_BAD_LIMIT = 3;
static float wrap180(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }

/* =============== Encoder (A2/A3) =============== */
/* ---- calibration ---- */
#define COUNTS_PER_10CM 140       // <-- set to your measured calibration
#define INVERT_DIR false          // set true if forward shows as negative
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

// Quadrature LUT (prev<<2|curr -> step)
const int8_t QDEC_LUT[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// A2 -> PC2 (PCINT10), A3 -> PC3 (PCINT11) on PORTC
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
static inline long  cmToCounts(float cm){
  float sc = cm / CM_PER_COUNT;
  if (sc >= 0) return (long)(sc + 0.5f); else return (long)(sc - 0.5f);
}

/* =============== State =============== */
enum State { IDLE=0, CENTERING=1, TURNING=2, DISTMOVE=3 };
State state = IDLE;
bool armed = false;

/* ===== Controls & smoothing ===== */
Servo myservo;
float current_steer_norm = 0.0f;
int   current_speed      = 0;      // 0..255
int   current_dir        = 0;      // +1 forward, -1 reverse, 0 stop
float steer_f = 0.0f, speed_f = 0.0f;
const float ALPHA_STEER = 0.18f, ALPHA_SPEED = 0.35f;

/* ============ Yaw / Turn control ============ */
float yaw_deg = 0.0f;
float target_yaw_deg = 0.0f;
int   turn_drive_dir = +1;      // +1 forward, -1 reverse
float Kp = 3.0f, Ki = 0.0f, Kd = 0.26f;
float pid_i = 0, prev_err = 0, d_filt = 0;
const float D_ALPHA = 0.35f;
unsigned long lastPIDms = 0;
const float    TURN_DONE_TOL_DEG = 2.5f;
const uint32_t TURN_MAX_MS       = 2500;
unsigned long  turnStartMs       = 0;

/* ======= Speed clamps ======= */
const int SPEED_MIN_CLAMP = 0;
const int SPEED_MAX_CLAMP = 255;

/* ======= Turn speed default ======= */
int TURN_PWM_DEFAULT = 22;
int turn_pwm_override = -1;

/* ===== Distance move (encoder) ===== */
long dist_start_counts = 0;
long dist_target_counts = 0;
int  dist_dir = +1;     // +1 fwd, -1 back
int  dist_pwm = 26;     // default PWM if none given
bool dist_report_ready = false;

/* ====== Buzzer helpers ====== */
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

/* ============ IMU ============ */
bool initIMU(){
  bno.setOprMode(BNO::eOprModeConfig); delay(25);
  if(bno.begin() != BNO::eStatusOK) return false;
  bno.setOprMode(BNO::eOprModeNdof); delay(30);
  lastImuOkMs = millis(); imuBadBurst = 0; return true;
}
bool reinitIMU(){
  bno.setOprMode(BNO::eOprModeConfig); delay(25);
  bool ok = (bno.begin() == BNO::eStatusOK);
  if(ok){
    bno.setOprMode(BNO::eOprModeNdof); delay(30);
    lastImuOkMs=millis(); imuBadBurst=0;
    Serial.println("INFO,IMU_REINIT_OK");
  } else {
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
    } else {
      imuBadBurst++;
    }
  } else {
    imuBadBurst++;
  }
  if(imuBadBurst >= IMU_BAD_LIMIT || (now - lastImuOkMs) > IMU_STALE_REINIT_MS) reinitIMU();
}

/* ========= Ultrasonic ========= */
volatile long distF=-1, distL=-1, distR=-1, distB=-1;

inline long usPingOnePin(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  unsigned long pw = pulseIn(pin, HIGH, US_TIMEOUT);
  if(pw == 0) return US_FAR_CM;
  float cm = pw * VELOCITY_TEMP(us_tempC) / 2.0f;
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
    case 3: distB = usPingOnePin(US_B_PIN); break;
  }
  us_phase = (us_phase + 1) & 3;
}

/* ============ Turn controller ============ */
static float shortestErr(float target, float now){ return wrap180(target - now); }
void runTurnController(){
  if(state != TURNING) return;
  int turn_pwm = (turn_pwm_override >= 0) ? turn_pwm_override : TURN_PWM_DEFAULT;
  float err = shortestErr(target_yaw_deg, yaw_deg);

  if (fabs(err) <= TURN_DONE_TOL_DEG || (millis()-turnStartMs) > TURN_MAX_MS){
    stopMotor(); steerTo(SERVO_CENTER);
    state = IDLE; pid_i=0; prev_err=0; d_filt=0; lastPIDms=0; turn_pwm_override=-1; turn_drive_dir=+1;
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    Serial.println("TURN_DONE");
    beep(60);
    return;
  }

  if (fabs(err) < 1.0f) err = 0.0f;
  unsigned long now = millis();
  float dt = (lastPIDms==0)? 0.02f : (now - lastPIDms)/1000.0f;
  lastPIDms = now;

  pid_i += err * dt;
  float d = (err - prev_err) / (dt > 1e-3f ? dt : 1e-3f);
  d_filt = (D_ALPHA * d) + (1.0f - D_ALPHA) * d_filt;
  prev_err = err;

  float u = Kp*err + Ki*pid_i + Kd*d_filt;  // degrees
  float u_norm = constrain(u/45.0f, -1.2f, 1.2f);

  // Flip steering when reversing so nose still rotates toward target.
  float steer_cmd = (turn_drive_dir >= 0) ? u_norm : -u_norm;

  setSteerNorm(steer_cmd);
  setDrive(turn_pwm, turn_drive_dir);
#if BUZZ_ENABLE
  buzzerRapid = true;
#endif
}

/* ===== Distance move controller ===== */
void startDistanceMove(float cm, int pwm, int dir /* +1 fwd, -1 back */){
  if(pwm <= 0) pwm = 26;
  dist_pwm = constrain(pwm, SPEED_MIN_CLAMP, SPEED_MAX_CLAMP);
  dist_dir = (dir >= 0) ? +1 : -1;

  noInterrupts();
  dist_start_counts = encCount;
  interrupts();

  long need = cmToCounts(fabs(cm));
  if(need < 1) need = 1;
  dist_target_counts = need;
  dist_report_ready = true;

  // keep current steering; just drive straight at last commanded steer
  state = DISTMOVE;
  beep(90);
  Serial.print("DIST_BEGIN,cm=");
  Serial.println(cm, 2);
}

void runDistanceMove(){
  if(state != DISTMOVE) return;
  // Snapshot progress atomically
  noInterrupts();
  long nowCounts = encCount;
  interrupts();

  long delta = nowCounts - dist_start_counts;
  long progressed = labs(delta);
  if(progressed >= dist_target_counts){
    stopMotor();
    state = IDLE;
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    Serial.println("DIST_DONE");
    beep(70);
    return;
  }

  // keep last steer, just drive
  setSteerNorm( (current_dir<0) ? -current_steer_norm : current_steer_norm );
  setDrive(dist_pwm, dist_dir);

  // progress preview at ~10 Hz
  static unsigned long lastP = 0;
  unsigned long now = millis();
  if(now - lastP >= 100){
    lastP = now;
    float cm = countsToCm(delta);
    Serial.print("DIST,cm=");
    Serial.println(cm, 2);
  }
}

/* ============ Serial protocol ============
 * START
 * CENTER,<norm>,<speed>            (forward)
 * BACK,<speed>                     (reverse; keep last steer)
 * BACKC,<norm>,<speed>             (reverse with steering)
 * TURN_ABS,<target_yaw>[,<pwm>[,REV]]
 * STEER_DEG,<deg>                  absolute servo degree (65..120)
 * TRIM_NORM,<value>
 * SET_CENTER,<deg>
 * SET_TURN_PWM,<pwm>
 * CLEAR_TURN_PWM
 * SET_US_TIMEOUT,<micros>
 * SET_US_FAR_CM,<cm>
 * STOP
 * PING -> PONG
 *
 * --- NEW (encoder distance) ---
 * ENC_ZERO
 * ENC_GET
 * FWD_CM,<cm>[,<pwm>]
 * BACK_CM,<cm>[,<pwm>]
 */
String rx;

void setArmed(bool on){
  if(on && !armed){
    armed = true;
    beep(100);
    Serial.println("ARMED");
  } else if(!on && armed){
    armed = false;
  }
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
      noInterrupts(); encCount = 0; interrupts(); beep(40);
      Serial.println("OK,ENC_ZERO");
    } else if(line.startsWith("ENC_GET")){
      long c; noInterrupts(); c = encCount; interrupts();
      Serial.print("ENC_CM,"); Serial.println(countsToCm(c), 2);
    }
    return;
  }

  // ---- universal STOP ----
  if(line.startsWith("STOP")){
    state=IDLE; current_speed=0; current_dir=0; setDrive(0,0); setSteerNorm(0.0f);
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    return;
  }

  // ---- encoder distance commands (NEW) ----
  if(line.startsWith("ENC_ZERO")){
    noInterrupts(); encCount = 0; interrupts(); beep(40);
    Serial.println("OK,ENC_ZERO");
    return;
  }
  if(line.startsWith("ENC_GET")){
    long c; noInterrupts(); c = encCount; interrupts();
    Serial.print("ENC_CM,"); Serial.println(countsToCm(c), 2);
    return;
  }
  if(line.startsWith("FWD_CM")){
    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    float cm = 0; int pwm = 26;
    if(c1>0){
      if(c2>0){ cm = line.substring(c1+1,c2).toFloat(); pwm = line.substring(c2+1).toInt(); }
      else    { cm = line.substring(c1+1).toFloat(); }
      current_dir = +1; // remember
      startDistanceMove(fabs(cm), (c2>0? pwm:26), +1);
      return;
    }
  }
  if(line.startsWith("BACK_CM")){
    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    float cm = 0; int pwm = 26;
    if(c1>0){
      if(c2>0){ cm = line.substring(c1+1,c2).toFloat(); pwm = line.substring(c2+1).toInt(); }
      else    { cm = line.substring(c1+1).toFloat(); }
      current_dir = -1; // remember
      startDistanceMove(fabs(cm), (c2>0? pwm:26), -1);
      return;
    }
  }

  // ---- original commands (unchanged) ----
  if(line.startsWith("CENTER")){
    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    if(c1>0 && c2>c1){
      float u=line.substring(c1+1,c2).toFloat();
      int spd=line.substring(c2+1).toInt();
      current_steer_norm = constrain(u,-1.0f,1.0f);
      current_speed = constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP);
      current_dir = (current_speed>0)? +1:0;
      if(state!=TURNING) state=CENTERING;
#if BUZZ_ENABLE
      buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    }
  }
  else if(line.startsWith("BACKC")){

    int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
    if(c1>0 && c2>c1){
      float u=line.substring(c1+1,c2).toFloat();
      int spd=line.substring(c2+1).toInt();
      current_steer_norm = constrain(u,-1.0f,1.0f);
      current_speed = constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP);
      current_dir = (current_speed>0)? -1:0;
      if(state!=TURNING) state=CENTERING;
#if BUZZ_ENABLE
      buzzerRapid=false; noTone(BUZZER_PIN);
#endif
    }
  }
  else if(line.startsWith("BACK")){
    int c1=line.indexOf(','); int spd=0;
    if(c1>0){ spd=line.substring(c1+1).toInt(); spd=constrain(spd,SPEED_MIN_CLAMP,SPEED_MAX_CLAMP); }
    current_speed = spd; current_dir = (spd>0)? -1:0;
    if(state!=TURNING) state=CENTERING;
#if BUZZ_ENABLE
    buzzerRapid=false; noTone(BUZZER_PIN);
#endif
  }
  else if(line.startsWith("TURN_ABS")){
    int c1=line.indexOf(','); if(c1>0){
      int c2=line.indexOf(',',c1+1);
      target_yaw_deg = wrap180(line.substring(c1+1,(c2>0?c2:line.length())).toFloat());
      turn_pwm_override=-1; turn_drive_dir=+1;
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
      state=TURNING; pid_i=0; prev_err=0; d_filt=0; lastPIDms=0; turnStartMs=millis();
#if BUZZ_ENABLE
      buzzerRapid=true; buzzLastMs=0; beep(120);
#endif
    }
  }
  else if(line.startsWith("STEER_DEG")){   // absolute servo set (65..120)
    int c1=line.indexOf(',');
    if(c1>0){
      int d=line.substring(c1+1).toInt();
      steerTo(d);
    }
  }
  else if(line.startsWith("TRIM_NORM")){
    int c1=line.indexOf(','); if(c1>0){
      STEER_TRIM_NORM=line.substring(c1+1).toFloat();
      STEER_TRIM_NORM=constrain(STEER_TRIM_NORM,-0.3f,0.3f);
      beep(40);
    }
  }
  else if(line.startsWith("SET_CENTER")){
    int c1=line.indexOf(','); if(c1>0){
      SERVO_CENTER=constrain(line.substring(c1+1).toInt(),SERVO_MIN,SERVO_MAX);
      syncCenterSpans(); steerTo(SERVO_CENTER); beep(80);
    }
  }
  else if(line.startsWith("SET_TURN_PWM")){
    int c1=line.indexOf(','); if(c1>0){
      TURN_PWM_DEFAULT=constrain(line.substring(c1+1).toInt(),SPEED_MIN_CLAMP,SPEED_MAX_CLAMP);
      beep(60);
    }
  }
  else if(line.startsWith("CLEAR_TURN_PWM")){
    turn_pwm_override=-1; beep(40);
  }
  else if(line.startsWith("SET_US_TIMEOUT")){
    int c1=line.indexOf(','); if(c1>0){ US_TIMEOUT=(unsigned long)line.substring(c1+1).toInt(); beep(40); }
  }
  else if(line.startsWith("SET_US_FAR_CM")){
    int c1=line.indexOf(','); if(c1>0){ US_FAR_CM=line.substring(c1+1).toInt(); beep(40); }
  }
}

/* ============ Telemetry (~60 Hz) ============ */
void sendTelemetry(){
  static unsigned long last = 0;
  unsigned long now = millis();
  if(now - last >= 16){
    last = now;
    Serial.print("TLM,yaw=");   Serial.print(yaw_deg, 1);
    Serial.print(",state=");    Serial.print((int)state);
    Serial.print(",steer=");    Serial.print(current_steer_norm, 2);
    Serial.print(",speed=");    Serial.print(current_speed);
    Serial.print(",dF=");       Serial.print(distF);
    Serial.print(",dL=");       Serial.print(distL);
    Serial.print(",dR=");       Serial.print(distR);
    Serial.print(",dB=");       Serial.print(distB);
    // extra field (safe: Python regex doesn't anchor end)
    long c; noInterrupts(); c = encCount; interrupts();
    Serial.print(",enc_cm=");   Serial.println(countsToCm(c), 2);
  }
}

/* ================= Setup / Loop ================= */
void setup(){
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  bts_init(); stopMotor();
  myservo.attach(SERVO_PIN); syncCenterSpans(); steerTo(SERVO_CENTER);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  prevState = readABFast();
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  PCICR  |= (1 << PCIE1);                          // enable PCINT for PORTC
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);       // A2, A3
#else
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoderAttach, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), handleEncoderAttach, CHANGE);
#endif

  Wire.begin(); Wire.setClock(100000);
  initIMU();
  digitalWrite(A0,1);
  delay(1000);
  digitalWrite(A0,0);
  Serial.println("READY");
}

void loop(){
  chirpRapidTick();
  readUSStaggered();

  // Serial lines
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){
      if(rx.length()>0){ handleLine(rx); rx=""; }
    } else {
      rx += ch; if(rx.length()>128) rx.remove(0);
    }
  }

  // Local button
  if(!armed && digitalRead(START_BTN) == LOW){
    delay(20);
    if(digitalRead(START_BTN) == LOW){
      setArmed(true);
      while(digitalRead(START_BTN) == LOW) { delay(5); }
    }
  }

  // IMU
  readYaw();
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
      break;

    case CENTERING:{
      steer_f += ALPHA_STEER * (current_steer_norm - steer_f);
      speed_f += ALPHA_SPEED * ((float)current_speed - speed_f);
      float u_eff = (current_dir < 0) ? -steer_f : steer_f; // flip in reverse
      setSteerNorm(u_eff);
      setDrive((int)speed_f, current_dir);
      break;
    }

    case TURNING:
      runTurnController();
      break;

    case DISTMOVE:
      runDistanceMove();
      break;
  }

  sendTelemetry();
}
