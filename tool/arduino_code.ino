// =====================================================================================
// arduino_code.ino ― white 1/5카 아두이노 Mega 2560 펌웨어 (짝: hl_ws/src/white/white/arduino.py)
// =====================================================================================
// 제어 로직은 ~/Arduino/test_arduino (2026-05-19 = 하네스 T8 PID Control Vr.1 + 속도 PID 안티 와인드업 ·
// PWM ±120 · 정지 때 PID 초기화) 그대로다 — 모터 · 엔코더 · 속도 PID · 조향 PID · PS2 값은 바꾸지 않았다.
// 고친 것은 통신 · 실행 순서 · 리모컨 우선권이다 (2026-11-09).
//
// ── 통신 (115200 bps 8N1, 줄 끝 '\n') ─────────────────────────────────────────────
//   PC → 아두이노
//     C,<vel>,<steer>   vel   = 목표 속도 [엔코더 틱 / 속도 PID 주기 20 ms], 정수 −255..255 (부호 = 방향)
//                       steer = 목표 조향 [도], 정수 — 조향 PID 가 −19..19 로 자른다
//     S,<0|1>           PC 제어 상태 — E 줄의 state 로 되돌려 보낼 뿐 동작은 바꾸지 않는다 (원래 코드와 같다)
//   아두이노 → PC
//     H,white_fw,<버전>,<baud>                       부팅 직후 한 번 — 어떤 펌웨어인지 PC 가 안다
//     E,<d_val>,<sensorValue>,<PWM>,<Steering_Angle>,<state>,<remote>,<dt_ms>
//                       속도 PID 주기(20 ms)마다 한 줄. 앞 5필드는 Vr.1 과 같은 뜻이다.
//                       d_val  = dt_ms 동안 센 엔코더 틱 (부호 = 방향)
//                       PWM    = 구동 출력 (리모컨이 쥐고 있으면 리모컨 PWM)
//                       remote = 1 이면 L1/R1 이 눌려 아두이노가 쥐고 있다 (PC 는 표시만 한다)
//                       dt_ms  = 이번 d_val 을 잰 시간 — PC 가 속도 환산에 그대로 쓴다
//
// ── Vr.1 · test_arduino 대비 고친 것 ──────────────────────────────────────────────
//   1) 115200 bps. 부팅 줄(H)로 펌웨어를 알아본다 — 2026-11-09 차에 엉뚱한 스케치(115200)가 올라가 있는데
//      PC 는 57600 으로 들어 E 줄을 하나도 못 읽은 일이 있었다.
//   2) loop() 의 delay(20) 를 없애고 millis() 로 20 ms 마다 [PS2 → 속도 PID → E 줄] 을 돈다. 명령 수신은
//      매 바퀴 한다 — PC 명령이 최대 20 ms 기다리지 않고 곧바로 반영된다. E 줄 간격 · d_val 주기가 일정하다
//      (Vr.1 은 루프 처리 시간에 따라 21~23 ms 로 흔들렸다). ※ 속도 PID 주기가 약 22 → 20 ms 로 줄어 같은 실속도의
//      틱 수 · 오차가 10 % 가량 작다 — 게인은 그대로 두었다.
//   3) 명령 파서 — 숫자가 아닌 글자가 섞이거나 칸이 모자라면 그 줄을 버린다(atoi 는 "C,1x" 를 1 로 읽었다).
//      64 바이트를 넘는 줄은 다음 '\n' 까지 통째로 버린다(전에는 넘친 꼬리를 새 줄로 읽었다).
//   4) 명령 시한(워치독) — 리모컨이 아닐 때 유효한 C 줄이 CMD_TIMEOUT_MS 넘게 없으면 속도 0 (조향은 마지막 값 유지).
//      PC 프로그램이 죽거나 USB 가 빠져도 차가 마지막 속도로 계속 가지 않는다. 부팅 직후도 0 이다.
//   5) ★리모컨 우선권은 아두이노가 전부 쥔다★ — L1/R1 을 누르는 동안 PC 의 C 명령은 받기만 하고 속도 · 조향
//      모두 쓰지 않는다(Vr.1 은 조향을 C 가 덮어써 리모컨과 20 ms 마다 다퉜다). 구동은 원래대로 리모컨 PWM
//      개루프, 그동안 속도 PID 는 모터를 건드리지 않고 상태만 비운다 — test_arduino 는 PID 를 리모컨 분기 뒤로
//      옮겨 놓아 리모컨 PWM 을 곧바로 0 으로 덮어썼다. 놓으면 원래 코드처럼 PC 명령(시한 안이면)을 다시 따른다.
//   6) 타이머 인터럽트(조향 PID)와 loop 가 같이 쓰는 변수를 volatile 로 두고, 여러 바이트 값은 인터럽트를
//      잠깐 막고 읽고 쓴다(16비트 값이 반쯤 바뀐 채 읽히지 않게).
//   7) 인터럽트가 digitalWrite(0, …) 로 0번 핀(= USB 시리얼 수신 RX0)을 건드리던 것을 13번 LED 로 옮겼다.
//      LED : 리모컨이 쥐면 켜짐, PC 명령이 들어오는 중이면 빠르게(5 Hz), 그 밖에는 느리게(1 Hz) 깜빡인다.
//
// 필요한 라이브러리 : MsTimer2, PS2X_lib (~/Arduino/libraries). 보드 : Arduino Mega 2560.
// Arduino IDE 는 스케치가 같은 이름의 폴더(arduino_code/arduino_code.ino) 안에 있어야 연다 — 열 때 옮기자고 묻는다.

#include <MsTimer2.h>
#include <PS2X_lib.h> // for v1.6
#include <SPI.h>
#include <math.h>
#include <stdlib.h>

#define FW_NAME     "white_fw"
#define FW_VERSION  "1.0"
#define SERIAL_BAUD 115200

const unsigned long CONTROL_PERIOD_MS = 20;    // PS2 → 속도 PID → E 줄 주기
const unsigned long CMD_TIMEOUT_MS    = 500;   // 명령 시한
const int           VEL_LIMIT         = 255;   // C 의 vel 범위 (원래 코드와 같다)

// ===================== PC 명령 (Raw Serial Protocol) =====================
int  pc_velocity = 0;               // 마지막으로 받은 C 의 vel
int  pc_steer    = 0;               // 마지막으로 받은 C 의 steer
unsigned long last_cmd_ms = 0;      // 마지막 유효 C 줄 시각
bool cmd_seen   = false;            // 부팅 뒤 C 를 한 번이라도 받았나
bool pc_new     = false;            // 아직 반영하지 않은 새 C 가 있다 (리모컨을 놓은 뒤 조향은 새 C 부터 따른다)
bool state      = false;            // S 명령 (에코만)

volatile int steer_angle = 0;       // 조향 목표 — loop 가 쓰고 조향 인터럽트가 읽는다

static char    rx_line[64];
static uint8_t rx_idx = 0;
static bool    rx_overflow = false; // 넘친 줄 — 다음 '\n' 까지 버린다

// ===================== Front Motor Drive (구동) =====================
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

void motor_control(int motor_pwm_output)
{
  if (motor_pwm_output > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor_pwm_output);

    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor_pwm_output);
  }
  else if (motor_pwm_output < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor_pwm_output);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor_pwm_output);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, 0);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, 0);
  }
}

// ===================== Encoders (SPI) =====================
#define ENC1_ADD 22
#define ENC2_ADD 23
signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();

  // Encoder 1 init
  digitalWrite(ENC1_ADD, LOW);
  SPI.transfer(0x88);  // write MDR0
  SPI.transfer(0x03);  // 4-byte mode, x4 quad
  digitalWrite(ENC1_ADD, HIGH);

  // Encoder 2 init
  digitalWrite(ENC2_ADD, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(ENC2_ADD, HIGH);
}

long readEncoder(int encoder_no)
{
  unsigned int c1, c2, c3, c4;
  long v;
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0x60); // read CNTR
  c1 = SPI.transfer(0x00);
  c2 = SPI.transfer(0x00);
  c3 = SPI.transfer(0x00);
  c4 = SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);
  v = ((long)c1 << 24) + ((long)c2 << 16) + ((long)c3 << 8) + (long)c4;
  return v;
}

void clearEncoderCount(int encoder_no) {
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0x98);  // write DTR
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);

  delayMicroseconds(100);

  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0xE0);  // load DTR -> CNTR
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);
}

// ===================== Motor Velocity PID (test_arduino 그대로) =====================
#define velo_Kp  12
#define velo_Ki  0.25
#define velo_Kd  15
#define VELO_PWM_LIMIT 120

long          Curr_val = 0, old_val = 0;
signed long   d_val = 0;       // dt_ms 동안의 엔코더 변화량, 부호 포함 (후진=음수)
int           velo_val = 0;
float         Error = 0, d_Error = 0, old_Error = 0, Sum_Error = 0;
int           PWM = 0;

void velo_control_callback() {
  Curr_val = -1L * readEncoder(1);   // 모터 방향에 맞게 부호 반전
  d_val    = Curr_val - old_val;
  old_val  = Curr_val;
}

void velo_pid_reset() {
  Error = d_Error = old_Error = Sum_Error = 0;
}

// 속도 PID 한 번 — CONTROL_PERIOD_MS 마다 loop 가 부른다 (엔코더는 그 전에 읽었다)
void Velo_PID_Control() {
  Error     = velo_val - d_val;
  d_Error   = Error - old_Error;
  Sum_Error += Error;

  if (velo_val == 0) {
    // 정지 명령 시 속도 PID 상태 초기화
    PWM       = 0;
    Sum_Error = 0;
    old_Error = 0;
  } else {
    PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);
    // 안티 와인드업: PWM 은 ±120 으로 포화시키고, 포화 시 이번 적분 누적을 취소
    if (PWM >  VELO_PWM_LIMIT) {
      PWM = VELO_PWM_LIMIT;
      Sum_Error -= Error;
    }
    if (PWM < -VELO_PWM_LIMIT) {
      PWM = -VELO_PWM_LIMIT;
      Sum_Error -= Error;
    }
  }

  motor_control(PWM);
  old_Error = Error;
}

// ===================== Steering PID (안정화 개정 — 값 그대로) =====================
#define Steering_Sensor A15
#define NEURAL_ANGLE 0
#define LEFT_STEER_ANGLE  -19
#define RIGHT_STEER_ANGLE  19
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

const int AD_MIN = -460;
const int AD_MAX =  423;

float Kp = 6.5;
float Ki = 3.0;
float Kd = 1.0;

const float STEER_DEADBAND_DEG = 0.8;
const float I_CLAMP = 80.0;
const int   PWM_LIMIT = 255;
const int   PWM_SLEW_PER_TICK = 15;
const float STEER_LPF_FC = 12.0f;

double error, error_old = 0.0;
double error_s = 0.0;
int pwm_output_prev = 0;

volatile int sensorValue = 0;        // 인터럽트가 쓰고 loop(E 줄)가 읽는다
float sensorValue_f = 0.0f;
int Steer_Angle_Measure = 0;
volatile int Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm)
{
  if ( (sensorValue >= AD_MAX) || (sensorValue <= AD_MIN) )
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
    return;
  }

  if (motor_pwm > 0) {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void Steer_PID_Control(float dt_s)
{
  error = (double)Steering_Angle - (double)Steer_Angle_Measure;

  if (fabs(error) <= STEER_DEADBAND_DEG) {
    steer_motor_control(0);
    error_s = 0.0;
    pwm_output_prev = 0;
    error_old = error;
    return;
  }

  double p_term = Kp * error;
  double d_term = Kd * ((error - error_old) / dt_s);

  double i_candidate = error_s + (Ki * error * dt_s);

  double u = p_term + i_candidate + d_term;
  double u_unsat = u;

  if (u > PWM_LIMIT) u = PWM_LIMIT;
  if (u < -PWM_LIMIT) u = -PWM_LIMIT;

  bool saturated = (u != u_unsat);
  if (!saturated || ((u > 0 && error < 0) || (u < 0 && error > 0))) {
    error_s = i_candidate;
    if (error_s > I_CLAMP)  error_s = I_CLAMP;
    if (error_s < -I_CLAMP) error_s = -I_CLAMP;
  }

  int u_cmd = (int)round(u);
  int du = u_cmd - pwm_output_prev;
  if (du >  PWM_SLEW_PER_TICK) u_cmd = pwm_output_prev + PWM_SLEW_PER_TICK;
  if (du < -PWM_SLEW_PER_TICK) u_cmd = pwm_output_prev - PWM_SLEW_PER_TICK;

  steer_motor_control(u_cmd);
  pwm_output_prev = u_cmd;
  error_old = error;
}

void steer_velo_control(float dt_s)
{
  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)
    Steering_Angle = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)
    Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;

  Steer_PID_Control(dt_s);
}

// MsTimer2 20 ms 인터럽트 — 조향 PID (원래 코드와 같다. 0번 핀 토글만 뺐다)
void control_callback()
{
  static unsigned long last_ms = millis();
  unsigned long now = millis();
  float dt_s = (now - last_ms) * 0.001f;
  if (dt_s <= 0.0f) dt_s = 0.02f;
  last_ms = now;

  int raw = analogRead(Steering_Sensor) - 512;
  float RC = 1.0f / (6.283185f * STEER_LPF_FC);
  float alpha = dt_s / (RC + dt_s);
  sensorValue_f = sensorValue_f + alpha * ((float)raw - sensorValue_f);
  sensorValue = (int)round(sensorValue_f);

  int sens = sensorValue;
  if (sens < AD_MIN) sens = AD_MIN;
  if (sens > AD_MAX) sens = AD_MAX;

  Steer_Angle_Measure = (int)round(
                          ( (double)(sens - AD_MIN) * (RIGHT_STEER_ANGLE - LEFT_STEER_ANGLE) / (double)(AD_MAX - AD_MIN) )
                          + LEFT_STEER_ANGLE
                        );

  Steering_Angle = NEURAL_ANGLE + steer_angle;   // 인터럽트 안이라 steer_angle 은 한 번에 읽힌다

  steer_velo_control(dt_s);
}

// ===================== Controller (PS2 — 값 그대로) =====================
#define PS2_DAT 17
#define PS2_CMD 16
#define PS2_SEL 15
#define PS2_CLK 14

PS2X ps2x;
byte type = 0;
byte vibrate = 0;
int con_st = 0;
int con_sp = 0;
int con_error = 0;
bool controller_true = false;

#define pressures   false
#define rumble      false

void controller() {
  ps2x.read_gamepad(false, vibrate);

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
    controller_true = true;
    con_sp = - (map(ps2x.Analog(PSS_LY), 0, 255, -10, 10)) * 10;
    con_st = map(ps2x.Analog(PSS_RX), 0, 255, -21, 21);
  } else {
    controller_true = false;
    con_sp = 0;
    con_st = 0;
  }
}

// ===================== PC 명령 수신 =====================
// 10진 정수 하나를 읽고 p 를 그 뒤로 옮긴다. 숫자가 없으면 false.
static bool take_int(const char*& p, long& out)
{
  char* end;
  out = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;
  return true;
}

// 한 줄 처리: "C,<vel>,<steer>" 또는 "S,<0|1>" — 형식이 하나라도 어긋나면 버린다
void process_rx_line(const char* line)
{
  long a, b;
  const char* p = line + 2;
  if (line[0] == 'C' && line[1] == ',') {
    if (!take_int(p, a) || *p != ',') return;
    p++;
    if (!take_int(p, b) || *p != '\0') return;
    if (a >  VEL_LIMIT) a =  VEL_LIMIT;
    if (a < -VEL_LIMIT) a = -VEL_LIMIT;
    if (b >  90) b =  90;          // 조향은 조향 PID 가 ±19 로 다시 자른다 — 여기서는 int 넘침만 막는다
    if (b < -90) b = -90;
    pc_velocity = (int)a;
    pc_steer    = (int)b;
    last_cmd_ms = millis();
    cmd_seen    = true;
    pc_new      = true;
    return;
  }
  if (line[0] == 'S' && line[1] == ',') {
    if (!take_int(p, a) || *p != '\0') return;
    state = (a != 0);
    return;
  }
}

// 바이트 단위로 읽어 '\n' 에서 한 줄을 처리한다 — 매 loop 바퀴마다 불러 지연이 없다
void serial_rx_poll()
{
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!rx_overflow && rx_idx > 0) {
        rx_line[rx_idx] = '\0';
        process_rx_line(rx_line);
      }
      rx_idx = 0;
      rx_overflow = false;
      continue;
    }
    if (rx_overflow) continue;
    if (rx_idx < (sizeof(rx_line) - 1)) {
      rx_line[rx_idx++] = ch;
    } else {
      rx_overflow = true;         // 이 줄은 '\n' 까지 통째로 버린다
      rx_idx = 0;
    }
  }
}

static void set_steer_target(int s)
{
  noInterrupts();                 // 16비트 값을 인터럽트가 반쯤 바뀐 채 읽지 않게
  steer_angle = s;
  interrupts();
}

// ===================== 텔레메트리 =====================
static char tx_line[64];

void send_telemetry(unsigned long dt_ms)
{
  int sv, sa;
  noInterrupts();
  sv = sensorValue;
  sa = Steering_Angle;
  interrupts();
  // 한 줄을 만들어 한 번에 보낸다 (최대 약 45 바이트 < TX 버퍼 64 — 기다리지 않는다)
  snprintf(tx_line, sizeof(tx_line), "E,%ld,%d,%d,%d,%d,%d,%lu\n",
           (long)d_val, sv, PWM, sa, state ? 1 : 0, controller_true ? 1 : 0, dt_ms);
  Serial.write(tx_line);
}

// ===================== Arduino setup/loop =====================
unsigned long t_ctrl = 0;           // 마지막 제어 주기 시각

void setup() {
  Serial.begin(SERIAL_BAUD);

  con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  type = ps2x.readType();

  pinMode(13, OUTPUT);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_ENB, OUTPUT);

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);

  initEncoders();
  clearEncoderCount(1);
  clearEncoderCount(2);

  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  pinMode(MOTOR3_ENB, OUTPUT);

  velo_pid_reset();
  old_val = Curr_val = -1L * readEncoder(1);
  error = error_s = error_old = 0.0;
  pwm_output_prev = 0;

  MsTimer2::set(20, control_callback);
  MsTimer2::start();

  // 부팅 줄 — PC 가 펌웨어 · baud 를 알아본다. PS2 결과(0 = 리모컨 수신기 정상)도 붙인다
  snprintf(tx_line, sizeof(tx_line), "H,%s,%s,%ld,ps2=%d\n", FW_NAME, FW_VERSION, (long)SERIAL_BAUD, con_error);
  Serial.write(tx_line);

  t_ctrl = millis();
}

void loop() {
  // ① PC 명령은 매 바퀴 받는다
  serial_rx_poll();

  unsigned long now = millis();
  bool cmd_fresh = cmd_seen && (now - last_cmd_ms <= CMD_TIMEOUT_MS);

  // LED : 리모컨 = 켜짐, PC 명령 들어오는 중 = 5 Hz, 그 밖 = 1 Hz
  if (controller_true)  digitalWrite(13, HIGH);
  else if (cmd_fresh)   digitalWrite(13, (now / 100) & 1);
  else                  digitalWrite(13, (now / 500) & 1);

  // ② 20 ms 마다 : PS2 → 엔코더 → 구동 → E 줄
  unsigned long dt = now - t_ctrl;
  if (dt < CONTROL_PERIOD_MS) return;
  t_ctrl = now;

  controller();
  velo_control_callback();          // d_val = 지난 dt 동안의 틱

  if (controller_true) {
    // ★리모컨이 전부 쥔다★ — PC 명령은 받기만 하고 쓰지 않는다. 구동은 원래대로 리모컨 PWM 개루프
    set_steer_target(-con_st);
    pc_new   = false;               // 쥐고 있는 동안 온 C 는 버린다 — 놓으면 원래 코드처럼 조향은 다음 C 까지 그대로
    velo_val = 0;
    velo_pid_reset();               // 놓은 뒤 PID 가 묵은 적분으로 튀지 않게
    PWM = con_sp;
    motor_control(con_sp);
  } else {
    // PC 명령을 따른다 — 시한이 지났으면 속도 0, 조향은 마지막 목표 유지
    velo_val = cmd_fresh ? pc_velocity : 0;
    if (pc_new) {
      set_steer_target(pc_steer);
      pc_new = false;
    }
    Velo_PID_Control();
  }

  send_telemetry(dt);
}
