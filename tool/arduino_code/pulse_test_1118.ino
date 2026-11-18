// =====================================================================================
// pulse_test_1118.ino ― 구동 모터 속도(엔코더 펄스) PID 튜닝 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 2026-11-18. pulsetest_interrupt_1112.ino 의 인터럽트 계측(2 · 3번 핀, CHANGE)에 PID 를 얹었다 — 핀 배치도 같다.
// REF(목표 펄스값)를 받아, 측정한 펄스값이 REF 에 맞도록 PWM 을 조절한다. PID 게인은 시리얼 명령으로 바로 바꾼다.
// 레거시(legacy_white.ino 의 Velo_PID_Control)는 구조만 참고했다 — 무엇을 남기고 뺐는지는 아래 '레거시와 다른 점'.
// 컴파일은 R4 코어(renesas_uno 1.6.0, FQBN arduino:renesas_uno:minima)로 통과했다 (2026-11-18).
// ★업로드 · 실제 동작은 아직 확인 못 했다★.
//
// ── 펄스값 (REF · 측정 모두 같은 단위) ───────────────────────────────────────────────
//   '최근 100 ms 동안 A · B 두 상에 들어온 에지 수의 합'. 정수.
//   인터럽트는 CHANGE (오르내림 모두) — 한 상 한 주기에 2, 두 상 합이라 엔코더 한 주기에 4 가 는다.
//   PID 는 10 ms 마다 돈다. 펄스값은 10 ms 마다 '방금까지의 100 ms' 로 새로 센다 (이동창 — 창이 겹친다).
//   pulsetest_interrupt_1112 출력(상마다 1 초 에지 수)과 견주면 : 펄스값 ≈ (A 의 1 초 수 + B 의 1 초 수) / 10
//     — 두 상이 같다면 '한 상 1 초 수의 1/5'.
//   분해능 = 에지 1 개. 펄스값이 한 자릿수인 저속에서는 거칠다.
//   방향은 모른다 (A · B 를 따로 셀 뿐 앞뒤를 가리지 않는다) — 늘 0 이상으로 센다.
//   ★한 상 배선이 빠지면 펄스값이 절반이 되고 PID 는 모터를 두 배로 돌린다★ — 먼저 pulsetest 로 두 상이 같은지 본다.
//
// ── 방향 ─────────────────────────────────────────────────────────────────────────
//   REF 부호 = 방향. + → DIR LOW, − → DIR HIGH (pulsetest 와 같다). PID 는 크기 |REF| 만 맞춘다.
//   PWM 은 0 ~ 상한 — 반대로 돌려 감속하지 않는다 (방향을 모르는 계측이라 역회전을 섞으면 측정이 꼬인다).
//     측정이 REF 보다 크면 PWM 이 0 까지 내려간다 (드라이버가 PWM Low 를 브레이크로 쓰면 브레이크, 아니면 관성).
//   REF 부호가 바뀌면 I 를 지운다.
//
// ── 제어 ─────────────────────────────────────────────────────────────────────────
//   10 ms(100 Hz)마다 : 펄스값 세기 → PID → PWM.
//   오차 = |REF| − 측정.   P = Kp · 오차.   I = Ki · 오차 적분.
//   D = −Kd · (측정의 변화율) — 목표를 바꿀 때 튀지 않게 오차가 아니라 측정값으로 미분한다.
//     측정이 10 ms 마다 에지 단위로 계단처럼 바뀌어 D 는 잡음이 크다 (1 에지/10 ms = 100 펄스/s) — 쓰려면 아주 작게.
//   출력(PWM)은 0 ~ 상한으로 자른다. 출력이 상한 · 0 에 걸린 쪽으로는 I 를 더 쌓지 않는다 (와인드업 방지), 0 ≤ I ≤ 상한.
//   REF 0 → PWM 0, PID 끄기, I 지우기.
//   게인 단위 : Kp = PWM/펄스, Ki = PWM/(펄스·s), Kd = PWM/(펄스/s).   (펄스 = 위 펄스값 1)
//   최소 PWM 보정 · 피드포워드 같은 것은 넣지 않았다 — PID 만으로 처음부터 튜닝한다.
//   부팅 직후는 정지(PWM 0)다. 명령 시한(무수신 정지)은 넣지 않았다 — 0 이나 's' 로 세운다.
//
// ── 레거시와 다른 점 (legacy_white.ino Velo_PID_Control 분석, 2026-11-18) ─────────────────
//   레거시 구동 PID 에는 와인드업 방지 같은 장치가 없다 (그런 장치는 조향 PID 에만 있다).
//   남김  측정 = 센 펄스 수 그대로 (레거시 d_val) · 출력 상한 255 · 목표 0 이면 PWM 0.
//   바꿈  적분 : Sum_Error += Error (상한 없음, dt 없음 — PWM 이 255 에 걸려도 계속 쌓였다)
//               → 조건부 적분 + 0 ~ 상한, dt(초)를 곱한다.
//         목표 0 일 때 I 를 지우지 않아 서는 동안 쌓인 값이 다음 출발에 튀던 것 → 지운다.
//         미분 : 오차의 차이 (Kd 15, 목표를 바꾸면 튐) → 측정값의 변화율, 기본 Kd 0.
//         주기 : millis ≥ 10 ms 검사였지만 loop 의 delay(20) 때문에 실제로는 20 ms 넘게 들쭉날쭉했고
//                계산한 dt_ms 는 쓰지 않았다 → micros 로 10 ms 고정.
//         세는 창 : 주기와 같았다 → 100 ms 이동창 (이 엔코더는 10 ms 에 에지가 몇 개 안 된다).
//   뺌    음수 PWM (역회전으로 감속) — 레거시 카운터(LS7366R)는 방향을 알았지만 여기 계측은 모른다.
//         black 백업의 'PWM ≥ 205 → 60' · '속도 ≥ 120 → 10' 처럼 넘으면 꺾어 떨어뜨리는 것 — 불연속이다.
//         조향 PID 의 출력 변화율 제한 · 데드밴드 — 응답을 가려 튜닝이 헷갈린다 / 속도 제어에서는 멈칫댄다.
//
// ── 입력 (PC → 아두이노) — 한 줄에 명령 하나, 줄 끝 '\n' · '\r' 어느 쪽이든 ────────────────
//   35       정수만 → REF (펄스값, 부호 = 방향), PID 켜기.  0 → 정지
//   p 1      Kp      i 0      Ki      d 0      Kd          (0 이상, 소수 가능)
//   m 255    PWM 상한 (0 ~ 255)
//   o 100    손 PWM (−255 ~ 255, PID 끄고 그대로 낸다 — pulsetest 와 같은 개루프 확인용. 'm' 상한은 그대로 걸린다)
//   s        정지 (PWM 0, PID 끄기, I 지우기)
//   글자는 대소문자 무관, 글자와 숫자 사이 공백은 있어도 없어도 된다. 알아듣지 못한 줄은 말없이 버린다.
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다.
//   ★응답 문구가 없다★ — 바꾼 게인은 화면에 나오지 않으니 친 값을 적어 둔다.
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   50 ms 마다 "<REF> <측정 펄스값> <PWM>" 세 정수만 한 줄. 문구는 없다 — 시리얼 플로터에 세 줄 그래프로 그대로 뜬다.
//   측정에는 지금 DIR 의 부호를 붙인다 (계측은 방향을 모른다 — REF 가 음수면 측정도 음수로 보인다).
//   PWM 은 지금 모터에 내고 있는 값이다 (−255 ~ 255, 음수 = 역방향 = DIR HIGH, 'm' 상한으로 자른 뒤). 정지면 0.
//     펄스값과 단위가 달라 같은 눈금에서는 높이가 맞지 않는다 — 상한에 붙었는지 · 떨리는지를 보는 용도.
//   PID 가 꺼져 있으면 (정지 · 손 PWM) REF 는 0 으로 나온다.
//
// ── 튜닝 순서 (제안) ──────────────────────────────────────────────────────────────
//   1) o 100 처럼 손 PWM 을 줘 펄스값이 나오는지, pulsetest 결과와 맞는지 본다 → s.
//   2) Ki · Kd 0 으로 Kp 만 : REF 를 0 ↔ 중간값으로 바꿔 가며 본다. P 만이면 REF 에 못 미친 채 선다 — 정상이다.
//      출렁이기 시작하는 Kp 를 찾고 그 절반쯤.
//   3) Ki 를 조금씩 올려 남은 오차를 없앤다.   4) 넘침이 크면 Ki 를 내리거나 Kd 를 아주 조금.
//   5) 정한 값을 아래 기본값에 적는다.
//
// ── 핀 (UNO R4 Minima, pulsetest_interrupt_1112 와 같다) ─────────────────────────────
//   모터 드라이버 1 : PWM 5 · DIR 6.   엔코더 A 2 · B 3 (외부 인터럽트가 되는 핀).

#include <math.h>
#include <stdlib.h>
#include <ctype.h>

// ===================== 핀 =====================
const uint8_t PIN_M1_PWM = 5;     // 모터 드라이버 1 PWM
const uint8_t PIN_M1_DIR = 6;     // 모터 드라이버 1 DIR (+ LOW · − HIGH)
const uint8_t PIN_ENC_A  = 2;     // 엔코더 A 상
const uint8_t PIN_ENC_B  = 3;     // 엔코더 B 상

#define SERIAL_BAUD 115200        // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

// ===================== 주기 · 창 =====================
const unsigned long CTRL_US     = 10000;               // 10 ms = 100 Hz
const float         DT_S        = CTRL_US * 1e-6f;
const uint8_t       WIN_TICKS   = 10;                  // 세는 창 = 10 주기 = 100 ms
const unsigned long TELEM_MS    = 50;                  // 출력 간격
const unsigned long CMD_IDLE_MS = 100;                 // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간
const int           PWM_MAX     = 255;
const long          REF_ABS_MAX = 10000;               // 오타로 큰 수가 들어와도 셈이 넘치지 않게만 자른다 (실제 한계 아님)

// ===================== PID 기본값 — 튜닝한 값을 여기에 적는다 =====================
float Kp        = 3.0f;     // PWM/펄스
float Ki        = 30.0f;    // PWM/(펄스·s)
float Kd        = 0.0f;     // PWM/(펄스/s)
int   pwm_limit = 255;      // 0 ~ 255

// ===================== 엔코더 펄스 카운터 (인터럽트 → loop) =====================
// 부팅 뒤 누적 에지 수. 되돌리지 않고 차이로 쓴다 — 32 비트 읽기는 Cortex-M4 에서 한 번에 끝나므로
// 인터럽트를 끄지 않아도 되고, 넘쳐 돌아도 부호 없는 뺄셈이라 맞다.
volatile uint32_t cnt_a = 0;
volatile uint32_t cnt_b = 0;

void onEncoderA() { cnt_a++; }
void onEncoderB() { cnt_b++; }

uint32_t win_hist[WIN_TICKS];   // 지난 10 주기 각각의 누적 에지 수 (A+B)
uint8_t  win_idx = 0;           // 다음에 덮어쓸 자리 = 가장 오래된 값 (10 주기 전)

// ===================== 상태 =====================
enum Mode : uint8_t { MODE_STOP, MODE_PID, MODE_OPEN };
Mode  mode      = MODE_STOP;
long  ref       = 0;        // REF (펄스값, 부호 = 방향). PID 가 꺼져 있으면 0
int   open_pwm  = 0;
long  meas      = 0;        // 측정 펄스값 (0 이상)
long  meas_prev = 0;        // 지난 주기의 측정 (D 항)
float i_term    = 0.0f;     // I 항 (PWM 단위)
int   pwm_out   = 0;        // 실제로 낸 PWM (+ / −)
bool  dir_neg   = false;    // 지금 DIR 이 − 쪽(HIGH)인가

unsigned long ctrl_last_us  = 0;
unsigned long telem_last_ms = 0;

// ===================== 펄스값 세기 =====================
uint32_t edgesTotal() { return cnt_a + cnt_b; }   // 두 읽기 사이에 낀 에지는 다음 주기에 잡힌다

void countPulses() {
  uint32_t total = edgesTotal();
  meas = (long)(total - win_hist[win_idx]);   // 10 주기 전과의 차이 = 최근 100 ms 에지 수
  win_hist[win_idx] = total;
  win_idx = (uint8_t)((win_idx + 1) % WIN_TICKS);
}

// ===================== 모터 (+ DIR LOW · − DIR HIGH) =====================
void setMotor(int v) {
  v = constrain(v, -pwm_limit, pwm_limit);
  if (v != 0) dir_neg = (v < 0);              // 0 이면 DIR 은 그대로 둔다 (출력 부호도 그대로)
  digitalWrite(PIN_M1_DIR, dir_neg ? HIGH : LOW);
  analogWrite(PIN_M1_PWM, v < 0 ? -v : v);
  pwm_out = v;
}

// ===================== PID =====================
// 크기만 맞춘다 : 0 ~ pwm_limit 을 구해 REF 부호를 붙여 돌려준다
int pidStep() {
  float lim  = (float)pwm_limit;
  float err  = (float)labs(ref) - (float)meas;
  float rate = (float)(meas - meas_prev) / DT_S;   // 펄스/s

  float p = Kp * err;
  float d = -Kd * rate;
  float i_new = constrain(i_term + Ki * err * DT_S, 0.0f, lim);

  float u = p + i_new + d;
  // 와인드업 방지 : 출력이 상한(위) · 0(아래)에 걸린 쪽으로는 I 를 더 쌓지 않는다
  bool sat_same_side = (u > lim && err > 0) || (u < 0.0f && err < 0);
  if (!sat_same_side) i_term = i_new;
  else                u = p + i_term + d;

  u = constrain(u, 0.0f, lim);
  int pwm = (int)lroundf(u);
  return ref < 0 ? -pwm : pwm;
}

void controlStep() {
  countPulses();
  switch (mode) {
    case MODE_PID:  setMotor(pidStep()); break;
    case MODE_OPEN: setMotor(open_pwm); break;
    default:        setMotor(0); break;
  }
  meas_prev = meas;
}

// ===================== 출력 : "<REF> <측정 펄스값> <PWM>" =====================
void printTelemetry() {
  char line[40];
  int n = snprintf(line, sizeof(line), "%ld %ld %d\r\n", ref, dir_neg ? -meas : meas, pwm_out);
  Serial.write((const uint8_t *)line, n);   // 한 번에 보낸다 (USB 전송 횟수를 줄인다)
}

// ===================== 명령 =====================
void stopAll() {   // 다음 주기를 기다리지 않고 바로 선다
  mode = MODE_STOP; ref = 0; open_pwm = 0; i_term = 0.0f; setMotor(0);
}

void setRef(long v) {
  if (v == 0) { stopAll(); return; }
  v = constrain(v, -REF_ABS_MAX, REF_ABS_MAX);
  bool flip = (ref != 0) && ((v < 0) != (ref < 0));
  if (mode != MODE_PID || flip) i_term = 0.0f;   // 새로 켜거나 방향이 바뀌면 I 를 지운다
  ref  = v;
  mode = MODE_PID;
}

// 앞뒤 공백 · 부호(+/−)만 허락하는 정수 읽기. 다른 글자가 섞이면 false
bool parseLong(const char *s, long *out) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return false;
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  *out = v;
  return true;
}

// 앞뒤 공백만 허락하는 실수 읽기. 다른 글자가 섞이면 false
bool parseFloat(const char *s, float *out) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return false;
  char *end;
  double v = strtod(s, &end);
  if (end == s) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  *out = (float)v;
  return true;
}

// 알아듣지 못한 줄은 그냥 버린다 (응답 문구 없음)
void handleLine(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  char c = (char)tolower((unsigned char)*s);

  if (c == 's') { stopAll(); return; }

  if (isalpha((unsigned char)c)) {
    float v;
    if (!parseFloat(s + 1, &v)) return;
    switch (c) {
      case 'p': if (v >= 0) Kp = v; break;
      case 'i': if (v >= 0) Ki = v; break;
      case 'd': if (v >= 0) Kd = v; break;
      case 'm': if (v >= 0 && v <= PWM_MAX) {
                  pwm_limit = (int)lroundf(v);
                  if (i_term > pwm_limit) i_term = pwm_limit;
                }
                break;
      case 'o': if (v >= -PWM_MAX && v <= PWM_MAX) {
                  open_pwm = (int)lroundf(v); mode = MODE_OPEN; ref = 0; i_term = 0.0f;
                }
                break;
      default:  break;
    }
    return;
  }

  // 정수만 → REF
  long v;
  if (!parseLong(s, &v)) return;
  setRef(v);
}

// ===================== 명령 수신 (막히지 않게 한 글자씩) =====================
char          cmd_buf[24];
uint8_t       cmd_len     = 0;
bool          cmd_bad     = false;   // 줄이 버퍼보다 길었다
unsigned long cmd_last_ms = 0;

void applyCommand() {
  if (cmd_len == 0 && !cmd_bad) return;   // "\r\n" 의 두 번째 글자 같은 빈 줄
  cmd_buf[cmd_len] = '\0';
  if (!cmd_bad) handleLine(cmd_buf);
  cmd_len = 0;
  cmd_bad = false;
}

void readCommand() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    cmd_last_ms = millis();
    if (c == '\n' || c == '\r') {
      applyCommand();
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    } else {
      cmd_bad = true;
    }
  }
  if ((cmd_len > 0 || cmd_bad) && millis() - cmd_last_ms >= CMD_IDLE_MS) applyCommand();
}

// ===================== setup · loop =====================
void setup() {
  pinMode(PIN_M1_DIR, OUTPUT);
  digitalWrite(PIN_M1_DIR, LOW);
  pinMode(PIN_M1_PWM, OUTPUT);
  analogWrite(PIN_M1_PWM, 0);

  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoderB, CHANGE);
  // R4 코어의 attachInterrupt 는 핀 설정을 '인터럽트 + 입력' 으로 통째로 덮어써 풀업이 빠질 수 있다 —
  // 풀업을 다시 켠다 (pinMode 를 뒤에 부르면 이번엔 인터럽트가 꺼진다). 오픈컬렉터 엔코더면 풀업이 꼭 있어야 한다.
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_A].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_B].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);

  uint32_t total = edgesTotal();   // 창을 지금 값으로 채운다 — 첫 100 ms 는 짧은 창으로 센다
  for (uint8_t k = 0; k < WIN_TICKS; k++) win_hist[k] = total;

  ctrl_last_us  = micros();
  telem_last_ms = millis();
}

void loop() {
  // 1) 10 ms 마다 펄스값 세기 + 제어 — 밀리면 몰아 돌리지 않고 다시 맞춘다
  unsigned long now_us = micros();
  if (now_us - ctrl_last_us >= CTRL_US) {
    ctrl_last_us += CTRL_US;
    if (now_us - ctrl_last_us >= CTRL_US) ctrl_last_us = now_us;
    controlStep();
  }

  // 2) 명령
  readCommand();

  // 3) 출력
  unsigned long now_ms = millis();
  if (now_ms - telem_last_ms >= TELEM_MS) {
    telem_last_ms = now_ms;
    printTelemetry();
  }
}
