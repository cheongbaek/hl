// =====================================================================================
// dualtest_1118.ino ― 주행(구동 속도) · 조향 동시 제어 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 2026-11-18. pulse_test_1118.ino(구동 속도 PID)와 steer_test_1116.ino(조향 PID)를 한 스케치로 합쳤다.
// 한 줄에 "목표 펄스 · 목표 조향각" 을 받아 두 PID 를 같은 10 ms 주기에서 함께 돌린다.
// 계측 · PID · 방향 규칙 · 게인은 두 스케치 그대로다 — 자세한 설명은 각 스케치 머리말을 본다.
// 게인을 바꾸는 시리얼 명령은 없다 (튜닝은 두 스케치로 따로 하고, 정한 값을 아래 기본값에 옮겨 적는다).
// 컴파일은 R4 코어(renesas_uno 1.6.0, FQBN arduino:renesas_uno:minima)로 통과했다 (2026-11-18).
// ★업로드 · 실제 동작은 아직 확인 못 했다★.
//
// ── 입력 (PC → 아두이노) — 한 줄에 하나, 줄 끝 '\n' · '\r' 어느 쪽이든 ─────────────────────
//   <목표 펄스> <목표 조향각>     두 숫자를 공백으로 띄운다.  예) 35 -20 → 펄스값 35 로 달리며 조향 −20°(좌)
//     목표 펄스   : 정수, 부호 = 방향(+ DIR LOW · − DIR HIGH). |값| 이 92 를 넘으면 ±92 로 자른다 (제한 속도).
//                   0 → 주행만 정지 (주행 PID 끄기, I 지우기, PWM 은 하강 제한을 따라 0 까지). 조향은 계속 목표각을 잡는다.
//     목표 조향각 : 정수(도). 음수 = 좌, 양수 = 우. −21 ~ 21 로 자른다 (제한각).
//   s        둘 다 바로 정지 (PWM 0 — 주행도 하강 제한 없이, 두 PID 끄기, I 지우기). 's' 로 시작하는 줄이면 뒤는 보지 않는다.
//   숫자가 하나뿐이거나 다른 글자가 섞인 줄은 말없이 버린다 (응답 문구 없음). 두 값 모두 정수만 — 소수점이 있으면 줄째 버린다 (예: 35 -20.5).
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다.
//   부팅 직후는 둘 다 정지(PWM 0)다. 첫 줄을 받으면 조향 PID 가 켜진다 (목표 펄스 0 이면 주행은 선 채로).
//   명령 시한(무수신 정지)은 넣지 않았다 — 's' 로 세운다. 조향은 PWM 0 이면 드라이버가 브레이크로 잡는다.
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   50 ms 마다 "<목표 펄스> <목표 조향각> <실제 펄스> <실제 조향각>" 네 숫자만 한 줄. 문구는 없다.
//     목표 펄스   : 자른 뒤의 값. 주행 PID 가 꺼져 있으면 0.
//     목표 조향각 : 자른 뒤의 값 (도, 정수). 정지 중에도 마지막 목표 그대로.
//     실제 펄스   : 최근 100 ms A+B 에지 수, 지금 DIR 의 부호를 붙인다 (계측은 방향을 모른다).
//     실제 조향각 : 가변저항(필터 후)을 각도로 환산한 값 (도, 소수 둘째 자리, 자르지 않는다).
//   펄스값(0 ~ 92)과 각도(±21)가 한 플로터에 함께 뜬다 — 눈금 고정용 ±21 직선은 넣지 않았다.
//
// ── 방향 (2026-11-16 사용자 교정 반영 — steer_test_1116 과 같다) ──────────────────────────
//   조향 모터 : MD20A 시험 버튼 기준 MB = 좌 · MA = 우 (사용자 확인).
//     데이터시트 진리표 : PWM High · DIR Low → MA 쪽,  DIR High → MB 쪽,  PWM Low → 브레이크(DIR 무관).
//     → DIR LOW = 우 · DIR HIGH = 좌 (상수 DIR_RIGHT = LOW).  조향 출력 + = 우, − = 좌.  PWM 0 이면 브레이크로 잡힌다.
//   조향 가변저항 : ★좌로 갈수록 값이 커진다★ (좌 끝 ADC 954 실측 — 레거시가 52 쪽을 LEFT 라 부른 것과 반대).
//     → ADC 954 = −21°(좌), ADC 52 = +21°(우), 0° ≈ ADC 503.  각도 음수 = 좌 · 양수 = 우.
//   steer_test_1116 이 이 두 설정으로 실차에서 방향 맞음을 확인했다 (2026-11-16) — 여기서도 그대로 쓴다.
//   주행 모터 : 목표 펄스 + → DIR LOW, − → DIR HIGH (pulsetest_interrupt · pulse_test 와 같다).
//
// ── 주행 (pulse_test_1118 그대로) ───────────────────────────────────────────────────
//   펄스값 = 최근 100 ms 동안 A · B 상 에지(CHANGE) 수의 합, 10 ms 마다 이동창으로 센다.
//   PID 는 크기 |목표| 만 맞춘다 : PWM 0 ~ 255, 조건부 적분(0 ≤ I ≤ 상한), D 는 측정값 미분. 목표 부호가 바뀌면 I 를 지운다.
//   하강 제한 : |PWM| 이 줄 때만 1 초에 1000 보다 빨리 줄지 않게 한다 (10 ms 에 10, 늘 때는 그대로) —
//     이 드라이버는 PWM 을 낮추면 그만큼 제동이 걸려 한 번에 떨어뜨리면 감속 충격이 난다 (2026-11-18 pulse_test 에서 정한 값).
//     이 한계에 막혀 있는 동안은 I 를 줄이지 않는다. 방향이 바뀌면 옛 방향 PWM 을 0 까지 내린 뒤 새 방향으로 낸다.
//
// ── 조향 (steer_test_1116 그대로) ───────────────────────────────────────────────────
//   가변저항 → 1차 저역 필터(15 Hz) → 각도 → PID → PWM ±150 (방향은 위 '방향').
//   |오차| ≤ 데드밴드 0.3° 면 PWM 0 (I 는 그대로). 조건부 적분(|I| ≤ 상한), D 는 측정값 미분. 끝 보호 없음.
//   ★우 끝 ADC 52 는 레거시 값(실측 아님)★ — 실측하면 steer_test 와 함께 고친다.
//
// ── 핀 (UNO R4 Minima) — 서로 겹치지 않는다 ──────────────────────────────────────────
//   주행 : 모터 드라이버 PWM 5 · DIR 6, 엔코더 A 2 · B 3 (외부 인터럽트).   ← pulsetest_interrupt_1112 와 같다
//   조향 : 모터 드라이버(Cytron MD20A) PWM 10 · DIR 8, 가변저항 A0.        ← steer_test_1116 과 같다
//   PWM 5 (P102) 와 10 (P112) 은 서로 다른 타이머 채널(GPT2 · GPT3)이라 analogWrite 가 서로 건드리지 않는다.

#include <math.h>
#include <stdlib.h>
#include <ctype.h>

// ===================== 핀 =====================
const uint8_t PIN_DRV_PWM   = 5;    // 구동 모터 드라이버 PWM
const uint8_t PIN_DRV_DIR   = 6;    // 구동 모터 드라이버 DIR (+ LOW · − HIGH)
const uint8_t PIN_ENC_A     = 2;    // 엔코더 A 상
const uint8_t PIN_ENC_B     = 3;    // 엔코더 B 상
const uint8_t PIN_STEER_PWM = 10;   // 조향 모터 드라이버(Cytron MD20A) PWM
const uint8_t PIN_STEER_DIR = 8;    // 조향 모터 드라이버(Cytron MD20A) DIR
const uint8_t PIN_STEER_POT = A0;   // 조향 가변저항

const uint8_t DIR_RIGHT = LOW;      // 우로 갈 때의 조향 DIR 수준 — MD20A DIR LOW → MA 쪽, 이 차는 MA = 우 (머리말 '방향')

#define SERIAL_BAUD 115200          // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

// ===================== 가변저항 ↔ 각도 (steer_test 그대로) =====================
const int   POT_LEFT_ADC  = 954;    // 좌 끝 ADC (실측 2026-11-16)
const int   POT_RIGHT_ADC =  52;    // 우 끝 ADC (레거시 Vr.1 값 −460 + 512, ★실측 아님★)
const float ANG_LEFT      = -21.0f; // 좌 끝 각 = 제한각 (음수 = 좌)
const float ANG_RIGHT     =  21.0f; // 우 끝 각 = 제한각

// ===================== 주기 · 창 · 필터 · 제한 =====================
const unsigned long CTRL_US     = 10000;                   // 10 ms = 100 Hz (주행 · 조향 같은 주기)
const float         DT_S        = CTRL_US * 1e-6f;
const uint8_t       WIN_TICKS   = 10;                      // 펄스값 세는 창 = 10 주기 = 100 ms
const float         POT_LPF_FC  = 15.0f;                   // 가변저항 1차 저역 필터 차단 주파수 (Hz)
const float         POT_ALPHA   = DT_S / (1.0f / (6.2831853f * POT_LPF_FC) + DT_S);
const unsigned long TELEM_MS    = 50;                      // 출력 간격
const unsigned long CMD_IDLE_MS = 100;                     // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간
const long          REF_ABS_MAX = 92;                      // 제한 속도 (펄스값)
const long          ANG_ABS_MAX = 21;                      // 제한각 (도, 정수) — 목표 조향각을 자르는 값

// ===================== PID 기본값 — 두 시험 스케치의 값 그대로 =====================
// 주행 (pulse_test_1118)
const float DRV_KP        = 3.0f;     // PWM/펄스
const float DRV_KI        = 30.0f;    // PWM/(펄스·s)
const float DRV_KD        = 0.0f;     // PWM/(펄스/s)
const int   DRV_PWM_LIMIT = 255;      // 0 ~ 255
const float DRV_DN_RATE   = 1000.0f;  // PWM/s — |PWM| 이 줄어드는 빠르기의 상한 (pulse_test 의 'r'). 0 = 제한 없음
// 조향 (steer_test_1116)
const float STR_KP        = 12.0f;    // PWM/도
const float STR_KI        = 20.0f;    // PWM/(도·s)
const float STR_KD        = 0.0f;     // PWM/(도/s)
const float STR_DEADBAND  = 0.3f;     // 도
const int   STR_PWM_LIMIT = 150;      // 0 ~ 255

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
// 주행
bool  drv_on      = false;    // 주행 PID 켜짐
long  ref         = 0;        // 목표 펄스 (부호 = 방향). PID 가 꺼져 있으면 0
long  meas        = 0;        // 실제 펄스 (0 이상)
long  meas_prev   = 0;        // 지난 주기의 실제 펄스 (D 항)
float drv_i       = 0.0f;     // 주행 I 항 (PWM 단위)
float drv_out_f   = 0.0f;     // 실제로 낸 구동 PWM (+ / −, 반올림 전) — 하강 제한의 기준
bool  drv_dir_neg = false;    // 지금 구동 DIR 이 − 쪽(HIGH)인가
// 조향
bool  str_on      = false;    // 조향 PID 켜짐
long  target_deg  = 0;        // 목표 조향각 (도, 정수만 — 입력이 정수만이다)
float pot_f       = 0.0f;     // analogRead (필터 후)
float ang_deg     = 0.0f;     // 실제 조향각 (필터 후, 자르지 않는다)
float ang_prev    = 0.0f;     // 지난 주기의 실제 조향각 (D 항)
float str_i       = 0.0f;     // 조향 I 항 (PWM 단위)

unsigned long ctrl_last_us  = 0;
unsigned long telem_last_ms = 0;

// ===================== 주행 : 펄스값 세기 =====================
uint32_t edgesTotal() { return cnt_a + cnt_b; }   // 두 읽기 사이에 낀 에지는 다음 주기에 잡힌다

void countPulses() {
  uint32_t total = edgesTotal();
  meas = (long)(total - win_hist[win_idx]);   // 10 주기 전과의 차이 = 최근 100 ms 에지 수
  win_hist[win_idx] = total;
  win_idx = (uint8_t)((win_idx + 1) % WIN_TICKS);
}

// ===================== 주행 : 모터 (+ DIR LOW · − DIR HIGH) =====================
void setDrive(float v) {
  v = constrain(v, (float)-DRV_PWM_LIMIT, (float)DRV_PWM_LIMIT);
  drv_out_f = v;
  int pw = (int)lroundf(v);
  if (pw != 0) drv_dir_neg = (pw < 0);        // 0 이면 DIR 은 그대로 둔다 (실제 펄스 부호도 그대로)
  digitalWrite(PIN_DRV_DIR, drv_dir_neg ? HIGH : LOW);
  analogWrite(PIN_DRV_PWM, pw < 0 ? -pw : pw);
}

// ===================== 주행 : 하강 제한 (|PWM| 이 줄 때만, pulse_test 그대로) =====================
// 이번 주기에 지금 방향으로 낼 수 있는 |PWM| 의 아래 한계. 제한이 없거나 거의 서 있으면 0
float drvSlewFloor() {
  if (DRV_DN_RATE <= 0.0f) return 0.0f;
  float lo = fabsf(drv_out_f) - DRV_DN_RATE * DT_S;
  return lo > 0.0f ? lo : 0.0f;
}

// 원하는 PWM(+ / −) → 실제로 낼 PWM. 늘 때는 그대로, 줄 때는 아래 한계까지만.
// 방향이 바뀌면 옛 방향으로 0 까지 먼저 내려간 뒤에 새 방향으로 낸다.
float drvApplySlew(float want) {
  float lo = drvSlewFloor();
  if (lo <= 0.0f) return want;
  bool same_dir = (want > 0.0f && drv_out_f > 0.0f) || (want < 0.0f && drv_out_f < 0.0f);
  if (same_dir && fabsf(want) >= lo) return want;
  return drv_out_f > 0.0f ? lo : -lo;          // 옛 방향 그대로 아래 한계까지만 내린다
}

// ===================== 주행 : PID — 크기만 맞춘다 (0 ~ 상한에 목표 부호를 붙인다) =====================
float drivePid() {
  float lim  = (float)DRV_PWM_LIMIT;
  float err  = (float)labs(ref) - (float)meas;
  float rate = (float)(meas - meas_prev) / DT_S;   // 펄스/s

  // 아래 한계 = 하강 제한 (지금 목표 방향으로 돌고 있을 때만, 아니면 0)
  bool  same_dir = (drv_out_f > 0.0f && ref > 0) || (drv_out_f < 0.0f && ref < 0);
  float lo = same_dir ? drvSlewFloor() : 0.0f;
  if (lo > lim) lo = lim;

  float p = DRV_KP * err;
  float d = -DRV_KD * rate;
  float i_new = constrain(drv_i + DRV_KI * err * DT_S, 0.0f, lim);

  float u = p + i_new + d;
  // 와인드업 방지 : 출력이 상한(위) · 아래 한계에 걸린 쪽으로는 I 를 더 쌓지 않는다
  bool sat_same_side = (u > lim && err > 0) || (u < lo && err < 0);
  if (!sat_same_side) drv_i = i_new;
  else                u = p + drv_i + d;

  u = constrain(u, 0.0f, lim);                 // 아래 한계는 drvApplySlew 가 건다
  return ref < 0 ? -u : u;
}

// ===================== 조향 : 가변저항 =====================
// ADC → 각도 (자르지 않는다). 좌 · 우 끝 값 중 어느 쪽이 커도 맞는다.
float adcToAngle(float adc) {
  return ANG_LEFT + (adc - POT_LEFT_ADC) * (ANG_RIGHT - ANG_LEFT) / (float)(POT_RIGHT_ADC - POT_LEFT_ADC);
}

void readPot() {
  pot_f  += POT_ALPHA * ((float)analogRead(PIN_STEER_POT) - pot_f);
  ang_deg = adcToAngle(pot_f);
}

// ===================== 조향 : 모터 (+ 우 · − 좌) =====================
void setSteer(int pwm) {
  pwm = constrain(pwm, -STR_PWM_LIMIT, STR_PWM_LIMIT);
  if (pwm != 0) digitalWrite(PIN_STEER_DIR, pwm > 0 ? DIR_RIGHT : !DIR_RIGHT);   // 0 이면 DIR 은 그대로 둔다
  analogWrite(PIN_STEER_PWM, pwm > 0 ? pwm : -pwm);
}

// ===================== 조향 : PID =====================
int steerPid() {
  float lim  = (float)STR_PWM_LIMIT;
  float err  = (float)target_deg - ang_deg;   // + 면 우로 가야 한다
  float rate = (ang_deg - ang_prev) / DT_S;   // 도/s
  ang_prev = ang_deg;

  if (fabsf(err) <= STR_DEADBAND) return 0;   // I 는 그대로

  float p = STR_KP * err;
  float d = -STR_KD * rate;
  float i_new = constrain(str_i + STR_KI * err * DT_S, -lim, lim);

  float u = p + i_new + d;
  // 와인드업 방지 : 출력이 상한에 걸린 쪽으로는 I 를 더 쌓지 않는다
  bool sat_same_side = (u > lim && err > 0) || (u < -lim && err < 0);
  if (!sat_same_side) str_i = i_new;
  else                u = p + str_i + d;

  u = constrain(u, -lim, lim);
  return (int)lroundf(u);
}

// ===================== 10 ms 제어 : 계측 둘 → PID 둘 =====================
void controlStep() {
  countPulses();
  readPot();

  setDrive(drvApplySlew(drv_on ? drivePid() : 0.0f));   // 정지 중이면 하강 제한을 따라 0 까지 ('s' 는 바로 0)
  meas_prev = meas;

  if (str_on) setSteer(steerPid());
  else        { ang_prev = ang_deg; setSteer(0); }
}

// ===================== 출력 : "<목표 펄스> <목표 조향각> <실제 펄스> <실제 조향각>" =====================
// R4 코어는 printf 계열의 %f 를 못 쓴다 (newlib nano) — 실제 각도는 0.01° 정수로 바꿔 소수 둘째 자리까지 직접 찍는다.
int printCenti(char *buf, size_t n, float v) {
  long c = lroundf(v * 100.0f);
  long a = labs(c);
  return snprintf(buf, n, "%s%ld.%02ld", c < 0 ? "-" : "", a / 100, a % 100);
}

void printTelemetry() {
  char line[64];
  size_t n = 0;
  n += snprintf(line + n, sizeof(line) - n, "%ld %ld %ld ", ref, target_deg, drv_dir_neg ? -meas : meas);
  n += printCenti(line + n, sizeof(line) - n, ang_deg);
  n += snprintf(line + n, sizeof(line) - n, "\r\n");
  Serial.write((const uint8_t *)line, n);   // 한 번에 보낸다 (USB 전송 횟수를 줄인다)
}

// ===================== 명령 =====================
void stopDrive() {   // 목표 펄스 0 : 주행 PID 를 끄고, PWM 은 controlStep 이 하강 제한을 따라 0 까지 내린다
  drv_on = false; ref = 0; drv_i = 0.0f;
}

void stopAll() {     // 's' : 다음 주기를 기다리지 않고 둘 다 바로 선다 (주행도 하강 제한 없이)
  stopDrive(); setDrive(0.0f);
  str_on = false; str_i = 0.0f; setSteer(0);
}

void setRef(long v) {
  if (v == 0) { stopDrive(); return; }
  v = constrain(v, -REF_ABS_MAX, REF_ABS_MAX);
  bool flip = (ref != 0) && ((v < 0) != (ref < 0));
  if (!drv_on || flip) drv_i = 0.0f;          // 새로 켜거나 방향이 바뀌면 I 를 지운다
  ref    = v;
  drv_on = true;
}

void setAngle(long v) {
  target_deg = constrain(v, -ANG_ABS_MAX, ANG_ABS_MAX);
  if (!str_on) {                               // 켤 때만 I 를 지우고, D 가 튀지 않게 지난 각을 맞춘다
    str_i    = 0.0f;
    ang_prev = ang_deg;
  }
  str_on = true;
}

bool isBlank(char c) { return c == ' ' || c == '\t'; }

// "<정수> <정수>" 만 받는다 (앞뒤 · 사이 공백은 몇 개든). 다른 꼴이면 false — 소수점이 있으면 버린다
bool parseRefAngle(const char *s, long *ref_out, long *ang_out) {
  char *end;
  while (isBlank(*s)) s++;
  long r = strtol(s, &end, 10);
  if (end == s || !isBlank(*end)) return false;   // 숫자가 없거나 바로 뒤가 공백이 아니다 (35.5 · 35, 따위)
  s = end;
  while (isBlank(*s)) s++;
  long a = strtol(s, &end, 10);
  if (end == s) return false;                     // 숫자가 없다
  while (isBlank(*end)) end++;
  if (*end != '\0') return false;                 // 뒤에 다른 글자 (-20.5 · -20, 따위)
  *ref_out = r;
  *ang_out = a;
  return true;
}

// 알아듣지 못한 줄은 그냥 버린다 (응답 문구 없음)
void handleLine(const char *s) {
  while (isBlank(*s)) s++;
  if (tolower((unsigned char)*s) == 's') { stopAll(); return; }

  long r, a;
  if (!parseRefAngle(s, &r, &a)) return;
  setRef(r);
  setAngle(a);
}

// ===================== 명령 수신 (막히지 않게 한 글자씩) =====================
char          cmd_buf[32];
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
  pinMode(PIN_DRV_DIR, OUTPUT);
  digitalWrite(PIN_DRV_DIR, LOW);
  pinMode(PIN_DRV_PWM, OUTPUT);
  analogWrite(PIN_DRV_PWM, 0);

  pinMode(PIN_STEER_DIR, OUTPUT);
  digitalWrite(PIN_STEER_DIR, DIR_RIGHT);
  pinMode(PIN_STEER_PWM, OUTPUT);
  analogWrite(PIN_STEER_PWM, 0);

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

  pot_f    = (float)analogRead(PIN_STEER_POT);   // 필터를 지금 값에서 시작한다
  readPot();
  ang_prev = ang_deg;

  ctrl_last_us  = micros();
  telem_last_ms = millis();
}

void loop() {
  // 1) 10 ms 마다 계측 + 제어 (주행 · 조향) — 밀리면 몰아 돌리지 않고 다시 맞춘다
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
