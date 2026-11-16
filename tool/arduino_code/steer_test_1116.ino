// =====================================================================================
// steer_test_1116.ino ― 조향 PID 튜닝 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 2026-11-16. 조향 모터 드라이버 하나(Cytron MD20A, PWM 10 · DIR 8)와 조향 가변저항(A0)만 쓴다.
// 레거시(legacy_white.ino, Mega Vr.1)에서 가져온 것은 ★가변저항 값뿐★이다 — 모터 드라이버가 다른 모델이라
// 핀 · 방향 · 제어 주기 · 필터 · PID 는 새로 짰다. PID 게인은 처음부터 다시 튜닝한다 (시리얼 명령으로 바로 바꾼다).
// 실차에서 돌려 봤다 (2026-11-16) — 방향 맞음, Kp 8 · Ki 0 / 10 시험 결과는 아래 'PID 기본값' 에 적었다.
//
// ── 각도 ───────────────────────────────────────────────────────────────────────────
//   −21° ~ +21°. 음수 = 좌, 양수 = 우. ★가변저항 값을 직선으로 이은 값이지 실제 바퀴 각이 아니다 — 나중에 실측해 고친다★
//     좌 끝 = ADC 954 (실측 2026-11-16)                        → −21°
//     우 끝 = ADC  52 (레거시 Vr.1 코드값 −460 + 512, ★실측 아님★) → +21°
//   → 가변저항 값은 ★좌로 갈수록 커진다★. 0° 는 ADC 503, 1° ≈ 21.5 카운트.
//   (레거시는 52 쪽에 LEFT(−19°) 이름을 붙였지만 실측으로는 큰 값 쪽이 좌다. 좌 끝 값도 레거시 935 → 실측 954.)
//   ADC 는 analogRead 그대로(0 ~ 1023, R4 Minima 기본 10 비트 · 기준 5 V). 가변저항은 5 V · GND 사이라고 본다.
//
// ── 방향 ─────────────────────────────────────────────────────────────────────────
//   드라이버 MD20A 데이터시트(Rev 1.0, 2019-02) 표 3 'PWM/DIR Input Truth Table' :
//     PWM High · DIR Low  → MA High · MB Low  (= 보드의 MA 시험 버튼과 같은 쪽, 데이터시트의 Forward)
//     PWM High · DIR High → MA Low  · MB High (= MB 버튼 쪽, Backward)
//     PWM Low             → MA · MB 둘 다 Low = 브레이크 (DIR 무관) — PWM 0 이면 조향이 브레이크로 잡힌다.
//   이 차는 MA 버튼 = 우, MB 버튼 = 좌 (사용자 확인 2026-11-16) → DIR LOW = 우 · DIR HIGH = 좌 → DIR_RIGHT = LOW.
//   출력 + = 우로 (ADC 가 작아지는 쪽), − = 좌로 (ADC 가 커지는 쪽). — 실차 확인 2026-11-16.
//   PID 를 켜기 전에 'o' 로 확인한다 :  o 60 → 바퀴가 우로 돌며 실제 조향각(둘째 숫자)이 커져야 맞다.
//     바퀴는 우로 도는데 각이 작아지면 가변저항 방향이 위와 반대다 — 이대로 PID 를 켜면 한쪽 끝까지 꺾인다.
//     o 60 에 바퀴가 좌로 돌면 모터 선(MA · MB)이나 DIR 배선(8번)이 버튼 확인 때와 다른 것이다.
//
// ── 끝 보호 없음 ──────────────────────────────────────────────────────────────────
//   기구 끝에 닿으면 모터가 헛돌 뿐 더 가지 않는다 (사용자 확인 2026-11-16) → 소프트웨어로 끝을 막지 않는다.
//   (레거시는 가변저항이 끝 값에 닿으면 모터를 양쪽 다 껐다.) 실제 조향각도 ±21° 로 자르지 않는다 — 끝 너머 값도 그대로 보인다.
//
// ── 제어 ─────────────────────────────────────────────────────────────────────────
//   10 ms(100 Hz)마다 : 가변저항 읽기 → 1차 저역 필터(15 Hz) → 각도 → PID → PWM.
//   P = Kp · 오차.  I = Ki · 오차 적분 (출력이 상한에 걸린 쪽으로는 더 쌓지 않는다, |I| ≤ PWM 상한).
//   D = −Kd · (측정 각의 변화율) — 목표를 바꿀 때 튀지 않게 오차가 아니라 측정값으로 미분한다.
//   |오차| ≤ 데드밴드면 모터 0(브레이크), I 는 그대로 둔다 (쌓지도 지우지도 않는다).
//   게인 단위 : Kp = PWM/도, Ki = PWM/(도·s), Kd = PWM/(도/s).
//   부팅 직후는 정지(PWM 0)다. 목표 각을 보내야 PID 가 켜진다. 명령 시한(무수신 정지)은 넣지 않았다 — 's' 로 세운다.
//
// ── 입력 (PC → 아두이노) — 한 줄에 명령 하나, 줄 끝 '\n' · '\r' 어느 쪽이든 ────────────────
//   10       숫자만 → 목표 각(도, ±21 로 자른다), PID 켜기          예) -7.5
//   p 5      Kp      i 2      Ki      d 0.3    Kd          (0 이상)
//   b 0.3    데드밴드(도)      m 150    PWM 상한 (0 ~ 255)
//   o 60     손 PWM (−255 ~ 255, PID 끄고 그대로 낸다 — 방향 확인용. PWM 상한 'm' 은 그대로 걸린다)
//   s        정지 (PWM 0, PID 끄기, I 지우기)
//   글자는 대소문자 무관, 글자와 숫자 사이 공백은 있어도 없어도 된다. 알아듣지 못한 줄은 말없이 버린다.
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다.
//   ★응답 문구가 없다★ — 바꾼 게인은 화면에 나오지 않으니 친 값을 적어 둔다.
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   50 ms 마다 "<명령 조향각> <실제 조향각> -21 21" 네 숫자만 한 줄 (도, 앞의 둘은 소수 둘째 자리). 문구는 없다.
//   뒤의 −21 · 21 은 늘 같은 값이다 — 시리얼 플로터의 세로 눈금이 ±21 로 고정돼 그래프가 출렁이지 않게 하는 직선 두 줄.
//   정지 · 손 PWM 중에도 나온다 (명령 각은 마지막 목표 그대로).
//
// ── 튜닝 순서 (제안) ──────────────────────────────────────────────────────────────
//   1) o 60 · o -60 으로 바퀴 방향 · 각도 방향 확인 → s.   2) Ki · Kd 0 으로 Kp 만 올려 목표를 0 ↔ ±10 으로 바꿔 가며 본다 (진동 직전의 절반쯤).
//   3) Kd 로 오버슈트를 줄인다.   4) 정상상태 오차가 남으면 Ki 를 조금씩.   5) 정한 값을 아래 기본값에 적는다.

#include <math.h>
#include <stdlib.h>
#include <ctype.h>

// ===================== 핀 =====================
const uint8_t PIN_STEER_PWM = 10;   // 조향 모터 드라이버(Cytron MD20A) PWM
const uint8_t PIN_STEER_DIR = 8;    // 조향 모터 드라이버(Cytron MD20A) DIR
const uint8_t PIN_STEER_POT = A0;   // 조향 가변저항

const uint8_t DIR_RIGHT = LOW;      // 우로 갈 때의 DIR 수준. MD20A 는 DIR LOW → MA 쪽, 이 차는 MA = 우 (머리말 '방향')

#define SERIAL_BAUD 115200          // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

// ===================== 가변저항 ↔ 각도 =====================
const int   POT_LEFT_ADC  = 954;    // 좌 끝 ADC (실측 2026-11-16)
const int   POT_RIGHT_ADC =  52;    // 우 끝 ADC (레거시 Vr.1 값 −460 + 512, ★실측 아님★)
const float ANG_LEFT      = -21.0f; // 좌 끝 각 (음수 = 좌)
const float ANG_RIGHT     =  21.0f; // 우 끝 각

// ===================== 제어 주기 · 필터 =====================
const unsigned long CTRL_US     = 10000;                   // 10 ms = 100 Hz
const float         DT_S        = CTRL_US * 1e-6f;
const float         POT_LPF_FC  = 15.0f;                   // 가변저항 1차 저역 필터 차단 주파수 (Hz)
const float         POT_ALPHA   = DT_S / (1.0f / (6.2831853f * POT_LPF_FC) + DT_S);
const unsigned long TELEM_MS    = 50;                      // 출력 간격
const unsigned long CMD_IDLE_MS = 100;                     // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간

// ===================== PID 기본값 — 튜닝한 값을 여기에 적는다 =====================
// 2026-11-16 실차 시험 :
//   Kp 8 단독     → 넘침 없이 다가가지만 목표 1 ~ 3° 앞에서 선다. 그때 PWM ≈ 10(가운데 근처) ~ 25(±18° 에서 바깥으로) —
//                   이보다 작으면 마찰 · 복원력을 못 이긴다. 감쇠는 충분해서 D 는 필요 없다.
//   Kp 8 · Ki 10 → 0 → ±5 는 빠른데 ±5 → 0 복귀가 느리다 (1 ~ 2 s, 1° 근처에서 계단처럼 멈칫).
//                   앞 동작에서 쌓인 I 가 데드밴드에서 그대로 남아 다음 반대 동작을 거꾸로 밀기 때문 (적분 이월).
//   → 위 데이터로 맞춘 모형 시뮬레이션에서 Kp 12 · Ki 20 · Kd 0 을 골랐다 (Ki 30 부터 넘침). ★실차 확인 전★
//     적분 이월은 게인만으로는 줄 뿐 남는다.
float Kp        = 12.0f;    // PWM/도
float Ki        = 20.0f;    // PWM/(도·s)
float Kd        = 0.0f;     // PWM/(도/s)
float deadband  = 0.3f;     // 도
int   pwm_limit = 150;      // 0 ~ 255 — 처음엔 낮게

// ===================== 상태 =====================
enum Mode : uint8_t { MODE_STOP, MODE_PID, MODE_OPEN };
Mode  mode       = MODE_STOP;
float target_deg = 0.0f;    // 명령 조향각
int   open_pwm   = 0;

float pot_f    = 0.0f;      // analogRead (필터 후)
float ang_deg  = 0.0f;      // 실제 조향각 (필터 후, 자르지 않는다)
float ang_prev = 0.0f;      // 지난 주기의 실제 조향각 (D 항)
float i_term   = 0.0f;      // I 항 (PWM 단위)
int   pwm_out  = 0;         // 실제로 낸 PWM (+ 우 · − 좌)

unsigned long ctrl_last_us  = 0;
unsigned long telem_last_ms = 0;

// ===================== 가변저항 =====================
// ADC → 각도 (자르지 않는다). 좌 · 우 끝 값 중 어느 쪽이 커도 맞는다.
float adcToAngle(float adc) {
  return ANG_LEFT + (adc - POT_LEFT_ADC) * (ANG_RIGHT - ANG_LEFT) / (float)(POT_RIGHT_ADC - POT_LEFT_ADC);
}

void readPot() {
  pot_f  += POT_ALPHA * ((float)analogRead(PIN_STEER_POT) - pot_f);
  ang_deg = adcToAngle(pot_f);
}

// ===================== 모터 (+ 우 · − 좌) =====================
void setSteer(int pwm) {
  pwm = constrain(pwm, -pwm_limit, pwm_limit);
  if (pwm != 0) digitalWrite(PIN_STEER_DIR, pwm > 0 ? DIR_RIGHT : !DIR_RIGHT);   // 0 이면 DIR 은 그대로 둔다
  analogWrite(PIN_STEER_PWM, pwm > 0 ? pwm : -pwm);
  pwm_out = pwm;
}

// ===================== PID =====================
int pidStep() {
  float err  = target_deg - ang_deg;          // + 면 우로 가야 한다
  float rate = (ang_deg - ang_prev) / DT_S;   // 도/s
  ang_prev = ang_deg;

  if (fabsf(err) <= deadband) return 0;       // I 는 그대로

  float p = Kp * err;
  float d = -Kd * rate;
  float i_new = i_term + Ki * err * DT_S;
  if (i_new >  pwm_limit) i_new =  pwm_limit;
  if (i_new < -pwm_limit) i_new = -pwm_limit;

  float u = p + i_new + d;
  // 와인드업 방지 : 출력이 상한에 걸린 쪽으로는 I 를 더 쌓지 않는다
  bool sat_same_side = (u > pwm_limit && err > 0) || (u < -pwm_limit && err < 0);
  if (!sat_same_side) i_term = i_new;
  else                u = p + i_term + d;

  if (u >  pwm_limit) u =  pwm_limit;
  if (u < -pwm_limit) u = -pwm_limit;
  return (int)lroundf(u);
}

void controlStep() {
  readPot();
  switch (mode) {
    case MODE_PID:  setSteer(pidStep()); break;
    case MODE_OPEN: ang_prev = ang_deg; setSteer(open_pwm); break;
    default:        ang_prev = ang_deg; setSteer(0); break;
  }
}

// ===================== 출력 : "<명령 조향각> <실제 조향각> -21 21" =====================
void printTelemetry() {
  Serial.print(target_deg, 2);
  Serial.print(' ');
  Serial.print(ang_deg, 2);
  Serial.print(' ');
  Serial.print(ANG_LEFT, 0);      // 플로터 눈금 고정용 직선 (−21)
  Serial.print(' ');
  Serial.println(ANG_RIGHT, 0);   // 플로터 눈금 고정용 직선 (21)
}

// ===================== 명령 수신 (막히지 않게 한 글자씩) =====================
char          cmd_buf[24];
uint8_t       cmd_len     = 0;
bool          cmd_bad     = false;   // 줄이 버퍼보다 길었다
unsigned long cmd_last_ms = 0;

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

void enterPid() {
  if (mode != MODE_PID) {   // 켤 때마다 I 를 지우고, D 가 튀지 않게 지난 각을 맞춘다
    i_term   = 0.0f;
    ang_prev = ang_deg;
  }
  mode = MODE_PID;
}

// 알아듣지 못한 줄은 그냥 버린다 (응답 문구 없음)
void handleLine(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  char c = (char)tolower((unsigned char)*s);
  float v;

  if (c == 's') {   // 다음 주기를 기다리지 않고 바로 선다
    mode = MODE_STOP; i_term = 0.0f; open_pwm = 0; setSteer(0);
    return;
  }

  if (isalpha((unsigned char)c)) {
    if (!parseFloat(s + 1, &v)) return;
    switch (c) {
      case 'p': if (v >= 0) Kp = v; break;
      case 'i': if (v >= 0) Ki = v; break;
      case 'd': if (v >= 0) Kd = v; break;
      case 'b': if (v >= 0) deadband = v; break;
      case 'm': if (v >= 0 && v <= 255) {
                  pwm_limit = (int)lroundf(v);
                  if (i_term >  pwm_limit) i_term =  pwm_limit;
                  if (i_term < -pwm_limit) i_term = -pwm_limit;
                }
                break;
      case 'o': if (v >= -255 && v <= 255) {
                  open_pwm = (int)lroundf(v); mode = MODE_OPEN; i_term = 0.0f;
                }
                break;
      default:  break;
    }
    return;
  }

  // 숫자만 → 목표 각
  if (!parseFloat(s, &v)) return;
  target_deg = constrain(v, ANG_LEFT, ANG_RIGHT);
  enterPid();
}

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
  pinMode(PIN_STEER_DIR, OUTPUT);
  digitalWrite(PIN_STEER_DIR, DIR_RIGHT);
  pinMode(PIN_STEER_PWM, OUTPUT);
  analogWrite(PIN_STEER_PWM, 0);

  Serial.begin(SERIAL_BAUD);

  pot_f    = (float)analogRead(PIN_STEER_POT);   // 필터를 지금 값에서 시작한다
  readPot();
  ang_prev = ang_deg;

  ctrl_last_us  = micros();
  telem_last_ms = millis();
}

void loop() {
  // 1) 10 ms 마다 제어 — 밀리면 몰아 돌리지 않고 다시 맞춘다
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
