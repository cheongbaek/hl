// =====================================================================================
// pulsetest_interrupt_1112.ino ― 엔코더 A · B 상 초당 펄스 수 + 모터 PWM 손 입력 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 펄스 · 센서 계측용 시험 스케치다 (2026-11-12). 지금은 모터 드라이버 1개와 엔코더 A · B(2 · 3번 핀)만 쓴다.
// 2026-11-16 : 출력을 'A · B 가 바뀔 때마다 두 핀 값' 에서 '1 초마다 상별 펄스 수' 로 바꿨다.
// 컴파일은 R4 코어(renesas_uno 1.6.0, FQBN arduino:renesas_uno:minima)로 통과했다 (2026-11-16).
// ★업로드 · 실제 동작은 아직 확인 못 했다★.
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   1 초에 한 줄, "<A> <B>". A · B 는 지난 1 초 동안 2 · 3번 핀에 들어온 펄스 수다 (상마다 따로 센다).
//   인터럽트를 CHANGE 로 걸어 오르내림(LOW ↔ HIGH)을 모두 센다 — 펄스 한 주기에 2 가 는다.
//   예) 240 238   ← 상마다 1 회전 N 펄스인 엔코더면 초당 회전수 ≈ 240 / (2N).
//   정상이면 A 와 B 가 거의 같다(±1). 크게 다르면 그 상의 배선 · 풀업 · 잡음을 의심한다.
//   방향은 따지지 않는다 — 정방향이든 역방향이든 양수로 센다.
//   모터가 서 있어도 "0 0" 이 1 초마다 나온다. 숫자 말고는 아무것도 내지 않는다 —
//   시리얼 플로터에 A · B 두 줄 그래프로 그대로 뜬다 (가로축 한 칸 = 1 초).
//   1 초는 millis() 로 잰다. loop 가 몇 ms 늦으면 그만큼 다음 창이 짧아져 길게 보면 맞는다 (한 창의 오차는 ±1 펄스 안팎).
//
// ── 입력 (PC → 아두이노) ───────────────────────────────────────────────────────────
//   정수 하나 + 줄 끝. −255..255 (넘으면 잘라 낸다).
//     양수 → DIR 0(LOW),  음수 → DIR 1(HIGH),  PWM = 절댓값.  0 이면 정지.
//   시리얼 모니터나 시리얼 플로터 아래의 입력 칸에서 보낸다. 줄 끝이 '\n' · '\r' 어느 쪽이든 받고,
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다. 숫자가 아닌 글자가 섞인 줄은 버린다 (응답 문구 없음).
//   부팅 직후는 PWM 0 이다. 명령 시한(무수신 정지)은 넣지 않았다 — 0 을 보내야 선다.
//
// ── 핀 (UNO R4 Minima) ────────────────────────────────────────────────────────────
//   PWM 되는 핀은 3 · 5 · 6 · 9 · 10 · 11 (보드의 ~ 표시) → 5번은 PWM 이 된다. 그대로 쓴다.
//   2 · 3 은 엔코더 시험용 (외부 인터럽트가 되는 핀). 카운터 클릭을 붙이면 엔코더는 그쪽으로 가고 2 · 3 은 빈다.
//   11 MOSI · 12 MISO · 13 SCK 는 카운터 클릭(SPI)에 남겨 둔다 → 남는 PWM 핀 9 · 10 을 모터 2 · 3 에 줬다.
//   0 · 1 은 Serial1 (R4 Minima 의 Serial 은 USB 라 비어 있지만 남겨 둔다). A1~A5 는 디지털 핀으로도 쓸 수 있다.
//   ★모터 2 · 3 · 카운터 클릭 · 아날로그 입력의 번호는 임의로 정한 자리표시다★ — 이 스케치는 아직 건드리지 않는다.

// ===================== 지금 쓰는 핀 =====================
const uint8_t PIN_M1_PWM = 5;     // 모터 드라이버 1 PWM
const uint8_t PIN_M1_DIR = 6;     // 모터 드라이버 1 DIR (양수 LOW · 음수 HIGH)
const uint8_t PIN_ENC_A  = 2;     // 엔코더 A 상
const uint8_t PIN_ENC_B  = 3;     // 엔코더 B 상

// ===================== 나중에 쓸 배선 — ★번호 임의★, 사용자가 고친다 =====================
// 업로드 때 컴파일 오류가 나서 통째로 주석 처리했다 (2026-11-16). 아직 이 스케치에서 쓰지 않는다 — 쓸 때 풀 것.
// 오류 원인은 PIN_ANALOG — R4 코어 cores/arduino/variant.h 에 같은 이름의 매크로 (10 << PIN_USE_POS) 가 있다.
// 풀 때는 이름을 바꿀 것 (예: PIN_POT). PIN_ 로 시작하는 다른 이름도 코어와 겹치지 않는지 확인한다.
// const uint8_t PIN_M2_PWM = 9;     // 모터 드라이버 2
// const uint8_t PIN_M2_DIR = 7;
// const uint8_t PIN_M3_PWM = 10;    // 모터 드라이버 3
// const uint8_t PIN_M3_DIR = 8;
// const uint8_t PIN_CNT_CS = 4;     // 카운터 클릭 칩 선택 (SPI 는 11 · 12 · 13 고정)
// const uint8_t PIN_ANALOG = A0;    // 아날로그 입력

#define SERIAL_BAUD 115200        // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

const long          PWM_MAX     = 255;
const unsigned long CMD_IDLE_MS = 100;   // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간
const unsigned long REPORT_MS   = 1000;  // 펄스 수를 내보내는 주기 (= 세는 창의 길이)

// ===================== 엔코더 펄스 카운터 (인터럽트 → loop) =====================
// 부팅 뒤 누적 펄스 수. 되돌리지 않고 loop 가 지난번 값과의 차이로 1 초치를 구한다 —
// 32 비트 읽기는 Cortex-M4 에서 한 번에 끝나므로 인터럽트를 끄지 않아도 되고, 넘쳐 돌아도 부호 없는 뺄셈이라 맞다.
volatile uint32_t cnt_a = 0;
volatile uint32_t cnt_b = 0;

void onEncoderA() { cnt_a++; }
void onEncoderB() { cnt_b++; }

// loop 쪽 기록 (setup 에서도 쓰므로 여기서 선언한다)
unsigned long report_last_ms = 0;   // 지금 창의 시작 시각
uint32_t      report_prev_a  = 0;   // 지난번 출력 때의 누적 수
uint32_t      report_prev_b  = 0;

// ===================== 모터 =====================
void setMotor(long v) {
  v = constrain(v, -PWM_MAX, PWM_MAX);
  digitalWrite(PIN_M1_DIR, v < 0 ? HIGH : LOW);
  analogWrite(PIN_M1_PWM, (int)(v < 0 ? -v : v));
}

// ===================== 명령 수신 (막히지 않게 한 글자씩) =====================
char          cmd_buf[16];
uint8_t       cmd_len     = 0;
bool          cmd_bad     = false;   // 줄이 버퍼보다 길었다
unsigned long cmd_last_ms = 0;

// 앞뒤 공백 · 부호(+/−)만 허락하는 정수 읽기. 다른 글자가 섞이면 false
bool parseValue(const char *s, long *out) {
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

void applyCommand() {
  if (cmd_len == 0 && !cmd_bad) return;   // "\r\n" 의 두 번째 글자 같은 빈 줄
  cmd_buf[cmd_len] = '\0';
  long v;
  bool ok = !cmd_bad && parseValue(cmd_buf, &v);
  cmd_len = 0;
  cmd_bad = false;
  if (ok) setMotor(v);
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

  report_last_ms = millis();   // 첫 창은 여기서부터
}

// ===================== 1 초마다 펄스 수 출력 =====================
void reportPulses() {
  unsigned long now = millis();
  if (now - report_last_ms < REPORT_MS) return;
  report_last_ms += REPORT_MS;                                        // 밀리지 않게 1 초씩 더한다
  if (now - report_last_ms >= REPORT_MS) report_last_ms = now;        // 1 초 넘게 늦었으면 몰아 내지 말고 다시 맞춘다

  uint32_t a = cnt_a;
  uint32_t b = cnt_b;
  char line[32];
  int n = snprintf(line, sizeof(line), "%lu %lu\r\n",
                   (unsigned long)(a - report_prev_a), (unsigned long)(b - report_prev_b));
  report_prev_a = a;
  report_prev_b = b;
  Serial.write((const uint8_t *)line, n);   // 한 번에 보낸다 (USB 전송 횟수를 줄인다)
}

void loop() {
  reportPulses();   // 1) 1 초마다 A · B 펄스 수
  readCommand();    // 2) PWM 명령
}
