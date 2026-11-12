// =====================================================================================
// pulsetest_interrupt_1112.ino ― 엔코더 A · B 상 파형 확인 + 모터 PWM 손 입력 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 펄스 · 센서 계측용 시험 스케치다 (2026-11-12). 지금은 모터 드라이버 1개와 엔코더 A · B(2 · 3번 핀)만 쓴다.
// ★컴파일도 업로드도 해 보지 않았다 — 미검증★ (이 노트북에 arduino-cli · R4 보드 코어가 없다).
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   "<A> <B>" 한 줄. A · B 는 2 · 3번 핀의 논리값이다 (HIGH = 1, LOW = 0).
//   ★주기 없이★ A 나 B 가 바뀔 때마다(인터럽트 CHANGE) 그 순간의 두 값을 한 줄씩 낸다 — 바뀐 것은 빠짐없이.
//   예) 정방향이면  0 0 → 1 0 → 1 1 → 0 1 → 0 0 …   (A 와 B 가 거의 동시에 바뀌면 같은 줄이 두 번 나올 수 있다)
//   숫자 말고는 아무것도 내지 않는다 — 시리얼 플로터에 A · B 두 줄 그래프로 그대로 뜬다.
//   ※ 바뀌는 것이 없으면(모터가 서 있으면) 아무것도 나오지 않는다. 플로터 가로축은 시간이 아니라 '변화 횟수' 다.
//   변화는 인터럽트가 링 버퍼(1024 개)에 쌓고 loop 가 꺼내 보낸다. 출력이 못 따라가 버퍼가 차면
//   그 뒤 변화는 버린다 — 플로터 파형이 중간에 건너뛰면 그 때문이다.
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
const uint8_t PIN_M2_PWM = 9;     // 모터 드라이버 2
const uint8_t PIN_M2_DIR = 7;
const uint8_t PIN_M3_PWM = 10;    // 모터 드라이버 3
const uint8_t PIN_M3_DIR = 8;
const uint8_t PIN_CNT_CS = 4;     // 카운터 클릭 칩 선택 (SPI 는 11 · 12 · 13 고정)
const uint8_t PIN_ANALOG = A0;    // 아날로그 입력

#define SERIAL_BAUD 115200        // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

const long          PWM_MAX     = 255;
const unsigned long CMD_IDLE_MS = 100;   // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간

// ===================== 엔코더 변화 링 버퍼 (인터럽트 → loop) =====================
const uint16_t EV_BUF_SIZE = 1024;       // 2 의 거듭제곱이어야 한다 (& 로 감는다)
volatile uint8_t  ev_buf[EV_BUF_SIZE];   // 비트1 = A, 비트0 = B
volatile uint16_t ev_head = 0;           // 인터럽트만 쓴다
volatile uint16_t ev_tail = 0;           // loop 만 쓴다

// A · B 어느 쪽이 바뀌어도 여기로 온다 — 그 순간의 두 값을 함께 쌓는다
void onEncoderEdge() {
  uint8_t s = (digitalRead(PIN_ENC_A) == HIGH ? 2 : 0) | (digitalRead(PIN_ENC_B) == HIGH ? 1 : 0);
  uint16_t next = (ev_head + 1) & (EV_BUF_SIZE - 1);
  if (next != ev_tail) {          // 꽉 찼으면 버린다
    ev_buf[ev_head] = s;
    ev_head = next;
  }
}

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
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoderEdge, CHANGE);
  // R4 코어의 attachInterrupt 는 핀 설정을 '인터럽트 + 입력' 으로 통째로 덮어써 풀업이 빠질 수 있다 —
  // 풀업을 다시 켠다 (pinMode 를 뒤에 부르면 이번엔 인터럽트가 꺼진다). 오픈컬렉터 엔코더면 풀업이 꼭 있어야 한다.
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_A].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_B].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
}

void loop() {
  // 1) 쌓인 변화를 한 바퀴에 한 줄씩 — 변화가 쏟아져도 명령 수신(정지 명령)이 밀리지 않게
  if (ev_tail != ev_head) {
    uint8_t s = ev_buf[ev_tail];
    ev_tail = (ev_tail + 1) & (EV_BUF_SIZE - 1);
    char line[5] = { (char)('0' + ((s >> 1) & 1)), ' ', (char)('0' + (s & 1)), '\r', '\n' };
    Serial.write((const uint8_t *)line, sizeof(line));   // 한 번에 보낸다 (USB 전송 횟수를 줄인다)
  }

  // 2) PWM 명령
  readCommand();
}
