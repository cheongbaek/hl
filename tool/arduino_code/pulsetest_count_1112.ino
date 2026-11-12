// =====================================================================================
// pulsetest_count_1112.ino ― 카운터 클릭(LS7366R)으로 엔코더 펄스 계측 + 모터 PWM 손 입력 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// pulsetest_interrupt_1112.ino 를 바탕으로 엔코더 입력만 2 · 3번 핀 → 카운터 클릭으로 바꿨다 (2026-11-12).
// 모터 드라이버 1개 · 입력 양식은 그쪽과 같다. 2 · 3번 핀은 이 스케치에서 쓰지 않는다.
// ★컴파일도 업로드도 해 보지 않았다 — 미검증★ (이 노트북에 arduino-cli · R4 보드 코어가 없다).
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   SAMPLE_PERIOD_MS(기본 100 ms)마다 한 줄 : 그동안 들어온 펄스 수 (정수, 부호 = 방향). 숫자만 낸다 —
//   시리얼 플로터에 한 줄 그래프로 뜬다. 모터가 서 있으면 0 이 계속 나온다.
//   펄스 수는 QUAD_MODE 배로 센 값이다. 기본 x4 = A · B 의 상승 · 하강을 모두 센다.
//   x1 이면 A 상 한 주기에 1 이다 — 엔코더 사양의 PPR 과 같은 단위.
//   어느 방향이 + 인지는 A · B 배선에 따라 다르다 — 돌려 보고 확인한다.
//   세는 일은 LS7366R 이 한다. 아두이노는 주기마다 32비트 카운터를 읽어 앞 값과의 차를 낸다.
//   ★카운터를 지우지 않는다★ — 읽는 사이에 들어온 펄스도 다음 주기에 빠짐없이 들어간다. 32비트가 넘쳐 돌아도 차는 맞다.
//   주기는 delay() 없이 micros() 로 잰다. 다음 시각을 '앞 시각 + 주기' 로 잡아 주기가 밀리지 않는다
//   (한 주기 넘게 밀리면 그때부터 다시 잡는다 — 그 줄은 더 긴 시간의 펄스 수다).
//   ※ 카운터 클릭이 응답하지 않으면(배선 · CS 번호가 틀리면) 0 만 계속 나온다 — 그럴 때 먼저 배선을 본다.
//
// ── 입력 (PC → 아두이노) — pulsetest_interrupt_1112.ino 와 같다 ─────────────────────
//   정수 하나 + 줄 끝. −255..255 (넘으면 잘라 낸다).
//     양수 → DIR 0(LOW),  음수 → DIR 1(HIGH),  PWM = 절댓값.  0 이면 정지.
//   시리얼 모니터나 시리얼 플로터 아래의 입력 칸에서 보낸다. 줄 끝이 '\n' · '\r' 어느 쪽이든 받고,
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다. 숫자가 아닌 글자가 섞인 줄은 버린다 (응답 문구 없음).
//   부팅 직후는 PWM 0 이다. 명령 시한(무수신 정지)은 넣지 않았다 — 0 을 보내야 선다.
//
// ── 카운터 클릭 (LS7366R, SPI) ─────────────────────────────────────────────────────
//   엔코더 A · B 는 카운터 클릭의 입력 단자로 들어간다. 아두이노와는 SPI : 11 MOSI · 12 MISO · 13 SCK (고정) + CS.
//   설정 : 쿼드러처 QUAD_MODE 배 · 자유 계수(free-running) · 인덱스 안 씀 · 4바이트 카운터. SPI 모드 0, 1 MHz.
//   EN(카운트 허가) 핀을 배선했으면 HIGH 로 둔다. 배선하지 않았으면 PIN_CNT_EN 을 −1 로.
//   INT(플래그) 핀은 쓰지 않는다. 보드에 3.3 V / 5 V 선택 점퍼가 있다면 5 V 로 (R4 Minima 는 5 V 논리).
//   ※ D13 이 SCK 라 보드 LED(D13)가 SPI 통신 때 깜빡일 수 있다.
//
// ── 핀 (UNO R4 Minima) ────────────────────────────────────────────────────────────
//   모터 1 은 PWM 5 · DIR 6 (5번은 PWM 이 되는 핀이다 — 3 · 5 · 6 · 9 · 10 · 11).
//   ★모터 2 · 3 · 카운터 클릭 CS · EN · 아날로그 입력의 번호는 임의로 정한 자리표시다★ — 실제 배선에 맞게 고친다.
//   모터 2 · 3 · 아날로그 입력은 이 스케치에서 아직 건드리지 않는다.

#include <SPI.h>

// ===================== 파라미터 =====================
const unsigned long SAMPLE_PERIOD_MS = 100;  // ★펄스를 모으고 내보내는 주기 [ms]★
const uint8_t       QUAD_MODE        = 4;    // 쿼드러처 배수 : 1 · 2 · 4 (그 밖의 값은 4 로 본다)

// ===================== 지금 쓰는 핀 =====================
const uint8_t PIN_M1_PWM = 5;     // 모터 드라이버 1 PWM
const uint8_t PIN_M1_DIR = 6;     // 모터 드라이버 1 DIR (양수 LOW · 음수 HIGH)
const uint8_t PIN_CNT_CS = 4;     // 카운터 클릭 칩 선택 — ★번호 임의★ (SPI 는 11 · 12 · 13 고정)
const int     PIN_CNT_EN = A1;    // 카운터 클릭 EN — ★번호 임의★. 배선하지 않았으면 −1

// ===================== 나중에 쓸 배선 — ★번호 임의★, 사용자가 고친다 =====================
const uint8_t PIN_M2_PWM = 9;     // 모터 드라이버 2
const uint8_t PIN_M2_DIR = 7;
const uint8_t PIN_M3_PWM = 10;    // 모터 드라이버 3
const uint8_t PIN_M3_DIR = 8;
const uint8_t PIN_ANALOG = A0;    // 아날로그 입력

#define SERIAL_BAUD 115200        // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

const long          PWM_MAX          = 255;
const unsigned long CMD_IDLE_MS      = 100;   // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간
const unsigned long SAMPLE_PERIOD_US = SAMPLE_PERIOD_MS * 1000UL;

// ===================== LS7366R =====================
// 명령 바이트 = 동작(B7 B6) | 레지스터(B5 B4 B3)
const uint8_t LS_CLR  = 0x00, LS_RD   = 0x40, LS_WR   = 0x80, LS_LOAD = 0xC0;
const uint8_t LS_MDR0 = 0x08, LS_MDR1 = 0x10, LS_CNTR = 0x20;

// MDR0 : B1 B0 = 쿼드러처 (01 x1 · 10 x2 · 11 x4), B3 B2 = 00 자유 계수, B5 B4 = 00 인덱스 안 씀, B6 = 0 비동기 인덱스, B7 = 0 필터 분주 1
const uint8_t LS_MDR0_VAL = (QUAD_MODE == 1) ? 0x01 : (QUAD_MODE == 2) ? 0x02 : 0x03;
// MDR1 : B1 B0 = 00 4바이트 카운터, B2 = 0 계수 허가, B4~B7 = 0 플래그 안 씀
const uint8_t LS_MDR1_VAL = 0x00;

const SPISettings LS_SPI(1000000, MSBFIRST, SPI_MODE0);

void lsCommand(uint8_t op) {
  SPI.beginTransaction(LS_SPI);
  digitalWrite(PIN_CNT_CS, LOW);
  SPI.transfer(op);
  digitalWrite(PIN_CNT_CS, HIGH);
  SPI.endTransaction();
}

void lsWrite(uint8_t op, uint8_t data) {
  SPI.beginTransaction(LS_SPI);
  digitalWrite(PIN_CNT_CS, LOW);
  SPI.transfer(op);
  SPI.transfer(data);
  digitalWrite(PIN_CNT_CS, HIGH);
  SPI.endTransaction();
}

// CNTR 을 읽는다 (LS7366R 이 CNTR → OTR 로 옮겨 놓고 내보내므로 읽는 순간의 값이 한 번에 잡힌다). 상위 바이트부터
uint32_t lsReadCounter() {
  SPI.beginTransaction(LS_SPI);
  digitalWrite(PIN_CNT_CS, LOW);
  SPI.transfer(LS_RD | LS_CNTR);
  uint32_t v = 0;
  for (uint8_t i = 0; i < 4; i++) v = (v << 8) | SPI.transfer(0x00);
  digitalWrite(PIN_CNT_CS, HIGH);
  SPI.endTransaction();
  return v;
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
uint32_t      cnt_prev = 0;    // 앞 주기에 읽은 카운터
unsigned long next_us  = 0;    // 다음 출력 시각 [micros]

void setup() {
  pinMode(PIN_M1_DIR, OUTPUT);
  digitalWrite(PIN_M1_DIR, LOW);
  pinMode(PIN_M1_PWM, OUTPUT);
  analogWrite(PIN_M1_PWM, 0);

  Serial.begin(SERIAL_BAUD);

  // 카운터 클릭
  pinMode(PIN_CNT_CS, OUTPUT);
  digitalWrite(PIN_CNT_CS, HIGH);
  if (PIN_CNT_EN >= 0) {
    pinMode(PIN_CNT_EN, OUTPUT);
    digitalWrite(PIN_CNT_EN, HIGH);
  }
  SPI.begin();
  lsWrite(LS_WR | LS_MDR0, LS_MDR0_VAL);
  lsWrite(LS_WR | LS_MDR1, LS_MDR1_VAL);
  lsCommand(LS_CLR | LS_CNTR);

  cnt_prev = lsReadCounter();
  next_us  = micros() + SAMPLE_PERIOD_US;
}

void loop() {
  // 1) 주기마다 펄스 수 — delay() 없이 시각만 본다
  unsigned long now = micros();
  if ((long)(now - next_us) >= 0) {
    next_us += SAMPLE_PERIOD_US;
    if ((long)(now - next_us) >= 0) next_us = now + SAMPLE_PERIOD_US;   // 한 주기 넘게 밀렸으면 다시 잡는다

    uint32_t cnt = lsReadCounter();
    long d = (long)(int32_t)(cnt - cnt_prev);   // 넘쳐 돌아도 맞는 차
    cnt_prev = cnt;
    Serial.println(d);
  }

  // 2) PWM 명령
  readCommand();
}
