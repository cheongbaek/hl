// =====================================================================================
// master_test_input.ino ― UNO R4 Minima 전 핀 입출력 시험 (시리얼 명령으로 핀 하나씩)
// =====================================================================================
// 모터 드라이버 · 센서 배선을 핀 하나씩 손으로 시험하는 스케치다 (2026-11-12).
// 카운터 클릭은 고려하지 않는다. 인터럽트를 쓰지 않는다.
// ★컴파일도 업로드도 해 보지 않았다 — 미검증★
//
// ── 명령 (시리얼 모니터에 한 줄에 하나. 대소문자 무관, 사이는 공백) ─────────────────────────
//   D<n> <값>   출력.  n = 0 ~ 13
//   A<n> <값>   출력.  n = 0 ~ 5   (A 핀은 디지털 출력만 된다)
//       값 = HIGH 또는 255  → HIGH
//       값 = LOW  또는 0    → LOW. 핀은 출력 그대로 0 V 로 붙잡는다 — 놓아 주지 않는다 (사용자 결정 2026-11-12)
//       값 = 1 ~ 254        → PWM. ★3 · 5 · 6 · 9 · 10 · 11 번만★ — 다른 핀이면 규칙에 없는 입력이다
//   D<n>        그 핀을 ★풀업 입력★으로 바꾸고 값을 0 · 1 로 계속 낸다. 출력 중이었으면 그 핀 출력은 끊긴다
//   A<n>        그 핀의 아날로그 값(10비트, 0 ~ 1023)을 계속 낸다. 출력 중이었으면 그 핀 출력은 끊긴다.
//               풀업은 쓰지 않는다 — 가변저항 값이 틀어지고, R4 의 analogRead 가 핀을 아날로그 모드로 바꾸며 풀업을 끈다
//   연속 출력은 숫자만, 시간 딜레이 없이 읽는 대로 낸다. 실제 줄 수는 USB 로 받아 가는 속도에 달렸다.
//
// ── 규칙 (사용자 지정 2026-11-12) ───────────────────────────────────────────────────
//   부팅하면 0 ~ 13 · A0 ~ A5 전부 입력(풀업 없음, 고임피던스)으로 두고 아무것도 내지 않은 채 명령을 기다린다.
//   유효한 출력 명령은 핀마다 쌓여 동시에 유지된다. 연속 출력은 한 번에 한 핀이다.
//     출력 명령      → 그 핀에 바로 낸다. 다른 핀 출력 · ★연속 출력은 그대로★
//                      (단 연속 출력 중인 바로 그 핀에 출력하면 그 연속 출력은 멈춘다 — 한 핀이 입력 · 출력을 같이 못 한다)
//     읽기 명령      → 하던 연속 출력을 멈추고 새 핀의 연속 출력을 시작한다. 핀 출력은 그대로
//                      (연속 출력 중인 바로 그 핀을 한 번 더 넣으면 연속 출력만 멈춘다 — 예) D5 보는 중에 D5)
//     규칙에 없는 줄 → ★모두 멈춘다★ — 연속 출력을 멈추고 0 ~ 13 · A0 ~ A5 전부를 부팅 때처럼 입력(풀업 없음)으로 놓아 준다
//                      예) 442, dsf, D20 HIGH, D12 140, A3 100, D5 HIGH X
//   예) D2 HIGH → D5 211 → D10 HIGH : 세 핀이 동시에 나간다
//       → D3      : 셋은 그대로, D3 값을 계속 낸다
//       → D6 HIGH : D6 도 나간다. D3 값은 계속 나온다
//       → D4      : D3 을 멈추고 D4 값. 출력은 그대로
//       → D4      : D4 연속 출력을 멈춘다 (같은 핀을 한 번 더). 출력은 그대로
//       → D5 · D5 HIGH : D5 값을 보다가, D5 연속 출력을 멈추고 D5 를 HIGH 로
//       → dsf     : 전부 멈춘다 (모든 핀 입력, 연속 출력 없음)
//   풀업은 디지털 읽기(D<n>)에만 쓴다. 부팅 · 모두 멈춤 때의 입력과 아날로그 읽기에는 쓰지 않는다.
//   빈 줄 · 공백뿐인 줄('\r\n' 의 남은 글자 등)은 아무 일도 없다 (연속 출력도 이어진다).
//   줄 끝 '\n' · '\r' 어느 쪽이든 받고, "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다. 응답 문구는 없다.
//
// ── R4 코어에서 확인한 것 (ArduinoCore-renesas 1.6.0 소스, 2026-11-12) ─────────────────
//   · PWM 하던 핀은 pinMode 를 다시 불러야 디지털이 된다 → HIGH · LOW · 읽기는 pinMode 부터 한다.
//   · 디지털로 바꿨던 핀에 analogWrite 를 하면 코어가 핀 설정이 바뀐 것을 알아보고 PWM 을 다시 켠다.
//   · pinMode 는 핀 설정 레지스터를 통째로 다시 쓴다 → 출력값도 LOW 로 시작한다.
//     그래서 이미 디지털 출력인 핀은 pinMode 를 건너뛰고 digitalWrite 만 한다 (DIR 을 HIGH ↔ LOW 로 바꿀 때 순간 LOW 가 끼지 않게).
//   · PWM 주파수는 코어 기본 약 490 Hz.
//   ⚠️ 3 · 11 번은 같은 타이머(GPT1)를 나눠 쓴다. 한쪽을 디지털로 바꿨다가 다시 PWM 으로 돌리면 코어가 타이머를 새로 켜서
//      다른 쪽 PWM 이 끊길 수 있다 (미검증) — 그러면 다른 쪽 값을 다시 넣는다.
//   · D13 에는 보드 LED 가 달려 있다. 코어가 부팅 때 출력 LOW 로 두는 것을 setup() 에서 입력으로 되돌린다.
//     LED 때문에 D13 의 풀업 입력값은 다른 핀과 다를 수 있다.

#include <ctype.h>

#define SERIAL_BAUD 115200   // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터도 115200 으로

const uint8_t       NUM_TEST_PINS = 20;    // D0 ~ D13 (0 ~ 13) + A0 ~ A5 (14 ~ 19)
const int           ADC_BITS      = 10;    // 아날로그 0 ~ 1023
const unsigned long CMD_IDLE_MS   = 100;   // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간

// ===================== 상태 =====================
// 핀마다 지금 상태 — 이미 디지털 출력이면 pinMode 를 건너뛴다
enum PinState : uint8_t { PS_IDLE, PS_OUT, PS_PWM, PS_IN_PULLUP, PS_ANALOG };
PinState pin_state[NUM_TEST_PINS];

enum StreamMode : uint8_t { STREAM_NONE, STREAM_DIGITAL, STREAM_ANALOG };
StreamMode stream_mode = STREAM_NONE;
uint8_t    stream_pin  = 0;

bool isPwmPin(uint8_t pin) {
  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
}

// ===================== 핀 동작 =====================
void writeDigital(uint8_t pin, PinStatus level) {
  if (pin_state[pin] != PS_OUT) {
    pinMode(pin, OUTPUT);   // PWM · 입력 · 아날로그에서 디지털 출력으로
    pin_state[pin] = PS_OUT;
  }
  digitalWrite(pin, level);
}

void writePwm(uint8_t pin, int value) {
  analogWrite(pin, value);  // 디지털이었으면 코어가 PWM 으로 되돌린다
  pin_state[pin] = PS_PWM;
}

void startDigitalStream(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  pin_state[pin] = PS_IN_PULLUP;
  stream_pin  = pin;
  stream_mode = STREAM_DIGITAL;
}

void startAnalogStream(uint8_t pin) {
  pin_state[pin] = PS_ANALOG;   // analogRead 가 핀을 아날로그 모드로 바꾼다 (그 핀 출력이 끊긴다)
  stream_pin  = pin;
  stream_mode = STREAM_ANALOG;
}

// 0 ~ 13 · A0 ~ A5 전부 입력(풀업 없음) — 아무 입출력도 하지 않는 상태 (부팅 때 · 모두 멈춤 때)
void releaseAllPins() {
  for (uint8_t p = 0; p < NUM_TEST_PINS; p++) {
    pinMode(p, INPUT);      // PWM · 출력 · 아날로그 · 풀업 모두 풀린다
    pin_state[p] = PS_IDLE;
  }
}

// 규칙에 없는 줄 — 연속 출력을 멈추고 모든 핀을 부팅 때처럼 놓아 준다
void stopAll() {
  stream_mode = STREAM_NONE;
  releaseAllPins();
}

// ===================== 명령 해석 =====================
// 숫자만 1 ~ max_digits 자리. 부호 · 다른 글자가 있으면 false
bool parseNumber(const char *s, uint8_t max_digits, int *out) {
  int v = 0;
  uint8_t k = 0;
  for (; s[k] != '\0'; k++) {
    if (k >= max_digits || s[k] < '0' || s[k] > '9') return false;
    v = v * 10 + (s[k] - '0');
  }
  if (k == 0) return false;
  *out = v;
  return true;
}

bool equalsIgnoreCase(const char *a, const char *b) {
  while (*a != '\0' && *b != '\0') {
    if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false;
    a++;
    b++;
  }
  return *a == *b;
}

// "D0" ~ "D13" → 0 ~ 13,  "A0" ~ "A5" → A0 ~ A5
bool parsePin(const char *s, uint8_t *pin, bool *is_analog) {
  int n;
  char c = (char)toupper((unsigned char)s[0]);
  if (!parseNumber(s + 1, 2, &n)) return false;
  if (c == 'D' && n <= 13) { *pin = (uint8_t)n;        *is_analog = false; return true; }
  if (c == 'A' && n <= 5)  { *pin = (uint8_t)(A0 + n); *is_analog = true;  return true; }
  return false;
}

// "HIGH" → 255,  "LOW" → 0,  "0" ~ "255"
bool parseLevel(const char *s, int *v) {
  if (equalsIgnoreCase(s, "HIGH")) { *v = 255; return true; }
  if (equalsIgnoreCase(s, "LOW"))  { *v = 0;   return true; }
  return parseNumber(s, 3, v) && *v <= 255;
}

// 유효한 명령이면 실행하고 true. 규칙에 없으면 아무것도 건드리지 않고 false
bool runCommand(char *tok[], uint8_t n) {
  if (n < 1 || n > 2) return false;

  uint8_t pin;
  bool    is_analog;
  if (!parsePin(tok[0], &pin, &is_analog)) return false;

  // 읽기 : "D9" · "A3"
  if (n == 1) {
    if (stream_mode != STREAM_NONE && stream_pin == pin) {   // 보고 있는 핀을 한 번 더 → 연속 출력만 멈춘다
      stream_mode = STREAM_NONE;
      return true;
    }
    if (is_analog) startAnalogStream(pin);
    else           startDigitalStream(pin);
    return true;
  }

  // 출력 : "D10 140" · "D12 HIGH" · "D6 0"
  int v;
  if (!parseLevel(tok[1], &v)) return false;
  if (v != 0 && v != 255 && !isPwmPin(pin)) return false;   // PWM 안 되는 핀에 1 ~ 254
  if (stream_mode != STREAM_NONE && stream_pin == pin) stream_mode = STREAM_NONE;   // 그 핀의 연속 출력만 멈춘다
  if (v == 255)    writeDigital(pin, HIGH);
  else if (v == 0) writeDigital(pin, LOW);
  else             writePwm(pin, v);
  return true;
}

// ===================== 명령 수신 (막히지 않게 한 글자씩) =====================
char          cmd_buf[24];
uint8_t       cmd_len     = 0;
bool          cmd_bad     = false;   // 줄이 버퍼보다 길었다
unsigned long cmd_last_ms = 0;

void applyCommand() {
  if (cmd_len == 0 && !cmd_bad) return;   // 빈 줄
  cmd_buf[cmd_len] = '\0';
  bool bad = cmd_bad;
  cmd_len = 0;
  cmd_bad = false;

  char   *tok[3];
  uint8_t n = 0;
  if (!bad) {
    for (char *p = strtok(cmd_buf, " \t"); p != NULL && n < 3; p = strtok(NULL, " \t")) tok[n++] = p;
    if (n == 0) return;                   // 공백뿐인 줄 = 빈 줄
  }

  if (bad || !runCommand(tok, n)) stopAll();   // ★규칙에 없는 줄이면 모두 멈춘다★
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
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_BITS);
  releaseAllPins();
}

void loop() {
  // 1) 연속 출력 — 시간 딜레이 없이
  if (stream_mode == STREAM_DIGITAL) {
    Serial.println(digitalRead(stream_pin) == HIGH ? 1 : 0);
  } else if (stream_mode == STREAM_ANALOG) {
    Serial.println(analogRead(stream_pin));
  }

  // 2) 명령
  readCommand();
}
