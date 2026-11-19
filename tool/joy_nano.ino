// =====================================================================================
// joy_nano.ino ― 조이스틱 보드 '나노' (Arduino Nano, ATmega328P) : 스틱 · 버튼 → PC  (LCD 없음)
// =====================================================================================
// 2026-11-19. joy_nolcd.ino(메가)를 나노로 옮긴 판 — ★핀만 다르고 나머지는 똑같다★.
// 같은 것 : 115200 bps · 250 Hz 출력 · "J," 줄 형식(필드 10개, on 포함) · 부팅 끝 "J,RESET" · SWA ON/OFF ·
//           스틱 방향(analogRead 그대로 — 1023 = 위 · 왼쪽) · 버튼 풀업(안 누름 1, 누름 0).
// → PC 쪽(broon/joy.py)은 고칠 것이 없다. 메가 대신 꽂으면 된다. ★필요한 라이브러리 없음★ (AVR 코어만).
// 컴파일만 했다 (arduino:avr:nano). 업로드 · 실제 동작은 아직 확인 못 했다.
//
// ── 핀 ──────────────────────────────────────────────────────────────────────────
//   L 스틱 가로 A4 · 세로 A5 · 버튼 D8      R 스틱 가로 A6 · 세로 A7 · 버튼 D5      SWA D7 · SWB D6
//   A6 · A7 은 나노에서 아날로그 전용(디지털 · 풀업 없음) — 스틱 축이라 analogRead 만 쓰니 괜찮다.
//   A4 · A5 는 I2C(SDA · SCL) 핀이지만 LCD 가 없으니 아날로그 입력으로 쓴다.
//
// ── 나노 → PC (115200 bps) ────────────────────────────────────────────────────────
//   ★모든 줄이 "J," 로 시작한다★ — PC 는 보드 종류와 상관없이 "J," 줄을 보내는 아두이노를 조이스틱으로 잡는다.
//   부팅 끝에 "J,RESET" 한 줄 — PC 가 곧바로 영점을 다시 잡는 신호.
//   4 ms (250 Hz) 마다 "J,x1,y1,k1,x2,y2,k2,swa,swb,on" 한 줄.
//     x1 ~ y2 : analogRead 그대로.  k1 · k2 · swa · swb : 지금 누름 상태 그대로 (0 = 누름).
//     on : SWA 로 바꾼 ON/OFF 상태 (1 = ON, 0 = OFF).
//
// ── PC → 나노 ─────────────────────────────────────────────────────────────────────
//   PC 는 아무것도 보내지 않는다. 혹시 들어오면 읽어서 버리기만 한다 (받는 버퍼가 차지 않게).
//
// ── 버튼 ────────────────────────────────────────────────────────────────────────
//   SWA : 누를 때마다 OFF ↔ ON (떨림 거르기 20 ms). 전원을 켜거나 리셋하면 OFF.
//   SWB : 보드 안에서 하는 일이 없다 — swb 필드로 누름 상태만 보낸다.
//   리셋 버튼 : 보드가 다시 켜지며 "J,RESET" → PC 가 영점을 다시 잡는다. ON/OFF 는 OFF.
//
// ── 올리기 ──────────────────────────────────────────────────────────────────────
//   보드 "Arduino Nano". 클론(CH340)이 업로드에 실패하면 Processor 를 "ATmega328P (Old Bootloader)" 로.
//   PC 판별(broon/boards.py)은 USB VID Arduino(2341) · CH340(1A86) · arduino.org(2A03) · FTDI(0403) 포트를 본다.
//   우리 나노는 FTDI FT232R(0403:6001, /dev/ttyUSB*) — FTDI 포트는 "J," 줄을 보낼 때만 조이스틱으로 잡힌다 (2026-11-19).

// ===================== 핀 =====================
const int X1 = A4;   // Joystick 1 (L) 가로
const int Y1 = A5;   // Joystick 1 (L) 세로
const int K1 = 8;    // Joystick 1 (L) 버튼

const int X2 = A6;   // Joystick 2 (R) 가로  (아날로그 전용 핀)
const int Y2 = A7;   // Joystick 2 (R) 세로  (아날로그 전용 핀)
const int K2 = 5;    // Joystick 2 (R) 버튼

const int SWA = 7;   // ON/OFF
const int SWB = 6;   // (누름 상태만 보낸다)

// ===================== 주기 · 시간 =====================
const unsigned long BAUD        = 115200;
const unsigned long OUT_US      = 4000;    // 출력 주기 4 ms = 250 Hz
const unsigned long DEBOUNCE_MS = 20;      // 버튼 떨림 거르기

// ===================== SWA (떨림 거르기) → ON/OFF =====================
uint8_t       swa_stable = 1;      // 거른 상태 (1 안 누름, 0 누름)
uint8_t       swa_raw    = 1;
unsigned long swa_raw_ms = 0;      // raw 가 마지막으로 바뀐 시각
bool          on_state   = false;  // 켜면 OFF

void handleSwa(unsigned long now) {
  uint8_t r = digitalRead(SWA);
  if (r != swa_raw) { swa_raw = r; swa_raw_ms = now; }
  if (r != swa_stable && now - swa_raw_ms >= DEBOUNCE_MS) {
    swa_stable = r;
    if (swa_stable == 0) on_state = !on_state;     // 누르는 순간 바꾼다
  }
}

// ===================== PC → 나노 : 읽어서 버린다 =====================
void drainPc() {
  while (Serial.available() > 0) Serial.read();
}

// ===================== 나노 → PC : "J,x1,y1,k1,x2,y2,k2,swa,swb,on" =====================
void sendJoy() {
  int x1 = analogRead(X1);
  int y1 = analogRead(Y1);
  int x2 = analogRead(X2);
  int y2 = analogRead(Y2);

  int k1  = digitalRead(K1);
  int k2  = digitalRead(K2);
  int swa = digitalRead(SWA);
  int swb = digitalRead(SWB);

  char line[48];
  int n = snprintf(line, sizeof(line), "J,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                   x1, y1, k1, x2, y2, k2, swa, swb, on_state ? 1 : 0);
  Serial.write((const uint8_t *)line, n);          // 한 번에 보낸다 (33 바이트 이하 = 115200 에서 2.9 ms 이하, 송신 버퍼 64 B 안)
}

// ===================== setup · loop =====================
unsigned long next_out_us = 0;

void setup() {
  Serial.begin(BAUD);
  pinMode(K1, INPUT_PULLUP);
  pinMode(K2, INPUT_PULLUP);
  pinMode(SWA, INPUT_PULLUP);
  pinMode(SWB, INPUT_PULLUP);
  // A4 · A5 · A6 · A7 : analogRead 만 하므로 pinMode 를 두지 않는다 (A6 · A7 은 디지털 기능이 없다)

  Serial.println(F("J,RESET"));   // 리셋(버튼 · 전원 · 포트 열림) 때 한 번 → PC 가 영점을 다시 잡는 신호
  next_out_us = micros();
}

void loop() {
  // 4 ms 마다 : SWA → ON/OFF, 스틱 · 버튼 값을 PC 로.  밀리면 몰아 보내지 않고 다시 맞춘다
  unsigned long now_us = micros();
  if ((long)(now_us - next_out_us) >= 0) {
    next_out_us += OUT_US;
    if ((long)(now_us - next_out_us) >= 0) next_out_us = now_us + OUT_US;
    handleSwa(millis());
    sendJoy();
  }
  drainPc();
}
