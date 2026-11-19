// =====================================================================================
// joy_nolcd.ino ― 조이스틱 보드 '메가' (Arduino Mega 2560) : 스틱 · 버튼 → PC  (LCD 없는 판)
// =====================================================================================
// 2026-11-19. joy.ino 에서 LCD 와 관련된 것을 모두 뺐다 (사용자 지정 — LCD 를 바꾸기 전까지 쓴다).
// 뺀 것 : I2C LCD(Wire · LiquidCrystal_I2C) · LCD 내용 만들기 · PC → 메가 'U' 줄 해석 · SWB 타이머(LCD 에만 보이던 것).
// 그대로인 것 : 핀 · 250 Hz 출력 · "J," 줄 형식(필드 10개, on 포함) · 부팅 끝 "J,RESET" · SWA ON/OFF.
// → PC 쪽(broon/joy.py · tool/joy.py)은 고칠 것이 없다. ★필요한 라이브러리 없음★ (메가 코어만).
// 컴파일만 했다 (arduino:avr:mega). 업로드 · 실제 동작은 아직 확인 못 했다.
//
// ── 핀 (joy4 · joy.ino 그대로) ──────────────────────────────────────────────────────
//   L 스틱 가로 A15 · 세로 A14 · 버튼 D51      R 스틱 가로 A11 · 세로 A12 · 버튼 D50      SWA D53 · SWB D52
//   아날로그는 1023 = 위 · 왼쪽.  버튼은 풀업 — 안 누름 1, 누름 0.
//
// ── 메가 → PC (115200 bps) ────────────────────────────────────────────────────────
//   ★모든 줄이 "J," 로 시작한다★ — PC 는 보드 종류와 상관없이 "J," 줄을 보내는 아두이노를 조이스틱으로 잡는다.
//   부팅 끝에 "J,RESET" 한 줄 — PC 가 곧바로 영점을 다시 잡는 신호.
//   4 ms (250 Hz) 마다 "J,x1,y1,k1,x2,y2,k2,swa,swb,on" 한 줄.
//     x1 ~ y2 : analogRead 그대로.  k1 · k2 · swa · swb : 지금 누름 상태 그대로 (0 = 누름).
//     on : SWA 로 바꾼 ON/OFF 상태 (1 = ON, 0 = OFF).
//
// ── PC → 메가 ─────────────────────────────────────────────────────────────────────
//   PC 는 LCD 용 "U,…" 줄을 계속 보낸다 — 쓸 곳이 없으니 읽어서 버리기만 한다 (받는 버퍼가 차지 않게).
//
// ── 버튼 ────────────────────────────────────────────────────────────────────────
//   SWA : 누를 때마다 OFF ↔ ON (떨림 거르기 20 ms). 전원을 켜거나 리셋하면 OFF.
//   SWB : 보드 안에서 하는 일이 없다 — swb 필드로 누름 상태만 보낸다.
//   리셋 버튼 : 보드가 다시 켜지며 "J,RESET" → PC 가 영점을 다시 잡는다. ON/OFF 는 OFF.

// ===================== 핀 =====================
const int X1 = A15;  // Joystick 1 (L) 가로
const int Y1 = A14;  // Joystick 1 (L) 세로
const int K1 = 51;   // Joystick 1 (L) 버튼

const int X2 = A11;  // Joystick 2 (R) 가로
const int Y2 = A12;  // Joystick 2 (R) 세로
const int K2 = 50;   // Joystick 2 (R) 버튼

const int SWA = 53;  // ON/OFF
const int SWB = 52;  // (누름 상태만 보낸다)

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

// ===================== PC → 메가 : 읽어서 버린다 =====================
void drainPc() {
  while (Serial.available() > 0) Serial.read();
}

// ===================== 메가 → PC : "J,x1,y1,k1,x2,y2,k2,swa,swb,on" =====================
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
  Serial.write((const uint8_t *)line, n);          // 한 번에 보낸다 (33 바이트 이하 = 115200 에서 2.9 ms 이하)
}

// ===================== setup · loop =====================
unsigned long next_out_us = 0;

void setup() {
  Serial.begin(BAUD);
  pinMode(K1, INPUT_PULLUP);
  pinMode(K2, INPUT_PULLUP);
  pinMode(SWA, INPUT_PULLUP);
  pinMode(SWB, INPUT_PULLUP);

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
