// =====================================================================================
// joy.ino ― 조이스틱 보드 '메가' (Arduino Mega 2560) : 스틱 · 버튼 → PC,  LCD 16×2 (타이머 · ON/OFF · REF · 조향각)
// =====================================================================================
// 2026-11-18. mad PC 의 joy4.ino (2026-09-15, 51줄) 를 가져와 고쳤다. PC 쪽 짝은 tool/joy.py, 차량 '우노' 는 dualtest_1118.ino.
// joy4 에서 바꾼 것 : 출력 주기 250 Hz 고정 · SWA = ON/OFF · SWB = 타이머 (둘 다 보드 안에서 돈다) · I2C LCD · PC → 메가 'U' 줄.
// 컴파일 · 업로드 · 실제 동작은 아직 확인 못 했다.
//
// ── 필요한 라이브러리 ─────────────────────────────────────────────────────────────
//   "LiquidCrystal I2C" (Frank de Brabander) — IDE 라이브러리 매니저에서 설치.  Wire 는 메가 코어에 들어 있다.
//
// ── 핀 (joy4 그대로) ────────────────────────────────────────────────────────────────
//   L 스틱 가로 A15 · 세로 A14 · 버튼 D51      R 스틱 가로 A11 · 세로 A12 · 버튼 D50      SWA D53 · SWB D52
//   아날로그는 1023 = 위 · 왼쪽.  버튼은 풀업 — 안 누름 1, 누름 0.
//   LCD : I2C 16×2 (뒤에 PCF8574 4핀 기판) — SDA 20 · SCL 21 · 5 V · GND.  (SPI 가 아니라 I2C 다)
//         주소 0x27 · 0x3F 중 응답하는 쪽을 쓴다. 둘 다 없으면 LCD 없이 나머지만 돈다.
//
// ── 메가 → PC (115200 bps) ────────────────────────────────────────────────────────
//   ★모든 줄이 "J," 로 시작한다★ — joy.py 는 보드 종류와 상관없이 "J," 줄을 보내는 아두이노를 조이스틱으로,
//   나머지 아두이노를 구동계(우노)로 잡는다. 그러니 이 접두어는 바꾸지 않는다.
//   부팅 끝에 "J,RESET" 한 줄 — PC 가 영점을 다시 잡는 신호 (joy4 와 같다).
//   4 ms (250 Hz) 마다 "J,x1,y1,k1,x2,y2,k2,swa,swb,on" 한 줄 — joy4 의 9 필드 뒤에 on 하나를 붙였다.
//     x1 ~ y2 : analogRead 그대로.  k1 · k2 · swa · swb : 지금 누름 상태 그대로 (0 = 누름).
//     on : SWA 로 바꾼 ON/OFF 상태 (1 = ON, 0 = OFF).  PC 는 OFF 면 우노에 "0 0" 을 보낸다.
//
// ── PC → 메가 ─────────────────────────────────────────────────────────────────────
//   "U,<목표 펄스>,<조향각>" — 우노가 낸 값을 PC 가 넘겨준다 (정수). LCD 둘째 줄에만 쓴다.
//   0.5 s 넘게 안 오면 (PC · 우노가 없으면) LCD 에 "---" 를 띄운다.
//   타이머 · ON/OFF 는 시리얼과 상관없이 보드 안에서만 돈다 — PC 가 없어도 SWA · SWB 가 듣는다.
//
// ── 버튼 (떨림 거르기 20 ms) ──────────────────────────────────────────────────────
//   SWA : 누를 때마다 OFF ↔ ON. 전원을 켜거나 리셋하면 OFF.
//   SWB : 짧게 (1 초 안에 뗌) — 멈춰 있으면 시작 · 재개, 도는 중이면 일시정지.  ★뗄 때★ 반영한다 (길게 누름과 가르려고).
//         길게 (1 초 이상) — 도는 중 · 일시정지 상관없이 00:00:00 으로 되돌리고 멈춘다. 누르고 있는 동안 1 초가 되는 순간 반영.
//   리셋 버튼 : 보드가 다시 켜지며 "J,RESET" → PC 가 3 초 뒤 그때 스틱 위치를 영점으로 잡는다 (joy_gold 와 같다). 타이머는 0, ON/OFF 는 OFF.
//
// ── LCD ─────────────────────────────────────────────────────────────────────────
//   "00:00:00     OFF"     시:분:초 (99:59:59 에서 멈춘다) · ON/OFF
//   "REF:+35 -12 00km"     목표 펄스 (+ 전진 · − 후진) · 조향각 (+ 우 · − 좌) · 속도계 (지금은 00 고정)
//   I2C LCD 는 한 글자 쓰는 데 약 1.5 ms 걸린다 (100 kHz). 한꺼번에 다시 쓰면 250 Hz 출력이 밀리므로
//   바뀐 글자만, 다음 출력까지 1.8 ms 이상 남았을 때 한 번에 하나씩 쓴다 (초 단위 갱신에는 충분하다).

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== 핀 (joy4 그대로) =====================
const int X1 = A15;  // Joystick 1 (L) 가로
const int Y1 = A14;  // Joystick 1 (L) 세로
const int K1 = 51;   // Joystick 1 (L) 버튼

const int X2 = A11;  // Joystick 2 (R) 가로
const int Y2 = A12;  // Joystick 2 (R) 세로
const int K2 = 50;   // Joystick 2 (R) 버튼

const int SWA = 53;  // ON/OFF
const int SWB = 52;  // 타이머

// ===================== 주기 · 시간 =====================
const unsigned long BAUD        = 115200;
const unsigned long OUT_US      = 4000;    // 출력 주기 4 ms = 250 Hz
const unsigned long DEBOUNCE_MS = 20;      // 버튼 떨림 거르기
const unsigned long LONG_MS     = 1000;    // SWB 길게 누름
const unsigned long PC_STALE_MS = 500;     // 이만큼 'U' 줄이 없으면 LCD 에 "---"
const unsigned long LCD_COMPOSE_MS = 50;   // LCD 내용을 다시 만드는 간격
const long          LCD_GAP_US  = 1800;    // 다음 출력까지 이만큼 남았을 때만 LCD 에 한 번 쓴다
const uint16_t      I2C_TIMEOUT_US = 2000; // I2C 가 막혀도 보드가 서지 않게 (LCD 선이 빠졌을 때)

// ===================== LCD =====================
LiquidCrystal_I2C lcd27(0x27, 16, 2);
LiquidCrystal_I2C lcd3f(0x3F, 16, 2);
LiquidCrystal_I2C *lcd = NULL;             // 찾은 쪽. 없으면 NULL

char    lcd_want[2][17];                   // 띄우고 싶은 내용
char    lcd_have[2][17];                   // LCD 에 지금 써 있는 내용
bool    lcd_cur_ok  = false;               // LCD 커서 위치를 아는가
uint8_t lcd_cur_row = 0, lcd_cur_col = 0;

// ===================== 버튼 (떨림 거르기) =====================
struct Button {
  uint8_t       pin;
  uint8_t       stable;      // 거른 상태 (1 안 누름, 0 누름)
  uint8_t       raw_last;
  unsigned long raw_ms;      // raw 가 마지막으로 바뀐 시각
};
Button btn_a = {SWA, 1, 1, 0};
Button btn_b = {SWB, 1, 1, 0};

// 거른 상태가 바뀌었으면 true
bool buttonUpdate(Button &b, unsigned long now) {
  uint8_t r = digitalRead(b.pin);
  if (r != b.raw_last) { b.raw_last = r; b.raw_ms = now; }
  if (r != b.stable && now - b.raw_ms >= DEBOUNCE_MS) { b.stable = r; return true; }
  return false;
}

// ===================== ON/OFF · 타이머 =====================
bool on_state = false;                     // 켜면 OFF

enum TimerState : uint8_t { T_STOP, T_RUN, T_PAUSE };
TimerState    t_state    = T_STOP;
unsigned long t_acc_ms   = 0;              // 지난 구간까지 쌓인 시간
unsigned long t_start_ms = 0;              // 지금 구간 시작 시각 (T_RUN 일 때)
unsigned long swb_down_ms = 0;
bool          swb_long_done = false;       // 이번 누름에서 길게 누름을 이미 반영했다

unsigned long timerMs(unsigned long now) {
  return t_acc_ms + (t_state == T_RUN ? now - t_start_ms : 0);
}

void handleButtons(unsigned long now) {
  if (buttonUpdate(btn_a, now) && btn_a.stable == 0) on_state = !on_state;   // 누르는 순간 바꾼다

  if (buttonUpdate(btn_b, now)) {
    if (btn_b.stable == 0) {                       // 눌렀다
      swb_down_ms   = now;
      swb_long_done = false;
    } else if (!swb_long_done) {                   // 짧게 눌렀다 뗐다
      if (t_state == T_RUN) { t_acc_ms = timerMs(now); t_state = T_PAUSE; }
      else                  { t_start_ms = now;        t_state = T_RUN;   }
    }
  }
  if (btn_b.stable == 0 && !swb_long_done && now - swb_down_ms >= LONG_MS) {   // 길게 — 0 으로
    t_state = T_STOP; t_acc_ms = 0; swb_long_done = true;
  }
}

// ===================== PC → 메가 : "U,<목표 펄스>,<조향각>" =====================
int           pc_ref = 0, pc_ang = 0;
unsigned long pc_ms  = 0;
bool          pc_seen = false;

char    pc_buf[24];
uint8_t pc_len = 0;
bool    pc_bad = false;

// 정수 하나를 읽고 끝 위치를 돌려준다. 숫자가 없으면 NULL
const char *readInt(const char *s, long *out) {
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s) return NULL;
  *out = v;
  return end;
}

void parsePcLine(unsigned long now) {
  if (pc_buf[0] != 'U' || pc_buf[1] != ',') return;
  long r, a;
  const char *p = readInt(pc_buf + 2, &r);
  if (!p || *p != ',') return;
  p = readInt(p + 1, &a);
  if (!p || *p != '\0') return;
  pc_ref  = (int)constrain(r, -99L, 99L);
  pc_ang  = (int)constrain(a, -99L, 99L);
  pc_ms   = now;
  pc_seen = true;
}

void readPc() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (pc_len > 0 && !pc_bad) { pc_buf[pc_len] = '\0'; parsePcLine(millis()); }
      pc_len = 0; pc_bad = false;
    } else if (pc_len < sizeof(pc_buf) - 1) {
      pc_buf[pc_len++] = c;
    } else {
      pc_bad = true;                               // 너무 긴 줄은 버린다
    }
  }
}

// ===================== LCD 내용 만들기 =====================
// "+35" 처럼 부호 + 두 자리 (0 은 +00)
void signed2(char *dst, int v) {
  dst[0] = v < 0 ? '-' : '+';
  int a = v < 0 ? -v : v;
  if (a > 99) a = 99;
  dst[1] = (char)('0' + a / 10);
  dst[2] = (char)('0' + a % 10);
}

void lcdCompose(unsigned long now) {
  unsigned long s = timerMs(now) / 1000UL;
  if (s > 99UL * 3600UL + 59UL * 60UL + 59UL) s = 99UL * 3600UL + 59UL * 60UL + 59UL;   // 99:59:59 에서 멈춘다
  unsigned int h = (unsigned int)(s / 3600UL), m = (unsigned int)((s / 60UL) % 60UL), sec = (unsigned int)(s % 60UL);
  snprintf(lcd_want[0], 17, "%02u:%02u:%02u     %s", h, m, sec, on_state ? " ON" : "OFF");

  if (pc_seen && now - pc_ms <= PC_STALE_MS) {
    memcpy(lcd_want[1], "REF:+00 +00 00km", 17);
    signed2(&lcd_want[1][4], pc_ref);
    signed2(&lcd_want[1][8], pc_ang);
  } else {
    memcpy(lcd_want[1], "REF:--- --- 00km", 17);
  }
}

// 바뀐 글자 하나 (또는 커서 옮기기 하나)만 쓴다 — I2C 한 번에 약 1.5 ms
void lcdStep() {
  if (lcd == NULL) return;
  for (uint8_t i = 0; i < 32; i++) {
    uint8_t r = i / 16, c = i % 16;
    if (lcd_want[r][c] == lcd_have[r][c]) continue;
    if (!lcd_cur_ok || lcd_cur_row != r || lcd_cur_col != c) {
      lcd->setCursor(c, r);
      lcd_cur_ok = true; lcd_cur_row = r; lcd_cur_col = c;
      return;
    }
    lcd->write((uint8_t)lcd_want[r][c]);
    lcd_have[r][c] = lcd_want[r][c];
    if (++lcd_cur_col >= 16) lcd_cur_ok = false;   // 줄 끝을 넘으면 다음 줄로 이어지지 않는다
    return;
  }
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void lcdSetup() {
  Wire.begin();
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  if      (i2cPresent(0x27)) lcd = &lcd27;
  else if (i2cPresent(0x3F)) lcd = &lcd3f;
  else return;                                     // LCD 없음 — 나머지만 돈다
  lcd->init();
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);       // init 이 Wire.begin 을 다시 부른다
  lcd->backlight();
  lcd->clear();
  for (uint8_t r = 0; r < 2; r++) { memset(lcd_have[r], ' ', 16); lcd_have[r][16] = '\0'; }
  lcd_cur_ok = false;
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
unsigned long compose_ms  = 0;

void setup() {
  Serial.begin(BAUD);
  pinMode(K1, INPUT_PULLUP);
  pinMode(K2, INPUT_PULLUP);
  pinMode(SWA, INPUT_PULLUP);
  pinMode(SWB, INPUT_PULLUP);

  for (uint8_t r = 0; r < 2; r++) { memset(lcd_want[r], ' ', 16); lcd_want[r][16] = '\0'; }
  lcdSetup();
  lcdCompose(millis());

  Serial.println(F("J,RESET"));   // 리셋(버튼/전원인가/포트열림) 시 1회 전송 -> 파이썬이 재보정 타이밍을 잡는 신호
  next_out_us = micros();
}

void loop() {
  // 1) 4 ms 마다 : 버튼 → ON/OFF · 타이머, 스틱 · 버튼 값을 PC 로.  밀리면 몰아 보내지 않고 다시 맞춘다
  unsigned long now_us = micros();
  if ((long)(now_us - next_out_us) >= 0) {
    next_out_us += OUT_US;
    if ((long)(now_us - next_out_us) >= 0) next_out_us = now_us + OUT_US;
    unsigned long now = millis();
    handleButtons(now);
    sendJoy();
    if (now - compose_ms >= LCD_COMPOSE_MS) { compose_ms = now; lcdCompose(now); }
  }

  // 2) PC → 메가 'U' 줄
  readPc();

  // 3) 남는 시간에 LCD 한 글자
  if ((long)(next_out_us - micros()) >= LCD_GAP_US) lcdStep();
}
