// =====================================================================================
// pulsetest_quad_1118.ino ― 엔코더 A · B 상 위상차 계측 + 모터 PWM 손 입력 시험 (Arduino UNO R4 Minima)
// =====================================================================================
// 2026-11-18. pulsetest_interrupt_1112.ino 를 바탕으로 출력만 '상별 펄스 수' → 'A · B 위상차' 로 바꿨다.
// 핀 배치 · PWM 입력 규약은 그대로다 (PWM 5 · DIR 6 · 엔코더 A 2 · B 3).
// 목적 : 카운터 클릭(LS7366R)의 x4 쿼드러처 계수가 이 엔코더에서 되는지 위상차로 확인한다.
//   LS7366R 데이터시트는 A · B 가 'ideally 90 degrees out-of-phase' 라고만 쓴다 — 90° 는 이상값이지 조건이 아니다.
//   실제 조건은 시간이다 : A 에지와 가장 가까운 B 에지 사이 t4 > 필터 클럭 주기 t3 (5 V 에서 26 ns 이상, 3.3 V 에서 52 ns 이상).
//   에지 간격 = min(|위상차|, 180 − |위상차|) / 360 × 주기 → 90° 에서 가장 넓고, 0° · ±180° 에 가까울수록 좁아진다.
// 컴파일은 R4 코어(renesas_uno 1.6.0, FQBN arduino:renesas_uno:minima)로 통과했다 (2026-11-18).
// ★업로드 · 실제 동작은 아직 확인 못 했다★.
//
// ── 출력 (아두이노 → PC) ───────────────────────────────────────────────────────────
//   A 상 상승 에지마다 위상차를 계산해 곧바로 한 줄 낸다 (1 초씩 모으지 않는다). 엔코더 한 주기에 한 줄.
//   "<위상차>" — 도 단위, 소수 첫째 자리, −179.9 ~ 180.0.   예) 89.6   -90.3
//   위상차 = (B 상승 시각 − 앞 A 상승 시각) / (A 상승 주기) × 360. 180 을 넘으면 360 을 빼서 음수로 쓴다.
//     + → B 가 A 보다 늦다 (A 가 앞선다).   − → B 가 앞선다.   회전 방향이 바뀌면 부호가 바뀐다.
//     정상 쿼드러처면 한 방향에서 +90 근처, 반대 방향에서 −90 근처다.
//     0 · ±180 근처면 두 상의 에지가 겹친다 → 쿼드러처 계수(x1 · x2 · x4 모두)가 안 된다.
//   숫자 말고는 아무것도 내지 않는다 — 시리얼 플로터에 한 줄 그래프로 뜬다 (가로축 한 칸 = 엔코더 한 주기, 시간이 아니다).
//   모터가 서 있으면 아무것도 나오지 않는다.
//   상승 에지끼리만 잰다. 듀티가 50 % 가 아니면 하강 에지끼리의 위상차는 이와 다를 수 있다 (이 스케치는 재지 않는다).
//
//   버리는 주기 (출력하지 않는다) :
//     · A 상승 사이에 B 상승이 0 개 또는 2 개 이상 — B 끊김 · 잡음(채터링) · 방향 전환.
//     · 주기가 바로 앞 주기의 2 배를 넘거나 절반이 안 된다 — 섰다가 다시 돌 때 정지 시간이 주기에 섞인 것.
//       부팅 · 재출발 뒤 처음 두 주기 안팎은 이 때문에 빠진다.
//     · 출력이 따라가지 못해 버퍼(RING_N 줄)가 찼을 때 새로 계산한 값.
//
//   시각은 micros() — 1 µs 단위다 (R4 코어는 AGT 타이머를 직접 읽어 인터럽트 안에서도 쓸 수 있다). 분해능 = 360 / 주기[µs] 도.
//     예) A 상 185 Hz (주기 5.4 ms) → 0.07°,   1 kHz → 0.36°,   5 kHz → 1.8°.
//   시각은 인터럽트 루틴에 들어간 뒤에 잡는다. A · B 가 같은 경로라 그 지연은 거의 상쇄되지만,
//   같은 우선순위(12)인 USB 인터럽트를 처리하는 중에 에지가 오면 그만큼 늦게 잡힌다 → 가끔 한 줄씩 튀는 값은 그 탓일 수 있다. 흐름을 본다.
//
// ── 입력 (PC → 아두이노) — pulsetest_interrupt_1112 와 같다 ─────────────────────────
//   정수 하나 + 줄 끝. −255..255 (넘으면 잘라 낸다).
//     양수 → DIR 0(LOW),  음수 → DIR 1(HIGH),  PWM = 절댓값.  0 이면 정지.
//   시리얼 모니터나 시리얼 플로터 아래의 입력 칸에서 보낸다. 줄 끝이 '\n' · '\r' 어느 쪽이든 받고,
//   "줄 끝 없음" 이어도 마지막 글자 뒤 100 ms 지나면 반영한다. 숫자가 아닌 글자가 섞인 줄은 버린다 (응답 문구 없음).
//   부팅 직후는 PWM 0 이다. 명령 시한(무수신 정지)은 넣지 않았다 — 0 을 보내야 선다.
//
// ── 핀 (UNO R4 Minima) ────────────────────────────────────────────────────────────
//   5 PWM · 6 DIR (모터 드라이버 1) · 2 엔코더 A · 3 엔코더 B (외부 인터럽트가 되는 핀). 그 밖의 핀은 쓰지 않는다.

// ===================== 쓰는 핀 =====================
const uint8_t PIN_M1_PWM = 5;     // 모터 드라이버 1 PWM
const uint8_t PIN_M1_DIR = 6;     // 모터 드라이버 1 DIR (양수 LOW · 음수 HIGH)
const uint8_t PIN_ENC_A  = 2;     // 엔코더 A 상
const uint8_t PIN_ENC_B  = 3;     // 엔코더 B 상

#define SERIAL_BAUD 115200        // R4 Minima 의 Serial 은 USB 라 실제로는 속도와 무관하다. 모니터 · 플로터도 115200 으로

const long          PWM_MAX     = 255;
const unsigned long CMD_IDLE_MS = 100;   // 줄 끝 없이 들어온 명령을 반영하기까지 기다리는 시간

// ===================== 위상차 계측 (인터럽트 → loop) =====================
// 인터럽트는 시각만 잡아 버퍼에 넣고, 나누기 · 출력은 loop 가 한다.
// A · B 인터럽트는 우선순위가 같아(12) 서로 끼어들지 않는다 → 아래 변수는 두 루틴 사이에서 따로 막지 않아도 된다.
volatile uint32_t a_rise_us   = 0;   // 앞 A 상승 시각
volatile uint32_t a_period_us = 0;   // 앞 주기 (0 = 아직 없음 → 다음 주기는 2 배 검사에 걸려 버려진다)
volatile uint32_t b_rise_us   = 0;   // 마지막 B 상승 시각
volatile uint8_t  b_rises     = 0;   // 앞 A 상승 뒤 B 상승 수 (2 에서 멈춘다)

// 인터럽트 → loop 버퍼. 한 칸 = 한 주기의 (B 지연, 주기). head == tail 이면 비었다.
const uint8_t     RING_N = 64;
volatile uint32_t ring_dt[RING_N];   // B 상승 − 앞 A 상승 [µs]
volatile uint32_t ring_t[RING_N];    // A 상승 주기 [µs]
volatile uint8_t  ring_head = 0;     // 인터럽트가 다음에 쓸 칸
volatile uint8_t  ring_tail = 0;     // loop 가 다음에 읽을 칸

void onEncoderB() {
  b_rise_us = micros();
  if (b_rises < 2) b_rises++;
}

void onEncoderA() {
  uint32_t now  = micros();
  uint32_t t    = now - a_rise_us;   // 이번 주기 (micros 가 넘쳐 돌아도 부호 없는 뺄셈이라 맞다)
  uint32_t prev = a_period_us;
  // B 상승이 꼭 한 번 · 주기가 앞 주기의 절반 ~ 2 배일 때만 쓴다
  if (b_rises == 1 && t > 0 && t / 2 <= prev && prev / 2 <= t) {
    uint8_t next = (uint8_t)((ring_head + 1) % RING_N);
    if (next != ring_tail) {                // 가득 찼으면 버린다
      ring_dt[ring_head] = b_rise_us - a_rise_us;
      ring_t[ring_head]  = t;
      ring_head = next;
    }
  }
  a_rise_us   = now;
  a_period_us = t;
  b_rises     = 0;
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
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoderB, RISING);
  // 풀업을 다시 켠다 — pulsetest_interrupt_1112 와 같은 처리다 (그 스케치로 2026-11-17 실측해 동작했다).
  // 코어 1.6.0 의 attachInterrupt 는 앞서 켠 풀업을 유지하지만(Interrupts.cpp), 다른 코어 판에서는 빠질 수 있다.
  // 오픈컬렉터 엔코더면 풀업이 꼭 있어야 한다.
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_A].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_ENC_B].pin,
                  IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
}

// ===================== 위상차가 나오는 대로 출력 =====================
void printPhases() {
  while (ring_tail != ring_head) {
    uint8_t  i  = ring_tail;
    uint32_t dt = ring_dt[i];
    uint32_t t  = ring_t[i];
    ring_tail = (uint8_t)((i + 1) % RING_N);   // 읽은 뒤에 넘긴다 — 그 전에는 인터럽트가 이 칸을 덮지 않는다

    // 0.1° 단위 정수로 반올림 (float 출력을 피한다). dt ≤ t 라 0 ~ 3600
    long x10 = (long)(((uint64_t)dt * 3600 + t / 2) / t);
    if (x10 > 1800) x10 -= 3600;               // −179.9 ~ 180.0
    long a = x10 < 0 ? -x10 : x10;
    char line[16];
    int n = snprintf(line, sizeof(line), "%s%ld.%ld\r\n", x10 < 0 ? "-" : "", a / 10, a % 10);
    Serial.write((const uint8_t *)line, n);
  }
}

void loop() {
  printPhases();    // 1) 계산된 위상차를 바로
  readCommand();    // 2) PWM 명령
}
