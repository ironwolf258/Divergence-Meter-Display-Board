#include "NixieConfig.h"

//================= Optional Debug =================
#define DEBUG_NIXIE
#ifdef DEBUG_NIXIE
  #include <stdarg.h>
  #include <stdio.h>
  static void DBG(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);  // works on all Arduino cores
  }
#else
  #define DBG(...) do{}while(0)
#endif
//==================================================

/* === Protocol Constants === */
constexpr uint8_t  SYNC0        = 0xA5;
constexpr uint8_t  SYNC1        = 0x5A;
constexpr uint8_t  PROTO_VER    = 0x01;

/* Commands (NOTE: CMD_SET_DOTS uses a single index, not a mask) */
enum : uint8_t {
  CMD_SET_DIGIT  = 0x01,
  CMD_SET_MULTI  = 0x02,
  CMD_SET_DOTS   = 0x03,   // payload: idx (0..7), or 0xFF = off
  CMD_SET_TIME8  = 0x04,
  CMD_PING       = 0x20,
  CMD_PONG       = 0x21,
  CMD_MODE       = 0x30
};

constexpr uint32_t SERIAL1_BAUD     = 115200;
constexpr uint16_t LINK_TIMEOUT_MS  = 2000;

/* CRC8 Dallas/Maxim (poly 0x31, init 0x00, MSB-first) */
static uint8_t crc8_maxim(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

/* === Device State === */
static uint32_t lastFrameMs = 0;
static bool     blanked     = false;

/* If your dot K155 output for tubes 3 and 6 is physically the same,
   map both indices to the same BCD code here. Adjust as needed. */
static const uint8_t DOT_BCD_MAP[8] = {
  0, 1, 2, 3, 4, 5, 3, 7   // tube 3 and 6 -> BCD 3 (example)
};

/* Parser State Machine */
enum RxState : uint8_t { WAIT_SYNC0, WAIT_SYNC1, WAIT_VER, WAIT_LEN, WAIT_CMD, WAIT_PAYLOAD, WAIT_CRC };
static RxState rxState = WAIT_SYNC0;
static uint8_t ver = 0, len = 0, cmd = 0;
static uint8_t payload[32];
static uint8_t payIdx = 0;

/* === Helpers === */
static void allClear() {
  for (uint8_t i = 0; i < NUM_TUBES; i++) tubes[i].clear();
  dots.clear(); // also turn off dots
}

static void setBlank(bool on) {
  blanked = on;
  if (on) allClear();
}

static void sendPong(uint8_t nonce) {
  uint8_t out[7] = { SYNC0, SYNC1, PROTO_VER, 2, CMD_PONG, nonce, 0 };
  const uint8_t crcbuf[4] = { PROTO_VER, 2, CMD_PONG, nonce };
  out[6] = crc8_maxim(crcbuf, sizeof(crcbuf));
  Serial1.write(out, sizeof(out));
}

/* === Command Handlers === */
static void handleSetDigit(uint8_t t, uint8_t d) {
  if (t < NUM_TUBES && d <= 10) tubes[t].display(d);
}

static void handleSetMulti(const uint8_t* p, uint8_t n) {
  if (n < 1) return;
  const uint8_t count = p[0];
  if (count == 0 || (uint8_t)(1 + count * 2) != n) return;
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t t = p[1 + i * 2];
    const uint8_t d = p[1 + i * 2 + 1];
    handleSetDigit(t, d);
  }
}

static void handleSetTime8(const uint8_t* p, uint8_t n) {
  if (n != 8) return;
  for (uint8_t i = 0; i < NUM_TUBES; i++) handleSetDigit(i, p[i]);
}

/* NEW: dots controlled by single index (0..7), 0xFF = off */
static void handleSetDots(uint8_t idx) {
  if (idx == 0xFF) {
    dots.clear();
#ifdef DEBUG_NIXIE
    DBG("[DOT] off\r\n");
#endif
    return;
  }
  if (idx > 7) idx = 7;
  uint8_t code = DOT_BCD_MAP[idx];
  dots.display(code);
#ifdef DEBUG_NIXIE
  DBG("[DOT] idx=%u -> BCD=%u\r\n", idx, code);
#endif
}

/* === Execute Parsed Command === */
static void execCommand(uint8_t c, const uint8_t* p, uint8_t n) {
  switch (c) {
    case CMD_SET_DIGIT: if (n == 2) handleSetDigit(p[0], p[1]); break;
    case CMD_SET_MULTI:            handleSetMulti(p, n);        break;
    case CMD_SET_DOTS:  if (n == 1) handleSetDots(p[0]);        break; // <--- added
    case CMD_SET_TIME8:            handleSetTime8(p, n);        break;
    case CMD_PING:       if (n == 1) sendPong(p[0]);            break;
    case CMD_MODE:       if (n == 1) setBlank(p[0] != 0);       break;
    default: break; // ignore unknown for forward-compat
  }
}

/* === Parser === */
static void parserFeed(uint8_t b) {
#ifdef DEBUG_NIXIE
  DBG("RX 0x%02X\r\n", b);
#endif
  switch (rxState) {
    case WAIT_SYNC0:
      rxState = (b == SYNC0) ? WAIT_SYNC1 : WAIT_SYNC0;
      break;
    case WAIT_SYNC1:
      rxState = (b == SYNC1) ? WAIT_VER : WAIT_SYNC0;
      break;
    case WAIT_VER:
      ver = b;
      rxState = (ver == PROTO_VER) ? WAIT_LEN : WAIT_SYNC0;
      break;
    case WAIT_LEN:
      len = b;
      payIdx = 0;
      rxState = WAIT_CMD;
      break;
    case WAIT_CMD:
      cmd = b;
      rxState = (len == 0) ? WAIT_CRC : WAIT_PAYLOAD;
      break;
    case WAIT_PAYLOAD:
      payload[payIdx++] = b;
      if (payIdx >= (uint8_t)(len - 1)) rxState = WAIT_CRC; // LEN counts CMD + payload
      break;
    case WAIT_CRC: {
      uint8_t buf[1 + 1 + 1 + 32];
      buf[0] = ver; buf[1] = len; buf[2] = cmd;
      for (uint8_t i = 0; i < (uint8_t)(len - 1); i++) buf[3 + i] = payload[i];
      const uint8_t calc = crc8_maxim(buf, (uint8_t)(3 + (len - 1)));
      if (calc == b) {
        lastFrameMs = millis();
#ifdef DEBUG_NIXIE
        DBG("[OK] ver=0x%02X len=%u cmd=0x%02X\r\n", ver, len, cmd);
#endif
        execCommand(cmd, payload, (uint8_t)(len - 1));
      }
#ifdef DEBUG_NIXIE
      else {
        DBG("[CRC ERR] got=0x%02X calc=0x%02X (ver=0x%02X len=%u cmd=0x%02X)\r\n",
            b, calc, ver, len, cmd);
      }
#endif
      rxState = WAIT_SYNC0;
    } break;
  }
}

/* === Setup / Loop === */
void setup() {
#ifdef DEBUG_NIXIE
  Serial.begin(115200);   // USB-CDC for debug prints
  delay(50);
  DBG("\r\nNixie Control (Debug)\r\n");
#endif

  Serial1.begin(SERIAL1_BAUD);   // UART link to host

  // Init tubes and dots
  for (uint8_t i = 0; i < NUM_TUBES; i++) tubes[i].begin();
  dots.begin();       // <--- init the dot K155 BCD pins
  lastFrameMs = millis();
  blanked = false;

  allClear(); // start blank (digits + dots)
}

void loop() {
  while (Serial1.available()) {
    parserFeed((uint8_t)Serial1.read());
  }

  // Link timeout → auto-blank (digits + dots)
  if (!blanked && (millis() - lastFrameMs) > LINK_TIMEOUT_MS) {
    setBlank(true);
#ifdef DEBUG_NIXIE
    DBG("[TIMEOUT] %u ms with no frames; blanking.\r\n", (unsigned)LINK_TIMEOUT_MS);
#endif
  }
}
