// ═══════════════════════════════════════════════════════════════════════════
// HaleHound-CYD GPS Module Implementation
// GT-U7 (UBLOX 7) GPS Support with TinyGPSPlus
// Created: 2026-02-07
// Updated: 2026-02-19 — Tactical instrument panel (compass, speed arc,
//                        sat bars, crosshairs, HDOP, pulsing fix dot)
// ═══════════════════════════════════════════════════════════════════════════

#include "gps_module.h"
#include "shared.h"
#include "utils.h"
#include "touch_buttons.h"
#include "icon.h"
#if __has_include("hlp_protocol.h")
  #include "hlp_protocol.h"
  #define HAS_HLP_PROTOCOL 1
#else
  #define HAS_HLP_PROTOCOL 0
  // Stub out HLP types/constants so GPS module compiles without C5 support
  #define HLP_SYNC_BYTE       0xFE
  #define HLP_MAX_PAYLOAD      1024
  #define HLP_FRAME_OVERHEAD   5
  #ifndef HLP_BAUD
  #define HLP_BAUD             460800
  #endif
  #define HLP_HEARTBEAT        0x01
  #define HLP_PING             0x06
  #define HLP_PONG             0x07
  #define HLP_GPS_DATA         0x10
  #define HLP_GPS_STATUS       0x13
  #define HLP_VERSION          0x04
  struct __attribute__((packed)) HlpGpsData {
      int32_t lat_e7; int32_t lon_e7; int32_t alt_cm;
      uint16_t speed_dmph; uint16_t course_d10;
      uint8_t satellites; uint8_t fix_type; uint16_t hdop_d100;
      uint16_t year; uint8_t month; uint8_t day;
      uint8_t hour; uint8_t minute; uint8_t second;
      uint8_t valid; uint32_t age_ms; uint32_t chars_processed;
  };
  struct __attribute__((packed)) HlpHeartbeat {
      uint32_t uptime_ms; uint32_t heap_free; uint8_t status;
  };
  enum HlpParseState { HLP_WAIT_SYNC, HLP_WAIT_TYPE, HLP_WAIT_LEN_H, HLP_WAIT_LEN_L, HLP_WAIT_PAYLOAD, HLP_WAIT_CRC };
  struct HlpParser {
      HlpParseState state; uint8_t type; uint16_t payloadLen; uint16_t payloadIdx;
      uint8_t payload[HLP_MAX_PAYLOAD];
      void reset() { state = HLP_WAIT_SYNC; type = 0; payloadLen = 0; payloadIdx = 0; }
      bool feed(uint8_t) { return false; }  // Never parses — C5 never detected
  };
  static inline uint16_t hlpBuildFrame(uint8_t* buf, uint8_t type, const uint8_t* payload, uint16_t payloadLen) {
      (void)buf; (void)type; (void)payload; (void)payloadLen; return 0;
  }
#endif
#include <TinyGPSPlus.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif

// ═══════════════════════════════════════════════════════════════════════════
// EXTERNAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

extern TFT_eSPI tft;

// ═══════════════════════════════════════════════════════════════════════════
// GPS OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

static TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);     // UART2 — pin determined by auto-scan
static GPSData currentData;
static bool gpsInitialized = false;
static unsigned long lastUpdateTime = 0;
static unsigned long lastDisplayUpdate = 0;
static unsigned long lastPulseUpdate = 0;
static int gpsActivePin = -1;           // Which GPIO ended up working

// Raw data capture — last 32 chars from UART2 for diagnostics
static char gpsRawBuf[33];             // circular capture buffer + null
static int gpsRawIdx = 0;
static uint32_t gpsDollarCount = 0;    // '$' chars seen (NMEA sentence starts)
static uint32_t gpsPassedCount = 0;    // valid checksum sentences
static uint32_t gpsSatUpdates = 0;     // how many times gps.satellites.isUpdated() fired
static int gpsActiveBaud = 9600;        // Which baud rate worked

// ═══════════════════════════════════════════════════════════════════════════
// C5 CO-PROCESSOR STATE (HaleHound-Alpha link)
// When c5Connected=true, GPS data arrives as HLP frames at 460800 baud
// instead of raw NMEA at 9600. All public API remains identical.
// ═══════════════════════════════════════════════════════════════════════════

bool c5_connected = false;             // Global — other modules can check this
static bool c5Mode = false;            // Internal — UART2 is in HLP mode
static HlpParser hlpParser;            // HLP frame parser state machine
static uint32_t c5FramesReceived = 0;  // Total HLP frames from C5
static uint32_t c5LastHeartbeat = 0;   // millis() of last heartbeat from C5
static uint32_t c5CharsReceived = 0;   // C5-reported chars_processed (from HlpGpsData)
static uint8_t  hlpTxBuf[HLP_FRAME_OVERHEAD + 4];  // Small TX buffer (PING/PONG only)

// ═══════════════════════════════════════════════════════════════════════════
// ICON BAR
// ═══════════════════════════════════════════════════════════════════════════

static void drawGPSIconBar() {
    tft.drawLine(0, ICON_BAR_Y - 1, SCREEN_WIDTH, ICON_BAR_Y - 1, HALEHOUND_MAGENTA);
    tft.fillRect(0, ICON_BAR_Y, SCREEN_WIDTH, ICON_BAR_H, HALEHOUND_DARK);
    tft.drawBitmap(10, ICON_BAR_Y, bitmap_icon_go_back, 16, 16, HALEHOUND_MAGENTA);
    tft.drawLine(0, ICON_BAR_BOTTOM, SCREEN_WIDTH, ICON_BAR_BOTTOM, HALEHOUND_HOTPINK);
}

// Check if back icon tapped - uses icon bar defines from cyd_config.h
static bool isGPSBackTapped() {
    uint16_t tx, ty;
    if (getTouchPoint(&tx, &ty)) {
        if (ty >= ICON_BAR_TOUCH_TOP && ty <= ICON_BAR_TOUCH_BOTTOM && tx >= 10 && tx < 30) {
            delay(150);
            return true;
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// COMPASS DIRECTION HELPER
// ═══════════════════════════════════════════════════════════════════════════

static const char* compassDirection(float heading) {
    if (heading >= 337.5f || heading < 22.5f)  return "N";
    if (heading < 67.5f)  return "NE";
    if (heading < 112.5f) return "E";
    if (heading < 157.5f) return "SE";
    if (heading < 202.5f) return "S";
    if (heading < 247.5f) return "SW";
    if (heading < 292.5f) return "W";
    return "NW";
}

// ═══════════════════════════════════════════════════════════════════════════
// INSTRUMENT: TACTICAL CROSSHAIRS (coordinate frame overlay)
//
// Corner brackets + center cross inside the coordinate frame.
// Drawn each update after the interior is cleared, before text.
// ═══════════════════════════════════════════════════════════════════════════

static void drawCrosshairs() {
    const int x1 = SCALE_X(10), y1 = SCALE_Y(66);      // top-left interior
    const int x2 = SCALE_X(230), y2 = SCALE_Y(110);     // bottom-right interior
    const int len = SCALE_X(15);                          // bracket arm length
    uint16_t color = HALEHOUND_GUNMETAL;

    // Top-left bracket
    tft.drawLine(x1, y1, x1 + len, y1, color);
    tft.drawLine(x1, y1, x1, y1 + len, color);

    // Top-right bracket
    tft.drawLine(x2, y1, x2 - len, y1, color);
    tft.drawLine(x2, y1, x2, y1 + len, color);

    // Bottom-left bracket
    tft.drawLine(x1, y2, x1 + len, y2, color);
    tft.drawLine(x1, y2, x1, y2 - len, color);

    // Bottom-right bracket
    tft.drawLine(x2, y2, x2 - len, y2, color);
    tft.drawLine(x2, y2, x2, y2 - len, color);

    // Center cross (small)
    int cx = SCREEN_WIDTH / 2, cy = SCALE_Y(88);
    tft.drawLine(cx - 4, cy, cx + 4, cy, color);
    tft.drawLine(cx, cy - 4, cx, cy + 4, color);
}

// ═══════════════════════════════════════════════════════════════════════════
// INSTRUMENT: COMPASS ROSE
//
// Spinning compass with heading needle, 8 tick marks (N/NE/E/SE/S/SW/W/NW),
// double rim, center dot, and heading + direction text below.
// ═══════════════════════════════════════════════════════════════════════════

static void drawCompass(float heading, bool valid) {
    const int cx = SCALE_X(40);
    const int cy = SCALE_Y(146);
    const int r = SCALE_X(22);

    // Clear compass area
    tft.fillRect(SCALE_X(4), SCALE_Y(118), SCALE_W(74), SCALE_H(64), TFT_BLACK);

    uint16_t rimColor = valid ? HALEHOUND_VIOLET : HALEHOUND_GUNMETAL;
    uint16_t needleColor = valid ? HALEHOUND_MAGENTA : HALEHOUND_GUNMETAL;

    // Double rim
    tft.drawCircle(cx, cy, r, rimColor);
    tft.drawCircle(cx, cy, r + 1, HALEHOUND_GUNMETAL);

    // 8 tick marks at compass points (N=0, NE=45, E=90, etc.)
    for (int i = 0; i < 8; i++) {
        float a = i * 45.0f * DEG_TO_RAD;
        int tickLen = (i % 2 == 0) ? SCALE_X(5) : SCALE_X(3);
        uint16_t tickColor = (i == 0) ? HALEHOUND_HOTPINK : HALEHOUND_GUNMETAL;

        int ox = cx + (int)(sin(a) * (float)r);
        int oy = cy - (int)(cos(a) * (float)r);
        int ix = cx + (int)(sin(a) * (float)(r - tickLen));
        int iy = cy - (int)(cos(a) * (float)(r - tickLen));

        tft.drawLine(ix, iy, ox, oy, tickColor);
    }

    // "N" label above compass
    tft.setTextSize(1);
    tft.setTextColor(HALEHOUND_HOTPINK);
    tft.setCursor(cx - 3, cy - r - 10);
    tft.print("N");

    // Heading needle (thick — 3 parallel lines)
    float rad = heading * DEG_TO_RAD;
    int tipX = cx + (int)(sin(rad) * (float)(r - SCALE_X(5)));
    int tipY = cy - (int)(cos(rad) * (float)(r - SCALE_X(5)));

    tft.drawLine(cx, cy, tipX, tipY, needleColor);
    tft.drawLine(cx + 1, cy, tipX + 1, tipY, needleColor);
    tft.drawLine(cx - 1, cy, tipX - 1, tipY, needleColor);

    // Tail (shorter, opposite direction)
    int tailX = cx - (int)(sin(rad) * (float)(r / 3));
    int tailY = cy + (int)(cos(rad) * (float)(r / 3));
    tft.drawLine(cx, cy, tailX, tailY, HALEHOUND_GUNMETAL);

    // Center dot (ring style)
    tft.fillCircle(cx, cy, 3, HALEHOUND_HOTPINK);
    tft.fillCircle(cx, cy, 1, HALEHOUND_DARK);

    // Heading text + compass direction below
    char buf[12];
    if (valid) {
        snprintf(buf, sizeof(buf), "%.0f %s", heading, compassDirection(heading));
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    tft.setTextColor(valid ? HALEHOUND_MAGENTA : HALEHOUND_GUNMETAL);
    tft.setTextSize(1);
    int tw = strlen(buf) * 6;
    tft.setCursor(cx - tw / 2, SCALE_Y(176));
    tft.print(buf);
}

// ═══════════════════════════════════════════════════════════════════════════
// INSTRUMENT: SPEED ARC GAUGE
//
// 270-degree arc (gap at bottom). Fills left-to-right with color gradient:
// magenta → hotpink → red. Speed value displayed inside the arc.
// Max speed: 120 km/h.
// ═══════════════════════════════════════════════════════════════════════════

static void drawSpeedArc(float speed, bool valid) {
    const int cx = SCREEN_WIDTH / 2;
    const int cy = SCALE_Y(152);
    const int outerR = SCALE_X(22);
    const int innerR = SCALE_X(16);
    const float maxSpeed = 120.0f;
    const int totalSweep = 270;

    // Clear speed area
    tft.fillRect(SCALE_X(82), SCALE_Y(118), SCALE_W(76), SCALE_H(64), TFT_BLACK);

    int fillSteps = 0;
    if (valid && speed > 0.5f) {
        fillSteps = (int)((speed / maxSpeed) * (float)totalSweep);
        if (fillSteps > totalSweep) fillSteps = totalSweep;
    }

    // Draw arc: sweep 270 degrees
    for (int step = 0; step <= totalSweep; step += 3) {
        float angleDeg = 225.0f - (float)step;
        float rad = angleDeg * DEG_TO_RAD;

        int ix = cx + (int)(cos(rad) * (float)innerR);
        int iy = cy - (int)(sin(rad) * (float)innerR);
        int ox = cx + (int)(cos(rad) * (float)outerR);
        int oy = cy - (int)(sin(rad) * (float)outerR);

        uint16_t color;
        if (step <= fillSteps && valid) {
            float frac = (float)step / (float)totalSweep;
            if (frac < 0.5f)       color = HALEHOUND_MAGENTA;
            else if (frac < 0.75f) color = HALEHOUND_HOTPINK;
            else                   color = 0xF800;
        } else {
            color = HALEHOUND_DARK;
        }

        tft.drawLine(ix, iy, ox, oy, color);
    }

    // Speed value inside arc center
    char buf[10];
    if (valid) {
        if (speed < 10.0f) snprintf(buf, sizeof(buf), "%.1f", speed);
        else               snprintf(buf, sizeof(buf), "%.0f", speed);
    } else {
        snprintf(buf, sizeof(buf), "--");
    }
    tft.setTextColor(valid ? HALEHOUND_MAGENTA : HALEHOUND_GUNMETAL);
    tft.setTextSize(1);
    int tw = strlen(buf) * 6;
    tft.setCursor(cx - tw / 2, cy - 3);
    tft.print(buf);

    // "km/h" label below arc
    tft.setTextColor(HALEHOUND_GUNMETAL);
    tft.setCursor(cx - 12, SCALE_Y(176));
    tft.print("km/h");
}

// ═══════════════════════════════════════════════════════════════════════════
// 16-BIT COLOR INTERPOLATION (565 format)
// ═══════════════════════════════════════════════════════════════════════════

static uint16_t lerpColor565(uint16_t c1, uint16_t c2, float t) {
    int r1 = (c1 >> 11) & 0x1F;
    int g1 = (c1 >> 5)  & 0x3F;
    int b1 =  c1        & 0x1F;
    int r2 = (c2 >> 11) & 0x1F;
    int g2 = (c2 >> 5)  & 0x3F;
    int b2 =  c2        & 0x1F;
    int r = r1 + (int)((float)(r2 - r1) * t);
    int g = g1 + (int)((float)(g2 - g1) * t);
    int b = b1 + (int)((float)(b2 - b1) * t);
    return (uint16_t)((r << 11) | (g << 5) | b);
}

// ═══════════════════════════════════════════════════════════════════════════
// INSTRUMENT: SATELLITE SIGNAL BARS
//
// 5 increasing-height bars (cell signal style). Each bar has a vertical
// gradient from HALEHOUND_DARK (bottom) to its target HaleHound color
// (top). Gradient per bar: VIOLET → VIOLET → MAGENTA → HOTPINK → BRIGHT.
// ═══════════════════════════════════════════════════════════════════════════

static void drawSatBars(int satellites) {
    const int barW = SCALE_X(10);
    const int gap = SCALE_X(3);
    const int startX = SCALE_X(168);
    const int bottomY = SCALE_Y(168);
    const int barHeights[] = {SCALE_H(8), SCALE_H(14), SCALE_H(20), SCALE_H(26), SCALE_H(32)};
    const int thresholds[] = {1, 3, 5, 7, 9};

    uint16_t barColors[] = {
        HALEHOUND_VIOLET,
        HALEHOUND_VIOLET,
        HALEHOUND_MAGENTA,
        HALEHOUND_HOTPINK,
        HALEHOUND_BRIGHT
    };

    // Clear satellite area
    tft.fillRect(SCALE_X(162), SCALE_Y(118), SCALE_W(74), SCALE_H(64), TFT_BLACK);

    // "SAT" label at top
    tft.setTextSize(1);
    tft.setTextColor(HALEHOUND_HOTPINK);
    tft.setCursor(SCALE_X(188), SCALE_Y(120));
    tft.print("SAT");

    for (int i = 0; i < 5; i++) {
        int x = startX + i * (barW + gap);
        int h = barHeights[i];
        int y = bottomY - h;

        if (satellites >= thresholds[i]) {
            for (int row = 0; row < h; row++) {
                float t = 1.0f - (float)row / (float)(h > 1 ? h - 1 : 1);
                uint16_t color = lerpColor565(HALEHOUND_DARK, barColors[i], t);
                tft.fillRect(x, y + row, barW, 1, color);
            }
        } else {
            tft.drawRect(x, y, barW, h, HALEHOUND_GUNMETAL);
        }
    }

    // Satellite count below bars
    char buf[6];
    snprintf(buf, sizeof(buf), "%d", satellites);
    int tw = strlen(buf) * 6;
    tft.setTextColor(satellites > 0 ? HALEHOUND_MAGENTA : HALEHOUND_GUNMETAL);
    tft.setCursor(startX + SCALE_X(25) - tw / 2, SCALE_Y(176));
    tft.print(buf);
}

// ═══════════════════════════════════════════════════════════════════════════
// INSTRUMENT: PULSING FIX INDICATOR
//
// Breathing circle that oscillates in size. Updates at 150ms intervals
// for smooth animation (faster than the 1-second main display update).
//
// Green  = 3D fix locked
// Magenta = searching / 2D fix
// Red    = no GPS data (static, no pulse)
// ═══════════════════════════════════════════════════════════════════════════

static void drawSkullIndicator(bool hasFix, bool hasData) {
    const int sx = SCREEN_WIDTH - SCALE_X(26);   // Right side of status box
    const int sy = SCALE_Y(226);                   // Skull top-left y

    // Clear skull area (use HALEHOUND_DARK to match status box interior)
    tft.fillRect(sx - 1, sy - 1, 18, 18, HALEHOUND_DARK);

    if (!hasData) {
        tft.drawBitmap(sx, sy, bitmap_icon_skull_tools, 16, 16, HALEHOUND_GUNMETAL);
        return;
    }

    if (hasFix) {
        tft.drawBitmap(sx, sy, bitmap_icon_skull_tools, 16, 16, HALEHOUND_MAGENTA);
    } else {
        bool pulseOn = (millis() / 300) % 2;
        uint16_t skullColor = pulseOn ? HALEHOUND_HOTPINK : HALEHOUND_DARK;
        tft.drawBitmap(sx, sy, bitmap_icon_skull_tools, 16, 16, skullColor);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GPS SCREEN — TACTICAL INSTRUMENT LAYOUT
// All coordinates use SCALE_X/Y macros for 2.8" (240×320) and 3.5" (320×480)
// ═══════════════════════════════════════════════════════════════════════════

static void drawGPSScreen() {
    tft.fillScreen(HALEHOUND_BLACK);
    drawStatusBar();
    drawGPSIconBar();

    // Glitch title — chromatic aberration effect
    drawGlitchText(SCALE_Y(55), "GPS TRACKER", &Nosifer_Regular10pt7b);
    tft.drawLine(0, SCALE_Y(58), SCREEN_WIDTH, SCALE_Y(58), HALEHOUND_HOTPINK);

    // Coordinate frame (double border)
    int frmW = SCREEN_WIDTH - SCALE_X(10);
    tft.drawRoundRect(SCALE_X(5), SCALE_Y(62), frmW, SCALE_H(52), 6, HALEHOUND_VIOLET);
    tft.drawRoundRect(SCALE_X(5) + 1, SCALE_Y(62) + 1, frmW - 2, SCALE_H(52) - 2, 5, HALEHOUND_GUNMETAL);

    // ALT + ACC labels
    tft.setTextSize(1);
    tft.setTextColor(HALEHOUND_HOTPINK);
    tft.setCursor(SCALE_X(8), SCALE_Y(188));
    tft.print("ALT");
    tft.setCursor(SCALE_X(130), SCALE_Y(188));
    tft.print("ACC");

    // Separator
    tft.drawLine(SCALE_X(5), SCALE_Y(200), SCREEN_WIDTH - SCALE_X(5), SCALE_Y(200), HALEHOUND_HOTPINK);

    // Date/Time labels
    tft.setTextColor(HALEHOUND_HOTPINK);
    tft.setCursor(SCALE_X(8), SCALE_Y(204));
    tft.print("DATE");
    tft.setCursor(SCALE_X(130), SCALE_Y(204));
    tft.print("TIME");

    // Separator
    tft.drawLine(SCALE_X(5), SCALE_Y(216), SCREEN_WIDTH - SCALE_X(5), SCALE_Y(216), HALEHOUND_HOTPINK);

    // Status box frame
    tft.drawRoundRect(SCALE_X(5), SCALE_Y(220), SCREEN_WIDTH - SCALE_X(10), SCALE_H(28), 4, HALEHOUND_VIOLET);

    // Diagnostic section labels
    tft.setTextColor(HALEHOUND_GUNMETAL);
    tft.setCursor(SCALE_X(8), SCALE_Y(254));
    tft.print(c5Mode ? "HLP" : "NMEA");
    tft.setCursor(SCALE_X(8), SCALE_Y(266));
    tft.print(c5Mode ? "LINK" : "PIN");
    tft.setCursor(SCALE_X(8), SCALE_Y(278));
    tft.print("AGE");
}

// ═══════════════════════════════════════════════════════════════════════════
// UPDATE GPS VALUES — Called every 1 second
//
// Redraws all dynamic content: coordinates with crosshairs, all three
// instruments, ALT/HDOP values, date/time, status box, diagnostics.
// ═══════════════════════════════════════════════════════════════════════════

static void updateGPSValues() {
    char buf[48];

    // ── Coordinate frame (clear interior, draw crosshairs, then values) ──
    tft.fillRect(SCALE_X(8), SCALE_Y(65), SCALE_W(224), SCALE_H(46), TFT_BLACK);
    drawCrosshairs();

    if (currentData.valid) {
        snprintf(buf, sizeof(buf), "%.6f %c",
                 fabs(currentData.latitude),
                 currentData.latitude >= 0 ? 'N' : 'S');
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextColor(HALEHOUND_MAGENTA);
        tft.setCursor(SCALE_X(12), SCALE_Y(84));
        tft.print(buf);

        snprintf(buf, sizeof(buf), "%.6f %c",
                 fabs(currentData.longitude),
                 currentData.longitude >= 0 ? 'E' : 'W');
        tft.setCursor(SCALE_X(12), SCALE_Y(104));
        tft.print(buf);
        tft.setFreeFont(NULL);
    } else {
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextColor(HALEHOUND_GUNMETAL);
        tft.setCursor(SCALE_X(28), SCALE_Y(92));
        tft.print("-- waiting --");
        tft.setFreeFont(NULL);
    }

    // ── Instrument panel (compass, speed arc, satellite bars) ──
    drawCompass(currentData.course, currentData.valid);
    drawSpeedArc(currentData.speed, currentData.valid);
    drawSatBars(currentData.satellites);

    // ── ALT + HDOP values ──
    tft.setTextSize(1);

    tft.fillRect(SCALE_X(30), SCALE_Y(188), SCALE_W(90), SCALE_H(10), TFT_BLACK);
    tft.setTextColor(currentData.valid ? HALEHOUND_MAGENTA : HALEHOUND_GUNMETAL);
    tft.setCursor(SCALE_X(30), SCALE_Y(188));
    if (currentData.valid) {
        snprintf(buf, sizeof(buf), "%.1fm", currentData.altitude);
        tft.print(buf);
    } else {
        tft.print("---");
    }

    tft.fillRect(SCALE_X(152), SCALE_Y(188), SCALE_W(83), SCALE_H(10), TFT_BLACK);
    if (currentData.hdop > 0.01 && currentData.valid) {
        float accFeet = currentData.hdop * 2.5f * 3.28084f;
        uint16_t accColor;
        if (accFeet < 16.0f)       accColor = HALEHOUND_BRIGHT;
        else if (accFeet < 33.0f)  accColor = HALEHOUND_HOTPINK;
        else                       accColor = 0xF800;
        tft.setTextColor(accColor);
        if (accFeet < 100.0f)
            snprintf(buf, sizeof(buf), "~%.0fft", accFeet);
        else
            snprintf(buf, sizeof(buf), ">100ft");
        tft.setCursor(SCALE_X(152), SCALE_Y(188));
        tft.print(buf);
    } else {
        tft.setTextColor(HALEHOUND_GUNMETAL);
        tft.setCursor(SCALE_X(152), SCALE_Y(188));
        tft.print("---");
    }

    // ── Date / Time ──
    tft.fillRect(SCALE_X(34), SCALE_Y(204), SCALE_W(90), SCALE_H(10), TFT_BLACK);
    tft.fillRect(SCALE_X(160), SCALE_Y(204), SCALE_W(75), SCALE_H(10), TFT_BLACK);

    if (currentData.valid && currentData.year > 2000) {
        tft.setTextColor(HALEHOUND_MAGENTA);
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                 currentData.year, currentData.month, currentData.day);
        tft.setCursor(SCALE_X(34), SCALE_Y(204));
        tft.print(buf);

        snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                 currentData.hour, currentData.minute, currentData.second);
        tft.setCursor(SCALE_X(160), SCALE_Y(204));
        tft.print(buf);
    } else {
        tft.setTextColor(HALEHOUND_GUNMETAL);
        tft.setCursor(SCALE_X(34), SCALE_Y(204));
        tft.print("----/--/--");
        tft.setCursor(SCALE_X(160), SCALE_Y(204));
        tft.print("--:--:--");
    }

    // ── Status box (color-coded) ──
    bool hasData = c5Mode ? (c5FramesReceived > 0) : (gps.charsProcessed() > 0);

    tft.fillRoundRect(SCALE_X(6), SCALE_Y(221), SCREEN_WIDTH - SCALE_X(12), SCALE_H(26), 3, HALEHOUND_DARK);
    tft.setTextSize(1);

    if (!hasData) {
        if (c5Mode) {
            drawCenteredText(SCALE_Y(230), "C5 LINKED - Waiting...", HALEHOUND_BRIGHT, 1);
        } else {
            drawCenteredText(SCALE_Y(230), "NO DATA - Check wiring", 0xF800, 1);
        }
    } else if (!currentData.valid) {
        if (currentData.satellites > 0) {
            snprintf(buf, sizeof(buf), "SEARCHING  %d sats", currentData.satellites);
            drawCenteredText(SCALE_Y(230), buf, HALEHOUND_HOTPINK, 1);
        } else {
            drawCenteredText(SCALE_Y(230), "NO FIX - Need sky view", HALEHOUND_VIOLET, 1);
        }
    } else {
        if (currentData.satellites >= 4) {
            snprintf(buf, sizeof(buf), "3D FIX  %d sats  LOCKED", currentData.satellites);
            drawCenteredText(SCALE_Y(230), buf, 0x07E0, 1);
        } else {
            snprintf(buf, sizeof(buf), "2D FIX  %d sats", currentData.satellites);
            drawCenteredText(SCALE_Y(230), buf, HALEHOUND_BRIGHT, 1);
        }
    }

    // ── Pulsing fix skull (inside status box, right side) ──
    drawSkullIndicator(currentData.valid, hasData);

    // ── Diagnostics ──
    tft.fillRect(SCALE_X(35), SCALE_Y(254), SCALE_W(200), SCALE_H(10), TFT_BLACK);
    tft.fillRect(SCALE_X(30), SCALE_Y(266), SCALE_W(200), SCALE_H(10), TFT_BLACK);
    tft.fillRect(SCALE_X(30), SCALE_Y(278), SCALE_W(200), SCALE_H(10), TFT_BLACK);

    tft.setTextColor(HALEHOUND_GUNMETAL);
    tft.setTextSize(1);

    if (c5Mode) {
        // C5 mode diagnostics: HLP frame count + C5-reported GPS chars
        snprintf(buf, sizeof(buf), "HLP %lu frm  GPS %lu ch",
                 (unsigned long)c5FramesReceived,
                 (unsigned long)c5CharsReceived);
    } else {
        snprintf(buf, sizeof(buf), "%lu chars  %lu ok  %lu fail",
                 (unsigned long)gps.charsProcessed(),
                 (unsigned long)gps.sentencesWithFix(),
                 (unsigned long)gps.failedChecksum());
    }
    tft.setCursor(SCALE_X(35), SCALE_Y(254));
    tft.print(buf);

    if (c5Mode) {
        snprintf(buf, sizeof(buf), "C5 ALPHA @ %d", HLP_BAUD);
    } else if (gpsActivePin >= 0) {
        snprintf(buf, sizeof(buf), "GPIO%d @ %d", gpsActivePin, gpsActiveBaud);
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    tft.setCursor(SCALE_X(30), SCALE_Y(266));
    tft.print(buf);

    if (currentData.valid) {
        snprintf(buf, sizeof(buf), "%lums", (unsigned long)currentData.age);
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    tft.setCursor(SCALE_X(30), SCALE_Y(278));
    tft.print(buf);
}

// ═══════════════════════════════════════════════════════════════════════════
// C5 HLP FRAME HANDLER — Convert HlpGpsData to currentData (GPSData)
// ═══════════════════════════════════════════════════════════════════════════

static void hlpApplyGpsData(const HlpGpsData* hlp) {
    // Convert fixed-point wire format back to doubles
    if (hlp->valid & 0x01) {  // location valid
        currentData.valid     = true;
        currentData.latitude  = (double)hlp->lat_e7 / 1e7;
        currentData.longitude = (double)hlp->lon_e7 / 1e7;
        lastUpdateTime = millis();
    }

    if (hlp->valid & 0x02) {  // altitude valid
        currentData.altitude = (double)hlp->alt_cm / 100.0;
    }

    currentData.speed      = (double)hlp->speed_dmph / 10.0;
    currentData.course     = (double)hlp->course_d10 / 10.0;
    currentData.satellites = hlp->satellites;
    currentData.hdop       = (double)hlp->hdop_d100 / 100.0;
    currentData.year       = hlp->year;
    currentData.month      = hlp->month;
    currentData.day        = hlp->day;
    currentData.hour       = hlp->hour;
    currentData.minute     = hlp->minute;
    currentData.second     = hlp->second;
    currentData.age        = hlp->age_ms;

    // Track C5 diagnostics
    c5CharsReceived = hlp->chars_processed;
}

// Process one complete HLP frame from C5
static void hlpProcessFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
    c5FramesReceived++;

    switch (type) {
        case HLP_GPS_DATA:
            if (len >= sizeof(HlpGpsData)) {
                hlpApplyGpsData((const HlpGpsData*)payload);
            }
            break;

        case HLP_HEARTBEAT:
            c5LastHeartbeat = millis();
            break;

        case HLP_PONG:
            // Handled inline during detection — shouldn't arrive here normally
            break;

        case HLP_VERSION:
            // Could log capabilities, but Phase 1 doesn't need it
            break;

        case HLP_GPS_STATUS:
            // Diagnostic counters — could display if needed
            break;

        default:
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// C5 DETECTION — Send PING at 460800, wait for PONG
// Returns true if C5 co-processor responds
// ═══════════════════════════════════════════════════════════════════════════

static bool tryC5Detection() {
    // Open UART2 at HLP baud rate on P1 pins
    gpsSerial.end();
    delay(50);
    gpsSerial.begin(HLP_BAUD, SERIAL_8N1, GPS_RX_PIN, 1);  // RX=GPIO3, TX=GPIO1
    delay(50);

    // Drain any garbage
    while (gpsSerial.available()) gpsSerial.read();

    // Build and send HLP_PING frame
    uint16_t frameSize = hlpBuildFrame(hlpTxBuf, HLP_PING, nullptr, 0);
    gpsSerial.write(hlpTxBuf, frameSize);
    gpsSerial.flush();

    // Wait for HLP_PONG response
    HlpParser detectParser;
    detectParser.reset();
    unsigned long start = millis();

    while (millis() - start < HLP_DETECT_TIMEOUT_MS) {
        while (gpsSerial.available() > 0) {
            uint8_t b = gpsSerial.read();
            if (detectParser.feed(b)) {
                if (detectParser.type == HLP_PONG) {
                    return true;
                }
                // Got a valid frame but not PONG — could be heartbeat, keep waiting
                if (detectParser.type == HLP_HEARTBEAT) {
                    c5LastHeartbeat = millis();
                    return true;  // Heartbeat also proves C5 is alive
                }
                detectParser.reset();
            }
        }
        delay(1);
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Try a specific pin/baud combo, return chars received in timeoutMs
static uint32_t tryGPSPin(int pin, int baud, int timeoutMs) {
    gpsSerial.end();
    delay(50);
    gpsSerial.begin(baud, SERIAL_8N1, pin, -1);
    delay(50);

    // Drain any garbage
    while (gpsSerial.available()) gpsSerial.read();

    uint32_t charsBefore = gps.charsProcessed();
    unsigned long start = millis();

    while (millis() - start < (unsigned long)timeoutMs) {
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
        delay(5);
    }

    return gps.charsProcessed() - charsBefore;
}

void gpsSetup() {
    if (gpsInitialized) return;

    memset(&currentData, 0, sizeof(currentData));
    currentData.valid = false;

    // ── Auto-scan: try multiple pins and baud rates ──
    // Show scanning screen
    tft.fillRect(0, SCALE_Y(60), SCREEN_WIDTH, SCALE_H(200), TFT_BLACK);
    drawGlitchText(SCALE_Y(55), "GPS TRACKER", &Nosifer_Regular10pt7b);
    tft.drawLine(0, SCALE_Y(58), SCREEN_WIDTH, SCALE_Y(58), HALEHOUND_HOTPINK);

    drawCenteredText(SCALE_Y(80), "SCANNING GPS...", HALEHOUND_HOTPINK, 2);

    // ── Step 0: Try C5 co-processor first (HLP at 460800 on P1) ──
    tft.fillRect(0, SCALE_Y(110), SCREEN_WIDTH, SCALE_H(60), TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(HALEHOUND_BRIGHT);
    tft.setCursor(SCALE_X(10), SCALE_Y(115));
    tft.print("C5 ALPHA @ 460800...");

    if (tryC5Detection()) {
        // C5 co-processor detected! GPS data arrives via HLP frames.
        c5Mode = true;
        c5_connected = true;
        hlpParser.reset();
        gpsActivePin = GPS_RX_PIN;
        gpsActiveBaud = HLP_BAUD;

        tft.setCursor(SCALE_X(10), SCALE_Y(130));
        tft.setTextColor(0x07E0);
        tft.print("C5 ALPHA LINKED!");

        tft.fillRect(0, SCALE_Y(170), SCREEN_WIDTH, SCALE_H(40), TFT_BLACK);
        drawCenteredText(SCALE_Y(180), "ALPHA @ 460800", 0x07E0, 1);

        delay(1500);
        gpsInitialized = true;
        return;
    }

    // C5 not found — fall back to direct NMEA scan (existing behavior)
    tft.setCursor(SCALE_X(10), SCALE_Y(130));
    tft.setTextColor(HALEHOUND_GUNMETAL);
    tft.print("No C5 - direct NMEA scan");
    c5Mode = false;
    c5_connected = false;
    delay(500);

    // Pin/baud combos to try — GPIO3 (P1 connector) first
    struct ScanEntry { int pin; int baud; const char* label; };
    ScanEntry scans[] = {
        { 3,  9600,  "P1 RX (GPIO3) @ 9600"   },
        { 3,  38400, "P1 RX (GPIO3) @ 38400"  },
        { 26, 9600,  "GPIO26 (spk) @ 9600"    },
        { 26, 38400, "GPIO26 (spk) @ 38400"   },
        { 1,  9600,  "P1 TX (GPIO1) @ 9600"   },
    };
    int numScans = 5;

    gpsActivePin = -1;
    gpsActiveBaud = 9600;

    for (int i = 0; i < numScans; i++) {
        // Show current attempt
        tft.fillRect(0, SCALE_Y(110), SCREEN_WIDTH, SCALE_H(60), TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(HALEHOUND_MAGENTA);
        tft.setCursor(SCALE_X(10), SCALE_Y(115));
        tft.printf("Try %d/%d: %s", i + 1, numScans, scans[i].label);

        // Progress bar
        int barW = (SCREEN_WIDTH - SCALE_X(20)) * (i + 1) / numScans;
        tft.fillRect(SCALE_X(10), SCALE_Y(135), SCREEN_WIDTH - SCALE_X(20), SCALE_H(8), HALEHOUND_DARK);
        tft.fillRect(SCALE_X(10), SCALE_Y(135), barW, SCALE_H(8), HALEHOUND_HOTPINK);

        uint32_t chars = tryGPSPin(scans[i].pin, scans[i].baud, 2500);

        // Show result for this attempt
        tft.setCursor(SCALE_X(10), SCALE_Y(150));
        if (chars > 10) {
            tft.setTextColor(0x07E0);
            tft.printf("FOUND! %lu chars", (unsigned long)chars);

            gpsActivePin = scans[i].pin;
            gpsActiveBaud = scans[i].baud;

            delay(1000);
            break;
        } else {
            tft.setTextColor(HALEHOUND_GUNMETAL);
            tft.printf("No data (%lu chars)", (unsigned long)chars);
        }
    }

    // Show final result
    tft.fillRect(0, SCALE_Y(170), SCREEN_WIDTH, SCALE_H(40), TFT_BLACK);
    if (gpsActivePin >= 0) {
        char resultBuf[40];
        snprintf(resultBuf, sizeof(resultBuf), "LOCKED: GPIO%d @ %d", gpsActivePin, gpsActiveBaud);
        drawCenteredText(SCALE_Y(180), resultBuf, 0x07E0, 1);
    } else {
        drawCenteredText(SCALE_Y(175), "NO GPS FOUND", 0xF800, 2);
        drawCenteredText(SCALE_Y(200), "Check wiring & power", HALEHOUND_GUNMETAL, 1);
        // Default to GPS_RX_PIN so screen still shows diagnostics
        gpsSerial.end();
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
        gpsActivePin = GPS_RX_PIN;
        gpsActiveBaud = GPS_BAUD;
    }

    delay(1500);
    gpsInitialized = true;
}

bool gpsInitSilent() {
    if (gpsInitialized) return true;

    memset(&currentData, 0, sizeof(currentData));
    currentData.valid = false;

    // Same pin/baud scan as gpsSetup() — just no screen drawing
    struct ScanEntry { int pin; int baud; };
    ScanEntry scans[] = {
        { 3,  9600  },
        { 3,  38400 },
        { 26, 9600  },
        { 26, 38400 },
        { 1,  9600  },
    };
    int numScans = 5;

    gpsActivePin = -1;
    gpsActiveBaud = 9600;

    for (int i = 0; i < numScans; i++) {
        uint32_t chars = tryGPSPin(scans[i].pin, scans[i].baud, 2500);
        if (chars > 10) {
            gpsActivePin = scans[i].pin;
            gpsActiveBaud = scans[i].baud;
            break;
        }
    }

    if (gpsActivePin < 0) {
        // No GPS found — set defaults so UART2 is at least open
        gpsSerial.end();
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
        gpsActivePin = GPS_RX_PIN;
        gpsActiveBaud = GPS_BAUD;
    }

    gpsInitialized = true;
    return (gpsActivePin >= 0);
}

void gpsUpdate() {
    if (c5Mode) {
        // ── C5 MODE: Parse HLP frames from co-processor ──
        while (gpsSerial.available() > 0) {
            uint8_t b = gpsSerial.read();
            if (hlpParser.feed(b)) {
                hlpProcessFrame(hlpParser.type, hlpParser.payload, hlpParser.payloadLen);
                hlpParser.reset();
            }
        }

        // Mark as invalid if no update for GPS_TIMEOUT_MS
        if (millis() - lastUpdateTime > GPS_TIMEOUT_MS) {
            currentData.valid = false;
        }

        // C5 watchdog — if no heartbeat for 10 seconds, C5 may have reset
        if (c5LastHeartbeat > 0 && (millis() - c5LastHeartbeat > 10000)) {
            // C5 might have died — data still valid until GPS_TIMEOUT_MS
            // Don't switch to NMEA mode automatically (would need baud change)
        }

        return;
    }

    // ── DIRECT NMEA MODE: Read TinyGPSPlus from UART2 (existing behavior) ──
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);

        // Capture raw chars for diagnostics
        if (c == '$') gpsDollarCount++;
        gpsRawBuf[gpsRawIdx] = (c >= 32 && c < 127) ? c : '.';  // printable or dot
        gpsRawIdx = (gpsRawIdx + 1) % 32;
        gpsRawBuf[gpsRawIdx] = '\0';
    }

    // Track valid sentences
    gpsPassedCount = gps.passedChecksum();

    // Update data structure — ONLY on valid fix.
    // VOID GPRMC sentences (no fix) must NOT overwrite last good coordinates.
    // WiFi scans cause brief GPS dropouts; preserving the last valid position
    // lets wardriving keep logging while GPS re-acquires between scans.
    if (gps.location.isUpdated() && gps.location.isValid()) {
        currentData.valid = true;
        currentData.latitude = gps.location.lat();
        currentData.longitude = gps.location.lng();
        currentData.age = gps.location.age();
        lastUpdateTime = millis();
    }

    if (gps.altitude.isUpdated()) {
        currentData.altitude = gps.altitude.meters();
    }

    if (gps.speed.isUpdated()) {
        currentData.speed = gps.speed.kmph();
    }

    if (gps.course.isUpdated()) {
        currentData.course = gps.course.deg();
    }

    if (gps.satellites.isUpdated()) {
        currentData.satellites = gps.satellites.value();
        gpsSatUpdates++;
    }

    if (gps.hdop.isUpdated()) {
        currentData.hdop = gps.hdop.value() / 100.0;
    }

    if (gps.date.isUpdated()) {
        currentData.year = gps.date.year();
        currentData.month = gps.date.month();
        currentData.day = gps.date.day();
    }

    if (gps.time.isUpdated()) {
        currentData.hour = gps.time.hour();
        currentData.minute = gps.time.minute();
        currentData.second = gps.time.second();
    }

    // Mark as invalid if data is stale
    if (millis() - lastUpdateTime > GPS_TIMEOUT_MS) {
        currentData.valid = false;
    }

    // Periodic debug output to serial monitor
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        Serial.printf("[GPS] Chars:%lu  Fix:%lu  Fail:%lu  Sats:%d  Valid:%d  HDOP:%.1f\n",
                      (unsigned long)gps.charsProcessed(),
                      (unsigned long)gps.sentencesWithFix(),
                      (unsigned long)gps.failedChecksum(),
                      currentData.satellites,
                      currentData.valid ? 1 : 0,
                      currentData.hdop);
        lastDebug = millis();
    }
}

void gpsScreen() {
    // Release UART0 so UART2 can claim GPIO pins without matrix conflict
    Serial.end();
    delay(50);

    // Initialize GPS if needed
    if (!gpsInitialized) {
        gpsSetup();
    } else {
        // Re-entry: force clean UART2 restart on the pin found during scan.
        // end() guarantees _uart=NULL so begin() does full driver install +
        // uart_set_pin(), properly reclaiming GPIO 3 from UART0's pin matrix.
        gpsSerial.end();
        delay(100);
        if (c5Mode) {
            // C5 mode: reopen at HLP baud with TX enabled (bidirectional link)
            gpsSerial.begin(HLP_BAUD, SERIAL_8N1, GPS_RX_PIN, 1);
            hlpParser.reset();
        } else {
            // Direct NMEA mode: RX only at discovered baud
            gpsSerial.begin(gpsActiveBaud, SERIAL_8N1, gpsActivePin, -1);
        }
        delay(100);
    }

    // Draw initial screen
    drawGPSScreen();
    updateGPSValues();

    // Main loop
    bool exitRequested = false;
    lastDisplayUpdate = millis();
    lastPulseUpdate = millis();

    while (!exitRequested) {
        // Update GPS data
        gpsUpdate();

        // Full display update every 1 second
        if (millis() - lastDisplayUpdate >= GPS_UPDATE_INTERVAL_MS) {
            updateGPSValues();
            lastDisplayUpdate = millis();
        }

        // Pulsing fix dot — smooth animation at 150ms intervals
        if (millis() - lastPulseUpdate >= 150) {
            bool hasData = c5Mode ? (c5FramesReceived > 0) : (gps.charsProcessed() > 0);
            drawSkullIndicator(currentData.valid, hasData);
            lastPulseUpdate = millis();
        }

        // Handle touch input
        touchButtonsUpdate();

        // Check for back button tap
        if (isGPSBackTapped()) {
            exitRequested = true;
        }

        // Check hardware buttons
        if (buttonPressed(BTN_BACK) || buttonPressed(BTN_BOOT)) {
            exitRequested = true;
        }

        delay(10);
    }

    // Close UART2
    gpsSerial.end();
    delay(50);
    if (!c5Mode) {
        // Direct NMEA mode: do NOT restart Serial (UART0).
        // GPIO 1 (UART0 TX) is physically wired to the GPS module's RX on P1.
        // Serial.begin() re-enables UART0 TX on GPIO 1 — txPin=-1 means
        // "no change" not "disable", so GPIO 1 stays as TX.  Any Serial.println()
        // then sends 115200-baud garbage to the NEO-6M (9600 baud), which interprets
        // random bytes as UBX binary commands and cold-restarts — killing satellite
        // tracking (6→3→0 sats, never recovers).
        // Fix: end Serial, physically detach GPIO 1 from UART0 via pinMode(OUTPUT),
        // then drive HIGH (UART idle state).  Serial.println() becomes a no-op.
        pinMode(1, OUTPUT);
        digitalWrite(1, HIGH);
    }
    // In C5 mode, GPIO 1 is the UART2 TX line to C5 — don't touch it.
    // C5 handles the GPS module directly, no NEO-6M on GPIO 1 to worry about.
}

bool gpsHasFix() {
    return currentData.valid;
}

GPSData gpsGetData() {
    return currentData;
}

String gpsGetLocationString() {
    if (!currentData.valid) {
        return "0.000000,0.000000";
    }
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.6f,%.6f",
             currentData.latitude, currentData.longitude);
    return String(buffer);
}

String gpsGetTimestamp() {
    char buffer[24];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             currentData.year, currentData.month, currentData.day,
             currentData.hour, currentData.minute, currentData.second);
    return String(buffer);
}

bool gpsIsFresh() {
    return (millis() - lastUpdateTime) < GPS_TIMEOUT_MS;
}

GPSStatus gpsGetStatus() {
    if (c5Mode) {
        // C5 mode: use currentData directly (populated from HLP frames)
        if (c5FramesReceived == 0) return GPS_NO_MODULE;
        if (!currentData.valid)    return GPS_SEARCHING;
        if (currentData.altitude != 0.0) return GPS_FIX_3D;
        return GPS_FIX_2D;
    }
    // Direct NMEA mode
    if (!gpsInitialized || gps.charsProcessed() < 10) {
        return GPS_NO_MODULE;
    }
    if (!gps.location.isValid()) {
        return GPS_SEARCHING;
    }
    if (gps.altitude.isValid()) {
        return GPS_FIX_3D;
    }
    return GPS_FIX_2D;
}

uint8_t gpsGetSatellites() {
    return currentData.satellites;
}

// ═══════════════════════════════════════════════════════════════════════════
// BACKGROUND GPS — for wardriving and other modules that need live GPS
// without the full GPS screen UI
// ═══════════════════════════════════════════════════════════════════════════

void gpsStartBackground() {
    // Kill UART0 (Serial) to free GPIO 3 for GPS UART2
    Serial.end();
    delay(10);

    // Force UART2 hard reset — previous screen closed UART2 and opened UART0
    // on the same GPIO 3. The pin matrix may still have UART0's IOMUX routing
    // active. Calling end() guarantees _uart is NULL so begin() does a FULL
    // driver install + uart_set_pin() which switches GPIO 3 from IOMUX mode
    // (UART0 direct) to GPIO matrix mode (UART2 routed). Without this, begin()
    // might take the "already initialized" shortcut and skip pin reconfiguration.
    gpsSerial.end();
    delay(100);

    if (c5Mode) {
        // C5 already detected — reopen HLP link at 460800 with TX
        gpsSerial.begin(HLP_BAUD, SERIAL_8N1, GPS_RX_PIN, 1);
        hlpParser.reset();
        delay(100);
        while (gpsSerial.available()) gpsSerial.read();
        return;
    }

    // Not in C5 mode yet — try C5 detection if not previously scanned
    if (!gpsInitialized) {
        if (tryC5Detection()) {
            c5Mode = true;
            c5_connected = true;
            hlpParser.reset();
            gpsActivePin = GPS_RX_PIN;
            gpsActiveBaud = HLP_BAUD;
            gpsInitialized = true;
            delay(100);
            while (gpsSerial.available()) gpsSerial.read();
            return;
        }
        // C5 not found — fall through to direct NMEA
    }

    // Physically detach GPIO 1 from UART0 TX and drive HIGH (UART idle).
    // GPIO 1 is wired to GPS module's RX on P1. pinMode(OUTPUT) disconnects
    // the IOMUX routing that Serial.begin() set up, so even if Serial gets
    // re-initialized elsewhere, GPIO 1 won't carry UART0 TX data.
    // NOTE: Only needed in direct NMEA mode. In C5 mode, GPIO 1 is UART2 TX.
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);

    if (gpsInitialized && gpsActivePin >= 0) {
        // GPS was scanned before — reopen UART2 on the known working pin
        gpsSerial.begin(gpsActiveBaud, SERIAL_8N1, gpsActivePin, -1);
    } else {
        // Never scanned — use default pin (GPIO 3 P1 connector @ 9600)
        gpsActivePin = GPS_RX_PIN;
        gpsActiveBaud = GPS_BAUD;
        gpsSerial.begin(gpsActiveBaud, SERIAL_8N1, gpsActivePin, -1);
        gpsInitialized = true;
    }

    // Let UART2 hardware stabilize before reading
    delay(100);

    // Drain any garbage from buffer
    while (gpsSerial.available()) gpsSerial.read();
}

void gpsStopBackground() {
    gpsSerial.end();
    delay(50);
    if (!c5Mode) {
        // Do NOT restart Serial — GPIO 1 (UART0 TX) is wired to GPS module's RX.
        // Serial.begin() with txPin=-1 means "no change" — GPIO 1 stays as UART0 TX.
        // Any Serial.println() then sends 115200-baud garbage to the NEO-6M.
        // Instead: detach GPIO 1 from UART0 and hold HIGH (UART idle state).
        // NOTE: In C5 mode, GPIO 1 is UART2 TX to C5 — don't mess with it.
        pinMode(1, OUTPUT);
        digitalWrite(1, HIGH);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// C5 CO-PROCESSOR STATUS
// ═══════════════════════════════════════════════════════════════════════════

bool gpsIsC5Connected() {
    return c5Mode;
}

// ═══════════════════════════════════════════════════════════════════════════
// DIAGNOSTICS — expose TinyGPSPlus counters for wardriving debug
// In C5 mode, returns C5-reported values where applicable
// ═══════════════════════════════════════════════════════════════════════════

uint32_t gpsCharsProcessed() {
    if (c5Mode) return c5CharsReceived;
    return (uint32_t)gps.charsProcessed();
}

uint32_t gpsSentencesWithFix() {
    return (uint32_t)gps.sentencesWithFix();
}

uint32_t gpsFailedChecksums() {
    return (uint32_t)gps.failedChecksum();
}

uint32_t gpsTimeSinceLastUpdate() {
    if (lastUpdateTime == 0) return 99999;
    return (uint32_t)(millis() - lastUpdateTime);
}

uint32_t gpsDollarsSeen() {
    return gpsDollarCount;
}

uint32_t gpsPassedChecksums() {
    return gpsPassedCount;
}

const char* gpsRawDataPreview() {
    // Return the raw buffer as a linear string (last 32 chars, oldest first)
    static char out[33];
    int start = gpsRawIdx;  // current write position = oldest char
    for (int i = 0; i < 32; i++) {
        out[i] = gpsRawBuf[(start + i) % 32];
        if (out[i] == 0) out[i] = ' ';
    }
    out[32] = '\0';
    return out;
}

// Direct TinyGPSPlus satellite value — bypasses currentData to check parser state
int32_t gpsRawSatValue() {
    return gps.satellites.value();
}

// How many times gps.satellites.isUpdated() returned true
uint32_t gpsSatUpdateCount() {
    return gpsSatUpdates;
}
