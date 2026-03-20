// ═══════════════════════════════════════════════════════════════════════════
// HaleHound-CYD Radio Test Tool
// Interactive hardware verification for NRF24L01+, CC1101, and GPS
// Tap a radio name to run its test. Results show inline as PASS/FAIL.
// Includes wiring reference diagrams.
// Created: 2026-02-19
// Updated: 2026-03-20 — NRF24 spectrum scan + TX test,
//                        CC1101 signal detection, GPS test (from Duggie)
// ═══════════════════════════════════════════════════════════════════════════

#include "radio_test.h"
#include "cyd_config.h"
#include "shared.h"
#include "utils.h"
#include "touch_buttons.h"
#include "icon.h"
#include "nrf24_config.h"
#include "gps_module.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <RF24.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

extern TFT_eSPI tft;

// Forward declarations for functions defined in HaleHound-CYD.ino
extern void drawStatusBar();
extern void drawInoIconBar();

// Forward declaration for drawMainScreen (defined later in this file)
static void drawMainScreen();

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN LAYOUT CONSTANTS
// 4 test buttons + wiring button, tighter spacing than before
// ═══════════════════════════════════════════════════════════════════════════

// Title at Y=60 (drawGlitchTitle)
// NRF24 button:   Y=80..99    (20px tall)
// NRF24 status:   Y=101 (one line)
// CC1101 button:  Y=115..134  (20px tall)
// CC1101 status:  Y=136 (one line)
// GPS button:     Y=150..169  (20px tall)
// GPS status:     Y=171 (one line)
// Wiring button:  Y=190..209  (20px tall)
// Hint:           Y=215

#define RT_NRF_BTN_Y     SCALE_Y(80)
#define RT_NRF_BTN_H     SCALE_H(20)
#define RT_NRF_STATUS_Y  SCALE_Y(101)
#define RT_NRF_HINT_Y    SCALE_Y(112)
#define RT_CC_BTN_Y      SCALE_Y(126)
#define RT_CC_BTN_H      SCALE_H(20)
#define RT_CC_STATUS_Y   SCALE_Y(147)
#define RT_CC_HINT_Y     SCALE_Y(158)
#define RT_GPS_BTN_Y     SCALE_Y(172)
#define RT_GPS_BTN_H     SCALE_H(20)
#define RT_GPS_STATUS_Y  SCALE_Y(193)
#define RT_GPS_HINT_Y    SCALE_Y(204)
#define RT_WIRE_BTN_Y    SCALE_Y(222)
#define RT_WIRE_BTN_H    SCALE_H(20)
#define RT_HINT_Y        SCALE_Y(248)
#define RT_BTN_X          10
#define RT_BTN_W         (SCREEN_WIDTH - 20)

// ═══════════════════════════════════════════════════════════════════════════
// DRAWING HELPERS
// ═══════════════════════════════════════════════════════════════════════════

static void drawRadioButton(int y, int h, const char* label, uint16_t color) {
    tft.fillRect(RT_BTN_X, y, RT_BTN_W, h, TFT_BLACK);
    tft.drawRoundRect(RT_BTN_X, y, RT_BTN_W, h, 4, color);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextSize(1);
    int tw = tft.textWidth(label);
    int tx = RT_BTN_X + (RT_BTN_W - tw) / 2;
    int ty = y + (h - 16) / 2;
    tft.setCursor(tx, ty);
    tft.print(label);
}

static void drawStatusLine(int y, const char* text, uint16_t color) {
    tft.fillRect(0, y, SCREEN_WIDTH, 12, TFT_BLACK);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextFont(1);
    tft.setTextSize(1);
    int tw = tft.textWidth(text);
    int tx = (SCREEN_WIDTH - tw) / 2;
    if (tx < 5) tx = 5;
    tft.setCursor(tx, y);
    tft.print(text);
}

static void drawTestingIndicator(int statusY) {
    drawStatusLine(statusY, "Testing...", TFT_YELLOW);
    // Clear troubleshoot hint line too
    tft.fillRect(0, statusY + 12, SCREEN_WIDTH, 12, TFT_BLACK);
}

// ═══════════════════════════════════════════════════════════════════════════
// SPI HELPERS (same proven patterns as runBootDiagnostics)
// ═══════════════════════════════════════════════════════════════════════════

static void deselectAllCS() {
    pinMode(SD_CS, OUTPUT);       digitalWrite(SD_CS, HIGH);
    pinMode(CC1101_CS, OUTPUT);   digitalWrite(CC1101_CS, HIGH);
    pinMode(NRF24_CSN, OUTPUT);   digitalWrite(NRF24_CSN, HIGH);
    pinMode(NRF24_CE, OUTPUT);    digitalWrite(NRF24_CE, LOW);
}

static void spiReset4MHz() {
    SPI.end();
    delay(10);
    SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI);
    SPI.setFrequency(4000000);
    delay(10);
}

// Raw NRF24 register read (manual CS toggle, no library dependency)
static byte rawNrfRead(byte reg) {
    byte val;
    digitalWrite(NRF24_CSN, LOW);
    delayMicroseconds(5);
    SPI.transfer(reg & 0x1F);    // R_REGISTER command
    val = SPI.transfer(0xFF);
    digitalWrite(NRF24_CSN, HIGH);
    return val;
}

// Raw NRF24 register write (manual CS toggle)
static void rawNrfWrite(byte reg, byte val) {
    digitalWrite(NRF24_CSN, LOW);
    delayMicroseconds(5);
    SPI.transfer((reg & 0x1F) | 0x20);  // W_REGISTER command
    SPI.transfer(val);
    digitalWrite(NRF24_CSN, HIGH);
}

// ═══════════════════════════════════════════════════════════════════════════
// NRF24 TEST — SPI check + spectrum scan + TX test (from Duggie)
// ═══════════════════════════════════════════════════════════════════════════

static void runNrfTest(int statusY, int hintY) {
    drawTestingIndicator(statusY);

    deselectAllCS();
    spiReset4MHz();

    // ── STAGE 1: SPI register check (existing proven logic) ──
    bool statusOK = false;
    byte statusVal = 0x00;
    int nrfDelays[] = {10, 100, 500};

    for (int attempt = 0; attempt < 3; attempt++) {
        delay(nrfDelays[attempt]);
        statusVal = rawNrfRead(0x07);  // 0x07 = STATUS register
        if (statusVal != 0x00 && statusVal != 0xFF) {
            statusOK = true;
            break;
        }
    }

    if (!statusOK) {
        char msg[48];
        if (statusVal == 0x00) {
            snprintf(msg, sizeof(msg), "FAIL  STATUS=0x00 (no power?)");
            drawStatusLine(statusY, msg, TFT_RED);
            { char hint[48]; snprintf(hint, sizeof(hint), "Check 3.3V and CSN wire (GPIO %d)", NRF24_CSN); drawStatusLine(hintY, hint, TFT_YELLOW); }
        } else {
            snprintf(msg, sizeof(msg), "FAIL  STATUS=0xFF (MISO stuck)");
            drawStatusLine(statusY, msg, TFT_RED);
            { char hint[48]; snprintf(hint, sizeof(hint), "Check MISO (GPIO 19) and CSN (GPIO %d)", NRF24_CSN); drawStatusLine(hintY, hint, TFT_YELLOW); }
        }
        return;
    }

    // Step 2: Write/readback test — write 0x3F to EN_AA, read it back
    rawNrfWrite(0x01, 0x3F);            // Write all-pipes-enabled
    delayMicroseconds(10);
    byte readback = rawNrfRead(0x01);   // Read it back
    rawNrfWrite(0x01, 0x00);            // Restore to disabled (our default)

    if (readback != 0x3F) {
        char msg[48];
        snprintf(msg, sizeof(msg), "FAIL  ST=0x%02X WR=0x%02X!=0x3F", statusVal, readback);
        drawStatusLine(statusY, msg, TFT_RED);
        drawStatusLine(hintY, "Check MOSI (GPIO 23) or 3.3V sag", TFT_YELLOW);
        return;
    }

    // SPI is good — now switch to RF24 library for spectrum + TX
    drawStatusLine(statusY, "SPI OK. Spectrum scan...", TFT_CYAN);
    tft.fillRect(0, hintY, SCREEN_WIDTH, 12, TFT_BLACK);

    // Restore SPI bus before RF24 library takes over
    SPI.end();
    delay(10);

    // ── STAGE 2: Spectrum scan (ported from Duggie) ──
    // Use the global nrf24Radio object from nrf24_config.h
    if (!nrf24Radio.begin()) {
        char msg[48];
        snprintf(msg, sizeof(msg), "PASS SPI  FAIL RF24 begin()");
        drawStatusLine(statusY, msg, TFT_YELLOW);
        drawStatusLine(hintY, "SPI works but library init failed", TFT_YELLOW);
        return;
    }

    nrf24Radio.setAutoAck(false);
    nrf24Radio.disableCRC();
    nrf24Radio.setRetries(0, 0);
    nrf24Radio.setPALevel(RF24_PA_LOW);   // Low power for RX scan
    nrf24Radio.setDataRate(RF24_2MBPS);
    nrf24Radio.startListening();

    const int NUM_CHANNELS = 126;
    const int NUM_SWEEPS   = 30;   // 30 sweeps (~4 seconds total)
    uint8_t signalCount[NUM_CHANNELS];
    memset(signalCount, 0, sizeof(signalCount));

    for (int sweep = 0; sweep < NUM_SWEEPS; sweep++) {
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            nrf24Radio.setChannel(ch);
            delayMicroseconds(200);
            nrf24Radio.startListening();
            delayMicroseconds(200);
            nrf24Radio.stopListening();

            if (nrf24Radio.testRPD()) {
                if (signalCount[ch] < 255) signalCount[ch]++;
            }
        }
    }

    nrf24Radio.stopListening();

    int activeChannels = 0;
    uint8_t peakVal = 0;
    int peakCh = 0;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        if (signalCount[ch] > 0) activeChannels++;
        if (signalCount[ch] > peakVal) {
            peakVal = signalCount[ch];
            peakCh = ch;
        }
    }

    // ── STAGE 3: TX attempt (ported from Duggie) ──
    nrf24Radio.setAutoAck(true);
    nrf24Radio.enableDynamicPayloads();
    nrf24Radio.setRetries(5, 15);
    nrf24Radio.setChannel(120);
    nrf24Radio.setPALevel(RF24_PA_LOW);
    nrf24Radio.setDataRate(RF24_1MBPS);

    uint8_t testAddr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24Radio.openWritingPipe(testAddr);
    nrf24Radio.openReadingPipe(1, testAddr);
    nrf24Radio.stopListening();

    uint8_t testPayload[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};
    nrf24Radio.write(testPayload, sizeof(testPayload));
    bool txOK = nrf24Radio.isChipConnected();  // Chip still alive after TX

    nrf24Radio.flush_tx();
    nrf24Radio.flush_rx();

    // Clean up — full reinit then power down
    nrf24Radio.begin();
    nrf24Radio.powerDown();

    // ── RESULTS ──
    char msg[48];
    if (activeChannels > 3 && txOK) {
        snprintf(msg, sizeof(msg), "PASS  %dch peak@%d TX:OK", activeChannels, peakCh);
        drawStatusLine(statusY, msg, TFT_GREEN);
        tft.fillRect(0, hintY, SCREEN_WIDTH, 12, TFT_BLACK);
    } else if (txOK) {
        snprintf(msg, sizeof(msg), "PASS  %dch (quiet) TX:OK", activeChannels);
        drawStatusLine(statusY, msg, TFT_GREEN);
        drawStatusLine(hintY, "Low activity normal if no 2.4G sources", TFT_CYAN);
    } else {
        snprintf(msg, sizeof(msg), "WARN  %dch TX:FAIL", activeChannels);
        drawStatusLine(statusY, msg, TFT_YELLOW);
        drawStatusLine(hintY, "SPI+RX OK but TX attempt failed", TFT_YELLOW);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 TEST — SPI check + spectrum scan + keyfob detection (from Duggie)
// ═══════════════════════════════════════════════════════════════════════════

static void runCC1101Test(int statusY, int hintY) {
    drawTestingIndicator(statusY);

    deselectAllCS();

    // Reset SPI for ELECHOUSE library (it does its own SPI.begin)
    SPI.end();
    delay(10);

    // Deselect NRF24 explicitly (ELECHOUSE won't do it)
    pinMode(NRF24_CSN, OUTPUT);   digitalWrite(NRF24_CSN, HIGH);
    pinMode(SD_CS, OUTPUT);       digitalWrite(SD_CS, HIGH);

    // Pre-check: ELECHOUSE library has blocking while(digitalRead(MISO))
    // that freezes forever if no CC1101 is connected. Safe probe first.
    if (!cc1101SafeCheck()) {
        drawStatusLine(statusY, "FAIL  No CC1101 detected", TFT_RED);
        char hint[48];
        snprintf(hint, sizeof(hint), "Check CS (GPIO %d) and 3.3V power", CC1101_CS);
        drawStatusLine(hintY, hint, TFT_YELLOW);
        return;
    }

    // CC1101 responded on SPI — safe to use ELECHOUSE library now
    ELECHOUSE_cc1101.setSpiPin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

    // Step 1: Check if chip responds on SPI
    bool detected = ELECHOUSE_cc1101.getCC1101();

    if (!detected) {
        drawStatusLine(statusY, "FAIL  No SPI response", TFT_RED);
        char hint[48];
        snprintf(hint, sizeof(hint), "Check CS (GPIO %d) and 3.3V power", CC1101_CS);
        drawStatusLine(hintY, hint, TFT_YELLOW);
        return;
    }

    // Step 2: Read VERSION register (0x31) — genuine CC1101 returns 0x14
    byte version = ELECHOUSE_cc1101.SpiReadStatus(0x31);

    if (version == 0x00 || version == 0xFF) {
        char msg[48];
        snprintf(msg, sizeof(msg), "FAIL  VER=0x%02X", version);
        drawStatusLine(statusY, msg, TFT_RED);
        drawStatusLine(hintY, "Check MISO (GPIO 19) solder joint", TFT_YELLOW);
        return;
    }

    // SPI verified — now do SubGHz spectrum scan (from Duggie)
    drawStatusLine(statusY, "SPI OK. Signal detect...", TFT_CYAN);
    tft.fillRect(0, hintY, SCREEN_WIDTH, 12, TFT_BLACK);

    // ── STAGE 2: Quick signal detection on 315 + 433.92 MHz ──
    // Ported from Duggie's keyfob detection — baseline + listen
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setRxBW(325.00);
    ELECHOUSE_cc1101.SetRx();

    float listenFreqs[] = {315.0, 433.92};
    int numListen = 2;

    // Measure baseline RSSI on each frequency (noise floor)
    int baseline[2] = {-100, -100};
    for (int f = 0; f < numListen; f++) {
        ELECHOUSE_cc1101.setMHZ(listenFreqs[f]);
        delay(10);
        ELECHOUSE_cc1101.SetRx();
        delay(10);
        int sum = 0;
        for (int s = 0; s < 20; s++) {
            sum += ELECHOUSE_cc1101.getRssi();
            delay(2);
        }
        baseline[f] = sum / 20;
    }

    // Listen for 3 seconds for any signal above noise floor
    bool signalDetected = false;
    float detectedFreq = 0;
    int detectedRSSI = -200;
    unsigned long listenStart = millis();
    unsigned long listenTimeout = 3000;   // 3 second quick listen

    while (millis() - listenStart < listenTimeout) {
        for (int f = 0; f < numListen; f++) {
            ELECHOUSE_cc1101.setMHZ(listenFreqs[f]);
            delayMicroseconds(500);
            ELECHOUSE_cc1101.SetRx();
            delay(5);

            for (int s = 0; s < 5; s++) {
                int rssi = ELECHOUSE_cc1101.getRssi();
                if (rssi > baseline[f] + 15 && rssi > -60) {
                    signalDetected = true;
                    detectedFreq = listenFreqs[f];
                    detectedRSSI = rssi;
                    break;
                }
                delayMicroseconds(500);
            }
            if (signalDetected) break;
        }
        if (signalDetected) break;
    }

    ELECHOUSE_cc1101.setSidle();

    // ── RESULTS ──
    char msg[48];
    if (version == 0x14 && signalDetected) {
        snprintf(msg, sizeof(msg), "PASS VER=0x14 %.0fMHz %ddBm", detectedFreq, detectedRSSI);
        drawStatusLine(statusY, msg, TFT_GREEN);
        drawStatusLine(hintY, "Genuine CC1101 + SubGHz signal!", TFT_GREEN);
    } else if (version == 0x14) {
        snprintf(msg, sizeof(msg), "PASS  VER=0x14 (genuine CC1101)");
        drawStatusLine(statusY, msg, TFT_GREEN);
        drawStatusLine(hintY, "No 315/433 signal (normal)", TFT_CYAN);
    } else {
        snprintf(msg, sizeof(msg), "WARN  VER=0x%02X (clone chip?)", version);
        drawStatusLine(statusY, msg, TFT_YELLOW);
        drawStatusLine(hintY, "Works but not genuine TI CC1101", TFT_YELLOW);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GPS TEST — Queries the RUNNING GPS module's diagnostics
// Does NOT open its own UART2 — gps_module.cpp already owns it.
// Just reads the existing counters to verify data is flowing.
// ═══════════════════════════════════════════════════════════════════════════

#if CYD_HAS_GPS

// Draw a HaleHound gradient progress bar (MAGENTA → HOTPINK)
// Extracts RGB565 components at runtime since colors are extern variables.
static void drawGradientBar(int y, int h, float progress) {
    int barX = RT_BTN_X + 2;
    int barW = RT_BTN_W - 4;
    int fillW = (int)(barW * progress);
    if (fillW < 1) fillW = 1;
    if (fillW > barW) fillW = barW;

    // Background (dark)
    tft.fillRect(barX, y, barW, h, HALEHOUND_DARK);

    // Extract RGB565 components from actual theme colors
    uint8_t r1 = (HALEHOUND_MAGENTA >> 11) & 0x1F;
    uint8_t g1 = (HALEHOUND_MAGENTA >> 5)  & 0x3F;
    uint8_t b1 =  HALEHOUND_MAGENTA        & 0x1F;
    uint8_t r2 = (HALEHOUND_HOTPINK >> 11) & 0x1F;
    uint8_t g2 = (HALEHOUND_HOTPINK >> 5)  & 0x3F;
    uint8_t b2 =  HALEHOUND_HOTPINK        & 0x1F;

    // Gradient fill — lerp between the two colors
    for (int x = 0; x < fillW; x++) {
        float t = (float)x / (float)barW;
        uint8_t r = r1 + (uint8_t)((int)(r2 - r1) * t);
        uint8_t g = g1 + (uint8_t)((int)(g2 - g1) * t);
        uint8_t b = b1 + (uint8_t)((int)(b2 - b1) * t);
        uint16_t col = (r << 11) | (g << 5) | b;
        tft.drawFastVLine(barX + x, y, h, col);
    }

    // Border
    tft.drawRect(barX - 1, y - 1, barW + 2, h + 2, HALEHOUND_MAGENTA);
}

static void runGpsTest(int statusY, int hintY) {
    // Clear status area
    tft.fillRect(0, statusY, SCREEN_WIDTH, 24, TFT_BLACK);

    int barY = statusY;
    int barH = 10;

    // ── PHASE 1: Initialize GPS if needed (headless — no screen jump) ──
    Serial.end();           // Free GPIO 3 for UART2
    delay(50);

    drawStatusLine(hintY, "Initializing GPS...", HALEHOUND_GUNMETAL);
    drawGradientBar(barY, barH, 0.1f);

    gpsInitSilent();        // Pin scan + baud detect (returns fast if already init'd)

    drawGradientBar(barY, barH, 0.3f);

    // ── PHASE 2: Open UART2 for data collection ──
    gpsStartBackground();
    delay(500);

    drawGradientBar(barY, barH, 0.4f);
    drawStatusLine(hintY, "Sampling GPS data...", HALEHOUND_GUNMETAL);

    // Snapshot counters BEFORE sampling
    uint32_t charsBefore   = gpsCharsProcessed();
    uint32_t dollarsBefore = gpsDollarsSeen();
    uint32_t passedBefore  = gpsPassedChecksums();

    // ── PHASE 3: Actively feed GPS parser for 3 seconds with animated bar ──
    unsigned long gpsTestStart = millis();
    while (millis() - gpsTestStart < 3000) {
        gpsUpdate();
        float elapsed = (float)(millis() - gpsTestStart) / 3000.0f;
        drawGradientBar(barY, barH, 0.4f + elapsed * 0.5f);  // 0.4 → 0.9
        delay(50);
    }

    drawGradientBar(barY, barH, 1.0f);

    // Snapshot counters AFTER sampling
    uint32_t charsAfter   = gpsCharsProcessed();
    uint32_t dollarsAfter = gpsDollarsSeen();
    uint32_t passedAfter  = gpsPassedChecksums();

    uint32_t newChars   = charsAfter - charsBefore;
    uint32_t newDollars = dollarsAfter - dollarsBefore;
    uint32_t newPassed  = passedAfter - passedBefore;

    int sats = gpsRawSatValue();
    GPSStatus status = gpsGetStatus();
    bool isC5 = gpsIsC5Connected();

    // Close UART2 and clean up
    gpsStopBackground();

    // Brief pause to show full bar
    delay(300);

    // Clear bar area for results
    tft.fillRect(0, statusY, SCREEN_WIDTH, 24, TFT_BLACK);

    // ── RESULTS ──
    char msg[48];

    if (newPassed > 2) {
        if (sats > 0) {
            snprintf(msg, sizeof(msg), "PASS  %lu$ %dsat %s",
                     (unsigned long)newPassed, sats, isC5 ? "C5" : "NMEA");
            drawStatusLine(statusY, msg, TFT_GREEN);
            if (status >= GPS_FIX_2D) {
                drawStatusLine(hintY, "GPS has fix!", TFT_GREEN);
            } else {
                drawStatusLine(hintY, "Searching for fix...", TFT_CYAN);
            }
        } else {
            snprintf(msg, sizeof(msg), "PASS  %lu$ 0sat (cold start?)",
                     (unsigned long)newPassed);
            drawStatusLine(statusY, msg, TFT_GREEN);
            drawStatusLine(hintY, "NMEA OK, waiting for satellites", TFT_CYAN);
        }
    } else if (newDollars > 0) {
        snprintf(msg, sizeof(msg), "WARN  %lu$ (checksum fail?)",
                 (unsigned long)newDollars);
        drawStatusLine(statusY, msg, TFT_YELLOW);
        drawStatusLine(hintY, "Data flowing but NMEA incomplete", TFT_YELLOW);
    } else if (newChars > 0) {
        snprintf(msg, sizeof(msg), "WARN  %lu chars (no NMEA)",
                 (unsigned long)newChars);
        drawStatusLine(statusY, msg, TFT_YELLOW);
        drawStatusLine(hintY, "Bytes but no NMEA — wrong baud?", TFT_YELLOW);
    } else {
        snprintf(msg, sizeof(msg), "FAIL  0 bytes in 3s");
        drawStatusLine(statusY, msg, TFT_RED);
        drawStatusLine(hintY, "Check GPS TX -> CYD P1 RX wiring", TFT_YELLOW);
    }
}

#endif // CYD_HAS_GPS

// ═══════════════════════════════════════════════════════════════════════════
// WIRING DIAGRAMS — Duggie's KiCad layouts as TFT block diagrams
// 5 pages: text reference + NRF24 diagram + GPS diagram + CC1101 diagram + PN532 diagram
// ═══════════════════════════════════════════════════════════════════════════

static int wiringPage = 0;
#define WIRING_NUM_PAGES 6

// Diagram layout constants
#define DIAG_LEFT_X     5
#define DIAG_LEFT_W     SCALE_W(85)
#define DIAG_RIGHT_X    SCALE_X(150)
#define DIAG_RIGHT_W    SCALE_W(85)
#define DIAG_TRACE_X1   (DIAG_LEFT_X + DIAG_LEFT_W)
#define DIAG_TRACE_X2   DIAG_RIGHT_X

// Draw a single pin row with colored trace line between two chip boxes
static void drawPinTrace(int y, const char* leftPin, const char* rightPin,
                          uint16_t color, bool dashed = false) {
    int traceY = y + 4;

    // Left label (right-aligned inside left box)
    tft.setTextColor(color, TFT_BLACK);
    int lw = strlen(leftPin) * 6;
    tft.setCursor(DIAG_LEFT_X + DIAG_LEFT_W - lw - 8, y);
    tft.print(leftPin);

    // Solder dots at box edges
    tft.fillCircle(DIAG_TRACE_X1, traceY, 2, color);
    tft.fillCircle(DIAG_TRACE_X2, traceY, 2, color);

    // Trace line (2px thick for visibility)
    if (dashed) {
        for (int x = DIAG_TRACE_X1 + 4; x < DIAG_TRACE_X2 - 4; x += 8) {
            tft.drawFastHLine(x, traceY, 4, color);
            tft.drawFastHLine(x, traceY + 1, 4, color);
        }
    } else {
        int len = DIAG_TRACE_X2 - DIAG_TRACE_X1 - 6;
        tft.drawFastHLine(DIAG_TRACE_X1 + 3, traceY, len, color);
        tft.drawFastHLine(DIAG_TRACE_X1 + 3, traceY + 1, len, color);
    }

    // Right label (left-aligned inside right box)
    tft.setCursor(DIAG_RIGHT_X + 8, y);
    tft.print(rightPin);
}

// Draw page navigation footer
static void drawPageNav(int page, int total) {
    tft.setTextFont(1);
    tft.setTextSize(1);

    int navY = SCREEN_HEIGHT - 33;

    // Left/right arrows
    tft.setTextColor(HALEHOUND_MAGENTA, TFT_BLACK);
    tft.setCursor(15, navY);
    tft.print("<");
    tft.setCursor(SCREEN_WIDTH - 21, navY);
    tft.print(">");

    // Page number
    char buf[8];
    snprintf(buf, sizeof(buf), "%d/%d", page + 1, total);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    int tw = strlen(buf) * 6;
    tft.setCursor((SCREEN_WIDTH - tw) / 2, navY);
    tft.print(buf);

    // Navigation hint
    tft.setTextColor(HALEHOUND_GUNMETAL, TFT_BLACK);
    tft.setCursor(22, SCREEN_HEIGHT - 15);
    tft.print("TAP </> = Page  BACK = Exit");
}

// ── Page 0: Text wiring reference (original pin lists) ──
static void drawWiringText() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "WIRING");

    tft.setTextFont(1);
    tft.setTextSize(1);
    int y = SCALE_Y(75);
    int lineH = 10;

    // NRF24 section
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(10, y);
    tft.print("--- NRF24L01+PA+LNA ---");
    y += lineH + 2;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, y);  tft.print("VCC  = 3.3V     GND = GND");
    y += lineH;
    tft.setCursor(10, y);  tft.printf("CSN  = GPIO %-3d CE  = GPIO %d", NRF24_CSN, NRF24_CE);
    y += lineH;
    tft.setCursor(10, y);  tft.printf("SCK  = GPIO %-3d MOSI= GPIO %d", VSPI_SCK, VSPI_MOSI);
    y += lineH;
    tft.setCursor(10, y);  tft.printf("MISO = GPIO %-3d IRQ = N/C", VSPI_MISO);
    y += lineH + 4;

    // CC1101 section
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(10, y);
    tft.print("--- CC1101 SubGHz ---");
    y += lineH + 2;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, y);  tft.print("VCC  = 3.3V     GND = GND");
    y += lineH;
    tft.setCursor(10, y);  tft.printf("CS   = GPIO %-3d SCK = GPIO %d", CC1101_CS, VSPI_SCK);
    y += lineH;
    tft.setCursor(10, y);  tft.printf("MOSI = GPIO %-3d MISO= GPIO %d", VSPI_MOSI, VSPI_MISO);
    y += lineH;
    tft.setCursor(10, y);  tft.printf("GDO0 = GPIO %-3d GDO2= GPIO %d", CC1101_GDO0, CC1101_GDO2);
    y += lineH;

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, y);  tft.print("GDO0=TX(out) GDO2=RX(in)");
    y += lineH + 4;

    // PN532 section
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(10, y);
    tft.print("--- PN532 RFID (13.56 MHz) ---");
    y += lineH + 2;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, y);  tft.print("VCC  = 3.3V     GND = GND");
    y += lineH;
    tft.setCursor(10, y);  tft.printf("CS   = GPIO %-3d SCK = GPIO %d", PN532_CS, VSPI_SCK);
    y += lineH;
    tft.setCursor(10, y);  tft.printf("MOSI = GPIO %-3d MISO= GPIO %d", VSPI_MOSI, VSPI_MISO);
    y += lineH;

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, y);  tft.print("DIP: Both switches SPI mode");
    y += lineH + 4;

    // Shared SPI note
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(10, y);
    tft.printf("All radios + SD + PN532 = VSPI  SD CS=%d", SD_CS);

    drawPageNav(0, WIRING_NUM_PAGES);
}

// ── Page 1: NRF24L01+ block diagram ──
static void drawNrf24Diagram() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "NRF24 WIRING");

    tft.setTextFont(1);
    tft.setTextSize(1);

    int pinSpace = SCALE_Y(18);
    int boxY = SCALE_Y(82);
    int boxH = 8 * pinSpace + 22;

    // Left box — ESP32
    tft.drawRect(DIAG_LEFT_X, boxY, DIAG_LEFT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_LEFT_X + 1, boxY + 1, DIAG_LEFT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_LEFT_X + 25, boxY + 4);
    tft.print("ESP32");

    // Right box — NRF24L01+
    tft.drawRect(DIAG_RIGHT_X, boxY, DIAG_RIGHT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_RIGHT_X + 1, boxY + 1, DIAG_RIGHT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_RIGHT_X + 8, boxY + 4);
    tft.print("NRF24L01+");

    // Pin traces — color coded like Duggie's KiCad
    int py = boxY + 20;
    drawPinTrace(py, "3.3V",  "VCC",  TFT_RED,            false);  py += pinSpace;
    drawPinTrace(py, "GND",   "GND",  TFT_WHITE,          false);  py += pinSpace;
    drawPinTrace(py, "IO18",  "SCK",  TFT_CYAN,           false);  py += pinSpace;
    drawPinTrace(py, "IO23",  "MOSI", TFT_CYAN,           false);  py += pinSpace;
    drawPinTrace(py, "IO19",  "MISO", TFT_CYAN,           false);  py += pinSpace;
    { char csnLabel[8]; snprintf(csnLabel, sizeof(csnLabel), "IO%d", NRF24_CSN);
    drawPinTrace(py, csnLabel, "CSN", HALEHOUND_MAGENTA,  false); }  py += pinSpace;
    { char ceLabel[8]; snprintf(ceLabel, sizeof(ceLabel), "IO%d", NRF24_CE);
    drawPinTrace(py, ceLabel, "CE",   HALEHOUND_MAGENTA,  false); }  py += pinSpace;
    drawPinTrace(py, "IO17",  "IRQ",  HALEHOUND_GUNMETAL, true);

    // Notes
    int noteY = boxY + boxH + 6;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(5, noteY);
    { char cnNote[48]; snprintf(cnNote, sizeof(cnNote), "3.3V+GND from CN1 (IO22/IO%d plug)", CC1101_CS); tft.print(cnNote); }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, noteY + 14);
    tft.print("No cap needed from this source!");
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, noteY + 28);
    tft.print("Shares VSPI with CC1101 + SD");

    drawPageNav(1, WIRING_NUM_PAGES);
}

// ── Page 2: GPS block diagram ──
static void drawGpsDiagram() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "GPS WIRING");

    tft.setTextFont(1);
    tft.setTextSize(1);

    int pinSpace = SCALE_Y(22);
    int boxY = SCALE_Y(95);
    int boxH = 4 * pinSpace + 22;

    // Left box — CYD P1 Connector
    tft.drawRect(DIAG_LEFT_X, boxY, DIAG_LEFT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_LEFT_X + 1, boxY + 1, DIAG_LEFT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_LEFT_X + 15, boxY + 4);
    tft.print("CYD  P1");

    // Right box — GT-U7 GPS
    tft.drawRect(DIAG_RIGHT_X, boxY, DIAG_RIGHT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_RIGHT_X + 1, boxY + 1, DIAG_RIGHT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_RIGHT_X + 6, boxY + 4);
    tft.print("GT-U7 GPS");

    // Pin traces
    int py = boxY + 22;
    drawPinTrace(py, "VIN",    "VCC", TFT_RED,            false);  py += pinSpace;
    drawPinTrace(py, "GND",    "GND", TFT_WHITE,          false);  py += pinSpace;
    drawPinTrace(py, "RX IO3", "TX",  TFT_CYAN,           false);  py += pinSpace;
    drawPinTrace(py, "TX IO1", "RX",  TFT_CYAN,          false);

    // Notes
    int noteY = boxY + boxH + 10;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("GPIO3 shared with CH340C USB!");
    noteY += 14;
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("Serial.end() before GPS init");

    drawPageNav(2, WIRING_NUM_PAGES);
}

// ── Page 3: CC1101 block diagram ──
static void drawCC1101Diagram() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "CC1101 HW-863");

    tft.setTextFont(1);
    tft.setTextSize(1);

    int pinSpace = SCALE_Y(18);
    int boxY = SCALE_Y(82);
    int boxH = 8 * pinSpace + 22;

    // Left box — ESP32
    tft.drawRect(DIAG_LEFT_X, boxY, DIAG_LEFT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_LEFT_X + 1, boxY + 1, DIAG_LEFT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_LEFT_X + 25, boxY + 4);
    tft.print("ESP32");

    // Right box — CC1101
    tft.drawRect(DIAG_RIGHT_X, boxY, DIAG_RIGHT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_RIGHT_X + 1, boxY + 1, DIAG_RIGHT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_RIGHT_X + 18, boxY + 4);
    tft.print("CC1101");

    // Pin traces — color coded like Duggie's KiCad
    int py = boxY + 20;
    drawPinTrace(py, "3.3V",  "VCC",     TFT_RED,           false);  py += pinSpace;
    drawPinTrace(py, "GND",   "GND",     TFT_WHITE,         false);  py += pinSpace;
    char csLabel[8];
    snprintf(csLabel, sizeof(csLabel), "IO%d", CC1101_CS);
    drawPinTrace(py, csLabel,  "CS",      HALEHOUND_MAGENTA, false);  py += pinSpace;
    drawPinTrace(py, "IO18",  "SCK",     TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO23",  "MOSI",    TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO19",  "MISO",    TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO22",  "GDO0 TX", HALEHOUND_HOTPINK, false);  py += pinSpace;
    drawPinTrace(py, "IO35",  "GDO2 RX", TFT_YELLOW,        false);

    // Notes
    int noteY = boxY + boxH + 6;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(5, noteY);
    { char cnNote[48]; snprintf(cnNote, sizeof(cnNote), "3.3V+GND from CN1 (IO22/IO%d plug)", CC1101_CS); tft.print(cnNote); }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, noteY + 14);
    tft.print("No cap needed from this source!");
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, noteY + 28);
    tft.print("GDO0=TX(out)  GDO2=RX(in)");
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(5, noteY + 42);
    tft.print("CS=CN1  GDO0/GDO2=P3 connector");

    drawPageNav(3, WIRING_NUM_PAGES);
}

// ── Page 4: CC1101 E07-433M20S PA module block diagram ──
static void drawCC1101PADiagram() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "CC1101 E07-PA");

    tft.setTextFont(1);
    tft.setTextSize(1);

    int pinSpace = SCALE_Y(16);
    int boxY = SCALE_Y(78);
    int boxH = 10 * pinSpace + 22;

    // Left box — ESP32
    tft.drawRect(DIAG_LEFT_X, boxY, DIAG_LEFT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_LEFT_X + 1, boxY + 1, DIAG_LEFT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_LEFT_X + 25, boxY + 4);
    tft.print("ESP32");

    // Right box — E07-433M20S
    tft.drawRect(DIAG_RIGHT_X, boxY, DIAG_RIGHT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_RIGHT_X + 1, boxY + 1, DIAG_RIGHT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_RIGHT_X + 6, boxY + 4);
    tft.print("E07-433M20S");

    // Pin traces — same base as HW-863 plus TX_EN/RX_EN
    int py = boxY + 20;
    drawPinTrace(py, "3.3V",  "VCC",     TFT_RED,           false);  py += pinSpace;
    drawPinTrace(py, "GND",   "GND",     TFT_WHITE,         false);  py += pinSpace;
    { char csLabel[8]; snprintf(csLabel, sizeof(csLabel), "IO%d", CC1101_CS);
    drawPinTrace(py, csLabel,  "CS",      HALEHOUND_MAGENTA, false); }  py += pinSpace;
    drawPinTrace(py, "IO18",  "SCK",     TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO23",  "MOSI",    TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO19",  "MISO",    TFT_CYAN,          false);  py += pinSpace;
    drawPinTrace(py, "IO22",  "GDO0 TX", HALEHOUND_HOTPINK, false);  py += pinSpace;
    drawPinTrace(py, "IO35",  "GDO2 RX", TFT_YELLOW,        false);  py += pinSpace;
#ifdef CC1101_TX_EN
    { char txLabel[8]; snprintf(txLabel, sizeof(txLabel), "IO%d", CC1101_TX_EN);
    drawPinTrace(py, txLabel, "TX_EN",   TFT_GREEN,         false); }  py += pinSpace;
#else
    drawPinTrace(py, "IO26",  "TX_EN",   TFT_GREEN,         false);  py += pinSpace;
#endif
#ifdef CC1101_RX_EN
    { char rxLabel[8]; snprintf(rxLabel, sizeof(rxLabel), "IO%d", CC1101_RX_EN);
    drawPinTrace(py, rxLabel, "RX_EN",   TFT_GREEN,         false); }
#else
    drawPinTrace(py, "IO0",   "RX_EN",   TFT_GREEN,         false);
#endif

    // Notes
    int noteY = boxY + boxH + 4;
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("TX_EN=HIGH:transmit RX_EN=HIGH:receive");
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, noteY + 13);
    tft.print("Enable: Settings > CC1101 > E07 PA");

    drawPageNav(4, WIRING_NUM_PAGES);
}

// ── Page 5: PN532 RFID block diagram ──
static void drawPN532Diagram() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "PN532 WIRING");

    tft.setTextFont(1);
    tft.setTextSize(1);

    int pinSpace = SCALE_Y(22);
    int boxY = SCALE_Y(85);
    int boxH = 6 * pinSpace + 22;

    // Left box — ESP32
    tft.drawRect(DIAG_LEFT_X, boxY, DIAG_LEFT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_LEFT_X + 1, boxY + 1, DIAG_LEFT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_LEFT_X + 25, boxY + 4);
    tft.print("ESP32");

    // Right box — PN532
    tft.drawRect(DIAG_RIGHT_X, boxY, DIAG_RIGHT_W, boxH, HALEHOUND_MAGENTA);
    tft.drawRect(DIAG_RIGHT_X + 1, boxY + 1, DIAG_RIGHT_W - 2, boxH - 2, HALEHOUND_DARK);
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(DIAG_RIGHT_X + 14, boxY + 4);
    tft.print("PN532 V3");

    // Pin traces
    int py = boxY + 22;
    drawPinTrace(py, "3.3V",  "VCC",  TFT_RED,            false);  py += pinSpace;
    drawPinTrace(py, "GND",   "GND",  TFT_WHITE,          false);  py += pinSpace;
    { char csLabel[8]; snprintf(csLabel, sizeof(csLabel), "IO%d", PN532_CS);
    drawPinTrace(py, csLabel, "SS",   HALEHOUND_MAGENTA,  false); }  py += pinSpace;
    drawPinTrace(py, "IO18",  "SCK",  TFT_CYAN,           false);  py += pinSpace;
    drawPinTrace(py, "IO23",  "MOSI", TFT_CYAN,           false);  py += pinSpace;
    drawPinTrace(py, "IO19",  "MISO", TFT_CYAN,           false);

    // Notes
    int noteY = boxY + boxH + 6;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("DIP: Both switches to SPI mode");
    noteY += 14;
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("LSBFIRST SPI (lib handles auto)");
    noteY += 14;
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.print("Shares VSPI with NRF24+CC1101+SD");
    noteY += 14;
    tft.setTextColor(HALEHOUND_HOTPINK, TFT_BLACK);
    tft.setCursor(5, noteY);
    tft.printf("CS=GPIO %d (was NRF24 IRQ, unused)", PN532_CS);

    drawPageNav(5, WIRING_NUM_PAGES);
}

// Page dispatcher
static void drawCurrentWiringPage() {
    switch (wiringPage) {
        case 0:  drawWiringText();    break;
        case 1:  drawNrf24Diagram();  break;
        case 2:  drawGpsDiagram();    break;
        case 3:  drawCC1101Diagram();   break;
        case 4:  drawCC1101PADiagram(); break;
        case 5:  drawPN532Diagram();    break;
        default: drawWiringText();    break;
    }
}

// Multi-page wiring viewer with LEFT/RIGHT navigation
static void showWiringScreen() {
    wiringPage = 0;
    drawCurrentWiringPage();

    bool exitWiring = false;
    while (!exitWiring) {
        touchButtonsUpdate();

        if (isBackButtonTapped() || buttonPressed(BTN_BACK) || buttonPressed(BTN_BOOT)) {
            exitWiring = true;
            break;
        }

        // Right arrow — tap right side of screen
        if (isTouchInArea(SCREEN_WIDTH - 60, SCREEN_HEIGHT - 45, 60, 40) || buttonPressed(BTN_RIGHT)) {
            wiringPage = (wiringPage + 1) % WIRING_NUM_PAGES;
            drawCurrentWiringPage();
            delay(250);
        }

        // Left arrow — tap left side of screen
        if (isTouchInArea(0, SCREEN_HEIGHT - 45, 60, 40) || buttonPressed(BTN_LEFT)) {
            wiringPage = (wiringPage + WIRING_NUM_PAGES - 1) % WIRING_NUM_PAGES;
            drawCurrentWiringPage();
            delay(250);
        }

        delay(20);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN SCREEN
// ═══════════════════════════════════════════════════════════════════════════

static void drawMainScreen() {
    tft.fillScreen(TFT_BLACK);
    drawStatusBar();
    drawInoIconBar();
    drawGlitchTitle(SCALE_Y(60), "RADIO TEST");

    // NRF24 button and status
    drawRadioButton(RT_NRF_BTN_Y, RT_NRF_BTN_H, "[ NRF24L01+ ]", HALEHOUND_MAGENTA);
    drawStatusLine(RT_NRF_STATUS_Y, "Status: --", HALEHOUND_GUNMETAL);

    // CC1101 button and status
    drawRadioButton(RT_CC_BTN_Y, RT_CC_BTN_H, "[ CC1101 ]", HALEHOUND_MAGENTA);
    drawStatusLine(RT_CC_STATUS_Y, "Status: --", HALEHOUND_GUNMETAL);

    // GPS button and status
#if CYD_HAS_GPS
    drawRadioButton(RT_GPS_BTN_Y, RT_GPS_BTN_H, "[ GPS ]", HALEHOUND_MAGENTA);
    drawStatusLine(RT_GPS_STATUS_Y, "Status: --", HALEHOUND_GUNMETAL);
#else
    drawRadioButton(RT_GPS_BTN_Y, RT_GPS_BTN_H, "[ GPS N/A ]", HALEHOUND_GUNMETAL);
    drawStatusLine(RT_GPS_STATUS_Y, "GPS disabled in config", HALEHOUND_GUNMETAL);
#endif

    // Wiring reference button
    drawRadioButton(RT_WIRE_BTN_Y, RT_WIRE_BTN_H, "[ WIRING ]", HALEHOUND_HOTPINK);

    // Hint
    drawCenteredText(RT_HINT_Y, "Tap radio to test", HALEHOUND_HOTPINK, 1);
}

void radioTestScreen() {
    drawMainScreen();

    bool exitRequested = false;

    while (!exitRequested) {
        touchButtonsUpdate();

        // Check back button (icon bar tap or hardware BOOT button)
        if (isBackButtonTapped() || buttonPressed(BTN_BACK) || buttonPressed(BTN_BOOT)) {
            exitRequested = true;
            break;
        }

        // Check NRF24 button tap
        if (isTouchInArea(RT_BTN_X, RT_NRF_BTN_Y, RT_BTN_W, RT_NRF_BTN_H)) {
            drawRadioButton(RT_NRF_BTN_Y, RT_NRF_BTN_H, "[ NRF24L01+ ]", TFT_WHITE);
            delay(100);
            drawRadioButton(RT_NRF_BTN_Y, RT_NRF_BTN_H, "[ NRF24L01+ ]", HALEHOUND_MAGENTA);

            runNrfTest(RT_NRF_STATUS_Y, RT_NRF_HINT_Y);

            tft.fillRect(0, RT_HINT_Y, SCREEN_WIDTH, 14, TFT_BLACK);
            drawCenteredText(RT_HINT_Y, "Tap again to re-test", HALEHOUND_GUNMETAL, 1);

            delay(300);  // Debounce
        }

        // Check CC1101 button tap
        if (isTouchInArea(RT_BTN_X, RT_CC_BTN_Y, RT_BTN_W, RT_CC_BTN_H)) {
            drawRadioButton(RT_CC_BTN_Y, RT_CC_BTN_H, "[ CC1101 ]", TFT_WHITE);
            delay(100);
            drawRadioButton(RT_CC_BTN_Y, RT_CC_BTN_H, "[ CC1101 ]", HALEHOUND_MAGENTA);

            runCC1101Test(RT_CC_STATUS_Y, RT_CC_HINT_Y);

            tft.fillRect(0, RT_HINT_Y, SCREEN_WIDTH, 14, TFT_BLACK);
            drawCenteredText(RT_HINT_Y, "Tap again to re-test", HALEHOUND_GUNMETAL, 1);

            delay(300);  // Debounce
        }

        // Check GPS button tap
#if CYD_HAS_GPS
        if (isTouchInArea(RT_BTN_X, RT_GPS_BTN_Y, RT_BTN_W, RT_GPS_BTN_H)) {
            drawRadioButton(RT_GPS_BTN_Y, RT_GPS_BTN_H, "[ GPS ]", TFT_WHITE);
            delay(100);
            drawRadioButton(RT_GPS_BTN_Y, RT_GPS_BTN_H, "[ GPS ]", HALEHOUND_MAGENTA);

            runGpsTest(RT_GPS_STATUS_Y, RT_GPS_HINT_Y);

            tft.fillRect(0, RT_HINT_Y, SCREEN_WIDTH, 14, TFT_BLACK);
            drawCenteredText(RT_HINT_Y, "Tap again to re-test", HALEHOUND_GUNMETAL, 1);

            delay(300);  // Debounce
        }
#endif

        // Check WIRING button tap
        if (isTouchInArea(RT_BTN_X, RT_WIRE_BTN_Y, RT_BTN_W, RT_WIRE_BTN_H)) {
            drawRadioButton(RT_WIRE_BTN_Y, RT_WIRE_BTN_H, "[ WIRING ]", TFT_WHITE);
            delay(100);

            showWiringScreen();

            // Redraw main screen when returning from wiring
            drawMainScreen();

            delay(300);  // Debounce
        }

        delay(20);
    }

    // Cleanup — restore SPI bus to clean state for spiManager
    SPI.end();
    delay(5);
    SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI);
    deselectAllCS();
}
