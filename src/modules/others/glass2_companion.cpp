/**********************************************************************
 *  Glass2 Companion Display - implementation
 *  See glass2_companion.h for an overview.
 *
 *  Hardware notes
 *  --------------------------------------------------------------
 *  - On m5stack-cardputer (the shared Cardputer / Cardputer ADV build
 *    target) the Grove port is GROVE_SDA=2 / GROVE_SCL=1, which is
 *    exactly what bruceConfigPins.i2c_bus defaults to (see
 *    boards/m5stack-cardputer/m5stack-cardputer.ini and
 *    src/core/configPins.h). We go through acquireI2CBus() rather
 *    than touching Wire directly, so this keeps working if the
 *    person remaps their I2C pins in Config > Dev Mode > I2C Pins,
 *    and plays by the same bus-arbitration rules as the RFID/PN532
 *    code and the I2C Finder tool.
 *  - Unit Glass2 is a bare SSD1309 (no MCU/buttons, unlike Unit Glass
 *    v1), I2C address 0x3C by default, 0x3D if its solder pad is
 *    bridged. We probe both via the existing check_i2c_address().
 *  - SSD1309 modules are normally externally-powered (no internal
 *    charge pump), so the init sequence below omits SSD1306's 0x8D
 *    charge-pump command. If the picture is blank/mirrored/dim on
 *    your particular unit, the first things to try are flipping
 *    0xA1<->0xA0 (segment remap) or 0xC8<->0xC0 (COM scan dir), or
 *    raising the 0x81 contrast byte.
 *
 *  Concurrency note
 *  --------------------------------------------------------------
 *  loop() is called from taskInputHandler, which runs as its own
 *  FreeRTOS task independent of the UI task -- intentionally, so the
 *  companion display keeps updating no matter what menu/tool is on
 *  the main screen. bus_HAL's i2c_bus arbitration (unlike sys_i2c) 
 *  isn't mutex-protected, so if you manually open another Grove-I2C
 *  tool (I2C Finder, PN532/RFID, SRIX) at the exact moment this is
 *  mid-transaction, the worst case is a one-off garbled frame or scan
 *  result -- not a crash or a hang. It self-corrects on the next
 *  cycle. This is a pre-existing property of i2c_bus's arbitration,
 *  not something introduced by this module; it's just that this is
 *  the first caller that lives on a different task.
 *
 *  Bus-sharing caveat
 *  --------------------------------------------------------------
 *  The Grove port is the only expansion port on Cardputer/ADV, and a
 *  few community features (a Grove-wired CC1101, iButton probes)
 *  reuse those same two pins as plain GPIO rather than I2C. Driving
 *  this display will interfere with those if they're plugged into
 *  Grove at the same time. That's why this module defaults OFF and
 *  only touches the bus once explicitly enabled from Others.
 **********************************************************************/

#include "glass2_companion.h"

#include "core/bus_HAL.h"
#include "core/i2c_finder.h"
#include "core/utils.h"
#include <Preferences.h>
#include <ctype.h>
#include <globals.h>
#include <string.h>

namespace Glass2Companion {

namespace {

constexpr uint8_t ADDR_PRIMARY   = 0x3C;
constexpr uint8_t ADDR_SECONDARY = 0x3D;
constexpr uint16_t PANEL_W = 128;
constexpr uint16_t PANEL_H = 64;
constexpr uint8_t  PAGES   = PANEL_H / 8; // 8

constexpr uint32_t PROBE_INTERVAL_MS = 2000;  // how often we look for the panel while absent
constexpr uint32_t RECHECK_MULT      = 3;     // while connected, re-probe every N * PROBE_INTERVAL_MS
constexpr uint32_t DRAW_INTERVAL_MS  = 1000;  // refresh rate of the status screen

Preferences prefs;
bool     _enabled      = false;
bool     _connected    = false;
uint8_t  _addr          = ADDR_PRIMARY;
uint32_t _lastProbe     = 0;
uint32_t _lastDraw      = 0;
uint32_t _recheckTick   = 0;

uint8_t fb[PAGES * PANEL_W]; // 1 byte = 8 vertical px, page-major, matches SSD1309 paging

// 5x7 font, column-encoded (bit0 = top row .. bit6 = bottom row).
// Covers: space - . : % 0-9 A-Z  (enough for the status screen below;
// unsupported characters fall back to space).
const char FONT_ORDER[] = " -.:%0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const uint8_t FONT5x7[][5] = {
            { 0x00, 0x00, 0x00, 0x00, 0x00 }, // 'space'
            { 0x08, 0x08, 0x08, 0x08, 0x08 }, // '-'
            { 0x00, 0x60, 0x60, 0x00, 0x00 }, // '.'
            { 0x00, 0x00, 0x36, 0x00, 0x00 }, // ':'
            { 0x43, 0x33, 0x08, 0x66, 0x61 }, // '%'
            { 0x3E, 0x51, 0x49, 0x45, 0x3E }, // '0'
            { 0x00, 0x42, 0x7F, 0x40, 0x00 }, // '1'
            { 0x42, 0x61, 0x51, 0x49, 0x46 }, // '2'
            { 0x21, 0x41, 0x45, 0x4B, 0x31 }, // '3'
            { 0x18, 0x14, 0x12, 0x7F, 0x10 }, // '4'
            { 0x27, 0x45, 0x45, 0x45, 0x39 }, // '5'
            { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, // '6'
            { 0x01, 0x71, 0x09, 0x05, 0x03 }, // '7'
            { 0x36, 0x49, 0x49, 0x49, 0x36 }, // '8'
            { 0x06, 0x49, 0x49, 0x29, 0x1E }, // '9'
            { 0x7E, 0x09, 0x09, 0x09, 0x7E }, // 'A'
            { 0x7F, 0x49, 0x49, 0x49, 0x36 }, // 'B'
            { 0x3E, 0x41, 0x41, 0x41, 0x41 }, // 'C'
            { 0x7F, 0x41, 0x41, 0x22, 0x1C }, // 'D'
            { 0x7F, 0x49, 0x49, 0x49, 0x41 }, // 'E'
            { 0x7F, 0x09, 0x09, 0x09, 0x01 }, // 'F'
            { 0x3E, 0x41, 0x41, 0x49, 0x79 }, // 'G'
            { 0x7F, 0x08, 0x08, 0x08, 0x7F }, // 'H'
            { 0x00, 0x41, 0x7F, 0x41, 0x00 }, // 'I'
            { 0x20, 0x40, 0x41, 0x3F, 0x01 }, // 'J'
            { 0x7F, 0x08, 0x14, 0x22, 0x41 }, // 'K'
            { 0x7F, 0x40, 0x40, 0x40, 0x40 }, // 'L'
            { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, // 'M'
            { 0x7F, 0x02, 0x0C, 0x10, 0x7F }, // 'N'
            { 0x3E, 0x41, 0x41, 0x41, 0x3E }, // 'O'
            { 0x7F, 0x09, 0x09, 0x09, 0x06 }, // 'P'
            { 0x3E, 0x41, 0x51, 0x21, 0x5E }, // 'Q'
            { 0x7F, 0x09, 0x19, 0x29, 0x46 }, // 'R'
            { 0x46, 0x49, 0x49, 0x49, 0x31 }, // 'S'
            { 0x01, 0x01, 0x7F, 0x01, 0x01 }, // 'T'
            { 0x3F, 0x40, 0x40, 0x40, 0x3F }, // 'U'
            { 0x1F, 0x20, 0x40, 0x20, 0x1F }, // 'V'
            { 0x3F, 0x40, 0x38, 0x40, 0x3F }, // 'W'
            { 0x63, 0x14, 0x08, 0x14, 0x63 }, // 'X'
            { 0x03, 0x04, 0x78, 0x04, 0x03 }, // 'Y'
            { 0x61, 0x51, 0x49, 0x45, 0x43 }, // 'Z'
};

// Probe both possible addresses via the project's own I2C scan helper
// (core/i2c_finder.h), remembering whichever answers in _addr.
bool probePresence() {
    if (check_i2c_address(ADDR_PRIMARY))   { _addr = ADDR_PRIMARY;   return true; }
    if (check_i2c_address(ADDR_SECONDARY)) { _addr = ADDR_SECONDARY; return true; }
    return false;
}

void sendCommand(TwoWire *wire, uint8_t cmd) {
    wire->beginTransmission(_addr);
    wire->write((uint8_t)0x00); // control byte: Co=0, D/C#=0 (command stream)
    wire->write(cmd);
    wire->endTransmission();
}

void sendCommand2(TwoWire *wire, uint8_t cmd, uint8_t arg) {
    wire->beginTransmission(_addr);
    wire->write((uint8_t)0x00);
    wire->write(cmd);
    wire->write(arg);
    wire->endTransmission();
}

// Push one page (128 bytes) of framebuffer data, chunked to stay well
// under typical I2C TX buffer limits.
void sendDataChunk(TwoWire *wire, const uint8_t *data, size_t len) {
    constexpr size_t CHUNK = 32;
    size_t sent = 0;
    while (sent < len) {
        size_t n = min(CHUNK, len - sent);
        wire->beginTransmission(_addr);
        wire->write((uint8_t)0x40); // control byte: Co=0, D/C#=1 (data stream)
        wire->write(data + sent, n);
        wire->endTransmission();
        sent += n;
    }
}

bool initPanel(TwoWire *wire) {
    // Generic SSD1306/1309-compatible init, no charge pump (external VCC).
    sendCommand(wire, 0xAE);        // display off
    sendCommand2(wire, 0xD5, 0x80); // clock divide / osc freq
    sendCommand2(wire, 0xA8, 0x3F); // multiplex ratio = 64
    sendCommand2(wire, 0xD3, 0x00); // display offset = 0
    sendCommand(wire, 0x40);        // display start line = 0
    sendCommand(wire, 0xA1);        // segment remap (flip if mirrored)
    sendCommand(wire, 0xC8);        // COM scan direction (flip if upside down)
    sendCommand2(wire, 0xDA, 0x12); // COM pins hardware config
    sendCommand2(wire, 0x81, 0x7F); // contrast, mid-range
    sendCommand2(wire, 0xD9, 0x22); // pre-charge period
    sendCommand2(wire, 0xDB, 0x20); // VCOMH deselect level
    sendCommand(wire, 0xA4);        // resume to RAM content display
    sendCommand(wire, 0xA6);        // normal (non-inverted)
    sendCommand2(wire, 0x20, 0x02); // memory addressing mode = page addressing
    sendCommand(wire, 0xAF);        // display on
    return true;
}

void clearFb() { memset(fb, 0, sizeof(fb)); }

// Writes one glyph at (page 0-7, col 0-127) into the framebuffer and
// returns the column just past it (5px glyph + 1px spacing).
uint16_t drawChar(uint8_t page, uint16_t col, char ch) {
    if (page >= PAGES || col + 5 >= PANEL_W) return col;
    ch = toupper((unsigned char)ch);
    const char *p = strchr(FONT_ORDER, ch);
    int idx = p ? (int)(p - FONT_ORDER) : 0; // unknown -> space
    const uint8_t *glyph = FONT5x7[idx];
    uint8_t *dst = &fb[page * PANEL_W + col];
    for (int i = 0; i < 5; i++) dst[i] = glyph[i];
    dst[5] = 0x00; // 1px spacer column
    return col + 6;
}

void drawText(uint8_t page, uint16_t col, const char *text) {
    for (const char *c = text; *c; c++) col = drawChar(page, col, *c);
}

void flushFrame(TwoWire *wire) {
    for (uint8_t page = 0; page < PAGES; page++) {
        sendCommand(wire, (uint8_t)(0xB0 + page)); // set page start address
        sendCommand(wire, 0x00);                   // lower column addr = 0
        sendCommand(wire, 0x10);                   // higher column addr = 0
        sendDataChunk(wire, &fb[page * PANEL_W], PANEL_W);
    }
}

void formatUptime(char *out, size_t outLen) {
    uint32_t s = millis() / 1000;
    uint32_t h = s / 3600;
    uint32_t m = (s % 3600) / 60;
    snprintf(out, outLen, "UP %luH%02luM", (unsigned long)h, (unsigned long)m);
}

void buildStatus() {
    clearFb();
    char line[24];

    drawText(0, 0, "BRUCE");

    // Bruce's own clock state (include/globals.h): timeStr is already
    // formatted ("HH:MM:SS" or "HH:MM:SS AM/PM" per bruceConfig.clock24hr),
    // clock_set tells us whether it's actually been synced yet.
    drawText(1, 0, clock_set ? timeStr : "--:--:--");

    snprintf(line, sizeof(line), "BATT %d%%", getBattery());
    drawText(2, 0, line);

    if (wifiConnected) {
        snprintf(line, sizeof(line), "WIFI %s", wifiIP.c_str());
    } else {
        snprintf(line, sizeof(line), "WIFI OFF");
    }
    drawText(3, 0, line);

    snprintf(line, sizeof(line), "HEAP %luKB", (unsigned long)(ESP.getFreeHeap() / 1024));
    drawText(4, 0, line);

    formatUptime(line, sizeof(line));
    drawText(5, 0, line);
}

void drawStatus() {
    TwoWire *wire = acquireI2CBus();
    buildStatus();
    flushFrame(wire);
    releaseI2CBus();
}

} // namespace

void setup() {
    prefs.begin("glass2cmp", false);
    _enabled = prefs.getBool("on", false);
}

void loop() {
    if (!_enabled) return;

    uint32_t now = millis();

    if (!_connected) {
        if (now - _lastProbe >= PROBE_INTERVAL_MS) {
            _lastProbe = now;
            if (probePresence()) {
                TwoWire *wire = acquireI2CBus();
                bool ok = initPanel(wire);
                releaseI2CBus();
                if (ok) {
                    _connected = true;
                    _lastDraw = 0; // force an immediate draw
                    _recheckTick = now;
                }
            }
        }
        return;
    }

    if (now - _lastDraw >= DRAW_INTERVAL_MS) {
        _lastDraw = now;
        drawStatus();
    }

    // Periodically make sure it's still plugged in.
    if (now - _recheckTick >= PROBE_INTERVAL_MS * RECHECK_MULT) {
        _recheckTick = now;
        if (!check_i2c_address(_addr)) { _connected = false; }
    }
}

void setEnabled(bool en) {
    _enabled = en;
    prefs.putBool("on", en);
    if (!en) { _connected = false; }
}

bool isEnabled()   { return _enabled; }
bool isConnected() { return _connected; }

} // namespace Glass2Companion
