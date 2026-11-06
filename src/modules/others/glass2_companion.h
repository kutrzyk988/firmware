#pragma once
/**********************************************************************
 *  Glass2 Companion Display
 *  --------------------------------------------------------------
 *  Drives an M5Stack "Unit Glass2" (1.51" transparent SSD1309 OLED,
 *  128x64, I2C addr 0x3C/0x3D) plugged into the Cardputer / Cardputer
 *  ADV Grove port, mirroring a small Bruce status readout onto it:
 *  clock, battery, WiFi/IP and free heap/uptime.
 *
 *  Self-contained: no new platformio.ini dependency (talks SSD1309
 *  directly over bus_HAL's acquireI2CBus()), no changes to
 *  bruceConfig's JSON schema (its own on/off switch lives in its own
 *  Preferences namespace, same pattern used by u2f.cpp).
 *
 *  Lives at: src/modules/others/glass2_companion.{h,cpp}
 *  Wired into: src/main.cpp (taskInputHandler) and
 *  src/core/menu_items/OthersMenu.cpp (the on/off toggle).
 **********************************************************************/

#include <Arduino.h>

namespace Glass2Companion {

// Call once from setup(), any time after bruceConfigPins has been
// loaded from storage. Cheap: only reads the persisted on/off flag,
// never touches I2C.
void setup();

// Call frequently (every pass of taskInputHandler's loop is the
// intended spot, alongside checkAndRecoverSysI2CBus()). Internally
// rate-limited: a no-op almost every call. Handles auto-detect,
// periodic redraw, and noticing when the panel's unplugged.
void loop();

// Runtime on/off switch for the Others-menu toggle. Turning on claims
// bruceConfigPins.i2c_bus via acquireI2CBus() whenever it needs it;
// turning off releases it and forgets the connection. Persisted
// across reboots.
void setEnabled(bool en);
bool isEnabled();

// True once a Glass2 has ACKed on the bus and been initialized.
bool isConnected();

} // namespace Glass2Companion
