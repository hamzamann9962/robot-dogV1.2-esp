/************************************************************
 * ESP32 DevKit V1 — Bluetooth AT validation
 * Edited for: hamza mannai — simple AT test using onboard LED only
 *
 * What it does
 * - Uses Serial (USB) for monitor @ 115200
 * - Uses Serial2 to talk to the BT module (HC-05/HC-06 style) on configurable pins
 * - Sends AT commands and checks for "+OK" in the response
 * - On success: turn LED ON briefly; on failure: blink LED fast
 *
 * Wiring (default pins — change below if needed)
 *   ESP32 TX2 (GPIO17) --> BT RX
 *   ESP32 RX2 (GPIO16) --> BT TX
 *   3V3 / 5V (per module spec) and GND common
 *
 * Notes (hamza mannai)
 * - Some modules enter AT mode only at specific baud (HC-05 default AT=38400),
 *   and sometimes only when KEY pin is held. Adjust BT_BAUD and KEY handling as needed.
 * - Responses often look like: "\r\n+OK\r\n" or "OK". We'll check for "+OK" or "OK".
 ************************************************************/

#include <Arduino.h>

// ===== User config (hamza mannai) =====
#define LED_PIN         2           // onboard LED (GPIO2 on many ESP32 DevKit V1)
#define BT_RX_PIN       16          // ESP32 RX2 (connect to BT TX)
#define BT_TX_PIN       17          // ESP32 TX2 (connect to BT RX)
#define MONITOR_BAUD    115200      // USB Serial
#define BT_BAUD         38400       // Try 38400 first for HC-05 AT mode; change if needed

// Optional: Name/Pin/Baud AT commands
#define BLUE_NAME       "BITTLE"
#define BLUE_PIN        "0000"
#define BLUE_BAUD_CODE  "8"         // HC-05: 4=9600,5=19200,6=38400,7=57600,8=115200,9=128000


HardwareSerial& BT = Serial2;

void led_on()  { digitalWrite(LED_PIN, HIGH); }
void led_off() { digitalWrite(LED_PIN, LOW);  }
void led_blink(int times, int on_ms=80, int off_ms=80) {
  for (int i=0;i<times;i++){ led_on(); delay(on_ms); led_off(); delay(off_ms); }
}

// Send AT command, wait for reply, return true if "+OK" or "OK" found
bool sendAT(const String& cmd, const String& param, uint32_t timeout_ms=500) {
  String full = "AT+" + cmd + param;
  // Flush input
  while (BT.available()) BT.read();
  // Send
  BT.println(full);
  Serial.print(">> "); Serial.println(full);

  uint32_t start = millis();
  String resp;
  while (millis() - start < timeout_ms) {
    while (BT.available()) {
      char c = (char)BT.read();
      resp += c;
    }
    if (resp.indexOf("+OK") >= 0 || resp.indexOf("OK") >= 0) {
      Serial.print("<< "); Serial.println(resp);
      // Success: solid ON briefly
      led_on();
      delay(150);
      led_off();
      return true;
    }
    delay(5);
  }
  Serial.print("<< (timeout) "); Serial.println(resp);
  // Failure: blink fast
  led_blink(3, 60, 60);
  return false;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  led_off();

  Serial.begin(MONITOR_BAUD);
  delay(100);
  Serial.println();
  Serial.println(F("[hamza mannai] ESP32 Bluetooth AT test starting..."));

  // Start Serial2 (BT link)
  BT.begin(BT_BAUD, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
  delay(100);
  Serial.print(F("[hamza mannai] Serial2 started @ "));
  Serial.println(BT_BAUD);

  // Basic probe
  Serial.println(F("[hamza mannai] Probing module (AT)..."));
  // Some modules require "AT" without newline; try both
  BT.print("AT\r\n");
  delay(100);
  String resp;
  while (BT.available()) resp += (char)BT.read();
  Serial.print("<< "); Serial.println(resp);
  if (resp.indexOf("+OK") >= 0 || resp.indexOf("OK") >= 0) { led_on(); delay(150); led_off(); }
  else { led_blink(3); }

  // Example commands (uncomment as needed)
  // sendAT("NAME", String(BLUE_NAME), 800);
  // sendAT("PIN",  String(BLUE_PIN),  800);
  // sendAT("BAUD", String(BLUE_BAUD_CODE), 800);

  Serial.println(F("[hamza mannai] Ready. Type single letters in Serial to trigger demo."));
  Serial.println(F("  n: set NAME   p: set PIN   b: set BAUD   r: AT inquiry"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      sendAT("", "", 600); // plain "AT"
    } else if (c == 'n') {
      sendAT("NAME", String(BLUE_NAME), 800);
    } else if (c == 'p') {
      sendAT("PIN", String(BLUE_PIN), 800);
    } else if (c == 'b') {
      sendAT("BAUD", String(BLUE_BAUD_CODE), 1200);
    }
  }
}
