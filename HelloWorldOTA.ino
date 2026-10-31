/*
  ESP32-WROOM-DA RC TX: OLED "Hello World" + Wi-Fi Web OTA

  GPIO21 = OLED SDA     GPIO22 = OLED SCL
  GPIO13 = Active-LOW Button 1: hold 3 s to enter/exit OTA
  GPIO18 = Status LED: solid ON normal, 100 ms ON/OFF in OTA
  GPIO23 = Rotary encoder SW (reserved for future TX firmware)
  GPIO14 = NOT USED

  OLED: SSD1306 128x32, I2C address 0x3C or 0x3D
  OTA AP password is randomized every time OTA mode starts and
  displayed on the OLED. The same password authenticates the web page.
  Login username: admin
  OTA address: http://192.168.4.1
  Requires OTA-capable flash partitions and adequate battery charge.

  Arduino libraries: Adafruit SSD1306, Adafruit GFX Library, Adafruit BusIO.
  Arduino ESP32 core: 3.3.x; target esp32:esp32:esp32da (8MB Flash).
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static constexpr uint8_t OLED_SDA = 21;
static constexpr uint8_t OLED_SCL = 22;
static constexpr uint8_t BUTTON_PIN = 13;
static constexpr uint8_t STATUS_LED = 18;
static constexpr uint32_t HOLD_MS = 3000;
static constexpr uint32_t DEBOUNCE_MS = 35;
static constexpr uint32_t BLINK_MS = 100;
static constexpr bool LED_ACTIVE_HIGH = true;

Adafruit_SSD1306 display(128, 32, &Wire, -1);
WebServer server(80);

bool oledOK = false;
bool otaMode = false;
bool otaUploading = false;
bool uploadAuthenticated = false;
bool uploadStarted = false;
bool uploadFailed = false;
bool uploadSuccess = false;
bool restartScheduled = false;
bool ledState = true;
bool buttonRawLast = HIGH;
bool buttonStable = HIGH;
bool pressHandled = false;

uint32_t lastEdgeAt = 0;
uint32_t pressedAt = 0;
uint32_t lastBlinkAt = 0;
uint32_t restartAt = 0;

char apSSID[25] = {};
char apPassword[17] = {};

void setLed(bool on) {
  ledState = on;
  digitalWrite(STATUS_LED, (on == LED_ACTIVE_HIGH) ? HIGH : LOW);
}

void showOLED() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (otaMode) {
    display.println("OTA 192.168.4.1");
    display.print("SSID: "); display.println(apSSID);
    display.print("PASS: "); display.println(apPassword);
    display.println("LOGIN: admin");
  } else {
    display.setCursor(31, 9);
    display.println("Hello World");
    display.setCursor(31, 21);
    display.println("NORMAL MODE");
  }
  display.display();
}

void generateCredentials() {
  const uint32_t seed = esp_random();
  snprintf(apSSID, sizeof(apSSID), "DRIFT-%04lX",
           (unsigned long)(seed & 0xFFFFUL));
  snprintf(apPassword, sizeof(apPassword), "%08lX%04lX",
           (unsigned long)esp_random(),
           (unsigned long)(esp_random() & 0xFFFFUL));
}

const char OTA_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drift TX OTA</title>
<style>body{background:#101827;color:#fff;font:16px sans-serif;
max-width:480px;margin:32px auto;padding:16px}
main{padding:20px;border-radius:12px;background:#263246}
input{width:95%;padding:12px}button{padding:12px;margin-top:14px;
background:#34d399;border:0;border-radius:8px;font-weight:700}</style></head>
<body><main><h2>ESP32-WROOM-DA OTA</h2>
<p>Select compiled <b>.ino.bin</b> firmware.</p>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" required>
<button type="submit">Upload Firmware</button></form>
<p>Do not disconnect power during upload.</p>
</main></body></html>
)HTML";

void onUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    uploadAuthenticated = server.authenticate("admin", apPassword);
    uploadFailed = false;
    uploadSuccess = false;
    uploadStarted = false;
    if (!uploadAuthenticated) {
      Serial.println("[OTA] Unauthorized upload");
      return;
    }
    otaUploading = true;
    if (!upload.filename.endsWith(".bin")) {
      Serial.println("[OTA] File must end in .bin");
      uploadFailed = true;
      return;
    }
    uploadStarted = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
    if (!uploadStarted) {
      uploadFailed = true;
      Serial.println("[OTA] Could not start update (check OTA partitions)");
      Update.printError(Serial);
    } else {
      Serial.printf("[OTA] Receiving: %s\n", upload.filename.c_str());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadAuthenticated && uploadStarted && !uploadFailed) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        uploadFailed = true;
        Update.printError(Serial);
        Update.abort();
        uploadStarted = false;
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadAuthenticated && uploadStarted && !uploadFailed) {
      uploadSuccess = Update.end(true);
      if (uploadSuccess) {
        Serial.printf("[OTA] SUCCESS, bytes=%lu\n",
                      (unsigned long)upload.totalSize);
      } else {
        uploadFailed = true;
        Update.printError(Serial);
      }
    }
    uploadStarted = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (uploadStarted) Update.abort();
    uploadStarted = false;
    uploadFailed = true;
    otaUploading = false;
    Serial.println("[OTA] Upload aborted");
  }
}

void configureWeb() {
  server.on("/", HTTP_GET, []() {
    if (!server.authenticate("admin", apPassword)) {
      server.requestAuthentication();
      return;
    }
    server.send_P(200, "text/html; charset=utf-8", OTA_PAGE);
  });
  server.on("/update", HTTP_POST, []() {
    if (!uploadAuthenticated) {
      otaUploading = false;
      server.requestAuthentication();
      return;
    }
    server.sendHeader("Connection", "close");
    if (uploadSuccess && !uploadFailed) {
      server.send(200, "text/plain",
                  "OTA SUCCESS - ESP32 restarting...");
      restartScheduled = true;
      restartAt = millis() + 1500;
    } else {
      server.send(500, "text/plain",
                  "OTA FAILED - check Serial Monitor at 115200 baud");
    }
    otaUploading = false;
    uploadAuthenticated = false;
  }, onUpload);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
}

void enterOTAMode() {
  generateCredentials();
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(apSSID, apPassword)) {
    Serial.println("[OTA ERROR] Wi-Fi AP did not start");
    WiFi.mode(WIFI_OFF);
    return;
  }
  server.begin();
  otaMode = true;
  lastBlinkAt = millis();
  setLed(false);
  showOLED();
  Serial.printf("[OTA] SSID=%s, PASS=%s, USER=admin\n",
                apSSID, apPassword);
  Serial.printf("[OTA] Web URL=http://%s/\n",
                WiFi.softAPIP().toString().c_str());
}

void leaveOTAMode() {
  if (otaUploading || restartScheduled) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  otaMode = false;
  setLed(true);
  showOLED();
  Serial.println("[NORMAL] OTA disabled");
}

void checkButton(uint32_t now) {
  const bool raw = digitalRead(BUTTON_PIN);
  if (raw != buttonRawLast) {
    buttonRawLast = raw;
    lastEdgeAt = now;
  }
  if (raw != buttonStable &&
      (uint32_t)(now - lastEdgeAt) >= DEBOUNCE_MS) {
    buttonStable = raw;
    if (raw == LOW) {
      pressedAt = now;
      pressHandled = false;
      Serial.println("[BUTTON13] LOW");
    } else {
      pressHandled = false;
      Serial.println("[BUTTON13] HIGH");
    }
  }
  if (buttonStable == LOW && !pressHandled &&
      (uint32_t)(now - pressedAt) >= HOLD_MS) {
    pressHandled = true;  // Must release before next 3-second hold
    if (otaUploading || restartScheduled) return;
    if (otaMode) leaveOTAMode();
    else enterOTAMode();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  setLed(true);
  buttonRawLast = buttonStable = digitalRead(BUTTON_PIN);
  pressedAt = millis();

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  for (uint8_t address : {uint8_t(0x3C), uint8_t(0x3D)}) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0 &&
        display.begin(SSD1306_SWITCHCAPVCC, address)) {
      oledOK = true;
      Serial.printf("[OLED] Found SSD1306 at 0x%02X\n", address);
      break;
    }
  }
  if (!oledOK) {
    Serial.println("[OLED] No SSD1306 at 0x3C/0x3D");
  }

  WiFi.mode(WIFI_OFF);
  configureWeb();
  showOLED();
  Serial.println("[NORMAL] Hello World, hold GPIO13 LOW 3 sec for OTA");
  Serial.println("[NOTE] OTA password shown on OLED after entering OTA");
}

void loop() {
  const uint32_t now = millis();
  checkButton(now);
  if (otaMode) {
    server.handleClient();
    if ((uint32_t)(now - lastBlinkAt) >= BLINK_MS) {
      lastBlinkAt = now;
      setLed(!ledState);
    }
  }
  if (restartScheduled && (int32_t)(millis() - restartAt) >= 0) {
    ESP.restart();
  }
  delay(2);
}
