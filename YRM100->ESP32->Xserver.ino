#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"

// ========================================
// Wi-Fi接続候補（優先順位順）
// 上から順に接続を試す
// ========================================
struct WiFiProfile {
  const char* ssid;
  const char* password;
  const char* label;
};

const WiFiProfile wifiProfiles[] = {
  { "masakiのiPhone",  "masaki3doraemon8",  "iPhoneテザリング" },
  { "31189g",  "nakai18931",   "自宅Wi-Fi" },
};
const int WIFI_PROFILE_COUNT = sizeof(wifiProfiles) / sizeof(wifiProfiles[0]);

int currentProfileIndex = -1;

// ========================================
// Xserver API
// ========================================
const char* API_URL       = "https://xs574402.xsrv.jp/api/scan.php";
const char* KEEPALIVE_URL = "https://xs574402.xsrv.jp/api/keepalive.php";
const char* API_KEY       = "";
const char* DEVICE_ID     = "ESP32-001";

// ========================================
// YRM100 UART
// ========================================
HardwareSerial rfid(1);
const int RX_PIN = 26;
const int TX_PIN = 27;

byte inventoryCmd[] = { 0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E };

// ========================================
// LED設定
// ========================================
#define LED_BLUE_PIN  2
#define LED_RGB_PIN   16
#define LED_COUNT     1
#define LED_CHANNEL   0

Freenove_ESP32_WS2812 rgb = Freenove_ESP32_WS2812(LED_COUNT, LED_RGB_PIN, LED_CHANNEL, TYPE_GRB);

// ========================================
// タイミング設定
// ========================================
const unsigned long READ_INTERVAL_MS        = 1500;
const unsigned long DEDUP_WINDOW_MS         = 10000;
const unsigned long RX_TIMEOUT_MS           = 1000;
const unsigned long RX_INTERBYTE_MS         = 300;
const unsigned long WIFI_CHECK_INTERVAL_MS  = 5000;
const unsigned long KEEPALIVE_INTERVAL_MS   = 10000;
const unsigned long LED_UPDATE_MS           = 100;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;

// ========================================
// 状態管理
// ========================================
enum SysState {
  ST_WIFI_CONNECTING,
  ST_WIFI_DISCONNECTED,
  ST_NORMAL,
  ST_POST_FAIL,
  ST_YRM_NO_RESPONSE,
  ST_FATAL
};

SysState currentState = ST_WIFI_CONNECTING;

bool sending = true;
unsigned long lastReadTime      = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastKeepaliveTime = 0;
unsigned long lastLedUpdateTime = 0;
unsigned long sendCount = 0;
unsigned long readCount = 0;

int wifiRetryCount = 0;
const int WIFI_MAX_RETRY = 10;

int yrmFailCount = 0;
const int YRM_FAIL_THRESHOLD = 5;

int postFailCount = 0;
const int POST_FAIL_THRESHOLD = 3;

// ========================================
// 重複除去
// ========================================
const int DEDUP_SLOTS = 8;
struct DedupEntry {
  String epc;
  unsigned long lastSentMs;
};
DedupEntry dedup[DEDUP_SLOTS];

// ========================================
// 関数プロトタイプ宣言
// ========================================
void connectWiFi();
bool connectWiFiProfiles();
void checkWiFi();
void readAndSend();
void processFrame(byte* frame, int len);
bool isDuplicate(const String& epc);
void markSent(const String& epc);
void sendToXserver(const String& epc, const String& rssi);
void sendKeepalive();
void setRGB(uint8_t r, uint8_t g, uint8_t b);
void updateLED();
void printHelp();

// ========================================
// setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // LED初期化
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  rgb.begin();
  rgb.setBrightness(30);
  setRGB(0, 0, 0);

  // YRM100 UART
  rfid.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  // Wi-Fi接続（優先順位順）
  currentState = ST_WIFI_CONNECTING;
  bool ok = connectWiFiProfiles();

  if (ok) {
    currentState = ST_NORMAL;
    wifiRetryCount = 0;
  } else {
    currentState = ST_WIFI_DISCONNECTED;
  }

  // 重複除去テーブル初期化
  for (int i = 0; i < DEDUP_SLOTS; i++) {
    dedup[i].epc = "";
    dedup[i].lastSentMs = 0;
  }

  printHelp();
}

// ========================================
// loop
// ========================================
void loop() {
  unsigned long now = millis();

  // -------- Sキーで一時停止 / 再開 --------
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      sending = !sending;
      Serial.println();
      if (sending) {
        Serial.println("▶ 送信再開（YRM100読み取り + Xserver POST）");
        lastReadTime = 0;
      } else {
        Serial.println("■ 一時停止");
      }
    }
  }

  // -------- Wi-Fi定期監視 --------
  if (now - lastWifiCheckTime >= WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheckTime = now;
    checkWiFi();
  }

  // -------- キープアライブ送信 --------
  if (now - lastKeepaliveTime >= KEEPALIVE_INTERVAL_MS) {
    lastKeepaliveTime = now;
    if (WiFi.status() == WL_CONNECTED) {
      sendKeepalive();
    }
  }

  // -------- YRM100読み取り + 送信 --------
  if (sending && WiFi.status() == WL_CONNECTED) {
    if (now - lastReadTime >= READ_INTERVAL_MS) {
      lastReadTime = now;
      readAndSend();
    }
  }

  // -------- LED更新 --------
  if (now - lastLedUpdateTime >= LED_UPDATE_MS) {
    lastLedUpdateTime = now;
    updateLED();
  }
}

// ========================================
// Wi-Fi接続（優先順位順に試す）
// ========================================
bool connectWiFiProfiles() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  for (int i = 0; i < WIFI_PROFILE_COUNT; i++) {
    Serial.printf("試行 %d/%d: %s (%s)\n",
                  i + 1, WIFI_PROFILE_COUNT,
                  wifiProfiles[i].label, wifiProfiles[i].ssid);

    WiFi.begin(wifiProfiles[i].ssid, wifiProfiles[i].password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
      delay(200);
      Serial.print(".");
      updateLED();  // LED点滅を維持
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      currentProfileIndex = i;
      Serial.printf("✅ 接続成功: %s\n", wifiProfiles[i].label);
      Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());
      return true;
    } else {
      Serial.printf("❌ 接続失敗: %s\n", wifiProfiles[i].label);
      WiFi.disconnect();
      delay(100);
    }
  }

  currentProfileIndex = -1;
  Serial.println("⚠ 全てのWi-Fiに接続できませんでした");
  return false;
}

// ========================================
// Wi-Fi監視
// ========================================
void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiRetryCount = 0;
    if (currentState == ST_WIFI_CONNECTING ||
        currentState == ST_WIFI_DISCONNECTED) {
      currentState = ST_NORMAL;
    }
    return;
  }

  wifiRetryCount++;
  currentState = ST_WIFI_DISCONNECTED;

  Serial.printf("WiFi disconnected (retry %d/%d)\n",
                wifiRetryCount, WIFI_MAX_RETRY);

  if (wifiRetryCount >= WIFI_MAX_RETRY) {
    currentState = ST_FATAL;
    Serial.println("WiFi reconnect limit reached. FATAL.");
    return;
  }

  // 優先順位順に再接続を試す
  connectWiFiProfiles();
}

// ========================================
// YRM100から読み取ってXserverへ送信
// ========================================
void readAndSend() {
  while (rfid.available()) rfid.read();

  rfid.write(inventoryCmd, sizeof(inventoryCmd));
  rfid.flush();

  byte buf[256];
  int len = 0;
  unsigned long start = millis();
  bool firstByteReceived = false;
  unsigned long firstByteMs = 0;

  while (len < (int)sizeof(buf)) {
    if (rfid.available()) {
      buf[len++] = rfid.read();
      if (!firstByteReceived) {
        firstByteReceived = true;
        firstByteMs = millis();
      }
    }
    if (firstByteReceived && (millis() - firstByteMs > RX_INTERBYTE_MS)) {
      break;
    }
    if (!firstByteReceived && (millis() - start > RX_TIMEOUT_MS)) {
      break;
    }
  }

  readCount++;

  if (len == 0) {
    yrmFailCount++;
    if (yrmFailCount >= YRM_FAIL_THRESHOLD) {
      if (currentState != ST_FATAL && currentState != ST_WIFI_DISCONNECTED) {
        currentState = ST_YRM_NO_RESPONSE;
      }
    }
    return;
  }

  // 応答あり → カウンタリセット
  yrmFailCount = 0;
  if (currentState == ST_YRM_NO_RESPONSE) {
    currentState = ST_NORMAL;
  }

  int i = 0;
  while (i < len) {
    if (buf[i] != 0xBB) {
      i++;
      continue;
    }
    int end = i + 1;
    while (end < len && buf[end] != 0x7E) {
      end++;
    }
    if (end >= len) break;

    int frameLen = end - i + 1;
    processFrame(&buf[i], frameLen);

    i = end + 1;
  }
}

// ========================================
// 1フレームを検証・処理
// ========================================
void processFrame(byte* frame, int len) {
  if (len < 24) return;
  if (frame[0] != 0xBB || frame[len - 1] != 0x7E) return;
  if (frame[1] == 0x01) return;
  if (frame[1] != 0x02) return;

  byte checksum = 0;
  for (int i = 1; i <= len - 3; i++) {
    checksum += frame[i];
  }
  if (checksum != frame[len - 2]) {
    Serial.printf("Checksum mismatch (expected %02X, got %02X)\n",
                  checksum, frame[len - 2]);
    return;
  }

  byte rssiByte = frame[5];
  char rssiStr[5];
  snprintf(rssiStr, sizeof(rssiStr), "%02X", rssiByte);

  char epcStr[25];
  for (int i = 0; i < 12; i++) {
    snprintf(epcStr + i * 2, 3, "%02X", frame[8 + i]);
  }
  epcStr[24] = '\0';

  if (isDuplicate(String(epcStr))) {
    return;
  }

  sendCount++;
  Serial.printf("[%lu] EPC=%s RSSI=%s\n", sendCount, epcStr, rssiStr);
  sendToXserver(String(epcStr), String(rssiStr));
  markSent(String(epcStr));
}

// ========================================
// 重複除去チェック
// ========================================
bool isDuplicate(const String& epc) {
  unsigned long now = millis();
  for (int i = 0; i < DEDUP_SLOTS; i++) {
    if (dedup[i].epc == epc) {
      if (now - dedup[i].lastSentMs < DEDUP_WINDOW_MS) {
        return true;
      }
      return false;
    }
  }
  return false;
}

// ========================================
// 重複除去テーブル更新
// ========================================
void markSent(const String& epc) {
  unsigned long now = millis();

  for (int i = 0; i < DEDUP_SLOTS; i++) {
    if (dedup[i].epc == epc) {
      dedup[i].lastSentMs = now;
      return;
    }
  }

  for (int i = 0; i < DEDUP_SLOTS; i++) {
    if (dedup[i].epc == "") {
      dedup[i].epc = epc;
      dedup[i].lastSentMs = now;
      return;
    }
  }

  int oldestIdx = 0;
  unsigned long oldestMs = dedup[0].lastSentMs;
  for (int i = 1; i < DEDUP_SLOTS; i++) {
    if (dedup[i].lastSentMs < oldestMs) {
      oldestMs = dedup[i].lastSentMs;
      oldestIdx = i;
    }
  }
  dedup[oldestIdx].epc = epc;
  dedup[oldestIdx].lastSentMs = now;
}

// ========================================
// XserverへPOST
// ========================================
void sendToXserver(const String& epc, const String& rssi) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("    WiFi not connected.");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, API_URL);
  http.addHeader("Content-Type", "application/json");
  if (strlen(API_KEY) > 0) {
    http.addHeader("X-API-Key", API_KEY);
  }

  String json = "{\"epc\":\"" + epc + "\",";
  json += "\"rssi\":\"" + rssi + "\",";
  json += "\"device_id\":\"" + String(DEVICE_ID) + "\"}";

  int httpCode = http.POST(json);

  if (httpCode >= 200 && httpCode < 300) {
    String response = http.getString();
    Serial.printf("    HTTP %d | %s\n", httpCode, response.c_str());
    postFailCount = 0;
    if (currentState == ST_POST_FAIL) {
      currentState = ST_NORMAL;
    }
  } else {
    postFailCount++;
    Serial.printf("    POST failed: %s\n", http.errorToString(httpCode).c_str());
    if (postFailCount >= POST_FAIL_THRESHOLD) {
      if (currentState != ST_FATAL) {
        currentState = ST_POST_FAIL;
      }
    }
  }

  http.end();
}

// ========================================
// キープアライブ送信
// ========================================
void sendKeepalive() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, KEEPALIVE_URL);
  http.addHeader("Content-Type", "application/json");
  if (strlen(API_KEY) > 0) {
    http.addHeader("X-API-Key", API_KEY);
  }

  String json = "{\"device_id\":\"" + String(DEVICE_ID) + "\"}";
  int httpCode = http.POST(json);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.println("    Keepalive OK");
  } else {
    Serial.printf("    Keepalive failed: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

// ========================================
// LED制御
// ========================================
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setLedColorData(0, r, g, b);
  rgb.show();
}

void updateLED() {
  static bool blinkOn = false;
  static unsigned long lastBlink = 0;
  unsigned long now = millis();

  switch (currentState) {

    case ST_WIFI_CONNECTING:
      // 青点滅（1Hz）
      if (now - lastBlink > 500) {
        lastBlink = now;
        blinkOn = !blinkOn;
        setRGB(0, 0, blinkOn ? 255 : 0);
      }
      break;

    case ST_WIFI_DISCONNECTED:
      // 青点滅（0.5Hz、ゆっくり）
      if (now - lastBlink > 1000) {
        lastBlink = now;
        blinkOn = !blinkOn;
        setRGB(0, 0, blinkOn ? 255 : 0);
      }
      break;

    case ST_NORMAL:
      // 緑点灯（微弱）+ 青LEDで心拍（2秒に1回）
      setRGB(0, 30, 0);
      if (now - lastBlink > 2000) {
        lastBlink = now;
        digitalWrite(LED_BLUE_PIN, HIGH);
        delay(50);
        digitalWrite(LED_BLUE_PIN, LOW);
      }
      break;

    case ST_POST_FAIL:
      // 橙点滅（1Hz）
      if (now - lastBlink > 500) {
        lastBlink = now;
        blinkOn = !blinkOn;
        setRGB(blinkOn ? 255 : 0, blinkOn ? 100 : 0, 0);
      }
      break;

    case ST_YRM_NO_RESPONSE:
      // 赤点滅（1Hz）
      if (now - lastBlink > 500) {
        lastBlink = now;
        blinkOn = !blinkOn;
        setRGB(blinkOn ? 255 : 0, 0, 0);
      }
      break;

    case ST_FATAL:
      // 赤高速点滅（10Hz）
      if (now - lastBlink > 50) {
        lastBlink = now;
        blinkOn = !blinkOn;
        setRGB(blinkOn ? 255 : 0, 0, 0);
      }
      break;
  }
}

// ========================================
// ヘルプ
// ========================================
void printHelp() {
  Serial.println();
  Serial.println("================================");
  Serial.println(" Wi-Fi接続:");
  for (int i = 0; i < WIFI_PROFILE_COUNT; i++) {
    Serial.printf("   %d. %s (%s)\n", i + 1,
                  wifiProfiles[i].label, wifiProfiles[i].ssid);
  }
  Serial.println("================================");
  Serial.println(" 動作:");
  Serial.println("   起動時 → Wi-Fi接続（上から順）→ 自動送信開始");
  Serial.println(" コマンド:");
  Serial.println("   S  : 一時停止 / 再開 トグル");
  Serial.println("================================");
  Serial.println(" YRM100: UART1 RX=26 TX=27 @115200");
  Serial.println(" 読み取り間隔: 1.5秒");
  Serial.println(" 重複除去: 同一EPCを10秒間再送しない");
  Serial.println(" キープアライブ: 10秒ごと");
  Serial.println();
  Serial.println(" LED診断:");
  Serial.println("   RGB青点滅(1Hz)   : Wi-Fi接続中");
  Serial.println("   RGB青点滅(0.5Hz) : Wi-Fi未接続");
  Serial.println("   RGB緑点灯        : 正常動作中");
  Serial.println("   RGB橙点滅        : POST失敗");
  Serial.println("   RGB赤点滅        : YRM100応答なし");
  Serial.println("   RGB赤高速点滅    : 致命的エラー");
  Serial.println("================================");
}
