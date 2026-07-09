/*
 * @file main.cpp
 * @brief TigerTagScale - Balance connectée ESP32 avec portail captif
 * @version 1.1.0 - Interface web servie depuis LittleFS
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <MFRC522.h>
#include <SPI.h>
#include <LittleFS.h>  // ← AJOUTÉ pour filesystem
#include <Update.h>
#include <esp_wifi.h>
#include <math.h>
#include <memory>
#include <stdarg.h>

// ============================================================================
// CONFIGURATION MATERIELLE
// ============================================================================

// OLED (I2C)
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// RFID RC522 (SPI)
#define RC522_SS    5
#define RC522_RST   27

// HX711 Balance
#define HX711_DOUT  32
#define HX711_SCK   33

// LED Heartbeat
#define LED_PIN     2

// WebSocket update interval (ms)
#define WS_UPDATE_INTERVAL_MS 250

// mDNS
#define MDNS_NAME   "tigerscale"

// mDNS lifecycle helpers
void startMDNS();
void onWiFiEvent(WiFiEvent_t event);

// Unique Setup SSID + mDNS name derived from MAC
String gSetupSsid;     // e.g. Setup-TigerScale-AB12
String gMdnsName;      // e.g. tigerscale-AB12

static String macSuffix4() {
    uint8_t mac[6];
    WiFi.macAddress(mac); // MAC[0]..MAC[5]
    char suf[5]; // 4 hex chars + NUL → last 2 bytes of MAC
    snprintf(suf, sizeof(suf), "%02X%02X", mac[4], mac[5]);
    return String(suf);
}

static String makeSetupSSID() {
    return String("Setup-TigerScale-") + macSuffix4();
}

// ============================================================================
// OBJETS GLOBAUX
// ============================================================================

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
HX711 scale;
MFRC522 rfid(RC522_SS, RC522_RST);
AsyncWebServer server(80);
Preferences prefs;
WiFiManager wm;

// ============================================================================
// VARIABLES DE CONFIGURATION
// ============================================================================

String apiKey = "";
String apiDisplayName = "";     // cached display name for validated API key
bool apiValid = false;          // last known validation state
uint32_t lastApiBroadcastMs = 0; // WS broadcast throttle for apiStatus
struct IntegrationConfig {
    const char* name;
    const char* apiPath;
    bool enabled;
    String url;
    String token;
    String username;
    String password;
};

IntegrationConfig spoolmanConfig = {"Spoolman", "/api/v1", false, "", "", "", ""};
IntegrationConfig filamanConfig = {"Filaman", "/api/v1", false, "", "", "", ""};
IntegrationConfig bambuddyConfig = {"Bambuddy", "/api/v1", false, "", "", "", ""};
float calibrationFactor = 406;
float currentWeight = 0.0;
// --- Hold mode variables ---
bool holdMode = false;
float holdWeight = 0.0f;
uint32_t holdStartMs = 0;
const float HOLD_THRESHOLD_ENTER = 0.5f;
const float HOLD_THRESHOLD_EXIT = 1.5f;
const uint32_t HOLD_TIME_MS = 700;
String lastUID = "";       // decimal UID for API/UI
String lastUIDHex = "";    // hex UID for logs/debug

bool wifiConnected = false;
bool cloudOK = false; // true if health endpoint returns {"ok":true}
bool wifiPortalDeferred = false;
bool wifiPortalActive = false;
uint32_t wifiPortalFallbackAtMs = 0;
const uint32_t WIFI_PORTAL_FALLBACK_DELAY_MS = 120000;
uint32_t lastHealthCheckMs = 0;
uint32_t lastApiValidationMs = 0;
const uint32_t BACKGROUND_CHECK_INTERVAL_MS = 300000;
const uint32_t POST_BOOT_CHECK_DELAY_MS = 5000;

// --- Auto push configuration ---
const float STABLE_EPSILON_G = 1.0f;        // max delta considered stable (g)
const uint32_t STABLE_WINDOW_MS = 1500;     // time window to be stable before sending
const float MIN_WEIGHT_TO_SEND_G = 5.0f;    // ignore tiny weights
const float RESEND_DELTA_G = 2.0f;          // change required to resend (g)
const uint32_t RESEND_COOLDOWN_MS = 15000;  // minimal delay between sends (ms)

// --- Reading stability / smoothing (reduce ±1g flicker; negatives still allowed) ---
const float EMA_ALPHA   = 0.20f;   // exponential moving average factor (0.1..0.3 recommended)
const int   MEDIAN_WINDOW = 5;     // odd number; 5 is a good trade-off

// State
float lastPushedWeight = NAN;
uint32_t stableSinceMs = 0;
float stableCandidate = NAN;
uint32_t lastPushMs = 0;

// --- Filters state (median + EMA) ---
static float gEmaWeight = 0.0f;
static bool  gEmaInit   = false;
static float gMedianBuf[MEDIAN_WINDOW] = {0};
static int   gMedianIdx = 0;
static int   gMedianCount = 0; // <= MEDIAN_WINDOW

// --- UI/Status for auto-send countdown & phase ---
volatile int sendCountdown = -1;         // -1 = no countdown, >=0 = seconds remaining
String sendPhase = "";                  // "" | "countdown" | "send" | "success" | "error"
uint32_t sendPhaseLastChangeMs = 0;      // for expiring transient phases (success/error)
bool spoolmanSyncPending = false;
String spoolmanDisplayLine1 = "";
String spoolmanDisplayLine2 = "";
uint32_t spoolmanDisplayUntilMs = 0;
bool otaRestartPending = false;
uint32_t otaRestartAtMs = 0;
String autoPushUid = "";
uint8_t autoPushAttempts = 0;
const size_t DEVICE_LOG_CAPACITY = 120;
String deviceLogLines[DEVICE_LOG_CAPACITY];
size_t deviceLogStart = 0;
size_t deviceLogCount = 0;

struct TigerTagData {
    bool valid = false;
    uint32_t tigerTagId = 0;
    uint32_t productId = 0;
    uint16_t materialId = 0;
    uint8_t aspect1Id = 0;
    uint8_t aspect2Id = 0;
    uint8_t typeId = 0;
    uint8_t diameterId = 0;
    uint16_t brandId = 0;
    uint8_t colorR = 0;
    uint8_t colorG = 0;
    uint8_t colorB = 0;
    uint8_t colorA = 0;
    uint8_t color2R = 0;
    uint8_t color2G = 0;
    uint8_t color2B = 0;
    uint8_t color2A = 0;
    uint8_t color3R = 0;
    uint8_t color3G = 0;
    uint8_t color3B = 0;
    uint8_t color3A = 0;
    uint32_t weightValue = 0;
    uint8_t weightUnitId = 0;
    uint16_t tempMin = 0;
    uint16_t tempMax = 0;
    uint8_t dryTemp = 0;
    uint8_t dryTime = 0;
};

struct TigerTagProductInfo {
    bool valid = false;
    String name;
    String material;
    String brand;
    String productType;
    String externalId;
    String colorHex;
    float diameterMm = NAN;
    float netWeightG = NAN;
    int extruderTemp = -1;
};

TigerTagData lastTigerTagData;

struct TigerTagResolvedMeta {
    String brandName;
    String materialName;
    String aspect1Name;
};

static String formatLogTimestamp(uint32_t ms) {
    uint32_t totalSeconds = ms / 1000UL;
    uint32_t hours = (totalSeconds / 3600UL) % 100UL;
    uint32_t minutes = (totalSeconds / 60UL) % 60UL;
    uint32_t seconds = totalSeconds % 60UL;
    char buf[16];
    snprintf(buf, sizeof(buf), "[%02lu:%02lu:%02lu]", (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
    return String(buf);
}

static void appendDeviceLogLine(const String& rawLine) {
    String line = rawLine;
    line.replace("\r", "");
    int start = 0;
    while (start <= line.length()) {
        int nl = line.indexOf('\n', start);
        String part = nl >= 0 ? line.substring(start, nl) : line.substring(start);
        part.trim();
        if (part.length() > 0) {
            if (part.length() > 240) {
                part = part.substring(0, 237) + "...";
            }
            String stamped = formatLogTimestamp(millis()) + " " + part;
            size_t slot = (deviceLogStart + deviceLogCount) % DEVICE_LOG_CAPACITY;
            deviceLogLines[slot] = stamped;
            if (deviceLogCount < DEVICE_LOG_CAPACITY) {
                deviceLogCount++;
            } else {
                deviceLogStart = (deviceLogStart + 1) % DEVICE_LOG_CAPACITY;
            }
        }
        if (nl < 0) break;
        start = nl + 1;
    }
}

static void clearDeviceLogs() {
    for (size_t i = 0; i < DEVICE_LOG_CAPACITY; ++i) deviceLogLines[i] = "";
    deviceLogStart = 0;
    deviceLogCount = 0;
}

static void deviceLogln(const String& line) {
    Serial.println(line);
    appendDeviceLogLine(line);
}

static void deviceLogf(const char* fmt, ...) {
    char stackBuf[384];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(stackBuf, sizeof(stackBuf), fmt, args);
    va_end(args);

    String line;
    if (written < 0) {
        line = "[log format error]";
    } else if ((size_t)written < sizeof(stackBuf)) {
        line = String(stackBuf);
    } else {
        std::unique_ptr<char[]> heapBuf(new char[(size_t)written + 1]);
        va_start(args, fmt);
        vsnprintf(heapBuf.get(), (size_t)written + 1, fmt, args);
        va_end(args);
        line = String(heapBuf.get());
    }

    Serial.print(line);
    appendDeviceLogLine(line);
}

// ============================================================================
// AFFICHAGE OLED
// ============================================================================

// 🔎 OLED Display: Utility to show multi-line status/info messages on the SSD1306 screen.
//    Used for user feedback, errors, and setup states.
void displayMessage(String line1, String line2 = "", String line3 = "", String line4 = "") {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);

    display.setCursor(0, 0);
    display.println(line1);

    if (line2.length() > 0) {
        display.setCursor(0, 16);
        display.println(line2);
    }

    if (line3.length() > 0) {
        display.setCursor(0, 32);
        display.println(line3);
    }

    if (line4.length() > 0) {
        display.setCursor(0, 48);
        display.println(line4);
    }

    display.display();
}

// 🔎 OLED Display: Shows the current weight and RFID UID, plus WiFi status, on the OLED.
//    This function is called frequently to update the main UI shown to the user.
String bootStageCode = "";
String bootStageLabel = "";

void setBootStage(const String& code, const String& label, const String& detail = "") {
    bootStageCode = code;
    bootStageLabel = label;
    Serial.printf("[BOOT %s] %s", code.c_str(), label.c_str());
    if (detail.length() > 0) {
        Serial.printf(" | %s", detail.c_str());
    }
    Serial.println();
    displayMessage(
        String("Boot ") + code,
        label,
        detail.length() ? detail : String("uptime ") + String(millis()) + "ms",
        "diag active"
    );
}

void displayWeight(float weight, const String& uid = "");

bool checkServerHealth();
bool pushWeightToCloud(float w, int& codeOut, String& respOut);
void handleAutoPush(float w);
bool validateApiKeyFirmware(const String& key, String& displayNameOut);
bool deleteApiKey();
void showSpoolmanStatus(const String& line1, const String& line2 = "", uint32_t durationMs = 1800);
void runBackgroundConnectivityChecks();
String normalizeSpoolmanBaseUrl(const String& raw);
String buildIntegrationApiBase(const IntegrationConfig& config);
static String formatHttpFailure(int code, const String& resp);
static String jsonEscape(const String& input);
bool isFilamanIntegration(const IntegrationConfig& config);
bool isBambuddyIntegration(const IntegrationConfig& config);
String findSpoolmanSpoolIdByUid(const IntegrationConfig& config, const String& uid, String& errorOut);
bool fetchTigerTagProductInfo(const String& uid, uint32_t productId, TigerTagProductInfo& infoOut, String& errorOut);
bool fetchTigerTagResolvedMeta(const TigerTagData& tagData, TigerTagResolvedMeta& metaOut, String& errorOut);
String findSpoolmanVendorIdByName(const IntegrationConfig& config, const String& vendorName, String& errorOut);
String ensureSpoolmanVendor(const IntegrationConfig& config, const String& vendorName, String& errorOut);
String findSpoolmanFilamentIdByExternalId(const IntegrationConfig& config, const String& externalId, String& errorOut);
String ensureSpoolmanFilament(const IntegrationConfig& config, const String& uid, const TigerTagData& tagData, String& errorOut);
String createSpoolmanSpool(const IntegrationConfig& config, const String& uid, const TigerTagData& tagData, float currentMeasuredWeight, String& errorOut);
bool syncWeightToSpoolman(const IntegrationConfig& config, const String& uid, float w, String& errorOut);
void saveIntegrationConfig();
void resetAutoPushState(bool clearUid);
bool isFilamanIntegration(const IntegrationConfig& config);
bool isBambuddyIntegration(const IntegrationConfig& config);
static bool connectStoredWifi(uint32_t timeoutMs, bool disconnectFirst, const String& statusLine2);
static bool hasStoredWifiCredentials();

static uint8_t chooseTextSizeForWidth(const String& text, uint8_t preferredSize, uint8_t minSize = 1) {
    int16_t x1 = 0, y1 = 0;
    uint16_t w = 0, h = 0;
    for (int size = preferredSize; size >= (int)minSize; --size) {
        display.setTextSize(size);
        display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
        if (w <= OLED_WIDTH) {
            return (uint8_t)size;
        }
    }
    return minSize;
}

static void prepareSpoolmanHttp(HTTPClient& http) {
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
}

struct FilamanSessionState {
    String baseUrl;
    String cookieHeader;
    String csrfToken;
    uint32_t acquiredMs;
};

static FilamanSessionState filamanSession;

static void invalidateFilamanSession() {
    filamanSession.baseUrl = "";
    filamanSession.cookieHeader = "";
    filamanSession.csrfToken = "";
    filamanSession.acquiredMs = 0;
}

static String extractCookieValue(const String& headerValue, const char* name) {
    String needle = String(name) + "=";
    int start = headerValue.indexOf(needle);
    if (start < 0) return "";
    start += needle.length();
    int end = headerValue.indexOf(';', start);
    if (end < 0) end = headerValue.length();
    return headerValue.substring(start, end);
}

static String buildFilamanAuthHeader(const String& rawToken) {
    String token = rawToken;
    token.trim();
    if (token.length() == 0) return "";

    if (token.startsWith("ApiKey ") || token.startsWith("Device ") || token.startsWith("Bearer ")) {
        return token;
    }
    if (token.startsWith("uak.")) {
        return "ApiKey " + token;
    }
    if (token.startsWith("dev.")) {
        return "Device " + token;
    }
    if (token.startsWith("eyJ")) {
        return "Bearer " + token;
    }
    return "ApiKey " + token;
}

static String buildTigerTagHexUid() {
    String hexUid = lastUIDHex;
    hexUid.trim();
    hexUid.toUpperCase();
    return hexUid;
}

static bool ensureFilamanSession(const IntegrationConfig& config, String& errorOut) {
    errorOut = "";
    String baseUrl = normalizeSpoolmanBaseUrl(config.url);
    if (baseUrl.length() == 0) {
        errorOut = "Filaman URL is empty";
        return false;
    }

    const bool hasCredentials = config.username.length() > 0 && config.password.length() > 0;
    if (!hasCredentials) {
        errorOut = "Filaman email/password not configured";
        return false;
    }

    const uint32_t nowMs = millis();
    if (filamanSession.baseUrl == baseUrl &&
        filamanSession.cookieHeader.length() > 0 &&
        filamanSession.csrfToken.length() > 0 &&
        (nowMs - filamanSession.acquiredMs) < (10UL * 60UL * 1000UL)) {
        return true;
    }

    HTTPClient loginHttp;
    String loginUrl = baseUrl + "/auth/login";
    if (!loginHttp.begin(loginUrl)) {
        errorOut = "Failed to connect to Filaman login";
        return false;
    }

    prepareSpoolmanHttp(loginHttp);
    const char* headerKeys[] = {"Set-Cookie"};
    loginHttp.collectHeaders(headerKeys, 1);
    loginHttp.addHeader("Content-Type", "application/json");

    String payload = "{\"email\":\"" + jsonEscape(config.username) + "\",\"password\":\"" + jsonEscape(config.password) + "\"}";
    int code = loginHttp.POST(payload);
    String response = loginHttp.getString();
    String cookieHeaderValue = loginHttp.header("Set-Cookie");
    loginHttp.end();

    if (code < 200 || code >= 300) {
        errorOut = formatHttpFailure(code, response);
        return false;
    }

    String sessionId = extractCookieValue(cookieHeaderValue, "session_id");
    String csrfToken = extractCookieValue(cookieHeaderValue, "csrf_token");
    if (sessionId.length() == 0 || csrfToken.length() == 0) {
        errorOut = "Filaman login did not return session cookies";
        return false;
    }

    filamanSession.baseUrl = baseUrl;
    filamanSession.cookieHeader = "session_id=" + sessionId + "; csrf_token=" + csrfToken;
    filamanSession.csrfToken = csrfToken;
    filamanSession.acquiredMs = nowMs;
    return true;
}

static void addIntegrationAuthHeader(HTTPClient& http, const IntegrationConfig& config) {
    if (isBambuddyIntegration(config)) {
        if (config.token.length() > 0) {
            http.addHeader("X-API-Key", config.token);
        }
        return;
    }

    if (isFilamanIntegration(config)) {
        if (config.token.length() > 0) {
            http.addHeader("Authorization", buildFilamanAuthHeader(config.token));
            return;
        }
        if (filamanSession.cookieHeader.length() > 0) {
            http.addHeader("Cookie", filamanSession.cookieHeader);
        }
        if (filamanSession.csrfToken.length() > 0) {
            http.addHeader("X-CSRF-Token", filamanSession.csrfToken);
        }
        return;
    }

    if (config.token.length() > 0) {
        http.addHeader("Authorization", "Bearer " + config.token);
    }
}

static bool prepareIntegrationRequest(HTTPClient& http, const IntegrationConfig& config, String& errorOut) {
    errorOut = "";
    if (isFilamanIntegration(config)) {
        if (config.token.length() == 0) {
            if (!ensureFilamanSession(config, errorOut)) {
                return false;
            }
        }
    }
    addIntegrationAuthHeader(http, config);
    return true;
}

static bool hasEnabledIntegration() {
    return spoolmanConfig.enabled || filamanConfig.enabled || bambuddyConfig.enabled;
}

bool isFilamanIntegration(const IntegrationConfig& config) {
    return String(config.name) == "Filaman";
}

bool isBambuddyIntegration(const IntegrationConfig& config) {
    return String(config.name) == "Bambuddy";
}

static String formatHttpFailure(int code, const String& resp) {
    String message = String("HTTP ") + code;
    if (code < 0) {
        message += " (" + HTTPClient::errorToString(code) + ")";
    }
    if (resp.length() > 0) {
        message += ": " + resp;
    }
    return message;
}

static String jsonEscape(const String& input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); i++) {
        const char c = input[i];
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out;
}

static uint16_t readU16LE(const uint8_t* data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint16_t readU16BE(const uint8_t* data) {
    return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static uint32_t readU32LE(const uint8_t* data) {
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);
}

static uint32_t readU32BE(const uint8_t* data) {
    return ((uint32_t)data[0] << 24)
        | ((uint32_t)data[1] << 16)
        | ((uint32_t)data[2] << 8)
        | (uint32_t)data[3];
}

static String colorToHex(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    char buf[9];
    snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", r, g, b, a);
    return String(buf);
}

static String colorRgbToHex(uint8_t r, uint8_t g, uint8_t b) {
    char buf[7];
    snprintf(buf, sizeof(buf), "%02X%02X%02X", r, g, b);
    return String(buf);
}

struct NamedColor {
    const char* name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static String nearestColorName(uint8_t r, uint8_t g, uint8_t b) {
    static const NamedColor palette[] = {
        {"Black", 0x16, 0x16, 0x16},
        {"White", 0xF5, 0xF5, 0xF5},
        {"Red", 0xF4, 0x43, 0x36},
        {"Blue", 0x21, 0x96, 0xF3},
        {"Green", 0x4C, 0xAF, 0x50},
        {"Yellow", 0xFF, 0xEB, 0x3B},
        {"Orange", 0xFF, 0x98, 0x00},
        {"Purple", 0x9C, 0x27, 0xB0},
        {"Pink", 0xE9, 0x1E, 0x63},
        {"Brown", 0x79, 0x55, 0x48},
        {"Gray", 0x9E, 0x9E, 0x9E},
        {"Silver", 0xC0, 0xC0, 0xC0}
    };

    uint32_t bestDistance = 0xFFFFFFFFu;
    const char* bestName = "Color";
    for (const NamedColor& candidate : palette) {
        const int dr = (int)r - candidate.r;
        const int dg = (int)g - candidate.g;
        const int db = (int)b - candidate.b;
        const uint32_t distance = (uint32_t)(dr * dr + dg * dg + db * db);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestName = candidate.name;
        }
    }
    return String(bestName);
}

static bool hasMeaningfulRgb(uint8_t r, uint8_t g, uint8_t b) {
    return !(r == 0 && g == 0 && b == 0);
}

static void appendUniqueColorName(String& list, const String& colorName) {
    if (!colorName.length()) return;
    if (list.length() == 0) {
        list = colorName;
        return;
    }

    int start = 0;
    while (start < list.length()) {
        int sep = list.indexOf('/', start);
        if (sep < 0) sep = list.length();
        if (list.substring(start, sep) == colorName) return;
        start = sep + 1;
    }

    list += "/";
    list += colorName;
}

static String buildFilamentNameFromTag(const TigerTagData& tagData, const String& materialName) {
    String primaryColor = nearestColorName(tagData.colorR, tagData.colorG, tagData.colorB);
    String secondaryColor = nearestColorName(tagData.color2R, tagData.color2G, tagData.color2B);
    String tertiaryColor = nearestColorName(tagData.color3R, tagData.color3G, tagData.color3B);
    String name;

    appendUniqueColorName(name, primaryColor);
    if (hasMeaningfulRgb(tagData.color2R, tagData.color2G, tagData.color2B)) {
        appendUniqueColorName(name, secondaryColor);
    }
    if (hasMeaningfulRgb(tagData.color3R, tagData.color3G, tagData.color3B)) {
        appendUniqueColorName(name, tertiaryColor);
    }

    if (materialName.length() > 0) {
        name += " " + materialName;
    }
    return name;
}

static String buildTigerTagExternalId(const String& uid, const TigerTagData& tagData) {
    bool hasValidProductId = !(tagData.productId == 0 || tagData.productId == 0xFFFFFFFFu);
    return hasValidProductId
        ? (String("tigertag:") + String(tagData.productId))
        : (String("tigertag-uid:") + uid);
}

static String getJsonStringValue(const JsonVariantConst& value) {
    if (value.is<const char*>()) return String(value.as<const char*>());
    if (value.is<String>()) return value.as<String>();
    if (value.is<int>()) return String(value.as<int>());
    if (value.is<long>()) return String(value.as<long>());
    if (value.is<float>()) return String(value.as<float>(), 3);
    if (value.is<double>()) return String(value.as<double>(), 3);
    return "";
}

static String getCustomFieldString(const JsonVariantConst& fields, const char* key) {
    if (fields.isNull()) return "";
    return getJsonStringValue(fields[key]);
}

static String decodeSpoolmanExtraValue(const JsonVariantConst& value) {
    if (value.is<const char*>()) {
        String raw = String((const char*)value);
        if (raw.length() >= 2 && raw[0] == '"' && raw[raw.length() - 1] == '"') {
            DynamicJsonDocument doc(256);
            if (deserializeJson(doc, raw) == DeserializationError::Ok && doc.is<const char*>()) {
                return String((const char*)doc.as<const char*>());
            }
        }
        return raw;
    }
    if (value.is<long>() || value.is<int>()) return String(value.as<int>());
    if (value.is<unsigned long>() || value.is<unsigned int>()) return String(value.as<unsigned int>());
    return "";
}

static float inferDensityFromMaterialName(const String& materialName) {
    String m = materialName;
    m.toUpperCase();
    if (m.indexOf("PLA") >= 0) return 1.24f;
    if (m.indexOf("PETG") >= 0) return 1.27f;
    if (m.indexOf("ABS") >= 0) return 1.04f;
    if (m.indexOf("ASA") >= 0) return 1.07f;
    if (m.indexOf("TPU") >= 0) return 1.21f;
    if (m.indexOf("NYLON") >= 0 || m.indexOf("PA") >= 0) return 1.14f;
    if (m.indexOf("PC") >= 0) return 1.20f;
    return 1.24f;
}

static String urlEncode(const String& input) {
    String out;
    const char* hex = "0123456789ABCDEF";
    for (size_t i = 0; i < input.length(); i++) {
        unsigned char c = (unsigned char)input[i];
        if ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[(c >> 4) & 0x0F];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

static void dumpPageBytes(const char* label, const uint8_t* data, size_t len) {
    Serial.printf("[TigerTag] %s:", label);
    for (size_t i = 0; i < len; i++) {
        Serial.printf(" %02X", data[i]);
    }
    Serial.println();
}

static bool readNTAGBlock(byte startPage, uint8_t out[16], String& errorOut) {
    byte buffer[18];
    byte size = sizeof(buffer);
    MFRC522::StatusCode status = rfid.MIFARE_Read(startPage, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        errorOut = String("MIFARE_Read page ") + startPage + " failed: " + rfid.GetStatusCodeName(status);
        return false;
    }
    memcpy(out, buffer, 16);
    return true;
}

static bool readTigerTagData(TigerTagData& dataOut, String& errorOut) {
    errorOut = "";
    uint8_t page4[16];
    uint8_t page8[16];
    uint8_t page12[16];
    if (!readNTAGBlock(4, page4, errorOut)) return false;
    if (!readNTAGBlock(8, page8, errorOut)) return false;
    if (!readNTAGBlock(12, page12, errorOut)) return false;
    dumpPageBytes("pages4-7", page4, 16);
    dumpPageBytes("pages8-11", page8, 16);
    dumpPageBytes("pages12-15", page12, 16);

    TigerTagData data;
    data.valid = true;
    data.tigerTagId = readU32BE(&page4[0]);
    data.productId = readU32BE(&page4[4]);
    data.materialId = readU16BE(&page4[8]);
    data.aspect1Id = page4[10];
    data.aspect2Id = page4[11];
    data.typeId = page4[12];
    data.diameterId = page4[13];
    data.brandId = readU16BE(&page4[14]);
    data.colorR = page8[0];
    data.colorG = page8[1];
    data.colorB = page8[2];
    data.colorA = page8[3];
    data.color2R = page12[4];
    data.color2G = page12[5];
    data.color2B = page12[6];
    data.color2A = page12[7];
    data.color3R = page12[8];
    data.color3G = page12[9];
    data.color3B = page12[10];
    data.color3A = page12[11];
    data.weightValue = ((uint32_t)page8[4] << 16) | ((uint32_t)page8[5] << 8) | (uint32_t)page8[6];
    data.weightUnitId = page8[7];
    data.tempMin = readU16BE(&page8[8]);
    data.tempMax = readU16BE(&page8[10]);
    data.dryTemp = page8[12];
    data.dryTime = page8[13];
    dataOut = data;
    return true;
}

String normalizeSpoolmanBaseUrl(const String& raw) {
    String value = raw;
    value.trim();
    if (value.length() > 0 &&
        value.indexOf("://") < 0 &&
        !value.startsWith("/")) {
        value = "http://" + value;
    }
    while (value.endsWith("/")) value.remove(value.length() - 1);
    return value;
}

String buildIntegrationApiBase(const IntegrationConfig& config) {
    String baseUrl = normalizeSpoolmanBaseUrl(config.url);
    if (baseUrl.length() == 0) return "";

    String normalizedPath = String(config.apiPath);
    while (normalizedPath.endsWith("/")) normalizedPath.remove(normalizedPath.length() - 1);
    if (!normalizedPath.startsWith("/")) normalizedPath = "/" + normalizedPath;

    if (baseUrl.endsWith(normalizedPath)) return baseUrl;
    return baseUrl + normalizedPath;
}

void saveIntegrationConfig() {
    prefs.begin("config", false);
    prefs.putBool("spoolmanEnabled", spoolmanConfig.enabled);
    prefs.putString("spoolmanUrl", spoolmanConfig.url);
    prefs.putString("spoolmanToken", spoolmanConfig.token);
    prefs.putString("spoolmanUsername", spoolmanConfig.username);
    prefs.putString("spoolmanPassword", spoolmanConfig.password);
    prefs.putBool("filamanEnabled", filamanConfig.enabled);
    prefs.putString("filamanUrl", filamanConfig.url);
    prefs.putString("filamanToken", filamanConfig.token);
    prefs.putString("filamanUsername", filamanConfig.username);
    prefs.putString("filamanPassword", filamanConfig.password);
    prefs.putBool("bambuddyEnabled", bambuddyConfig.enabled);
    prefs.putString("bambuddyUrl", bambuddyConfig.url);
    prefs.putString("bambuddyToken", bambuddyConfig.token);
    prefs.end();
}

bool fetchTigerTagProductInfo(const String& uid, uint32_t productId, TigerTagProductInfo& infoOut, String& errorOut) {
    errorOut = "";
    infoOut = TigerTagProductInfo();

    HTTPClient http;
    String url = String("https://api.tigertag.io/api:tigertag/product/get?uid=") + uid +
                 "&product_id=" + String(productId) + "&lang=en";
    Serial.printf("[TigerTag] Fetch product info via %s\n", url.c_str());
    if (!http.begin(url)) {
        errorOut = "TigerTag http begin failed";
        return false;
    }

    int code = http.GET();
    String body = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = String("TigerTag HTTP ") + code + ": " + body;
        return false;
    }

    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        errorOut = String("TigerTag json parse failed: ") + err.c_str();
        return false;
    }

    TigerTagProductInfo info;
    info.valid = true;
    info.externalId = String("tigertag:") + String(productId);

    JsonVariant root = doc.as<JsonVariant>();
    info.name = String((const char*)(root["name"] | ""));
    info.material = String((const char*)(root["material"] | ""));
    info.brand = String((const char*)(root["brand"] | ""));
    info.productType = String((const char*)(root["product_type"] | ""));

    if (root["color"].is<const char*>()) {
        info.colorHex = String((const char*)root["color"]);
        info.colorHex.replace("#", "");
    }
    if (root["measure_value"].is<float>() || root["measure_value"].is<int>()) {
        info.netWeightG = root["measure_value"].as<float>();
    }
    if (root["diameter"].is<float>() || root["diameter"].is<int>()) {
        info.diameterMm = root["diameter"].as<float>();
    }

    infoOut = info;
    return true;
}

static bool fetchTigerTagSimpleName(const String& url, const char* fieldName, String& valueOut, String& errorOut) {
    errorOut = "";
    valueOut = "";
    HTTPClient http;
    if (!http.begin(url)) {
        errorOut = "TigerTag http begin failed";
        return false;
    }
    int code = http.GET();
    String body = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = String("TigerTag HTTP ") + code + ": " + body;
        return false;
    }
    DynamicJsonDocument doc(2048);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        errorOut = String("TigerTag json parse failed: ") + err.c_str();
        return false;
    }
    if (doc[fieldName].is<const char*>()) valueOut = String((const char*)doc[fieldName]);
    if (!valueOut.length() && doc["label"].is<const char*>()) valueOut = String((const char*)doc["label"]);
    return valueOut.length() > 0;
}

bool fetchTigerTagResolvedMeta(const TigerTagData& tagData, TigerTagResolvedMeta& metaOut, String& errorOut) {
    errorOut = "";
    metaOut = TigerTagResolvedMeta();
    String err;
    String brandUrl = String("https://api.tigertag.io/api:tigertag/brand/get?id=") + String(tagData.brandId);
    String materialUrl = String("https://api.tigertag.io/api:tigertag/material/filament/get?id=") + String(tagData.materialId) + "&lang=en";
    String aspectUrl = String("https://api.tigertag.io/api:tigertag/aspect/get?id=") + String(tagData.aspect1Id);

    fetchTigerTagSimpleName(brandUrl, "name", metaOut.brandName, err);
    if (!err.isEmpty()) Serial.printf("[TigerTag] Brand lookup failed for %u: %s\n", (unsigned)tagData.brandId, err.c_str());
    err = "";
    fetchTigerTagSimpleName(materialUrl, "label", metaOut.materialName, err);
    if (!err.isEmpty()) Serial.printf("[TigerTag] Material lookup failed for %u: %s\n", (unsigned)tagData.materialId, err.c_str());
    err = "";
    if (tagData.aspect1Id != 0 && tagData.aspect1Id != 0xFF) {
        fetchTigerTagSimpleName(aspectUrl, "name", metaOut.aspect1Name, err);
        if (!err.isEmpty()) Serial.printf("[TigerTag] Aspect lookup failed for %u: %s\n", (unsigned)tagData.aspect1Id, err.c_str());
    }
    return metaOut.brandName.length() > 0 || metaOut.materialName.length() > 0 || metaOut.aspect1Name.length() > 0;
}

String findSpoolmanVendorIdByName(const IntegrationConfig& config, const String& vendorName, String& errorOut) {
    if (isFilamanIntegration(config)) {
        errorOut = "";
        if (!vendorName.length()) return "";
        const String baseUrl = buildIntegrationApiBase(config);
        for (int page = 1; page <= 20; ++page) {
            HTTPClient http;
            String url = baseUrl + "/manufacturers?page=" + String(page) + "&page_size=200";
            if (!http.begin(url)) {
                errorOut = "http begin failed";
                return "";
            }
            if (!prepareIntegrationRequest(http, config, errorOut)) {
                http.end();
                return "";
            }
            int code = http.GET();
            String resp = http.getString();
            http.end();
            if (code < 200 || code >= 300) {
                errorOut = formatHttpFailure(code, resp);
                return "";
            }

            DynamicJsonDocument doc(32768);
            DeserializationError err = deserializeJson(doc, resp);
            if (err) {
                errorOut = String("json parse failed: ") + err.c_str();
                return "";
            }

            JsonArray items = doc["items"].as<JsonArray>();
            for (JsonObject item : items) {
                String name = getJsonStringValue(item["name"]);
                if (name.equalsIgnoreCase(vendorName)) {
                    return String((int)item["id"]);
                }
            }

            int total = doc["total"] | 0;
            int pageSize = doc["page_size"] | 200;
            if (items.size() == 0 || page * pageSize >= total) break;
        }
        return "";
    }

    errorOut = "";
    if (!vendorName.length()) return "";
    const String baseUrl = buildIntegrationApiBase(config);
    HTTPClient http;
    String url = baseUrl + "/vendor?name=" + urlEncode(vendorName);
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }
    int code = http.GET();
    if (code < 200 || code >= 300) {
        String resp = http.getString();
        http.end();
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }
    DynamicJsonDocument filter(128);
    filter[0]["id"] = true;
    DynamicJsonDocument doc(2048);
    DeserializationError err = deserializeJson(doc, *http.getStreamPtr(), DeserializationOption::Filter(filter));
    http.end();
    if (err) {
        errorOut = String("json parse failed: ") + err.c_str();
        return "";
    }
    JsonArray vendors = doc.as<JsonArray>();
    if (vendors.size() > 0) return String((int)vendors[0]["id"]);
    return "";
}

String ensureSpoolmanVendor(const IntegrationConfig& config, const String& vendorName, String& errorOut) {
    errorOut = "";
    if (!vendorName.length()) return "";
    String vendorId = findSpoolmanVendorIdByName(config, vendorName, errorOut);
    if (vendorId.length() > 0) return vendorId;
    if (errorOut.length() > 0) return "";

    DynamicJsonDocument payload(256);
    payload["name"] = vendorName;
    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = buildIntegrationApiBase(config);
    const String url = isFilamanIntegration(config) ? baseUrl + "/manufacturers" : baseUrl + "/vendor";
    deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }
    int code = http.POST(payloadStr);
    String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }
    DynamicJsonDocument responseDoc(1024);
    DeserializationError createErr = deserializeJson(responseDoc, resp);
    if (createErr) {
        errorOut = String("json parse failed: ") + createErr.c_str();
        return "";
    }
    return String((int)responseDoc["id"]);
}

String findSpoolmanFilamentIdByExternalId(const IntegrationConfig& config, const String& externalId, String& errorOut) {
    errorOut = "";
    const String baseUrl = buildIntegrationApiBase(config);
    if (baseUrl.length() == 0) {
        errorOut = String(config.name) + " url missing";
        return "";
    }

    if (isFilamanIntegration(config)) {
        for (int page = 1; page <= 20; ++page) {
            HTTPClient http;
            String url = baseUrl + "/filaments?page=" + String(page) + "&page_size=200";
            Serial.printf("[%s] Lookup filament via %s\n", config.name, url.c_str());
            if (!http.begin(url)) {
                errorOut = "http begin failed";
                return "";
            }
            if (!prepareIntegrationRequest(http, config, errorOut)) {
                http.end();
                return "";
            }

            int code = http.GET();
            String resp = http.getString();
            http.end();
            if (code < 200 || code >= 300) {
                errorOut = formatHttpFailure(code, resp);
                return "";
            }

            DynamicJsonDocument doc(65536);
            DeserializationError err = deserializeJson(doc, resp);
            if (err) {
                errorOut = String("json parse failed: ") + err.c_str();
                return "";
            }

            JsonArray items = doc["items"].as<JsonArray>();
            for (JsonObject item : items) {
                String cfExternalId = getCustomFieldString(item["custom_fields"], "spoolman_external_id");
                if (cfExternalId == externalId) {
                    return String((int)item["id"]);
                }
            }

            int total = doc["total"] | 0;
            int pageSize = doc["page_size"] | 200;
            if (items.size() == 0 || page * pageSize >= total) break;
        }
        return "";
    }

    HTTPClient http;
    String url = baseUrl + "/filament?external_id=" + urlEncode(String("\"") + externalId + "\"");
    Serial.printf("[%s] Lookup filament via %s\n", config.name, url.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }

    int code = http.GET();
    if (code < 200 || code >= 300) {
        String resp = http.getString();
        http.end();
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }

    DynamicJsonDocument filter(128);
    filter[0]["id"] = true;
    DynamicJsonDocument doc(2048);
    DeserializationError err = deserializeJson(doc, *http.getStreamPtr(), DeserializationOption::Filter(filter));
    http.end();
    if (err) {
        errorOut = String("json parse failed: ") + err.c_str();
        return "";
    }

    JsonArray filaments = doc.as<JsonArray>();
    if (filaments.size() > 0) {
        return String((int)filaments[0]["id"]);
    }
    return "";
}

String ensureSpoolmanFilament(const IntegrationConfig& config, const String& uid, const TigerTagData& tagData, String& errorOut) {
    errorOut = "";
    TigerTagProductInfo productInfo;
    String tigerErr;
    bool hasProductInfo = fetchTigerTagProductInfo(uid, tagData.productId, productInfo, tigerErr);
    if (!hasProductInfo) {
        Serial.printf("[TigerTag] Product lookup failed for uid=%s product_id=%u: %s\n", uid.c_str(), (unsigned)tagData.productId, tigerErr.c_str());
    }

    String externalId = buildTigerTagExternalId(uid, tagData);
    String filamentId = findSpoolmanFilamentIdByExternalId(config, externalId, errorOut);
    if (filamentId.length() > 0) {
        Serial.printf("[%s] Using existing filament %s for %s\n", config.name, filamentId.c_str(), externalId.c_str());
        return filamentId;
    }
    if (errorOut.length() > 0) return "";

    String materialName = hasProductInfo && productInfo.material.length() ? productInfo.material : String("Material ") + String(tagData.materialId);
    String brandName = hasProductInfo && productInfo.brand.length() ? productInfo.brand : "";
    String aspect1Name = "";
    if (!hasProductInfo || !brandName.length() || materialName.startsWith("Material ")) {
        TigerTagResolvedMeta resolvedMeta;
        String metaErr;
        if (fetchTigerTagResolvedMeta(tagData, resolvedMeta, metaErr)) {
            if (!brandName.length() && resolvedMeta.brandName.length()) brandName = resolvedMeta.brandName;
            if (materialName.startsWith("Material ") && resolvedMeta.materialName.length()) materialName = resolvedMeta.materialName;
            aspect1Name = resolvedMeta.aspect1Name;
        }
    }
    String filamentName = buildFilamentNameFromTag(tagData, materialName);
    String colorHex = hasProductInfo && productInfo.colorHex.length() ? productInfo.colorHex : colorToHex(tagData.colorR, tagData.colorG, tagData.colorB, tagData.colorA);
    String primaryRgbHex = colorRgbToHex(tagData.colorR, tagData.colorG, tagData.colorB);
    String secondaryRgbHex = colorRgbToHex(tagData.color2R, tagData.color2G, tagData.color2B);
    String tertiaryRgbHex = colorRgbToHex(tagData.color3R, tagData.color3G, tagData.color3B);
    bool hasSecondColor = hasMeaningfulRgb(tagData.color2R, tagData.color2G, tagData.color2B) && secondaryRgbHex != primaryRgbHex;
    bool hasThirdColor = hasMeaningfulRgb(tagData.color3R, tagData.color3G, tagData.color3B) &&
        tertiaryRgbHex != primaryRgbHex &&
        tertiaryRgbHex != secondaryRgbHex;
    float diameter = !isnan(productInfo.diameterMm) ? productInfo.diameterMm : (tagData.diameterId == 56 ? 1.75f : (tagData.diameterId == 221 ? 2.85f : NAN));
    float netWeight = !isnan(productInfo.netWeightG) ? productInfo.netWeightG : (tagData.weightValue > 0 ? (float)tagData.weightValue : NAN);
    float density = inferDensityFromMaterialName(materialName);
    int extruderTemp = (tagData.tempMin > 0 && tagData.tempMax >= tagData.tempMin) ? (int)((tagData.tempMin + tagData.tempMax) / 2) : -1;
    String vendorId = "";
    if (brandName.length()) {
        String vendorErr;
        vendorId = ensureSpoolmanVendor(config, brandName, vendorErr);
        if (vendorErr.length()) Serial.printf("[%s] Vendor ensure failed for %s: %s\n", config.name, brandName.c_str(), vendorErr.c_str());
    }
    if (isFilamanIntegration(config) && vendorId.length() == 0) {
        String vendorErr;
        vendorId = ensureSpoolmanVendor(config, brandName.length() ? brandName : "TigerTag", vendorErr);
        if (vendorErr.length()) Serial.printf("[%s] Fallback manufacturer ensure failed: %s\n", config.name, vendorErr.c_str());
    }

    DynamicJsonDocument payload(2048);
    if (isFilamanIntegration(config)) {
        if (!vendorId.length()) {
            errorOut = "manufacturer missing";
            return "";
        }
        payload["manufacturer_id"] = vendorId.toInt();
        payload["designation"] = filamentName;
        payload["material_type"] = materialName;
        if (!isnan(diameter)) payload["diameter_mm"] = diameter;
        if (brandName.length()) payload["manufacturer_color_name"] = filamentName;
        if (!isnan(netWeight) && netWeight > 0) payload["raw_material_weight_g"] = netWeight;
        if (density > 0) payload["density_g_cm3"] = density;
        JsonObject customFields = payload.createNestedObject("custom_fields");
        customFields["spoolman_external_id"] = externalId;
    } else {
        payload["name"] = filamentName;
        payload["material"] = materialName;
        payload["external_id"] = externalId;
        if (aspect1Name.length()) payload["comment"] = aspect1Name;
        if (vendorId.length()) payload["vendor_id"] = vendorId.toInt();
        if (hasSecondColor || hasThirdColor) {
            String multiColors = primaryRgbHex;
            if (hasSecondColor) multiColors += "," + secondaryRgbHex;
            if (hasThirdColor) multiColors += "," + tertiaryRgbHex;
            payload["multi_color_hexes"] = multiColors;
            payload["multi_color_direction"] = "longitudinal";
        } else {
            payload["color_hex"] = colorHex;
        }
        payload["density"] = density;
        if (!isnan(diameter)) payload["diameter"] = diameter;
        if (!isnan(netWeight) && netWeight > 0) payload["weight"] = netWeight;
        if (extruderTemp > 0) payload["settings_extruder_temp"] = extruderTemp;
    }

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = buildIntegrationApiBase(config);
    const String url = isFilamanIntegration(config) ? baseUrl + "/filaments" : baseUrl + "/filament";
    deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }
    int code = http.POST(payloadStr);
    String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }

    DynamicJsonDocument responseDoc(2048);
    DeserializationError createErr = deserializeJson(responseDoc, resp);
    if (createErr) {
        errorOut = String("json parse failed: ") + createErr.c_str();
        return "";
    }

    String createdFilamentId = String((int)responseDoc["id"]);
    Serial.printf("[%s] Created filament %s for %s\n", config.name, createdFilamentId.c_str(), externalId.c_str());
    return createdFilamentId;
}

String createSpoolmanSpool(const IntegrationConfig& config, const String& uid, const TigerTagData& tagData, float currentMeasuredWeight, String& errorOut) {
    String filamentId = ensureSpoolmanFilament(config, uid, tagData, errorOut);
    if (filamentId.length() == 0) return "";

    String externalId = buildTigerTagExternalId(uid, tagData);

    DynamicJsonDocument payload(1024);
    payload["filament_id"] = filamentId.toInt();
    int currentWeight = (int)(currentMeasuredWeight + (currentMeasuredWeight >= 0 ? 0.5f : -0.5f));
    if (currentWeight < 0) currentWeight = 0;
    if (isFilamanIntegration(config)) {
        payload["remaining_weight_g"] = currentWeight;
        payload["rfid_uid"] = uid;
        payload["external_id"] = externalId;
        if (tagData.weightValue > 0) payload["initial_total_weight_g"] = tagData.weightValue;
        JsonObject customFields = payload.createNestedObject("custom_fields");
        customFields["spoolman_external_id"] = externalId;
    } else {
        payload["remaining_weight"] = currentWeight;
        if (tagData.weightValue > 0) payload["initial_weight"] = tagData.weightValue;
        JsonObject extra = payload.createNestedObject("extra");
        extra["rfid_uid"] = uid;
    }

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = buildIntegrationApiBase(config);
    const String url = isFilamanIntegration(config) ? baseUrl + "/spools" : baseUrl + "/spool";
    deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }
    int code = http.POST(payloadStr);
    String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }

    DynamicJsonDocument responseDoc(2048);
    DeserializationError createErr = deserializeJson(responseDoc, resp);
    if (createErr) {
        errorOut = String("json parse failed: ") + createErr.c_str();
        return "";
    }

    String spoolId = String((int)responseDoc["id"]);
    deviceLogf("[%s] Created spool %s for UID %s\n", config.name, spoolId.c_str(), uid.c_str());
    return spoolId;
}

static bool findBambuddySpoolByUid(
    const IntegrationConfig& config,
    const String& uid,
    String& spoolIdOut,
    int& labelWeightOut,
    String& errorOut
) {
    spoolIdOut = "";
    labelWeightOut = 1000;
    errorOut = "";

    const String baseUrl = buildIntegrationApiBase(config);
    if (baseUrl.length() == 0) {
        errorOut = String(config.name) + " url missing";
        return false;
    }

    String tagUidHex = buildTigerTagHexUid();
    if (tagUidHex.length() == 0) {
        errorOut = "missing tag hex uid";
        return false;
    }

    HTTPClient http;
    const String url = baseUrl + "/inventory/spools/by-tag/" + urlEncode(tagUidHex);
    deviceLogf("[%s] Lookup UID %s (hex %s) via %s\n", config.name, uid.c_str(), tagUidHex.c_str(), url.c_str());
    prepareSpoolmanHttp(http);
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return false;
    }
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return false;
    }

    const int code = http.GET();
    const String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        if (code != 404 && code != 405) {
            errorOut = formatHttpFailure(code, resp);
            return false;
        }
    }

    if (code >= 200 && code < 300) {
        DynamicJsonDocument doc(2048);
        DeserializationError err = deserializeJson(doc, resp);
        if (err) {
            errorOut = String("json parse failed: ") + err.c_str();
            return false;
        }

        String spoolTagUid = getJsonStringValue(doc["tag_uid"]);
        spoolTagUid.toUpperCase();
        if (spoolTagUid != tagUidHex) {
            errorOut = "bambuddy by-tag lookup returned mismatched tag_uid";
            return false;
        }

        spoolIdOut = String((int)doc["id"]);
        labelWeightOut = doc["label_weight"] | 1000;
        return true;
    }

    // Fallback for older Bambuddy versions that may not expose /by-tag/.
    HTTPClient fallbackHttp;
    const String fallbackUrl = baseUrl + "/inventory/spools?include_archived=true";
    deviceLogf("[%s] Fallback lookup UID %s (hex %s) via %s\n", config.name, uid.c_str(), tagUidHex.c_str(), fallbackUrl.c_str());
    prepareSpoolmanHttp(fallbackHttp);
    if (!fallbackHttp.begin(fallbackUrl)) {
        errorOut = "http begin failed";
        return false;
    }
    if (!prepareIntegrationRequest(fallbackHttp, config, errorOut)) {
        fallbackHttp.end();
        return false;
    }

    const int fallbackCode = fallbackHttp.GET();
    const String fallbackResp = fallbackHttp.getString();
    fallbackHttp.end();
    if (fallbackCode < 200 || fallbackCode >= 300) {
        errorOut = formatHttpFailure(fallbackCode, fallbackResp);
        return false;
    }

    DynamicJsonDocument filter(256);
    JsonObject filterItem = filter.createNestedObject();
    filterItem["id"] = true;
    filterItem["tag_uid"] = true;
    filterItem["label_weight"] = true;

    DynamicJsonDocument doc(12288);
    DeserializationError err = deserializeJson(doc, fallbackResp, DeserializationOption::Filter(filter));
    if (err) {
        errorOut = String("json parse failed: ") + err.c_str();
        return false;
    }

    JsonArray spools = doc.as<JsonArray>();
    for (JsonObject spool : spools) {
        String spoolTagUid = getJsonStringValue(spool["tag_uid"]);
        spoolTagUid.toUpperCase();
        if (spoolTagUid == tagUidHex) {
            spoolIdOut = String((int)spool["id"]);
            labelWeightOut = spool["label_weight"] | 1000;
            return true;
        }
    }

    errorOut = "no spool with bambuddy tag_uid";
    return false;
}

static String createBambuddySpool(
    const IntegrationConfig& config,
    const String& uid,
    const TigerTagData& tagData,
    float currentMeasuredWeight,
    String& errorOut
) {
    errorOut = "";
    TigerTagProductInfo productInfo;
    String tigerErr;
    bool hasProductInfo = fetchTigerTagProductInfo(uid, tagData.productId, productInfo, tigerErr);

    String materialName = hasProductInfo && productInfo.material.length() ? productInfo.material : String("Material ") + String(tagData.materialId);
    String brandName = hasProductInfo && productInfo.brand.length() ? productInfo.brand : "";
    String aspect1Name = "";
    if (!hasProductInfo || !brandName.length() || materialName.startsWith("Material ")) {
        TigerTagResolvedMeta resolvedMeta;
        String metaErr;
        if (fetchTigerTagResolvedMeta(tagData, resolvedMeta, metaErr)) {
            if (!brandName.length() && resolvedMeta.brandName.length()) brandName = resolvedMeta.brandName;
            if (materialName.startsWith("Material ") && resolvedMeta.materialName.length()) materialName = resolvedMeta.materialName;
            aspect1Name = resolvedMeta.aspect1Name;
        }
    }

    String colorHex = hasProductInfo && productInfo.colorHex.length()
        ? productInfo.colorHex
        : colorToHex(tagData.colorR, tagData.colorG, tagData.colorB, tagData.colorA);
    colorHex.toUpperCase();
    String colorName;
    appendUniqueColorName(colorName, nearestColorName(tagData.colorR, tagData.colorG, tagData.colorB));
    if (hasMeaningfulRgb(tagData.color2R, tagData.color2G, tagData.color2B)) {
        appendUniqueColorName(colorName, nearestColorName(tagData.color2R, tagData.color2G, tagData.color2B));
    }
    if (hasMeaningfulRgb(tagData.color3R, tagData.color3G, tagData.color3B)) {
        appendUniqueColorName(colorName, nearestColorName(tagData.color3R, tagData.color3G, tagData.color3B));
    }
    int labelWeight = !isnan(productInfo.netWeightG) && productInfo.netWeightG > 0
        ? (int)(productInfo.netWeightG + 0.5f)
        : (tagData.weightValue > 0 ? (int)tagData.weightValue : 1000);
    int remainingWeight = (int)(currentMeasuredWeight + (currentMeasuredWeight >= 0 ? 0.5f : -0.5f));
    if (remainingWeight < 0) remainingWeight = 0;
    int usedWeight = labelWeight - remainingWeight;
    if (usedWeight < 0) usedWeight = 0;

    DynamicJsonDocument payload(1536);
    payload["material"] = materialName;
    if (aspect1Name.length()) payload["subtype"] = aspect1Name;
    if (colorName.length()) payload["color_name"] = colorName;
    if (brandName.length()) payload["brand"] = brandName;
    if (colorHex.length() == 8) payload["rgba"] = colorHex;
    payload["label_weight"] = labelWeight;
    payload["weight_used"] = usedWeight;
    payload["weight_locked"] = true;
    payload["tag_uid"] = buildTigerTagHexUid();
    payload["data_origin"] = "tigertag";
    if (hasProductInfo && productInfo.name.length()) {
        payload["note"] = productInfo.name + " | TigerTag UID " + uid;
    } else {
        payload["note"] = "TigerTag UID " + uid;
    }

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = buildIntegrationApiBase(config);
    const String url = baseUrl + "/inventory/spools";
    deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }
    int code = http.POST(payloadStr);
    String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = formatHttpFailure(code, resp);
        return "";
    }

    DynamicJsonDocument responseDoc(4096);
    DeserializationError createErr = deserializeJson(responseDoc, resp);
    if (createErr) {
        errorOut = String("json parse failed: ") + createErr.c_str();
        return "";
    }
    return String((int)responseDoc["id"]);
}

static String createBambuddyPlaceholderSpool(
    const IntegrationConfig& config,
    const String& uid,
    float currentMeasuredWeight,
    String& errorOut
) {
    errorOut = "";

    const String tagUidHex = buildTigerTagHexUid();
    if (tagUidHex.length() == 0) {
        errorOut = "missing tag hex uid";
        return "";
    }

    int labelWeight = (int)(currentMeasuredWeight + (currentMeasuredWeight >= 0 ? 0.5f : -0.5f));
    if (labelWeight <= 0) labelWeight = 1;

    DynamicJsonDocument payload(512);
    payload["material"] = "Unknown";
    payload["label_weight"] = labelWeight;
    payload["weight_used"] = 0;
    payload["weight_locked"] = true;
    payload["tag_uid"] = tagUidHex;
    payload["data_origin"] = "tigertag";
    payload["note"] = "TigerTag UID " + uid;

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = buildIntegrationApiBase(config);
    const String url = baseUrl + "/inventory/spools";
    deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }

    int code = http.POST(payloadStr);
    String resp = http.getString();
    http.end();
    if (code < 200 || code >= 300) {
        errorOut = formatHttpFailure(code, resp);
        return "";
    }

    DynamicJsonDocument responseDoc(2048);
    DeserializationError createErr = deserializeJson(responseDoc, resp);
    if (createErr) {
        errorOut = String("json parse failed: ") + createErr.c_str();
        return "";
    }

    String spoolId = String((int)responseDoc["id"]);
    deviceLogf("[%s] Created placeholder spool %s for UID %s\n", config.name, spoolId.c_str(), uid.c_str());
    return spoolId;
}

String findSpoolmanSpoolIdByUid(const IntegrationConfig& config, const String& uid, String& errorOut) {
    errorOut = "";
    const String baseUrl = buildIntegrationApiBase(config);
    if (baseUrl.length() == 0) {
        errorOut = String(config.name) + " url missing";
        return "";
    }

    if (isBambuddyIntegration(config)) {
        String spoolId;
        int labelWeight = 1000;
        if (findBambuddySpoolByUid(config, uid, spoolId, labelWeight, errorOut)) {
            return spoolId;
        }
        return "";
    }

    if (isFilamanIntegration(config)) {
        String externalId = buildTigerTagExternalId(uid, lastTigerTagData);
        for (int page = 1; page <= 20; ++page) {
            HTTPClient http;
            const String url = baseUrl + "/spools?page=" + String(page) + "&page_size=200&include_archived=true";
            deviceLogf("[%s] Lookup UID %s via %s\n", config.name, uid.c_str(), url.c_str());
            prepareSpoolmanHttp(http);
            if (!http.begin(url)) {
                errorOut = "http begin failed";
                return "";
            }

            if (!prepareIntegrationRequest(http, config, errorOut)) {
                http.end();
                return "";
            }
            const int code = http.GET();
            const String resp = http.getString();
            http.end();
            if (code < 200 || code >= 300) {
                errorOut = formatHttpFailure(code, resp);
                return "";
            }

            DynamicJsonDocument doc(131072);
            DeserializationError err = deserializeJson(doc, resp);
            if (err) {
                errorOut = String("json parse failed: ") + err.c_str();
                return "";
            }

            JsonArray spools = doc["items"].as<JsonArray>();
            for (JsonObject spool : spools) {
                String spoolUid = getJsonStringValue(spool["rfid_uid"]);
                String spoolExternalId = getJsonStringValue(spool["external_id"]);
                String filamentExternalId = getCustomFieldString(spool["filament"]["custom_fields"], "spoolman_external_id");
                if (spoolUid == uid || spoolExternalId == externalId || filamentExternalId == externalId) {
                    String spoolId = String((int)spool["id"]);
                    bool needsPatch = spoolUid.length() == 0 || spoolExternalId.length() == 0;
                    if (needsPatch) {
                        HTTPClient patchHttp;
                        String patchUrl = baseUrl + "/spools/" + spoolId;
                        if (!patchHttp.begin(patchUrl)) {
                            errorOut = "http begin failed";
                            return "";
                        }
                        patchHttp.addHeader("Content-Type", "application/json");
                        if (!prepareIntegrationRequest(patchHttp, config, errorOut)) {
                            patchHttp.end();
                            return "";
                        }
                        String patchPayload = String("{\"rfid_uid\":\"") + jsonEscape(uid) + "\",\"external_id\":\"" + jsonEscape(externalId) + "\"}";
                        int patchCode = patchHttp.PATCH(patchPayload);
                        String patchResp = patchHttp.getString();
                        patchHttp.end();
                        if (patchCode < 200 || patchCode >= 300) {
                            errorOut = formatHttpFailure(patchCode, patchResp);
                            return "";
                        }
                    }
                    return spoolId;
                }
            }

            int total = doc["total"] | 0;
            int pageSize = doc["page_size"] | 200;
            if (spools.size() == 0 || page * pageSize >= total) break;
        }

        errorOut = "no spool with filaman identifiers";
        return "";
    }

    HTTPClient http;
    const String url = baseUrl + "/spool";
    deviceLogf("[%s] Lookup UID %s via %s\n", config.name, uid.c_str(), url.c_str());
    prepareSpoolmanHttp(http);
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }

    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return "";
    }

    const int code = http.GET();
    if (code < 200 || code >= 300) {
        const String resp = http.getString();
        http.end();
        errorOut = formatHttpFailure(code, resp);
        return "";
    }

    const String resp = http.getString();
    http.end();

    DynamicJsonDocument filter(256);
    filter[0]["id"] = true;
    filter[0]["extra"]["rfid_uid"] = true;

    DynamicJsonDocument doc(32768);
    DeserializationError err = deserializeJson(doc, resp, DeserializationOption::Filter(filter));
    if (err) {
        errorOut = String("json parse failed: ") + err.c_str();
        return "";
    }

    JsonArray spools = doc.as<JsonArray>();
    Serial.printf("[%s] Lookup response contains %u spool entries\n", config.name, (unsigned)spools.size());
    for (JsonObject spool : spools) {
        String extraUid = decodeSpoolmanExtraValue(spool["extra"]["rfid_uid"]);
        extraUid.trim();
        if (extraUid == uid) {
            String spoolId = String((int)spool["id"]);
            Serial.printf("[%s] UID %s matched spool %s\n", config.name, uid.c_str(), spoolId.c_str());
            return spoolId;
        }
    }

    errorOut = "no spool with extra_rfid_uid";
    return "";
}

bool syncWeightToSpoolman(const IntegrationConfig& config, const String& uid, float w, String& errorOut) {
    errorOut = "";
    if (!wifiConnected || !WiFi.isConnected()) {
        errorOut = "wifi disconnected";
        return false;
    }

    const String baseUrl = buildIntegrationApiBase(config);
    if (baseUrl.length() == 0) {
        errorOut = String(config.name) + " url missing";
        return false;
    }

    if (isBambuddyIntegration(config)) {
        String spoolId;
        int labelWeight = 1000;
        if (!findBambuddySpoolByUid(config, uid, spoolId, labelWeight, errorOut)) {
            if (errorOut != "no spool with bambuddy tag_uid") {
                return false;
            }
            if (lastTigerTagData.valid) {
                deviceLogf("[%s] No spool found for UID %s, creating one from TigerTag data\n", config.name, uid.c_str());
                spoolId = createBambuddySpool(config, uid, lastTigerTagData, w, errorOut);
                labelWeight = lastTigerTagData.weightValue > 0 ? (int)lastTigerTagData.weightValue : 1000;
            } else {
                deviceLogf("[%s] No spool found for UID %s, creating placeholder from UID/weight only\n", config.name, uid.c_str());
                spoolId = createBambuddyPlaceholderSpool(config, uid, w, errorOut);
                labelWeight = (int)(w + (w >= 0 ? 0.5f : -0.5f));
                if (labelWeight <= 0) labelWeight = 1;
            }
            if (spoolId.length() == 0) return false;
        }

        int remainingWeight = (int)(w + (w >= 0 ? 0.5f : -0.5f));
        if (remainingWeight < 0) remainingWeight = 0;
        int usedWeight = labelWeight - remainingWeight;
        if (usedWeight < 0) usedWeight = 0;

        HTTPClient http;
        const String url = baseUrl + "/inventory/spools/" + spoolId;
        prepareSpoolmanHttp(http);
        if (!http.begin(url)) {
            errorOut = "http begin failed";
            return false;
        }
        http.addHeader("Content-Type", "application/json");
        if (!prepareIntegrationRequest(http, config, errorOut)) {
            http.end();
            return false;
        }
        String payload = String("{\"weight_used\":") + String(usedWeight) + ",\"weight_locked\":true}";
        deviceLogf("[%s] PATCH %s payload=%s\n", config.name, url.c_str(), payload.c_str());
        int code = http.PATCH(payload);
        String resp = http.getString();
        http.end();
        if (code < 200 || code >= 300) {
            errorOut = formatHttpFailure(code, resp);
            return false;
        }
        return true;
    }

    if (isFilamanIntegration(config)) {
        String externalId = buildTigerTagExternalId(uid, lastTigerTagData);
        auto postMeasurement = [&](String& measurementError) -> bool {
            HTTPClient http;
            const String url = baseUrl + "/spool-measurements";
            prepareSpoolmanHttp(http);
            if (!http.begin(url)) {
                measurementError = "http begin failed";
                return false;
            }
            http.addHeader("Content-Type", "application/json");
            if (!prepareIntegrationRequest(http, config, measurementError)) {
                http.end();
                return false;
            }
            const int weightInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
            const int clampedWeight = weightInt < 0 ? 0 : weightInt;
            String payload = String("{\"rfid_uid\":\"") + jsonEscape(uid) +
                "\",\"external_id\":\"" + jsonEscape(externalId) +
                "\",\"measured_weight_g\":" + String(clampedWeight) + "}";
            deviceLogf("[%s] POST %s payload=%s\n", config.name, url.c_str(), payload.c_str());
            int code = http.POST(payload);
            String resp = http.getString();
            http.end();
            if (code >= 200 && code < 300) return true;
            measurementError = formatHttpFailure(code, resp);
            return false;
        };

        String measurementError;
        if (postMeasurement(measurementError)) return true;

        String lookupError;
        String spoolId = findSpoolmanSpoolIdByUid(config, uid, lookupError);
        if (spoolId.length() == 0) {
            if (!lastTigerTagData.valid) {
                errorOut = measurementError.length() ? measurementError : lookupError;
                return false;
            }
            Serial.printf("[%s] No Filaman spool found for UID %s, creating one from TigerTag data\n", config.name, uid.c_str());
            spoolId = createSpoolmanSpool(config, uid, lastTigerTagData, w, lookupError);
            if (spoolId.length() == 0) {
                errorOut = lookupError.length() ? lookupError : measurementError;
                return false;
            }
        }

        if (postMeasurement(measurementError)) return true;
        errorOut = measurementError;
        return false;
    }

    const String spoolId = findSpoolmanSpoolIdByUid(config, uid, errorOut);
    String resolvedSpoolId = spoolId;
    if (resolvedSpoolId.length() == 0) {
        if (errorOut == "no spool with extra_rfid_uid") {
            if (!lastTigerTagData.valid) {
                errorOut = "no spool with extra_rfid_uid and no TigerTag payload available";
                return false;
            }
            deviceLogf("[%s] No spool found for UID %s, creating one from TigerTag data\n", config.name, uid.c_str());
            resolvedSpoolId = createSpoolmanSpool(config, uid, lastTigerTagData, w, errorOut);
            if (resolvedSpoolId.length() == 0) return false;
        } else {
            return false;
        }
    }

    HTTPClient http;
    const String url = baseUrl + "/spool/" + resolvedSpoolId;
    prepareSpoolmanHttp(http);
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    if (!prepareIntegrationRequest(http, config, errorOut)) {
        http.end();
        return false;
    }

    const int weightInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
    const int clampedWeight = weightInt < 0 ? 0 : weightInt;
    String payload = String("{\"remaining_weight\":") + String(clampedWeight) + "}";
    deviceLogf("[%s] PATCH %s payload=%s\n", config.name, url.c_str(), payload.c_str());
    const int code = http.PATCH(payload);
    const String resp = http.getString();
    http.end();

    if (code >= 200 && code < 300) return true;

    errorOut = formatHttpFailure(code, resp);
    return false;
}

// 🔎 OLED Display: Main function for rendering weight and tag info on the OLED.
//    Shows WiFi status, weight (large digits), UID, and device IP.
void displayWeight(float weight, const String& uid) {
    display.clearDisplay();
    const bool showingStatus = spoolmanDisplayUntilMs > millis() && spoolmanDisplayLine1.length() > 0;
    display.setTextWrap(false);
    
     // En-tête avec titre et statut WiFi
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Tiger-Scale");
    
    display.setTextSize(1);
    display.setCursor(80, 0);
    display.println(wifiConnected ? "WiFi" : "----");

    // Hold mode indicator (🅗 at x=112, y=0)
    if (holdMode) { display.setCursor(120, 0); display.print("H"); }
    
    // Poids au centre (grande taille) — entier uniquement
    if (showingStatus) {
        display.setTextSize(chooseTextSizeForWidth(spoolmanDisplayLine1, 2, 1));
        display.setCursor(0, 20);
        display.println(spoolmanDisplayLine1);
        if (spoolmanDisplayLine2.length() > 0) {
            display.setTextSize(1);
            display.setCursor(0, 46);
            display.println(spoolmanDisplayLine2);
        }
        display.display();
        return;
    }

    int wInt = (int)(weight + (weight >= 0 ? 0.5f : -0.5f));
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(wInt);
    display.println(" g");
    
    if (uid.length() > 0) {
        display.setTextSize(1);
        display.setCursor(0, 45);
        display.print("UID:");
        display.println(uid);
    }
    
    // IP en dessous de l'UID
    display.setTextSize(1);
    display.setCursor(0, 56);
    if (wifiConnected) {
        display.print("IP: ");
        display.println(WiFi.localIP().toString().c_str());
    }
    
    display.display();
}

void showSpoolmanStatus(const String& line1, const String& line2, uint32_t durationMs) {
    spoolmanDisplayLine1 = line1;
    spoolmanDisplayLine2 = line2;
    spoolmanDisplayUntilMs = millis() + durationMs;
}

void resetAutoPushState(bool clearUid) {
    sendPhase = "";
    sendCountdown = -1;
    stableSinceMs = 0;
    stableCandidate = NAN;
    autoPushAttempts = 0;
    autoPushUid = "";
    if (clearUid) {
        lastUID = "";
    }
}

static size_t otaCommandMaxSize(uint8_t command) {
    if (command == U_FLASH) {
        return (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    }
    return UPDATE_SIZE_UNKNOWN;
}

static void handleOtaUpload(
    AsyncWebServerRequest *request,
    const String& filename,
    size_t index,
    uint8_t *data,
    size_t len,
    bool final,
    uint8_t command,
    const char* targetLabel
) {
    if (index == 0) {
        Serial.printf("[OTA] Starting %s upload: %s\n", targetLabel, filename.c_str());
        showSpoolmanStatus("OTA Upload", targetLabel, 4000);
        if (command != U_FLASH) {
            LittleFS.end();
        }
        if (!Update.begin(otaCommandMaxSize(command), command)) {
            Update.printError(Serial);
        }
    }

    if (!Update.hasError() && Update.write(data, len) != len) {
        Update.printError(Serial);
    }

    if (final) {
        if (Update.end(true)) {
            Serial.printf("[OTA] %s update complete (%u bytes)\n", targetLabel, (unsigned)(index + len));
            showSpoolmanStatus("OTA Complete", targetLabel, 5000);
            otaRestartPending = true;
            otaRestartAtMs = millis() + 5000;
        } else {
            Update.printError(Serial);
            showSpoolmanStatus("OTA Failed", targetLabel, 5000);
        }
    }
}

// ============================================================================
// PORTAIL CAPTIF & CONFIGURATION
// ============================================================================

void configModeCallback(WiFiManager *myWiFiManager) {
    displayMessage(
        "CONFIG MODE",
        "Connect to WiFi",
        gSetupSsid.length() ? gSetupSsid : "Setup-TigerScale"
    );
}

void saveConfigCallback() {
    displayMessage("Saving...", "WiFi config OK", "Reconnecting...");
    delay(800);
}

static bool connectStoredWifi(uint32_t timeoutMs, bool disconnectFirst, const String& statusLine2) {
    displayMessage("Connecting WiFi", statusLine2);
    if (disconnectFirst) {
        WiFi.disconnect(false, false);
        delay(250);
    }
    WiFi.begin();

    const uint32_t attemptStart = millis();
    while (millis() - attemptStart < timeoutMs) {
        if (WiFi.status() == WL_CONNECTED) {
            return true;
        }
        delay(250);
    }
    return WiFi.status() == WL_CONNECTED;
}

static bool hasStoredWifiCredentials() {
    wifi_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
        return false;
    }
    return cfg.sta.ssid[0] != '\0';
}

// 🔎 WiFiManager: Handles WiFi configuration and captive portal using WiFiManager.
//    This enables easy setup via a smartphone/laptop without hardcoding credentials.
//    After connection, sets up mDNS and checks cloud health.
void setupWiFi() {
    WiFiManagerParameter custom_api_key("apikey", "API Key (optionnel)", apiKey.c_str(), 64);
    
    wm.addParameter(&custom_api_key);
    wm.setAPCallback(configModeCallback);
    wm.setSaveConfigCallback(saveConfigCallback);
    wm.setConfigPortalTimeout(180);
    
    setBootStage("W1", "WiFi init");
    gSetupSsid = makeSetupSSID();
    gMdnsName = String("tigerscale-") + macSuffix4();
    WiFi.setHostname(gMdnsName.c_str());

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    const bool haveSavedWifi = hasStoredWifiCredentials();

    bool connected = connectStoredWifi(30000, false, "Try 1/2");
    if (!connected) {
        setBootStage("W2", "WiFi retry", "attempt 2/2");
        connected = connectStoredWifi(20000, true, "Try 2/2");
    }

    if (!connected) {
        if (haveSavedWifi) {
            Serial.println("[WiFi] Stored credentials found; staying in STA mode and retrying in background");
            setBootStage("W3", "WiFi deferred", "saved creds retry");
            wifiPortalDeferred = true;
            wifiPortalActive = false;
            wifiPortalFallbackAtMs = millis() + WIFI_PORTAL_FALLBACK_DELAY_MS;
        } else {
            setBootStage("W4", "WiFi AP mode", gSetupSsid);
            delay(1200);
            wifiPortalDeferred = false;
            wifiPortalActive = true;
            if (!wm.startConfigPortal(gSetupSsid.c_str())) {
                displayMessage("AP timeout", "Restarting...");
                delay(3000);
                ESP.restart();
            }
        }
    }
    
    apiKey = custom_api_key.getValue();
    if (apiKey.length() > 0) {
        prefs.begin("config", false);
        prefs.putString("apiKey", apiKey);
        prefs.end();
    }
    if (WiFi.isConnected()) {
        startMDNS();
    }
    wifiConnected = WiFi.isConnected();
    if (wifiConnected) {
        wifiPortalDeferred = false;
        wifiPortalActive = false;
        wm.stopWebPortal();
        wm.stopConfigPortal();
    }

    displayMessage(
        wifiConnected ? "WiFi Connected!" : "WiFi Offline",
        wifiConnected ? WiFi.SSID() : "Reconnect pending",
        wifiConnected ? WiFi.localIP().toString() : gSetupSsid,
        wifiConnected ? "Starting web..." : "Auto reconnect on"
    );
    Serial.printf("[BOOT W9] WiFi result | connected=%s ip=%s portalDeferred=%s\n",
        wifiConnected ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        wifiPortalDeferred ? "true" : "false");
    delay(600);
}

// ============================================================================
// LITTLEFS : Initialisation et debug
// ============================================================================

// Recursive directory listing
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  File root = fs.open(dirname);
  if(!root){
    Serial.printf("❌ [LITTLEFS] Failed to open dir: %s\n", dirname);
    return;
  }
  if(!root.isDirectory()){
    Serial.printf("❌ [LITTLEFS] Not a dir: %s\n", dirname);
    return;
  }
  Serial.printf("📁 [LITTLEFS] Listing: %s\n", dirname);
  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.printf("DIR  %s\n", file.name());
      if (levels){
        String sub = String(dirname);
        if (!sub.endsWith("/")) sub += "/";
        String childName = String(file.name());
        int slash = childName.lastIndexOf('/');
        if (slash >= 0) childName = childName.substring(slash + 1);
        sub += childName;
        listDir(fs, sub.c_str(), levels - 1);
      }
    } else {
      Serial.printf("FILE %s (%u)\n", file.name(), (unsigned)file.size());
    }
    file = root.openNextFile();
  }
}

// ============================================
// SERVIR FICHIERS STATIQUES DEPUIS LITTLEFS
// ============================================
// 🔎 LittleFS Initialization: Mounts the internal filesystem to serve web UI and assets.
//    The /www directory contains all static web content (HTML, CSS, JS, images).
//    This allows the ESP32 to serve a rich web interface directly from flash.
void setupFileSystem() {
    setBootStage("F1", "LittleFS mount");
    Serial.println("\n[LITTLEFS] Initialisation...");
    
    if (!LittleFS.begin(true)) {  // true = format si échec
        Serial.println("❌ [LITTLEFS] Échec montage!");
        displayMessage("ERROR", "Filesystem FAIL", "Check data/");
        delay(3000);
        return;
    }
    
    Serial.println("✅ [LITTLEFS] Monté avec succès");
    
    // Debug : vérifier /www existe
    File root = LittleFS.open("/www");
    if (!root) {
        Serial.println("⚠️  [LITTLEFS] Dossier /www introuvable!");
        Serial.println("    → Uploadez le filesystem: pio run --target uploadfs");
        return;
    }
    // Recursive listing including /www/img etc.
    listDir(LittleFS, "/www", 3);
    setBootStage("F9", "LittleFS ready");
}

// Validate API key against TigerTag CDN (firmware-side)
bool validateApiKeyFirmware(const String& key, String& displayNameOut) {
    displayNameOut = "";
    if (key.length() == 0) return false;
    HTTPClient http;
    http.setTimeout(3000);
    String url = String("https://cdn.tigertag.io/pingbyapikey?key=") + key;
    if (!http.begin(url)) {
        Serial.println("[APIKEY] http.begin failed");
        return false;
    }
    int code = http.GET();
    bool ok = false;
    if (code == 200) {
        String body = http.getString();
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, body);
        if (!err) {
            ok = doc["success"] | false;
            if (ok && doc["displayName"].is<const char*>()) {
                displayNameOut = String(doc["displayName"].as<const char*>());
            }
        } else {
            Serial.printf("[APIKEY] JSON parse error: %s\n", err.c_str());
        }
    } else {
        Serial.printf("[APIKEY] HTTP %d\n", code);
    }
    http.end();
    return ok;
}

// Delete stored API key and display name, reset runtime flags
bool deleteApiKey() {
    Serial.println("[APIKEY] deleteApiKey(): begin");
    bool removed = false;
    bool okBegin = prefs.begin("config", false);
    if (!okBegin) {
        Serial.println("[APIKEY] prefs.begin('config') FAILED");
    } else {
        bool r1 = prefs.remove("apiKey");
        bool r2 = prefs.remove("apiName");
        prefs.end();
        removed = (r1 || r2);
        Serial.printf("[APIKEY] prefs.remove apiKey=%s apiName=%s -> removed=%s\n", r1?"true":"false", r2?"true":"false", removed?"true":"false");
    }
    apiKey = "";
    apiDisplayName = "";
    apiValid = false;
    Serial.println("[APIKEY] deleteApiKey(): end");
    return removed;
}

// ============================================================================
// SERVEUR WEB & API
// ============================================================================

// ⚠️ SUPPRIMÉ : const char index_html[] PROGMEM = R"rawliteral(...
// Les fichiers HTML sont maintenant servis depuis LittleFS

AsyncWebSocket ws("/ws");

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected\n", client->id());
        // Send an immediate snapshot so the UI updates right away on connect
        int wIntSnap = (int)(currentWeight + (currentWeight >= 0 ? 0.5f : -0.5f));
        char snap[96];
        snprintf(snap, sizeof(snap), "{\"weight\":%d,\"uid\":\"%s\"}", wIntSnap, lastUID.c_str());
        client->text(snap);
        // Also push current API status so the UI reflects it immediately on fresh load
        {
            StaticJsonDocument<192> out;
            out["type"] = "apiStatus";
            out["valid"] = apiValid;
            if (apiValid && apiDisplayName.length()) out["displayName"] = apiDisplayName;
            String outStr; serializeJson(out, outStr);
            client->text(outStr);
        }
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (!info->final || info->opcode != WS_TEXT) return; // handle simple single-frame text only
        String msg = String((const char*)data).substring(0, len);

        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, msg);
        if (err) {
            Serial.printf("[WS] bad JSON: %s\n", err.c_str());
            return;
        }
        const char* mtype = doc["type"] | "";
        if (strcmp(mtype, "updateApiKey") == 0) {
            String newKey = String(doc["value"] | "");
            newKey.trim();
            if (newKey.length() == 0) {
                displayMessage("API key FAIL", "Check key");
                delay(600);
                displayWeight(currentWeight, lastUID);
                client->text("{\"type\":\"apiStatus\",\"valid\":false}");
                return;
            }
            String displayName;
            bool ok = validateApiKeyFirmware(newKey, displayName);
            if (ok) {
                // Persist only if valid
                apiKey = newKey;
                apiValid = true;
                apiDisplayName = displayName;
                prefs.begin("config", false);
                prefs.putString("apiKey", apiKey);
                prefs.putString("apiName", apiDisplayName);
                prefs.end();
                // Notify UI
                displayMessage("API key OK", apiDisplayName);
                delay(600);
                displayWeight(currentWeight, lastUID);
                StaticJsonDocument<192> out;
                out["type"] = "apiStatus";
                out["valid"] = true;
                out["displayName"] = apiDisplayName;
                String outStr; serializeJson(out, outStr);
                client->text(outStr);
                // Optional: also echo the stored key (if UI needs to sync)
                // client->text(String("{\"type\":\"apiKey\",\"value\":\"") + apiKey + "\"}");
            } else {
                displayMessage("API key FAIL", "Check key");
                delay(600);
                displayWeight(currentWeight, lastUID);
                client->text("{\"type\":\"apiStatus\",\"valid\":false}");
            }
        }
        else if (strcmp(mtype, "deleteApiKey") == 0) {
            bool ok = deleteApiKey();
            displayMessage(ok ? "API key deleted" : "Delete failed", ok ? "Credentials cleared" : "Check storage");
            delay(600);
            displayWeight(currentWeight, lastUID);
            // Inform only the requester about the result
            {
                StaticJsonDocument<96> out;
                out["type"] = "deleteApiKeyResult";
                out["success"] = ok;
                String outStr; serializeJson(out, outStr);
                client->text(outStr);
            }
            // Broadcast new API status to all clients
            {
                StaticJsonDocument<96> st;
                st["type"] = "apiStatus";
                st["valid"] = false;
                String s; serializeJson(st, s);
                ws.textAll(s);
            }
        }
    }
}

// ============================================
// SERVEUR WEB & API
// ============================================
void setupWebServer() {
    setBootStage("S1", "Web routes");
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    

    // ============================================
    // Page principale (préférer index.html.gz si présent)
    // ============================================
    // 🔎 Routing: Serve index.html(.gz) for root, with no-cache headers for fast dev iteration.
    //    Tries .gz first for compressed transfer. Caching is disabled for HTML to ensure the UI updates immediately after changes.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/www/index.html.gz")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/index.html.gz", "text/html; charset=utf-8");
            response->addHeader("Content-Encoding", "gzip");
            response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            response->addHeader("Pragma", "no-cache");
            request->send(response);
            return;
        }
        if (LittleFS.exists("/www/index.html")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/index.html", "text/html; charset=utf-8");
            response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            response->addHeader("Pragma", "no-cache");
            request->send(response);
            return;
        }
        request->send(404, "text/plain", "index.html(.gz) not found - uploadfs required");
    });
    
    // CSS (style.css, fallback to .gz), cache 24h
    // 🔎 Routing: Serve CSS, prefer uncompressed for debugging, fallback to .gz.
    //    Caching enabled (24h) as CSS changes infrequently.
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/www/style.css")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/style.css", "text/css");
            response->addHeader("Cache-Control", "max-age=86400");
            request->send(response);
            return;
        }
        if (LittleFS.exists("/www/style.css.gz")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/style.css.gz", "text/css");
            response->addHeader("Content-Encoding", "gzip");
            response->addHeader("Cache-Control", "max-age=86400");
            request->send(response);
            return;
        }
        request->send(404, "text/plain", "style.css(.gz) not found");
    });
    server.serveStatic("/styles.css", LittleFS, "/www/styles.css").setCacheControl("no-store");
    server.serveStatic("/favicon.ico", LittleFS, "/www/favicon.ico").setCacheControl("no-store");
    server.serveStatic("/favicon.png", LittleFS, "/www/favicon.png").setCacheControl("no-store");
    server.serveStatic("/manifest.json", LittleFS, "/www/manifest.json").setCacheControl("no-store");
    server.serveStatic("/sw.js", LittleFS, "/www/sw.js").setCacheControl("no-store");
    
    // JavaScript (app.js, fallback to .gz), no-store
    // 🔎 Routing: Serve JavaScript, prefer uncompressed for debugging, fallback to .gz.
    //    Caching disabled (no-store) to ensure new code is always loaded.
    server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/www/app.js")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/app.js", "application/javascript");
            response->addHeader("Cache-Control", "no-store");
            request->send(response);
            return;
        }
        if (LittleFS.exists("/www/app.js.gz")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/www/app.js.gz", "application/javascript");
            response->addHeader("Content-Encoding", "gzip");
            response->addHeader("Cache-Control", "no-store");
            request->send(response);
            return;
        }
        request->send(404, "text/plain", "app.js(.gz) not found");
    });
    server.serveStatic("/script.js", LittleFS, "/www/script.js").setCacheControl("no-store");
    
    // Static mapping for images (explicit, cache-safe during dev)
    // 🔎 Routing: Serve static image assets from /www/img.
    //    Cache disabled (no-store) for development; can be set to long-term cache in production.
    server.serveStatic("/img", LittleFS, "/www/img")
          .setCacheControl("no-store");
    
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            String body = String((char*)data).substring(0, len);
            int keyStart = body.indexOf("\"apiKey\":\"") + 10;
            int keyEnd = body.indexOf("\"", keyStart);
            apiKey = body.substring(keyStart, keyEnd);
            
            prefs.begin("config", false);
            prefs.putString("apiKey", apiKey);
            prefs.end();
             
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
    );

    server.on("/api/spoolman", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<2048> doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                request->send(400, "application/json", String("{\"success\":false,\"error\":\"") + err.c_str() + "\"}");
                return;
            }

            spoolmanConfig.enabled = doc["enabled"] | false;
            spoolmanConfig.url = normalizeSpoolmanBaseUrl(doc["url"] | "");
            spoolmanConfig.token = String((const char*)(doc["token"] | ""));
            spoolmanConfig.token.trim();
            spoolmanConfig.username = String((const char*)(doc["username"] | ""));
            spoolmanConfig.username.trim();
            spoolmanConfig.password = String((const char*)(doc["password"] | ""));
            spoolmanConfig.password.trim();
            saveIntegrationConfig();
            deviceLogf("[Spoolman] Config saved. enabled=%s url=%s token=%s\n",
                spoolmanConfig.enabled ? "true" : "false",
                spoolmanConfig.url.c_str(),
                spoolmanConfig.token.length() ? "<set>" : "<empty>");

            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    server.on("/api/filaman", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<2048> doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                request->send(400, "application/json", String("{\"success\":false,\"error\":\"") + err.c_str() + "\"}");
                return;
            }

            filamanConfig.enabled = doc["enabled"] | false;
            filamanConfig.url = normalizeSpoolmanBaseUrl(doc["url"] | "");
            filamanConfig.token = String((const char*)(doc["token"] | ""));
            filamanConfig.token.trim();
            filamanConfig.username = String((const char*)(doc["username"] | ""));
            filamanConfig.username.trim();
            filamanConfig.password = String((const char*)(doc["password"] | ""));
            filamanConfig.password.trim();
            invalidateFilamanSession();
            saveIntegrationConfig();
            deviceLogf("[Filaman] Config saved. enabled=%s url=%s token=%s username=%s password=%s\n",
                filamanConfig.enabled ? "true" : "false",
                filamanConfig.url.c_str(),
                filamanConfig.token.length() ? "<set>" : "<empty>",
                filamanConfig.username.length() ? "<set>" : "<empty>",
                filamanConfig.password.length() ? "<set>" : "<empty>");

            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    server.on("/api/bambuddy", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<1024> doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                request->send(400, "application/json", String("{\"success\":false,\"error\":\"") + err.c_str() + "\"}");
                return;
            }

            bambuddyConfig.enabled = doc["enabled"] | false;
            bambuddyConfig.url = normalizeSpoolmanBaseUrl(doc["url"] | "");
            bambuddyConfig.token = String((const char*)(doc["token"] | ""));
            bambuddyConfig.token.trim();
            saveIntegrationConfig();
            deviceLogf("[Bambuddy] Config saved. enabled=%s url=%s token=%s\n",
                bambuddyConfig.enabled ? "true" : "false",
                bambuddyConfig.url.c_str(),
                bambuddyConfig.token.length() ? "<set>" : "<empty>");

            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    server.on("/api/ota/firmware", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            const bool ok = !Update.hasError();
            request->send(ok ? 200 : 500, "application/json", ok ? "{\"success\":true,\"restarting\":true}" : "{\"success\":false}");
        },
        [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
            handleOtaUpload(request, filename, index, data, len, final, U_FLASH, "Firmware");
        }
    );

    server.on("/api/ota/filesystem", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            const bool ok = !Update.hasError();
            request->send(ok ? 200 : 500, "application/json", ok ? "{\"success\":true,\"restarting\":true}" : "{\"success\":false}");
        },
        [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
            handleOtaUpload(request, filename, index, data, len, final, U_SPIFFS, "Filesystem");
        }
    );
    
    server.on("/api/reset-wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", "{\"status\":\"resetting\"}");
        delay(1000);
        wm.resetSettings();
        ESP.restart();
    });
    
    server.on("/api/factory-reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", "{\"status\":\"factory reset\"}");
        delay(1000);
        prefs.begin("config", false);
        prefs.clear();
        prefs.end();
        wm.resetSettings();
        ESP.restart();
    });
    
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        {
            int wInt = (int)(currentWeight + (currentWeight >= 0 ? 0.5f : -0.5f));
            json += "\"weight\":" + String(wInt) + ",";
            // Insert rawWeight and smoothWeight after weight
            json += "\"rawWeight\":" + String(currentWeight, 2) + ",";
            json += "\"smoothWeight\":" + String((int)(currentWeight + (currentWeight >= 0 ? 0.5f : -0.5f))) + ",";
        }
        // Hold mode info
        json += "\"hold\":" + String(holdMode ? "true" : "false") + ",";
        json += "\"holdWeight\":" + String((int)(holdWeight + (holdWeight>=0?0.5f:-0.5f))) + ",";
        json += "\"uid\":\"" + lastUID + "\",";
        json += "\"uid_hex\":\"" + lastUIDHex + "\",";
        json += "\"wifi\":\"" + WiFi.SSID() + "\",";
        json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        json += "\"mdns\":\"" + gMdnsName + ".local\",";
        json += "\"cloud\":\"" + String(cloudOK ? "ok" : "down") + "\",";
        json += "\"apiKey\":\"" + apiKey + "\",";
        json += "\"apiValid\":" + String(apiValid ? "true" : "false") + ",";
        json += "\"displayName\":\"" + apiDisplayName + "\",";
        json += "\"calibrationFactor\":" + String(calibrationFactor, 4) + ",";
        json += "\"uptime_ms\":" + String(millis()) + ","; // milliseconds since boot
        json += "\"uptime_s\":" + String(millis() / 1000) + ",";
        // sendToCloud status: "3","2","1","send","success","error" or ""
        String stc;
        if (sendPhase == "countdown" && sendCountdown >= 0)       stc = String(sendCountdown);
        else if (sendPhase == "send")                              stc = "send";
        else if (sendPhase == "success")                           stc = "success";
        else if (sendPhase == "error")                             stc = "error";
        else                                                        stc = "";
        json += "\"sendToCloud\":\"" + stc + "\"";
        json += "}";
        request->send(200, "application/json", json);
    });

    server.on("/api/integrations", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"spoolmanEnabled\":" + String(spoolmanConfig.enabled ? "true" : "false") + ",";
        json += "\"spoolmanUrl\":\"" + jsonEscape(spoolmanConfig.url) + "\",";
        json += "\"spoolmanToken\":\"" + jsonEscape(spoolmanConfig.token) + "\",";
        json += "\"spoolmanUsername\":\"" + jsonEscape(spoolmanConfig.username) + "\",";
        json += "\"spoolmanPassword\":\"" + jsonEscape(spoolmanConfig.password) + "\",";
        json += "\"filamanEnabled\":" + String(filamanConfig.enabled ? "true" : "false") + ",";
        json += "\"filamanUrl\":\"" + jsonEscape(filamanConfig.url) + "\",";
        json += "\"filamanToken\":\"" + jsonEscape(filamanConfig.token) + "\",";
        json += "\"filamanUsername\":\"" + jsonEscape(filamanConfig.username) + "\",";
        json += "\"filamanPassword\":\"" + jsonEscape(filamanConfig.password) + "\",";
        json += "\"bambuddyEnabled\":" + String(bambuddyConfig.enabled ? "true" : "false") + ",";
        json += "\"bambuddyUrl\":\"" + jsonEscape(bambuddyConfig.url) + "\",";
        json += "\"bambuddyToken\":\"" + jsonEscape(bambuddyConfig.token) + "\"";
        json += "}";
        request->send(200, "application/json", json);
    });

    server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(32768);
        JsonArray lines = doc.createNestedArray("lines");
        for (size_t i = 0; i < deviceLogCount; ++i) {
            size_t idx = (deviceLogStart + i) % DEVICE_LOG_CAPACITY;
            lines.add(deviceLogLines[idx]);
        }
        doc["count"] = (int)deviceLogCount;
        String body;
        serializeJson(doc, body);
        request->send(200, "application/json", body);
    });

    server.on("/api/logs/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
        clearDeviceLogs();
        request->send(200, "application/json", "{\"success\":true}");
    });

    // REST: set/validate API key
    server.on("/api/apikey", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String body = String((const char*)data).substring(0, len);
            // extract { "key": "..." }
            int kp = body.indexOf("\"key\"");
            if (kp < 0) { request->send(400, "application/json", "{\"success\":false,\"error\":\"missing key\"}"); return; }
            int colon = body.indexOf(':', kp);
            if (colon < 0) { request->send(400, "application/json", "{\"success\":false,\"error\":\"bad json\"}"); return; }
            int q1 = body.indexOf('"', colon+1);
            int q2 = (q1 >= 0) ? body.indexOf('"', q1+1) : -1;
            if (q1 < 0 || q2 < 0) { request->send(400, "application/json", "{\"success\":false,\"error\":\"bad json\"}"); return; }
            String newKey = body.substring(q1+1, q2);
            newKey.trim();
            if (newKey.length() == 0) { request->send(400, "application/json", "{\"success\":false,\"error\":\"empty key\"}"); return; }

            String dn;
            bool ok = validateApiKeyFirmware(newKey, dn);
            if (ok) {
                apiKey = newKey;
                apiValid = true;
                if (dn.length()) apiDisplayName = dn;
                prefs.begin("config", false);
                prefs.putString("apiKey", apiKey);
                prefs.putString("apiName", apiDisplayName);
                prefs.end();
                request->send(200, "application/json", String("{\"success\":true,\"displayName\":\"") + apiDisplayName + "\"}");
            } else {
                apiValid = false;
                request->send(200, "application/json", "{\"success\":false}");
            }
        }
    );

    // REST: delete API key
    server.on("/api/apikey", HTTP_DELETE, [](AsyncWebServerRequest *request){
        bool ok = deleteApiKey();
        request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
    });
    // Alias with trailing slash for robustness
    server.on("/api/apikey/", HTTP_DELETE, [](AsyncWebServerRequest *request){
        bool ok = deleteApiKey();
        request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
    });
    // Compatibility: allow /api/apikey?method=delete with any HTTP verb, and also handle raw DELETE here
    server.on("/api/apikey", HTTP_ANY, [](AsyncWebServerRequest *request){
        // If true DELETE, handle directly (guards against handler-order issues)
        if (request->method() == HTTP_DELETE) {
            bool ok = deleteApiKey();
            request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
            return;
        }
        // RPC-style compatibility
        if (request->hasParam("method")) {
            String m = request->getParam("method")->value();
            m.toLowerCase();
            if (m == "delete") {
                bool ok = deleteApiKey();
                request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
                return;
            }
        }
        request->send(404, "text/plain", "Not Found");
    });

    // Compatibility handler for trailing slash path as well
    server.on("/api/apikey/", HTTP_ANY, [](AsyncWebServerRequest *request){
        if (request->method() == HTTP_DELETE) {
            bool ok = deleteApiKey();
            request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
            return;
        }
        if (request->hasParam("method")) {
            String m = request->getParam("method")->value();
            m.toLowerCase();
            if (m == "delete") {
                bool ok = deleteApiKey();
                request->send(200, "application/json", String("{\"success\":") + (ok?"true":"false") + "}");
                return;
            }
        }
        request->send(404, "text/plain", "Not Found");
    });

    // Simplified GET endpoint to delete API key (for tools that can't send DELETE)
    server.on("/api/apikey/delete", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.println("[APIKEY] GET /api/apikey/delete");
        bool ok = deleteApiKey();
        if (ok) {
            request->send(200, "application/json", "{\"success\":true}");
        } else {
            request->send(200, "application/json", "{\"success\":false}");
        }
    });

    // Diagnostic endpoint: replies before attempting deletion, to debug transport vs deletion issues
    server.on("/api/apikey/delete-test", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "application/json", "{\"ok\":true}\n");
        // Perform deletion after sending (for debugging potential response path issues)
        bool ok = deleteApiKey();
        Serial.printf("[APIKEY] delete-test post-send result=%s\n", ok?"true":"false");
    });

    // Ultra-simple endpoint as requested: http://<ip>/apikeydelete
    server.on("/apikeydelete", HTTP_GET, [](AsyncWebServerRequest *request){
        bool ok = deleteApiKey();
        request->send(200, "text/plain", ok ? "ok" : "fail");
    });

    // Simple ping endpoint to diagnose transport issues
    server.on("/api/ping", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "pong");
    });
    
    // REST: set weight (send to cloud) — expects { weight, uid? }
    server.on("/api/weight", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String body = String((const char*)data).substring(0, len);
            // extract weight number
            int wp = body.indexOf("weight");
            if (wp < 0) { request->send(400, "application/json", "{\"error\":\"missing weight\"}"); return; }
            int colon = body.indexOf(':', wp);
            if (colon < 0) { request->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
            String num = body.substring(colon+1);
            num.trim();
            while (num.length() && (num[num.length()-1] < '0' || num[num.length()-1] > '9') && num[num.length()-1] != '.') num.remove(num.length()-1);
            while (num.length() && ((num[0] < '0' || num[0] > '9') && num[0] != '-' && num[0] != '.')) num.remove(0,1);
            float w = num.toFloat();
            int wi = (int)(w + (w >= 0 ? 0.5f : -0.5f));
            if (w <= 0 && num.indexOf('0') != 0 && num.indexOf('.') != 0) { request->send(400, "application/json", "{\"error\":\"invalid weight\"}"); return; }

            // optional uid override
            String uidOverride = lastUID;
            int up = body.indexOf("\"uid\"");
            if (up >= 0) {
                int c2 = body.indexOf(':', up);
                int uq1 = (c2 >= 0) ? body.indexOf('"', c2+1) : -1;
                int uq2 = (uq1 >= 0) ? body.indexOf('"', uq1+1) : -1;
                if (uq1 >= 0 && uq2 > uq1) uidOverride = body.substring(uq1+1, uq2);
            }

            if (apiKey.length() == 0) { request->send(400, "application/json", "{\"error\":\"missing apiKey\"}"); return; }
            if (uidOverride.length() == 0) { request->send(400, "application/json", "{\"error\":\"missing uid (present a tag)\"}"); return; }

            HTTPClient http;
            const char* url = "https://us-central1-tigertag-connect.cloudfunctions.net/setSpoolWeightByRfid";
            if (!http.begin(url)) { request->send(500, "application/json", "{\"error\":\"http begin failed\"}"); return; }
            http.addHeader("Content-Type", "application/json");
            http.addHeader("x-api-key", apiKey);
            String payload = String("{\"uid\":\"") + uidOverride + "\",\"weight\":" + String(wi) + "}";
            int code = http.POST(payload);
            String resp = http.getString();
            http.end();

            if (code >= 200 && code < 300) {
                currentWeight = (float)wi;
                displayMessage("Synced OK", String(wi) + " g", "to cloud");
                delay(700);
                lastUID = "";
                lastPushedWeight = NAN;
                stableSinceMs = 0;
                stableCandidate = NAN;
                displayWeight(currentWeight, lastUID);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                String err = String("{\"error\":\"upstream ") + code + "\",\"body\":" + '"' + resp + '"' + "}";
                request->send(502, "application/json", err);
            }
        }
    );

    server.on("/api/push-weight", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String body = String((const char*)data).substring(0, len);
            int wp = body.indexOf("weight");
            if (wp < 0) { request->send(400, "application/json", "{\"error\":\"missing weight\"}"); return; }
            int colon = body.indexOf(':', wp);
            if (colon < 0) { request->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
            String num = body.substring(colon+1);
            num.trim();
            while (num.length() && (num[num.length()-1] < '0' || num[num.length()-1] > '9') && num[num.length()-1] != '.' ) num.remove(num.length()-1);
            while (num.length() && ( (num[0] < '0' || num[0] > '9') && num[0] != '-' && num[0] != '.' )) num.remove(0,1);
            float w = num.toFloat();
            int wi = (int)(w + (w >= 0 ? 0.5f : -0.5f));
            if (w <= 0 && num.indexOf('0') != 0 && num.indexOf('.') != 0) { request->send(400, "application/json", "{\"error\":\"invalid weight\"}"); return; }

            if (apiKey.length() == 0) { request->send(400, "application/json", "{\"error\":\"missing apiKey\"}"); return; }
            if (lastUID.length() == 0) { request->send(400, "application/json", "{\"error\":\"missing uid (present a tag)\"}"); return; }

            HTTPClient http;
            const char* url = "https://us-central1-tigertag-connect.cloudfunctions.net/setSpoolWeightByRfid";
            if (!http.begin(url)) { request->send(500, "application/json", "{\"error\":\"http begin failed\"}"); return; }
            http.addHeader("Content-Type", "application/json");
            http.addHeader("x-api-key", apiKey);
            String payload = String("{\"uid\":\"") + lastUID + "\",\"weight\":" + String(wi) + "}";
            int code = http.POST(payload);
            String resp = http.getString();
            http.end();

            if (code >= 200 && code < 300) {
                currentWeight = (float)wi;
                displayMessage("Synced OK", String(wi) + " g", "to cloud");
                delay(700);
                lastUID = "";
                lastPushedWeight = NAN;
                stableSinceMs = 0;
                stableCandidate = NAN;
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", wi, lastUID.c_str());
                ws.textAll(buf);
                displayWeight(currentWeight, lastUID);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                String err = String("{\"error\":\"upstream ") + code + "\",\"body\":" + '"' + resp + '"' + "}";
                request->send(502, "application/json", err);
            }
        }
    );

    server.on("/api/tare", HTTP_POST, [](AsyncWebServerRequest *request){
        scale.tare();
        currentWeight = 0.0f;
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"weight\":%.2f,\"uid\":\"%s\"}", currentWeight, lastUID.c_str());
        ws.textAll(buf);
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });

    server.on("/api/calibration", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String body = String((const char*)data).substring(0, len);
            int p = body.indexOf("factor");
            if (p < 0) p = body.indexOf("value");
            if (p < 0) { request->send(400, "application/json", "{\"error\":\"missing factor/value\"}"); return; }
            int colon = body.indexOf(':', p);
            if (colon < 0) { request->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
            String num = body.substring(colon+1); num.trim();
            while (num.length() && (num[num.length()-1] < '0' || num[num.length()-1] > '9') && num[num.length()-1] != '.' && num[num.length()-1] != '-') num.remove(num.length()-1);
            while (num.length() && ((num[0] < '0' || num[0] > '9') && num[0] != '-' && num[0] != '.')) num.remove(0,1);
            float f = num.toFloat();
            if (f == 0.0f) { request->send(400, "application/json", "{\"error\":\"invalid factor\"}"); return; }

            calibrationFactor = f;
            scale.set_scale(calibrationFactor);
            prefs.begin("config", false);
            prefs.putFloat("calFactor", calibrationFactor);
            prefs.end();
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
    );
    
    // Page 404
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.printf("[404] %s %s\n", request->method() == HTTP_GET ? "GET" : request->method() == HTTP_POST ? "POST" : request->method() == HTTP_DELETE ? "DELETE" : request->method() == HTTP_PUT ? "PUT" : "OTHER", request->url().c_str());
        request->send(404, "text/plain", "404 Not Found");
    });
    
    server.begin();
    Serial.println("✅ Serveur web démarré sur port 80");
    setBootStage("S9", "Web server ready");
}

// Helper: push weight to TigerTag Cloud Function
bool pushWeightToCloud(float w, int& codeOut, String& respOut) {
    codeOut = 0;
    respOut = "";
    if (!wifiConnected || !WiFi.isConnected()) return false;
    if (apiKey.length() == 0 || lastUID.length() == 0) return false;

    HTTPClient http;
    const char* url = "https://us-central1-tigertag-connect.cloudfunctions.net/setSpoolWeightByRfid";
    if (!http.begin(url)) return false;
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", apiKey);
    int wInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
    String payload = String("{\"uid\":\"") + lastUID + "\",\"weight\":" + String(wInt) + "}";
    codeOut = http.POST(payload);
    respOut = http.getString();
    http.end();
    if (codeOut >= 200 && codeOut < 300) {
        return true;
    }
    Serial.printf("[AutoPush] Upstream error %d: %s\n", codeOut, respOut.c_str());
    return false;
}

void handleAutoPush(float w) {
    const uint32_t now = millis();

    // Reset transient success/error after 1.5s
    if ((sendPhase == "success" || sendPhase == "error") && (now - sendPhaseLastChangeMs > 1500)) {
        sendPhase = "";
        sendCountdown = -1;
    }

    // Preconditions to consider any auto-send
    if (w < MIN_WEIGHT_TO_SEND_G) {
        resetAutoPushState(true);
        return;
    }
    if (apiKey.length() == 0 || lastUID.length() == 0 || !WiFi.isConnected()) {
        resetAutoPushState(false);
        return;
    }

    if (autoPushUid != lastUID) {
        autoPushUid = lastUID;
        autoPushAttempts = 0;
    }

    if (autoPushAttempts >= 2) {
        resetAutoPushState(true);
        return;
    }

    // Initialize stability tracking
    if (isnan(stableCandidate)) {
        stableCandidate = w;
        stableSinceMs = now;
        sendPhase = "countdown";
        // initial countdown (ceil to next second)
        int remMs = (int)STABLE_WINDOW_MS;
        sendCountdown = (remMs + 999) / 1000; // e.g., 1500ms -> 2
    }

    // If value deviates beyond epsilon, restart stability window
    if (fabs(w - stableCandidate) > STABLE_EPSILON_G) {
        stableCandidate = w;
        stableSinceMs = now;
        sendPhase = "countdown";
        int remMs = (int)STABLE_WINDOW_MS;
        sendCountdown = (remMs + 999) / 1000;
        return;
    }

    // Update countdown while within the stability window
    uint32_t elapsed = now - stableSinceMs;
    if (elapsed < STABLE_WINDOW_MS) {
        int remMs = (int)(STABLE_WINDOW_MS - elapsed);
        int newCount = (remMs + 999) / 1000;
        if (newCount != sendCountdown) sendCountdown = newCount; // 3..2..1 style
        return;
    }

    // Past stability window: consider cooldown/delta rules
    if (!isnan(lastPushedWeight)) {
        if (fabs(w - lastPushedWeight) < RESEND_DELTA_G) return;
        if (now - lastPushMs < RESEND_COOLDOWN_MS) return;
    }

    // Ready to send
    sendPhase = "send";
    sendCountdown = 0;

    displayMessage("Sending...", String("UID ") + lastUID, String(w, 1) + " g");
    int pushCode = 0;
    String pushResp;
    autoPushAttempts++;
    bool ok = pushWeightToCloud(w, pushCode, pushResp);
    if (ok) {
        int wInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
        lastPushedWeight = w;
        lastPushMs = now;
        displayMessage("Synced OK", String(wInt) + " g", "to cloud");
        delay(700);
        resetAutoPushState(true);
        lastPushedWeight = NAN;
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", wInt, lastUID.c_str());
        ws.textAll(buf);
        displayWeight((float)wInt, lastUID);
        sendPhase = "success";
        sendPhaseLastChangeMs = millis();
        sendCountdown = -1;
    } else {
        displayMessage("Sync failed", "Check WiFi/API", String(w, 1) + " g");
        delay(700);
        if (pushCode == 404 || autoPushAttempts >= 2) {
            resetAutoPushState(true);
        }
        displayWeight(w, lastUID);
        sendPhase = "error";
        sendPhaseLastChangeMs = millis();
        sendCountdown = -1;
    }
}

// ============================================================================
// mDNS LIFECYCLE HELPERS
// ============================================================================

void startMDNS() {
    MDNS.end();
    delay(50);
    if (WiFi.isConnected()) {
        if (MDNS.begin(gMdnsName.c_str())) {
            MDNS.addService("http", "tcp", 80);
            Serial.println("[mDNS] started: http://" + gMdnsName + ".local");
        } else {
            Serial.println("[mDNS] start failed");
        }
    }
}

void runBackgroundConnectivityChecks() {
    if (!WiFi.isConnected()) return;

    const uint32_t now = millis();
    if (now < POST_BOOT_CHECK_DELAY_MS) return;

    if (lastHealthCheckMs == 0 || now - lastHealthCheckMs >= BACKGROUND_CHECK_INTERVAL_MS) {
        lastHealthCheckMs = now;
        cloudOK = checkServerHealth();
    }

    if (apiKey.length() > 0 && (apiValid == false || lastApiValidationMs == 0 || now - lastApiValidationMs >= BACKGROUND_CHECK_INTERVAL_MS)) {
        lastApiValidationMs = now;
        String dn;
        const bool ok = validateApiKeyFirmware(apiKey, dn);
        apiValid = ok;
        if (ok && dn.length()) {
            apiDisplayName = dn;
            prefs.begin("config", false);
            prefs.putString("apiName", apiDisplayName);
            prefs.end();
        }
    }
}

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
#ifdef SYSTEM_EVENT_STA_GOT_IP
        case SYSTEM_EVENT_STA_GOT_IP:
#endif
            wifiConnected = true;
            wifiPortalDeferred = false;
            wifiPortalActive = false;
            lastHealthCheckMs = 0;
            lastApiValidationMs = 0;
            Serial.println("[WiFi] GOT_IP: " + WiFi.localIP().toString());
            if (bootStageCode != "09") {
                setBootStage("W8", "WiFi got IP", WiFi.localIP().toString());
            }
            startMDNS();
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
#ifdef SYSTEM_EVENT_STA_DISCONNECTED
        case SYSTEM_EVENT_STA_DISCONNECTED:
#endif
            wifiConnected = false;
            cloudOK = false;
            Serial.println("[WiFi] DISCONNECTED");
            if (bootStageCode != "09") {
                setBootStage("WD", "WiFi lost", WiFi.SSID());
            }
            MDNS.end();
            WiFi.reconnect();
            break;
        default:
            break;
    }
}

// ============================================================================
// GESTION BALANCE
// ============================================================================

void setupScale() {
    setBootStage("H1", "Scale init");
    scale.begin(HX711_DOUT, HX711_SCK);
    scale.set_scale(calibrationFactor);
    scale.tare();
    
    displayMessage("Scale OK", "Tare done");
    Serial.println("[BOOT H9] Scale ready");
    delay(1000);
}

float readWeight() {
    if (!scale.is_ready()) {
        return currentWeight; // keep last value if ADC not ready
    }

    // 1) Fast raw read (low latency)
    // The installed load cell orientation is inverted relative to the logical
    // "weight added to platform" direction, so normalize it here once.
    float raw = -scale.get_units(1);

    // 2) Update small median window
    gMedianBuf[gMedianIdx] = raw;
    gMedianIdx = (gMedianIdx + 1) % MEDIAN_WINDOW;
    if (gMedianCount < MEDIAN_WINDOW) gMedianCount++;

    // Compute median (tiny N → insertion sort)
    float tmp[MEDIAN_WINDOW];
    for (int i = 0; i < gMedianCount; ++i) tmp[i] = gMedianBuf[i];
    for (int i = 1; i < gMedianCount; ++i) {
        float key = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    float med = (gMedianCount > 0) ? tmp[gMedianCount/2] : raw;

    // 3) Exponential moving average for extra smoothing
    if (!gEmaInit) { gEmaWeight = med; gEmaInit = true; }
    else { gEmaWeight = gEmaWeight + EMA_ALPHA * (med - gEmaWeight); }

    currentWeight = gEmaWeight; // smoothed float (can be negative)
    return currentWeight;
}

// ============================================================================
// GESTION RFID
// ============================================================================

static String u64ToDec(uint64_t v) {
    if (v == 0) return String("0");
    char buf[21];
    buf[20] = '\0';
    int i = 20;
    while (v > 0 && i > 0) {
        uint64_t q = v / 10ULL;
        uint8_t r = (uint8_t)(v - q * 10ULL);
        buf[--i] = '0' + r;
        v = q;
    }
    return String(&buf[i]);
}

void setupRFID() {
    setBootStage("R1", "RFID init");
    SPI.begin();
    rfid.PCD_Init();
    displayMessage("RFID OK", "RC522 ready");
    Serial.println("[BOOT R9] RFID ready");
    delay(1000);
}

String readRFID() {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return "";
    }

    String hexStr; hexStr.reserve(rfid.uid.size * 2);
    uint64_t decVal = 0ULL;
    for (byte i = 0; i < rfid.uid.size; i++) {
        byte b = rfid.uid.uidByte[i];
        if (b < 0x10) hexStr += '0';
        hexStr += String(b, HEX);
        decVal = (decVal << 8) | b;
    }
    hexStr.toUpperCase();

    lastUIDHex = hexStr;
    String decStr = u64ToDec(decVal);

    TigerTagData tagData;
    String tagReadError;
    if (readTigerTagData(tagData, tagReadError)) {
        lastTigerTagData = tagData;
        deviceLogf("[TigerTag] Read payload: tigerTagId=%u productId=%u materialId=%u brandId=%u diameterId=%u weight=%u unit=%u\n",
            (unsigned)tagData.tigerTagId,
            (unsigned)tagData.productId,
            (unsigned)tagData.materialId,
            (unsigned)tagData.brandId,
            (unsigned)tagData.diameterId,
            (unsigned)tagData.weightValue,
            (unsigned)tagData.weightUnitId);
    } else {
        lastTigerTagData = TigerTagData();
        deviceLogf("[TigerTag] Payload read failed for UID %s: %s\n", decStr.c_str(), tagReadError.c_str());
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return decStr;
}

// ============================================================================
// CLOUD HEALTH CHECK
// ============================================================================

bool checkServerHealth() {
    HTTPClient http;
    http.setTimeout(1500);
    const char* url = "https://healthz-s3bqq5xmtq-uc.a.run.app/";
    if (!http.begin(url)) {
        Serial.println("[HEALTHZ] begin() failed");
        return false;
    }
    int code = http.GET();
    bool ok = false;
    if (code == 200) {
        String body = http.getString();
        ok = (body.indexOf("\"ok\":true") >= 0);
        Serial.printf("[HEALTHZ] 200 body=%s\n", body.c_str());
    } else {
        Serial.printf("[HEALTHZ] HTTP %d\n", code);
    }
    http.end();
    Serial.println(ok ? "✅ Server health OK" : "❌ Server health FAIL");
    return ok;
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("[BOOT 00] Reset -> setup()");
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(21, 22);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("Erreur OLED"));
        while (1);
    }
    
    setBootStage("01", "OLED ready", "v1.1.0");
    delay(2500);
    
    setBootStage("02", "Prefs load");
    prefs.begin("config", true);
    apiKey = prefs.getString("apiKey", "");
    calibrationFactor = prefs.getFloat("calFactor", calibrationFactor);
    apiDisplayName = prefs.getString("apiName", "");
    spoolmanConfig.enabled = prefs.getBool("spoolmanEnabled", false);
    spoolmanConfig.url = normalizeSpoolmanBaseUrl(prefs.getString("spoolmanUrl", ""));
    spoolmanConfig.token = prefs.getString("spoolmanToken", "");
    spoolmanConfig.username = prefs.getString("spoolmanUsername", "");
    spoolmanConfig.password = prefs.getString("spoolmanPassword", "");
    filamanConfig.enabled = prefs.getBool("filamanEnabled", false);
    filamanConfig.url = normalizeSpoolmanBaseUrl(prefs.getString("filamanUrl", ""));
    filamanConfig.token = prefs.getString("filamanToken", "");
    filamanConfig.username = prefs.getString("filamanUsername", "");
    filamanConfig.password = prefs.getString("filamanPassword", "");
    bambuddyConfig.enabled = prefs.getBool("bambuddyEnabled", false);
    bambuddyConfig.url = normalizeSpoolmanBaseUrl(prefs.getString("bambuddyUrl", ""));
    bambuddyConfig.token = prefs.getString("bambuddyToken", "");
    prefs.end();
    
    setBootStage("03", "WiFi event hook");
    WiFi.onEvent(onWiFiEvent);
    setupWiFi();
    if (WiFi.isConnected()) {
        startMDNS();
    }

    setBootStage("04", "Filesystem");
    setupFileSystem();
    setBootStage("05", "Web server");
    setupWebServer();

    setBootStage("06", "Scale");
    setupScale();
    setBootStage("07", "RFID");
    setupRFID();
    
    setBootStage("09", "Ready");
    displayMessage(
        "READY!",
        "IP: " + WiFi.localIP().toString(),
        gMdnsName + ".local",
        "Place an Spool.."
    );
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastBlink = 0;
    static unsigned long lastWifiRetry = 0;

    if (otaRestartPending && millis() >= otaRestartAtMs) {
        ESP.restart();
    }
    
    if (millis() - lastBlink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    }

    if (!WiFi.isConnected() && millis() - lastWifiRetry > 15000) {
        lastWifiRetry = millis();
        if (hasStoredWifiCredentials()) {
            Serial.println("[WiFi] Retry stored STA connection");
            WiFi.mode(WIFI_STA);
            WiFi.reconnect();
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.begin();
            }
        }
    }

    if (!WiFi.isConnected() && wifiPortalDeferred && !wifiPortalActive && millis() >= wifiPortalFallbackAtMs) {
        Serial.println("[WiFi] STA retry timeout reached, starting config portal");
        wifiPortalDeferred = false;
        wifiPortalActive = true;
        setBootStage("WA", "Config portal", gSetupSsid);
        delay(1200);
        if (!wm.startConfigPortal(gSetupSsid.c_str())) {
            displayMessage("AP timeout", "Restarting...");
            delay(3000);
            ESP.restart();
        }
        wifiConnected = WiFi.isConnected();
        wifiPortalActive = false;
        if (wifiConnected) {
            startMDNS();
            wm.stopWebPortal();
            wm.stopConfigPortal();
            displayMessage(
                "WiFi Connected!",
                WiFi.SSID(),
                WiFi.localIP().toString(),
                "Starting web..."
            );
            delay(600);
        }
    }
    
    String uid = readRFID();
    if (uid.length() > 0 && uid != lastUID) {
        lastUID = uid;
        spoolmanSyncPending = hasEnabledIntegration();
        if (spoolmanSyncPending) {
            showSpoolmanStatus("Reading", "Please wait", 6000);
        } else {
            String shortUid = uid;
            if (shortUid.length() > 8) shortUid = shortUid.substring(shortUid.length() - 8);
            showSpoolmanStatus("Tag Read", shortUid, 1800);
        }
        displayWeight(currentWeight, lastUID);
        deviceLogln("UID detected (DEC): " + lastUID + "  (HEX): " + lastUIDHex);
    }
    
    float weight = readWeight();
    currentWeight = weight;

    if (spoolmanSyncPending && lastUID.length() > 0) {
        showSpoolmanStatus("Reading", "Processing...", 6000);
        displayWeight(currentWeight, lastUID);
        bool overallOk = true;
        String failedName;
        String failedError;

        IntegrationConfig* integrations[] = {&spoolmanConfig, &filamanConfig, &bambuddyConfig};
        for (IntegrationConfig* integration : integrations) {
            if (!integration->enabled) continue;

            String integrationError;
            deviceLogf("[%s] Starting sync for UID %s at weight %.2f g\n", integration->name, lastUID.c_str(), weight);
            const bool ok = syncWeightToSpoolman(*integration, lastUID, weight, integrationError);
            if (ok) {
                deviceLogf("[%s] Sync success for UID %s\n", integration->name, lastUID.c_str());
            } else {
                overallOk = false;
                deviceLogf("[%s] Sync failed for UID %s: %s\n", integration->name, lastUID.c_str(), integrationError.c_str());
                if (failedName.length() == 0) {
                    failedName = integration->name;
                    failedError = integrationError;
                }
            }
        }

        if (overallOk) {
            showSpoolmanStatus("Integrations OK", "Weight synced");
        } else {
            String shortErr = failedError;
            if (shortErr.length() > 18) shortErr = shortErr.substring(0, 18);
            showSpoolmanStatus(failedName + " ERR", shortErr, 2500);
        }
        spoolmanSyncPending = false;
    }

    // --- Hold mode logic ---
    float displayedWeight = weight;
    if (!holdMode) {
        if (fabs(weight - holdWeight) < HOLD_THRESHOLD_ENTER) {
            if (holdStartMs == 0) holdStartMs = millis();
            if (millis() - holdStartMs > HOLD_TIME_MS) {
                holdMode = true;
                holdWeight = weight;
            }
        } else {
            holdStartMs = 0;
            holdWeight = weight;
        }
    } else {
        if (fabs(weight - holdWeight) > HOLD_THRESHOLD_EXIT) {
            holdMode = false;
            holdStartMs = 0;
            holdWeight = weight;
        }
    }
    displayedWeight = holdMode ? holdWeight : weight;

    if (millis() - lastUpdate > WS_UPDATE_INTERVAL_MS) {
        displayWeight(displayedWeight, lastUID);
        
        int wInt = (int)(displayedWeight + (displayedWeight >= 0 ? 0.5f : -0.5f));
        String json = "{\"weight\":" + String(wInt) + 
                      ",\"uid\":\"" + lastUID + "\"}";
        ws.textAll(json);
        ws.cleanupClients();
        
        lastUpdate = millis();
    }

    // Periodically rebroadcast API status so late joiners / stale UIs sync automatically
    if (millis() - lastApiBroadcastMs > 5000) { // every 5s
        if (ws.count() > 0) {
            StaticJsonDocument<192> out;
            out["type"] = "apiStatus";
            out["valid"] = apiValid;
            if (apiValid && apiDisplayName.length()) out["displayName"] = apiDisplayName;
            String outStr; serializeJson(out, outStr);
            ws.textAll(outStr);
        }
        lastApiBroadcastMs = millis();
    }

    runBackgroundConnectivityChecks();
    handleAutoPush(weight);
    
    delay(10);
}
