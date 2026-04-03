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
#include <math.h>

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
String spoolmanUrl = "";
String spoolmanToken = "";
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
};

// ============================================================================
// AFFICHAGE OLED
// ============================================================================

// 🔎 OLED Display: Utility to show multi-line status/info messages on the SSD1306 screen.
//    Used for user feedback, errors, and setup states.
void displayMessage(String line1, String line2 = "", String line3 = "", String line4 = "") {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

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
void displayWeight(float weight, const String& uid = "");

bool checkServerHealth();
bool pushWeightToCloud(float w);
void handleAutoPush(float w);
bool validateApiKeyFirmware(const String& key, String& displayNameOut);
bool deleteApiKey();
void showSpoolmanStatus(const String& line1, const String& line2 = "", uint32_t durationMs = 1800);
String normalizeSpoolmanBaseUrl(const String& raw);
String findSpoolmanSpoolIdByUid(const String& uid, String& errorOut);
bool fetchTigerTagProductInfo(const String& uid, uint32_t productId, TigerTagProductInfo& infoOut, String& errorOut);
bool fetchTigerTagResolvedMeta(const TigerTagData& tagData, TigerTagResolvedMeta& metaOut, String& errorOut);
String findSpoolmanVendorIdByName(const String& vendorName, String& errorOut);
String ensureSpoolmanVendor(const String& vendorName, String& errorOut);
String findSpoolmanFilamentIdByExternalId(const String& externalId, String& errorOut);
String ensureSpoolmanFilament(const String& uid, const TigerTagData& tagData, String& errorOut);
String createSpoolmanSpool(const String& uid, const TigerTagData& tagData, float currentMeasuredWeight, String& errorOut);
bool syncWeightToSpoolman(const String& uid, float w, String& errorOut);
void saveSpoolmanConfig();

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
    while (value.endsWith("/")) value.remove(value.length() - 1);
    return value;
}

void saveSpoolmanConfig() {
    prefs.begin("config", false);
    prefs.putString("spoolmanUrl", spoolmanUrl);
    prefs.putString("spoolmanToken", spoolmanToken);
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

    fetchTigerTagSimpleName(brandUrl, "name", metaOut.brandName, err);
    if (!err.isEmpty()) Serial.printf("[TigerTag] Brand lookup failed for %u: %s\n", (unsigned)tagData.brandId, err.c_str());
    err = "";
    fetchTigerTagSimpleName(materialUrl, "label", metaOut.materialName, err);
    if (!err.isEmpty()) Serial.printf("[TigerTag] Material lookup failed for %u: %s\n", (unsigned)tagData.materialId, err.c_str());
    return metaOut.brandName.length() > 0 || metaOut.materialName.length() > 0;
}

String findSpoolmanVendorIdByName(const String& vendorName, String& errorOut) {
    errorOut = "";
    if (!vendorName.length()) return "";
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    HTTPClient http;
    String url = baseUrl + "/api/v1/vendor?name=" + urlEncode(vendorName);
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    if (spoolmanToken.length() > 0) http.addHeader("Authorization", "Bearer " + spoolmanToken);
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

String ensureSpoolmanVendor(const String& vendorName, String& errorOut) {
    errorOut = "";
    if (!vendorName.length()) return "";
    String vendorId = findSpoolmanVendorIdByName(vendorName, errorOut);
    if (vendorId.length() > 0) return vendorId;
    if (errorOut.length() > 0) return "";

    DynamicJsonDocument payload(256);
    payload["name"] = vendorName;
    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    const String url = baseUrl + "/api/v1/vendor";
    Serial.printf("[Spoolman] POST %s payload=%s\n", url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (spoolmanToken.length() > 0) http.addHeader("Authorization", "Bearer " + spoolmanToken);
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

String findSpoolmanFilamentIdByExternalId(const String& externalId, String& errorOut) {
    errorOut = "";
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    if (baseUrl.length() == 0) {
        errorOut = "spoolman url missing";
        return "";
    }

    HTTPClient http;
    String url = baseUrl + "/api/v1/filament?external_id=" + externalId;
    url = baseUrl + "/api/v1/filament?external_id=" + urlEncode(externalId);
    Serial.printf("[Spoolman] Lookup filament via %s\n", url.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    if (spoolmanToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + spoolmanToken);
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

String ensureSpoolmanFilament(const String& uid, const TigerTagData& tagData, String& errorOut) {
    errorOut = "";
    TigerTagProductInfo productInfo;
    String tigerErr;
    bool hasProductInfo = fetchTigerTagProductInfo(uid, tagData.productId, productInfo, tigerErr);
    if (!hasProductInfo) {
        Serial.printf("[TigerTag] Product lookup failed for uid=%s product_id=%u: %s\n", uid.c_str(), (unsigned)tagData.productId, tigerErr.c_str());
    }

    bool hasValidProductId = !(tagData.productId == 0 || tagData.productId == 0xFFFFFFFFu);
    String externalId = hasValidProductId
        ? (String("tigertag:") + String(tagData.productId))
        : (String("tigertag-uid:") + uid);
    String filamentId = findSpoolmanFilamentIdByExternalId(externalId, errorOut);
    if (filamentId.length() > 0) {
        Serial.printf("[Spoolman] Using existing filament %s for %s\n", filamentId.c_str(), externalId.c_str());
        return filamentId;
    }
    if (errorOut.length() > 0) return "";

    String filamentName = hasProductInfo && productInfo.name.length()
        ? productInfo.name
        : (hasValidProductId ? (String("TigerTag Product ") + String(tagData.productId)) : (String("TigerTag UID ") + uid));
    String materialName = hasProductInfo && productInfo.material.length() ? productInfo.material : String("Material ") + String(tagData.materialId);
    String brandName = hasProductInfo && productInfo.brand.length() ? productInfo.brand : "";
    if (!hasProductInfo || !brandName.length() || materialName.startsWith("Material ")) {
        TigerTagResolvedMeta resolvedMeta;
        String metaErr;
        if (fetchTigerTagResolvedMeta(tagData, resolvedMeta, metaErr)) {
            if (!brandName.length() && resolvedMeta.brandName.length()) brandName = resolvedMeta.brandName;
            if (materialName.startsWith("Material ") && resolvedMeta.materialName.length()) materialName = resolvedMeta.materialName;
        }
    }
    if (!hasProductInfo) {
        if (brandName.length() && materialName.length()) filamentName = brandName + " " + materialName;
        else if (materialName.length()) filamentName = materialName;
    }
    String colorHex = hasProductInfo && productInfo.colorHex.length() ? productInfo.colorHex : colorToHex(tagData.colorR, tagData.colorG, tagData.colorB, tagData.colorA);
    float diameter = !isnan(productInfo.diameterMm) ? productInfo.diameterMm : (tagData.diameterId == 56 ? 1.75f : (tagData.diameterId == 221 ? 2.85f : NAN));
    float netWeight = !isnan(productInfo.netWeightG) ? productInfo.netWeightG : (tagData.weightValue > 0 ? (float)tagData.weightValue : NAN);
    float density = inferDensityFromMaterialName(materialName);
    int extruderTemp = (tagData.tempMin > 0 && tagData.tempMax >= tagData.tempMin) ? (int)((tagData.tempMin + tagData.tempMax) / 2) : -1;
    String vendorId = "";
    if (brandName.length()) {
        String vendorErr;
        vendorId = ensureSpoolmanVendor(brandName, vendorErr);
        if (vendorErr.length()) Serial.printf("[Spoolman] Vendor ensure failed for %s: %s\n", brandName.c_str(), vendorErr.c_str());
    }

    DynamicJsonDocument payload(1024);
    payload["name"] = filamentName;
    payload["material"] = materialName;
    payload["external_id"] = externalId;
    if (vendorId.length()) payload["vendor_id"] = vendorId.toInt();
    payload["color_hex"] = colorHex;
    payload["density"] = density;
    if (!isnan(diameter)) payload["diameter"] = diameter;
    if (!isnan(netWeight) && netWeight > 0) payload["weight"] = netWeight;
    if (extruderTemp > 0) payload["settings_extruder_temp"] = extruderTemp;

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    const String url = baseUrl + "/api/v1/filament";
    Serial.printf("[Spoolman] POST %s payload=%s\n", url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (spoolmanToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + spoolmanToken);
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
    Serial.printf("[Spoolman] Created filament %s for %s\n", createdFilamentId.c_str(), externalId.c_str());
    return createdFilamentId;
}

String createSpoolmanSpool(const String& uid, const TigerTagData& tagData, float currentMeasuredWeight, String& errorOut) {
    String filamentId = ensureSpoolmanFilament(uid, tagData, errorOut);
    if (filamentId.length() == 0) return "";

    DynamicJsonDocument payload(1024);
    payload["filament_id"] = filamentId.toInt();
    int currentWeight = (int)(currentMeasuredWeight + (currentMeasuredWeight >= 0 ? 0.5f : -0.5f));
    if (currentWeight < 0) currentWeight = 0;
    payload["remaining_weight"] = currentWeight;
    if (tagData.weightValue > 0) payload["initial_weight"] = tagData.weightValue;
    JsonObject extra = payload.createNestedObject("extra");
    extra["rfid_uid"] = uid;

    String payloadStr;
    serializeJson(payload, payloadStr);

    HTTPClient http;
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    const String url = baseUrl + "/api/v1/spool";
    Serial.printf("[Spoolman] POST %s payload=%s\n", url.c_str(), payloadStr.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }
    http.addHeader("Content-Type", "application/json");
    if (spoolmanToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + spoolmanToken);
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
    Serial.printf("[Spoolman] Created spool %s for UID %s\n", spoolId.c_str(), uid.c_str());
    return spoolId;
}

String findSpoolmanSpoolIdByUid(const String& uid, String& errorOut) {
    errorOut = "";
    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    if (baseUrl.length() == 0) {
        errorOut = "spoolman url missing";
        return "";
    }

    HTTPClient http;
    const String url = baseUrl + "/api/v1/spool";
    Serial.printf("[Spoolman] Lookup UID %s via %s\n", uid.c_str(), url.c_str());
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return "";
    }

    if (spoolmanToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + spoolmanToken);
    }

    const int code = http.GET();
    if (code < 200 || code >= 300) {
        const String resp = http.getString();
        http.end();
        errorOut = String("HTTP ") + code;
        if (resp.length() > 0) errorOut += ": " + resp;
        return "";
    }

    DynamicJsonDocument filter(256);
    filter[0]["id"] = true;
    filter[0]["extra_rfid_uid"] = true;

    DynamicJsonDocument doc(24576);
    DeserializationError err = deserializeJson(doc, *http.getStreamPtr(), DeserializationOption::Filter(filter));
    http.end();
    if (err) {
        errorOut = String("json parse failed: ") + err.c_str();
        return "";
    }

    JsonArray spools = doc.as<JsonArray>();
    Serial.printf("[Spoolman] Lookup response contains %u spool entries\n", (unsigned)spools.size());
    for (JsonObject spool : spools) {
        const char* extraUidRaw = spool["extra_rfid_uid"] | "";
        String extraUid = String(extraUidRaw);
        extraUid.trim();
        if (extraUid == uid) {
            String spoolId = String((int)spool["id"]);
            Serial.printf("[Spoolman] UID %s matched spool %s\n", uid.c_str(), spoolId.c_str());
            return spoolId;
        }
    }

    errorOut = "no spool with extra_rfid_uid";
    return "";
}

bool syncWeightToSpoolman(const String& uid, float w, String& errorOut) {
    errorOut = "";
    if (!wifiConnected || !WiFi.isConnected()) {
        errorOut = "wifi disconnected";
        return false;
    }

    const String baseUrl = normalizeSpoolmanBaseUrl(spoolmanUrl);
    if (baseUrl.length() == 0) {
        errorOut = "spoolman url missing";
        return false;
    }

    const String spoolId = findSpoolmanSpoolIdByUid(uid, errorOut);
    String resolvedSpoolId = spoolId;
    if (resolvedSpoolId.length() == 0) {
        if (errorOut == "no spool with extra_rfid_uid") {
            if (!lastTigerTagData.valid) {
                errorOut = "no spool with extra_rfid_uid and no TigerTag payload available";
                return false;
            }
            Serial.printf("[Spoolman] No spool found for UID %s, creating one from TigerTag data\n", uid.c_str());
            resolvedSpoolId = createSpoolmanSpool(uid, lastTigerTagData, w, errorOut);
            if (resolvedSpoolId.length() == 0) return false;
        } else {
            return false;
        }
    }

    HTTPClient http;
    const String url = baseUrl + "/api/v1/spool/" + resolvedSpoolId;
    if (!http.begin(url)) {
        errorOut = "http begin failed";
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    if (spoolmanToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + spoolmanToken);
    }

    const int weightInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
    const int clampedWeight = weightInt < 0 ? 0 : weightInt;
    String payload = String("{\"remaining_weight\":") + String(clampedWeight) + "}";
    Serial.printf("[Spoolman] PATCH %s payload=%s\n", url.c_str(), payload.c_str());
    const int code = http.PATCH(payload);
    const String resp = http.getString();
    http.end();

    if (code >= 200 && code < 300) return true;

    errorOut = String("HTTP ") + code;
    if (resp.length() > 0) errorOut += ": " + resp;
    return false;
}

// 🔎 OLED Display: Main function for rendering weight and tag info on the OLED.
//    Shows WiFi status, weight (large digits), UID, and device IP.
void displayWeight(float weight, const String& uid) {
    display.clearDisplay();
    
     // En-tête avec titre et statut WiFi
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Tiger-Scale");
    
    display.setTextSize(1);
    display.setCursor(100, 0);
    display.println(wifiConnected ? "WiFi" : "----");

    // Hold mode indicator (🅗 at x=112, y=0)
    if (holdMode) { display.setCursor(112, 0); display.print("🅗"); }
    
    // Poids au centre (grande taille) — entier uniquement
    int wInt = (int)(weight + (weight >= 0 ? 0.5f : -0.5f));
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(wInt);
    display.println(" g");
    
    // UID or transient Spoolman status
    if (spoolmanDisplayUntilMs > millis() && spoolmanDisplayLine1.length() > 0) {
        display.setTextSize(1);
        display.setCursor(0, 40);
        display.println(spoolmanDisplayLine1);
        if (spoolmanDisplayLine2.length() > 0) {
            display.setCursor(0, 49);
            display.println(spoolmanDisplayLine2);
        }
    } else if (uid.length() > 0) {
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
    displayMessage("Saving...", "Wi‑Fi config OK", "Reconnecting...");
    delay(800);
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
    
    displayMessage("Connecting to WiFi...", "Waiting...");
    gSetupSsid = makeSetupSSID();
    gMdnsName = String("tigerscale-") + macSuffix4();
    WiFi.setHostname(gMdnsName.c_str());
    
    if (!wm.autoConnect(gSetupSsid.c_str())) {
        displayMessage("WiFi ERROR", "Restarting...");
        delay(3000);
        ESP.restart();
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
    wifiConnected = true;
    wm.stopWebPortal();
    wm.stopConfigPortal();

    // Check TigerTag cloud health (lightweight)
    cloudOK = checkServerHealth();

    displayMessage(
        "WiFi Connected!",
        WiFi.SSID(),
        WiFi.localIP().toString(),
        cloudOK ? "Cloud: OK" : "Cloud: FAIL"
    );
    delay(2000);
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
        String sub = String(file.name());
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

            spoolmanUrl = normalizeSpoolmanBaseUrl(doc["url"] | "");
            spoolmanToken = String((const char*)(doc["token"] | ""));
            spoolmanToken.trim();
            saveSpoolmanConfig();
            Serial.printf("[Spoolman] Config saved. url=%s token=%s\n", spoolmanUrl.c_str(), spoolmanToken.length() ? "<set>" : "<empty>");

            request->send(200, "application/json", "{\"success\":true}");
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
        json += "\"spoolmanUrl\":\"" + jsonEscape(spoolmanUrl) + "\",";
        json += "\"spoolmanToken\":\"" + jsonEscape(spoolmanToken) + "\",";
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
                displayMessage("Synced \xE2\x9C\x93", String(wi) + " g", "to cloud");
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
                displayMessage("Synced \xE2\x9C\x93", String(wi) + " g", "to cloud");
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
}

// Helper: push weight to TigerTag Cloud Function
bool pushWeightToCloud(float w) {
    if (!wifiConnected || !WiFi.isConnected()) return false;
    if (apiKey.length() == 0 || lastUID.length() == 0) return false;

    HTTPClient http;
    const char* url = "https://us-central1-tigertag-connect.cloudfunctions.net/setSpoolWeightByRfid";
    if (!http.begin(url)) return false;
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", apiKey);
    int wInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
    String payload = String("{\"uid\":\"") + lastUID + "\",\"weight\":" + String(wInt) + "}";
    int code = http.POST(payload);
    String resp = http.getString();
    http.end();
    if (code >= 200 && code < 300) {
        return true;
    }
    Serial.printf("[AutoPush] Upstream error %d: %s\n", code, resp.c_str());
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
    if (w < MIN_WEIGHT_TO_SEND_G || apiKey.length() == 0 || lastUID.length() == 0 || !WiFi.isConnected()) {
        sendPhase = "";            // idle
        sendCountdown = -1;
        stableSinceMs = 0;
        stableCandidate = NAN;
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
    bool ok = pushWeightToCloud(w);
    if (ok) {
        int wInt = (int)(w + (w >= 0 ? 0.5f : -0.5f));
        lastPushedWeight = w;
        lastPushMs = now;
        displayMessage("Synced \xE2\x9C\x93", String(wInt) + " g", "to cloud");
        delay(700);
        lastUID = "";
        lastPushedWeight = NAN;
        stableSinceMs = 0;
        stableCandidate = NAN;
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", wInt, lastUID.c_str());
        ws.textAll(buf);
        displayWeight((float)wInt, lastUID);
        sendPhase = "success";
        sendPhaseLastChangeMs = millis();
        sendCountdown = -1;
    } else {
        displayMessage("Sync failed", "Check Wi‑Fi/API", String(w, 1) + " g");
        delay(700);
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

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
#ifdef SYSTEM_EVENT_STA_GOT_IP
        case SYSTEM_EVENT_STA_GOT_IP:
#endif
            wifiConnected = true;
            Serial.println("[WiFi] GOT_IP: " + WiFi.localIP().toString());
            startMDNS();
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
#ifdef SYSTEM_EVENT_STA_DISCONNECTED
        case SYSTEM_EVENT_STA_DISCONNECTED:
#endif
            wifiConnected = false;
            Serial.println("[WiFi] DISCONNECTED");
            MDNS.end();
            break;
        default:
            break;
    }
}

// ============================================================================
// GESTION BALANCE
// ============================================================================

void setupScale() {
    scale.begin(HX711_DOUT, HX711_SCK);
    scale.set_scale(calibrationFactor);
    scale.tare();
    
    displayMessage("Scale OK", "Tare done");
    delay(1000);
}

float readWeight() {
    if (!scale.is_ready()) {
        return currentWeight; // keep last value if ADC not ready
    }

    // 1) Fast raw read (low latency)
    float raw = scale.get_units(1);

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
    SPI.begin();
    rfid.PCD_Init();
    displayMessage("RFID OK", "RC522 ready");
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
        Serial.printf("[TigerTag] Read payload: tigerTagId=%u productId=%u materialId=%u brandId=%u diameterId=%u weight=%u unit=%u\n",
            (unsigned)tagData.tigerTagId,
            (unsigned)tagData.productId,
            (unsigned)tagData.materialId,
            (unsigned)tagData.brandId,
            (unsigned)tagData.diameterId,
            (unsigned)tagData.weightValue,
            (unsigned)tagData.weightUnitId);
    } else {
        lastTigerTagData = TigerTagData();
        Serial.printf("[TigerTag] Payload read failed for UID %s: %s\n", decStr.c_str(), tagReadError.c_str());
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
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(21, 22);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("Erreur OLED"));
        while (1);
    }
    
    displayMessage("TigerTagScale", "Starting...", "v1.1.0");
    delay(2000);
    
    prefs.begin("config", true);
    apiKey = prefs.getString("apiKey", "");
    calibrationFactor = prefs.getFloat("calFactor", calibrationFactor);
    apiDisplayName = prefs.getString("apiName", "");
    spoolmanUrl = normalizeSpoolmanBaseUrl(prefs.getString("spoolmanUrl", ""));
    spoolmanToken = prefs.getString("spoolmanToken", "");
    prefs.end();
    
    WiFi.onEvent(onWiFiEvent);
    setupWiFi();
    if (WiFi.isConnected()) {
        startMDNS();
    }

    // On boot: validate existing API key once
    if (apiKey.length() > 0 && WiFi.isConnected()) {
        String dn;
        apiValid = validateApiKeyFirmware(apiKey, dn);
        if (apiValid) {
            if (dn.length()) apiDisplayName = dn;
            prefs.begin("config", false);
            prefs.putString("apiName", apiDisplayName);
            prefs.end();
        }
    }
    
    setupFileSystem();  // ← AJOUTÉ : Monte LittleFS
    setupWebServer();
    setupScale();
    setupRFID();
    
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
    
    if (millis() - lastBlink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    }
    
    String uid = readRFID();
    if (uid.length() > 0 && uid != lastUID) {
        lastUID = uid;
        spoolmanSyncPending = true;
        Serial.println("UID detected (DEC): " + lastUID + "  (HEX): " + lastUIDHex);
    }
    
    float weight = readWeight();
    currentWeight = weight;

    if (spoolmanSyncPending && lastUID.length() > 0) {
        String spoolmanError;
        Serial.printf("[Spoolman] Starting sync for UID %s at weight %.2f g\n", lastUID.c_str(), weight);
        const bool spoolmanOk = syncWeightToSpoolman(lastUID, weight, spoolmanError);
        if (spoolmanOk) {
            Serial.printf("[Spoolman] Sync success for UID %s\n", lastUID.c_str());
            showSpoolmanStatus("Spoolman OK", "Weight synced");
        } else {
            Serial.printf("[Spoolman] Sync failed for UID %s: %s\n", lastUID.c_str(), spoolmanError.c_str());
            String shortErr = spoolmanError;
            if (shortErr.length() > 18) shortErr = shortErr.substring(0, 18);
            showSpoolmanStatus("Spoolman ERR", shortErr, 2500);
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

    handleAutoPush(weight);
    
    delay(10);
}
