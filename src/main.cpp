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

// ============================================================================
// FONCTION D'ARRONDI CENTRALISÉE
// ============================================================================

/**
 * 🔎 Arrondit un poids float à un entier
 * Utilise l'arrondi arithmétique standard
 * Positif: 50.2→50, 50.5→51, 50.9→51
 * Négatif: -1.2→-1, -1.5→-2, -1.9→-2
 */
static inline int roundWeight(float weight) {
    return (int)(weight + (weight >= 0 ? 0.5f : -0.5f));
}

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
float calibrationFactor = 406;
float currentWeight = 0.0;
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

// ============================================================================
// 🔄 FONCTIONS DE FILTRE DE POIDS - VERSION COMPLÈTE OPTIMISÉE
// ============================================================================
// 🔎 Configuration de stabilité (à ajuster selon capteur/feedback)
const float EMA_ALPHA_FINE      = 0.05f;   // Très lent (5% adaptatif)
const float EMA_ALPHA_FAST      = 0.12f;   // Adaptif lors de changement rapide
const int   MEDIAN_WINDOW_LARGE = 15;      // 15 lectures (plus robustesse)
const float HYSTERESIS_THRESHOLD = 0.5f;   // Hystérésis: 0.5g (évite scintillement)
const float DEAD_ZONE_G         = 1.0f;   // Zone morte: ignore bruits < 1.0g
const uint32_t STABLE_DISPLAY_MS = 1500;    // N'affiche que si stable 800ms
const float MIN_WEIGHT_CHANGE_TO_RESET_G = 50.0f;  // Seuil de retrait de bobine (grammes)
static float gLastCloudWeight = NAN;               // Poids net du cloud sauvegardé
static float gLastSentWeight = NAN;                // Poids BRUT envoyé au cloud (avec spool)
static uint32_t gCloudWeightSetMs = 0;             // Timestamp quand le poids cloud a été reçu

// 🔎 État interne du filtre
static float gEmaWeight         = 0.0f;
static bool  gEmaInit           = false;
static float gMedianBuf[MEDIAN_WINDOW_LARGE] = {0};
static int   gMedianIdx         = 0;
static int   gMedianCount       = 0;
static float gLastDisplayedWeight = 0.0f;
static uint32_t gStableStartMs  = 0;
static bool  gIsStable          = false;

// --- Cloud sync result cache (server-computed net weight_available) ---
static bool  gLastNetValid   = false;
static float gLastNetWeight  = NAN;   // server weight_available
static float gLastRawWeight  = NAN;   // server weight (raw, includes container)
static float gLastContainer  = 0.0f;  // server container_weight

// --- UI/Status for auto-send countdown & phase ---
volatile int sendCountdown = -1;         // -1 = no countdown, >=0 = seconds remaining
String sendPhase = "";                  // "" | "countdown" | "send" | "success" | "error"
uint32_t sendPhaseLastChangeMs = 0;      // for expiring transient phases (success/error)

enum OledState {
    OLED_STATE_IDLE,           // Au repos
    OLED_STATE_WEIGHING,       // Pesage en cours
    OLED_STATE_UID_DETECTED,   // UID détecté
    OLED_STATE_SENDING,        // Envoi au cloud
    OLED_STATE_SUCCESS,        // Succès
    OLED_STATE_ERROR           // Erreur
};

OledState currentOledState = OLED_STATE_IDLE;
uint32_t oledStateChangeMs = 0;
const uint32_t OLED_MESSAGE_DURATION_MS = 2000;  // 2 secondes pour lire
const uint32_t OLED_ERROR_DURATION_MS = 3000;    // 3 secondes pour erreur

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

// New state-aware renderer
void displayWeightWithState(float weight, const String& uid, OledState state) {
    display.clearDisplay();
    
    // En-tête avec titre et statut WiFi
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Tiger-Scale");
    display.setCursor(100, 0);
    display.println(wifiConnected ? "WiFi" : "----");
    
    // Poids au centre (grande taille)
    // Affiche le poids NET (cloud) en grand lorsque l'état est IDLE avec un net présent
    float bigVal = weight;
    if (state == OLED_STATE_IDLE && !isnan(gLastCloudWeight)) {
        bigVal = gLastCloudWeight;  // montrer le net à la place du poids balance
    }
    int wInt = roundWeight(bigVal);
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(wInt);
    display.println("g");
    
    // État en bas selon l'état actuel
    display.setTextSize(1);
    display.setCursor(0, 50);
    
    switch (state) {
        case OLED_STATE_IDLE:
            if (!isnan(gLastCloudWeight)) {
                // Poids net déjà affiché en grand ; on évite d'afficher le poids balance
                display.setCursor(0, 41);  // move slightly up to add spacing
                display.println("Remaining");
                display.setCursor(0, 56);
                display.println("Remove Filament");
            } else {
                display.println("Ready to weigh");
            }
            break;
            
        case OLED_STATE_WEIGHING:
            display.println("Weighing...");
            break;
            
        case OLED_STATE_UID_DETECTED:
            display.print("UID: ");
            display.println(uid.substring(0, 16));
            // display.setCursor(0, 56);
            // display.println("Ready to push");
            break;
            
        case OLED_STATE_SENDING:
            // display.println("Sending...");
            display.setCursor(0, 56);
            display.print("UID: ");
            display.println(uid.substring(0, 16));
            break;
            
        case OLED_STATE_SUCCESS:
            display.print("Net: ");
            display.println(String(roundWeight(gLastNetWeight)) + " g");
            display.setCursor(0, 56);
            display.println("✓ Synced!");
            break;
            
        case OLED_STATE_ERROR:
            display.println("✗ Error!");
            display.setCursor(0, 56);
            display.println("Check WiFi/API");
            break;
    }
    
    display.display();
}


bool checkServerHealth();
bool pushWeightToCloud(float w);
void handleAutoPush(float w);
bool validateApiKeyFirmware(const String& key, String& displayNameOut);
bool deleteApiKey();

// Parse Cloud Function JSON response to extract weight_available (net), weight (raw) and container_weight
static bool parseCloudNetWeights(const String& resp, float& netOut, float& rawOut, float& containerOut) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, resp);
    if (err) {
        Serial.printf("[CloudParse] JSON error: %s\n", err.c_str());
        return false;
    }
    bool success = doc["success"] | false;
    if (!success) {
        Serial.println("[CloudParse] success=false");
        return false;
    }
    // Extract values
    if (doc.containsKey("weight_available")) {
        netOut = doc["weight_available"].as<float>();
        rawOut = doc["weight"]            | NAN;
        containerOut = doc["container_weight"] | 0.0f;
        return true;
    }
    Serial.println("[CloudParse] missing weight_available");
    return false;
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
                displayWeightWithState(currentWeight, lastUID, currentOledState);
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
                displayWeightWithState(currentWeight, lastUID, currentOledState);
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
                displayWeightWithState(currentWeight, lastUID, currentOledState);
                client->text("{\"type\":\"apiStatus\",\"valid\":false}");
            }
        }
        else if (strcmp(mtype, "deleteApiKey") == 0) {
            bool ok = deleteApiKey();
            displayMessage(ok ? "API key deleted" : "Delete failed", ok ? "Credentials cleared" : "Check storage");
            delay(600);
            displayWeightWithState(currentWeight, lastUID, currentOledState);
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

void resetWeightFilters() {
    gEmaWeight = 0.0f;
    gEmaInit = false;
    gMedianIdx = 0;
    gMedianCount = 0;
    gLastDisplayedWeight = 0.0f;
    gStableStartMs = 0;
    gIsStable = false;
    memset(gMedianBuf, 0, sizeof(gMedianBuf));
    Serial.println("[FILTER] ✅ Weight filters reset - fresh start!");
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
    server.serveStatic("/img", LittleFS, "/www/img").setCacheControl("no-store");
    
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
            int wInt = roundWeight(currentWeight);
            json += "\"weight\":" + String(wInt) + ",";
            // Insert rawWeight and smoothWeight after weight
            json += "\"rawWeight\":" + String(currentWeight, 2) + ",";
        }
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
            int wi = roundWeight(w);
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
                float net = NAN, raw = NAN, cont = 0.0f;
                bool okParse = parseCloudNetWeights(resp, net, raw, cont);
                int shown = wi;
                if (okParse) {
                    // Prefer server-computed net (weight_available)
                    currentWeight = net;
                    shown = roundWeight(net);
                    // cache last server values
                    gLastNetValid = true;
                    gLastNetWeight = net;
                    gLastRawWeight = raw;
                    gLastContainer = cont;
                } else {
                    currentWeight = (float)wi; // fallback
                    gLastNetValid = false;
                }
                // Ne pas afficher "Weight Available" - juste mettre à jour l'état
                currentOledState = OLED_STATE_IDLE;
                oledStateChangeMs = millis();
                
                lastUID = "";
                lastPushedWeight = NAN;
                stableSinceMs = 0;
                stableCandidate = NAN;
                displayWeightWithState(currentWeight, lastUID, currentOledState);
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
            int wi = roundWeight(w);
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
                // Prefer server-computed net (weight_available) for UI + WS
                float net = NAN, raw = NAN, cont = 0.0f;
                bool okParse = parseCloudNetWeights(resp, net, raw, cont);
                int shown = wi;
                if (okParse) {
                    currentWeight = net;
                    shown = roundWeight(net);
                    // cache last server values
                    gLastNetValid = true;
                    gLastNetWeight = net;
                    gLastRawWeight = raw;
                    gLastContainer = cont;
                } else {
                    currentWeight = (float)wi;
                    gLastNetValid = false;
                }
                // Ne pas afficher "Weight Available" - juste mettre à jour l'état
                currentOledState = OLED_STATE_IDLE;
                oledStateChangeMs = millis();
                
                lastUID = "";
                lastPushedWeight = NAN;
                stableSinceMs = 0;
                stableCandidate = NAN;
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", shown, lastUID.c_str());
                ws.textAll(buf);
                displayWeightWithState(currentWeight, lastUID, currentOledState);
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
        resetWeightFilters();
        lastUID = "";
        
        // ← AJOUTER: Sauvegarder la tare
        float currentOffset = scale.get_offset();
        prefs.begin("config", false);
        prefs.putFloat("tareFactor", currentOffset);
        prefs.end();
        Serial.printf("[TARE] Tare sauvegardée: %f\n", currentOffset);
        
        // Forcer l'affichage immédiatement
        displayWeightWithState(currentWeight, lastUID, currentOledState);
        
        char buf[64];
        int wInt = roundWeight(currentWeight); 
        snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", wInt, lastUID.c_str());
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
            resetWeightFilters();

            displayWeightWithState(currentWeight, lastUID, currentOledState);
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
    int wInt = roundWeight(w);
    String payload = String("{\"uid\":\"") + lastUID + "\",\"weight\":" + String(wInt) + "}";
    int code = http.POST(payload);
    String resp = http.getString();
    http.end();

    if (code >= 200 && code < 300) {
        // Parse server-computed net weight (weight_available) and cache for later display
        float net = NAN, raw = NAN, cont = 0.0f;
        gLastNetValid = parseCloudNetWeights(resp, net, raw, cont);
        if (gLastNetValid) {
            gLastNetWeight = net;
            gLastRawWeight = raw;
            gLastContainer = cont;
            Serial.printf("[AutoPush] server net=%0.2f raw=%0.2f container=%0.2f\n", net, raw, cont);
        } else {
            Serial.println("[AutoPush] response OK but missing weight_available; fallback to sent weight");
        }
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

    displayMessage("Sending...", String("UID ") + lastUID, String(roundWeight(w)) + " g");
    bool ok = pushWeightToCloud(w);
    if (ok) {
        // Poids net reçu du cloud
        float toDisplay = (gLastNetValid && !isnan(gLastNetWeight)) ? gLastNetWeight : w;
        int wInt = roundWeight(toDisplay);

        lastPushedWeight = toDisplay;
        lastPushMs = now;

        // SAUVEGARDER le poids ENVOYÉ et le poids net du cloud
        gLastSentWeight = w;                    // ← Poids BRUT envoyé (avec spool)
        gLastCloudWeight = toDisplay;           // ← Poids NET reçu (sans spool)
        gCloudWeightSetMs = now;
        Serial.printf("[CLOUD] Sent: %.2f g, Net: %.2f g\n", w, toDisplay);

        // Ne pas afficher "Weight Available" - laisser l'état OLED gérer
        // Mettre à jour l'état OLED directement
        currentOledState = OLED_STATE_IDLE;
        oledStateChangeMs = millis();

        lastUID = "";
        lastPushedWeight = NAN;
        stableSinceMs = 0;
        stableCandidate = NAN;

        char buf[64];
        snprintf(buf, sizeof(buf), "{\"weight\":%d,\"uid\":\"%s\"}", wInt, lastUID.c_str());
        ws.textAll(buf);

        // Afficher le poids net reçu du cloud
        displayWeightWithState(toDisplay, lastUID, OLED_STATE_IDLE);
        currentWeight = toDisplay;

        sendPhase = "success";
        sendPhaseLastChangeMs = millis();
        sendCountdown = -1;
        // reset cache after use to avoid stale values
        gLastNetValid = false;
    } else {
        displayMessage("Sync failed", "Check Wi‑Fi/API", String(roundWeight(w)) + " g");
        delay(2000);

        currentOledState = OLED_STATE_ERROR;
        oledStateChangeMs = millis();

        displayWeightWithState(w, lastUID, OLED_STATE_ERROR);
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
    
    // ← MODIFIER: Charger la tare sauvegardée au lieu de faire tare()
    prefs.begin("config", true);
    float savedTare = prefs.getFloat("tareFactor", 0.0f);
    prefs.end();
    
    if (savedTare != 0.0f) {
        // Restaurer la tare sauvegardée
        scale.set_offset(savedTare);
        Serial.printf("[SCALE] Tare restaurée: %f\n", savedTare);
        displayMessage("Scale OK", "Tare restored");
    } else {
        // Première utilisation: faire un tare
        scale.tare();
        Serial.println("[SCALE] Tare effectuée (première utilisation)");
        displayMessage("Scale OK", "Tare done");
    }
    
    delay(1000);
}

/**
 * 🔎 Détecte si le capteur est en train de changer rapidement
 * Retourne true si changement > 2g en 100ms
 */
static bool isRapidChange(float raw) {
    static float gLastRaw = 0.0f;
    static uint32_t gLastRawTime = 0;
    
    uint32_t now = millis();
    uint32_t dt = now - gLastRawTime;
    
    if (dt < 50) return false; // Trop proche, ignorer
    
    float delta = fabs(raw - gLastRaw);
    gLastRaw = raw;
    gLastRawTime = now;
    
    // Si delta > 2g en moins de 100ms → changement rapide (utilisateur action)
    if (dt < 100 && delta > 2.0f) {
        return true;
    }
    return false;
}

/**
 * 🔎 Applique la dead zone (zone morte)
 * Élimine les bruits de très faible amplitude
 */
static float applyDeadZone(float value) {
    float absVal = fabs(value);
    if (absVal < DEAD_ZONE_G) {
        return 0.0f;
    }
    // Rendre la transition douce au lieu d'un "saut"
    return (value >= 0) 
        ? (value - DEAD_ZONE_G) 
        : (value + DEAD_ZONE_G);
}

/**
 * 🔎 Applique l'hystérésis pour éviter le scintillement
 * Les petits changements (< seuil) ne mettent pas à jour l'affichage
 */
static float applyHysteresis(float newValue, float lastValue) {
    float delta = fabs(newValue - lastValue);
    
    // Si la différence est trop petite, garder l'ancienne valeur
    if (delta < HYSTERESIS_THRESHOLD) {
        return lastValue;
    }
    return newValue;
}

/**
 * 🔎 Calcule la médiane de manière efficace (insertion sort)
 * Robuste contre les pics de bruit
 */
static float computeMedian() {
    if (gMedianCount == 0) return gEmaWeight;
    
    // Tri insertion (O(n²) mais n petit = 11)
    float tmp[MEDIAN_WINDOW_LARGE];
    memcpy(tmp, gMedianBuf, gMedianCount * sizeof(float));
    
    for (int i = 1; i < gMedianCount; ++i) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j+1] = tmp[j];
            j--;
        }
        tmp[j+1] = key;
    }
    
    // Retourner la médiane (ou moyenne des 2 du milieu si pair)
    if (gMedianCount % 2 == 1) {
        return tmp[gMedianCount / 2];
    } else {
        return (tmp[gMedianCount/2 - 1] + tmp[gMedianCount/2]) / 2.0f;
    }
}

float readWeight() {
    if (!scale.is_ready()) {
        return currentWeight; // Garder dernière valeur si ADC pas prêt
    }

    // ========== ÉTAPE 1: Lecture brute ==========
    float raw = scale.get_units(1);  // 1 lecture rapide du capteur
    
    // ========== ÉTAPE 2: Fenêtre médiane (robustesse) ==========
    // Stocke les 11 dernières lectures
    gMedianBuf[gMedianIdx] = raw;
    gMedianIdx = (gMedianIdx + 1) % MEDIAN_WINDOW_LARGE;
    if (gMedianCount < MEDIAN_WINDOW_LARGE) gMedianCount++;
    
    // Calcule la médiane (valeur du milieu après tri)
    float medianVal = computeMedian();
    
    // ========== ÉTAPE 3: Détection changement rapide ==========
    // Aide à différencier "utilisateur pose objet" vs "bruit capteur"
    bool rapidChange = isRapidChange(raw);
    float alphaToUse = rapidChange ? EMA_ALPHA_FAST : EMA_ALPHA_FINE;
    
    // ========== ÉTAPE 4: Lissage exponentiel adaptatif ==========
    if (!gEmaInit) {
        gEmaWeight = medianVal;
        gEmaInit = true;
    } else {
        // EMA: newValue = oldValue + alpha * (measured - oldValue)
        // Converge progressivement vers la vraie valeur
        gEmaWeight = gEmaWeight + alphaToUse * (medianVal - gEmaWeight);
    }
    
    // ========== ÉTAPE 5: Hysteresis (anti-scintillement) ==========
    // Les changements < 0.3g ne mettent pas à jour l'affichage
    float withHysteresis = applyHysteresis(gEmaWeight, gLastDisplayedWeight);
    
    // ========== ÉTAPE 6: Dead zone (ignore bruits mineurs) ==========
    // Tous les poids entre -0.15g et +0.15g → 0g
    float withDeadZone = applyDeadZone(withHysteresis);
    
    // ========== ÉTAPE 7: Tracking stabilité ==========
    // Détecte si le poids s'est stabilisé depuis 800ms
    float delta = fabs(withDeadZone - gLastDisplayedWeight);
    if (delta < 0.2f) {
        // Valeur quasi identique → on progresse vers stabilité
        if (gStableStartMs == 0) {
            gStableStartMs = millis();
            gIsStable = false;
        } else if (millis() - gStableStartMs > STABLE_DISPLAY_MS) {
            gIsStable = true;  // Marqué comme stable après 800ms sans changement
        }
    } else {
        // Changement détecté → reset stabilité
        gStableStartMs = millis();
        gIsStable = false;
    }
    
    // Mémoriser pour la prochaine itération (hysteresis + stabilité)
    gLastDisplayedWeight = withDeadZone;
    currentWeight = withDeadZone;
    
    // ========== DEBUG (optionnel) ==========
    //static uint32_t lastDebug = 0;
    //if (millis() - lastDebug > 500) {
    //    Serial.printf("[WEIGHT] raw=%.2f median=%.2f ema=%.2f final=%.2f stable=%s alpha=%s\n",
    //        raw, medianVal, gEmaWeight, currentWeight, gIsStable?"YES":"NO", 
    //        rapidChange?"FAST":"FINE");
    //    lastDebug = millis();
    // }
    
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

    rfid.PICC_HaltA();
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
        currentOledState = OLED_STATE_UID_DETECTED;
        oledStateChangeMs = millis();
        Serial.println("UID detected (DEC): " + lastUID + "  (HEX): " + lastUIDHex);
    }
    
    float weight = readWeight();

    // ===== DÉTECTION RETRAIT DE BOBINE =====
    // Si on a un poids envoyé au cloud et une différence > 50g → retrait détecté
    if (!isnan(gLastSentWeight)) {
        float delta = fabs(weight - gLastSentWeight);  // ← Comparer avec le poids ENVOYÉ, pas le net!
        
        if (delta > MIN_WEIGHT_CHANGE_TO_RESET_G) {
            Serial.printf("[RETRAIT] Détecté! Envoyé: %.2f, Actuel: %.2f, Delta: %.2f\n", 
                          gLastSentWeight, weight, delta);
            
            // Réinitialiser pour nouveau pesage
            gLastSentWeight = NAN;         // ← reset du poids BRUT envoyé
            gLastCloudWeight = NAN;
            gCloudWeightSetMs = 0;
            lastUID = "";
            currentOledState = OLED_STATE_IDLE;
            
            // Pas d'écran spécial pour le retrait - retour direct à IDLE
            // displayMessage() supprimé pour éviter les transitions confuses
        }
    }

    float displayedWeight = weight;

    if (millis() - lastUpdate > WS_UPDATE_INTERVAL_MS) {
        // Gestion transitions d'état OLED
        uint32_t now = millis();

        // Éviter le rollback à "Ready to weigh" :
        // Pendant la phase de countdown, forcer l'état SENDING (affiche "Sending... Xs")
        if (sendPhase == "countdown") {
            if (currentOledState != OLED_STATE_SENDING) {
                currentOledState = OLED_STATE_SENDING;
                oledStateChangeMs = now;
            }
        }

        // Retour à IDLE après affichage message (succès/erreur/UID détecté)
        if ((currentOledState == OLED_STATE_UID_DETECTED ||
             currentOledState == OLED_STATE_SUCCESS ||
             currentOledState == OLED_STATE_ERROR) &&
            (now - oledStateChangeMs > OLED_MESSAGE_DURATION_MS)) {
            if (sendPhase == "countdown") {
                currentOledState = OLED_STATE_SENDING; // rester sur SENDING (compte à rebours)
            } else {
                currentOledState = OLED_STATE_IDLE;
            }
        }

        // Passer à SENDING si handleAutoPush a initié l'envoi
        if (sendPhase == "send" && currentOledState != OLED_STATE_SENDING) {
            currentOledState = OLED_STATE_SENDING;
            oledStateChangeMs = now;
        }

        // Afficher avec l'état courant
        displayWeightWithState(displayedWeight, lastUID, currentOledState);

        int wInt = roundWeight(displayedWeight);
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