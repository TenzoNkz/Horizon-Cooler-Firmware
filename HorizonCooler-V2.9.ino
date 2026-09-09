/* =================================================================================
   [ UNDANG-UNDANG PENGEMBANGAN FIRMWARE HORIZON COOLER ]

   PASAL 1  : Firmware wajib menjaga kompatibilitas board, library, protokol, dan
              hardware yang sudah digunakan.
   PASAL 2  : Setiap perubahan harus diuji untuk compile/syntax, BLE, SYNC,
              voltage, Adaptive Mode, sensor, RGB, OTA, dan boot state.
   PASAL 3  : Komentar diperbolehkan untuk menjelaskan safety, state machine,
              timing, protokol, dan workaround penting.
   PASAL 4  : Buffer BLE/command wajib memiliki batas ukuran dan divalidasi.
   PASAL 5  : Data konfigurasi boleh disimpan di Preferences/NVS; data runtime
              dan safety tidak boleh bergantung hanya pada NVS.
   PASAL 6  : Penulisan NVS wajib ditunda/debounce dan tidak boleh setiap loop.
   PASAL 7  : Boot state harus aman: voltage 5V dan Adaptive Mode OFF.
   PASAL 8  : Hindari delay() pada loop utama; gunakan millis(), state machine,
              timeout, callback, dan yield() bila diperlukan.
   PASAL 9  : BLE RX menerima command aplikasi; BLE TX mengirim status/telemetry.
   PASAL 10 : Command invalid atau tidak dikenal wajib diabaikan dengan aman.
   PASAL 11 : Parser BLE harus tahan terhadap CR/LF, fragmentasi, multi-command,
              dan buffer overflow.
   PASAL 12 : SYNC wajib menggunakan <SYNC_START> ... <SYNC_END>.
   PASAL 13 : Setelah SYNC, gunakan Realtime Delta Sync. Satu notifikasi hanya
              memuat 1 baris utuh untuk 1 kelompok fungsi yang berubah.
   PASAL 14 : Sensor NTC harus divalidasi dan fault wajib mengarah ke kondisi aman.
   PASAL 15 : Manual Mode tidak boleh diubah algoritma temperatur saat Adaptive OFF.
   PASAL 16 : Adaptive Mode boleh menurunkan/mengatur voltage demi keselamatan.
   PASAL 17 : Setiap transisi voltage wajib memiliki dead-time hardware yang aman.
   PASAL 18 : 9V battery threshold bukan setting terpisah; 9V adalah zona di antara
              batas 5V dan 12V.
   PASAL 19 : Batas battery temperature configurable hanya 20C sampai 50C.
   PASAL 20 : LB5 dan LB12 adalah batas editable. LB5 selalu minimal 2 derajat
              di bawah LB12, sedangkan seluruh rentang 9V adalah LB5+1 sampai
              LB12-1 dan tidak dapat diedit.
   PASAL 21 : Profile Adaptive 0 menggunakan proteksi temperatur internal.
   PASAL 22 : Profile Adaptive 1 menggunakan battery temperature dari aplikasi
              melalui command BTP:x.x.
   PASAL 23 : Jika battery temperature tidak tersedia, Profile 1 wajib memilih
              kondisi voltage yang lebih aman.
   PASAL 24 : Battery temperature dari aplikasi tidak boleh dikirim terus-menerus;
              cukup saat state zona 5V/9V/12V berubah atau saat sinkronisasi awal.
   PASAL 25 : Overheat protection memiliki prioritas tertinggi.
   PASAL 26 : Hysteresis wajib digunakan agar voltage tidak berosilasi akibat noise.
   PASAL 27 : OTA harus memiliki timeout, validasi, dan safe fallback.
   PASAL 28 : Feedback LED tidak boleh menghambat BLE/sensor/voltage control.
   PASAL 29 : Safety hardware lebih tinggi daripada kenyamanan UI.
   PASAL 30 : Jika state tidak diketahui, pilih kondisi hardware paling aman.
   PASAL 31 : Perubahan firmware, sekecil apa pun, WAJIB menaikkan FIRMWARE_VERSION.
   PASAL 32 : Format versi resmi adalah V<major>.<minor>.
   PASAL 33 : Jangan memperbaiki satu fitur dengan membuat regresi fitur lain.

   [ TARGET PERILAKU BATTERY TEMPERATURE ]
       Battery Temp < LB5        -> maksimum 5V
       LB5 <= Battery Temp < LB12 -> 9V
       Battery Temp >= LB12       -> 12V

   =================================================================================
*/

#include <WiFi.h>           
#include <WebServer.h>      
#include <Update.h>         
#include <HTTPClient.h>     
#include <HTTPUpdate.h>     
#include <WiFiClientSecure.h>
#include <WS2812FX.h>       
#include <Preferences.h>    
#include <BLEDevice.h>      
#include <BLEServer.h>      
#include <BLEUtils.h>       
#include <BLE2902.h>        
#include <math.h>           

#define FIRMWARE_VERSION "V2.9"

#define PIN_NTC         0       
#define PIN_BTN_LEFT    5       
#define PIN_BTN_RIGHT   1       
#define PIN_OPTO_9V     3       
#define PIN_OPTO_12V    4       
#define PIN_LED         7       
#define TOTAL_LED       32      
#define MAX_LED_MODE    55      

#define TEMP_OFFSET     0.0f    

#define UUID_SECRET_SERVICE "f472a1b9-3c8d-4e9f-8a2b-6c7d8e9f0a1b" 
#define UUID_COMM_SERVICE   "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d" 
#define UUID_CHAR_RX        "b2c3d4e5-f6a7-4b5c-8d9e-1f2a3b4c5d6e" 
#define UUID_CHAR_TX        "c3d4e5f6-a7b8-4c5d-8e9f-2a3b4c5d6e7f" 

uint8_t gammaLUT[256]; 

char otaSsid[64] = {0};
char otaPass[64] = {0};
char otaUrl[250] = {0};

class HorizonThermistor 
{
  private:
    int       sensorPin;        
    float     fixedResistance;  
    float     ntcNominalRes;    
    float     invNominalTemp;   
    float     invBValue;        

  public:
    HorizonThermistor(int pin, float refRes, float nomRes, float nomTempC, float bVal) 
    {
        sensorPin          = pin;                         
        fixedResistance    = refRes;                      
        ntcNominalRes      = nomRes;                      
        invNominalTemp     = 1.0f / (nomTempC + 273.15f); 
        invBValue          = 1.0f / bVal;                 
    }

    int readRawMv() 
    {
        return analogReadMilliVolts(sensorPin);
    }

    float calculateCelsius(float mv) 
    {
        if (mv <= 50.0f) 
        {
            return -999.0f;
        }     
        if (mv >= 3250.0f) 
        {
            return 999.0f;
        }      
        float rNtc = fixedResistance * ((3300.0f / mv) - 1.0f); 
        if (rNtc <= 0.0f) 
        {
            return -999.0f;
        } 
        float tKelvin = 1.0f / (invNominalTemp + invBValue * log((double)(rNtc / ntcNominalRes)));
        return tKelvin - 273.15f; 
    }
};

struct ButtonState
{
    bool          currentState   = LOW;   
    bool          isLongPressed  = false; 
    int           tapCount       = 0;     
    unsigned long lastTapTime    = 0;     
    unsigned long pressStartTime = 0;     
};

HorizonThermistor ntcSensor(PIN_NTC, 9800.0f, 10000.0f, 25.0f, 3950.0f);
WS2812FX          ws2812fx = WS2812FX(TOTAL_LED, PIN_LED, NEO_GRB + NEO_KHZ800); 
Preferences       flashMemory; 
WebServer         server(80);                  
ButtonState       btnLeft;  
ButtonState       btnRight; 

bool          isEmergencyMode        = false; 
bool          isWifiActive           = false; 
bool          isSafeFlashMode        = false; 
bool          deviceEverConnected    = false; 
bool          isEmergencyLedBlinking = true;  
unsigned long lastEmergencyBlinkTime = 0;
bool          emergencyBlinkState    = false;
unsigned long wifiStartTime          = 0;     
unsigned long wifiTimeout            = 0;     

bool               isBleActive          = false; 
bool               isBlePermitted       = false; 
bool               isBleInitialized     = false; 
BLECharacteristic *globalTxChar         = NULL;  
bool          bleNeedsReadvertise  = false; 
unsigned long bleReadvertiseTimer  = 0;     

volatile int  bleTargetVoltage     = -1;    
volatile bool bleCmdToggleRgb      = false; 
volatile bool bleCmdNextMode       = false; 
volatile bool bleCmdPrevMode       = false; 
volatile int  bleTargetBrightness  = -1;    
volatile bool bleCmdSetBrightness  = false; 
volatile bool isAiModeActive       = false; 
volatile bool isBleClientConnected = false; 
volatile bool triggerCloudOta      = false; 
volatile bool isOtaStandby         = false; 
volatile uint8_t aiModeType        = 0; 
volatile float phoneBatteryTemp    = -999.0f;
volatile int   batteryProtectionLevel = -1; 

uint8_t       ledModeIndex              = 0;     
uint8_t       currentBrightness         = 255;   
bool          isLedOn                   = true;  
bool          isVoltageIndicatorActive  = false; 
unsigned long voltageIndicatorStartTime = 0;     

bool          isRadioLedBlinking        = false; 
int           radioBlinkPhase           = 0;     
uint32_t      radioBlinkColor           = 0;     
unsigned long lastRadioBlinkTime        = 0;     
bool          ledStateBeforeBlink       = false; 

int           currentVoltage             = 0;     
bool          isVoltageTransitioning     = false; 
unsigned long voltageTransitionStartTime = 0;     
int           targetTransitionVoltage    = 0;     
int           pendingVoltageRequest      = -1;
unsigned long lastVoltageExecutionTime   = 0;
bool          hasVoltageExecution        = false;

bool          adaptiveHoldAfter12Drop    = false;
bool          adaptiveLockedTo5          = false;
unsigned long adaptiveOverheatStartTime = 0;
unsigned long adaptiveSafeSince         = 0;

float         currentTemperature = 20.0f; 
unsigned long lastAdcReadTime    = 0;     
uint32_t      adcAccumulator     = 0;     
uint8_t       adcSampleCount     = 0;

bool          triggerInstantSync = false; 

unsigned long lastBrightnessChangeTime  = 0;
bool          pendingBrightnessSave     = false;

int           limitHot   = 45;
int           limitBat5v = 25;
int           limitBat9vMin = 26;
int           limitBat9vMax = 34;
int           limitBat12v= 35;
unsigned long lastTempLimitSaveTime = 0;
bool          pendingTempLimitSave = false;

volatile int  aiMaxVoltage = 2; 

float lastSentTemp = -999.0f;
int lastSentVoltage = -1;
bool lastSentRgb = false;
bool lastSentAi = false;
int lastSentAimType = -1;
int lastSentBr = -1;
int lastSentMode = -1;
int lastSentHot = -1;
int lastSentBat5 = -1;
int lastSentBat9Min = -1;
int lastSentBat9Max = -1;
int lastSentBat12 = -1;
uint8_t deltaGroupCursor = 0;
unsigned long lastTempSyncTime = 0;

void cleanBuffer(char* dest, const char* src, size_t maxLen);
uint8_t getHwBrightness(uint8_t appBrightness);
void sendInstantSyncData();
void sendRealtimeStateRow();
void sendRealtimeRgbRow();
void sendRealtimeTempRow();
void applyVoltageColor();                      
void startRadioBlink(uint32_t targetColor);    
void processRadioBlink();                      
void playBlink1000(uint32_t targetColor);
void setupOtaRoutes();
void stopWifi();                               
void enableSilentWifi(unsigned long duration); 
void suspendBle();                             
void resumeBle();                              
void disableAllRadios();                       
void enableBleSignal();                        
void performCloudOTA();
int calculateAiMaxVoltage();
void sanitizeTempLimits();

void safeYield(unsigned long ms) 
{
    unsigned long start = millis();
    while(millis() - start < ms) 
    {
        yield();
    }
}

void cleanBuffer(char* dest, const char* src, size_t maxLen)
{
    size_t dIdx = 0;
    for (size_t sIdx = 0; src[sIdx] != '\0' && dIdx < maxLen - 1; sIdx++)
    {
        if (src[sIdx] != '\r' && src[sIdx] != '\n')
        {
            dest[dIdx] = src[sIdx];
            dIdx++;
        }
    }
    dest[dIdx] = '\0';
}

uint8_t getHwBrightness(uint8_t appBrightness) 
{
    if (appBrightness > 255) 
    {
        appBrightness = 255;
    }
    return gammaLUT[appBrightness]; 
}

void sendInstantSyncData()
{
    sanitizeTempLimits();
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        float displayTemp = currentTemperature;

        const char* voltStr = "5V";
        if (currentVoltage == 1) 
        {
            voltStr = "9V";
        }
        else if (currentVoltage == 2) 
        {
            voltStr = "12V";
        }

        char frameBuffer[384];
        sprintf(frameBuffer, 
            "<SYNC_START>\n"
            "MD:%d\n"
            "BRV:%d\n"
            "RGB:%d\n"
            "TMP:%.1f\n"
            "VOL:%s\n"
            "AI:%d\n"
            "AIM:%d\n"
            "LHT:%d\n"
            "LB5:%d\n"
            "LB9:%d-%d\n"
            "LB12:%d\n"
            "VER:%s\n"
            "<SYNC_END>\n",
            ledModeIndex,
            currentBrightness,
            isLedOn ? 1 : 0,
            displayTemp,
            voltStr,
            isAiModeActive ? 1 : 0,
            aiModeType,
            limitHot,
            limitBat5v,
            limitBat9vMin,
            limitBat9vMax,
            limitBat12v,
            FIRMWARE_VERSION
        );

        globalTxChar->setValue((uint8_t*)frameBuffer, strlen(frameBuffer)); 
        globalTxChar->notify(); 

        lastSentMode = ledModeIndex;
        lastSentBr = currentBrightness;
        lastSentRgb = isLedOn;
        lastSentTemp = currentTemperature;
        lastSentVoltage = currentVoltage;
        lastSentAi = isAiModeActive;
        lastSentAimType = aiModeType;
        lastSentHot = limitHot;
        lastSentBat5 = limitBat5v;
        lastSentBat9Min = limitBat9vMin;
        lastSentBat9Max = limitBat9vMax;
        lastSentBat12 = limitBat12v;
        lastTempSyncTime = millis();
    }
}









void applyVoltageColor()
{
    if (currentVoltage == 0) 
    {
        ws2812fx.setColor(0xFF0000); 
    }
    else if (currentVoltage == 1) 
    {
        ws2812fx.setColor(0x00FF00); 
    }
    else 
    {
        ws2812fx.setColor(0x0000FF); 
    }
}

void startRadioBlink(uint32_t targetColor)
{
    isRadioLedBlinking       = true;  
    isVoltageIndicatorActive = false; 
    radioBlinkPhase          = 0;     
    radioBlinkColor          = targetColor; 
    lastRadioBlinkTime       = millis(); 
    ledStateBeforeBlink      = isLedOn;  
    ws2812fx.stop(); 
}

void processRadioBlink()
{
    if (isRadioLedBlinking == false) 
    {
        return; 
    }
    unsigned long elapsed = millis() - lastRadioBlinkTime; 
    uint8_t safeBrightnessLevel = (currentBrightness < 64) ? 64 : currentBrightness;
    uint8_t safeBrightness = getHwBrightness(safeBrightnessLevel);

    if (elapsed < 250) 
    {
        if (radioBlinkPhase != 1) 
        {
            ws2812fx.setBrightness(safeBrightness); 
            for (int i = 0; i < TOTAL_LED; i++) 
            {
                ws2812fx.setPixelColor(i, radioBlinkColor); 
            }
            ws2812fx.show(); 
            radioBlinkPhase = 1; 
        }
    }
    else if (elapsed < 500) 
    {
        if (radioBlinkPhase != 2) 
        {
            ws2812fx.clear(); 
            ws2812fx.show(); 
            radioBlinkPhase = 2; 
        }
    }
    else if (elapsed < 750) 
    {
        if (radioBlinkPhase != 3) 
        {
            ws2812fx.setBrightness(safeBrightness); 
            for (int i = 0; i < TOTAL_LED; i++) 
            {
                ws2812fx.setPixelColor(i, radioBlinkColor); 
            }
            ws2812fx.show(); 
            radioBlinkPhase = 3; 
        }
    }
    else if (elapsed < 1000) 
    {
        if (radioBlinkPhase != 4) 
        {
            ws2812fx.clear(); 
            ws2812fx.show(); 
            radioBlinkPhase = 4; 
        }
    }
    else 
    {
        isRadioLedBlinking = false; 
        if (ledStateBeforeBlink == true) 
        {
            ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
            ws2812fx.setMode(ledModeIndex); 
            applyVoltageColor(); 
        }
        else 
        {
            ws2812fx.setBrightness(0); 
            ws2812fx.stop(); 
            ws2812fx.clear(); 
            ws2812fx.show(); 
        }
    }
}

void playBlink1000(uint32_t targetColor)
{
    ws2812fx.stop();
    uint8_t safeBrightnessLevel = (currentBrightness < 64) ? 64 : currentBrightness;
    ws2812fx.setBrightness(getHwBrightness(safeBrightnessLevel));
    
    for (int b = 0; b < 2; b++)
    {
        for (int i = 0; i < TOTAL_LED; i++)
        {
            ws2812fx.setPixelColor(i, targetColor);
        }
        ws2812fx.show();
        safeYield(250);
        
        ws2812fx.clear();
        ws2812fx.show();
        safeYield(250);
    }
}

int calculateAiMaxVoltage()
{
    if (aiModeType == 0)
    {
        return 2;
    }

    if (phoneBatteryTemp < -100.0f)
    {
        return 0;
    }

    if (phoneBatteryTemp < (float)limitBat5v)
    {
        return 0;
    }

    if (phoneBatteryTemp >= (float)limitBat12v)
    {
        return 2;
    }

    return 1;
}

void sanitizeTempLimits()
{
    limitHot = constrain(limitHot, 30, 60);

    limitBat5v = constrain(limitBat5v, 20, 48);
    limitBat12v = constrain(limitBat12v, 22, 50);

    // Keep at least one whole integer degree available for 9V.
    if (limitBat12v < limitBat5v + 2)
    {
        if (limitBat5v >= 58)
        {
            limitBat5v = 48;
            limitBat12v = 50;
        }
        else
        {
            limitBat12v = limitBat5v + 2;
        }
    }

    limitBat9vMin = limitBat5v + 1;
    limitBat9vMax = limitBat12v - 1;
}

int getBatteryProtectionLevel()
{
    if (phoneBatteryTemp < -100.0f)
    {
        return 0;
    }

    if (phoneBatteryTemp < (float)limitBat5v)
    {
        return 0;
    }

    if (phoneBatteryTemp >= (float)limitBat12v)
    {
        return 2;
    }

    return 1;
}

void setupOtaRoutes()
{
    server.on("/", HTTP_GET, []()
    {
        server.send(200, "text/html", 
            "<h1>Horizon Cooler (Fail-Safe OTA)</h1>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update'>"
            "<input type='submit' value='Upload / Update'>"
            "</form>");
    });

    server.on("/update", HTTP_POST, []()
    {
        if (Update.hasError())
        {
            server.send(500, "text/plain", "UPDATE FAILED!");
        }
        else
        {
            server.send(200, "text/plain", "FIRMWARE RECEIVED. REBOOTING KERNEL..."); 
            safeYield(1000); 
            WiFi.softAPdisconnect(true); 
            WiFi.mode(WIFI_OFF); 
            ESP.restart(); 
        }
    }, []() 
    {
        HTTPUpload& uploadData = server.upload(); 
        if (uploadData.status == UPLOAD_FILE_START) 
        {
            Update.begin(UPDATE_SIZE_UNKNOWN); 
        }
        else if (uploadData.status == UPLOAD_FILE_WRITE) 
        {
            Update.write(uploadData.buf, uploadData.currentSize); 
        }
        else if (uploadData.status == UPLOAD_FILE_END) 
        {
            Update.end(true); 
        }
    });
}

void stopWifi()
{
    if (isWifiActive == true) 
    {
        server.stop(); 
        WiFi.softAPdisconnect(true); 
        WiFi.mode(WIFI_OFF); 
        isWifiActive    = false; 
        isSafeFlashMode = false; 
        if (isBlePermitted == true) 
        {
            resumeBle(); 
        }
    }
}

void enableSilentWifi(unsigned long duration)
{
    disableAllRadios(); 
    WiFi.mode(WIFI_AP); 
    WiFi.softAP("Horizon Cooler"); 
    server.begin(); 
    isWifiActive          = true; 
    wifiTimeout           = duration; 
    wifiStartTime         = millis(); 
    deviceEverConnected   = false; 
}

void suspendBle()
{
    if (isBleActive == true && isBleInitialized == true) 
    {
        BLEDevice::getAdvertising()->stop(); 
        isBleActive = false; 
        isBleClientConnected = false; 
    }
}

void resumeBle()
{
    if (isBleInitialized == false) 
    {
        enableBleSignal(); 
    }
    else if (isBleActive == false) 
    {
        BLEDevice::getAdvertising()->start(); 
        isBleActive = true; 
    }
}

void disableAllRadios()
{
    stopWifi(); 
    suspendBle(); 
}

class BleConnCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer* bleServer)
    {
        isBleClientConnected = true; 
        bleNeedsReadvertise  = false; 
        lastSentTemp = -999.0f;
        lastSentVoltage = -1;
        lastSentRgb = !isLedOn;
        lastSentAi = !isAiModeActive;
        lastSentBr = -1;
        lastSentMode = -1;
        lastSentHot = -1;
        lastSentBat5 = -1;
        lastSentBat9Min = -1;
        lastSentBat9Max = -1;
        lastSentBat12 = -1;
        BLEDevice::getAdvertising()->stop(); 
    }

    void onDisconnect(BLEServer* bleServer)
    {
        isBleClientConnected = false; 
        if (isBleActive == true && isWifiActive == false) 
        {
            bleNeedsReadvertise = true; 
            bleReadvertiseTimer = millis(); 
        }
    }
};

class BleMsgCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic* bleCharacteristic)
    {
        if (isBleActive == false) 
        {
            return; 
        }

        uint8_t *rawData    = bleCharacteristic->getData(); 
        size_t   dataLength = bleCharacteristic->getLength(); 
        if (dataLength == 0 || rawData == nullptr) 
        {
            return; 
        }

        char rawStr[256];
        size_t copyLen = dataLength < sizeof(rawStr) - 1 ? dataLength : sizeof(rawStr) - 1;
        memcpy(rawStr, rawData, copyLen);
        rawStr[copyLen] = '\0';

        char* token = strtok(rawStr, "\n");
        while (token != NULL)
        {
            if (strncmp(token, "SSID:", 5) == 0) 
            {
                cleanBuffer(otaSsid, token + 5, 64);
            } 
            else if (strncmp(token, "PASS:", 5) == 0) 
            {
                cleanBuffer(otaPass, token + 5, 64);
            } 
            else if (strncmp(token, "URL:", 4) == 0) 
            {
                cleanBuffer(otaUrl, token + 4, 250);
            }
            else
            {
                size_t len = strlen(token);
                if (len > 191) 
                {
                    len = 191;
                }
                char cmdBuf[192]; 
                size_t idx = 0; 
                for (size_t i = 0; i < len; i++)
                {
                    char c = token[i]; 
                    if (c != ' ' && c != '\r') 
                    {
                        if (c >= 'a' && c <= 'z') 
                        {
                            c = c - 32;  
                        }
                        cmdBuf[idx] = c; 
                        idx++; 
                    }
                }
                cmdBuf[idx] = '\0'; 

                if (strncmp(cmdBuf, "CFG:", 4) == 0 ||
                    strncmp(cmdBuf, "ADAPT:", 6) == 0 ||
                    strncmp(cmdBuf, "TEMPSET:", 8) == 0 ||
                    strncmp(cmdBuf, "PHONE:", 6) == 0)
                {
                    char* group = cmdBuf + ((cmdBuf[0] == 'C') ? 4 :
                                            (cmdBuf[0] == 'A') ? 6 :
                                            (cmdBuf[0] == 'T') ? 8 : 6);
                    char* savePtr = NULL;
                    char* item = strtok_r(group, ";", &savePtr);
                    while (item != NULL)
                    {
                        char* eq = strchr(item, '=');
                        if (eq != NULL)
                        {
                            *eq = '\0';
                            const char* k = item;
                            const char* v = eq + 1;

                            if (strcmp(k, "AI") == 0 || strcmp(k, "ON") == 0)
                            {
                                int enabled = atoi(v) != 0 ? 1 : 0;
                                if (enabled && !isAiModeActive)
                                {
                                    // Take control immediately. Safety downshifts are evaluated by the
                                    // Adaptive state machine; upward changes use its 5-second timer.
                                    isAiModeActive = true;
                                    adaptiveHoldAfter12Drop = false;
                                    adaptiveLockedTo5 = false;
                                    adaptiveOverheatStartTime = 0;
                                    adaptiveSafeSince = 0;
                                    aiMaxVoltage = calculateAiMaxVoltage();
                                }
                                else if (!enabled && isAiModeActive)
                                {
                                    isAiModeActive = false;
                                    adaptiveHoldAfter12Drop = false;
                                    adaptiveLockedTo5 = false;
                                    adaptiveOverheatStartTime = 0;
                                    adaptiveSafeSince = 0;
                                    pendingVoltageRequest = 0;
                                    targetTransitionVoltage = 0;
                                    isVoltageTransitioning = true;
                                    voltageTransitionStartTime = millis();
                                    digitalWrite(PIN_OPTO_9V, LOW);
                                    digitalWrite(PIN_OPTO_12V, LOW);
                                }
                            }
                            else if (strcmp(k, "MODE") == 0 || strcmp(k, "AIM") == 0)
                            {
                                int parsedAiMode = atoi(v);
                                if (parsedAiMode == 0 || parsedAiMode == 1)
                                {
                                    aiModeType = (uint8_t)parsedAiMode;
                                    if (isAiModeActive)
                                    {
                                        aiMaxVoltage = calculateAiMaxVoltage();
                                    }
                                }
                            }
                            else if (strcmp(k, "LHT") == 0)
                            {
                                limitHot = atoi(v);
                            }
                            else if (strcmp(k, "LB5") == 0)
                            {
                                limitBat5v = atoi(v);
                            }
                            else if (strcmp(k, "LB12") == 0)
                            {
                                limitBat12v = atoi(v);
                            }
                            else if (strcmp(k, "VOL") == 0)
                            {
                                int requested = 0;
                                if (strcmp(v, "9V") == 0 || strcmp(v, "1") == 0) requested = 1;
                                else if (strcmp(v, "12V") == 0 || strcmp(v, "2") == 0) requested = 2;
                                pendingVoltageRequest = requested;
                            }
                            else if (strcmp(k, "RGB") == 0)
                            {
                                isLedOn = atoi(v) != 0;
                            }
                            else if (strcmp(k, "MD") == 0)
                            {
                                int requestedMode = atoi(v);
                                if (requestedMode >= 0 && requestedMode <= MAX_LED_MODE) ledModeIndex = (uint8_t)requestedMode;
                            }
                            else if (strcmp(k, "BR") == 0)
                            {
                                int requestedBrightness = atoi(v);
                                if (requestedBrightness >= 1 && requestedBrightness <= 255) currentBrightness = (uint8_t)requestedBrightness;
                            }
                            else if (strcmp(k, "BT") == 0)
                            {
                                float parsedBatteryTemp = atof(v);
                                if (parsedBatteryTemp >= -20.0f && parsedBatteryTemp <= 100.0f)
                                {
                                    phoneBatteryTemp = parsedBatteryTemp;
                                    batteryProtectionLevel = getBatteryProtectionLevel();
                                }
                            }
                        }
                        item = strtok_r(NULL, ";", &savePtr);
                    }

                    sanitizeTempLimits();
                    if (isAiModeActive)
                    {
                        aiMaxVoltage = calculateAiMaxVoltage();
                    }
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strcmp(cmdBuf, "OTAENTER") == 0) 
                {
                    isOtaStandby = true; 
                    digitalWrite(PIN_OPTO_9V, LOW); 
                    digitalWrite(PIN_OPTO_12V, LOW); 
                    currentVoltage = 0;
                    isAiModeActive = false;
                    ws2812fx.stop(); 
                    ws2812fx.clear(); 
                    ws2812fx.show();
                }
                else if (strcmp(cmdBuf, "OTAEXIT") == 0) 
                {
                    isOtaStandby = false;
                    if (isLedOn == true) 
                    {
                        ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
                        ws2812fx.setMode(ledModeIndex); 
                        applyVoltageColor(); 
                        ws2812fx.start(); 
                    }
                }
                else if (strcmp(cmdBuf, "5V") == 0) 
                {
                    bleTargetVoltage = 0;    
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                            }
                }
                else if (strcmp(cmdBuf, "9V") == 0) 
                {
                    bleTargetVoltage = 1;    
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                            }
                }
                else if (strcmp(cmdBuf, "12V") == 0) 
                {
                    bleTargetVoltage = 2;    
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                            }
                }
                else if (strcmp(cmdBuf, "AION") == 0) 
                {
                    if (isAiModeActive == false) 
                    {
                        // Take control immediately. The state machine handles safety downshift now
                        // and keeps upward changes on the 5-second adaptive timer.
                        isAiModeActive = true;
                        adaptiveHoldAfter12Drop = false;
                        adaptiveLockedTo5 = false;
                        adaptiveOverheatStartTime = 0;
                        adaptiveSafeSince = 0;
                        aiMaxVoltage = calculateAiMaxVoltage();
                    }
                }
                else if (strcmp(cmdBuf, "AIOFF") == 0) 
                {
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false;
                        adaptiveHoldAfter12Drop = false;
                        adaptiveLockedTo5 = false;
                        adaptiveOverheatStartTime = 0;
                        adaptiveSafeSince = 0;
        
                        targetTransitionVoltage  = 0; 
                        isVoltageTransitioning   = true; 
                        voltageTransitionStartTime = millis(); 
                        digitalWrite(PIN_OPTO_9V,  LOW); 
                        digitalWrite(PIN_OPTO_12V, LOW);
                    }
                }
                else if (strcmp(cmdBuf, "RGBTOGGLE") == 0 || strcmp(cmdBuf, "RGB_TOGGLE") == 0) 
                {
                    bleCmdToggleRgb = true; 
                }
                else if (strcmp(cmdBuf, "RGBNEXT") == 0 || strcmp(cmdBuf, "RGB_NEXT") == 0) 
                {
                    bleCmdNextMode = true; 
                }
                else if (strcmp(cmdBuf, "RGBPREV") == 0 || strcmp(cmdBuf, "RGB_PREV") == 0) 
                {
                    bleCmdPrevMode = true; 
                }
                else if (strncmp(cmdBuf, "BR:", 3) == 0) 
                {
                    int parsedVal = atoi(cmdBuf + 3); 
                    if (parsedVal >= 1 && parsedVal <= 255) 
                    {
                        bleTargetBrightness = parsedVal; 
                        bleCmdSetBrightness = true; 
                    }
                }
                else if (strncmp(cmdBuf, "AIM:", 4) == 0)
                {
                    int parsedAiMode = atoi(cmdBuf + 4);
                    if (parsedAiMode == 0 || parsedAiMode == 1)
                    {
                        aiModeType = (uint8_t)parsedAiMode;
                        if (isAiModeActive == true)
                        {
                            aiMaxVoltage = calculateAiMaxVoltage();
                        }
                    }
                }
                else if (strncmp(cmdBuf, "BTP:", 4) == 0)
                {
                    float parsedBatteryTemp = atof(cmdBuf + 4);
                    if (parsedBatteryTemp >= -20.0f && parsedBatteryTemp <= 100.0f)
                    {
                        int previousLevel = batteryProtectionLevel;
                        phoneBatteryTemp = parsedBatteryTemp;
                        batteryProtectionLevel = getBatteryProtectionLevel();

                        if (isAiModeActive == true && aiModeType == 1)
                        {
                            if (batteryProtectionLevel != previousLevel || previousLevel < 0)
                            {
                                aiMaxVoltage = calculateAiMaxVoltage();
                            }
                        }
                    }
                }
                else if (strncmp(cmdBuf, "LHT:", 4) == 0)
                {
                    limitHot = atoi(cmdBuf + 4);
                    sanitizeTempLimits();
                    if (isAiModeActive == true)
                    {
                        aiMaxVoltage = calculateAiMaxVoltage();
                    }
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strncmp(cmdBuf, "LB5:", 4) == 0)
                {
                    limitBat5v = atoi(cmdBuf + 4);
                    sanitizeTempLimits();
                    if (isAiModeActive == true)
                    {
                        aiMaxVoltage = calculateAiMaxVoltage();
                    }
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strncmp(cmdBuf, "LB9:", 4) == 0)
                {
                    sanitizeTempLimits();
                }
                else if (strncmp(cmdBuf, "LB12:", 5) == 0)
                {
                    limitBat12v = atoi(cmdBuf + 5);
                    sanitizeTempLimits();
                    if (isAiModeActive == true)
                    {
                        aiMaxVoltage = calculateAiMaxVoltage();
                    }
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strcmp(cmdBuf, "SYNC") == 0) 
                {
                    triggerInstantSync = true;  
                }
                else if (strcmp(cmdBuf, "CLOUDOTA") == 0) 
                {
                    triggerCloudOta = true;  
                }
            }
            token = strtok(NULL, "\n");
        }
    }
};

void enableBleSignal()
{
    disableAllRadios(); 

    if (isBleInitialized == false) 
    {
        BLEDevice::init("Horizon Cooler"); 
        BLEServer *bleServer = BLEDevice::createServer(); 
        bleServer->setCallbacks(new BleConnCallbacks()); 

        BLEService *commService = bleServer->createService(UUID_COMM_SERVICE); 

        BLECharacteristic *rxChar = commService->createCharacteristic(
            UUID_CHAR_RX,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR
        );
        rxChar->setCallbacks(new BleMsgCallbacks()); 

        globalTxChar = commService->createCharacteristic(
            UUID_CHAR_TX,
            BLECharacteristic::PROPERTY_NOTIFY
        );
        globalTxChar->addDescriptor(new BLE2902()); 

        commService->start(); 

        BLEService *secretService = bleServer->createService(UUID_SECRET_SERVICE);
        secretService->start(); 

        BLEAdvertising *bleAd = BLEDevice::getAdvertising(); 
        bleAd->addServiceUUID(UUID_SECRET_SERVICE); 
        bleAd->setScanResponse(true); 
        bleAd->setMinPreferred(0x06); 
        bleAd->setMinPreferred(0x12); 

        isBleInitialized = true; 
    }

    BLEDevice::getAdvertising()->start(); 
    isBleActive       = true; 
    isBlePermitted    = true; 
    isBleClientConnected = false; 
    bleNeedsReadvertise = false; 
}

void restoreBleAfterOtaFailure()
{
    WiFi.disconnect(false);
    WiFi.mode(WIFI_OFF);
    isWifiActive = false;
    isSafeFlashMode = false;
    isOtaStandby = false;
    triggerCloudOta = false;

    if (isBleInitialized == true)
    {
        BLEDevice::getAdvertising()->start();
        isBleActive = true;
        isBlePermitted = true;
        isBleClientConnected = false;
        bleNeedsReadvertise = false;
    }
    else
    {
        enableBleSignal();
    }

    if (isLedOn == true)
    {
        ws2812fx.setBrightness(getHwBrightness(currentBrightness));
        ws2812fx.setMode(ledModeIndex);
        applyVoltageColor();
        ws2812fx.start();
    }
    else
    {
        ws2812fx.setBrightness(0);
        ws2812fx.stop();
        ws2812fx.clear();
        ws2812fx.show();
    }
}

void performCloudOTA()
{
    triggerCloudOta = false;

    // OTA safety state: always force 5V and suspend Adaptive control.
    digitalWrite(PIN_OPTO_9V, LOW);
    digitalWrite(PIN_OPTO_12V, LOW);
    currentVoltage = 0;
    isAiModeActive = false;

    // Persist the complete OTA configuration once, after BLE has delivered all fields.
    flashMemory.putString("ota_ssid", otaSsid);
    flashMemory.putString("ota_pass", otaPass);
    flashMemory.putString("ota_url", otaUrl);

    ws2812fx.stop();
    ws2812fx.clear();
    ws2812fx.show();

    if (isBleInitialized == true)
    {
        BLEDevice::getAdvertising()->stop();
        isBleActive = false;
    }

    if (otaSsid[0] == '\0' || otaUrl[0] == '\0')
    {
        playBlink1000(0xFF0000);
        restoreBleAfterOtaFailure();
        return;
    }

    WiFi.disconnect(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    safeYield(200);

    bool connected = false;
    for (int attempt = 0; attempt < 2 && connected == false; attempt++)
    {
        WiFi.disconnect(false);
        safeYield(250);
        WiFi.begin(otaSsid, otaPass);

        unsigned long startWait = millis();
        while (millis() - startWait < 20000UL)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                connected = true;
                break;
            }
            safeYield(100);
        }
    }

    if (connected == false)
    {
        // WiFi failed: red blink, then immediately return to BLE without rebooting.
        playBlink1000(0xFF0000);
        restoreBleAfterOtaFailure();
        return;
    }

    // WiFi connected: blue blink, then download/install firmware.
    playBlink1000(0x0000FF);

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(20000);

    t_httpUpdate_return ret = httpUpdate.update(client, otaUrl);

    if (ret == HTTP_UPDATE_OK)
    {
        // OTA success: green blink, then reboot.
        playBlink1000(0x00FF00);
        WiFi.disconnect(false);
        WiFi.mode(WIFI_OFF);
        safeYield(500);
        ESP.restart();
        return;
    }

    // OTA download/install failed: red blink, then restore BLE without reboot.
    playBlink1000(0xFF0000);
    restoreBleAfterOtaFailure();
}

void setup()
{
    pinMode(PIN_NTC,       INPUT);  
    pinMode(PIN_BTN_LEFT,  INPUT);  
    pinMode(PIN_BTN_RIGHT, INPUT);  
    pinMode(PIN_OPTO_9V,   OUTPUT); 
    pinMode(PIN_OPTO_12V,  OUTPUT); 

    analogReadResolution(12); 

    digitalWrite(PIN_OPTO_9V,  LOW); 
    digitalWrite(PIN_OPTO_12V, LOW); 

    for (int i = 0; i <= 255; i++) 
    {
        float norm = i / 255.0f; 
        int val = (int)(pow(norm, 2.5f) * 255.0f); 
        if (i > 0 && val < 3) 
        {
            val = 3; 
        }
        if (val > 255) 
        {
            val = 255; 
        }
        gammaLUT[i] = (uint8_t)val; 
    }

    flashMemory.begin("funcooler", false); 
    ledModeIndex      = flashMemory.getUChar("mode",       0); 
    currentBrightness = flashMemory.getUChar("brightness", 255); 
    isLedOn           = flashMemory.getBool("status", true); 
    
    currentVoltage    = 0;
    isAiModeActive    = false;
    aiModeType        = 0;
    phoneBatteryTemp  = -999.0f;
    batteryProtectionLevel = -1;

    limitHot          = flashMemory.getInt("limit_hot", 45);
    limitBat5v        = flashMemory.getInt("limit_b5", 25);
    limitBat12v       = flashMemory.getInt("limit_b12", 35);
    sanitizeTempLimits();
    
    flashMemory.getString("ota_ssid", otaSsid, 64); 
    flashMemory.getString("ota_pass", otaPass, 64); 
    flashMemory.getString("ota_url", otaUrl, 250); 

    if (ledModeIndex > MAX_LED_MODE) 
    {
        ledModeIndex = 0; 
    }
    if (currentBrightness < 1 || currentBrightness > 255) 
    {
        currentBrightness = 255; 
    }

    digitalWrite(PIN_OPTO_9V,  LOW); 
    digitalWrite(PIN_OPTO_12V, LOW); 

    ws2812fx.init();  
    ws2812fx.clear(); 
    ws2812fx.show();  

    unsigned long bootCheck = millis(); 
    while (millis() - bootCheck < 2000) 
    {
        if (digitalRead(PIN_BTN_LEFT) == HIGH && digitalRead(PIN_BTN_RIGHT) == HIGH)
        {
            isEmergencyMode = true; 
            break; 
        }
        safeYield(10); 
    }

    setupOtaRoutes(); 

    if (isEmergencyMode == true) 
    {
        uint8_t safeOtaBrightness = (currentBrightness < 64) ? 64 : currentBrightness; 
        ws2812fx.setBrightness(getHwBrightness(safeOtaBrightness)); 
        for (int i = 0; i < TOTAL_LED; i++)
        {
            ws2812fx.setPixelColor(i, 0xFF0000); 
        }
        ws2812fx.show(); 
        emergencyBlinkState = true; 
        lastEmergencyBlinkTime = millis(); 

        WiFi.mode(WIFI_AP); 
        WiFi.softAP("OTA_DARURAT"); 
        server.begin(); 
        return; 
    }

    if (isLedOn == true) 
    {
        ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
        ws2812fx.setMode(ledModeIndex); 
        applyVoltageColor(); 
        ws2812fx.start(); 
    }
    else 
    {
        ws2812fx.setBrightness(0); 
        ws2812fx.stop(); 
        ws2812fx.clear(); 
        ws2812fx.show(); 
    }

    enableSilentWifi(10000); 
}

void loop()
{
    if (triggerCloudOta == true)
    {
        performCloudOTA(); 
    }

    if (triggerInstantSync == true)
    {
        triggerInstantSync = false; 
        sendInstantSyncData(); 
    }

    if (isOtaStandby == true) 
    {
        return; 
    }

    if (isEmergencyMode == true) 
    {
        server.handleClient(); 
        if (WiFi.softAPgetStationNum() > 0) 
        {
            if (isEmergencyLedBlinking == true) 
            {
                ws2812fx.setBrightness(0); 
                ws2812fx.clear(); 
                ws2812fx.show(); 
                isEmergencyLedBlinking = false; 
            }
        }
        else 
        {
            isEmergencyLedBlinking = true; 
            if (millis() - lastEmergencyBlinkTime >= 500)
            {
                lastEmergencyBlinkTime = millis(); 
                emergencyBlinkState = !emergencyBlinkState; 
                uint8_t safeOtaBrightness = (currentBrightness < 64) ? 64 : currentBrightness; 
                ws2812fx.setBrightness(getHwBrightness(safeOtaBrightness)); 
                if (emergencyBlinkState == true)
                {
                    for (int i = 0; i < TOTAL_LED; i++)
                    {
                        ws2812fx.setPixelColor(i, 0xFF0000); 
                    }
                }
                else
                {
                    ws2812fx.clear(); 
                }
                ws2812fx.show(); 
            }
        }
        return; 
    }

    if (isWifiActive == true) 
    {
        server.handleClient(); 
        int clientCount = WiFi.softAPgetStationNum(); 
        if (clientCount > 0) 
        {
            deviceEverConnected = true; 
        }
        bool clientDisconnected = (clientCount == 0 && deviceEverConnected == true); 
        bool timedOut           = (clientCount == 0 && millis() - wifiStartTime >= wifiTimeout); 
        if (clientDisconnected == true || timedOut == true) 
        {
            stopWifi(); 
        }
    }

    if (isWifiActive == true && WiFi.softAPgetStationNum() > 0) 
    {
        if (isSafeFlashMode == false) 
        {
            isSafeFlashMode          = true; 
            isRadioLedBlinking       = false; 
            isVoltageIndicatorActive = false; 
            ws2812fx.setBrightness(0); 
            ws2812fx.stop(); 
            ws2812fx.clear(); 
            ws2812fx.show(); 
            digitalWrite(PIN_OPTO_9V,  LOW); 
            digitalWrite(PIN_OPTO_12V, LOW); 
        }
    }
    else if (isSafeFlashMode == true) 
    {
        isSafeFlashMode = false; 
        if (isLedOn == true) 
        {
            ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
            ws2812fx.setMode(ledModeIndex); 
            applyVoltageColor(); 
            ws2812fx.start(); 
        }
        digitalWrite(PIN_OPTO_9V,  (currentVoltage == 1) ? HIGH : LOW); 
        digitalWrite(PIN_OPTO_12V, (currentVoltage == 2) ? HIGH : LOW); 
    }

    if (isSafeFlashMode == true) 
    {
        return; 
    }

    if (isRadioLedBlinking == false && isVoltageIndicatorActive == false)
    {
        ws2812fx.service(); 
    }
    processRadioBlink(); 

    if (millis() - lastAdcReadTime >= 20) 
    {
        lastAdcReadTime = millis(); 
        adcAccumulator += ntcSensor.readRawMv(); 
        adcSampleCount++; 

        if (adcSampleCount >= 3) 
        {
            float avgMv = (float)adcAccumulator / 3.0f; 
            float fetchedTemp = ntcSensor.calculateCelsius(avgMv); 
            
            if (fetchedTemp == -999.0f || fetchedTemp == 999.0f) 
            {
                currentTemperature = 999.0f; 
            }
            else 
            {
                currentTemperature = fetchedTemp + TEMP_OFFSET; 
            }
            
            adcAccumulator = 0; 
            adcSampleCount = 0; 
        }
    }

    if (isAiModeActive == true)
    {
        const unsigned long now = millis();
        aiMaxVoltage = calculateAiMaxVoltage();

        if (adaptiveLockedTo5)
        {
            if (currentVoltage != 0 && !isVoltageTransitioning)
            {
                targetTransitionVoltage = 0;
                isVoltageTransitioning = true;
                voltageTransitionStartTime = now;
                digitalWrite(PIN_OPTO_9V, LOW);
                digitalWrite(PIN_OPTO_12V, LOW);
            }
        }
        else if (currentVoltage == 2 && currentTemperature >= (float)limitHot && !isVoltageTransitioning)
        {
            targetTransitionVoltage = 1;
            isVoltageTransitioning = true;
            voltageTransitionStartTime = now;
            adaptiveHoldAfter12Drop = true;
            adaptiveOverheatStartTime = now;
            adaptiveSafeSince = 0;
            digitalWrite(PIN_OPTO_9V, LOW);
            digitalWrite(PIN_OPTO_12V, LOW);
        }
        else if (currentVoltage == 1 && currentTemperature >= (float)limitHot && !isVoltageTransitioning && !adaptiveHoldAfter12Drop)
        {
            targetTransitionVoltage = 0;
            isVoltageTransitioning = true;
            voltageTransitionStartTime = now;
            adaptiveLockedTo5 = true;
            adaptiveHoldAfter12Drop = false;
            adaptiveSafeSince = 0;
            digitalWrite(PIN_OPTO_9V, LOW);
            digitalWrite(PIN_OPTO_12V, LOW);
        }
        else if (adaptiveHoldAfter12Drop && currentVoltage == 1 && !isVoltageTransitioning)
        {
            if (currentTemperature < (float)limitHot)
            {
                adaptiveHoldAfter12Drop = false;
                adaptiveOverheatStartTime = 0;
                adaptiveSafeSince = 0;
            }
            else if (now - adaptiveOverheatStartTime >= 5000UL)
            {
                targetTransitionVoltage = 0;
                isVoltageTransitioning = true;
                voltageTransitionStartTime = now;
                adaptiveLockedTo5 = true;
                adaptiveHoldAfter12Drop = false;
                adaptiveSafeSince = 0;
                digitalWrite(PIN_OPTO_9V, LOW);
                digitalWrite(PIN_OPTO_12V, LOW);
            }
        }
        else
        {
            if (currentVoltage > aiMaxVoltage && !isVoltageTransitioning)
            {
                targetTransitionVoltage = aiMaxVoltage;
                isVoltageTransitioning = true;
                voltageTransitionStartTime = now;
                digitalWrite(PIN_OPTO_9V, LOW);
                digitalWrite(PIN_OPTO_12V, LOW);
                adaptiveSafeSince = 0;
            }
            else if (currentTemperature <= (float)(limitHot - 3) && currentVoltage < aiMaxVoltage)
            {
                if (adaptiveSafeSince == 0) adaptiveSafeSince = now;
                if (now - adaptiveSafeSince >= 5000UL && !isVoltageTransitioning)
                {
                    int nextVoltage = currentVoltage + 1;
                    if (nextVoltage > aiMaxVoltage) nextVoltage = aiMaxVoltage;
                    if (nextVoltage != currentVoltage)
                    {
                        targetTransitionVoltage = nextVoltage;
                        isVoltageTransitioning = true;
                        voltageTransitionStartTime = now;
                        adaptiveSafeSince = now;
                        digitalWrite(PIN_OPTO_9V, LOW);
                        digitalWrite(PIN_OPTO_12V, LOW);
                    }
                }
            }
            else
            {
                adaptiveSafeSince = 0;
            }
        }
    }

    if (isVoltageTransitioning == true && (millis() - voltageTransitionStartTime >= 100))
    {
        isVoltageTransitioning = false;
        currentVoltage = targetTransitionVoltage;
        lastVoltageExecutionTime = millis();
        hasVoltageExecution = true;
        pendingVoltageRequest = -1;
        digitalWrite(PIN_OPTO_9V, (currentVoltage == 1) ? HIGH : LOW);
        digitalWrite(PIN_OPTO_12V, (currentVoltage == 2) ? HIGH : LOW);
        applyVoltageColor();
    }

    isVoltageIndicatorActive = false;



    if (isBleActive == true && isBleClientConnected == true)
    {
        const unsigned long now = millis();
        char rowBuffer[192];

        // Hotside temperature is guaranteed at 1 Hz. Voltage and Adaptive state
        // piggyback on this same complete STATE row whenever they change.
        const bool stateChanged =
            currentVoltage != lastSentVoltage ||
            isAiModeActive != lastSentAi ||
            aiModeType != lastSentAimType;
        const bool tempTelemetryDue = (now - lastTempSyncTime >= 1000UL);
        if (stateChanged || tempTelemetryDue)
        {
            const char* voltStr = "5V";
            if (currentVoltage == 1) voltStr = "9V";
            else if (currentVoltage == 2) voltStr = "12V";

            snprintf(
                rowBuffer,
                sizeof(rowBuffer),
                "STATE:TMP=%.1f;VOL=%s;AI=%d;AIM=%d\n",
                currentTemperature,
                voltStr,
                isAiModeActive ? 1 : 0,
                aiModeType
            );
            globalTxChar->setValue((uint8_t*)rowBuffer, strlen(rowBuffer));
            globalTxChar->notify();

            lastSentTemp = currentTemperature;
            lastSentVoltage = currentVoltage;
            lastSentAi = isAiModeActive;
            lastSentAimType = aiModeType;
            lastTempSyncTime = now;
        }

        // Other functional groups are event-driven: transmit the complete row
        // as soon as a value changes, and never repeatedly when unchanged.
        const bool rgbChanged =
            isLedOn != lastSentRgb ||
            ledModeIndex != lastSentMode ||
            currentBrightness != lastSentBr;
        if (rgbChanged)
        {
            snprintf(
                rowBuffer,
                sizeof(rowBuffer),
                "RGBSTATE:RGB=%d;MD=%d;BR=%d\n",
                isLedOn ? 1 : 0,
                ledModeIndex,
                currentBrightness
            );
            globalTxChar->setValue((uint8_t*)rowBuffer, strlen(rowBuffer));
            globalTxChar->notify();
            lastSentRgb = isLedOn;
            lastSentMode = ledModeIndex;
            lastSentBr = currentBrightness;
        }

        const bool tempSettingChanged =
            limitHot != lastSentHot ||
            limitBat5v != lastSentBat5 ||
            limitBat9vMin != lastSentBat9Min ||
            limitBat9vMax != lastSentBat9Max ||
            limitBat12v != lastSentBat12;
        if (tempSettingChanged)
        {
            snprintf(
                rowBuffer,
                sizeof(rowBuffer),
                "TEMPSET:LHT=%d;LB5=%d;LB9=%d-%d;LB12=%d\n",
                limitHot,
                limitBat5v,
                limitBat9vMin,
                limitBat9vMax,
                limitBat12v
            );
            globalTxChar->setValue((uint8_t*)rowBuffer, strlen(rowBuffer));
            globalTxChar->notify();
            lastSentHot = limitHot;
            lastSentBat5 = limitBat5v;
            lastSentBat9Min = limitBat9vMin;
            lastSentBat9Max = limitBat9vMax;
            lastSentBat12 = limitBat12v;
        }
    }

    if (bleCmdSetBrightness == true) 
    {
        bleCmdSetBrightness = false; 
        if (bleTargetBrightness < 1) 
        {
            bleTargetBrightness = 1; 
        }
        currentBrightness = (uint8_t)bleTargetBrightness; 

        if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
        }

        pendingBrightnessSave = true; 
        lastBrightnessChangeTime = millis(); 
    }

    if (pendingBrightnessSave == true && (millis() - lastBrightnessChangeTime >= 2000))
    {
        pendingBrightnessSave = false; 
        if (flashMemory.getUChar("brightness", 255) != currentBrightness) 
        {
            flashMemory.putUChar("brightness", currentBrightness); 
        }
    }

    if (pendingTempLimitSave == true && (millis() - lastTempLimitSaveTime >= 2000))
    {
        pendingTempLimitSave = false; 
        flashMemory.putInt("limit_hot", limitHot); 
        flashMemory.putInt("limit_b5", limitBat5v); 
        flashMemory.putInt("limit_b12", limitBat12v); 
    }

    if (bleCmdToggleRgb == true) 
    {
        bleCmdToggleRgb = false; 
        isLedOn         = !isLedOn; 

        if (isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            if (isLedOn == true) 
            {
                ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
                ws2812fx.setMode(ledModeIndex); 
                applyVoltageColor(); 
                ws2812fx.start(); 
            }
            else 
            {
                ws2812fx.setBrightness(0); 
                ws2812fx.stop(); 
                ws2812fx.clear(); 
                ws2812fx.show(); 
            }
        }

        if (flashMemory.getBool("status", true) != isLedOn) 
        {
            flashMemory.putBool("status", isLedOn); 
        }
    }

    if (bleCmdNextMode == true) 
    {
        bleCmdNextMode = false; 

        if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            ledModeIndex = (ledModeIndex + 1) % (MAX_LED_MODE + 1); 
            ws2812fx.setMode(ledModeIndex); 
            applyVoltageColor(); 

            if (flashMemory.getUChar("mode", 0) != ledModeIndex) 
            {
                flashMemory.putUChar("mode", ledModeIndex); 
            }
        }
    }

    if (bleCmdPrevMode == true) 
    {
        bleCmdPrevMode = false; 

        if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            if (ledModeIndex == 0) 
            {
                ledModeIndex = MAX_LED_MODE; 
            }
            else 
            {
                ledModeIndex = ledModeIndex - 1; 
            }

            ws2812fx.setMode(ledModeIndex); 
            applyVoltageColor(); 
            ws2812fx.start(); 

            if (flashMemory.getUChar("mode", 0) != ledModeIndex) 
            {
                flashMemory.putUChar("mode", ledModeIndex); 
            }
        }
    }

    if (bleTargetVoltage != -1)
    {
        pendingVoltageRequest = constrain(bleTargetVoltage, 0, 2);
        bleTargetVoltage = -1;
    }

    if (pendingVoltageRequest != -1 && !isAiModeActive && !isVoltageTransitioning)
    {
        const unsigned long now = millis();
        const bool ready = !hasVoltageExecution || (now - lastVoltageExecutionTime >= 1000UL);
        if (ready && pendingVoltageRequest != currentVoltage)
        {
            targetTransitionVoltage = pendingVoltageRequest;
            isVoltageTransitioning = true;
            voltageTransitionStartTime = now;
            digitalWrite(PIN_OPTO_9V, LOW);
            digitalWrite(PIN_OPTO_12V, LOW);
        }
        else if (pendingVoltageRequest == currentVoltage)
        {
            pendingVoltageRequest = -1;
        }
    }

    if (bleNeedsReadvertise == true && (millis() - bleReadvertiseTimer >= 500)) 
    {
        bleNeedsReadvertise = false; 
        if (isBleActive == true && isBleInitialized == true && isWifiActive == false) 
        {
            BLEDevice::getAdvertising()->start(); 
        }
    }

    bool leftState = digitalRead(PIN_BTN_LEFT); 
    bool rightState = digitalRead(PIN_BTN_RIGHT); 

    if ((isWifiActive == true && WiFi.softAPgetStationNum() > 0) || isEmergencyMode == true) 
    {
        leftState = LOW; 
        rightState = LOW; 
        btnLeft.currentState = LOW; 
        btnRight.currentState = LOW; 
        btnLeft.tapCount = 0; 
        btnRight.tapCount = 0; 
    }

    if (leftState == HIGH && btnLeft.currentState == LOW) 
    {
        btnLeft.pressStartTime = millis(); 
        btnLeft.currentState   = HIGH; 
        btnLeft.isLongPressed  = false; 
    }

    if (leftState              == HIGH &&
        btnLeft.currentState   == HIGH &&
        btnLeft.isLongPressed  == false &&
        millis() - btnLeft.pressStartTime >= 5000)
    {
        btnLeft.isLongPressed = true; 
        if (isWifiActive == false) 
        {
            enableSilentWifi(30000); 
            startRadioBlink(0xFFFF00); 
        }
        else 
        {
            stopWifi(); 
            startRadioBlink(0xFF00FF); 
        }
    }

    if (leftState == LOW && btnLeft.currentState == HIGH) 
    {
        if (btnLeft.isLongPressed == false && millis() - btnLeft.pressStartTime <= 500) 
        {
            btnLeft.tapCount++; 
            btnLeft.lastTapTime = millis(); 
        }
        btnLeft.currentState = LOW; 
    }

    if (btnLeft.tapCount > 0 && millis() - btnLeft.lastTapTime > 500) 
    {
        if (btnLeft.tapCount == 1) 
        {
            if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
            {
                ledModeIndex = (ledModeIndex + 1) % (MAX_LED_MODE + 1); 
                ws2812fx.setMode(ledModeIndex); 
                applyVoltageColor(); 

                if (flashMemory.getUChar("mode", 0) != ledModeIndex) 
                {
                    flashMemory.putUChar("mode", ledModeIndex); 
                }
                }
        }
        else 
        {
            isLedOn = !isLedOn; 
            if (isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
            {
                if (isLedOn == true) 
                {
                    ws2812fx.setBrightness(getHwBrightness(currentBrightness)); 
                    ws2812fx.setMode(ledModeIndex); 
                    applyVoltageColor(); 
                    ws2812fx.start(); 
                }
                else 
                {
                    ws2812fx.setBrightness(0); 
                    ws2812fx.stop(); 
                    ws2812fx.clear(); 
                    ws2812fx.show(); 
                }
            }
            if (flashMemory.getBool("status", true) != isLedOn) 
            {
                flashMemory.putBool("status", isLedOn); 
            }
            }
        btnLeft.tapCount = 0; 
    }

    if (rightState == HIGH && btnRight.currentState == LOW) 
    {
        btnRight.pressStartTime = millis(); 
        btnRight.currentState   = HIGH; 
        btnRight.isLongPressed  = false; 
    }

    if (rightState              == HIGH &&
        btnRight.currentState   == HIGH &&
        btnRight.isLongPressed  == false &&
        millis() - btnRight.pressStartTime >= 5000)
    {
        btnRight.isLongPressed = true; 
        if (isBlePermitted == false) 
        {
            isBlePermitted = true; 
            if (isWifiActive == false) 
            {
                enableBleSignal(); 
            }
            startRadioBlink(0x00FFFF); 
        }
        else 
        {
            isBlePermitted = false;     
            suspendBle(); 
            startRadioBlink(0xFF00FF); 
        }
    }

    if (rightState == LOW && btnRight.currentState == HIGH) 
    {
        if (btnRight.isLongPressed == false && millis() - btnRight.pressStartTime <= 500) 
        {
            btnRight.tapCount++; 
            btnRight.lastTapTime = millis(); 
        }
        btnRight.currentState = LOW; 
    }

    if (btnRight.tapCount > 0 && millis() - btnRight.lastTapTime > 500) 
    {
        if (isAiModeActive == false && isVoltageTransitioning == false) 
        {
            int calcTarget = currentVoltage; 
            if (btnRight.tapCount == 1) 
            {
                calcTarget = currentVoltage + 1; 
                if (calcTarget > 2) 
                {
                    calcTarget = 2; 
                }
            }
            else 
            {
                calcTarget = currentVoltage - 1; 
                if (calcTarget < 0) 
                {
                    calcTarget = 0; 
                }
            }

            if (calcTarget != currentVoltage)
            {
                pendingVoltageRequest = calcTarget;
            }
        }
        btnRight.tapCount = 0; 
    }
}
