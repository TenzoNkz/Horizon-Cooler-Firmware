/* =================================================================================
   =================================================================================
   [ UNDANG-UNDANG PENGEMBANGAN HORIZON COOLER (RULES & CODING DIRECTIVES) ]
   =================================================================================

   --- BAB I: KEDISIPLINAN SINTAKS & KOMPILASI MOBILE ---
   PASAL 1: Wajib melakukan simulasi mental alur eksekusi sebelum memodifikasi kode.
   PASAL 2: Kurung kurawal pembuka { dan penutup } WAJIB berada di baris tersendiri (Vertikal Murni).
   PASAL 3: Kode WAJIB murni ASCII, bebas ketergantungan asing, agar 100% lolos kompilasi via ArduinoDroid di Android.
   PASAL 4: DILARANG DITULIS komentar apapun di dalam tubuh fungsi C++ untuk menjaga kebersihan kompilasi.

   --- BAB II: MANAJEMEN MEMORI, BLE & NV-RAM ---
   PASAL 5: Dilarang menggunakan kelas objek 'String' pada BLE/NTC; wajib 'char array'.
   PASAL 6: Nilai NV-RAM (Mode, Brightness, Limit Suhu, Status RGB) hanya ditulis jika terjadi perubahan dan wajib ditunda minimal 2000 ms.
   PASAL 7: Status Volatil (Voltase & AI Mode) DILARANG disimpan di NV-RAM. Sistem wajib boot-up default di posisi teraman: 5V dan AI OFF.
   
   --- BAB III: KERNEL ANTI-BLOCKING & SINKRONISASI ---
   PASAL 8: DILARANG KERAS menggunakan fungsi delay() di seluruh baris kode. Semua jeda WAJIB menggunakan millis() dan yield() agar sistem RTOS/BLE tidak membeku.
   PASAL 9: Saat mendapat perintah "SYNC" dari App, ESP32 WAJIB menembakkan seluruh data dalam format berpagar <SYNC_START> ... <SYNC_END> agar UI terisi serempak tanpa lag.
   PASAL 10: Setelah sinkronisasi awal, pembaruan data HANYA dikirim jika ada perubahan nilai fisik (Delta-Sync).

   --- BAB IV: KELISTRIKAN & PROTEKSI HARDWARE ---
   PASAL 11: JEDA INDUKSI MUTLAK. Setiap transisi voltase WAJIB diputus ke 5V (LOW-LOW) selama 150 ms sebelum target aktif guna meredam inrush current MP2315.
   PASAL 12: Jika Suhu NTC mencapai Limit Hot (default 45C), AI Mode WAJIB memotong daya ke 5V. Pemulihan naik voltase hanya jika suhu turun 5C di bawah batas, dievaluasi per 5 detik.
   PASAL 13: Mode Manual (AI OFF) MENGUNCI voltase pilihan pengguna tanpa campur tangan sistem suhu otomatis.
   PASAL 14: Jika NTC putus/korslet (<50mV atau >3250mV), injeksikan nilai 999C agar sistem langsung memotong daya demi keselamatan.

   --- BAB V: SISTEM VISUAL & KEDIPAN KONDISI OTA ---
   PASAL 15: Seluruh kedipan feedback wajib menggunakan manipulasi buffer bit langsung (playBlink1000) tanpa memanggil setMode WS2812FX.
   PASAL 16: ATURAN KEDIPAN KONDISI OTA:
       - WiFi Terhubung Sukses : Kedip Biru 1000 ms (250 ms Nyala / 250 ms Mati x 2) -> Padam -> Eksekusi Update OTA.
       - WiFi Gagal Terhubung   : Kedip Merah 1000 ms (250 ms Nyala / 250 ms Mati x 2) -> Standby.
       - Update OTA Berhasil    : Kedip Hijau 1000 ms (250 ms Nyala / 250 ms Mati x 2) -> ESP.restart().
       - Update OTA Gagal       : Kedip Merah 1000 ms (250 ms Nyala / 250 ms Mati x 2) -> Standby.

   --- BAB VI: PROTOKOL VERSI FIRMWARE (1 DESIMAL) ---
   PASAL 17: Format versi HANYA menggunakan 1 angka di belakang titik (Contoh: V2.2). Update besar mengubah angka depan (Contoh: V2.0 -> V3.0).
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

#define FIRMWARE_VERSION "V2.2"

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
volatile bool bleCmdSendAiStatus   = false; 
volatile bool triggerCloudOta      = false; 
volatile bool isOtaStandby         = false; 

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

float         currentTemperature = 20.0f; 
unsigned long lastAdcReadTime    = 0;     
uint32_t      adcAccumulator     = 0;     
uint8_t       adcSampleCount     = 0;
unsigned long lastAiCheckTime    = 0;     

bool          triggerInstantSync = false; 

unsigned long lastBrightnessChangeTime  = 0;
bool          pendingBrightnessSave     = false;

int           limitHot   = 45;
int           limitBat5v = 25;
int           limitBat9v = 30;
int           limitBat12v= 35;
unsigned long lastTempLimitSaveTime = 0;
bool          pendingTempLimitSave = false;

volatile int  aiMaxVoltage = 2; 

float lastSentTemp = -999.0f;
int lastSentVoltage = -1;
bool lastSentRgb = false;
bool lastSentAi = false;
int lastSentBr = -1;
int lastSentMode = -1;
int lastSentHot = -1;
int lastSentBat5 = -1;
int lastSentBat9 = -1;
int lastSentBat12 = -1;
unsigned long lastTempSyncTime = 0;

void cleanBuffer(char* dest, const char* src, size_t maxLen);
uint8_t getHwBrightness(uint8_t appBrightness);
void sendInstantSyncData();
void sendModeToWeb();
void sendTempToWeb();
void sendBrightnessToWeb();
void sendRgbStatusToWeb();
void sendVoltageToWeb();
void sendAiStatusToWeb();
void sendVersionToWeb();
void sendTempLimitsToWeb();
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
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        float displayTemp = currentTemperature;
        if (displayTemp < 20.0f) 
        {
            displayTemp = 20.0f;
        }
        if (displayTemp > 50.0f) 
        {
            displayTemp = 50.0f;
        }

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
            "LHT:%d\n"
            "LB5:%d\n"
            "LB9:%d\n"
            "LB12:%d\n"
            "VER:%s\n"
            "<SYNC_END>\n",
            ledModeIndex,
            currentBrightness,
            isLedOn ? 1 : 0,
            displayTemp,
            voltStr,
            isAiModeActive ? 1 : 0,
            limitHot,
            limitBat5v,
            limitBat9v,
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
        lastSentHot = limitHot;
        lastSentBat5 = limitBat5v;
        lastSentBat9 = limitBat9v;
        lastSentBat12 = limitBat12v;
    }
}

void sendModeToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        sprintf(sendBuffer, "MD:%d\n", ledModeIndex); 
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendTempToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        float displayTemp = currentTemperature;
        if (displayTemp < 20.0f) 
        {
            displayTemp = 20.0f;
        }
        if (displayTemp > 50.0f) 
        {
            displayTemp = 50.0f;
        }
        char tempBuffer[16]; 
        sprintf(tempBuffer, "TMP:%.1f\n", displayTemp); 
        globalTxChar->setValue((uint8_t*)tempBuffer, strlen(tempBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendBrightnessToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        sprintf(sendBuffer, "BRV:%d\n", currentBrightness); 
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendRgbStatusToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        if (isLedOn == true) 
        {
            sprintf(sendBuffer, "RGB:1\n"); 
        }
        else 
        {
            sprintf(sendBuffer, "RGB:0\n"); 
        }
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendVoltageToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        if (currentVoltage == 0) 
        {
            sprintf(sendBuffer, "VOL:5V\n"); 
        }
        else if (currentVoltage == 1) 
        {
            sprintf(sendBuffer, "VOL:9V\n"); 
        }
        else 
        {
            sprintf(sendBuffer, "VOL:12V\n"); 
        }
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendAiStatusToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        if (isAiModeActive == true) 
        {
            sprintf(sendBuffer, "AI:1\n"); 
        }
        else 
        {
            sprintf(sendBuffer, "AI:0\n"); 
        }
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendVersionToWeb() 
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char sendBuffer[16]; 
        sprintf(sendBuffer, "VER:%s\n", FIRMWARE_VERSION); 
        globalTxChar->setValue((uint8_t*)sendBuffer, strlen(sendBuffer)); 
        globalTxChar->notify(); 
    }
}

void sendTempLimitsToWeb()
{
    if (isBleActive == true && isBleClientConnected == true && globalTxChar != NULL) 
    {
        char buf[64];
        sprintf(buf, "LHT:%d\nLB5:%d\nLB9:%d\nLB12:%d\n", limitHot, limitBat5v, limitBat9v, limitBat12v);
        globalTxChar->setValue((uint8_t*)buf, strlen(buf)); 
        globalTxChar->notify(); 
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
        lastSentBat9 = -1;
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

        char rawStr[512];
        size_t copyLen = dataLength < 511 ? dataLength : 511;
        memcpy(rawStr, rawData, copyLen);
        rawStr[copyLen] = '\0';

        char* token = strtok(rawStr, "\n");
        while (token != NULL)
        {
            if (strncmp(token, "SSID:", 5) == 0) 
            {
                cleanBuffer(otaSsid, token + 5, 64);
                flashMemory.putString("ota_ssid", otaSsid);
            } 
            else if (strncmp(token, "PASS:", 5) == 0) 
            {
                cleanBuffer(otaPass, token + 5, 64);
                flashMemory.putString("ota_pass", otaPass);
            } 
            else if (strncmp(token, "URL:", 4) == 0) 
            {
                cleanBuffer(otaUrl, token + 4, 250);
                flashMemory.putString("ota_url", otaUrl);
            }
            else
            {
                size_t len = strlen(token);
                if (len > 31) 
                {
                    len = 31;
                }
                char cmdBuf[32]; 
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

                if (strcmp(cmdBuf, "OTAENTER") == 0) 
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
                        bleCmdSendAiStatus = true; 
                    }
                }
                else if (strcmp(cmdBuf, "9V") == 0) 
                {
                    bleTargetVoltage = 1;    
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                        bleCmdSendAiStatus = true; 
                    }
                }
                else if (strcmp(cmdBuf, "12V") == 0) 
                {
                    bleTargetVoltage = 2;    
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                        bleCmdSendAiStatus = true; 
                    }
                }
                else if (strcmp(cmdBuf, "AION") == 0) 
                {
                    if (isAiModeActive == false) 
                    {
                        isAiModeActive = true; 
                        bleCmdSendAiStatus = true; 
                        lastAiCheckTime = millis(); 
                        aiMaxVoltage = 2; 

                        targetTransitionVoltage  = 0; 
                        isVoltageTransitioning   = true; 
                        voltageTransitionStartTime = millis(); 
                        digitalWrite(PIN_OPTO_9V,  LOW); 
                        digitalWrite(PIN_OPTO_12V, LOW);
                    }
                }
                else if (strcmp(cmdBuf, "AIOFF") == 0) 
                {
                    if (isAiModeActive == true) 
                    {
                        isAiModeActive = false; 
                        bleCmdSendAiStatus = true; 

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
                else if (strncmp(cmdBuf, "LHT:", 4) == 0)
                {
                    limitHot = atoi(cmdBuf + 4);
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strncmp(cmdBuf, "LB5:", 4) == 0)
                {
                    limitBat5v = atoi(cmdBuf + 4);
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strncmp(cmdBuf, "LB9:", 4) == 0)
                {
                    limitBat9v = atoi(cmdBuf + 4);
                    pendingTempLimitSave = true;
                    lastTempLimitSaveTime = millis();
                }
                else if (strncmp(cmdBuf, "LB12:", 5) == 0)
                {
                    limitBat12v = atoi(cmdBuf + 5);
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

void performCloudOTA() 
{
    digitalWrite(PIN_OPTO_9V,  LOW); 
    digitalWrite(PIN_OPTO_12V, LOW); 
    currentVoltage = 0; 
    isAiModeActive = false; 

    ws2812fx.stop(); 
    ws2812fx.clear(); 
    ws2812fx.show(); 

    if (isBleInitialized == true) 
    {
        BLEDevice::getAdvertising()->stop();
    }
    
    WiFi.disconnect(true);
    safeYield(300);
    
    WiFi.mode(WIFI_STA); 
    safeYield(300); 

    WiFi.begin(otaSsid, otaPass); 
    
    unsigned long startWait = millis(); 
    bool connected = false; 
    
    while (millis() - startWait < 15000) 
    { 
        if (WiFi.status() == WL_CONNECTED) 
        {
            connected = true; 
            break; 
        }
        safeYield(100); 
    }

    if (connected == true) 
    {
        // PASAL 16: WiFi Terhubung Sukses -> Kedip Biru 1000 ms
        playBlink1000(0x0000FF); 

        WiFiClientSecure client; 
        client.setInsecure(); 
        client.setTimeout(20000); 
        
        t_httpUpdate_return ret = httpUpdate.update(client, otaUrl); 
        
        if (ret == HTTP_UPDATE_OK) 
        {
            // PASAL 16: Update Berhasil -> Kedip Hijau 1000 ms -> Reboot
            playBlink1000(0x00FF00); 
            WiFi.disconnect(true); 
            WiFi.mode(WIFI_OFF); 
            safeYield(500); 
            ESP.restart(); 
        }
        else 
        {
            // PASAL 16: Update Gagal -> Kedip Merah 1000 ms
            playBlink1000(0xFF0000); 
        }
    }
    else 
    {
        // PASAL 16: WiFi Gagal Terhubung -> Kedip Merah 1000 ms
        playBlink1000(0xFF0000); 
    }

    WiFi.disconnect(true); 
    WiFi.mode(WIFI_OFF); 
    triggerCloudOta = false; 
    isOtaStandby = false; 
    
    safeYield(1000);
    ESP.restart();
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

    limitHot          = flashMemory.getInt("limit_hot", 45);
    limitBat5v        = flashMemory.getInt("limit_b5", 25);
    limitBat9v        = flashMemory.getInt("limit_b9", 30);
    limitBat12v       = flashMemory.getInt("limit_b12", 35);
    
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

    if (millis() - lastAdcReadTime >= 100) 
    {
        lastAdcReadTime = millis(); 
        adcAccumulator += ntcSensor.readRawMv(); 
        adcSampleCount++; 

        if (adcSampleCount >= 10) 
        {
            float avgMv = (float)adcAccumulator / 10.0f; 
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
        if (currentTemperature >= (float)limitHot && currentVoltage > 0) 
        {
            if (currentVoltage == 2) 
            {
                aiMaxVoltage = 1; 
            }
            else if (currentVoltage == 1) 
            {
                aiMaxVoltage = 0; 
            }

            targetTransitionVoltage      = 0; 
            isVoltageTransitioning       = true; 
            voltageTransitionStartTime   = millis(); 
            
            digitalWrite(PIN_OPTO_9V,  LOW); 
            digitalWrite(PIN_OPTO_12V, LOW); 
            
            lastAiCheckTime = millis(); 
        }
        else if (millis() - lastAiCheckTime >= 5000) 
        {
            lastAiCheckTime = millis(); 
            int calcTarget  = currentVoltage; 

            if (currentTemperature <= (float)(limitHot - 5)) 
            {
                if (currentVoltage < aiMaxVoltage) 
                {
                    calcTarget = currentVoltage + 1; 
                }
            }

            if (calcTarget != currentVoltage && isVoltageTransitioning == false) 
            {
                targetTransitionVoltage      = calcTarget; 
                isVoltageTransitioning       = true; 
                voltageTransitionStartTime   = millis(); 

                digitalWrite(PIN_OPTO_9V,  LOW); 
                digitalWrite(PIN_OPTO_12V, LOW); 
            }
        }
    }

    if (isVoltageTransitioning == true && (millis() - voltageTransitionStartTime >= 150)) 
    {
        isVoltageTransitioning = false; 
        currentVoltage         = targetTransitionVoltage; 

        digitalWrite(PIN_OPTO_9V,  (currentVoltage == 1) ? HIGH : LOW); 
        digitalWrite(PIN_OPTO_12V, (currentVoltage == 2) ? HIGH : LOW); 

        if (isRadioLedBlinking == false) 
        {
            if (isLedOn == true) 
            {
                isVoltageIndicatorActive = true;  
                voltageIndicatorStartTime = millis(); 
                ws2812fx.stop(); 
                uint8_t safeBrightnessLevel = (currentBrightness < 64) ? 64 : currentBrightness; 
                ws2812fx.setBrightness(getHwBrightness(safeBrightnessLevel)); 
                uint32_t vColor = 0xFF0000; 
                if (currentVoltage == 1)
                {
                    vColor = 0x00FF00; 
                }
                else if (currentVoltage == 2)
                {
                    vColor = 0x0000FF; 
                }
                for (int i = 0; i < TOTAL_LED; i++)
                {
                    ws2812fx.setPixelColor(i, vColor); 
                }
                ws2812fx.show(); 
            }
        }
    }

    if (isVoltageIndicatorActive == true && (millis() - voltageIndicatorStartTime >= 1000)) 
    {
        isVoltageIndicatorActive = false; 
        if (isRadioLedBlinking == false) 
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
    }

    if (isBleActive == true && isBleClientConnected == true)
    {
        if (millis() - lastTempSyncTime >= 2000)
        {
            lastTempSyncTime = millis(); 
            if (fabs(currentTemperature - lastSentTemp) >= 0.2f)
            {
                sendTempToWeb(); 
                lastSentTemp = currentTemperature; 
            }
        }
        if (currentVoltage != lastSentVoltage)
        {
            sendVoltageToWeb(); 
            lastSentVoltage = currentVoltage; 
        }
        if (isLedOn != lastSentRgb)
        {
            sendRgbStatusToWeb(); 
            lastSentRgb = isLedOn; 
        }
        if (isAiModeActive != lastSentAi)
        {
            sendAiStatusToWeb(); 
            lastSentAi = isAiModeActive; 
        }
        if (currentBrightness != lastSentBr)
        {
            sendBrightnessToWeb(); 
            lastSentBr = currentBrightness; 
        }
        if (ledModeIndex != lastSentMode)
        {
            sendModeToWeb(); 
            lastSentMode = ledModeIndex; 
        }
        if (limitHot != lastSentHot || limitBat5v != lastSentBat5 || limitBat9v != lastSentBat9 || limitBat12v != lastSentBat12)
        {
            sendTempLimitsToWeb(); 
            lastSentHot = limitHot; 
            lastSentBat5 = limitBat5v; 
            lastSentBat9 = limitBat9v; 
            lastSentBat12 = limitBat12v; 
        }
    }

    if (bleCmdSendAiStatus == true) 
    {
        bleCmdSendAiStatus = false; 
        sendAiStatusToWeb(); 
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
        flashMemory.putInt("limit_b9", limitBat9v); 
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
        sendRgbStatusToWeb(); 
    }

    if (bleCmdNextMode == true) 
    {
        bleCmdNextMode = false; 

        if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            ws2812fx.stop(); 
            ledModeIndex = (ledModeIndex + 1) % (MAX_LED_MODE + 1); 
            ws2812fx.setMode(ledModeIndex); 
            applyVoltageColor(); 
            ws2812fx.start(); 

            if (flashMemory.getUChar("mode", 0) != ledModeIndex) 
            {
                flashMemory.putUChar("mode", ledModeIndex); 
            }
            sendModeToWeb(); 
        }
    }

    if (bleCmdPrevMode == true) 
    {
        bleCmdPrevMode = false; 

        if (isLedOn == true && isRadioLedBlinking == false && isVoltageIndicatorActive == false) 
        {
            ws2812fx.stop(); 
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
            sendModeToWeb(); 
        }
    }

    if (bleTargetVoltage != -1) 
    {
        if (bleTargetVoltage != currentVoltage && isVoltageTransitioning == false) 
        {
            if (isAiModeActive == true) 
            {
                isAiModeActive = false; 
                bleCmdSendAiStatus = true; 
            }
            
            targetTransitionVoltage  = bleTargetVoltage; 
            isVoltageTransitioning   = true; 
            voltageTransitionStartTime = millis(); 

            digitalWrite(PIN_OPTO_9V,  LOW); 
            digitalWrite(PIN_OPTO_12V, LOW); 
        }
        bleTargetVoltage = -1; 
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
                ws2812fx.stop(); 
                ledModeIndex = (ledModeIndex + 1) % (MAX_LED_MODE + 1); 
                ws2812fx.setMode(ledModeIndex); 
                applyVoltageColor(); 
                ws2812fx.start(); 

                if (flashMemory.getUChar("mode", 0) != ledModeIndex) 
                {
                    flashMemory.putUChar("mode", ledModeIndex); 
                }
                sendModeToWeb(); 
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
            sendRgbStatusToWeb(); 
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
                targetTransitionVoltage  = calcTarget; 
                isVoltageTransitioning   = true; 
                voltageTransitionStartTime = millis(); 

                digitalWrite(PIN_OPTO_9V,  LOW); 
                digitalWrite(PIN_OPTO_12V, LOW); 
            }
        }
        btnRight.tapCount = 0; 
    }
}
