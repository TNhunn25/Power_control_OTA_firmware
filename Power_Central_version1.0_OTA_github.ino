
#include "ethernet_handler.h"
#include "mqtt_handler.h"
#include "led_status.h"
#include "config.h"
#include "config_portal.h"
#include "wifi_handler.h"
#include "lcd_i2c.h"
#include <esp_system.h>
#include "OTAUpdater.h"
#include <Ethernet.h>
#include <SSLClient.h>
#include "github_trust_anchors.h"

#include "Hardware_esp/pcf8575_io.h"
#include "Hardware_esp/central_controller.h"

constexpr int kI2cSdaPin = PCF8575IO::SDA_PIN;
constexpr int kI2cSclPin = PCF8575IO::SCL_PIN;
constexpr int kLcdSdaPin = kI2cSdaPin;
constexpr int kLcdSclPin = kI2cSclPin;

constexpr int kEncoderClkPin = P10;
constexpr int kEncoderDtPin = P9;
constexpr int kEncoderSwPin = P8;
constexpr bool kEncoderReverse = true; // Flip rotation so right turn advances output pages from CH1-4 to CH5-8

// ORIG: LCD used its own I2C_LCD object and PCF8575 used global Wire.
// Both devices now share the same explicit bus and the same SDA/SCL pins.
TwoWire I2C_SHARED = TwoWire(0);

PCF8575IO pcf(I2C_SHARED);
ManualPcf8575IO manualPcf(I2C_SHARED);
EthernetUDPHandler ethHandler;
MqttHandler mqttHandler;
LedStatus led_status;
CentralController centralController;
ConfigPortal configPortal;
LcdI2C lcd(0x27, 16, 2, I2C_SHARED);

OTAUpdater githubOta;
EthernetClient githubOtaClient;             // socket W5500 cho HTTP thuong (port 80)
EthernetClient githubOtaSecureBaseClient;   // socket W5500 nen cho TLS
constexpr int kSslEntropyPin = 4;           // chan analog lay entropy cho BearSSL RNG
SSLClient githubOtaSecureClient(githubOtaSecureBaseClient, TAs, (size_t)TAs_NUM, kSslEntropyPin); // boc TLS cho HTTPS GitHub

namespace
{
    const char *resetReasonName(esp_reset_reason_t reason)
    {
        switch (reason)
        {
        case ESP_RST_UNKNOWN:
            return "Unknown";
        case ESP_RST_POWERON:
            return "PowerOn"; // reset do ngu?n di?n cung c?p b? ng?t r?i c?p l?i
        case ESP_RST_EXT:
            return "ExtPin"; // reset do t?c d?ng v?o ch?n EN ho?c RST c?a board m?ch
        case ESP_RST_SW:
            return "Software"; // reset do ph?n m?m g?i h?m esp_restart() d? kh?i d?ng l?i
        case ESP_RST_PANIC:
            return "Panic"; // reset do ph?n m?m b? l?i nghi?m tr?ng, v? d? l?i tr?n b? nh?, l?i truy c?p b? nh?, l?i chia cho 0, v.v.
        case ESP_RST_INT_WDT:
            return "InterruptWDT"; // reset do watchdog timer b?t du?c ng?t b? treo (v? d? do l?i ph?n m?m l?m m?t ph?n h?i ng?t trong th?i gian d?i)
        case ESP_RST_TASK_WDT:
            return "TaskWDT"; // task b? treo qu? l?u, watchdog reset
        case ESP_RST_WDT:
            return "OtherWDT"; // reset do watchdog timer chung, c? th? do ph?n m?m ho?c ph?n c?ng
        case ESP_RST_DEEPSLEEP:
            return "DeepSleep"; // th?c d?y sau deep sleep
        case ESP_RST_BROWNOUT:
            return "Brownout"; // s?t ?p ngu?n
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "Other";
        }
    }

    void logResetReason()
    {
        const esp_reset_reason_t reason = esp_reset_reason();
        Serial.print(F("[SYS] Reset reason="));
        Serial.print(static_cast<int>(reason));
        Serial.print(F(" ("));
        Serial.print(resetReasonName(reason));
        Serial.println(F(")"));
    }

    bool previousMqttConnected = false;
    bool lastPublishedOutputs[OUT_COUNT] = {};
    bool hasPublishedSnapshot = false;
    bool lastPublishedAutoMode = false;
    uint8_t lastPublishedPowerSource = 0;
    bool pendingStatusPublish = false;
    bool pendingStatusForce = false;
    bool lcdEnabled = false;
    bool encoderEnabled = false;
    uint8_t encoder_lastClkState = HIGH;
    uint8_t encoder_lastButtonReading = HIGH;
    uint8_t encoder_buttonState = HIGH;
    bool encoder_buttonClicked = false;
    uint32_t encoder_lastDebounceMs = 0;
    uint16_t encoder_debounceMs = 100;
    uint32_t encoder_lastRotationMs = 0;
    const uint16_t kEncoderIgnoreButtonAfterRotateMs = 180;
    const uint16_t kEncoderRotationDebounceMs = 120;
    int8_t encoder_stepAccumulator = 0;
    // encoder long-press to open config portal
    bool encoder_buttonWasDown = false;
    bool encoder_pressHandled = false;
    uint32_t encoder_pressStartMs = 0;
    const uint32_t kEncoderLongPressMs = 3000; // ms to consider a long-press

    // bool lcdInMenuPage = false;
    // bool lcdInNetworkPage = true;
    // bool lcdShowNetworkIp = false;
    // bool lcdInIpPage = false;
    // bool lcdPreviousPageWasNetwork = false;
    // bool lcdInConfigMode = false;
    bool lcdSavedInNetworkPage = false;
    bool lcdSavedInIpPage = false;
    bool lcdSavedShowNetworkIp = false;
    bool lcdSavedInMenuPage = false;

    bool lcdInNetworkPage = true;
    bool lcdInMenuPage = false;

    bool lcdInSettingPage = false;
    uint8_t lcdSettingPage = 0;

    bool lcdInMenuDetailPage = false;
    uint8_t lcdMenuDetailPage = 0;
    uint8_t lcdIpInfoPage = 0;
    bool lcdIpEditMode = false;
    uint8_t lcdIpEditOctet = 0;
    EthStaticConfig lcdIpEditConfig;
    bool lcdIpEditDirty = false;

    bool lcdShowNetworkIp = false;
    bool lcdInIpPage = false;
    bool lcdPreviousPageWasNetwork = false;
    bool lcdInConfigMode = false;

    const uint8_t LCD_MENU_IP_NET = 0;
    const uint8_t LCD_MENU_CONFIG_NETWORK = 1;
    const uint8_t LCD_MENU_UUID_DEVICE = 2;
    const uint8_t LCD_MENU_VERSION = 3;
    const uint8_t LCD_MENU_BACK_SETTING = 4;
    const uint8_t LCD_MENU_COUNT = 5;

    const uint8_t LCD_DETAIL_NONE = 0;
    const uint8_t LCD_DETAIL_IP_NET = 1;
    const uint8_t LCD_DETAIL_UUID_DEVICE = 2;

    const uint8_t LCD_IP_PAGE_IP = 0;
    const uint8_t LCD_IP_PAGE_SUBNET = 1;
    const uint8_t LCD_IP_PAGE_GATEWAY = 2;
    const uint8_t LCD_IP_PAGE_BACK_MENU = 3;
    const uint8_t LCD_IP_INFO_PAGE_COUNT = 4;
    const uint8_t LCD_IP_EDIT_PAGE_COUNT = 3;

    uint8_t lcdSavedOutputPage = 0;
    uint32_t lcdIpPageStartMs = 0;
    const uint32_t kLcdIpTimeoutMs = 30000;
    uint32_t lcdSettingLastActivityMs = 0;
    const uint32_t kLcdSettingTimeoutMs = 30000;
    uint8_t currentOutputCount = 4;
    uint8_t lcdOutputPage = 0;
    const uint8_t kLcdOutputsPerPage = 4;
    uint32_t lastLcdRefreshMs = 0;
    uint32_t lastLcdReinitMs = 0;
    const uint32_t kLcdReinitIntervalMs = 5UL * 60UL * 1000UL; // Khoi tao lai LCD moi 5 phut
    uint32_t lastDeviceConfigCheckMs = 0;
    uint32_t checkconnect_status = 0;
    const uint32_t kDeviceConfigCheckMs = 5000;
    const uint8_t kDebugInputT13Pin = T13;
    const uint32_t kDebugInputT13PollMs = 100;
    uint32_t lastDebugInputT13Ms = 0;
    uint8_t lastDebugInputT13Level = HIGH;
    bool hasDebugInputT13Level = false;
    const uint32_t kAutoRebootIntervalMs = 24UL * 60UL * 60UL * 1000UL; // Auto reset board sau 24h
    uint32_t autoRebootStartMs = 0;
    bool autoRebootPending = false;
    String lastLcdLine0;
    String lastLcdLine1;
    String serialDebugLine;

    bool isValidPcfPin(int pin)
    {
        return pin >= 0 && pin < 16;
    }

    bool readPcfSnapshot(uint16_t &pcfState)
    {
        return pcf.readAll(pcfState);
    }

    uint8_t readPcfPinLevel(uint16_t pcfState, int pin)
    {
        if (!isValidPcfPin(pin))
            return HIGH;
        return ((pcfState >> pin) & 0x01u) ? HIGH : LOW;
    }

    uint8_t readPcfPinLevel(int pin)
    {
        uint16_t pcfState = 0;
        if (!readPcfSnapshot(pcfState))
            return HIGH;
        return readPcfPinLevel(pcfState, pin);
    }

    bool readInputT13Level(uint8_t &level, uint16_t &pcfState)
    {
        if (!isValidPcfPin(kDebugInputT13Pin))
            return false;

        if (!readPcfSnapshot(pcfState))
            return false;

        level = readPcfPinLevel(pcfState, kDebugInputT13Pin);
        return true;
    }

    // Forward declare MQTT helper used before its definition
    void publishMqttStatusneucan(bool force = false);
    void requestMqttStatusPublish(bool force = false);

    uint8_t powerSourceFromInputT13Level(uint8_t level)
    {
        return level == HIGH ? 1 : 0;
    }

    void printInputT13Level(uint8_t level, uint16_t pcfState)
    {
        uint8_t prev = ui8_power_source;
        Serial.print(F("[INDBG] T13="));
        Serial.print(level == HIGH ? 1 : 0);
        Serial.print(F(" raw=0x"));
        Serial.println(pcfState, HEX);

        ui8_power_source = powerSourceFromInputT13Level(level);

        if (ui8_power_source == 0 || ui8_power_source != prev)
        {
            centralController.handlePowerSourceChanged(ui8_power_source);
        }

        if (ui8_power_source != prev)
        {
            mqttHandler.publishOnline(true);
            requestMqttStatusPublish(false); // publish status ngay khi co thay doi nguon dien, khong can doi trang thai auto/manual, de hien thi nhanh hon tren app
        }
        // mqttHandler.publishOnline(true);
    }

    String getPowerSourceText()
    {
        return ui8_power_source ? "AC" : "BAT";
    }

    void printInputT13Level() // x? l? d?c n?t c?a anh Danh- c?m xo?
    {
        uint8_t level = HIGH;
        uint16_t pcfState = 0;
        if (!readInputT13Level(level, pcfState))
        {
            Serial.println(F("[INDBG] T13 read failed"));
            return;
        }

        printInputT13Level(level, pcfState);
    }

    void updateInputT13Debug(uint32_t now)
    {
        if ((uint32_t)(now - lastDebugInputT13Ms) < kDebugInputT13PollMs)
            return;
        lastDebugInputT13Ms = now;

        uint8_t level = HIGH;
        uint16_t pcfState = 0;
        if (!readInputT13Level(level, pcfState))
            return;

        if (!hasDebugInputT13Level || level != lastDebugInputT13Level)
        {
            hasDebugInputT13Level = true;
            lastDebugInputT13Level = level;
            printInputT13Level(level, pcfState);
        }
    }

    void syncInputT13PowerSourceNow()
    {
        uint8_t level = HIGH;
        uint16_t pcfState = 0;
        if (!readInputT13Level(level, pcfState))
        {
            Serial.println(F("[INDBG] T13 initial read failed"));
            return;
        }

        hasDebugInputT13Level = true;
        lastDebugInputT13Level = level;
        lastDebugInputT13Ms = millis();
        printInputT13Level(level, pcfState);
    }

    void prepareEncoderPcfPins()
    {
        if (isValidPcfPin(kEncoderClkPin))
            pcf.prepareInputPin((uint8_t)kEncoderClkPin);
        if (isValidPcfPin(kEncoderDtPin))
            pcf.prepareInputPin((uint8_t)kEncoderDtPin);
        if (isValidPcfPin(kEncoderSwPin))
            pcf.prepareInputPin((uint8_t)kEncoderSwPin);
    }

    int8_t readEncoderStep(uint16_t pcfState, uint32_t now)
    {
        const uint8_t clkState = readPcfPinLevel(pcfState, kEncoderClkPin);
        if (clkState == encoder_lastClkState)
            return 0;

        encoder_lastClkState = clkState;
        if (clkState != LOW)
            return 0;

        if ((uint32_t)(now - encoder_lastRotationMs) < kEncoderRotationDebounceMs)
            return 0;

        const uint8_t dtState = readPcfPinLevel(pcfState, kEncoderDtPin);
        int8_t step = (dtState == clkState) ? 1 : -1;
        return kEncoderReverse ? -step : step;
    }

    // Forward declare helper used by encoder handling
    void forceLcdSerialRefresh();

    void resetLcdToNetworkPage()
    {
        lcdInNetworkPage = true;
        lcdInMenuPage = false;
        lcdInSettingPage = false;
        lcdInMenuDetailPage = false;
        lcdMenuDetailPage = LCD_DETAIL_NONE;

        lcdSettingPage = LCD_MENU_IP_NET;
        lcdIpInfoPage = 0;

        lcdShowNetworkIp = false;
        lcdInIpPage = false;
        lcdIpPageStartMs = 0;
        lcdSettingLastActivityMs = 0;
    }

    void enterLcdMenuPage();
    void exitLcdMenuPage();
    void backToLcdMenuPage();
    void beginIpEdit();
    bool saveIpEditConfig();

    void updateEncoderButton(uint16_t pcfState, uint32_t now)
    {
        if (!isValidPcfPin(kEncoderSwPin))
            return;
        const uint8_t reading = readPcfPinLevel(pcfState, kEncoderSwPin);
        if (reading != encoder_lastButtonReading)
        {
            encoder_lastDebounceMs = now;
            encoder_lastButtonReading = reading;
            if (lcdInSettingPage)
                lcdSettingLastActivityMs = now;
        }

        if ((uint32_t)(now - encoder_lastDebounceMs) > encoder_debounceMs && reading != encoder_buttonState)
        {
            encoder_buttonState = reading;
            if (encoder_buttonState == LOW &&
                (uint32_t)(now - encoder_lastRotationMs) > kEncoderIgnoreButtonAfterRotateMs)
            {
                encoder_buttonClicked = true;
            }
        }

        // Long-press detection: open/close config portal when encoder button held
        if (reading == LOW)
        {
            if (!encoder_buttonWasDown)
            {
                encoder_buttonWasDown = true;
                encoder_pressHandled = false;
                encoder_pressStartMs = now;
            }
            // else if (!encoder_pressHandled && (uint32_t)(now - encoder_pressStartMs) >= kEncoderLongPressMs)
            // {
            //     encoder_pressHandled = true;
            //     encoder_buttonClicked = false; // prevent short-click action
            //     Serial.println(F("[ENC] Long press detected - toggling config portal"));
            //     configPortal.togglePortal();
            //     forceLcdSerialRefresh();
            // }
            else if (!encoder_pressHandled && (uint32_t)(now - encoder_pressStartMs) >= kEncoderLongPressMs)
            {
                encoder_pressHandled = true;
                encoder_buttonClicked = false;

                if (lcdInMenuDetailPage &&
                    lcdMenuDetailPage == LCD_DETAIL_IP_NET &&
                    !lcdIpEditMode &&
                    lcdIpInfoPage < LCD_IP_EDIT_PAGE_COUNT)
                {
                    beginIpEdit();
                    Serial.println(F("[ENC] Long press -> enter IP edit mode"));
                    forceLcdSerialRefresh();
                    return;
                }

                if (lcdIpEditMode && lcdInMenuDetailPage && lcdMenuDetailPage == LCD_DETAIL_IP_NET)
                {
                    if (lcdIpInfoPage + 1 < LCD_IP_EDIT_PAGE_COUNT)
                    {
                        saveIpEditConfig();
                        lcdIpEditMode = false;
                        lcdIpEditOctet = 0;
                        lcdIpInfoPage++;
                        Serial.println(F("[ENC] Long press -> next IP page"));
                        forceLcdSerialRefresh();
                    }
                    else
                    {
                        const bool saved = saveIpEditConfig();
                        lcdIpEditMode = false;
                        lcdIpEditOctet = 0;
                        if (saved)
                            Serial.println(F("[ENC] Long press -> IP saved"));
                        else
                            Serial.println(F("[ENC] Long press -> IP save failed"));
                        backToLcdMenuPage();
                    }
                    return;
                }

                // O bat cu trang chinh nao, giu 3s de vao MENU
                if (!lcdInSettingPage)
                {
                    Serial.println(F("[ENC] Long press -> MENU"));
                    enterLcdMenuPage();
                }
                // Dang o MENU hoac trang con, giu 3s de thoat MENU
                else
                {
                    Serial.println(F("[ENC] Long press MENU -> exit"));
                    exitLcdMenuPage();
                }
            }
        }
        else
        {
            encoder_buttonWasDown = false;
            encoder_pressStartMs = 0;
            encoder_pressHandled = false;
        }
    }

    void enterLcdMenuPage()
    {
        lcdInSettingPage = true;
        lcdInMenuDetailPage = false;
        lcdMenuDetailPage = LCD_DETAIL_NONE;

        lcdSettingPage = LCD_MENU_IP_NET;
        lcdIpInfoPage = 0;
        lcdIpEditMode = false;
        lcdIpEditOctet = 0;
        lcdIpEditDirty = false;
        lcdSettingLastActivityMs = millis();

        lcdInNetworkPage = false;
        lcdInMenuPage = false;
        lcdInIpPage = false;
        lcdShowNetworkIp = false;
        lcdIpPageStartMs = 0;

        Serial.println(F("[LCDDBG] Enter MENU"));
        forceLcdSerialRefresh();
    }

    void exitLcdMenuPage()
    {
        if (!lcdInSettingPage)
            return;

        lcdInSettingPage = false;
        lcdInMenuDetailPage = false;
        lcdMenuDetailPage = LCD_DETAIL_NONE;

        lcdSettingPage = LCD_MENU_IP_NET;
        lcdIpInfoPage = 0;
        lcdIpEditMode = false;
        lcdIpEditOctet = 0;
        lcdIpEditDirty = false;
        lcdSettingLastActivityMs = 0;

        lcdInNetworkPage = true;
        lcdInMenuPage = false;
        lcdInIpPage = false;
        lcdShowNetworkIp = false;

        Serial.println(F("[LCDDBG] Exit MENU"));
        forceLcdSerialRefresh();
    }

    void backToLcdMenuPage()
    {
        if (!lcdInSettingPage)
            return;

        lcdInMenuDetailPage = false;
        lcdMenuDetailPage = LCD_DETAIL_NONE;
        lcdIpInfoPage = 0;
        lcdIpEditMode = false;
        lcdIpEditOctet = 0;
        lcdIpEditDirty = false;

        lcdSettingLastActivityMs = millis();

        Serial.println(F("[LCDDBG] Detail -> MENU"));
        forceLcdSerialRefresh();
    }

    String formatOutputState(bool outputState)
    {
        return outputState ? "ON" : "OFF";
    }

    String padRight(String text, uint8_t width);

    String formatLcdChannel(uint8_t channel, bool outputState)
    {
        String s = String("CH") + channel + ":" + formatOutputState(outputState);
        return padRight(s, 7); // fixed-width column so ON/OFF doesn't shift layout
    }

    String padRight(String text, uint8_t width)
    {
        while (text.length() < width)
            text += ' ';
        if (text.length() > width)
            text.remove(width);
        return text;
    }

    uint8_t getOutputPageCount()
    {
        return (currentOutputCount + kLcdOutputsPerPage - 1) / kLcdOutputsPerPage;
    }

    String getNetworkLabel()
    {
        switch (mqttHandler.getActiveNetwork())
        {
        case MqttHandler::ActiveNetwork::Ethernet:
            return "ETH";
        case MqttHandler::ActiveNetwork::WiFi:
            return "WIFI";
        default:
            return "NONE";
        }
    }

    IPAddress getActiveNetworkIp()
    {
        switch (mqttHandler.getActiveNetwork())
        {
        case MqttHandler::ActiveNetwork::Ethernet:
            return Ethernet.localIP();
        case MqttHandler::ActiveNetwork::WiFi:
            return WiFi.localIP();
        default:
            return IPAddress(0, 0, 0, 0);
        }
    }

    IPAddress getActiveSubnet()
    {
        switch (mqttHandler.getActiveNetwork())
        {
        case MqttHandler::ActiveNetwork::Ethernet:
            return Ethernet.subnetMask();

        case MqttHandler::ActiveNetwork::WiFi:
            return WiFi.subnetMask();

        default:
            return IPAddress(0, 0, 0, 0);
        }
    }

    IPAddress getActiveGateway()
    {
        switch (mqttHandler.getActiveNetwork())
        {
        case MqttHandler::ActiveNetwork::Ethernet:
            return Ethernet.gatewayIP();

        case MqttHandler::ActiveNetwork::WiFi:
            return WiFi.gatewayIP();

        default:
            return IPAddress(0, 0, 0, 0);
        }
    }

    IPAddress &getCurrentIpEditField()
    {
        switch (lcdIpInfoPage)
        {
        case LCD_IP_PAGE_IP:
            return lcdIpEditConfig.ip;
        case LCD_IP_PAGE_SUBNET:
            return lcdIpEditConfig.mask;
        default:
            return lcdIpEditConfig.gateway;
        }
    }

    String formatIpEditValue(const IPAddress &ip, uint8_t selectedOctet)
    {
        String result;
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (i > 0)
                result += '.';
            if (i == selectedOctet)
                result += '>';
            result += String(ip[i]);
        }
        return result;
    }

    void beginIpEdit()
    {
        lcdIpEditMode = true;
        lcdIpEditOctet = 0;
        lcdIpEditDirty = false;
        if (!loadEthStaticConfig(lcdIpEditConfig))
            lcdIpEditConfig = defaultEthStaticConfig();
        lcdSettingLastActivityMs = millis();
    }

    bool saveIpEditConfig()
    {
        if (!isValidEthStaticConfig(lcdIpEditConfig))
            return false;

        const bool saved = saveEthStaticConfig(lcdIpEditConfig);
        if (saved)
        {
            ethHandler.applyStaticConfig(lcdIpEditConfig);
            lcdIpEditDirty = false;
        }
        return saved;
    }

    String getConnectionStatus()
    {
        return mqttHandler.getActiveNetwork() == MqttHandler::ActiveNetwork::None ? "OFFLINE" : "ONLINE";
    }

    // String getModeText()
    // {
    //     return centralController.isAutoModeActive() ? "AUTO" : "MAN";
    // }

    // void writeLcdLines(const String &line0, const String &line1)
    // {
    //     if (!lcdEnabled)
    //         return;

    //     lcd.printLine(0, line0);
    //     lcd.printLine(1, line1);

    //     if (line0 != lastLcdLine0 || line1 != lastLcdLine1)
    //     {
    //         Serial.print(F("[LCD] "));
    //         Serial.print(line0);
    //         Serial.print(F(" | "));
    //         Serial.println(line1);
    //         lastLcdLine0 = line0;
    //         lastLcdLine1 = line1;
    //     }
    // }

    void writeLcdLines(const String &line0, const String &line1)
    {
        if (!lcdEnabled)
            return;

        // Neu noi dung khong doi thi khong ghi lai LCD
        // Tranh bi chop man hinh lien tuc
        if (line0 == lastLcdLine0 && line1 == lastLcdLine1)
            return;

        lcd.printLine(0, line0);
        lcd.printLine(1, line1);

        Serial.print(F("[LCD] "));
        Serial.print(line0);
        Serial.print(F(" | "));
        Serial.println(line1);

        lastLcdLine0 = line0;
        lastLcdLine1 = line1;
    }

    void renderLcdOutputs()
    {
        bool outputs[OUT_COUNT] = {};
        centralController.copyPhysicalOutputStates(outputs);

        const uint8_t pageCount = getOutputPageCount();
        if (lcdOutputPage >= pageCount)
            lcdOutputPage = 0;

        const uint8_t offset = lcdOutputPage * kLcdOutputsPerPage;
        const uint8_t endIndex = min<uint8_t>(currentOutputCount, offset + kLcdOutputsPerPage);

        String line0 = "";
        String line1 = "";

        // Arrange up to 4 channels on the 2-line LCD as: [CH1 CH2] on line0 and [CH3 CH4] on line1
        for (uint8_t i = 0; i < kLcdOutputsPerPage; ++i)
        {
            const uint8_t idx = offset + i;
            if (idx >= endIndex)
                break;

            String chText = formatLcdChannel(idx + 1, outputs[idx]);
            if (i < (kLcdOutputsPerPage / 2))
            {
                if (line0.length() > 0)
                    line0 += ' ';
                line0 += chText;
            }
            else
            {
                if (line1.length() > 0)
                    line1 += ' ';
                line1 += chText;
            }
        }

        // Ensure lines fit the LCD width
        line0 = padRight(line0, 16);
        line1 = padRight(line1, 16);

        writeLcdLines(line0, line1);
    }

    void renderLcdNetwork()
    {
        if (lcdInConfigMode)
        {
            writeLcdLines(F("CONFIG NETWORK"), F(""));
            return;
        }

        const String networkLabel = getNetworkLabel();
        const String modeText = centralController.isAutoModeActive() ? "AUTO" : "MAN";
        const String powerText = getPowerSourceText();
        if (lcdShowNetworkIp)
        {
            writeLcdLines(F("IP DEVICE:"), getActiveNetworkIp().toString());
        }
        else
        {
            writeLcdLines(networkLabel + " " + getConnectionStatus(), String("MODE:") + modeText + " PW:" + powerText);
        }
    }

    // String getConnectionStatus()
    // {
    //     return mqttHandler.getActiveNetwork() == MqttHandler::ActiveNetwork::None ? "OFFLINE" : "ONLINE";
    // }

    String getWifiName()
    {
        if (WiFi.status() == WL_CONNECTED)
            return WiFi.SSID();

        return "NOT CONNECTED";
    }

    String getMenuItemText()
    {
        switch (lcdSettingPage)
        {
        case LCD_MENU_IP_NET:
            return "> IP NET CONFIG";

        case LCD_MENU_CONFIG_NETWORK:
            return "> CONFIG NETWORK";

        case LCD_MENU_UUID_DEVICE:
            return "> UUID DEVICE";

        case LCD_MENU_VERSION:
            return "> VERSION INFO";

        case LCD_MENU_BACK_SETTING:
            return "> BACK SETTING";

        default:
            lcdSettingPage = LCD_MENU_IP_NET;
            return "> IP NET CONFIG";
        }
    }

    void renderLcdMenu()
    {
        writeLcdLines(F("MENU"), getMenuItemText());
    }

    void renderLcdIpNetDetail()
    {
        if (lcdIpEditMode)
        {
            const IPAddress &ip = getCurrentIpEditField();
            const char *label = (lcdIpInfoPage == LCD_IP_PAGE_IP) ? "EDIT IP:" : (lcdIpInfoPage == LCD_IP_PAGE_SUBNET) ? "EDIT SUBNET:"
                                                                                                                       : "EDIT GATE:";
            const String line0 = String(label) + " " + String(lcdIpInfoPage + 1) + "/" + String(LCD_IP_EDIT_PAGE_COUNT);
            const String line1 = formatIpEditValue(ip, lcdIpEditOctet);
            writeLcdLines(line0, line1);
            return;
        }

        switch (lcdIpInfoPage)
        {
        case LCD_IP_PAGE_IP:
            writeLcdLines(F("IP DEVICE:"), getActiveNetworkIp().toString());
            break;

        case LCD_IP_PAGE_SUBNET:
            writeLcdLines(F("SUBNET:"), getActiveSubnet().toString());
            break;

        case LCD_IP_PAGE_GATEWAY:
            writeLcdLines(F("GATEWAY:"), getActiveGateway().toString());
            break;

        case LCD_IP_PAGE_BACK_MENU:
            writeLcdLines(F("BACK TO MENU"), F("PRESS BUTTON"));
            break;

        default:
            lcdIpInfoPage = LCD_IP_PAGE_IP;
            writeLcdLines(F("IP DEVICE:"), getActiveNetworkIp().toString());
            break;
        }
    }

    void renderLcdUuidDetail()
    {
        char out[32];
        snprintf(out, sizeof(out), "PDM_%05lu", (unsigned long)(ESP.getEfuseMac() % 100000ULL));
        writeLcdLines(F("UUID DEVICE:"), out);
    }

    void showVersionMenu()
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Version:");
        lcd.setCursor(0, 1);
        lcd.print(FW_VERSION);
        delay(2000); // hi?n th? 2 gi?y

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Build:");
        lcd.setCursor(0, 1);
        lcd.print(FW_BUILD_DATE);
        delay(2000); // hi?n th? 2 gi?y
                     // Quay l?i menu hi?n t?i
    }

    void renderLcdSetting()
    {
        if (lcdInMenuDetailPage)
        {
            if (lcdMenuDetailPage == LCD_DETAIL_IP_NET)
                renderLcdIpNetDetail();
            else if (lcdMenuDetailPage == LCD_DETAIL_UUID_DEVICE)
                renderLcdUuidDetail();
            else
                renderLcdMenu();

            return;
        }

        renderLcdMenu();
    }

    void forceLcdSerialRefresh();

    void stepLcdOutputPage(int8_t step)
    {
        if (step == 0)
            return;

        const uint8_t pageCount = getOutputPageCount();
        if (pageCount == 0)
            return;

        if (lcdInIpPage)
        {
            lcdInIpPage = false;
            lcdInNetworkPage = lcdPreviousPageWasNetwork;
            lcdShowNetworkIp = false;
            lcdIpPageStartMs = 0;
        }

        // N?u dang ? trong SETTING th? xoay ch? d?i trang setting
        if (lcdInSettingPage)
        {
            lcdSettingLastActivityMs = millis();

            // Neu dang xem chi tiet IP NET CONFIG
            if (lcdInMenuDetailPage && lcdMenuDetailPage == LCD_DETAIL_IP_NET)
            {
                if (lcdIpEditMode)
                {
                    // Xoay encoder: tang/gi?m gi? tr? octet hi?n t?i (0-255 wraparound)
                    IPAddress &field = getCurrentIpEditField();
                    uint8_t value = field[lcdIpEditOctet];
                    if (step > 0)
                    {
                        value++;
                        if (value > 255)
                            value = 0;
                    }
                    else
                    {
                        if (value > 0)
                            value--;
                        else
                            value = 255;
                    }
                    field[lcdIpEditOctet] = value;
                    lcdIpEditDirty = true;
                    Serial.print(F("[LCDDBG] Octet "));
                    Serial.print(lcdIpEditOctet);
                    Serial.print(F(" -> "));
                    Serial.println(value);
                    forceLcdSerialRefresh();
                    return;
                }

                // Xoay se chuyen IP -> SUBNET -> GATEWAY -> BACK TO MENU khi khong o che do edit
                if (step > 0)
                {
                    if (lcdIpInfoPage + 1 < LCD_IP_INFO_PAGE_COUNT)
                        lcdIpInfoPage++;
                    else
                        lcdIpInfoPage = LCD_IP_PAGE_IP;
                }
                else
                {
                    if (lcdIpInfoPage > 0)
                        lcdIpInfoPage--;
                    else
                        lcdIpInfoPage = LCD_IP_INFO_PAGE_COUNT - 1;
                }

                forceLcdSerialRefresh();

                Serial.print(F("[LCDDBG] IP info page="));
                Serial.println(lcdIpInfoPage);
                return;
            }

            // Neu dang xem UUID thi xoay khong lam gi
            if (lcdInMenuDetailPage && lcdMenuDetailPage == LCD_DETAIL_UUID_DEVICE)
            {
                forceLcdSerialRefresh();
                return;
            }

            // Dang o MENU chinh: xoay doi lua chon dong 2
            // readEncoderStep() already applies kEncoderReverse.
            int8_t menuStep = step;
            if (menuStep > 0)
            {
                if (lcdSettingPage + 1 < LCD_MENU_COUNT)
                    lcdSettingPage++;
                else
                    lcdSettingPage = 0;
            }
            else if (menuStep < 0)
            {
                if (lcdSettingPage > 0)
                    lcdSettingPage--;
                else
                    lcdSettingPage = LCD_MENU_COUNT - 1;
            }

            forceLcdSerialRefresh();

            Serial.print(F("[LCDDBG] Menu item="));
            Serial.println(lcdSettingPage);
            return;
        }

        // Luong ngoai MENU:
        // NET -> OUT -> NET
        // MENU chi vao bang nhan giu encoder 3s
        if (step > 0)
        {
            if (lcdInNetworkPage)
            {
                lcdInNetworkPage = false;
                lcdInMenuPage = false;
                lcdOutputPage = 0;
            }
            else
            {
                if (lcdOutputPage + 1 < pageCount)
                {
                    lcdOutputPage++;
                }
                else
                {
                    lcdInNetworkPage = true;
                    lcdInMenuPage = false;
                }
            }
        }
        else
        {
            if (lcdInNetworkPage)
            {
                lcdInNetworkPage = false;
                lcdInMenuPage = false;
                lcdOutputPage = pageCount - 1;
            }
            else
            {
                if (lcdOutputPage > 0)
                {
                    lcdOutputPage--;
                }
                else
                {
                    lcdInNetworkPage = true;
                    lcdInMenuPage = false;
                }
            }
        }

        lcdShowNetworkIp = false;
        lcd.clear();
        forceLcdSerialRefresh();

        Serial.print(F("[LCDDBG] Page="));
        if (lcdInNetworkPage)
            Serial.println(F("NET"));
        else
            Serial.println(F("OUT"));
    }

    void toggleLcdButtonPage()
    {
        if (lcdInIpPage)
        {
            lcdInIpPage = false;
            lcdInNetworkPage = lcdPreviousPageWasNetwork;
            lcdShowNetworkIp = false;
            lcdIpPageStartMs = 0;
            forceLcdSerialRefresh();
            Serial.println(F("[LCDDBG] Page=PREV"));
        }
        else
        {
            lcdPreviousPageWasNetwork = lcdInNetworkPage;
            lcdInIpPage = true;
            lcdInNetworkPage = true;
            lcdShowNetworkIp = true;
            lcdIpPageStartMs = millis();
            forceLcdSerialRefresh();
            Serial.println(F("[LCDDBG] Page=IP"));
        }
    }

    void enterLcdConfigMode()
    {
        if (lcdInConfigMode)
            return;

        lcdInConfigMode = true;
        lcdSavedInNetworkPage = lcdInNetworkPage;
        lcdSavedInIpPage = lcdInIpPage;
        lcdSavedShowNetworkIp = lcdShowNetworkIp;
        lcdSavedOutputPage = lcdOutputPage;

        lcdInSettingPage = false;
        lcdSettingPage = LCD_MENU_IP_NET;
        lcdSettingLastActivityMs = 0;
        lcdInMenuPage = false;

        lcdInIpPage = false;
        lcdInNetworkPage = true;
        lcdShowNetworkIp = false;
        // lcd.clear();
        forceLcdSerialRefresh();
    }

    void exitLcdConfigMode()
    {
        // Neu dang khong o config mode thi khong lam gi ca
        // Tranh clear LCD lien tuc moi vong loop
        if (!lcdInConfigMode)
            return;

        lcdInConfigMode = false;

        lcdInNetworkPage = true;
        lcdInMenuPage = false;
        lcdInSettingPage = false;
        lcdSettingPage = LCD_MENU_IP_NET;
        lcdSettingLastActivityMs = 0;

        lcdInIpPage = false;
        lcdShowNetworkIp = false;

        forceLcdSerialRefresh();
    }

    void updateLcdPageFromEncoder()
    {
        if (!encoderEnabled || configPortal.isActive())
            return;

        uint16_t pcfState = 0;
        if (!readPcfSnapshot(pcfState))
            return;

        const uint32_t now = millis();
        const int8_t step = readEncoderStep(pcfState, now);
        if (step != 0)
        {
            encoder_lastRotationMs = now;
            encoder_buttonClicked = false;
            if (encoder_stepAccumulator == 0 || encoder_stepAccumulator == step)
                encoder_stepAccumulator += step;
            else
                encoder_stepAccumulator = step;
        }

        updateEncoderButton(pcfState, now);

        if (encoder_stepAccumulator != 0)
        {
            if (encoder_stepAccumulator >= 2)
            {
                stepLcdOutputPage(1);
                encoder_stepAccumulator = 0;
                return;
            }
            else if (encoder_stepAccumulator <= -2)
            {
                stepLcdOutputPage(-1);
                encoder_stepAccumulator = 0;
                return;
            }
        }

        // if (encoder_buttonClicked)
        // {
        //     if (lcdInSettingPage)
        //     {
        //         encoder_buttonClicked = false;
        //         return;
        //     }
        //     // encoder_buttonClicked = false;
        //     // toggleLcdButtonPage();
        //     // return;
        // }

        if (encoder_buttonClicked)
        {
            encoder_buttonClicked = false;

            if (lcdInSettingPage)
            {
                lcdSettingLastActivityMs = millis();

                // Neu dang o trang chi tiet IP NET CONFIG
                if (lcdInMenuDetailPage && lcdMenuDetailPage == LCD_DETAIL_IP_NET)
                {
                    if (lcdIpEditMode)
                    {
                        // Short button press: move to next octet
                        if (lcdIpEditOctet < 3)
                        {
                            lcdIpEditOctet++;
                            Serial.print(F("[LCDDBG] Move to octet "));
                            Serial.println(lcdIpEditOctet);
                        }
                        else
                        {
                            // Finish editing this IP field, move to next (Subnet/Gateway)
                            lcdIpEditOctet = 0;
                            Serial.println(F("[LCDDBG] Octet cycling complete, ready for next IP field"));
                        }
                        forceLcdSerialRefresh();
                    }

                    if (!lcdIpEditMode && lcdIpInfoPage == LCD_IP_PAGE_BACK_MENU)
                    {
                        Serial.println(F("[LCDDBG] IP detail -> MENU"));
                        backToLcdMenuPage();
                    }
                    return;
                }

                // Neu dang o trang con khac thi bam ngan quay lai MENU
                if (lcdInMenuDetailPage)
                {
                    backToLcdMenuPage();
                    return;
                }

                // Dang o MENU, bam ngan de chon chuc nang
                switch (lcdSettingPage)
                {
                case LCD_MENU_IP_NET:
                    lcdInMenuDetailPage = true;
                    lcdMenuDetailPage = LCD_DETAIL_IP_NET;
                    lcdIpInfoPage = 0;
                    lcdIpEditMode = false;
                    Serial.println(F("[LCDDBG] Open IP NET CONFIG"));
                    forceLcdSerialRefresh();
                    return;

                case LCD_MENU_CONFIG_NETWORK:
                    Serial.println(F("[LCDDBG] Open CONFIG NETWORK HTTP"));
                    configPortal.togglePortal();
                    forceLcdSerialRefresh();
                    return;

                case LCD_MENU_UUID_DEVICE:
                    lcdInMenuDetailPage = true;
                    lcdMenuDetailPage = LCD_DETAIL_UUID_DEVICE;
                    Serial.println(F("[LCDDBG] Open UUID DEVICE"));
                    forceLcdSerialRefresh();
                    return;

                case LCD_MENU_VERSION:
                    Serial.println(F("[LCDDBG] Show VERSION INFO"));
                    showVersionMenu();
                    return;

                case LCD_MENU_BACK_SETTING:
                    Serial.println(F("[LCDDBG] BACK SETTING"));
                    exitLcdMenuPage();
                    return;
                }

                return;
            }
        }
    }

    void reinitLcdIfDue()
    {
        if (!lcdEnabled)
            return;

        const uint32_t now = millis();
        if ((uint32_t)(now - lastLcdReinitMs) < kLcdReinitIntervalMs) // 5 minutes
            return;

        lastLcdReinitMs = now;
        lcd.begin(kLcdSdaPin, kLcdSclPin);
        lcd.backlight();
        forceLcdSerialRefresh();
        Serial.println(F("[LCDDBG] Khoi dong lai LCD sau 5 phut"));
    }
    void updateLcdDisplay(bool force = false)
    {
        if (!lcdEnabled)
            return;

        const uint32_t now = millis();
        if (!force && (uint32_t)(now - lastLcdRefreshMs) < 250UL)
            return;

        if (lcdInIpPage && lcdIpPageStartMs != 0 && (uint32_t)(now - lcdIpPageStartMs) >= kLcdIpTimeoutMs)
        {
            lcdInIpPage = false;
            lcdInNetworkPage = lcdPreviousPageWasNetwork;
            lcdShowNetworkIp = false;
            lcdIpPageStartMs = 0;
            lcd.clear();
        }

        if (lcdInSettingPage &&
            lcdSettingLastActivityMs != 0 &&
            (uint32_t)(now - lcdSettingLastActivityMs) >= kLcdSettingTimeoutMs)
        {
            if (lcdInMenuDetailPage)
            {
                Serial.println(F("[LCDDBG] Detail timeout -> MENU"));
                backToLcdMenuPage();
            }
            else
            {
                Serial.println(F("[LCDDBG] MENU timeout -> exit"));
                exitLcdMenuPage();
            }
        }

        lastLcdRefreshMs = now;
        // if (lcdInIpPage)
        //     renderLcdNetwork();
        // else if (lcdInNetworkPage)
        //     renderLcdNetwork();
        // else
        //     renderLcdOutputs();

        if (lcdInConfigMode)
            renderLcdNetwork();
        else if (lcdInSettingPage)
            renderLcdSetting();
        else if (lcdInIpPage)
            renderLcdNetwork();
        else if (lcdInNetworkPage)
            renderLcdNetwork();
        else
            renderLcdOutputs();
    }

    void printLcdSerialHelp()
    {
        Serial.println(F("[LCDDBG] Commands:"));
        Serial.println(F("[LCDDBG]   lcd next / n   : next page"));
        Serial.println(F("[LCDDBG]   lcd prev / p   : previous page"));
        Serial.println(F("[LCDDBG]   lcd btn / b    : toggle IP page and return"));
        Serial.println(F("[LCDDBG]   lcd net        : show network status page"));
        Serial.println(F("[LCDDBG]   lcd ip         : show network IP page"));
        Serial.println(F("[LCDDBG]   lcd out        : show current output page"));
        Serial.print(F("[LCDDBG]   lcd out1..out"));
        Serial.print(getOutputPageCount());
        Serial.println(F(" : show output page"));
        Serial.println(F("[LCDDBG]   lcd refresh    : force refresh"));
        Serial.println(F("[LCDDBG]   enc raw       : print encoder PCF pin levels"));
        Serial.println(F("[MANDBG]   pcf2 / man raw: print manual PCF2 button levels"));
        Serial.println(F("[INDBG]    in13 / t13    : print input T13 level"));
        Serial.println(F("[LCDDBG] Alias: enc cw / enc ccw / enc btn"));
        Serial.println(F("[LCDDBG]   hold 3s on MENU : open setting"));
    }

    void printManualPcf2Raw()
    {
        uint16_t pcf2Raw = 0;
        uint16_t pcf2Buttons = 0;
        const bool rawOk = manualPcf.readAll(pcf2Raw);
        const bool buttonsOk = manualPcf.readButtonsSnapshot(pcf2Buttons);

        if (!rawOk && !buttonsOk)
        {
            Serial.println(F("[MANDBG] PCF2 0x21 read failed"));
            return;
        }

        Serial.print(F("[MANDBG] PCF2 raw="));
        if (rawOk)
        {
            Serial.print(F("0x"));
            Serial.print(pcf2Raw, HEX);
        }
        else
        {
            Serial.print(F("FAIL"));
        }

        Serial.print(F(" buttons="));
        if (buttonsOk)
        {
            Serial.print(F("0x"));
            Serial.print(pcf2Buttons, HEX);
        }
        else
        {
            Serial.print(F("FAIL"));
        }

        for (uint8_t i = 0; i < ManualPcf8575IO::BUTTON_COUNT; i++)
        {
            const uint8_t pin = manualPcf.BUTTON_PINS[i];
            const bool rawLevel = rawOk ? (((pcf2Raw >> pin) & 0x01u) != 0) : ManualPcf8575IO::releasedLevel();
            const bool buttonLevel = buttonsOk ? (((pcf2Buttons >> pin) & 0x01u) != 0) : rawLevel;
            Serial.print(F(" BTN"));
            Serial.print(i + 1);
            Serial.print(F("(P"));
            Serial.print(pin);
            Serial.print(F(")=raw:"));
            Serial.print(rawLevel ? F("HIGH") : F("LOW"));
            Serial.print(F("/btn:"));
            Serial.print(buttonLevel ? F("HIGH") : F("LOW"));
            Serial.print(F("/"));
            Serial.print(ManualPcf8575IO::isPressedLevel(buttonLevel) ? F("PRESSED") : F("FREE"));
        }
        Serial.println();
    }

    void forceLcdSerialRefresh() // Vi?t l?i LCD ngay l?p t?c, b? qua co ch? h?n ch? t?n s? refresh
    {
        if (!lcdEnabled)
        {
            Serial.println(F("[LCDDBG] LCD disabled"));
            return;
        }

        lcd.clear();
        lastLcdLine0 = "";
        lastLcdLine1 = "";
        updateLcdDisplay(true);
    }

    void selectLcdOutputPage(uint8_t page) // page is 0-based index
    {
        lcdInNetworkPage = false;
        lcdInIpPage = false;
        lcdShowNetworkIp = false;
        lcdInMenuPage = false;
        const uint8_t pageCount = getOutputPageCount();
        lcdOutputPage = page % pageCount;
        forceLcdSerialRefresh();
        Serial.print(F("[LCDDBG] OUT page="));
        Serial.println(lcdOutputPage + 1);
    }

    void handleLcdSerialCommand(String cmd) // cmd is already trimmed and lowercased by caller
    {
        cmd.trim();
        cmd.toLowerCase();
        if (cmd.length() == 0)
            return;

        if (cmd == "help" || cmd == "lcd help" || cmd == "?")
        {
            printLcdSerialHelp();
            return;
        }

        if (cmd == "pcf2" || cmd == "manual raw" || cmd == "man raw")
        {
            printManualPcf2Raw();
            return;
        }

        if (cmd.startsWith("lcd "))
        {
            cmd.remove(0, 4);
            cmd.trim();
        }
        else if (cmd.startsWith("enc "))
        {
            cmd.remove(0, 4);
            cmd.trim();
        }

        if (cmd == "n" || cmd == "next" || cmd == "cw")
        {
            stepLcdOutputPage(1);
        }
        else if (cmd == "p" || cmd == "prev" || cmd == "ccw")
        {
            stepLcdOutputPage(-1);
        }
        else if (cmd == "b" || cmd == "btn" || cmd == "button")
        {
            toggleLcdButtonPage();
        }
        else if (cmd == "net" || cmd == "network")
        {
            lcdInNetworkPage = true;
            lcdInMenuPage = false;
            lcdInSettingPage = false;
            lcdSettingPage = LCD_MENU_IP_NET;
            lcdSettingLastActivityMs = 0;
            lcdShowNetworkIp = false;
            lcdInIpPage = false;
            lcdIpPageStartMs = 0;
            forceLcdSerialRefresh(); // Lu?n reset timeout khi v?o trang network
            Serial.println(F("[LCDDBG] Page=NET"));
        }
        else if (cmd == "ip")
        {
            lcdPreviousPageWasNetwork = lcdInNetworkPage;
            lcdInIpPage = true;
            lcdInNetworkPage = true;
            lcdShowNetworkIp = true;
            lcdIpPageStartMs = millis();
            forceLcdSerialRefresh();
            Serial.println(F("[LCDDBG] Page=IP"));
        }
        else if (cmd == "out" || cmd == "outputs")
        {
            lcdInNetworkPage = false;
            lcdInMenuPage = false;
            lcdInSettingPage = false;
            lcdSettingPage = LCD_MENU_IP_NET;
            lcdSettingLastActivityMs = 0;
            lcdShowNetworkIp = false;
            lcdInIpPage = false;
            lcdIpPageStartMs = 0;
            forceLcdSerialRefresh();
            Serial.print(F("[LCDDBG] Page=OUT"));
            Serial.println(lcdOutputPage + 1);
        }
        else if (cmd == "refresh" || cmd == "r")
        {
            forceLcdSerialRefresh();
            Serial.println(F("[LCDDBG] Refreshed"));
        }
        else if (cmd == "raw" || cmd == "enc raw")
        {
            uint16_t pcfState = 0;
            if (!readPcfSnapshot(pcfState))
            {
                Serial.println(F("[ENCDBG] PCF read failed"));
                return;
            }

            Serial.print(F("[ENCDBG] CLK="));
            Serial.print(readPcfPinLevel(pcfState, kEncoderClkPin));
            Serial.print(F(" DT="));
            Serial.print(readPcfPinLevel(pcfState, kEncoderDtPin));
            Serial.print(F(" SW="));
            Serial.print(readPcfPinLevel(pcfState, kEncoderSwPin));
            Serial.print(F(" raw=0x"));
            Serial.println(pcfState, HEX);
        }
        else if (cmd == "in13" || cmd == "input13" || cmd == "t13")
        {
            printInputT13Level();
        }
        else if (cmd.startsWith("out"))
        {
            const int page = cmd.substring(3).toInt();
            const uint8_t pageCount = getOutputPageCount();
            if (page >= 1 && page <= pageCount)
                selectLcdOutputPage((uint8_t)(page - 1));
            else
                Serial.print(F("[LCDDBG] Use lcd out1..out")), Serial.println(pageCount);
        }

        else if (cmd == "menu")
        {
            lcdInNetworkPage = false;
            lcdInMenuPage = true;
            lcdShowNetworkIp = false;
            lcdInIpPage = false;
            lcdIpPageStartMs = 0;
            forceLcdSerialRefresh();
            Serial.println(F("[LCDDBG] Page=MENU"));
        }

        else
        {
            Serial.println(F("[LCDDBG] Unknown command. Type: lcd help"));
        }
    }

    void handleSerialDebug() // ??c l?nh t? Serial d? di?u khi?n LCD ho?c in th?ng tin debug, gi?p test v? debug d? d?ng hon m? kh?ng c?n ph?i thao t?c tr?c ti?p tr?n thi?t b? ho?c qua MQTT
    {
        while (Serial.available() > 0)
        {
            const char c = (char)Serial.read();
            if (c == '\r' || c == '\n')
            {
                if (serialDebugLine.length() > 0)
                {
                    handleLcdSerialCommand(serialDebugLine);
                    serialDebugLine = "";
                }
                continue;
            }

            if (c >= 32 && c <= 126 && serialDebugLine.length() < 64)
                serialDebugLine += c;
        }
    }
    void check_connection_status() 
    {
        if ((uint32_t)(millis() - checkconnect_status) >= 30000)
        {
            checkconnect_status = millis();
            mqttHandler.publishOnline(true);
            Serial.println(F("30s publish connection"));
        }
    }

    void publishMqttStatusneucan(bool force)
    {
        bool outputs[OUT_COUNT] = {};
        centralController.copyPhysicalOutputStates(outputs);
        const bool autoMode = centralController.isAutoModeActive();
        const bool connected = mqttHandler.isConnected();

        bool changed = force || !hasPublishedSnapshot || (autoMode != lastPublishedAutoMode) || (ui8_power_source != lastPublishedPowerSource);
        for (uint8_t i = 0; i < OUT_COUNT; i++)
        {
            if (outputs[i] != lastPublishedOutputs[i])
                changed = true;
        }

        if (connected && changed && mqttHandler.publishStatus(autoMode, outputs, ui8_power_source, currentOutputCount))
        {
            for (uint8_t i = 0; i < OUT_COUNT; i++)
                lastPublishedOutputs[i] = outputs[i];
            lastPublishedAutoMode = autoMode;
            lastPublishedPowerSource = ui8_power_source;
            hasPublishedSnapshot = true;
        }

        previousMqttConnected = connected;
    }

    void requestMqttStatusPublish(bool force) // ??nh d?u r?ng c?n ph?i xu?t tr?ng th?i l?n MQTT, v? n?u c? nhi?u l?n y?u c?u li?n ti?p th? ch? c?n th?c hi?n xu?t m?t l?n duy nh?t trong v?ng loop() ti?p theo
    {
        pendingStatusPublish = true;
        pendingStatusForce = pendingStatusForce || force;
    }

    void flushPendingMqttStatus()
    {
        if (!pendingStatusPublish)
            return;

        const bool force = pendingStatusForce;
        pendingStatusPublish = false;
        pendingStatusForce = false;
        publishMqttStatusneucan(force);
    }

    bool isAutoRebootBlocked()
    {
        // AUTO REBOOT 24H:
        // Project hien tai chua co ham accessor de check emergency,
        // nen tam thoi chi hoan reboot khi relay dang chuyen trang thai.
        return centralController.isRelayTransitionActive();
    }

    void taskAutoReboot(uint32_t now)
    {
        // AUTO REBOOT 24H:
        // Du 24 gio thi danh dau cho reboot.
        // Luc nao relay da idle moi reboot de tranh ngat giua qua trinh stagger.
        if (!autoRebootPending)
        {
            if ((uint32_t)(now - autoRebootStartMs) < kAutoRebootIntervalMs)
                return;

            autoRebootPending = true;
        }

        if (isAutoRebootBlocked())
            return;

        // Luu trang thai output hien tai truoc khi reboot board.
        centralController.persistOutputStateNow();
        Serial.println(F("[SYSTEM] uptime 24h reached, reboot board."));
        Serial.flush();
        ESP.restart();
    }
}

void onNewCommand(const IoCommand &cmd)
{
    if (cmd.opcode == DASHBOARD_INFO)
    {
        mqttHandler.publishOnline(true);
        return;
    }

    bool commandApplied = false;
    if (cmd.opcode == IO_COMMAND)
        commandApplied = centralController.handleCommand(cmd);

    if (commandApplied)
        ethHandler.sendCommand(cmd);

    requestMqttStatusPublish(commandApplied);
}

// Tra ve true neu an toan de OTA: khong co output/relay nao dang ON va relay khong dang chuyen trang thai.
bool isOtaSafeNow()
{
    if (centralController.isRelayTransitionActive())
        return false;

    bool outputs[OUT_COUNT] = {};
    centralController.copyPhysicalOutputStates(outputs);
    for (uint8_t i = 0; i < OUT_COUNT; i++)
    {
        if (outputs[i])
            return false; // con output dang ON, hoan OTA de khong ngat tai
    }
    return true;
}

void setup()
{
    Serial.begin(115200);
    autoRebootStartMs = millis();
    // Secure the PCF8575 relay outputs before any other peripheral init or boot delay.
    centralController.begin();
    syncInputT13PowerSourceNow();
    led_status.begin(33, true);

    delay(50);
    logResetReason();
    Serial.println(F("\n[SYS] Booting MQTT_PROTOCOL"));
    Serial.println(F("[SYS] setup -> ethHandler.begin"));
    ethHandler.begin();
    Serial.println(F("[SYS] setup -> peripherals"));

    DeviceConfig deviceCfg;
    loadDeviceConfig(deviceCfg);
    currentOutputCount = deviceCfg.outputCount >= 4 && deviceCfg.outputCount <= 8 ? deviceCfg.outputCount : 4;
    lcdOutputPage = 0;
    lcdInNetworkPage = true;
    lcdInMenuPage = false;
    lcdInSettingPage = false;
    lcdSettingPage = LCD_MENU_IP_NET;
    lcdSettingLastActivityMs = 0;
    lcdInIpPage = false;
    lcdShowNetworkIp = false;

    lcdEnabled = (kLcdSdaPin >= 0 && kLcdSclPin >= 0);
    encoderEnabled = (isValidPcfPin(kEncoderClkPin) && isValidPcfPin(kEncoderDtPin));
    if (lcdEnabled)
    {
        lcd.begin(kLcdSdaPin, kLcdSclPin);
        lcd.backlight();
        lastLcdReinitMs = millis();
        lcd.printLine(0, "Power Control");
        lcd.printLine(1, "Starting...");

        Serial.print("starting lcd");
        // show starting screen briefly before continuing initialization
        delay(2000);
    }
    if (encoderEnabled)
    {
        prepareEncoderPcfPins();
        uint16_t pcfState = 0;
        if (readPcfSnapshot(pcfState))
        {
            encoder_lastClkState = readPcfPinLevel(pcfState, kEncoderClkPin);
            if (isValidPcfPin(kEncoderSwPin))
            {
                encoder_lastButtonReading = readPcfPinLevel(pcfState, kEncoderSwPin);
                encoder_buttonState = encoder_lastButtonReading;
            }
        }
        Serial.print(F("[ENC] PCF8575 pins CLK="));
        Serial.print(kEncoderClkPin);
        Serial.print(F(" DT="));
        Serial.print(kEncoderDtPin);
        Serial.print(F(" SW="));
        Serial.println(kEncoderSwPin);
    }
    led_status.setState(LedStatus::STATE_NORMAL);
    mqttHandler.onCommandReceived(onNewCommand);
    mqttHandler.setAutoTransport();
    configPortal.begin(CONFIG_BUTTON_PIN, &ethHandler, &mqttHandler, &led_status, &centralController);
    setupWiFi();
    Serial.println(F("[SYS] setup -> mqttHandler.begin"));
    mqttHandler.begin();
    mqttHandler.update();
    updateLcdDisplay(true);
    printLcdSerialHelp();
    Serial.println(F("[SYS] setup done"));

    OTAUpdater::Config cfg;
    cfg.versionJsonUrl = OTA_GITHUB_VERSION_URL;
    cfg.firmwareBaseUrl = OTA_GITHUB_FIRMWARE_BASE_URL;
    cfg.firmwareFilePattern = OTA_GITHUB_FIRMWARE_PATTERN;

    cfg.versionKey = OTA_GITHUB_VERSION_KEY; // key version trong json
    cfg.idKey = OTA_GITHUB_ID_KEY;          // key ID trong json

    cfg.deviceName = OTA_GITHUB_DEVICE_NAME;   // ten thiet bi (hoac "ALL")
    cfg.currentVersion = FW_VERSION;    // version hien tai cua firmware

    cfg.intervalMs = OTA_GITHUB_CHECK_INTERVAL_MS; // chu ky check

    githubOta.setOtaSafeCallback(isOtaSafeNow);
    githubOta.begin(cfg, githubOtaClient, githubOtaSecureClient);
}

void loop()
{
    const uint32_t now = millis();

    // 1. Local hardware update: doc encoder, cap nhat trang thai LCD, xu ly input T13 debug command tu Serial
    handleSerialDebug();
    updateLcdPageFromEncoder();
    centralController.update();
    led_status.update();
    updateInputT13Debug(now);

    // 2. Network sau command qua Ethernet (W5500)
    ethHandler.update();
    // checkWiFiConnection(); 
    ethHandler.checkNodeTimeouts();

    // 3. Publish / UI / config
    if (centralController.consumeOutputChanged())
    {
        updateLcdDisplay(true);
    }
    

    //4. GitHub OTA update qua Ethernet (W5500), HTTPS truc tiep tu GitHub Raw
    // Chi OTA khi Ethernet (W5500) da link up. has_connect_link cap nhat tu ethHandler.update().
    if (has_connect_link)
    {
        githubOta.tick(OTAUpdater::NET_ETHERNET); // check moi 60s, tai version+firmware tu GitHub Raw HTTPS qua W5500
    }

    flushPendingMqttStatus();
    check_connection_status(); // check every 30s va publish len MQTT de bao cho broker biet thiet bi van online

    if ((uint32_t)(now - lastDeviceConfigCheckMs) >= kDeviceConfigCheckMs)
    {
        lastDeviceConfigCheckMs = now;
        DeviceConfig deviceCfg;
        if (loadDeviceConfig(deviceCfg) && deviceCfg.outputCount != currentOutputCount)
        {
            currentOutputCount = deviceCfg.outputCount;
            if (lcdOutputPage >= getOutputPageCount())
                lcdOutputPage = 0;
        }
    }

    const bool mqttConnected = mqttHandler.isConnected();
    has_mqtt_connection = mqttConnected;

    if (mqttConnected && !previousMqttConnected)
    {
        mqttHandler.publishOnline(true);
    }

    previousMqttConnected = mqttConnected;

    if (configPortal.isActive())
    {
        if (!lcdInConfigMode)
            enterLcdConfigMode();
    }
    else
    {
        if (lcdInConfigMode)
            exitLcdConfigMode();
    }

    reinitLcdIfDue();
    updateLcdDisplay();
    taskAutoReboot(now);
}
