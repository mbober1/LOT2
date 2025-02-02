#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <stdlib.h>
#include <WiFi.h>
#include "../src/oled_stuff/SSD1306Wire.h"
#include <string>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#define BAND 868E6

TaskHandle_t LoRaTask;
static uint8_t mydata[] = "notdetected!";
static osjob_t sendjob;
static osjob_t wifijob;

bool loraisconnected = false;
bool isshitdetected = false;
int lorarssi = 0;

/*Wifi stuff*/
const char *ssid = "ESP32-Access-Point";
const char *password = "twojstarypijany";
wifi_sta_list_t wifi_sta_list;
tcpip_adapter_sta_list_t adapter_sta_list;



void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("!!!Handheld device detected!!!");
    sprintf((char *)mydata, "devicedetected");
    isshitdetected = true;
}

void WiFiStationDiconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("!!!Handheld device lost!!!");
    sprintf((char *)mydata, "oooojasnygrzyb");
    isshitdetected = false;
}

/*Wifi end*/

SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);

void do_send(osjob_t *j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t PROGMEM APPEUI[8] = {0x06, 0xCB, 0x3F, 0xC3, 0xB5, 0x7E, 0xAC, 0x2A};

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x63, 0x7A, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};

void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x73, 0xBD, 0x16, 0xBC, 0x83, 0x27, 0x48, 0xF9, 0x4F, 0x45, 0x32, 0x48, 0x6D, 0x9E, 0x20, 0x42};

void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 2;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}};

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        loraisconnected = true;
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     Serial.println(F("EV_RFU1"));
    ||     break;
    */
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            lorarssi = LMIC.rssi;
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    Serial.println(F("EV_SCAN_FOUND"));
    ||    break;
    */
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    // LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 1000);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void do_wifi(void *arg)
{
    while (1)
    {

        wifi_ap_record_t ap_info;
        wifi_sta_list_t wifi_sta_list;
        tcpip_adapter_sta_list_t adapter_sta_list;

        memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
        memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));

        esp_wifi_sta_get_ap_info(&ap_info);
        esp_wifi_ap_get_sta_list(&wifi_sta_list);
        tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

        for (int i = 0; i < adapter_sta_list.num; i++)
        {

            tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
            Serial.print("station nr ");
            Serial.println(i);

            Serial.print("MAC: ");

            for (int i = 0; i < 6; i++)
            {

                Serial.printf("%02X", station.mac[i]);
                if (i < 5)
                    Serial.print(":");
            }
            Serial.println();
        }

        int wifinum = WiFi.softAPgetStationNum();
        display.clear();
        display.display();
        display.drawStringMaxWidth(0, 0, 128, "WiFi dev num: " + String(wifinum));

        if (loraisconnected == false)
            display.drawStringMaxWidth(0, 10, 128, "LoRaWAN disconnected");
        else
        {
            display.drawStringMaxWidth(0, 10, 128, "LoRaWAN connected!");
        }

        display.drawStringMaxWidth(0, 20, 128, "LoRaWAN RSSI: " + String(LMIC.rssi) + "dBm");

        if (isshitdetected == false)
            display.drawStringMaxWidth(0, 30, 128, "Handheld device was not detected");
        else
        {
            display.drawStringMaxWidth(0, 30, 128, "Handheld device connected!");
        }

        display.display();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    Serial.begin(115200);

    /*
    Wifi init here
    */
    memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
    memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));

    Serial.print("Setting AP (Access Point)…");
    WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(WiFiStationDiconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    WiFi.mode(WIFI_AP);
    WiFi.begin(ssid, password);
    Serial.println(WiFi.softAPIP());

    /*
    End Wifi init
    */

    /*OLED init here */
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "OLED initial done!");
    display.display();
    // display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    display.clear();
    /* end oled init*/

    srand(time(NULL));
    Serial.println(F("Starting"));
    delay(1000);

#ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    xTaskCreate(do_wifi, "LoRa task", 30000, NULL, 3, &LoRaTask);
}

void loop()
{
    os_runloop_once();
}