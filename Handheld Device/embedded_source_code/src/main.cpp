#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "WiFi.h"
#include "SSD1306Wire.h"

#define DEFAULT_WATCHDOG_EMERGENCY (10 * 60) // 10 seconds
#define DEFAULT_TIMEOUT_EMERGENCY (10 * 30)  // 10 seconds

String wifi, lora, emergency_string;
int mess_len = 0;
SSD1306Wire display(0x3c, 21, 22, -1, GEOMETRY_128_64);
static bool emergency = false;
int watchdog_emergency = 10 * 1; // 1s
int timeout_emergency = 10 * 10; // 10s

static const u1_t PROGMEM APPEUI[8] = {0xA2, 0xCB, 0x35, 0xFA, 0x39, 0x4E, 0x0F, 0x0A};
static const u1_t PROGMEM DEVEUI[8] = {0x95, 0x7A, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
static const u1_t PROGMEM APPKEY[16] = {0x69, 0xE8, 0x59, 0x72, 0x0F, 0xB9, 0x85, 0xB7, 0x91, 0x32, 0xAB, 0x45, 0xB5, 0xE8, 0x0D, 0xA1};

void do_send(osjob_t *j);
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

enum emergency
{
    NO_EMERGENCY = 0x00,
    EMERGENCY = 0x01
};

static uint8_t mydata = NO_EMERGENCY;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 5;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}};

void onEvent(ev_t ev)
{
    Serial.print("LoRa event: ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println("SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        Serial.println("BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        Serial.println("BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        Serial.println("BEACON_TRACKED");
        break;
    case EV_JOINING:
        Serial.println("JOINING");
        break;
    case EV_JOINED:
        Serial.println("JOINED");

        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case RFU1:
    ||     Serial.println("RFU1");
    ||     break;
    */
    case EV_JOIN_FAILED:
        Serial.println("JOIN_FAILED");
        break;
    case EV_REJOIN_FAILED:
        Serial.println("REJOIN_FAILED");
        break;
    case EV_TXCOMPLETE:
        Serial.println("TXCOMPLETE (includes waiting for RX windows)");
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ack");
        if (LMIC.dataLen)
        {
            char mess[LMIC.dataLen];
            memcpy(mess, LMIC.frame, LMIC.dataLen);
            mess_len = LMIC.dataLen;
            Serial.printf("Received %d bytes of payload, RSSI: %d\n", LMIC.dataLen, LMIC.rssi);
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println("LOST_TSYNC");
        break;
    case EV_RESET:
        Serial.println("RESET");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println("RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        Serial.println("LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        Serial.println("LINK_ALIVE");
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    Serial.println("SCAN_FOUND");
    ||    break;
    */
    case EV_TXSTART:
        Serial.println("TXSTART");
        break;
    case EV_TXCANCELED:
        Serial.println("TXCANCELED");
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println("JOIN_TXCOMPLETE: no JoinAccept");
        break;

    default:
        Serial.print("Unknown event: ");
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println("OP_TXRXPEND, not sending");
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, &mydata, 1, 1);
        Serial.println("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void set_emergency()
{
    mydata = EMERGENCY;
    emergency = true;
    emergency_string = "EMERGENCY CALL";
    digitalWrite(12, 1);
}

static uint32_t button_time;

void IRAM_ATTR Ext_INT1_ISR() // rising edge
{
    set_emergency();
    // button_time = millis();
    // watchdog_emergency = DEFAULT_WATCHDOG_EMERGENCY;
    // timeout_emergency = DEFAULT_TIMEOUT_EMERGENCY;
}


void setup()
{
    Serial.begin(115200);
    pinMode(4, INPUT_PULLDOWN); // BUTTOn
    pinMode(12, OUTPUT);        // LED
    digitalWrite(12, 0);
    WiFi.mode(WIFI_STA);
    WiFi.begin("ESP32-Access-Point", "twojstarypijany");

    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.display();

    os_init();
    LMIC_reset();
LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 1000);
    do_send(&sendjob);
    attachInterrupt(4, Ext_INT1_ISR, RISING);
}

void loop()
{
    display.clear();

    if (WiFi.status() == WL_CONNECTED)
    {
        wifi = "WiFi: " + String(WiFi.RSSI()) + " dBm";
    }
    else
    {
        wifi = String("WiFi not connected");
    }

    if (mess_len == 4)
    {
        lora = "Go back to the shelter!";
    }
    else
    {
        lora = "The track is safe :)";
    }

    display.drawString(0, 0, wifi);
    display.drawString(0, 10, "LORA RSSI: " + String(LMIC.rssi) + " dBm");
    display.drawString(0, 40, lora);
    display.drawString(0, 50, emergency_string);
    display.display();

    os_runloop_once();
}