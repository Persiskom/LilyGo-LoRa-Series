#include <lmic.h>
#include <hal/hal.h>
#include "boards.h"
#include <TinyGPS++.h>

TinyGPSPlus gps;
double curlat, curlng;
#define DSIZE sizeof(double)

// Chose LSB mode on the console and then copy it here.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// LSB mode
static const u1_t PROGMEM DEVEUI[8] = {0x89, 0x9d, 0x2d, 0xae, 0x4e, 0xdd, 0x37, 0x72};
// MSB mode
static const u1_t PROGMEM APPKEY[16] = {
    0x0e, 0x69, 0x61, 0x1b, 0x1c, 0x9b, 0xae, 0x44, 0x77, 0xa3, 0x5c, 0x97, 0xb4, 0x27, 0x4e, 0xe4
};

// Pin mapping
#ifdef STM32L073xx
const lmic_pinmap lmic_pins = {
    .nss = RADIO_CS_PIN,
    .rxtx = RADIO_SWITCH_PIN,
    .rst = RADIO_RST_PIN,
    .dio = {RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_DIO2_PIN},
    .rx_level = HIGH
};
#else
const lmic_pinmap lmic_pins = {
    .nss = RADIO_CS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RADIO_RST_PIN,
    .dio = {RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_BUSY_PIN}
};
#endif

static osjob_t sendjob;
static osjob_t rejoinjob;
static osjob_t infojob;
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 30;
static const unsigned INFO_INTERVAL = 2;
static const unsigned REJOIN_INTERVAL = 60;
static String lora_msg = "";

void os_getArtEui(u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui(u1_t *buf)
{
    memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey(u1_t *buf)
{
    memcpy_P(buf, APPKEY, 16);
}

void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID, CAN'T REACH SATELLITE"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}

void do_info(osjob_t *j)
{
    // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial1.available() > 0)
        if (gps.encode(Serial1.read())){
            displayInfo();
            break;
        }
    os_setTimedCallback(&infojob, os_getTime() + sec2osticks(INFO_INTERVAL), do_info);
}

void do_rejoin(osjob_t *j)
{
    if (joinStatus == EV_JOINING)
    {
        Serial.println("Joining process takes too long, rejoining...");
        LMIC_startJoining();
        os_setTimedCallback(&rejoinjob, os_getTime() + sec2osticks(REJOIN_INTERVAL), do_rejoin);
    }
}

void do_send(osjob_t *j)
{
    if (joinStatus == EV_JOINING)
    {
        Serial.println(F("Not joined yet"));
        // Check if there is not a current TX/RX job running
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
    else if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        if (gps.location.isValid())
        {
            Serial.println(F("OP_TXRXPEND,sending ..."));
            curlng = gps.location.lng();
            curlat = gps.location.lat();
            static uint8_t coordByte[DSIZE * 2];
            memcpy(coordByte, &curlat, DSIZE);
            memcpy(coordByte + DSIZE, &curlng, DSIZE);
            // Prepare upstream data transmission at the next possible time.
            LMIC_setTxData2(1, coordByte, sizeof(coordByte), 0);
        }
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

        #ifdef HAS_DISPLAY
            if (u8g2)
            {
                char buf[256];
                u8g2->clearBuffer();
                snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
                u8g2->drawStr(0, 12, buf);
                u8g2->sendBuffer();
            }
        #endif
    }
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK)
            {
                Serial.println(F("Received ack"));
                lora_msg = "Received ACK.";
            }

            lora_msg = "rssi:" + String(LMIC.rssi) + " snr: " + String(LMIC.snr);

            if (LMIC.dataLen)
            {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                // Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
                // Serial.println();
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            lora_msg = "OTAA joining....";
            joinStatus = EV_JOINING;
            #ifdef HAS_DISPLAY
                if (u8g2)
                {
                    u8g2->clearBuffer();
                    u8g2->drawStr(0, 12, "OTAA joining....");
                    u8g2->sendBuffer();
                }
            #endif
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
            lora_msg = "OTAA Joining failed";
            #ifdef HAS_DISPLAY
                if (u8g2)
                {
                    u8g2->clearBuffer();
                    u8g2->drawStr(0, 12, "OTAA joining failed");
                    u8g2->sendBuffer();
                }
            #endif
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            lora_msg = "Joined!";
            joinStatus = EV_JOINED;

            #ifdef HAS_DISPLAY
                if (u8g2)
                {
                    u8g2->clearBuffer();
                    u8g2->drawStr(0, 12, "Joined TTN!");
                    u8g2->sendBuffer();
                }
            #endif
            delay(3);
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);

            // Display info to serial
            do_info(&infojob);
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
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setupLMIC(void)
{
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    // LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    // LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(spreadFactor, 14);

    // Start job
    LMIC_startJoining();

    // Send GPS data
    do_send(&sendjob);

    // Rejoin job
    os_setTimedCallback(&rejoinjob, os_getTime() + sec2osticks(REJOIN_INTERVAL), do_rejoin);
}

void loopLMIC(void)
{
    os_runloop_once();
}