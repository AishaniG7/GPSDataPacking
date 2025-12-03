#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include "gpsPacket.h"

#define LED_PIN PIN_015       // Onboard LED, change if different
#define OLED_RESET -1         // OLED reset pin
#define GPS_SERIAL Serial1    // GPS module serial port
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64


#define SX126X_CS    (32 + 13) // P1.13
#define SX126X_DIO1  (0 + 10)  // P0.10
#define SX126X_BUSY  (0 + 29)  // P0.29
#define SX126X_RESET (0 + 9)   // P0.09
#define SX126X_RXEN  (0 + 17)  // P0.17
#define SX126X_TXEN  RADIOLIB_NC

//roles
#define ROLE_TX
//#define ROLE_RX uncomment this for the RX board


// LoRa modem settings
const float FREQ_MHZ = 906.875;
const uint8_t SF = 11;
const unsigned long BW = 250000;
const uint8_t CR = 5;
const uint16_t PREAMBLE = 16;
const uint8_t SYNCWORD = 0x2B;


Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET);
uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
TinyGPSPlus gps;
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);

char latBuffer[30], lngBuffer[30], dateBuffer[30];
double lastLat = 0, lastLng = 0;
TinyGPSDate lastDate;

volatile bool receivedFlag = false; // set by ISR when a packet arrives

void setFlag() {
    receivedFlag = true;
}

void setup() {
    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, LOW);

    GPS_SERIAL.begin(9600);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        digitalWrite(PIN_LED1, HIGH);
        while (true);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();

    Serial.print(F("[SX1262] Initializing ... "));
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) { delay(10); }
    }

    radio.setFrequency(FREQ_MHZ);
    radio.setSpreadingFactor(SF);
    radio.setBandwidth(BW);
    radio.setCodingRate(CR);
    radio.setPreambleLength(PREAMBLE);
    radio.setSyncWord(SYNCWORD);

    radio.setDio1Action(setFlag);

    Serial.print(F("[SX1262] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) { delay(10); }
    }

    display.clearDisplay();
    display.display();
}

void loop() {

    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    bool locationUpdated = gps.location.isUpdated();
    bool dateUpdated = gps.date.isUpdated();

    //displays gps coords and transmits
	//rn its transmitting gps coordinates back to itself
	#ifdef ROLE_TX 
		if (locationUpdated) {
			double lat = gps.location.lat();
			double lng = gps.location.lng();
			bool redraw = false;

			if (lat != lastLat) {
				lastLat = lat;
				snprintf(latBuffer, sizeof(latBuffer), "Lat: %3.6f", lat);
				redraw = true;
			}
			if (lng != lastLng) {
				lastLng = lng;
				snprintf(lngBuffer, sizeof(lngBuffer), "Lng: %3.6f", lng);
				redraw = true;
			}

			if (redraw) {
				// Update OLED
				display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
				display.setCursor(0, 0);
				display.print(latBuffer);
				display.setCursor(0, 10);
				display.print(lngBuffer);
				display.display();

				// Build GPS packet
				uint8_t outBuf[sizeof(GPSPacket)];
				buildPacket(lastLat, lastLng, outBuf);

				// Transmit
				radio.standby();
				int st = radio.transmit(outBuf, sizeof(GPSPacket));
				if (st == RADIOLIB_ERR_NONE) Serial.println("TX: GPS packet sent");
				else {
					Serial.print("TX error: "); Serial.println(st);
				}

				// Return to RX
				radio.startReceive();
			}
    }
	#endif

    if (dateUpdated) {
        TinyGPSDate date = gps.date;
        if (date.isValid() && date.value() != lastDate.value()) {
            lastDate = date;
            snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d",
                     date.year(), date.month(), date.day());
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            display.setCursor(0, 20);
            display.print(dateBuffer);
            display.display();
        }
    }

    //recieving gps packets
	#ifdef ROLE_RX
		if (receivedFlag) {
			receivedFlag = false;

			uint8_t inBuf[sizeof(GPSPacket)];
			int state = radio.readData(inBuf, sizeof(GPSPacket));

			if (state == RADIOLIB_ERR_NONE) {
				float rxLat, rxLng;
				uint8_t rxNode;
				parsePacket(inBuf, rxLat, rxLng, rxNode);

				Serial.print("Node "); Serial.print(rxNode);
				Serial.print(" Lat="); Serial.print(rxLat, 6);
				Serial.print(" Lng="); Serial.println(rxLng, 6);

				// Update OLED
				display.setCursor(0, 40);
				display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
				display.print("RX ");
				display.print(rxNode);
				display.print(": ");
				display.print(rxLat, 4);
				display.print(",");
				display.print(rxLng, 4);
				display.display();

				// Debug info
				Serial.print("RSSI: "); Serial.println(radio.getRSSI());
				Serial.print("SNR: "); Serial.println(radio.getSNR());
			}
			else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
				Serial.println("CRC error!");
				radio.startReceive()
			}
			else {
				Serial.print("RX error: "); Serial.println(state);
				radio.startReceive()
			}

			TinyGPSDate date = gps.date;
			snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d",
					date.year(), date.month(), date.day());
			display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
			display.setCursor(0,20);
			display.print(dateBuffer);
			display.display();

			// Re-enable RX mode
			radio.startReceive();
		}
	#endif
}
