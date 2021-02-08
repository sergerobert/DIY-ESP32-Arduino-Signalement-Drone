/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
    Used with ESP32 + BN220, modified by Julien Launay 14/09/2020
    Modified by Serge Robert 07/02/2021
    
    With https://github.com/f5soh/balise_esp32/blob/droneID_FR_testing/droneID_FR.h
    Connect ESP32 Pin16 (Rx)-> GPS reciever white wire (Tx)
    Connect ESP32 Pin17 (Tx)-> GPS receiver  green wire (Rx)
 */

// * This version is largely based on the one published in Modele Magazine * /
 
#include <Arduino.h>

#include <WiFi.h>
//#include <Wire.h>

#include <TinyGPS++.h>

// Note : modification of the name of the library (several versions published), here release used by Modele Magazine

#include "droneID_FRa.h"

extern "C" {
#include "esp_wifi.h"
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

#define UBLOX_GPS_OBJECT()  TinyGPSPlus gps
#define GPS_BANUD_RATE 9600
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define PMU_IRQ             35
#define GPS_POWER_CTRL_CH   3

UBLOX_GPS_OBJECT();

droneIDFR drone_idfr;

/********************************************************************************************************************
 * MODIFY THE VALUES HERE
 ********************************************************************************************************************/
// Set these to your desired credentials.
/**
  * The name of the access point (SSID) can be changed !!!
  */
const char ssid[] = "droneID_FR";

// Mot de pass du wifi
const char *password = "Test";
/**
    * * ID Alphatango drone that will be recorded with the 000 trigram (authorized by the DSAC for Manufacturers of devices
removable electronic reporting) + model + serial number - for example the registration number of your drone on Alphatango, without UAS-FR)
  */ 
const char drone_id[] = "000000000000000000000000000000";


/********************************************************************************************************************/
// Do not modify this part
// The transmission t uses channel 6 in accordance with the regulations
static constexpr uint8_t wifi_channel = 6;
// Ensure the drone_id is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Ensure the drone_id is max 30 letters
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination
// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
        0x80, 0x00,                                     // 0-1: Frame Control
        0x00, 0x00,                                     // 2-3: Duration
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             // 4-9: Destination address (broadcast)
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,             // 10-15: Source address FAKE  // TODO should bet set manually
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,             // 16-21: Source address FAKE
        0x00, 0x00,                                     // 22-23: Sequence / fragment number (done by the SDK)
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
        0xB8, 0x0B,                                     // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
        0x21, 0x04,                                     // 34-35: Capability info
        0x03, 0x01, 0x06,                               // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
        0x00, 0x20,                                     // 39-40: SSID parameter set, 0x20:maxlength:content
                                                        // 41-XX: SSID (max 32)
};

bool has_set_home = false;
double home_alt = 0.0;


uint8_t program = 0;


uint64_t dispMap = 0;

String dispInfo;
char buff[5][256];

uint64_t gpsSec = 0;
bool pmu_irq = false;

#define ARRARY_SIZE(a)   (sizeof(a) / sizeof(a[0]))


String baChStatus = "No charging";
String recv = "";

int8_t P1;
uint64_t beaconSec = 0;
const byte led_gpio = 2; //port number of the LED on the ESP32 GPIO2
bool stat_led = false;
/**
 * Configuration phase.
 */
void setup()
{
    Serial.begin(115200);

    delay(1000);
    //iinitialization of the GPS. Remember to cross Rx and Tx between ESP32 and GPS!
    Serial2.begin(GPS_BANUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    pinMode(led_gpio, OUTPUT);


/********************************************************************************************************************
 * access point initialization  
 */
    /**
     * Access point broadcast only SSID.
     */
    Serial.println("Starting AP");
    WiFi.softAP(ssid, nullptr, wifi_channel);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.print("AP mac address: ");
    Serial.println(WiFi.macAddress());
    wifi_config_t conf_current;
    esp_wifi_get_config(WIFI_IF_AP, &conf_current);
    // Change WIFI AP default beacon interval sending to 1s.
    conf_current.ap.beacon_interval = 1000;
    drone_idfr.set_drone_id(drone_id);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));
    delay(2000);
    
    esp_err_t r = esp_wifi_get_max_tx_power(&P1);
    
    Serial.print("Tx power Value (dBm)=");
    Serial.println(P1*0.25);
    //check if Tansmit power is at his max (20 dBm -> 100mW)
    if (P1>77) {
      byte i;
      for(i=0;i<10;i++){
        digitalWrite(led_gpio, HIGH);   
        delay(50);                      
        digitalWrite(led_gpio, LOW);   
        delay(50);
      }
    }
}

/**
 * Main code. With an infinite loop.
 */
void loop()
{
    static uint64_t gpsMap = 0;
    
    switch (program) {
    case 0:
        // Here the code read  data coming from the GPS and forward to the TinyGPS ++ library.
        while (Serial2.available())
            gps.encode(Serial2.read());
        // The code analyzes if the GPS has a problem
        if (millis() > 5000 && gps.charsProcessed() < 10) {
            snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
            snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
            
            Serial.println(buff[1]);
            digitalWrite(led_gpio, HIGH);
            return;
        }
        //The code verifies that the GPS position is valid
        if (!gps.location.isValid()) {
            if (millis() - gpsMap > 1000) {
                snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
                snprintf(buff[1], sizeof(buff[1]), "Positioning(%llu)", gpsSec++);

                Serial.print("Sentences ");
                Serial.print(gps.passedChecksum());
                Serial.print("  / Sentences failed checksum =");
                Serial.println(gps.failedChecksum());
               
                Serial.print("Number of satellites in use (u32) =");
                Serial.print(gps.satellites.value()); // Number of satellites in use (u32)
                Serial.print("  Horizontal Dim. of Precision (100ths-i32)= ");
                Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
                
                Serial.println(buff[1]);
                digitalWrite(led_gpio, HIGH);
                
                gpsMap = millis();
                delay(10);
                digitalWrite(led_gpio, LOW);
            }
        } else {
             // the code analyzes whether the GPS position is valid.
             // The code determines the starting point for the flight when the expected precision is provided by the GPS receiver
             // 6 SAT and 2 meters  - these value could be change for test
            if (!has_set_home && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
                Serial.println("Setting Home Position");
                //drone_idfr.set_home_lat_lon(gps.location.lat(), gps.location.lng());
                has_set_home = true;
                home_alt = gps.altitude.meters();
                Serial.println("Starting altitude ="+String(home_alt));
                drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
                digitalWrite(led_gpio, HIGH); //Blue Led onbord ON fixe, all is ok!

            }
            // The data (latitude, longitude, altitude, ...) are processed with the functions of the drone identification C library to provide a frame format compliant with the regulations.
            drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());

            drone_idfr.set_heading(gps.course.deg());
            drone_idfr.set_ground_speed(gps.speed.mps());
            drone_idfr.set_heigth(gps.altitude.meters() - home_alt);
            // Here is the code for sending the information to be displayed in the monitor via the USB port (for the tests).
            if (millis() - gpsMap > 1000) {
                
                snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
                snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
                snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
                snprintf(buff[3], sizeof(buff[3]), "satellites:%u", gps.satellites.value());
               
                Serial.println(buff[0]);
                Serial.println(buff[1]);
                Serial.println(buff[2]);
                Serial.println(buff[3]);
                
                gpsMap = millis();
            }
        }
        break;

    }
    /**
     * The code emits the drone identification frame: either every 3 seconds, or if the drones have moved at least 30 meters in less than 3 seconds.
* - either every 3 seconds,
      * - either if the drone has moved at least 30 meters in less than 30 seconds or 10 meters / seconds or 36 km / h.
      * - only if the Home position is already defined.
      * - and in the case of new GPS data.
     */
    if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
        Serial.println("Send beacon");
        // toggle the LED to see beacon sended
        if (stat_led) {
          digitalWrite(led_gpio, LOW);
          stat_led = false;
        }
        else {
          digitalWrite(led_gpio, HIGH);
          stat_led = true;
        }
        float time_elapsed = (float(millis() - beaconSec) / 1000); 
        beaconSec = millis();
        
        Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
        Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
        /**
         * Name of the SSID inserted in the frame
         */
        // write new SSID into beacon frame
        const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
        beaconPacket[40] = ssid_size;  // set size
        memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
        const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
        /**
         * construction la trame d'identification
         */
        const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
        
        // Comment out this block after the tests. it allows you to view the frame on the monitor via the usb (com) port.
        
        Serial.println("beaconPacket : ");
        for (auto i=0; i<sizeof(beaconPacket);i++) {
            Serial.print(beaconPacket[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");

        /**
         * Send the frame
         */

        ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));
        /**
         * Reset sending conditions
         */
        drone_idfr.set_last_send();
    }
    

}
