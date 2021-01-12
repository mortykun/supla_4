/**
 * Supla.org NodeMCU WiFi minimal example
 * Author: Programistyk - Kamil Kaminski <kamil@programistyk.pl>
 * 
 * This example shows how to configure SuplaDevice for building for NodeMCU within Arduino IDE
 */


#include <srpc.h>
#include <log.h>
#include <eh.h>
#include <proto.h>
#include <IEEE754tools.h>
// We define our own ethernet layer
#define SUPLADEVICE_CPP
#include <SuplaDevice.h>
#include <lck.h>

#include <WiFiClient.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiServer.h>
#include <ESP8266WiFiGeneric.h>
#include <WiFiClientSecure.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiSTA.h>
#include <WiFiUdp.h>

#include <OneWire.h>
#include <DallasTemperature.h>
int button1 = 0; //wartosc początkowa dla przycisku 1

WiFiClient client;

OneWire oneWire(4); //  (D2) - Pin number definiujemyna którym pinie będzie podłączony czujnik DS18B20
DallasTemperature sensors(&oneWire);


// Setup Supla connection
const char* ssid     = "xxxxxxx";
const char* password = "xxxxxxx";



// DS18B20 Sensor read implementation
double get_temperature(int channelNumber, double last_val) { //ta część kodu odpowiada za odczyt temperatury

    double t = -275;

    if ( sensors.getDeviceCount() > 0 )
    {
        sensors.requestTemperatures();
        t = sensors.getTempCByIndex(0);
    };

    return t;
}




void setup() {
    Serial.begin(115200);

    pinMode (16, INPUT_PULLUP); //ustawiamy Pin 16 (D0) na przycisk
    // Init DS18B20 library
    sensors.begin();

    // Set temperature callback
    SuplaDevice.setTemperatureCallback(&get_temperature);

    delay(10);

    // ﻿Replace the falowing GUID
    char GUID[SUPLA_GUID_SIZE] = {0xC2,0x57,0x7A,0x1A,0xA0,0x5C,0xE2,0xA6,0xDB,0xD5,0x48,0xEB,0x2A,0x96,0x28,0x14};
    // ﻿pobieramy identyfikator urządzenia ze strony https://www.supla.org/arduino/get-guid i wprowadzamy wiersz wyżej

    // Ethernet MAC address
    uint8_t mac[6] = {0x00, 0x05, 0x04, 0x03, 0x02, 0x01}; // adres mac wpisujemy dowolny

    /*
     * Having your device already registered at cloud.supla.org,
     * you want to change CHANNEL sequence or remove any of them,
     * then you must also remove the device itself from cloud.supla.org.
     * Otherwise you will get "Channel conflict!" error.
     */

    // CHANNEL0 - RELAY
    SuplaDevice.addRelay(D5, true);           //definiujemy co chcemy widzieć pod danym kanałem. Tu mamy przekaźnik. true oznacza, ze
    //aktywujemy go masą. D5 to GPIO14. mozna wpisywać naprzemiennie

    // CHANNEL1 - RELAY
    SuplaDevice.addRelay(D6, true);

    // CHANNEL2 - RELAY
    SuplaDevice.addRelay(D7, true);

    // CHANNEL3 - RELAY
    SuplaDevice.addRelay(D8, true);

    // CHANNEL4 - Thermometer DS18B20
    SuplaDevice.addDS18B20Thermometer();  //kanał 4 to temomemetr


    // CHANNEL - Opening sensor (Normal Open)
    //SuplaDevice.addSensorNO(A0); // A0 - ﻿sensor otwarcia drzwi. na razie go nie używam. podłączamy go przez rezystor 10k
    // jezeli nie wiesz o czym mowa to zajrzyj tu https://forbot.pl/blog/microswitche-proste-czujniki-przeszkod-id1870


    // CHANNEL5 - Opening sensor (Normal Open)
    //SuplaDevice.addSensorNO(A1); // A1 - ﻿Pin number where the sensor is connected




    // CHANNEL6 - DHT22 Sensor // pozostałe możliwe czujniki do wykorzystania
    // SuplaDevice.addDHT11();
    // SuplaDevice.addAM2302();
    // SuplaDevice.addDHT22();

    SuplaDevice.begin(GUID,              // Global Unique Identifier
                      mac,               // Ethernet MAC address
                      "svrxxxxx",  // SUPLA server address   // w miejsca xxxx wprowadzamy dane z clouda
                      xxxx,                 // Location ID
                      "xxxx");               // Location Password

}

void loop() {
    SuplaDevice.iterate();


// ponizej kod umozliwiający włączać i wyłączać przekaźnik 1 z dodatkowego fizycznego przycisku dzwonkowego. W moim przypadku 
// otwieranie/zamykanie bramy

    TSD_SuplaChannelNewValue przycisk1; //ustaw nazwe dla przycisku
    przycisk1.SenderID = 0; // Powiadom clouda, że załączasz recznie. W przypadku siłowników ma być 0
    przycisk1.ChannelNumber = 1; // nr kanału przekaźnika
    przycisk1.DurationMS = 0; //czas wlaczenia

    button1 = digitalRead(16);
    if(digitalRead(16)==LOW){ // tu dodajemy jeszcze raz zeby nie pstrykalo samo czyli przerwa i ponowne zapytanie
        delay(100);
        if(digitalRead(16)==LOW){   //sprawdzam 2 razy stan na PINie 16 w odstępach 100ms . bez tego było dużo zakłuceń
            przycisk1.value[0] = !przycisk1.value[0];
            SuplaDevice.channelSetValue(&przycisk1);
            while(digitalRead(16)==LOW);
            delay(20);
        }

    }}

// Supla.org ethernet layer
int supla_arduino_tcp_read(void *buf, int count) {
    _supla_int_t size = client.available();

    if ( size > 0 ) {
        if ( size > count ) size = count;
        return client.read((uint8_t *)buf, size);
    };

    return -1;
};

int supla_arduino_tcp_write(void *buf, int count) {
    return client.write((const uint8_t *)buf, count);
};

bool supla_arduino_svr_connect(const char *server, int port) {
    return client.connect(server, 2015);
}

bool supla_arduino_svr_connected(void) {
    return client.connected();
}

void supla_arduino_svr_disconnect(void) {
    client.stop();
}

void supla_arduino_eth_setup(uint8_t mac[6], IPAddress *ip) {

    // Serial.println("WiFi init");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //    Serial.print(".");
    }

    //Serial.print("\nlocalIP: ");
    //Serial.println(WiFi.localIP());
    //Serial.print("subnetMask: ");
    //Serial.println(WiFi.subnetMask());
    //Serial.print("gatewayIP: ");
    //Serial.println(WiFi.gatewayIP());
}

SuplaDeviceCallbacks supla_arduino_get_callbacks(void) {
    SuplaDeviceCallbacks cb;

    cb.tcp_read = &supla_arduino_tcp_read;
    cb.tcp_write = &supla_arduino_tcp_write;
    cb.eth_setup = &supla_arduino_eth_setup;
    cb.svr_connected = &supla_arduino_svr_connected;
    cb.svr_connect = &supla_arduino_svr_connect;
    cb.svr_disconnect = &supla_arduino_svr_disconnect;
    cb.get_temperature = NULL; //&get_temperature;
    cb.get_temperature_and_humidity = NULL;
    cb.get_rgbw_value = NULL;
    cb.set_rgbw_value = NULL;

    return cb;
}