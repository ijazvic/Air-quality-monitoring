//Uključivanje biblioteka
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include "AdafruitIO_WiFi.h"
#include <Wire.h>
#include <math.h>


// Adafruit IO podaci za pristup
#define IO_USERNAME "ilijajazz"
#define IO_KEY "aio_aNIG048z2cn3mpB7A1lc5kVbCBcV"
#define WIFI_SSID "MillenniumCaffe"
#define WIFI_PASS "pitajBrunE1304?"


// Definiranje pinova za wifi
#if defined(USE_AIRLIFT) || defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || \
    defined(ADAFRUIT_PYPORTAL)
// Konfiguracija pinova za ESP32 konekciju
#if !defined(SPIWIFI_SS) // ako WiFi definicija nije u varijanti ploče
#define SPIWIFI SPI
#define SPIWIFI_SS 10 // Chip select pin
#define NINA_ACK 9    // BUSY ili READY pin
#define NINA_RESETN 6 // Reset pin
#define NINA_GPIO0 -1 // Nije spojeno
#endif
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, SPIWIFI_SS, NINA_ACK, NINA_RESETN, NINA_GPIO0, &SPIWIFI);
#else
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
#endif


// GPIO pinovi za senzore
#define DHT22_PIN 25 // Pin za DHT22 senzor
#define RELAY_PIN 32 // Pin za relej
#define PIN_MQ135 36 // Pin za MQ135 senzor
#define LED_PIN 14   // Pin za LED diodu
#define BTN_PIN 33   // Pin za gumb


// Konstante za MQ135 senzor
#define RLOAD 10.0      // Otpor na ploči (u kOhm)
#define RZERO 76.63     // Kalibracijski otpor pri atmosferskom CO2 nivou
#define ATMOCO2 415.58  // Atmosferski CO2 nivo za kalibraciju
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018
#define CORE -0.003333333
#define CORF -0.001923077
#define CORG 1.130128205
#define PARA 116.6020682
#define PARB 2.769034857


// Adresa I2C za BMP085/BMP180
#define BMP085_I2CADDR 0x77

// BMP085 registri
#define BMP085_CAL_AC1 0xAA
#define BMP085_CAL_AC2 0xAC
#define BMP085_CAL_AC3 0xAE
#define BMP085_CAL_AC4 0xB0
#define BMP085_CAL_AC5 0xB2
#define BMP085_CAL_AC6 0xB4
#define BMP085_CAL_B1  0xB6
#define BMP085_CAL_B2  0xB8
#define BMP085_CAL_MB  0xBA
#define BMP085_CAL_MC  0xBC
#define BMP085_CAL_MD  0xBE
#define BMP085_CONTROL 0xF4
#define BMP085_TEMPDATA 0xF6
#define BMP085_PRESSUREDATA 0xF6
#define BMP085_READTEMPCMD 0x2E
#define BMP085_READPRESSURECMD 0x34

// Načini rada BMP085
#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD 1
#define BMP085_HIGHRES 2
#define BMP085_ULTRAHIGHRES 3


// Globalne varijable za kalibracijske podatke za BMP180
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
uint8_t oversampling;

//Globalne varijable za spremanje vrijednosti senzora
float resistance;
float PPM;
float temperature;
float humidity;
int pressure;
float altitude;

//Timeri za slanje i ispis podataka
unsigned long previousMillisPrint = 0;
unsigned long previousMillisSend = 0;
const long intervalPrint = 2000; // Interval za ispis u ms (2 sekunde)
const long intervalSend = 60000; // Interval za slanje na Adafruit IO u ms (60 sekundi)

// Inicijalizacija Adafruit IO feed-ova za temperature, vlažnost, ppm, tlak i visinu
AdafruitIO_Feed *temperature_feed = io.feed("temperature");
AdafruitIO_Feed *humidity_feed = io.feed("humidity");
AdafruitIO_Feed *ppm_feed = io.feed("ppm");
AdafruitIO_Feed *pressure_feed = io.feed("pressure");
AdafruitIO_Feed *altitude_feed = io.feed("altitude");


//////////////////////////////////// Rad s BMP 180 senzorom ////////////////////////////////////
// Inicijalizacija BMP180 senzora
bool begin(uint8_t mode) {
    if (mode > BMP085_ULTRAHIGHRES)
        mode = BMP085_ULTRAHIGHRES;
    oversampling = mode;

    if (read8(0xD0) != 0x55)
        return false;

    ac1 = read16(BMP085_CAL_AC1);
    ac2 = read16(BMP085_CAL_AC2);
    ac3 = read16(BMP085_CAL_AC3);
    ac4 = read16(BMP085_CAL_AC4);
    ac5 = read16(BMP085_CAL_AC5);
    ac6 = read16(BMP085_CAL_AC6);
    b1 = read16(BMP085_CAL_B1);
    b2 = read16(BMP085_CAL_B2);
    mb = read16(BMP085_CAL_MB);
    mc = read16(BMP085_CAL_MC);
    md = read16(BMP085_CAL_MD);

    return true;
}

// Izračun B5 vrijednosti 
int32_t computeB5(int32_t UT) {
    int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
    int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
    return X1 + X2;
}

// Čitanje sirove temperature za BMP 180 radi dobivanja podataka za izračun atmosferskog pritiska
uint16_t readRawTemperature(void) {
    write8(BMP085_CONTROL, BMP085_READTEMPCMD);
    delay(5); // 5ms kašnjenje
    return read16(BMP085_TEMPDATA);
}

// Čitanje sirovog tlaka
uint32_t readRawPressure(void) {
    uint32_t raw;

    write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

    if (oversampling == BMP085_ULTRALOWPOWER)
        delay(5); // 5ms
    else if (oversampling == BMP085_STANDARD)
        delay(8); // 8ms
    else if (oversampling == BMP085_HIGHRES)
        delay(14); // 14ms
    else
        delay(26); // 26ms

    raw = read16(BMP085_PRESSUREDATA);
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA + 2);
    raw >>= (8 - oversampling);

    return raw;
}

// Čitanje amotsferskog pritiska
int32_t readPressure(void) {
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    UT = readRawTemperature();
    UP = readRawPressure();

    B5 = computeB5(UT);

    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + (int32_t)3791) >> 4);
        return p;
}

// Čitanje nadmorske visine
float readAltitude(float sealevelPressure) {
    float altitude;
    float pressure = readPressure();
    altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));
    return altitude;
}

// Čitanje 8-bitnog registra preko I2C
uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(BMP085_I2CADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_I2CADDR, 1);
    return Wire.read();
}

// Čitanje 16-bitnog registra preko I2C
uint16_t read16(uint8_t reg) {
    uint16_t value;
    Wire.beginTransmission(BMP085_I2CADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_I2CADDR, 2);
    value = (Wire.read() << 8) | Wire.read();
    return value;
}

// Pisanje 8-bitnog registra preko I2C
void write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BMP085_I2CADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
//////////////////////////////////// Kraj koda za rad s BMP 180 senzorom ////////////////////////////////////


//////////////////////////////////// Rad s DHT22 senzorom ////////////////////////////////////
// Čitanje podataka s DHT22 senzora
bool readDHT22(int *data) {
    uint8_t lastState = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;

    // Inicijacija komunikacije s DHT22 senzorom
    pinMode(DHT22_PIN, OUTPUT);
    digitalWrite(DHT22_PIN, LOW);
    delay(20);
    digitalWrite(DHT22_PIN, HIGH);
    delayMicroseconds(30);
    pinMode(DHT22_PIN, INPUT);

    // Čitanje odgovora s DHT22 senzora
    for (i = 0; i < 85; i++) {
        counter = 0;
        while (digitalRead(DHT22_PIN) == lastState) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) {
                break;
            }
        }
        lastState = digitalRead(DHT22_PIN);

        if (counter == 255) {
            break;
        }

        if ((i >= 4) && (i % 2 == 0)) {
            data[j / 8] <<= 1;
            if (counter > 16) {
                data[j / 8] |= 1;
            }
            j++;
        }
    }

    // Provjera checksuma
    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        return true;
    } else {
        return false;
    }
}
//////////////////////////////////// Kraj koda za rad s DHT22 senzorom ////////////////////////////////////


//////////////////////////////////// Rad s MQ135 senzorom ////////////////////////////////////
// Dobivanje korekcijskog faktora za MQ135 senzor
float getCorrectionFactor(float t, float h) {
    if (t < 20) {
        return CORA * t * t - CORB * t + CORC - (h - 33.0) * CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

// Dobivanje otpora MQ135 senzora
float getResistance(int pin) {
    int val = analogRead(pin);
    return ((4095.0 / (float)val) - 1.0) * RLOAD;
}

// Dobivanje korigiranog otpora MQ135 senzora
float getCorrectedResistance(float resistance, float t, float h) {
    return resistance / getCorrectionFactor(t, h);
}

// Dobivanje PPM vrijednosti CO2
float getPPM(float resistance, float t, float h) {
    return PARA * pow((getCorrectedResistance(resistance, t, h) / RZERO), -PARB);
}
//////////////////////////////////// Kraj koda za rad s MQ135 senzorom ////////////////////////////////////


// Inicijalizacija
void setup() {
    //Postavljanje pinova za rad s senzorima
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);

    //Postavljanje serijske komunikacije
    Serial.begin(115200);

    // Inicijalizacija I2C komunikacije
    Wire.begin(); 
    while (!Serial); // Čeka se otvaranje serijskog monitora

    Serial.print("Spajanje na Adafruit IO ");
    io.connect(); // Spajanje na Adafruit IO

    // Čeka se uspostava veze
    while (io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.println(io.statusText()); // Ispis statusa veze

    // Provjera inicijalizacije BMP180
    if (!begin(BMP085_STANDARD)) {
        Serial.println("Inicijalizacija BMP180 nije uspjela!");
        while (1);
    }else{
        Serial.println("Inicijalizacija BMP085 uspješna!");
    }
    
    // Provjera inicijalizacije MQ135 senzora
    if (getResistance(PIN_MQ135) > 0) {
        Serial.println("Inicijalizacija MQ135 senzora uspješna!");
    } else {
        Serial.println("Nije uspjela inicijalizacija MQ135 senzora!");
    }

    // Provjera inicijalizacije DHT22 senzora
    int checkData[5] = {0, 0, 0, 0, 0}; // Polje za podatke o temperaturi i vlažnosti
    while (!readDHT22(checkData)) {
        Serial.println("Nije uspjela inicijalizacija DHT22 senzora!");
    } 
    Serial.println("Inicijalizacija DHT22 senzora uspješna!");
    
    Serial.println();
    delay(1000);  // Čekanje 1 sekundu za stabilizaciju serijske komunikacije
}

// Glavna petlja
void loop() {
    io.run(); // Pokretanje Adafruit IO komunikacije

    unsigned long currentMillis = millis();

    // Ispis podataka svakih 2 sekunde
    if (currentMillis - previousMillisPrint >= intervalPrint) {
        previousMillisPrint = currentMillis;

        int data[5] = {0, 0, 0, 0, 0}; // Polje za podatke o temperaturi i vlažnosti

        // Čitanje podataka s DHT22 senzora
        resistance = getResistance(PIN_MQ135);
        pressure = readPressure();
        if (readDHT22(data) && resistance < 200 && (pressure > 0 && pressure < 150000)) {
            temperature = data[2] & 0x7F;
            temperature *= 256;
            temperature += data[3];
            temperature *= 0.1;

            if (data[2] & 0x80) {
                temperature *= -1;
            }

            humidity = data[0];
            humidity *= 256;
            humidity += data[1];
            humidity *= 0.1;

            PPM = getPPM(resistance, temperature, humidity);
            altitude = readAltitude(101500); // Tlak na razini mora u Pa

            Serial.print("Temperatura: ");
            Serial.print(temperature);
            Serial.println(" *C");
            Serial.print("Vlaznost: ");
            Serial.print(humidity);
            Serial.println(" %\t");           
            Serial.print("PPM: ");
            Serial.print(PPM);
            Serial.println(" ppm");
            Serial.print("Pritisak = ");
            Serial.print(pressure);
            Serial.println(" Pa");
            Serial.print("Visina = ");
            Serial.print(altitude);
            Serial.println(" metara");
            Serial.println();

        } else {
            Serial.println("Neuspjelo čitanje podataka s jednim od senzora.");
        }

        // Upravljanje LED diodom i relejem prema uvjetima
        if (digitalRead(BTN_PIN) == LOW) {
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("Prskalice su uključene.");
        } else if (PPM > 1000.0) {
            digitalWrite(RELAY_PIN, HIGH);
            if (temperature > 27) {
                digitalWrite(LED_PIN, HIGH);
                Serial.println("Prskalice su uključene.");
            } else {
                digitalWrite(LED_PIN, LOW);
            }
        } else {
            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
        }
    }

    // Slanje podataka na Adafruit IO svake minute
    if (currentMillis - previousMillisSend >= intervalSend) {
        previousMillisSend = currentMillis;

        temperature_feed->save(temperature);
        humidity_feed->save(humidity);
        ppm_feed->save(PPM);
        pressure_feed->save(pressure);
        altitude_feed->save(altitude);
    }
}
