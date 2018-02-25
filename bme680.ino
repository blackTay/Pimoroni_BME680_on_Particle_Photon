// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

// This #include statement was automatically added by the Particle IDE.
// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_BME680.h>

/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

void callback(char* topic, byte* payload, unsigned int length);
MQTT client("host", 1883,callback);

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

double temperatureInC = 0;
double relativeHumidity = 0;
double pressureHpa = 0;
double gasResistanceKOhms = 0;
double approxAltitudeInM = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  
}
void setup() {
  RGB.brightness(0); // optional to turn off status led on the partcle
  if (!bme.begin(0x76)) {
    Particle.publish("Log", "Could not find a valid BME680 sensor, check wiring!");
  } else {
    Particle.publish("Log", "bme.begin() success =)");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    Particle.variable("temperature", &temperatureInC, DOUBLE);
    Particle.variable("humidity", &relativeHumidity, DOUBLE);
    Particle.variable("pressure", &pressureHpa, DOUBLE);
    Particle.variable("gas", &gasResistanceKOhms, DOUBLE);
    Particle.variable("altitude", &approxAltitudeInM, DOUBLE);
  }
  
}

void loop() {
  client.connect("ParticlePhoton");    
  if (! bme.performReading()) {
    Particle.publish("Log", "Failed to perform reading :(");
  } else {
    temperatureInC = bme.temperature;
    relativeHumidity = bme.humidity;
    pressureHpa = bme.pressure / 100.0;
    gasResistanceKOhms = bme.gas_resistance / 1000.0;
    approxAltitudeInM = bme.readAltitude(SEALEVELPRESSURE_HPA);

    String data = String::format(
      "{"
        "\"temperatureInC\":%.2f,"
        "\"humidityPercentage\":%.2f,"
        "\"pressureHpa\":%.2f,"
        "\"gasResistanceKOhms\":%.2f,"
        "\"approxAltitudeInM\":%.2f"
      "}",
      temperatureInC,
      relativeHumidity,
      pressureHpa,
      gasResistanceKOhms,
      approxAltitudeInM);

    //Particle.publish("Sensor 3", data, 60, PRIVATE);
     if (client.isConnected()) {
        client.publish("Home/bme680sensor",data);
    }
  }
  delay(10 * 1000);
}

