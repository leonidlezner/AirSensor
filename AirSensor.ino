#include <Adafruit_Si7021.h>
#include "RF24.h"
#include <SPI.h>


#define PIN_CSN 11
#define PIN_CE  12
#define VBATPIN A7

RF24 radio(PIN_CE, PIN_CSN);

Adafruit_Si7021 sensor = Adafruit_Si7021();

const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

int sensorId = 0;
uint8_t sensorError = 0;

#define CHARGE_CURVE_POINTS 7
unsigned int batt_mv_curve[] = { 4200, 4030, 3860, 3830, 3790, 3700, 3600 };
unsigned char batt_charge_curve[] = { 100, 76, 52, 42, 30, 11, 0 };

struct __attribute__((packed)) SensorPacket
{
  uint8_t sensorId;
  uint8_t status;
  uint32_t localTime;
  uint8_t battery;
  uint16_t temperature;
  uint16_t humidity;
  uint16_t airquality;
  uint16_t pressure;
};

unsigned char charge_lookup(unsigned int * mv_curve, unsigned char * charge_curve, unsigned int mv_in) {
  unsigned char charge = 0;

  if (mv_in > mv_curve[0]) {
    return batt_charge_curve[0];
  }

  while (*charge_curve > 0) {
    int vp1 = *mv_curve;
    int vp2 = *(mv_curve + 1);
    int chp1 = *charge_curve;
    int chp2 = *(charge_curve + 1);

    if (mv_in <= vp1 && mv_in > vp2) {
      float ratio = 1.0 * (chp2 - chp1) / (vp2 - vp1);
      float offset = chp1 - vp1 * ratio;

      charge = (unsigned int)(offset + ratio * mv_in);

      return charge;
    }

    charge_curve++;
    mv_curve++;
  }

  return charge;
}

unsigned int getBatteryVoltage() {
  float measuredvbat = analogRead(VBATPIN);

  // Measured by a voltage divider
  measuredvbat *= 2;

  // Reference voltage
  measuredvbat *= 3.3;

  // 10 bit ADC
  measuredvbat /= 1.024;

  return measuredvbat;
}

void setup(void) {
  Serial.begin(115200);

  sensorId = 0;

  radio.begin();

  radio.setPayloadSize(sizeof(SensorPacket));

  radio.setDataRate(RF24_250KBPS);

  radio.setCRCLength(RF24_CRC_16);

  radio.openWritingPipe(pipes[0]);

  radio.printDetails();

  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    sensorError = 1;
  }
}

uint8_t readBatteryCharge() {
  float voltage = getBatteryVoltage();
  uint8_t charge = charge_lookup(batt_mv_curve, batt_charge_curve, voltage);
}

void loop(void) {
  SensorPacket packet;

  packet.sensorId = sensorId;
  packet.status = sensorError;
  packet.localTime = int(micros() / 1000);
  packet.temperature = int(sensor.readTemperature() * 100);
  packet.battery = readBatteryCharge();
  packet.humidity = int(sensor.readHumidity() * 100);
  packet.airquality = 0;
  packet.pressure = 0;

  radio.powerUp();

  radio.write(&packet, sizeof(SensorPacket));

  radio.powerDown();

  Serial.print("Time: ");
  Serial.println(packet.localTime);

  Serial.print("Batt: ");
  Serial.println(packet.battery);
  
  Serial.print("Temp: ");
  Serial.println(packet.temperature);

  Serial.print("Hum: ");
  Serial.println(packet.humidity);

  Serial.println("---");

  delay(1000);
}
