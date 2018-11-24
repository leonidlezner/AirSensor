#ifndef __CONFIG_H__
#define __CONFIG_H__


#define PIN_CSN 11
#define PIN_CE  12
#define VBATPIN A7

#define CHARGE_CURVE_POINTS 7

const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

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

#endif
