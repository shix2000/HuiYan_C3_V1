
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define PINCH_PIN 3

Adafruit_MPU6050 mpu;

const uint32_t SAMPLE_MS = 20;
const uint32_t DEBOUNCE_MS = 25;

bool rawLast = HIGH;
bool stableState = HIGH;
uint32_t lastChange = 0;
uint32_t lastSample = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PINCH_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bool found = mpu.begin(0x68, &Wire);

  if (!found) {
    found = mpu.begin(0x69, &Wire);
  }

  if (!found) {
    Serial.println("MPU6050 INIT FAILED");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("HuiYan V0.3 READY");
  Serial.println("Press and hold to record gesture.");

  Serial.println("time_ms,ax,ay,az,gx,gy,gz");
}

void loop() {
  uint32_t now = millis();
  bool raw = digitalRead(PINCH_PIN);

  // Button debounce
  if (raw != rawLast) {
    lastChange = now;
  }

  if (now - lastChange >= DEBOUNCE_MS) {
    if (raw != stableState) {
      stableState = raw;

      if (stableState == LOW) {
        Serial.println("# PINCH_START");
        lastSample = now - SAMPLE_MS;
      } else {
        Serial.println("# PINCH_END");
      }
    }
  }

  rawLast = raw;

  // Only collect data during pinch
  if (stableState == LOW &&
      now - lastSample >= SAMPLE_MS) {

    lastSample = now;

    sensors_event_t a, g, temp;

    if (mpu.getEvent(&a, &g, &temp)) {
      Serial.printf(
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        (unsigned long)now,
        a.acceleration.x,
        a.acceleration.y,
        a.acceleration.z,
        g.gyro.x,
        g.gyro.y,
        g.gyro.z
      );
    }
  }
}
