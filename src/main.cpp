#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define SCL 47
#define SDA 21

void setup() {
  Serial.begin(115200);
  // 最多等 3 秒，避免无监视器时程序卡死
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  Wire.begin(SDA, SCL);  

  // 初始化 MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

#define THRESHOLD 0.3  // 变化阈值，单位 m/s²

float lastX = 0, lastY = 0, lastZ = 0;

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float x = a.acceleration.x;
  float y = a.acceleration.y;
  float z = a.acceleration.z;

  if (abs(x - lastX) > THRESHOLD ||
      abs(y - lastY) > THRESHOLD ||
      abs(z - lastZ) > THRESHOLD) {
    lastX = x; lastY = y; lastZ = z;
    Serial.print("AccelX:"); Serial.print(x);
    Serial.print(",AccelY:"); Serial.print(y);
    Serial.print(",AccelZ:"); Serial.println(z);
  }

  delay(10);
}
