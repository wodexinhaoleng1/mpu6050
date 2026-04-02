#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

#define SCL 47
#define SDA 21

// 调整此值来校准最大力度：轻摇约5，用力摇约15~25 (m/s²)
#define MAX_DELTA 20.0f

// EMA平滑系数 (0~1)：越大响应越快，越小越平滑
#define SMOOTH_FACTOR 0.4f

float lastX = 0, lastY = 0, lastZ = 0;
float smoothedIntensity = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000)
    delay(10);

  Serial.println("MPU6050 Shake Intensity (0-10)");

  Wire.begin(SDA, SCL);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // 使用 16G 量程，适合捕捉剧烈晃动
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float x = a.acceleration.x;
  float y = a.acceleration.y;
  float z = a.acceleration.z;

  // 计算三轴加速度变化量的向量模长
  float dx = x - lastX;
  float dy = y - lastY;
  float dz = z - lastZ;
  float delta = sqrtf(dx * dx + dy * dy + dz * dz);

  lastX = x;
  lastY = y;
  lastZ = z;

  // 映射到 0~10
  float rawIntensity = (delta / MAX_DELTA) * 10.0f;
  if (rawIntensity > 10.0f) rawIntensity = 10.0f;

  // 指数移动平均平滑，避免数值跳变
  smoothedIntensity = SMOOTH_FACTOR * rawIntensity + (1.0f - SMOOTH_FACTOR) * smoothedIntensity;

  // 四舍五入为整数 0~10
  int intensity = (int)(smoothedIntensity + 0.5f);

  Serial.println(intensity);  // 直接输出力度值，供外部读取

  delay(20);
}
