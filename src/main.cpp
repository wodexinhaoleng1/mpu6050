/**
 * main.cpp — LIS2DH12 摇动数据采集固件
 *
 * 以 50Hz 连续输出 CSV 数据，中断检测到的峰值/摇动事件标记在对应行。
 * 数据可直接粘贴到 Excel / Python 进行特征标注与分析。
 *
 * === 使用方式 ===
 *   检测到摇动 → 自动开始记录（打印 Session N START）
 *   串口发送任意字符 → 停止本次记录（打印 Session N END）
 *   再次摇动 → 自动开始下一条记录
 *
 * === CSV 列说明 ===
 *   session_id     — 记录编号，每次自动开始递增
 *   timestamp_ms   — 时间戳 (ms)
 *   raw_x/y/z_ms2  — 三轴原始加速度 (m/s²)
 *   dyn_x/y/z_ms2  — 去重力动态加速度 (m/s²)
 *   dyn_mag_ms2    — 动态加速度合矢量
 *   event          — 0=普通采样, 1=峰值检测, 2=摇动触发
 *   peak_ms2       — 中断时刻的主轴峰值 (m/s²)
 *   smoothed_ipi_ms — 平滑峰值间隔 (ms)，反映摇动速率
 *   recent_peaks   — 窗口内前置峰次数
 *   fast           — 1=快摇(IPI<250ms), 0=慢摇
 *   volume         — 摇动强度映射音量 (0-100，event=2 时有效)
 */

#include <Arduino.h>
#include <SparkFun_LIS2DH12.h>
#include <Wire.h>
#include <math.h>

// ── 引脚定义 ──────────────────────────────────────────────────────────────────
#define IMU_SCL      27
#define IMU_SDA      26
#define IMU_INT1_PIN 13

// ── 摇动检测参数 ──────────────────────────────────────────────────────────────
#define MAX_DYNAMIC_ACC  25.0f
#define PEAK_MIN          3.5f
#define SHAKE_COOLDOWN    500
#define MIN_COOLDOWN      150
#define IPI_ALPHA         0.4f
#define FAST_SHAKE_IPI    250
#define SHAKE_RATE_WINDOW 1500
#define MIN_PEAK_RATE     2
#define MIN_VOLUME        5
#define MAX_VOLUME        100
#define SHAKE_LEVELS      8

// ── 采样参数 ──────────────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS  20   // 50Hz，与 ODR 匹配

// ── 全局 IMU 状态 ─────────────────────────────────────────────────────────────
static SPARKFUN_LIS2DH12 imu;
static uint8_t            imuAddr      = 0;
static volatile bool      imuShakeFlag = false;

// ── 重力低通滤波（由 sampleAndLog 在 50Hz 稳定更新）────────────────────────────
static float gx_g = 0.0f, gy_g = 0.0f, gz_g = 9.8f;

// ── 摇动状态机 ────────────────────────────────────────────────────────────────
static unsigned long prevPeakTime    = 0;
static float         smoothedIPI     = (float)SHAKE_COOLDOWN;
static int           lastCountedSign = 0;
static const int     PEAK_HIST_SIZE  = 4;
static unsigned long peakHistory[PEAK_HIST_SIZE] = {0};
static int           peakHistIdx     = 0;
static unsigned long lastShakeTime   = 0;

// ── 待输出事件（processShakeInterrupt 写入，sampleAndLog 读取并清除）───────────
struct PendingEvent {
  int   type;          // 0=无, 1=峰值, 2=摇动触发
  float peak;          // 主轴峰值动态加速度 (m/s²)
  float smoothedIPI;   // 平滑峰值间隔 (ms)
  int   recentPeaks;   // 窗口内峰次
  bool  fast;          // 是否为快摇
  int   volume;        // 触发音量（仅 type==2 有效）
};
static PendingEvent pendingEvent = {0, 0.0f, (float)SHAKE_COOLDOWN, 0, false, 0};

// ── 采集会话控制 ──────────────────────────────────────────────────────────────
static bool recording  = false;  // 是否正在记录
static int  sessionNum = 0;      // 当前会话编号

// ── ISR ───────────────────────────────────────────────────────────────────────
static void IRAM_ATTR onImuInt1() {
  imuShakeFlag = true;
}

// ── 寄存器写入 ────────────────────────────────────────────────────────────────
static void imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ── IMU 初始化 ────────────────────────────────────────────────────────────────
static bool initIMU() {
  Wire.begin(IMU_SDA, IMU_SCL);

  uint8_t addr = 0;
  for (uint8_t a : {0x18, 0x19}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { addr = a; break; }
  }
  if (addr == 0 || !imu.begin(addr)) {
    Serial.println("# [IMU] LIS2DH12 not found (tried 0x18 & 0x19)");
    return false;
  }

  imu.setScale(LIS2DH12_8g);
  imuAddr = addr;

  imuWriteReg(0x20, 0x47);  // CTRL_REG1: ODR=50Hz, 正常模式, XYZ 全开
  imuWriteReg(0x21, 0x01);  // CTRL_REG2: HP_IA1=1，高通滤波去重力
  imuWriteReg(0x22, 0x40);  // CTRL_REG3: I1_IA1=1，IA1 路由到 INT1
  imuWriteReg(0x24, 0x08);  // CTRL_REG5: LIR_INT1=1，锁存中断
  imuWriteReg(0x30, 0x3F);  // INT1_CFG: 6轴 OR 模式
  imuWriteReg(0x32, 0x03);  // INT1_THS: ~187mg，低阈值及时唤醒
  imuWriteReg(0x33, 0x00);  // INT1_DURATION: 立即触发

  // 清除上电锁存中断
  Wire.beginTransmission(imuAddr);
  Wire.write(0x31);
  Wire.endTransmission();
  Wire.requestFrom((int)imuAddr, 1);
  if (Wire.available()) Wire.read();

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), onImuInt1, RISING);

  Serial.printf("# [IMU] LIS2DH12 OK @ 0x%02X, INT1 -> GPIO%d\n", addr, IMU_INT1_PIN);
  return true;
}

// ── 处理摇动中断：读取即时峰值，更新状态机，写入 pendingEvent ─────────────────
static void processShakeInterrupt() {
  if (!imuShakeFlag) return;
  imuShakeFlag = false;
  unsigned long now = millis();

  float x = imu.getX() * 9.80665f / 1000.0f;
  float y = imu.getY() * 9.80665f / 1000.0f;
  float z = imu.getZ() * 9.80665f / 1000.0f;

  // 释放 LIR 锁存（读 INT1_SRC）
  Wire.beginTransmission(imuAddr);
  Wire.write(0x31);
  Wire.endTransmission();
  Wire.requestFrom((int)imuAddr, 1);
  if (Wire.available()) Wire.read();

  // 使用 sampleAndLog 维护的重力估计计算动态加速度
  float dx = x - gx_g;
  float dy = y - gy_g;
  float dz = z - gz_g;

  float adx = fabsf(dx), ady = fabsf(dy), adz = fabsf(dz);
  float signal;
  if      (adx >= ady && adx >= adz) signal = dx;
  else if (ady >= adx && ady >= adz) signal = dy;
  else                                signal = dz;

  float peak    = fabsf(signal);
  int   curSign = (signal >= 0.0f) ? 1 : -1;

  if (peak < PEAK_MIN) return;

  bool dirChanged = (lastCountedSign == 0 || curSign != lastCountedSign);

  if (dirChanged) {
    if (prevPeakTime > 0) {
      float ipi = (float)(now - prevPeakTime);
      if (ipi < 2000.0f) {
        smoothedIPI = IPI_ALPHA * ipi + (1.0f - IPI_ALPHA) * smoothedIPI;
      } else {
        smoothedIPI = (float)SHAKE_COOLDOWN;
      }
    }
    prevPeakTime    = now;
    lastCountedSign = curSign;
    peakHistory[peakHistIdx] = now;
    peakHistIdx = (peakHistIdx + 1) % PEAK_HIST_SIZE;
  }

  unsigned long effectiveCooldown = (unsigned long)constrain(
    smoothedIPI * 0.75f, (float)MIN_COOLDOWN, (float)SHAKE_COOLDOWN);

  int recentCount = 0;
  for (int i = 0; i < PEAK_HIST_SIZE; i++) {
    if (peakHistory[i] > 0 &&
        (now - peakHistory[i]) < (unsigned long)SHAKE_RATE_WINDOW &&
        peakHistory[i] != now) {
      recentCount++;
    }
  }

  PendingEvent ev;
  ev.peak       = peak;
  ev.smoothedIPI = smoothedIPI;
  ev.recentPeaks = recentCount;
  ev.fast       = false;
  ev.volume     = 0;
  ev.type       = 1;  // 默认标记为 PEAK

  if (dirChanged && recentCount >= MIN_PEAK_RATE &&
      (now - lastShakeTime) > effectiveCooldown) {
    lastShakeTime = now;
    float t = (peak - PEAK_MIN) / (MAX_DYNAMIC_ACC - PEAK_MIN);
    if (t > 1.0f) t = 1.0f;
    int step = (int)(t * (float)SHAKE_LEVELS);
    if (step >= SHAKE_LEVELS) step = SHAKE_LEVELS - 1;
    ev.volume = MIN_VOLUME + step * (MAX_VOLUME - MIN_VOLUME) / (SHAKE_LEVELS - 1);
    ev.fast   = (smoothedIPI < (float)FAST_SHAKE_IPI);
    ev.type   = 2;  // SHAKE 触发
  }

  // 优先保留更高级别的事件（SHAKE > PEAK）
  if (ev.type >= pendingEvent.type) {
    pendingEvent = ev;
  }
}

// ── 50Hz 连续采样并输出 CSV ───────────────────────────────────────────────────
static void sampleAndLog() {
  static unsigned long lastSampleTime = 0;
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  float x = imu.getX() * 9.80665f / 1000.0f;
  float y = imu.getY() * 9.80665f / 1000.0f;
  float z = imu.getZ() * 9.80665f / 1000.0f;

  // 更新重力低通滤波（截止 ~0.5Hz，时间常数 ~0.65s）
  const float GRAV_ALPHA = 0.97f;
  gx_g = GRAV_ALPHA * gx_g + (1.0f - GRAV_ALPHA) * x;
  gy_g = GRAV_ALPHA * gy_g + (1.0f - GRAV_ALPHA) * y;
  gz_g = GRAV_ALPHA * gz_g + (1.0f - GRAV_ALPHA) * z;

  float dx = x - gx_g;
  float dy = y - gy_g;
  float dz = z - gz_g;
  float dyn_mag = sqrtf(dx*dx + dy*dy + dz*dz);

  // 取出待输出事件并清除
  PendingEvent ev = pendingEvent;
  pendingEvent.type = 0;

  if (!recording) return;

  Serial.printf("%d,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.1f,%d,%d,%d\n",
                sessionNum,
                now,
                x, y, z,
                dx, dy, dz,
                dyn_mag,
                ev.type,
                ev.peak,
                ev.smoothedIPI,
                ev.recentPeaks,
                (int)ev.fast,
                ev.volume);
}

// ── Enter 键开始/结束记录 ─────────────────────────────────────────────────────
static void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();  // 清空缓冲
  if (c != '\r' && c != '\n') return;       // 只响应 Enter

  if (!recording) {
    recording = true;
    sessionNum++;
    Serial.printf("# === Session %d START ===\n", sessionNum);
    Serial.println("session_id,timestamp_ms,raw_x_ms2,raw_y_ms2,raw_z_ms2,dyn_x_ms2,dyn_y_ms2,dyn_z_ms2,dyn_mag_ms2,event,peak_ms2,smoothed_ipi_ms,recent_peaks,fast,volume");
  } else {
    recording = false;
    Serial.printf("# === Session %d END ===\n", sessionNum);
    Serial.println("# 按 Enter 开始下一条记录");
  }
}

// ── Arduino 入口 ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# LIS2DH12 Shake Data Collector");
  Serial.println("# 按 Enter 开始记录，摇动设备，再按 Enter 结束本条记录");
  Serial.println("# event: 0=normal  1=peak_detected  2=shake_triggered");

  if (!initIMU()) {
    Serial.println("# [FATAL] IMU init failed, halting.");
    while (true) { delay(1000); }
  }

  Serial.println("# Ready. 按 Enter 键开始第一条记录.");
}

void loop() {
  handleSerial();
  processShakeInterrupt();
  sampleAndLog();
}
