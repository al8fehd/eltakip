// ============================================================================
//  deneyap_hand_tracker_onboard.ino — 1 onboard + 10 external IMUs
//
//  Board   : Deneyap Kart v2 (ESP32-S3-WROOM-1-N8R8, Arduino framework)
//  Sensors : onboard LSM6DSM -> IMU_0; 10x MPU6050 behind 2x TCA9548A
//              Mux1 @0x70, ch0..7  -> IMU_1..IMU_8
//              Mux2 @0x71, ch0..1  -> IMU_9..IMU_10
//  Fusion  : Madgwick (IMU-only, accel+gyro) per IMU -> quaternion orientation
//  Link    : USB serial 921600 baud, binary frames, CRC-16/CCITT-FALSE
//
//  Frame layout (multi-byte values little-endian):
//    0xA5 0x5A | type(1) | seq(1) | len(1) | payload[len] | crc_lo(1) | crc_hi(1)
//    crc16 covers [type, seq, len, payload...], init 0xFFFF, poly 0x1021.
//    type 0x01 pose : flags(1) | status_lo(1) | status_hi(1) | 11x quaternion
//                     quaternion = w,x,y,z as int16 Q15 (x/32767 -> float)
//                     index 0 = IMU_0 hand reference, 1..10 = fingers IMU_1..10
//    type 0x02 log  : ASCII text (calibration report, runtime status)
//    flags bit0 = calibration complete (pose stream valid)
//    flags bit1 = TX overflow occurred at least once
//    status_lo bit i / status_hi bit (i-8) set <=> IMU_i healthy
//
//  IMU mapping (mirrored in pc_app/pose.py):
//    IMU_0   hand dorsum (global orientation reference)   onboard LSM6DSM
//    IMU_1   thumb lower      IMU_2  thumb upper          Mux1 ch0..1
//    IMU_3   index lower      IMU_4  index upper          Mux1 ch2..3
//    IMU_5   middle lower     IMU_6  middle upper         Mux1 ch4..5
//    IMU_7   ring lower       IMU_8  ring upper           Mux1 ch6..7
//    IMU_9   pinky lower      IMU_10 pinky upper          Mux2 ch0..1
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

struct SensorNode;

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Official Deneyap Kart v2 pinout (variants/deneyapkartv2/pins_arduino.h):
//   SDA = GPIO47 (header pin D10), SCL = GPIO21 (header pin D11).
static const uint8_t  PIN_SDA       = 47;
static const uint8_t  PIN_SCL       = 21;
static const uint32_t I2C_CLK_HZ    = 400000UL;   // drop to 100000UL if wiring is long/noisy
static const uint32_t SERIAL_BAUD   = 921600UL;

static const uint32_t STREAM_PERIOD_MS = 10;      // 100 Hz target (50-100 Hz stream rate)
static const uint16_t CALIB_SAMPLES    = 256;     // resting samples per IMU at boot
static const float    MADGWICK_BETA    = 0.10f;   // fusion gain: 0.05 calmer, 0.2 snappier
static const float    GYRO_DEADZONE_RAD = 0.022f;
static const uint8_t  ZUPT_STILL_TICKS  = 40;

// MPU6050 ranges: accel +/-4g (8192 LSB/g), gyro +/-2000 deg/s (16.4 LSB/(deg/s))
static const uint8_t  MPU_I2C_ADDR = 0x68;        // AD0 tied low on every module
static const uint8_t  ACCEL_CFG    = 0x08;        // +/-4g
static const uint8_t  GYRO_CFG     = 0x18;        // +/-2000 deg/s
static const float    ACC_STEP     = 1.0f / 8192.0f;
static const float    GYR_STEP_RAD = (PI / 180.0f) / 16.4f;
static const uint8_t  IMU_COUNT      = 11;
static const uint8_t  EXT_IMU_COUNT  = 10;
static const uint8_t  MAX_FAIL_STREAK = 5;        // consecutive I2C failures before flagging IMU bad
static const float    LSM_ACC_STEP_G = 0.000122f; // +/-4g, 0.122 mg/LSB
static const float    LSM_GYR_STEP_RAD = (PI / 180.0f) * 0.070f; // +/-2000 dps

// TCA9548A addresses
static const uint8_t  MUX1_ADDR = 0x70;           // A0,A1,A2 -> GND
static const uint8_t  MUX2_ADDR = 0x71;           // A0 -> 3V3 (HIGH), A1,A2 -> GND

// ----------------------------------------------------------------------------
// CRC-16 / CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
// Mirrored in pc_app/protocol.py — keep both sides in sync.
// ----------------------------------------------------------------------------
static uint16_t crc16_ccitt(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ----------------------------------------------------------------------------
// Madgwick AHRS filter — IMU variant (accel + gyro only), float32.
// Standard implementation from S. O. H. Madgwick, "An efficient orientation
// filter for inertial and inertial/magnetic sensor arrays" (2010), section
// 3.3 with the magnetometer terms removed (MPU6050 has no magnetometer).
// ----------------------------------------------------------------------------
class MadgwickFilter {
 public:
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // (w, x, y, z), sensor <- world

  void updateAG(float gx, float gy, float gz,   // gyro, rad/s
                float ax, float ay, float az,   // accel, g (direction is enough)
                float dt, float beta) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Rate of change of quaternion from gyroscope.
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer measurement (gravity direction).
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return;
    float recipNorm = 1.0f / n;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic.
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    // Gradient-descent corrective step (accelerometer only).
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q0 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
             + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
             + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // Apply feedback, integrate gyro contribution, normalise.
    qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm; q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm; q[3] = q3 * recipNorm;
  }
};

// ----------------------------------------------------------------------------
// TCA9548A — 8-channel I2C multiplexer.
// Select writes the control register (1 << ch); verifyChannel reads it back,
// which proves the mux is present and the channel latched.
// ----------------------------------------------------------------------------
class Tca9548a {
 public:
  explicit Tca9548a(uint8_t addr) : addr_(addr) {}

  bool select(uint8_t ch) {
    Wire.beginTransmission(addr_);
    Wire.write((uint8_t)(1u << ch));
    return Wire.endTransmission() == 0;
  }

  void deselect() {
    Wire.beginTransmission(addr_);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
  }

  bool verifyChannel(uint8_t ch) {
    if (Wire.requestFrom(addr_, (uint8_t)1) < 1) return false;
    return Wire.read() == (int)(1u << ch);
  }

 private:
  uint8_t addr_;
};

// ----------------------------------------------------------------------------
// MPU6050 driver (I2C). Kept as a small standalone class so another IMU
// (BNO055, MPU9250, ...) can be swapped in with the same begin()/readRaw()
// contract; everything above this layer is IMU-agnostic.
// ----------------------------------------------------------------------------
class Mpu6050 {
 public:
  bool begin() {
    if (!writeReg(0x6B, 0x80)) return false;   // device reset
    pollWaitMs(50);                            // reset takes ~30 ms
    if (!writeReg(0x6B, 0x01)) return false;   // wake, auto clock (PLL X gyro)
    if (!writeReg(0x1A, 0x02)) return false;   // DLPF: gyro 98 Hz / accel 94 Hz
    if (!writeReg(0x1B, GYRO_CFG)) return false;
    if (!writeReg(0x1C, ACCEL_CFG)) return false;
    if (!writeReg(0x38, 0x00)) return false;   // no interrupts
    uint8_t who = 0;
    return readReg(0x75, who) && who == 0x68;  // WHO_AM_I
  }

  // Raw accel (LSB, +/-4g) and gyro (LSB, +/-2000 deg/s) in one burst read.
  bool readRaw(int16_t acc[3], int16_t gyr[3]) {
    uint8_t buf[14];
    if (!readRegs(0x3B, buf, sizeof(buf))) return false;
    for (uint8_t i = 0; i < 3; i++) {
      acc[i] = (int16_t)(((uint16_t)buf[i * 2] << 8) | buf[i * 2 + 1]);
      gyr[i] = (int16_t)(((uint16_t)buf[8 + i * 2] << 8) | buf[8 + i * 2 + 1]);
    }
    return true;
  }

 private:
  bool readReg(uint8_t reg, uint8_t& out) { return readRegs(reg, &out, 1); }

  bool readRegs(uint8_t reg, uint8_t* out, uint8_t n) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(MPU_I2C_ADDR, n) < n) return false;
    for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
    return true;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  static void pollWaitMs(uint32_t ms) {
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < ms) { }
  }
};

// ----------------------------------------------------------------------------
// LSM6DSM onboard IMU, accessed directly on the main I2C bus.
// ----------------------------------------------------------------------------
class Lsm6dsm {
 public:
  bool begin() {
    const uint8_t addresses[] = {0x6B, 0x6A};
    for (uint8_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
      address_ = addresses[i];
      uint8_t who = 0;
      if (readReg(0x0F, who) && who == 0x6A) {
        return writeReg(0x12, 0x44) &&  // BDU + register auto-increment
               writeReg(0x10, 0x48) &&  // 104 Hz, +/-4g
               writeReg(0x11, 0x4C);    // 104 Hz, +/-2000 dps
      }
    }
    return false;
  }

  bool readRaw(int16_t acc[3], int16_t gyr[3]) {
    uint8_t buf[6];
    if (!readRegs(0x22, buf, sizeof(buf))) return false;
    for (uint8_t i = 0; i < 3; i++)
      gyr[i] = (int16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);

    if (!readRegs(0x28, buf, sizeof(buf))) return false;
    for (uint8_t i = 0; i < 3; i++)
      acc[i] = (int16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);
    return true;
  }

 private:
  uint8_t address_ = 0;

  bool readReg(uint8_t reg, uint8_t& out) { return readRegs(reg, &out, 1); }

  bool readRegs(uint8_t reg, uint8_t* out, uint8_t n) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(address_, n) < n) return false;
    for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
    return true;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
};

// ----------------------------------------------------------------------------
// Sensor node: one IMU on one mux channel, with its filter and calibration.
// ----------------------------------------------------------------------------
struct SensorNode {
  Tca9548a*     mux;                       // mux this IMU hangs off
  uint8_t       ch;                        // mux channel
  Mpu6050       imu;
  MadgwickFilter filt;

  bool     present = false;                // WHO_AM_I confirmed at boot
  bool     ok      = false;                // healthy right now (status bit source)
  uint8_t  failStreak = 0;

  int16_t  gyrBias[3] = {0, 0, 0};         // raw LSB offsets
  int16_t  accBias[3] = {0, 0, 0};         // raw LSB offsets (includes gravity, see calib)
  float    gyrBiasF[3] = {0.0f, 0.0f, 0.0f}; // adaptive bias in rad/s
  uint8_t  quietTicks = 0;

  float    q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

static Tca9548a mux1(MUX1_ADDR);
static Tca9548a mux2(MUX2_ADDR);

static void deselectMuxes() {
  mux1.deselect();
  mux2.deselect();
}

// IMU_1..IMU_10 -> (mux, channel); IMU_0 is the onboard LSM6DSM.
static SensorNode nodes[EXT_IMU_COUNT] = {
  {&mux1, 0}, {&mux1, 1}, {&mux1, 2}, {&mux1, 3},
  {&mux1, 4}, {&mux1, 5}, {&mux1, 6}, {&mux1, 7},
  {&mux2, 0}, {&mux2, 1},
};
static Lsm6dsm onboardImu;
static MadgwickFilter onboardFilt;
static int16_t onboardGyrBias[3] = {0, 0, 0};
static float onboardGyrBiasF[3] = {0.0f, 0.0f, 0.0f};
static uint8_t onboardQuietTicks = 0;
static float onboardQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static bool onboardPresent = false, onboardOk = false;
static uint8_t onboardFailStreak = 0;

// ----------------------------------------------------------------------------
// TX ring buffer — decouples frame generation from the USB serial drain so a
// slow or absent host never stalls the sensor loop. Drops are counted (and
// reported to the host via the flags byte).
// ----------------------------------------------------------------------------
class TxRing {
 public:
  static constexpr size_t CAP = 2048;

  bool push(const uint8_t* p, size_t n) {
    if (n > CAP - pending_) { dropped_ += n; return false; }
    while (n--) {
      buf_[tail_] = *p++;
      tail_ = (tail_ + 1) % CAP;
      pending_++;
    }
    return true;
  }

  // Write as much as the driver accepts in one call; never blocks.
  void drain(Stream& ser) {
    size_t free = ser.availableForWrite();
    while (pending_ && free) {
      size_t k = pending_ < free ? pending_ : free;
      size_t maxk = CAP - head_;              // ring wrap: keep the chunk contiguous
      if (k > maxk) k = maxk;
      size_t w = ser.write(&buf_[head_], k);
      if (w == 0) break;
      head_ += w;
      if (head_ == CAP) head_ = 0;
      pending_ -= w;
      free = (free > w) ? free - w : 0;
    }
  }

  size_t pending() const { return pending_; }
  size_t dropped() const { return dropped_; }

 private:
  uint8_t buf_[CAP];
  size_t  head_ = 0, tail_ = 0, pending_ = 0, dropped_ = 0;
};

// ----------------------------------------------------------------------------
// Wire-protocol encoder. Frames are built on the stack, then queued.
// ----------------------------------------------------------------------------
class FrameWriter {
 public:
  void beginFrame(uint8_t type) {
    len_ = 0;
    buf_[0] = 0xA5; buf_[1] = 0x5A;
    buf_[2] = type;
    buf_[3] = seq_++;
  }

  void addByte(uint8_t b) { buf_[5 + len_++] = b; }

  void addBytes(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) buf_[5 + len_ + i] = p[i];
    len_ += n;
  }

  bool ship(TxRing& ring) {
    buf_[4] = (uint8_t)len_;
    uint16_t crc = crc16_ccitt(&buf_[2], (size_t)len_ + 3);   // type..payload
    buf_[5 + len_]     = (uint8_t)(crc & 0xFF);
    buf_[5 + len_ + 1] = (uint8_t)(crc >> 8);
    return ring.push(buf_, (size_t)5 + len_ + 2);
  }

 private:
  uint8_t buf_[253];      // 5 header + 247 payload + 2 crc (len fits in 1 byte)
  uint8_t len_ = 0;
  uint8_t seq_ = 0;
};

// Quaternion -> int16 Q15, little-endian, clamped. Server divides by 32767.
static void packQ15(const float q[4], uint8_t out[8]) {
  for (uint8_t i = 0; i < 4; i++) {
    float v = q[i] * 32767.0f;
    if (v > 32767.0f) v = 32767.0f;
    else if (v < -32767.0f) v = -32767.0f;
    int16_t s = (int16_t)lrintf(v);
    out[i * 2]     = (uint8_t)(s & 0xFF);
    out[i * 2 + 1] = (uint8_t)((uint16_t)s >> 8);
  }
}

static TxRing tx;

// Human-readable diagnostics as type-0x02 log frames (safe in a terminal).
static void sendLog(const char* text) {
  FrameWriter fw;
  fw.beginFrame(0x02);
  fw.addBytes((const uint8_t*)text, strlen(text));
  fw.ship(tx);
}

// ----------------------------------------------------------------------------
// Startup resting calibration: CALIB_SAMPLES taken per IMU while the sensor
// reports a plausibly still signal.
//  - gyro bias : mean raw rate (should be ~0), subtracted before fusion
//  - accel mean: recorded for diagnostics; never subtracted because the
//    fusion filter needs the measured 1g gravity direction.
// ----------------------------------------------------------------------------
static bool calibrateNode(SensorNode& node) {
  if (!node.mux->select(node.ch)) return false;

  int32_t gs[3] = {0, 0, 0}, as[3] = {0, 0, 0};
  uint16_t took = 0;
  int16_t acc[3], gyr[3];

  for (uint16_t tries = 0; tries < 4096 && took < CALIB_SAMPLES; tries++) {
    delay(5);
    if (!node.imu.readRaw(acc, gyr)) continue;         // will fail -> node marked bad
    if (gyr[0] < -250 || gyr[0] > 250 ||
        gyr[1] < -250 || gyr[1] > 250 ||
        gyr[2] < -250 || gyr[2] > 250) continue;
    // Stillness gate: |a| within +/-25% of 1 g (0.56g^2 .. 1.56g^2 in LSB^2).
    int64_t a2 = (int64_t)acc[0] * acc[0] + (int64_t)acc[1] * acc[1] + (int64_t)acc[2] * acc[2];
    if (a2 < 37100000LL || a2 > 105000000LL) continue;
    for (uint8_t i = 0; i < 3; i++) { gs[i] += gyr[i]; as[i] += acc[i]; }
    took++;
  }

  bool ok = (took >= CALIB_SAMPLES);
  if (ok) {
    for (uint8_t i = 0; i < 3; i++) {
      node.gyrBias[i] = (int16_t)(gs[i] / (int32_t)took);
      node.accBias[i] = (int16_t)(as[i] / (int32_t)took);
      node.gyrBiasF[i] = node.gyrBias[i] * GYR_STEP_RAD;
    }
  }
  node.mux->deselect();
  return ok;
}

static bool calibrateOnboard() {
  deselectMuxes();
  int32_t gs[3] = {0, 0, 0};
  uint16_t took = 0;
  int16_t acc[3], gyr[3];

  for (uint16_t tries = 0; tries < 4096 && took < CALIB_SAMPLES; tries++) {
    delay(10);
    deselectMuxes();
    if (!onboardImu.readRaw(acc, gyr)) continue;
    if (gyr[0] < -250 || gyr[0] > 250 ||
        gyr[1] < -250 || gyr[1] > 250 ||
        gyr[2] < -250 || gyr[2] > 250) continue;
    float ax = acc[0] * LSM_ACC_STEP_G;
    float ay = acc[1] * LSM_ACC_STEP_G;
    float az = acc[2] * LSM_ACC_STEP_G;
    float a2 = ax * ax + ay * ay + az * az;
    if (a2 < 0.56f || a2 > 1.56f) continue;
    for (uint8_t i = 0; i < 3; i++) gs[i] += gyr[i];
    took++;
  }

  if (took < CALIB_SAMPLES) return false;
  for (uint8_t i = 0; i < 3; i++) {
    onboardGyrBias[i] = (int16_t)(gs[i] / (int32_t)took);
    onboardGyrBiasF[i] = onboardGyrBias[i] * LSM_GYR_STEP_RAD;
  }
  return true;
}

static bool     calibDone_ = false;
static uint32_t frameCount_ = 0;

// ----------------------------------------------------------------------------
// Boot: 1) serial + I2C, 2) probe both muxes, 3) init + calibrate all IMUs,
// 4) report. All log output goes out as framed messages, never raw text.
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);                                  // let USB plug-in vibrations settle before calibration

  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLK_HZ);

  bool mux1ok = mux1.select(0) && mux1.verifyChannel(0);
  bool mux2ok = mux2.select(0) && mux2.verifyChannel(0);
  mux1.deselect();
  mux2.deselect();

  if (!mux1ok) sendLog("MUX1 (0x70) NOT FOUND - check wiring");
  if (!mux2ok) sendLog("MUX2 (0x71) NOT FOUND - check A0/address pin");

  uint8_t good = 0;
  deselectMuxes();
  onboardPresent = onboardImu.begin();
  if (onboardPresent && calibrateOnboard()) {
    onboardOk = true;
    good++;
    char line[64];
    snprintf(line, sizeof(line), "CAL IMU_0 ok  gyb=%d,%d,%d",
             onboardGyrBias[0], onboardGyrBias[1], onboardGyrBias[2]);
    sendLog(line);
  } else {
    onboardPresent = false;
    sendLog("CAL IMU_0 FAILED (onboard LSM6DSM)");
  }

  for (uint8_t i = 0; i < EXT_IMU_COUNT; i++) {
    SensorNode& n = nodes[i];
    bool muok = (n.mux == &mux1) ? mux1ok : mux2ok;
    bool initOk = false;

    if (muok && n.mux->select(n.ch)) {
      initOk = n.imu.begin();
      n.present = initOk;
      n.mux->deselect();
    }

    if (initOk && calibrateNode(n)) {
      n.ok = true;
      good++;
      char line[96];
      snprintf(line, sizeof(line),
               "CAL IMU_%u ok  gyb=%d,%d,%d  ab=%d,%d,%d",
               i + 1, n.gyrBias[0], n.gyrBias[1], n.gyrBias[2],
               n.accBias[0], n.accBias[1], n.accBias[2]);
      sendLog(line);
    } else {
      n.present = false;
      n.ok = false;
      char line[64];
      snprintf(line, sizeof(line), "CAL IMU_%u FAILED (mux%u ch%u)",
               i + 1, (n.mux == &mux1) ? 1u : 2u, n.ch);
      sendLog(line);
    }
  }

  char line[64];
  snprintf(line, sizeof(line), "CALIB_DONE good=%u/%u", good, IMU_COUNT);
  sendLog(line);
  calibDone_ = true;
}

// ----------------------------------------------------------------------------
// One scheduled tick (~10 ms): poll all IMUs through the muxes, fuse each,
// stream one pose frame. Never blocks: no delay() inside.
// ----------------------------------------------------------------------------
static uint32_t lastTickUs_   = 0;
static uint32_t lastTickMs_   = 0;
static uint32_t lastRateMs_   = 0;
static uint32_t rateFrames_   = 0;

static void streamTick() {
  uint32_t nowUs = micros();
  uint32_t nowMs = (uint32_t)(nowUs / 1000);

  float dt = (float)(nowUs - lastTickUs_) / 1e6f;
  if (dt > 0.05f) dt = 0.05f;
  else if (dt < 0.002f) dt = 0.002f;
  lastTickUs_ = nowUs;

  uint8_t statusLo = 0, statusHi = 0;
  int16_t acc[3], gyr[3];

  if (onboardPresent) {
    deselectMuxes();
    if (!onboardImu.readRaw(acc, gyr)) {
      onboardQuietTicks = 0;
      if (++onboardFailStreak >= MAX_FAIL_STREAK) {
        onboardFailStreak = 0;
        if (onboardOk) {
          onboardOk = false;
          sendLog("IMU_0 FAILED");
        }
      }
    } else {
      onboardFailStreak = 0;
      if (!onboardOk) {
        onboardOk = true;
        sendLog("IMU_0 OK");
      }
      statusLo |= 0x01;
      float ax = acc[0] * LSM_ACC_STEP_G;
      float ay = acc[1] * LSM_ACC_STEP_G;
      float az = acc[2] * LSM_ACC_STEP_G;
      float rawGx = gyr[0] * LSM_GYR_STEP_RAD;
      float rawGy = gyr[1] * LSM_GYR_STEP_RAD;
      float rawGz = gyr[2] * LSM_GYR_STEP_RAD;
      float gx = rawGx - onboardGyrBiasF[0];
      float gy = rawGy - onboardGyrBiasF[1];
      float gz = rawGz - onboardGyrBiasF[2];
      float aMag = sqrtf(ax * ax + ay * ay + az * az);
      if (fabsf(gx) < GYRO_DEADZONE_RAD && fabsf(gy) < GYRO_DEADZONE_RAD &&
          fabsf(gz) < GYRO_DEADZONE_RAD && aMag >= 0.88f && aMag <= 1.12f) {
        if (onboardQuietTicks <= ZUPT_STILL_TICKS) onboardQuietTicks++;
        if (onboardQuietTicks > ZUPT_STILL_TICKS) {
          onboardGyrBiasF[0] = 0.995f * onboardGyrBiasF[0] + 0.005f * rawGx;
          onboardGyrBiasF[1] = 0.995f * onboardGyrBiasF[1] + 0.005f * rawGy;
          onboardGyrBiasF[2] = 0.995f * onboardGyrBiasF[2] + 0.005f * rawGz;
          gx = rawGx - onboardGyrBiasF[0];
          gy = rawGy - onboardGyrBiasF[1];
          gz = rawGz - onboardGyrBiasF[2];
        }
      } else {
        onboardQuietTicks = 0;
      }
      if (fabsf(gx) < GYRO_DEADZONE_RAD) gx = 0.0f;
      if (fabsf(gy) < GYRO_DEADZONE_RAD) gy = 0.0f;
      if (fabsf(gz) < GYRO_DEADZONE_RAD) gz = 0.0f;
      onboardFilt.updateAG(gx, gy, gz, ax, ay, az, dt, MADGWICK_BETA);
      for (uint8_t k = 0; k < 4; k++) onboardQ[k] = onboardFilt.q[k];
    }
  }

  for (uint8_t i = 0; i < EXT_IMU_COUNT; i++) {
    SensorNode& n = nodes[i];
    if (!n.present) continue;

    if (!n.mux->select(n.ch) || !n.imu.readRaw(acc, gyr)) {
      n.mux->deselect();
      n.quietTicks = 0;
      if (++n.failStreak >= MAX_FAIL_STREAK) {
        n.failStreak = 0;
        if (n.ok) {
          n.ok = false;
          char l[48];
          snprintf(l, sizeof(l), "IMU_%u FAILED", i + 1);
          sendLog(l);
        }
      }
      continue;
    }
    n.mux->deselect();

    n.failStreak = 0;
    if (!n.ok) {
      n.ok = true;
      char l[48];
      snprintf(l, sizeof(l), "IMU_%u OK", i + 1);
      sendLog(l);
    }
    uint8_t wireIndex = i + 1;
    if (wireIndex < 8) statusLo |= (uint8_t)(1u << wireIndex);
    else               statusHi |= (uint8_t)(1u << (wireIndex - 8));

    // Bias-corrected, scaled units: accel in g, gyro in rad/s.
    float ax = acc[0] * ACC_STEP;
    float ay = acc[1] * ACC_STEP;
    float az = acc[2] * ACC_STEP;
    float rawGx = gyr[0] * GYR_STEP_RAD;
    float rawGy = gyr[1] * GYR_STEP_RAD;
    float rawGz = gyr[2] * GYR_STEP_RAD;
    float gx = rawGx - n.gyrBiasF[0];
    float gy = rawGy - n.gyrBiasF[1];
    float gz = rawGz - n.gyrBiasF[2];
    float aMag = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(gx) < GYRO_DEADZONE_RAD && fabsf(gy) < GYRO_DEADZONE_RAD &&
        fabsf(gz) < GYRO_DEADZONE_RAD && aMag >= 0.88f && aMag <= 1.12f) {
      if (n.quietTicks <= ZUPT_STILL_TICKS) n.quietTicks++;
      if (n.quietTicks > ZUPT_STILL_TICKS) {
        n.gyrBiasF[0] = 0.995f * n.gyrBiasF[0] + 0.005f * rawGx;
        n.gyrBiasF[1] = 0.995f * n.gyrBiasF[1] + 0.005f * rawGy;
        n.gyrBiasF[2] = 0.995f * n.gyrBiasF[2] + 0.005f * rawGz;
        gx = rawGx - n.gyrBiasF[0];
        gy = rawGy - n.gyrBiasF[1];
        gz = rawGz - n.gyrBiasF[2];
      }
    } else {
      n.quietTicks = 0;
    }
    if (fabsf(gx) < GYRO_DEADZONE_RAD) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADZONE_RAD) gy = 0.0f;
    if (fabsf(gz) < GYRO_DEADZONE_RAD) gz = 0.0f;

    n.filt.updateAG(gx, gy, gz, ax, ay, az, dt, MADGWICK_BETA);
    for (uint8_t k = 0; k < 4; k++) n.q[k] = n.filt.q[k];
  }

  // Assemble + queue the pose frame.
  FrameWriter fw;
  fw.beginFrame(0x01);
  fw.addByte((uint8_t)(0x01 | (tx.dropped() ? 0x02 : 0x00)));
  fw.addByte(statusLo);
  fw.addByte(statusHi);
  uint8_t out[8];
  packQ15(onboardQ, out);
  fw.addBytes(out, 8);
  for (uint8_t i = 0; i < EXT_IMU_COUNT; i++) {
    packQ15(nodes[i].q, out);
    fw.addBytes(out, 8);
  }
  fw.ship(tx);
  frameCount_++;

  // Periodic stream-rate report (log frame, readable in a terminal).
  if (lastRateMs_ != 0 && (nowMs - lastRateMs_) >= 5000) {
    uint32_t span = nowMs - lastRateMs_;
    float hz = (float)(frameCount_ - rateFrames_) * 1000.0f / (float)span;
    char line[72];
    snprintf(line, sizeof(line), "RATE %.1f Hz  frames=%lu  txdrop=%lu  healthy=%u",
             hz, (unsigned long)frameCount_, (unsigned long)tx.dropped(),
             (unsigned)((int)__builtin_popcount(statusLo) + __builtin_popcount(statusHi)));
    sendLog(line);
    rateFrames_ = frameCount_;
    lastRateMs_ = nowMs;
  } else if (lastRateMs_ == 0) {
    lastRateMs_ = nowMs;
  }
}

// ----------------------------------------------------------------------------
// Steady state: drain TX whenever there is room, then run the next scheduled
// tick. Everything is timer-driven; neither loop path ever blocks.
// ----------------------------------------------------------------------------
void loop() {
  tx.drain(Serial);

  if (!calibDone_) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastTickMs_) >= STREAM_PERIOD_MS) {
    lastTickMs_ = now;
    streamTick();
  }
}


