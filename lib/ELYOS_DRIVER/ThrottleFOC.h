#pragma once
#include <Arduino.h>
#include <math.h>

class ThrottleFOC {
public:
  struct Config {
    // ---------- ADC / pedal ----------
    uint8_t pedalPin = A0;
    uint16_t adcMin = 200;          // calibrate!
    uint16_t adcMax = 3890;         // calibrate!
    float deadband = 0.02f;         // 2% pedal dead zone
    float faultLowMargin = 20.0f;   // ADC counts below adcMin tolerated
    float faultHighMargin = 20.0f;  // ADC counts above adcMax tolerated

    // ---------- Filtering ----------
    // pedal LPF time constant in seconds
    float pedalFilterTau = 0.030f;  // 30 ms

    // ---------- Pedal shaping ----------
    // iq_norm = blendLinear * p + (1-blendLinear) * p^2
    float blendLinear = 0.30f;

    // ---------- Current limits ----------
    float iqMax = 30.0f;            // absolute allowed q-axis current [A]
    float iqLaunchMax = 12.0f;      // lower current cap near zero speed [A]
    float launchSpeedRadPerSec = 8.0f; // below this, use launch limit

    // Optional speed-based derating
    float speedForFullIq = 40.0f;   // rad/s where full iqMax becomes available

    // ---------- Rate limiting ----------
    float iqRampUp = 80.0f;         // A/s, slower for efficiency
    float iqRampDown = 180.0f;      // A/s, faster for safety/natural release

    // ---------- Optional creep / stiction ----------
    bool enableMinStartCurrent = false;
    float minStartPedal = 0.08f;    // if pedal exceeds this, optionally inject small current
    float minStartCurrent = 2.0f;   // enough only to overcome stiction if needed

    // ---------- Safety behavior ----------
    bool cutToZeroOnFault = true;
  };

  explicit ThrottleFOC(const Config& cfg) : cfg_(cfg) {}

  void begin() {
    analogReadResolution(12);  // Teensy 4.1 supports up to 12-bit ADC read setting
    pinMode(cfg_.pedalPin, INPUT);
    lastMicros_ = micros();
    pedalFilt_ = 0.0f;
    iqRef_ = 0.0f;
    fault_ = false;
  }

  // motorSpeedRadPerSec: use motor.shaftVelocity() or your own filtered vehicle/motor speed
  float update(float motorSpeedRadPerSec) {
    const uint32_t now = micros();
    float dt = (now - lastMicros_) * 1e-6f;
    lastMicros_ = now;

    // protect against dt glitches
    if (dt <= 0.0f || dt > 0.1f) {
      dt = 0.001f; // default 1 ms
    }

    // 1) Read pedal ADC
    uint16_t adc = analogRead(cfg_.pedalPin);

    // 2) Range/fault check
    fault_ = isAdcFault(adc);

    // 3) Normalize raw pedal to 0..1
    float pedal = normalizePedal(adc);

    // 4) Deadband
    pedal = applyDeadband(pedal, cfg_.deadband);

    // 5) Light LPF on pedal
    pedalFilt_ = lowPass(pedalFilt_, pedal, cfg_.pedalFilterTau, dt);

    // 6) Shape pedal with blended linear/quadratic curve
    float shaped = shapePedal(pedalFilt_);

    // 7) Compute effective current ceiling
    float iqCeiling = computeEffectiveIqLimit(fabsf(motorSpeedRadPerSec));

    // 8) Map shaped pedal to desired current
    float iqDesired = shaped * iqCeiling;

    // Optional minimum start current
    if (cfg_.enableMinStartCurrent &&
        pedalFilt_ > cfg_.minStartPedal &&
        iqDesired < cfg_.minStartCurrent) {
      iqDesired = cfg_.minStartCurrent;
    }

    // 9) Fault reaction
    if (fault_ && cfg_.cutToZeroOnFault) {
      iqDesired = 0.0f;
    }

    // 10) Asymmetric slew-rate limit
    iqRef_ = slewLimit(iqRef_, iqDesired, cfg_.iqRampUp, cfg_.iqRampDown, dt);

    // 11) Final clamp
    iqRef_ = constrainf(iqRef_, 0.0f, cfg_.iqMax);

    return iqRef_;
  }

  // Optional: immediate reset to zero current, e.g. on brake/safety event
  void reset() {
    pedalFilt_ = 0.0f;
    iqRef_ = 0.0f;
    fault_ = false;
    lastMicros_ = micros();
  }

  bool faulted() const { return fault_; }
  float filteredPedal() const { return pedalFilt_; }
  float iqCommand() const { return iqRef_; }

private:
  Config cfg_;
  uint32_t lastMicros_ = 0;
  float pedalFilt_ = 0.0f;
  float iqRef_ = 0.0f;
  bool fault_ = false;

  static float constrainf(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
  }

  bool isAdcFault(uint16_t adc) const {
    if (adc + cfg_.faultLowMargin < cfg_.adcMin) return true;
    if (adc > cfg_.adcMax + cfg_.faultHighMargin) return true;
    return false;
  }

  float normalizePedal(uint16_t adc) const {
    const float denom = (float)(cfg_.adcMax - cfg_.adcMin);
    if (denom <= 1.0f) return 0.0f;
    float p = ((float)adc - (float)cfg_.adcMin) / denom;
    return constrainf(p, 0.0f, 1.0f);
  }

  static float applyDeadband(float p, float deadband) {
    if (deadband <= 0.0f) return p;
    if (p <= deadband) return 0.0f;
    return (p - deadband) / (1.0f - deadband);
  }

  static float lowPass(float y, float x, float tau, float dt) {
    if (tau <= 0.0f) return x;
    float alpha = dt / (tau + dt);
    return y + alpha * (x - y);
  }

  float shapePedal(float p) const {
    float linearPart = cfg_.blendLinear * p;
    float quadPart = (1.0f - cfg_.blendLinear) * p * p;
    return constrainf(linearPart + quadPart, 0.0f, 1.0f);
  }

  float computeEffectiveIqLimit(float absSpeed) const {
    // launch cap at very low speed for smooth starts and lower peaks
    if (absSpeed <= cfg_.launchSpeedRadPerSec) {
      return min(cfg_.iqLaunchMax, cfg_.iqMax);
    }

    // blend from launch limit to full iqMax as speed rises
    if (absSpeed < cfg_.speedForFullIq) {
      float t = (absSpeed - cfg_.launchSpeedRadPerSec) /
                (cfg_.speedForFullIq - cfg_.launchSpeedRadPerSec);
      t = constrainf(t, 0.0f, 1.0f);
      return cfg_.iqLaunchMax + t * (cfg_.iqMax - cfg_.iqLaunchMax);
    }

    return cfg_.iqMax;
  }

  static float slewLimit(float current, float target,
                         float upRate, float downRate, float dt) {
    float delta = target - current;
    float upStep = upRate * dt;
    float downStep = downRate * dt;

    if (delta > 0.0f) {
      if (delta > upStep) delta = upStep;
    } else {
      if (-delta > downStep) delta = -downStep;
    }
    return current + delta;
  }
};