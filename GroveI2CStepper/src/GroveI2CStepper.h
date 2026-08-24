#ifndef GROVE_I2C_STEPPER_H
#define GROVE_I2C_STEPPER_H

#include <Arduino.h>
#include <Wire.h>

class GroveI2CStepper {
public:
  // Default addresses for Seeed Grove - Mini I2C Motor Driver v1.1
  // Confirmed by I2C scan on the v1.1 hardware:
  // CH1: 0x65 (Arduino/Wire 7-bit)
  // CH2: 0x60 (Arduino/Wire 7-bit)
  GroveI2CStepper(uint16_t stepsPerRevolution = 200,
                  uint8_t channel1Address = 0x65,
                  uint8_t channel2Address = 0x60,
                  TwoWire &wire = Wire);

  bool begin(uint32_t i2cClock = 100000UL);

  void setMaxSpeed(float stepsPerSecond);
  float maxSpeed() const;

  // Signed speed in steps/s. Positive and negative values reverse direction.
  void setSpeed(float stepsPerSecond);
  float speed() const;

  // Convenience functions using rpm instead of steps/s.
  void setRPM(float rpm);
  float rpm() const;

  // DRV8830 VSET value: 0..63.
  void setDriveLevel(uint8_t level);
  uint8_t driveLevel() const;

  // Non-blocking. Call as often as possible from loop().
  // Returns true when one physical step was generated.
  bool runSpeed();

  // Stop generating new steps but keep the current phase energized.
  void stop();

  // Re-energize the current phase (useful after release()).
  void hold();

  // Disable both H-bridges (motor shaft becomes free, apart from detent torque).
  void release();

  bool isRunning() const;

  long currentPosition() const;
  void setCurrentPosition(long position);

  uint16_t stepsPerRevolution() const;

  // If the mechanical direction is opposite to what you want, invert it here.
  void setDirectionInverted(bool inverted);
  bool directionInverted() const;

  // DRV8830 fault helpers.
  uint8_t faultChannel1();
  uint8_t faultChannel2();
  bool clearFaults();

  // Bus/device diagnostics.
  bool channel1Detected();
  bool channel2Detected();

private:
  static constexpr uint8_t REG_CONTROL = 0x00;
  static constexpr uint8_t REG_FAULT   = 0x01;

  static constexpr uint8_t MODE_COAST   = 0x00; // IN2=0, IN1=0
  static constexpr uint8_t MODE_FORWARD = 0x01; // IN2=0, IN1=1
  static constexpr uint8_t MODE_REVERSE = 0x02; // IN2=1, IN1=0
  static constexpr uint8_t MODE_BRAKE   = 0x03; // IN2=1, IN1=1

  TwoWire *_wire;
  uint8_t _ch1;
  uint8_t _ch2;
  uint16_t _stepsPerRevolution;

  float _maxSpeed;
  float _speed;
  uint32_t _stepIntervalUs;
  uint32_t _nextStepUs;

  uint8_t _driveLevel;
  int8_t _phase;
  long _position;
  bool _directionInverted;
  bool _released;

  bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);
  bool readRegister(uint8_t address, uint8_t reg, uint8_t &value);
  bool probe(uint8_t address);

  bool setBridge(uint8_t address, uint8_t mode, uint8_t level);
  bool applyPhase(int8_t phase);
  void updateInterval();
};

#endif
