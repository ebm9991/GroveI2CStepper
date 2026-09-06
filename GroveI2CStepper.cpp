#include "GroveI2CStepper.h"
#include <math.h>

GroveI2CStepper::GroveI2CStepper(uint16_t stepsPerRevolution,
                                 uint8_t channel1Address,
                                 uint8_t channel2Address,
                                 TwoWire &wire)
: _wire(&wire),
  _ch1(channel1Address),
  _ch2(channel2Address),
  _stepsPerRevolution(stepsPerRevolution),
  _maxSpeed(200.0f),
  _speed(0.0f),
  _stepIntervalUs(0),
  _nextStepUs(0),
  _driveLevel(30),
  _phase(0),
  _position(0),
  _directionInverted(false),
  _released(true)
{
}

bool GroveI2CStepper::begin(uint32_t i2cClock)
{
  _wire->begin();
  _wire->setClock(i2cClock);

  delayMicroseconds(100); // DRV8830 requires >60 us after power-up before START.

  bool ok1 = probe(_ch1);
  bool ok2 = probe(_ch2);

  // Start safely with both channels disabled.
  setBridge(_ch1, MODE_COAST, 0);
  setBridge(_ch2, MODE_COAST, 0);
  _released = true;
  _speed = 0.0f;
  _stepIntervalUs = 0;
  _nextStepUs = micros();

  return ok1 && ok2;
}

void GroveI2CStepper::setMaxSpeed(float stepsPerSecond)
{
  if (stepsPerSecond < 0.0f) {
    stepsPerSecond = -stepsPerSecond;
  }
  if (stepsPerSecond < 0.01f) {
    stepsPerSecond = 0.01f;
  }

  _maxSpeed = stepsPerSecond;

  if (_speed > _maxSpeed) {
    _speed = _maxSpeed;
  } else if (_speed < -_maxSpeed) {
    _speed = -_maxSpeed;
  }
  updateInterval();
}

float GroveI2CStepper::maxSpeed() const
{
  return _maxSpeed;
}

void GroveI2CStepper::setSpeed(float stepsPerSecond)
{
  if (stepsPerSecond > _maxSpeed) {
    stepsPerSecond = _maxSpeed;
  } else if (stepsPerSecond < -_maxSpeed) {
    stepsPerSecond = -_maxSpeed;
  }

  const float oldSpeed = _speed;
  const bool wasStopped = (fabsf(oldSpeed) < 0.0001f);
  const bool willStop = (fabsf(stepsPerSecond) < 0.0001f);
  const bool directionChanged =
      (!wasStopped && !willStop && ((oldSpeed > 0.0f) != (stepsPerSecond > 0.0f)));

  _speed = stepsPerSecond;
  updateInterval();

  if (willStop) {
    return;
  }

  const uint32_t now = micros();

  // Start immediately after a true stop or after a direction reversal.
  if (wasStopped || directionChanged) {
    _nextStepUs = now;
    return;
  }

  // If the previous (slower) speed had scheduled the next step far in the
  // future, do not keep that stale deadline after increasing the speed.
  // Clamp the remaining wait to at most one interval at the NEW speed.
  const int32_t remaining = (int32_t)(_nextStepUs - now);
  if (remaining > 0 && (uint32_t)remaining > _stepIntervalUs) {
    _nextStepUs = now + _stepIntervalUs;
  }
}

float GroveI2CStepper::speed() const
{
  return _speed;
}

void GroveI2CStepper::setRPM(float rpmValue)
{
  if (_stepsPerRevolution == 0) {
    setSpeed(0.0f);
    return;
  }
  setSpeed(rpmValue * (float)_stepsPerRevolution / 60.0f);
}

float GroveI2CStepper::rpm() const
{
  if (_stepsPerRevolution == 0) {
    return 0.0f;
  }
  return _speed * 60.0f / (float)_stepsPerRevolution;
}

void GroveI2CStepper::setDriveLevel(uint8_t level)
{
  if (level > 63) {
    level = 63;
  }
  _driveLevel = level;

  // If currently holding/running, apply the new level immediately.
  if (!_released) {
    applyPhase(_phase);
  }
}

uint8_t GroveI2CStepper::driveLevel() const
{
  return _driveLevel;
}

bool GroveI2CStepper::runSpeed()
{
  if (fabsf(_speed) < 0.0001f || _stepIntervalUs == 0) {
    return false;
  }

  uint32_t now = micros();
  if ((int32_t)(now - _nextStepUs) < 0) {
    return false;
  }

  int8_t logicalDirection = (_speed > 0.0f) ? 1 : -1;
  int8_t phaseDirection = _directionInverted ? -logicalDirection : logicalDirection;

  _phase += phaseDirection;
  if (_phase > 3) {
    _phase = 0;
  } else if (_phase < 0) {
    _phase = 3;
  }

  if (!applyPhase(_phase)) {
    // Do not update the software position if the I2C command failed.
    _nextStepUs = now + _stepIntervalUs;
    return false;
  }

  _released = false;
  _position += logicalDirection;

  // Keep a stable cadence. If the loop was delayed, resynchronize instead of
  // trying to burst many I2C steps at once.
  uint32_t candidate = _nextStepUs + _stepIntervalUs;
  if ((int32_t)(now - candidate) >= 0) {
    _nextStepUs = now + _stepIntervalUs;
  } else {
    _nextStepUs = candidate;
  }

  return true;
}

void GroveI2CStepper::stop()
{
  _speed = 0.0f;
  _stepIntervalUs = 0;
  // Intentionally keep the current phase energized.
}

void GroveI2CStepper::hold()
{
  _speed = 0.0f;
  _stepIntervalUs = 0;
  applyPhase(_phase);
  _released = false;
}

void GroveI2CStepper::release()
{
  _speed = 0.0f;
  _stepIntervalUs = 0;
  setBridge(_ch1, MODE_COAST, 0);
  setBridge(_ch2, MODE_COAST, 0);
  _released = true;
}

bool GroveI2CStepper::isRunning() const
{
  return fabsf(_speed) >= 0.0001f;
}

long GroveI2CStepper::currentPosition() const
{
  return _position;
}

void GroveI2CStepper::setCurrentPosition(long position)
{
  _position = position;
}

uint16_t GroveI2CStepper::stepsPerRevolution() const
{
  return _stepsPerRevolution;
}

void GroveI2CStepper::setDirectionInverted(bool inverted)
{
  _directionInverted = inverted;
}

bool GroveI2CStepper::directionInverted() const
{
  return _directionInverted;
}

uint8_t GroveI2CStepper::faultChannel1()
{
  uint8_t value = 0xFF;
  readRegister(_ch1, REG_FAULT, value);
  return value;
}

uint8_t GroveI2CStepper::faultChannel2()
{
  uint8_t value = 0xFF;
  readRegister(_ch2, REG_FAULT, value);
  return value;
}

bool GroveI2CStepper::clearFaults()
{
  // FAULT register bit 7 = CLEAR.
  bool ok1 = writeRegister(_ch1, REG_FAULT, 0x80);
  bool ok2 = writeRegister(_ch2, REG_FAULT, 0x80);
  return ok1 && ok2;
}

bool GroveI2CStepper::channel1Detected()
{
  return probe(_ch1);
}

bool GroveI2CStepper::channel2Detected()
{
  return probe(_ch2);
}

bool GroveI2CStepper::writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(address);
  _wire->write(reg);
  _wire->write(value);
  return (_wire->endTransmission() == 0);
}

bool GroveI2CStepper::readRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
  _wire->beginTransmission(address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom(address, (uint8_t)1) != 1) {
    return false;
  }

  value = _wire->read();
  return true;
}

bool GroveI2CStepper::probe(uint8_t address)
{
  _wire->beginTransmission(address);
  return (_wire->endTransmission() == 0);
}

bool GroveI2CStepper::setBridge(uint8_t address, uint8_t mode, uint8_t level)
{
  if (level > 63) {
    level = 63;
  }
  uint8_t control = (uint8_t)((level << 2) | (mode & 0x03));
  return writeRegister(address, REG_CONTROL, control);
}

bool GroveI2CStepper::applyPhase(int8_t phase)
{
  bool ok1 = false;
  bool ok2 = false;

  switch (phase & 0x03) {
    case 0: // A+ B+
      ok1 = setBridge(_ch1, MODE_FORWARD, _driveLevel);
      ok2 = setBridge(_ch2, MODE_FORWARD, _driveLevel);
      break;

    case 1: // A- B+
      ok1 = setBridge(_ch1, MODE_REVERSE, _driveLevel);
      ok2 = setBridge(_ch2, MODE_FORWARD, _driveLevel);
      break;

    case 2: // A- B-
      ok1 = setBridge(_ch1, MODE_REVERSE, _driveLevel);
      ok2 = setBridge(_ch2, MODE_REVERSE, _driveLevel);
      break;

    default: // A+ B-
      ok1 = setBridge(_ch1, MODE_FORWARD, _driveLevel);
      ok2 = setBridge(_ch2, MODE_REVERSE, _driveLevel);
      break;
  }

  return ok1 && ok2;
}

void GroveI2CStepper::updateInterval()
{
  float absSpeed = fabsf(_speed);
  if (absSpeed < 0.0001f) {
    _stepIntervalUs = 0;
    return;
  }

  float interval = 1000000.0f / absSpeed;
  if (interval < 1.0f) {
    interval = 1.0f;
  }
  _stepIntervalUs = (uint32_t)interval;
}
