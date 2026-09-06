#include <GroveI2CStepper.h>

GroveI2CStepper motor(200);

// This example only demonstrates the intended PID interface.
// Replace measuredAngle with the angle read from your AS5600.

float setpoint = 90.0f;
float measuredAngle = 0.0f;
float Kp = 1.0f;

void setup()
{
  Serial.begin(115200);

  if (!motor.begin()) {
    Serial.println("Erreur I2C moteur");
    while (1) {}
  }

  motor.setDriveLevel(30);
  motor.setMaxSpeed(100);
}

void loop()
{
  // measuredAngle = readAS5600();

  float error = setpoint - measuredAngle;
  float pidOutput = Kp * error;

  // PID output directly becomes a signed motor speed in steps/s.
  motor.setSpeed(pidOutput);
  motor.runSpeed();
}
