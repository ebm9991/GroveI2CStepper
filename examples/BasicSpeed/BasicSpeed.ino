#include <GroveI2CStepper.h>

// NEMA17 1.8 deg/step = 200 full steps/revolution.
GroveI2CStepper motor(200);

void setup()
{
  Serial.begin(115200);

  if (!motor.begin()) {
    Serial.println("Erreur: les deux DRV8830 ne repondent pas sur I2C.");
    while (1) {}
  }

  motor.setDriveLevel(30);  // 0..63
  motor.setMaxSpeed(100);   // steps/s
  motor.setSpeed(30);       // +30 steps/s

  Serial.println("Moteur: +30 pas/s");
}

void loop()
{
  // Non-blocking: call continuously.
  motor.runSpeed();

  // Example serial commands:
  // 1 = +30 steps/s
  // 2 = -30 steps/s
  // 0 = stop and hold
  // R = release coils
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '1') {
      motor.setSpeed(30);
      Serial.println("+30 pas/s");
    }
    else if (c == '2') {
      motor.setSpeed(-30);
      Serial.println("-30 pas/s");
    }
    else if (c == '0') {
      motor.stop();
      Serial.println("Arret avec maintien");
    }
    else if (c == 'R' || c == 'r') {
      motor.release();
      Serial.println("Bobines liberees");
    }
  }
}
