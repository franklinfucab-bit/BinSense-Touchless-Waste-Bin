#include <Stepper.h>

const int STEPS_PER_REV = 2048;
const int STEPS_90 = STEPS_PER_REV / 4;  // 512

#define IN1 15
#define IN2 16
#define IN3 17
#define IN4 18

Stepper lid(STEPS_PER_REV, IN1, IN3, IN2, IN4);

void setup() {
  Serial.begin(115200);
  lid.setSpeed(8);  // start slow; raise later if torque/speed OK
  Serial.println("90 deg open");
  lid.step(STEPS_90);
  delay(1500);
  Serial.println("90 deg close");
  lid.step(-STEPS_90);
}

void loop() {}