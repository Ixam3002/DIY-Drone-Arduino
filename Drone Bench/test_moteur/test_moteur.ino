#include <Servo.h>

Servo escA, escB, escC, escD;

const int ESCA_PIN = 9, ESCB_PIN = 5, ESCC_PIN = 10, ESCD_PIN = 6;

void setup() {
  // put your setup code here, to run once:

  escA.attach(ESCA_PIN, 1000, 2000);
  escB.attach(ESCB_PIN, 1000, 2000);
  escC.attach(ESCC_PIN, 1000, 2000);
  escD.attach(ESCD_PIN, 1000, 2000);

  escA.writeMicroseconds(1000);
  escB.writeMicroseconds(1000);
  escC.writeMicroseconds(1000);
  escD.writeMicroseconds(1000);

  delay(1000);

  escA.writeMicroseconds(1100);
  escB.writeMicroseconds(1100);
  escC.writeMicroseconds(1100);
  escD.writeMicroseconds(1100);

}

void loop() {
  // put your main code here, to run repeatedly:

}
