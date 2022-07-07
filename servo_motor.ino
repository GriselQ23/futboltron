#include <Servo.h>
Servo servomotor;
void setup() {
  servomotor.attach(3);
}

void loop() {
  servomotor.write(0);
  delay(600);
  servomotor.write(90);
 // delay(2000);
 // servomotor.write(180);
  delay(600);
}
