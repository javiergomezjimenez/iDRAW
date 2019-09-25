#include <Servo.h>

Servo pinza;
int agarre;

void setup() {
  // put your setup code here, to run once:
pinza.attach(8);
}

void loop() {
  // put your main code here, to run repeatedly:
if(agarre==0)pinza.write(90);
else if(agarre==1)pinza.write(20);
} 
