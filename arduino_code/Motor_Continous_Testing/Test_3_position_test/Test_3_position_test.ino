#include <Servo.h>
// 
Servo servo;

const byte servoPin = 9;

void setup()
{
  Serial.begin(9600);
  servo.write(90);
  servo.attach(servoPin);

}

void loop()
{
  if (Serial.available())
  {
    int pos = Serial.parseInt();
   // prevent harm to servo from over extension
    if (pos > 180)
    {
      pos = 180;
    }
    if (pos < 0)
    {
      pos - 0;
    }
    servo.write(pos);
  }
}
