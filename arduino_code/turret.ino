#include <Servo.h>
int input;
int increment;
Servo baseServo;
Servo laserServo;
int pos = 0;
void setup() {
    Serial.begin(9600); 
    delay(2000);  
    increment = 0;
    Serial.println("Type something!");
    baseServo.attach(8);
    laserServo.attach(9);
    baseServo.write(angleConvert(100)); 
    laserServo.write(angleConvert(195)); 
}
float angleConvert(float inputAngle){
  return (inputAngle / 360.0) * 270.0;
}
float zeroAngle(float angle, float zero){
  return (zero-angle);
}



//Input format (a/b)angle(type float or int)
//Ex a-180.0:b129
void loop() {
  float angle = 0;
  int decimalPlace = 1;
  bool decimalBool = false;
  int motorId = 0;
  bool negative = false;
  while(true){
    if(Serial.available()){
      input = Serial.read();
      if(input == 97){
        motorId = 0;
        continue;
      }
      if(input == '-'){
        negative = true;
        continue;
      }
      if(input == 98){
        motorId = 1;
        continue;
      }
      if(input==46){
        decimalBool = true;
        continue;
      }
      if(!(input > 47 && input < 58)){
        break;
      }
      
      if(decimalBool){
        decimalPlace*=10;
        angle += (input - 48.0)/decimalPlace;
      } else{
        angle *= 10;
        angle += input - 48;
      }
      
    }
  }
  if(negative){
    angle*=-1;
  }
  Serial.print("MotorId: " );
  Serial.print(motorId);
  Serial.print(". Going to: " );
  
  if(motorId == 0){
    angle = zeroAngle(angle, 100);
    if(angle>155){
      angle = 155;
    }
    if(angle<45){
      angle = 45;
    }
    Serial.println(angle);
    baseServo.write(angleConvert(angle));         
    delay(15);
  } else if(motorId == 1){
    angle = zeroAngle(angle, 195);
    if(angle>230){
      angle = 230;
    }
    if(angle<150){
      angle = 150;
    }
    Serial.println(angle);
    laserServo.write(angleConvert(angle));         
    delay(15);
  }
  

}
