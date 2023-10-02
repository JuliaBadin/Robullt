#include <LiquidCrystal_I2C.h>
//#include <Servo.h>
#include <Ultrasonic.h>
#include <ControlMotor.h>

//LiquidCrystal_I2C lcd(0x27, 16, 2);
//Servo servo;

int trigPin = 7;     // trig pin of ultrasonic sensor
int echoPin = 8;     // Echo pin of ultrasonic sensor

//Ultrasonic sensor(7,8,24000); // (Trig pin, Echo pin, maximum distance unit is us) That is, 24000us = about 4 meters 
ControlMotor control(9,10,5,6,11,3); // Right motor 1, right motor 2, left motor 1, left motor 2, right PWM, left PWM 

int speed = 150; //Declare a variable to store the motor speed. The initial speed is 150. 
int measurement_speed = 5;//Adjust sensor measurement speed. 
long int distance, duration;
int random_value;//Save the random value. 


void setup() {

  Serial.begin(9600);

  /*lcd.init(); // initializes LCD
  lcd.backlight(); // turn on backlight
  lcd.setCursor(0, 0); // set cursor at top left*/

  /*servo.attach(12);
  servo.write(0);
  delay(2000);*/

  pinMode(trigPin, OUTPUT);         // set trig pin as output
  pinMode(echoPin, INPUT);          //set echo pin as input to capture reflected waves*/

}

void loop() {
  control.Motor(150,1);//The car moves forward at a speed of 150. 
  //distance=sensor.Ranging(CM);//Measure the distance and store it in the distance variable. 

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // measure duration of pulse from ECHO pin
  duration = pulseIn(echoPin, HIGH);
  // calculate the distance
  distance = 0.017 * duration;

  //delay(measurement_speed);//Delay to control sensor measurement speed.
 
  Serial.println(distance);//Print the distance. 

  random_value = random (2);//Create a random value to prevent the car from turning in only one direction. 
  
  while(distance<40){//Applies when the distance to the obstacle is less than 40cm. 
    delay(measurement_speed);//Delay to control sensor measurement speed. 
    control.Motor(0,1);//Stop the motor. 
    //distance = sensor.Ranging(CM); 

    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // measure duration of pulse from ECHO pin
    duration = pulseIn(echoPin, HIGH);
    // calculate the distance
    distance = 0.017 * duration;
    Serial.println(distance);//Print the distance. 

    delay(2000); 

    if(random_value==0){
      control.Motor(170,100);//The car turns to the right for 0.4 seconds. 
      delay(400);
    } 
    
    else if (random_value==1){     
      control.Motor(170,-100);//The car turns left for 0.4 seconds. 
      delay(400);
    } 
  } 
  
  /*servo.write(180);
  delay(1000);
  servo.write(0);
  delay(1000);*/
}