#include <Ultrasonic.h>
#include <ControlMotor.h>
#include <Servo.h>

Servo servo;
const int servoPin = 11;

const int trigPin = 13;
const int echoPin = 12;

const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 2; // Right motor Direction 1
const int in4Pin = 4; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

enum Motor {LEFT, RIGHT};

unsigned int sensorAngle[7] = {45, 60, 75, 90, 105, 120, 135};
unsigned int distance[7];

void setup(){
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);

  servo.attach(servoPin);
  servo.write (90);

  //go(LEFT, 0);
  //go(RIGHT, 0);
  
  //Scan the surroundings before starting
  servo.write(sensorAngle[0]);
  delay(200);
  for (unsigned int i = 0 ; i < 7 ; i++){
    readNextDistance(), delay (200);
  }
}

void loop(){
  readNextDistance();
  // See if something is too close at any angle
  unsigned bool tooClose = false;
  for (unsigned int i = 0 ; i < 7 ; i++){
    if (distance [i] < 300){ //300 mm
      tooClose = true;
    }
    if(tooClose){
      // Something's nearby: back up left
      go(LEFT, -180);
      go(RIGHT, -80);
      //next improvement: choose the longest
      //distance to turn towards it
    }else{
      // Nothing in our way: go forward
      go(LEFT, 255);
      go(RIGHT, 255);
    }
    // Check the next direction in 50 ms
    delay (50);
  }
}

void go(enum Motor m, int speed){
  //if x=y is true then var 1 is selected, if not var 2 is selected
  digitalWrite(m == LEFT ? in1Pin : in3Pin, speed > 0 ? HIGH : LOW);
  digitalWrite(m == LEFT ? in2Pin : in4Pin, speed <= 0 ? HIGH : LOW);
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed);
}

// Read distance from the ultrasonic sensor , return distance in mm
// Speed of sound in dry air, 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d = p*10ˆ−6s*343m/s = p*0.00343m = p*0.343mm/us
unsigned int readDistance(){
  digitalWrite(trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite(trigPin, LOW);
  unsigned long period = pulseIn(echoPin, HIGH); //wave response time (microseconds)
  Serial.println(period*343/2000);
  return period*343/2000; //formula to convert time in distance (mm)
}

// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed. This takes a reading, then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance (){
  static unsigned int angleIndex = 0;
  static signed int step = 1;
  distance[angleIndex] = readDistance(); //stores the distance for each angle
  angleIndex+=step;
  if (angleIndex == 6){ //array index is in the last position
    step = -1; //index will start going back in the distance array
  }else if (angleIndex == 0){
    step = 1; //index will start to rise again
  }
  servo.write(sensorAngle[angleIndex]); //change servo angle
}
