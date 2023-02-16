#include <Servo.h>//Library for Servos
#include <QTRSensors.h>//Library for the IR Sensor Array
#include <NewPing.h>//Library for Ultrasonic sensor

// RANGES FOR ULTRASONIC
#define TRIG_PIN 10
#define ECHO_PIN 11
#define MAX_RANGE 15
#define MIN_RANGE 1

// SERVOS
#define LEFT_MOTOR 6
#define RIGHT_MOTOR 3
#define THRESHOLD 1800

// LED PINS
#define RED_LED 7
#define GREEN_LED 8
#define BLUE_LED 9

//LINE FOLLOWING GAIN
#define KP 50

//Ultrasonic object
NewPing ultraSonic = NewPing(TRIG_PIN, ECHO_PIN, 100);

//Servo objects
Servo leftMotor;  
Servo rightMotor;

//Reflectance sensor object
const int numSensors = 5;
QTRSensorsRC qtr((char[]) {A1, A2, A3, A4, A5}, 5);
unsigned int sensorValues[numSensors];

//enum for defining states of the system
enum STATE{
  HOME,
  SEARCH_MODE,
  CHECKING_INTERSECTION,
  CHECKING_RIGHT_LANE,
  CHECKING_LEFT_LANE,
  PARKING_MODE,
  // GO_TO_EXIT,
  END,
  TEST,
};

//Inital State is when Robot is at the Home Position
STATE currentState = HOME;


void setup() {
  // put your setup code here, to run once:

  currentState = HOME;
  
  Serial.begin(9600);
  Serial.print("Hello!");

  // Ultrasonic Setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //Motor Setup
  rightMotor.attach(RIGHT_MOTOR);
  leftMotor.attach(LEFT_MOTOR);

  // initialize digital pin LED_BUILTIN as an output
  // to indicate END of program (STATE)
  pinMode(LED_BUILTIN, OUTPUT);

  // Indication LED Set-up
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

}

int intersectionCount = 0;
int OccupiedParkingSpotsCount = 0;
bool parkRight = false;
bool parkLeft = false;


void loop() {

  // do{

  // }while(true);
  

  // delay(3000); 

  switch (currentState) {

    case HOME:
      Serial.print("HOME STATE!");
      currentState = SEARCH_MODE;

    case SEARCH_MODE:
      Serial.println("SEARCH STATE!");
      // follows line until an intersection is detected
      lineFollow(KP);
      // check again if it is an intersection
      // if (intersectionDetected()) {
        // Serial.println("Intersection detected!");
        // increment global intersection counter
        intersectionCount++;
        redLedBlink();
        // check if it is the last intersection (for now conisdering only 4 intersections in total)
        if (intersectionCount == 7){
          // Serial.println(" Intersection number = \t");
          // Serial.println(intersectionCount);
          // Serial.println("DONE!!!!!");
          currentState = END;
        }

        else {
          // check if it is an odd intersection
          if (intersectionCount % 2 != 0) {
            // Serial.println("ODD intersection. Checking RIGHT lane!");
            // Serial.println(" Intersection number = \t");
            // Serial.println(intersectionCount);
            currentState = CHECKING_RIGHT_LANE;
          }

          // check if it is an even intersection
          if (intersectionCount % 2 == 0) {
            // Serial.println("EVEN intersection. Parking mode!"); 
            // Serial.println(" Intersection number = \t");
            // Serial.println(intersectionCount);
            currentState = PARKING_MODE;
          }
        // }

      }
      break;

    case CHECKING_RIGHT_LANE:
      if (!object_in_Right_Lane()) {
        parkRight = true;
        // Serial.print("No object on the right");
        currentState = PARKING_MODE;
      }
      else {
        currentState = CHECKING_LEFT_LANE;
        OccupiedParkingSpotsCount++;
      }
      break;

    case CHECKING_LEFT_LANE:
      if (!object_in_Left_Lane()) {
        parkLeft = true;
        // Serial.print("No object on the left");
        currentState = PARKING_MODE;
      }
      else {
        currentState = SEARCH_MODE;
        OccupiedParkingSpotsCount++;
      }
      break;

    case PARKING_MODE:
      // Serial.println("PARKING MODE!");
      // lineFollow(KP);
      // if (intersectionDetected()){
      //   intersectionCount++;
      if (intersectionCount % 2 == 0) {
        // Serial.println(" Intersection number = \t");
        // Serial.println(intersectionCount);
        // Serial.println("parkRight flag =");
        // Serial.println(parkRight);
        // Serial.println("parkLeft flag =");
        // Serial.println(parkLeft);
        if (parkRight) {
          park_right();
          // Serial.println("PARKING RIGHT!");
          currentState = END;
        } else if (parkLeft) {
          park_left();
          // Serial.println("PARKING LEFT!");
          currentState = END;
        }
        else {
          botForward();
          currentState = SEARCH_MODE;
          // Serial.println("GO BACK TO SEARCH MODE!");
        }
      }
      else {
        botForward();
        currentState = SEARCH_MODE;
        // Serial.println("GO BACK TO SEARCH MODE!");
      }
      
      
      break;

    case END:
      // Serial.println("END MODE!");
      currentState = end();
      // Serial.println("OccupiedParkingSpotsCount = ");
      // Serial.println(OccupiedParkingSpotsCount);
      break;

    case TEST:
      currentState = test();
      break;
  }
}
  

STATE test(){ 

  // bool objectFoundinLane = false;
  // objectFoundinLane = linefollowWithObjectDetection(KP);
  // Serial.print("objectFoundinLane = \t");
  // Serial.println(objectFoundinLane);


  return TEST;

}

STATE end(){

  // Blink the inbuilt-LED (on pin 13) continuously to indicate that the program is finished.
  showBinary(OccupiedParkingSpotsCount);

  while(true){
    digitalWrite(LED_BUILTIN, HIGH);   
    delay(1000);                     
    digitalWrite(LED_BUILTIN, LOW);    
    delay(1000);
  }

  return END;
}

void park_right(){

  // turn right
  botForward();
  turnRight();
  // line follow until intersection
  botForward();
  lineFollow(KP);
  // move backwards to park
  botBackward();
  greenLedBlink();

}

void park_left(){
  // turn left
  botForward();
  turnLeft();
  // line follow until intersection
  botForward();
  lineFollow(KP);
  // move backwards to park
  botBackward();
  greenLedBlink();
}

//=============================================================================================
// void object_in_Left_Lane()
// ----- Function checking if there is an object present along the LEFT lane using the ultrasonic sensor
// return true if an object is present, false otherwise
//=============================================================================================
bool object_in_Left_Lane(){
  bool objectFoundinLeftLane = false;
  // botForward();
  // botBackward();
  turnLeft();
  botForward();
  objectFoundinLeftLane = linefollowWithObjectDetection(KP, 'l');
  // ping Ultrasonic sensor
  turn180();  // linefollow
  lineFollow(KP);
  botForward();
  turnLeft();
  // return true or false
  return objectFoundinLeftLane;

}

//=============================================================================================
// void object_in_Right_Lane()
// ----- Function checking if there is an object present along the RIGHT lane using the ultrasonic sensor
// return true if an object is present, false otherwise
//=============================================================================================
bool object_in_Right_Lane(){

  bool objectFound = 0;

  botForward();
  turnRight();
  botForward();
  lineFollow(KP);
  // ping Ultrasonic sensor
  turn180();  // linefollow
  //botForward();
  // turn 180
  objectFound =linefollowWithObjectDetection(KP, 'r');
  botForward();
  turnRight();

  // botForward();
  // return true or false
  return objectFound;
  

}

//=============================================================================================
// void intersectionDetected()
// ----- Function that uses the extreme left and right sensors on the sensor array
// ----- to check if an intersection has been reached
//=============================================================================================
bool intersectionDetected(){
    qtr.read(sensorValues);
    uint16_t leftSen2 = sensorValues[0];
    uint16_t rightSen2 = sensorValues[4];

    if(leftSen2 > THRESHOLD && rightSen2 > THRESHOLD){return true;}
    else{ return false;}

}



//=============================================================================================
// void botForward()
// ----- Function moving the bot forward the robots length
//=============================================================================================
void botForward(){
  rightMotor.writeMicroseconds(1445);
  leftMotor.writeMicroseconds(1555);
  delay(500);
  stopMotors();
}

//=============================================================================================
// void botBackward()
// ----- Function moving the bot backward 1/2 the robots length
//=============================================================================================
void botBackward(){
  leftMotor.writeMicroseconds(1445);
  rightMotor.writeMicroseconds(1555);
  delay(500);
  stopMotors();
}

//=============================================================================================
// void turnRight()
// ----- Function for turning the bot backward right and stopping when the front sensor array
// ----- detects a black line.
//=============================================================================================
void turnRight(){
  uint16_t midSen;
  uint16_t rightSen;
  
  //Turn for a little with no sensor input
  leftMotor.writeMicroseconds(1600);
  rightMotor.writeMicroseconds(1600);
  delay(300);

  //Keep turning until the right sensor detects black
  do{
    qtr.read(sensorValues);
    rightSen = sensorValues[3];

  }while(rightSen < 2400);

  delay(300);

  //Finally turn until the mid sensor detects black
  do{
    qtr.read(sensorValues);
    midSen = sensorValues[2];

  }while(midSen < 2400);
  

  stopMotors();
}

//=============================================================================================
// void turnLeft()
// ----- Function for turning the bot backward left and stopping when the front sensor array
// ----- detects a black line.
//=============================================================================================
void turnLeft(){
  uint16_t midSen;
  uint16_t leftSen;
  
  //Turn for a little with no sensor input
  leftMotor.writeMicroseconds(1300);
  rightMotor.writeMicroseconds(1300);
  delay(300);


  //Keep turning until the left sensor detects black
  do{
    qtr.read(sensorValues);
    leftSen = sensorValues[1];

  
  }while(leftSen < 2400);

  //Finally turn until the mid sensor detects black
  do{
    qtr.read(sensorValues);
    midSen = sensorValues[2];
    
    }while(midSen < 2400);


  stopMotors();
}

//=============================================================================================
// void turn180()
// ----- Function for turning the bot 180 degrees, This is done by timing the bot with its 
// ----- rotation speed until it has turned 180 degrees
//=============================================================================================
void turn180(){

  leftMotor.writeMicroseconds(1800);
  rightMotor.writeMicroseconds(1800);
  delay(1350);

  uint16_t leftSen;
  
  //keep turning  until the mid sensor detects black
  do{
    qtr.read(sensorValues);
    leftSen = sensorValues[1];
    
    }while(leftSen < 2400);

  stopMotors();
}



//=============================================================================================
// void drive(float leftPower, float rightPower)
// ----- Function that drives the bot at the power level defined by passing a value in
//=============================================================================================
void drive(float leftPower, float rightPower){

  //2000 is maximum in one direction and 1000 is maximum in other direction
  //1500 is stop
  leftMotor.writeMicroseconds(leftPower);
  rightMotor.writeMicroseconds(rightPower);
  delay(20);
}


//=============================================================================================
// void stopMotors()
// ----- Function that stops the bots servos
//=============================================================================================
void stopMotors(){
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  delay(20);
}


//=============================================================================================
// void linefollow(float Kp)
// ----- This function controls the robot to follow a line until an intersection is detected.
//=============================================================================================
void lineFollow(int Kp){

  //Variables local to this function
  uint16_t x_leftSen;
  uint16_t leftSen;
  uint16_t midSen;
  uint16_t rightSen;
  uint16_t x_rightSen;
  int error = 0;
  int leftSpeed ;
  int rightSpeed;
  int iniMotorPower = 100;

  do{

  qtr.read(sensorValues);
  x_leftSen = sensorValues[0];
  leftSen = sensorValues[1];
  midSen = sensorValues[2];
  rightSen = sensorValues[3];
  x_rightSen = sensorValues[4];


  if ((sensorValues[0]< 1000)&&(sensorValues[1]< 1000)&&(sensorValues[2] < 1000)&&(sensorValues[3] < 1000)&&(sensorValues[4] < 1000)) {
    
    // shut off motors when on all 5 sensors detect white surface but stay in the line-following loop

    leftSpeed = 1500;
    rightSpeed = 1500;

  }

  else if ((sensorValues[0] > 2400)&&(sensorValues[1] > 2400)&&(sensorValues[2] > 2400)&&(sensorValues[3] > 2400)&&(sensorValues[4] > 2400)) {

    // exit the loop when on all 5 sensors detect black surface 
    // double check that it is actually an intersection and then break out of the line-following loop

    delay(75);
    if (intersectionDetected()){
      break;
    }

  }
  

  else {

    // Line-following control based on difference in motor speeds proportional to the error (devation from middle sensor being on the line) in sensor values
    // sensor reading when on black surface --> 2500
    // sensor reading when on white surface --> < 1500
    /// threshodl chosen --> 2200

    if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]>= THRESHOLD)) 
    {error = -4;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]>= THRESHOLD)) 
    {error = -3; }

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = -2;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = -1;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 0;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 1;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 2;}

    else if((sensorValues[0]>= THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 3;}

    else if((sensorValues[0]>= THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 4;}


  leftSpeed = (1500 + iniMotorPower - Kp * error);
  rightSpeed = (1500 - iniMotorPower - Kp * error);

  }

  drive(leftSpeed, rightSpeed);

  } while(true);

  // when an intersection is detected break out of the line-following loop and shut off the servo motors

  stopMotors();
  // blueLedBlink();
  
}

//=============================================================================================
// void linefollowWithObjectDetection(float Kp)
// ----- This function controls the robot to follow a line until an intersection is detected
// ----- WHILST also pinging the ultrasonic sensor
// ----- returns a boolen to indicate whether an object was detected while line following
// 2nd argument : true = right lane, false= left lane
//=============================================================================================
int linefollowWithObjectDetection(int Kp , char lane){

  //Variables local to this function
  uint16_t x_leftSen;
  uint16_t leftSen;
  uint16_t midSen;
  uint16_t rightSen;
  uint16_t x_rightSen;
  int error = 0;
  int leftSpeed ;
  int rightSpeed;
  int iniMotorPower = 100;
  int distance;
  int object_distance_counter = 0;
  bool object = false;
  bool ledblinkflag = true;
  float total_pings =0;
  float object_pings = 0;
  float final_difference =0;
  float ratio;
  float objectFoundTime = 0;
  float initial_difference = 0;
  float objectLostTime = 0;
  float endTime = 0;
  float object_time = 0;
  float TotalTime = 0;
  float startTime = 0;
  int i = 0;
  float temp;


  startTime = millis();

  do{

  qtr.read(sensorValues);
  x_leftSen = sensorValues[0];
  leftSen = sensorValues[1];
  midSen = sensorValues[2];
  rightSen = sensorValues[3];
  x_rightSen = sensorValues[4];


  if ((sensorValues[0]< 1000)&&(sensorValues[1]< 1000)&&(sensorValues[2] < 1000)&&(sensorValues[3] < 1000)&&(sensorValues[4] < 1000)) {
    
    // shut off motors when on all 5 sensors detect white surface but stay in the line-following loop

    leftSpeed = 1500;
    rightSpeed = 1500;

  }

  else if ((sensorValues[0] > 2400)&&(sensorValues[1] > 2400)&&(sensorValues[2] > 2400)&&(sensorValues[3] > 2400)&&(sensorValues[4] > 2400)) {

    // exit the loop when on all 5 sensors detect black surface 
    // double check that it is actually an intersection and then break out of the line-following loop

    delay(75);
    if (intersectionDetected()){
      break;
    }

  }
  

  else {

    // Line-following control based on difference in motor speeds proportional to the error (devation from middle sensor being on the line) in sensor values
    // sensor reading when on black surface --> 2500
    // sensor reading when on white surface --> < 1500
    /// threshodl chosen --> 2200

    if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]>= THRESHOLD)) 
    {error = -4;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]>= THRESHOLD)) 
    {error = -3; }

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = -2;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]>= THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = -1;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 0;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]>= THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 1;}

    else if((sensorValues[0]< THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 2;}

    else if((sensorValues[0]>= THRESHOLD)&&(sensorValues[1]>= THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 3;}

    else if((sensorValues[0]>= THRESHOLD)&&(sensorValues[1]< THRESHOLD)&&(sensorValues[2]< THRESHOLD)&&(sensorValues[3]< THRESHOLD)&&(sensorValues[4]< THRESHOLD)) 
    {error = 4;}


  leftSpeed = (1500 + iniMotorPower - Kp * error);
  rightSpeed = (1500 - iniMotorPower - Kp * error);

  //---------------------CHEKCING FOR PARKED OBJECT/CAR --------------------------------------------------------------------

  distance = ultraSonic.ping_cm();
  total_pings++;

  if (distance >= MIN_RANGE && distance < MAX_RANGE){
    object_distance_counter++;
    if(object_distance_counter ==10){
      object = true;
      blueLedBlink();
      // OccupiedParkingSpotsCount++;
      objectFoundTime = millis();
    }
  }

  if (distance >= MAX_RANGE){
    if (object == true){
      i++;
      if (i==3)
        {
        objectLostTime = millis();
        object = false;
        }
    }
  }

  if ((object_distance_counter >= 30) && (ledblinkflag == true)){
    // blueLedBlink();
    ledblinkflag = false;
  }

  // if (object_distance_counter >= 10){
  //   object = true;
  // }


  //------------------------------------------------------------------------------------------------------------------------


  }

  drive(leftSpeed, rightSpeed);

  } while(true);

  // when an intersection is detected break out of the line-following loop and shut off the servo motors
  stopMotors();

  //EndTime 
  endTime = millis();

  //initial_difference calculation
  initial_difference = objectFoundTime - startTime;

  // final_difference
  final_difference = endTime - objectLostTime;

  //object time
  object_time = objectLostTime - objectFoundTime;

  //Total time difference
  TotalTime = endTime - startTime;

  //Ratio
  ratio = object_time/TotalTime;



  // Serial.println("TotalTime = \t");
  // Serial.print(TotalTime);
  // Serial.println("object_time = \t");
  // Serial.print(object_time);
  // Serial.println("objectFoundTime = \t");
  // Serial.print(objectFoundTime);
  // Serial.println("objectLostTime = \t");
  // Serial.print(objectLostTime);
  // Serial.println("initial_difference = \t");
  // Serial.print(initial_difference);
  // Serial.println("final_difference = \t");
  // Serial.print(final_difference);
  // Serial.println("ratio = \t");
  // Serial.print(ratio);

  if(ratio <= 0.27){ // small object

    if (lane == 'l'){
      temp = initial_difference;
      initial_difference = final_difference;
      final_difference = temp; 
    }
    // else if (lane == 'r'){
    //   // do nothing
    // }

    if(initial_difference < final_difference){
      //small_object_is_back --> equivalent to available parking
      // Serial.println("small object in the BACK, I will park");
      return false;
    }

    else if(initial_difference > final_difference){
      //small_object_is_front --> equivalent to occupied parking
      // Serial.println("small object in the FRONT, I will NOT park, NO NO.");
      return true;
    }
  }
  else{
    //no_space --> equivalent to occupied parking
    // Serial.println("LARGE object. AMMOOO! NOT parking.");
    return true;
  }

}


void redLedBlink(){
  // for object detection with ultrasonic
  digitalWrite(RED_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  delay(300);
}

void greenLedBlink(){
  digitalWrite(GREEN_LED, HIGH);
  delay(1000);
  digitalWrite(GREEN_LED, LOW);
  delay(1000);
}

void blueLedBlink(){ 
  // for intersection detection
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BLUE_LED, LOW);
  delay(100);
}

void showBinary(int number) {
  
  digitalWrite(RED_LED, bitRead(number, 0));
  digitalWrite(GREEN_LED, bitRead(number, 1));
  digitalWrite(BLUE_LED, bitRead(number, 2));

}


