#include <PID_v1.h>

//for small bot, black on inside, red on outside for connecting motor wires
//MUST connect it to the Blynk to run (e.g. to start reading the ultrasonic sensor value)
#include <Pixy2.h>
#include <NewPing.h>
#define RH_ENCODER_A 18  //right encoder A interrupt pin
#define RH_ENCODER_B 19 // right encoder B interrupt pin
#define LH_ENCODER_A 20 //left encoder A interrupt pin
#define LH_ENCODER_B 21 //left encoder B interrupt pin
#define leftMotorPWM 3 //motor B
#define rightMotorPWM 5 //motor A
#define rightMotorIN1 11 //motor A
#define rightMotorIN2 12 //motor A
#define leftMotorIN3 9 // motor B
#define leftMotorIN4 10 //(motor B)
#define leftEncoderInterrupt 3 //MEGA interrupt pin 20= attachInterrupt3
#define rightEncoderInterrupt 5 //MEGA interrupt pin18=attachInterrupt5

//definitios for ultrasonic sensor system
#define leftSensorTP 30//Left Ultrasonic Sensor Trigger Pin
#define leftSensorEP 31 //Left Ultrasonic Sensor Echo Pin
#define middleSensorTP 32//Middle Ultrasonic Sensor Trigger Pin
#define middleSensorEP 33 //Middle Ultrasonic Sensor Echo Pin
#define rightSensorTP 34//Right Ultrasonic Sensor Trigger Pin
#define rightSensorEP 35 //Right Ultrasonic Sensor Echo Pin
#define pingSpeed 150 // Ping frequency (in milliseconds), fastest we should ping is about 35ms per sensor
#define criticalD 30// critical distance for object detection (robot stops immediately)
#define warningD 100 //warning distance for object to come to a smooth stop 
#define maxDetectionD 500 //maximum detection range ultrasonic sensors 

// variables to store the number of encoder pulses
// for each motor
volatile long leftCount = 0; //encoder ticks counter used in ISR
volatile long rightCount = 0; //encoder ticks counter used in ISR
float time = 0.0; //initalise time [should this be unsigned long? - Daniel]
float Previoustime = 0.0;
float SampleTime;
float leftWheelRev;
float rightWheelRev;
float rightchange;
float leftchange;
double leftEncoderSpeed;
double rightEncoderSpeed;
float oldleftCount = 0.0;
float oldrightCount = 0.0;

//variables for PID auto forward
float errorLSpeed = 0.0;
float errorRSpeed = 0.0;
double PWMLSpeed = 0;
double PWMRSpeed = 0;
double desiredLSpeed;
double desiredRSpeed;

//Variables for general remote control
double robotSpeed = 1.5; //default robot speed=1.5m/s
int leftPWMSpeed = 0; //sets speed of left motor to 0 initially
int rightPWMSpeed = 0; //sets speed of right motor to 0 initially
char RCMode = 1;
char autoForwardMode = 0; //mode for autoforward button
char AFMaxSpeedReached;


//Variables for obstacle avoidance
long leftDistanceCm = 500; //measures distance recorded by LeftSensor
long middleDistanceCm = 500; //measures distance recorded by MiddleSensor
long rightDistanceCm = 500; //measures distance recorded by RightSensor
unsigned long pingTimer1, pingTimer2, pingTimer3;
char middleObjectDetected = 0; //sub-mode for RC control
char leftObjectDetected = 0; //sub-mode for RC control
char rightObjectDetected = 0; //sub-mode for RC control
char objectTooClose = 0; //sub-mode for RC control
NewPing leftSensor(leftSensorTP, leftSensorEP, maxDetectionD); // Left Sensor: trigger pin, echo pin, maximum detection distance in cm
NewPing middleSensor(middleSensorTP, middleSensorEP, maxDetectionD); // Middle Sensor: trigger pin, echo pin, maximum detection distance in cm
NewPing rightSensor(rightSensorTP, rightSensorEP, maxDetectionD); // Right Sensor: trigger pin, echo pin, maximum detection distance in cm

//PIXY initialization
Pixy2 pixy; //make a global instance of pixy
int xpos;
int objectWidth;
int objectHeight;
long objectArea;
int frameWidth;
int frameHeight;
double yspeed = 0.0; //initial value of x and y speed to 0
double xspeed = 0.0;
char pixyMoveDir; //sets the direction in which robot is moving in PIXY mode
unsigned long pixytime = 0; //int to store count of how long Photo-Bot has been tracked for
double dist; //define a variable for distance to subject
double desiredDistance = 1.5; //set a value of distance desired, in metres

//Filtering Initialisation
//Kalman Filter for Pixy Tracking
// kalman variables
float R = 0.000324211;  // R= Variance of Raw Sensor Data. variance determined using excel and reading samples of raw sensor data
float Q1 = 1e-3; // Q=process noise covariance
float Pc1 = 0.0;
float G1 = 0.0; //G = Kalman Gain
float P1 = 1.0;
float Xp1 = 0.0;
float measuredValue = 0.0; //Z= measured value
double KDistance1 = 0.0;

#define BLYNK_USE_DIRECT_CONNECT

// You could use a spare Hardware Serial on boards that have it (like Mega)
#include <SoftwareSerial.h>
//SoftwareSerial DebugSerial(2, 3); // RX, TX

//PID Functions

//PID for movement(PIXY)
PID myPID(&KDistance1, &yspeed, &desiredDistance, 150.0, 65.0, 15.0, REVERSE);
//PID for movement (RC)
PID myPID1(&leftEncoderSpeed, &PWMLSpeed, &desiredLSpeed, 25.0, 150.0, 4.5, DIRECT); //crucial to define the first 3 things as doubles
PID myPID2(&rightEncoderSpeed, &PWMRSpeed, &desiredRSpeed, 25.0, 150.0, 4.5, DIRECT);


//#define BLYNK_PRINT DebugSerial
#include <BlynkSimpleSerialBLE.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
//char auth[] = "5c5ed91503384efeb34521691400c068"; //S7, Isabelle's Phone
char auth[] = "ccd11955dec54b7e99a166b6e517204f"; //HC's Phones

void setup()
{
  //BLYNK SETUP
  // Debug console
  //  DebugSerial.begin(9600);
  //  DebugSerial.println("Waiting for connections...");
  // Blynk will work through Serial
  // 9600 baud rate used for HC-05 and works
  Serial.begin(9600);
  Blynk.begin(Serial, auth); //begin using Blynk

  // initialising motor control pins
  pinMode(leftMotorPWM, OUTPUT); //control left motor speed
  pinMode(rightMotorPWM, OUTPUT); //control right motor speed
  pinMode(leftMotorIN3, OUTPUT);  //left motors forward
  pinMode(leftMotorIN4, OUTPUT);  //left motors reverse
  pinMode(rightMotorIN1, OUTPUT);  //right motors forward
  pinMode(rightMotorIN2, OUTPUT);  //right motors reverse

  // initialising motor encoder pins
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  //set up of ultrasonic sensor pins
  pinMode(leftSensorTP, OUTPUT); //sets TP of leftSensor as output
  pinMode(leftSensorEP, INPUT); //sets EP of leftSensor as input
  pinMode(middleSensorTP, OUTPUT); //sets TP of middleSensor as output
  pinMode(middleSensorEP, INPUT); //sets EP of middleSensor as input
  pinMode(rightSensorTP, OUTPUT); //sets TP of rightSensor as output
  pinMode(rightSensorEP, INPUT); //sets EP of rightSensor as input

  // initialize hardware interrupts and bluetooth interface
  attachInterrupt(leftEncoderInterrupt, leftEncoderEvent, CHANGE);
  attachInterrupt(rightEncoderInterrupt, rightEncoderEvent, CHANGE);
  leftCount = 0;
  rightCount = 0;

  // initialize pixy
  pixy.init();
  frameWidth = pixy.frameWidth; //sets the frame width to the pixy camera
  frameHeight = pixy.frameHeight; //sets the frame height to the pixy camera

  // Set PID functions to run automatically
  myPID.SetMode(AUTOMATIC); //PID settings for PixyTracking
  myPID1.SetMode(AUTOMATIC); //PID settings for RC Left Motor
  myPID2.SetMode(AUTOMATIC); //PID Settings for RC Right Motor

  myPID.SetOutputLimits(-255, 255);
  myPID1.SetOutputLimits(-255, 255);
  myPID2.SetOutputLimits(-255, 255);
}

void loop()
{
  Blynk.run();
  checkAllSensors();
  checkEncoders();

  //auto-Forward mode
  if (RCMode == 1) {
    if (autoForwardMode == 1) {
      //      Serial.println("autoForward");
      if (objectTooClose == 1) {
        stop(); //Photo-Bot comes to immediate stop as object is within critical distance
        //        Serial.print(" CRITICAL D ");
      }
      else if (objectTooClose == 0) {
        if ((leftObjectDetected) == 1) { //objects detected within warning distance in all directions
          smoothStop();
          //          Serial.print(" WARNING D ");
        }
        else if ((middleObjectDetected) == 1) { //objects detected within warning distance in all directions
          smoothStop();
          //          Serial.print(" WARNING D ");
        }
        else if ((rightObjectDetected) == 1) { //objects detected within warning distance in all directions
          smoothStop();
          //          Serial.print(" WARNING D ");
        }
        else {
          moveForward();
          //          setLSpeed(100);
          //          setRSpeed(100);
          //          Serial.print("TRYING");
        }
      }
    }
  }

  // Tracking mode
  if (RCMode == 2) { //use if, not while cos once its in while it can't seem to respond to blynk commands at all
    //get blocks (PIXY receive data)

    pixy.ccc.getBlocks();
    int tempxpos = 0; //temp variable for summing object x position
    int tempOW = 0; //temp variable for summing object width
    int tempOH = 0; //temp variable for summing object height

    // Only run if blocks are detected!
    if (objectTooClose == 0) //make sure nothing in critical distance first (i.e. if something too close, don't move
    {
      if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age > 15) //if there are blocks, and they appear on the screen for a more than a brief moment
      {
        pixytime = millis(); //record the last time that the pixy was able to find the object
        for (char j = 1 ; j <= 3; j++) //do an average over multiple readings, but honestly its quite accurate in general (averaging doesn't do much)
        {
          pixy.ccc.getBlocks();
          tempxpos = tempxpos + pixy.ccc.blocks[0].m_x;
          tempOW = tempOW + pixy.ccc.blocks[0].m_width;
          tempOH = tempOH + pixy.ccc.blocks[0].m_height; //add 3 readings together

          delay(50);
        }
        //take the average of the 3 readings to obtain more accurate readings
        xpos = tempxpos / 3;
        objectWidth = tempOW / 3;
        objectHeight = tempOH / 3;

        YEvaluation(); //evaluates the y error
        XEvaluation(); //evaluates the x error
        if ( middleObjectDetected == 1) { //same warning distance code as the joystick
          yspeed = 1; //Photo-Bot not able to move forward, motion is limited to only spinning left or right
        }
        else if (leftObjectDetected == 1) {
          if ((xspeed < 0) && (yspeed > 0)) { //Photo-Bot instructed to veer left
            yspeed = 1; //Photo-Bot spins left
          }
        }
        else if (rightObjectDetected == 1) {
          if ((xspeed > 0) && (yspeed > 0)) { //Photo-Bot instructed to veer right
            yspeed = 1; //Photo-Bot spins right
          }
        }

        rightPWMSpeed = (yspeed - xspeed) / 1; //multiplier to scale speed as necessary
        leftPWMSpeed = (yspeed + xspeed) / 1;
        //        Serial.print("R = ");
        //        Serial.println(rightPWMSpeed);
        //        Serial.print("L = ");
        //        Serial.println(leftPWMSpeed);
        setLSpeed(leftPWMSpeed);
        setRSpeed(rightPWMSpeed);
      }
      else { //if object not found
        unsigned long pixylost = millis(); //variable to show how long it has been since PIXY lost the subject
        if ((pixylost - pixytime) >= 5000) {
          smoothStop();
          //          Serial.print(" PIXY IS LOST ");
        }
        else {
          //use the previous readings
          //          int tempyspeed = yspeed; //give a placeholder variable

          //obstacle detection
          //          if ( middleObjectDetected == 1) { //same warning distance code as the joystick
          //            yspeed = 1; //Photo-Bot not able to move forward, motion is limited to only spinning left or right
          //          }
          //          else if (leftObjectDetected == 1) {
          //            if ((xspeed < 0) && (yspeed > 0)) { //Photo-Bot instructed to veer left
          //              yspeed = 1; //Photo-Bot spins left
          //            }
          //          }
          //          else if (rightObjectDetected == 1) {
          //            if ((xspeed > 0) && (yspeed > 0)) { //Photo-Bot instructed to veer right
          //              yspeed = 1; //Photo-Bot spins right
          //            }
          //          }
          //
          //          rightPWMSpeed = (yspeed - xspeed) / 1; //multiplier to scale speed as necessary
          //          leftPWMSpeed = (yspeed + xspeed) / 1;
          //          setLSpeed(leftPWMSpeed);
          //          setRSpeed(rightPWMSpeed);
          smoothStop();
          //          yspeed = tempyspeed; //reset the y speed so it can check again and move forward if able
        }
      }
      //            delay(200);
    }
  }
  delay(10);
}

void checkEncoders() {
  time = millis();
  SampleTime = time - Previoustime;
  Previoustime = time;

  //since encoder feedback resolution is 374 for 1 revolution (shaft revolution), AND gear reduction ratio is 34, 1 encoder count== 1/(374/34) Revolution of wheel
  leftchange = leftCount - oldleftCount;
  rightchange = rightCount - oldrightCount;
  leftWheelRev = leftchange / (374 * SampleTime / 1000); //left wheel RPS
  rightWheelRev = rightchange / (374 * SampleTime / 1000); //right wheel RPS
  leftEncoderSpeed = leftWheelRev * 2 * 3.1415 * 0.05; //velocity=r*w (radius of wheel is 5cm)
  rightEncoderSpeed = rightWheelRev * 2 * 3.1415 * 0.05; //velocity=r*w (radius of wheel is 5cm)
  oldleftCount = leftCount;
  oldrightCount = rightCount;
  //    Serial.print(leftCount);
  //    Serial.print(" ");
  //    Serial.print(rightCount);
  //      Serial.print(" LChange Ticks");
  //      Serial.print(leftchange);
  //      Serial.print(" RChange Ticks");
  //      Serial.print(rightchange);
  //Printing of values to Serial Monitor for debugging
  //  Serial.print("SampleTime: ");
  //  Serial.print(SampleTime);
  //  Serial.print( "LeftSpeed(m/s):");
  //
  //  Serial.print(0);
  //  Serial.print(" ");
  //  Serial.print(1);
  //  Serial.print(" ");
  //  Serial.print(2);
  //  Serial.print(" ");
  //    Serial.print(leftEncoderSpeed);
  //    //  Serial.print( "RightSpeed(m/s):");
  //    Serial.print(" ");
  //    Serial.println(rightEncoderSpeed);
  //  Serial.println();
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}


void moveForward() {           //move forward(all motors rotate in forward direction at a given speed in m/s)
  desiredLSpeed = robotSpeed;
  desiredRSpeed = robotSpeed;

  myPID1.Compute();
  myPID2.Compute(); //does the PID calculation

  //Both motors at 1.5m/s
  setLSpeed(PWMLSpeed);
  setRSpeed(PWMRSpeed);
  delay(80); //important to keep it stable

}


void spinLeft() {     //turn left (left side motors rotate in forward direction, right side motors rotates in reverse direction)
  for (int i = 70; i <= 180; i++) { //increase speed incrementally
    setLSpeed(-i);
    setRSpeed(i);
    delay(8);
  }
}

void spinRight() {     //turn right (right side motors rotate in forward direction, left side motors rotates in reverse direction)
  for (int i = 70; i <= 180; i++) { //increase speed incrementally
    setLSpeed(i);
    setRSpeed(-i);
    delay(8);
  }
}

void stop() {     //STOP (all motors stop)

  setLSpeed(1); //set motor speed to 1 cos its not high enough to get it actually moving, but for some reason when its 0 it goes nuts
  setRSpeed(1);
  AFMaxSpeedReached = 0; // resets the AutoForward Max Speed placeholder
}

void smoothStop() {
  while ((abs(leftPWMSpeed) && abs(rightPWMSpeed)) > 30) { //reduced to 30 for the small robot
     setLSpeed(leftPWMSpeed/1.25); //arbitrary constant to decrease left motor speed proportionately
    setRSpeed(rightPWMSpeed/1.25); //arbitrary constant to decrease right motor speed proportionately
    delay(10);
  }
  stop();
}

int setRSpeed(int rightPWMSpeed) {
  if (rightPWMSpeed >= 0) {
    digitalWrite(rightMotorIN1, LOW);
    digitalWrite(rightMotorIN2, HIGH);
    analogWrite(rightMotorPWM, rightPWMSpeed);
    //    Serial.print(" R = ");
    //    Serial.print(rightPWMSpeed);
  }
  else if (rightPWMSpeed < 0) {
    digitalWrite(rightMotorIN1, HIGH);
    digitalWrite(rightMotorIN2, LOW);
    analogWrite(rightMotorPWM, abs(rightPWMSpeed));
    //    Serial.print(" R = ");
    //    Serial.print(rightPWMSpeed);
  }
}
int setLSpeed(int leftPWMSpeed) {
  if (leftPWMSpeed >= 0) {
    digitalWrite(leftMotorIN3, LOW);
    digitalWrite(leftMotorIN4, HIGH);
    analogWrite(leftMotorPWM, leftPWMSpeed);
    //    Serial.print(" L = "); //print the speeds of each motor to Serial Monitor for debugging
    //    Serial.print(leftPWMSpeed);
  }
  else if (leftPWMSpeed < 0) {
    digitalWrite(leftMotorIN3, HIGH);
    digitalWrite(leftMotorIN4, LOW);
    analogWrite(leftMotorPWM, abs(leftPWMSpeed));
    //    Serial.print(" L = "); //print the speeds of each motor to Serial Monitor for debugging
    //    Serial.print(leftPWMSpeed);
  }
}
void checkAllSensors() {
  if (millis() >= pingTimer1) {
    pingTimer1 += pingSpeed; // Make sensor 1 fire again 150ms later (pingSpeed)
    middleDistanceCm = middleSensor.ping_cm();
    if (middleDistanceCm == 0) { //objects further than maxDetectionD will return 0
      middleDistanceCm = maxDetectionD;
    }
  }
  if (millis() >= pingTimer2) {
    pingTimer2 = pingTimer1 + (pingSpeed / 3); // Make sensor 2 fire again 50ms after middle sensor fires
    leftDistanceCm = leftSensor.ping_cm();
    if (leftDistanceCm == 0) { //objects further than maxDetectionD will return 0
      leftDistanceCm = maxDetectionD;
    }
  }
  if (millis() >= pingTimer3) {
    pingTimer3 = pingTimer1 + (pingSpeed * 2 / 3); // Make sensor 3 fire again 100ms after middle sensor fires; 50ms after left sensor
    rightDistanceCm = rightSensor.ping_cm();
    if (rightDistanceCm == 0) { //objects further than maxDetectionD will return 0
      rightDistanceCm = maxDetectionD;

    }
  }
  if ((middleDistanceCm) < criticalD) { //if objects are detected within critical distance in any direction, Photo-Bot stops IMMEDIATELY
    objectTooClose = 1;
    //    Serial.println(" CRITICAL M");
    //    Serial.print(" MID DIST ");
    //    Serial.println(middleDistanceCm);
  }
  else if ((leftDistanceCm) < criticalD) { //if objects are detected within critical distance in any direction, Photo-Bot stops IMMEDIATELY
    objectTooClose = 1;
    //    Serial.println(" CRITICAL L");
    //    Serial.print(" LEFT DIST ");
    //    Serial.println(leftDistanceCm);
  }
  else if ((rightDistanceCm) < criticalD) { //if objects are detected within critical distance in any direction, Photo-Bot stops IMMEDIATELY
    objectTooClose = 1;
    //    Serial.print(" CRITICAL R");
    //    Serial.print(" RIGHT DIST ");
    //    Serial.println(rightDistanceCm);
  }
  else {
    objectTooClose = 0;
  }

  if (middleDistanceCm < warningD) {
    middleObjectDetected = 1; //sub-Mode middleObjectDetected is triggered (affects decision of robot movement in main loop)
  }
  else {
    middleObjectDetected = 0;
  }
  if (leftDistanceCm < warningD) {
    leftObjectDetected = 1; //sub-Mode leftObjectDetected is triggered (affects decision of robot movement in main loop)
  }
  else {
    leftObjectDetected = 0;
  }
  if (rightDistanceCm < warningD) {
    rightObjectDetected = 1; //sub-Mode rightObjectDetected is triggered (affects decision of robot movement in main loop)
  }
  else {
    rightObjectDetected = 0;
  }
  //
  //  Serial.print(" Left Object Distance:");
  Serial.print(leftDistanceCm);
  Serial.print(" ");
  //  Serial.print("cm");


  //  Serial.print(" Middle Object Distance:");
  Serial.print(middleDistanceCm);
  Serial.print(" ");
  //  Serial.print("cm");


  //  Serial.print(" Right Object Distance:");
  Serial.println(rightDistanceCm);
  //  Serial.println("cm");
}


BLYNK_WRITE(V0) //Stop Function (button)
{
  int pinValue = param.asInt();
  if (pinValue == 1) { //overrides autoforward mode
    smoothStop();
    autoForwardMode = 0;
    //    Serial.print("STOP");
  }
}

BLYNK_WRITE(V1) //joystick function (button)
{ if (RCMode == 1)
  {
    if (objectTooClose == 0) {
      int xposition = (param[0].asInt()) ; //find the x position from joystick
      int yposition = (param[1].asInt() + 1) ; //find the y position from joystick

      // Add a 1 to both so that default is 1,1. If default is 0,0 it tends to mess up

      if ( middleObjectDetected == 1) {
        if (yposition > 0) {//only if user trying to move Robot forward (i.e. let the user move it backward)
          yposition = 1; //Photo-Bot not able to move forward, motion is limited to only spinning left or right
        }
      }
      else if (leftObjectDetected == 1) {
        if ((xposition < 0) && (yposition > 0)) { //Photo-Bot instructed to veer left
          yposition = 1; //Photo-Bot spins left
        }
      }
      else if (rightObjectDetected == 1) {
        if ((xposition > 0) && (yposition > 0)) { //Photo-Bot instructed to veer right
          yposition = 1; //Photo-Bot spins right
        }
      }

      if (yposition > 0) {

        leftPWMSpeed = (yposition + xposition) / 1; //map the appropriate speed based on direction of joystick;
        rightPWMSpeed = (yposition - xposition) / 1;
      }
      else {//reverse the order for the backward motion
        yposition = yposition / 0.75; //increase backward effect
        xposition = xposition / 2; //decrease turning effect - should make it easier to control
        leftPWMSpeed = (yposition - xposition) / 1; //map the appropriate speed based on direction of joystick;
        rightPWMSpeed = (yposition + xposition) / 1; //change divider to 1 when using on Photo-Bot


      }
      setLSpeed(leftPWMSpeed);
      setRSpeed(rightPWMSpeed);
      //      delay(50);
      //      Serial.print(" L = ");
      //      Serial.println(leftPWMSpeed);
      //      Serial.print(" R = ");
      //      Serial.println(rightPWMSpeed);

    }
    else {
      //      Serial.println(" CRITICAL JOYSTICK ");
      stop(); //stop immediately if it encounters some object at critical distance
    }
  }
}


BLYNK_WRITE(V2) //spin right function (button)
{ if (RCMode == 1)
  {
    if (objectTooClose == 0) { //Photo-Bot allowed to spin unless object within critical distance
      int pinValue = param.asInt(); //when button is pressed
      if (pinValue == 1) {
        spinRight();
        //        Serial.print("SPINRIGHT");
      }
      else if (pinValue == 0) { //when released, stop
        smoothStop();
        //        Serial.print("STOP");
      }
    }
    else {
      smoothStop();
    }
  }
}

BLYNK_WRITE(V3) //spin left function (button)
{ if (RCMode == 1)
  { if (objectTooClose == 0) { //Photo-Bot allowed to spin unless object within critical distance
      int pinValue = param.asInt(); //when button is pressed
      if (pinValue == 1) {
        spinLeft();
        //        Serial.print("SPINLEFT");
      }
      else if (pinValue == 0) { //when released, stop
        smoothStop();
        //        Serial.print("STOP");
      }
    }
    else {
      smoothStop();
    }
  }
}

BLYNK_WRITE(V4) //auto forward function (Switch)
{ if (RCMode == 1)
  { int pinValue = param.asInt(); //when switch is on
    if (pinValue == 0) { //when turned off, stop no matter what the sensors say
      smoothStop();
      autoForwardMode = 0; //autoForward Mode set to off
      //      Serial.print("STOP");
      //      Serial.print("Mode:");
      //      Serial.println(autoForwardMode);
    }
    if ((leftObjectDetected) == 0)
    { if ((middleObjectDetected == 0))
      { if ((rightObjectDetected) == 0)
        { //no objects detected within warning distance in all directions
          if (pinValue == 1) {
            autoForwardMode = 1; //autoForwardMode set to on
            //            Serial.print("Mode:");
            //            Serial.println(autoForwardMode);
          }
        }
        else {
          smoothStop();

        }
      }
      else {
        smoothStop();

      }
    }
    else {
      smoothStop();

    }

  }

}

BLYNK_WRITE(V5) //switch modes between PIXY and RC
//only STOP works in PIXY Mode (Mode 2)
{
  int pinValue = param.asInt();
  if (pinValue == 1) {
    smoothStop();
    //    Serial.print("STOP");
    RCMode = 2;
  }
  else if (pinValue == 0) {
    smoothStop();
    //    Serial.print("STOP");
    RCMode = 1;
  }
}

BLYNK_WRITE(V6) //Speed control function (slider)
{
  robotSpeed = param.asDouble(); //when slider is released
  //  Serial.print(" SLIDER= ");
  //  Serial.println(robotSpeed); // to test how often it updates? is it jamming the system
}

void YEvaluation()
{
  desiredDistance = 1.5; //set a value for the distance desired, in metres
  float bufwidth = 0.1; // set a buffer region
  float minD = (desiredDistance - bufwidth); //set a desired minimum reasonable filming distance
  float maxD = (desiredDistance + bufwidth);

  objectArea = abs(objectWidth * objectHeight); //must have abs cos sometimes it gives negative values
  //  Serial.print("Object Area = ");
  //  Serial.println(objectArea);
  //Based on detection up to 1.5m, plus 0.5m for buffer (anything over this, just set a uniform fast speed)
  //need to calibrate again; this was only using a laptop cover

  dist = (1.0 / 1.085) * (log(45006.0 / objectArea)); //based on testing
  PixyKalmanFilter(dist); //applies Kalman Filtering to distance
  //  Serial.print(KDistance1*100);
  //  Serial.print(" ");
  //  Serial.print("Distance = ");
  //    Serial.print(dist * 100); //multiply by 100 so it is visible
  //    Serial.print(" ");
  //  Serial.print(dist);
  //  Serial.print(" ");
  //  Serial.println(yspeed);
  //  float y_error = dist - desiredDistance; //find a y error
  float y_error = KDistance1 - desiredDistance; //find a y error
  //  Serial.print("y error = ");
  //  Serial.println(y_error);

  myPID.Compute();
  delay(80);
  //  Serial.print("y speed = ");
  //  Serial.print(" ");
  //  Serial.println(yspeed);

}

void XEvaluation() {

  float x_error = xpos - (frameWidth / 2); //set x error to be between xposition of object and middle of frame
  //base on test on laptop cover, the error ranges from approx -100 (left of bot) to 100 (right of bot)
  //  Serial.println(x_error);
  if (abs(x_error) > 10) {//have a buffer zone
    xspeed = x_error / 2.25; //based on calibration factor of 3
  }
  else {
    xspeed = 0;
  }
  //  Serial.print("xspeed = ");
  //  Serial.println(xspeed);
}

double PixyKalmanFilter(double measuredValue) {
  Xp1 = KDistance1;
  Pc1 = P1 + Q1; //Pc is the covariance matrix priori of the error estimated. Q is the process noise covariance determined arbitrarily
  G1 = Pc1 / (Pc1 + R);  // G is the Kalman gain. R is the measurement covariance matrix determined by measuring variance of static noise measurements using Excel
  KDistance1 = G1 * (measuredValue - Xp1) + Xp1; // the kalman estimate of the sensor voltage
  P1 = (1 - G1) * Pc1; //Calculate  new state estimate error
}
