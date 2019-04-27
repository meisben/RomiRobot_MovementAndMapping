/*
 * ~~~~~Prerequisites~~~~~

   This code uses the 'Guassian' library, download it here: https://github.com/ivanseidel/Gaussian

   ~~~~~Details~~~~~~~~~~~~

   Authors: Natalia Gonzalez Cadiente, Bruno Castro Ibarburu, Nawid Keshtmand, Ben Money-Coomes
   Date: 27/4/19
   Purpose: Implement mapping movement and mapping behaviour for the Romi robot
   References: Uses implementation as described at https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

   ~~~~Version Control~~~~~
   
   |------> BRANCH of v4.45e1 - Working code used in CW1
   CW2_v2.30 - [Checkpoint] This is the code that will be used for the formative assessment
   CW2_v2.30 - [Checkpoint] With bluetooth constant print of [Time(s),PoseX(mm),PoseY(mm),PoseTheta(deg)]
   CW2_v2.32 --->> SCRAP: I am blocking the obstacle code to pyqtgraph connection
   CW2_v2.33 - Working on including kalman filter

*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Library Includes.                                                             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "encoders.h"
#include "pid.h"
#include "motors.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "button.h" // note that this is only used for the button press so that i don't have to deal with debouncing etc,      // https://github.com/JChristensen/JC_Button 
//------Added from baseline
#include "pins.h"
#include "utils.h"
#include "irproximity.h"
#include "mapping.h"
#include <Wire.h>
//#include "imu.h"
//#include "magnetometer.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Definitions                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define LOOP_DELAY 50

//Pin definitions for motor
#define R_PWM_PIN 10
#define R_DIR_PIN 16
#define L_PWM_PIN  9
#define L_DIR_PIN 15

//Serial configuration
#define BAUD_RATE 9600

//Map border (area around outside of map that ROMI will not enter)
#define MAP_BORDER 300

//--------Definitions for line following-------------

#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor

// -------Definitions for buttons and buzzer
const char buzzerPin = 6; //buzzer
const char buttonPinA = 14; //PinA
const char buttonPinB = 30;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Variables for counting time.                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//--------Variables and setup for timers-------------
unsigned long general_timer;
unsigned long time_of_read_lineStraightOrTurning;
unsigned long time_of_read_lineStraightOrTurningPID;
unsigned long time_of_read_lineStraightOrTurningMotors;
unsigned long start_time; // this is used to print a timestamp for bluetooth data

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Global Variables.                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//----Variable for PID inputs
//[DELETE] float target = 2000; //Target encoder count

//----Variable for disabling distance sensor (e.g. for experiments that don't need one!)
//[DELETE] bool distanceSensorDisabled = false;

//-----Variables for keeping track of 'Romi State' and whether a movement sequence is complete

bool movementSequenceComplete = false; //to determine in our main loop whether the movment sequence is complete
unsigned char stateRobot = 0; //keeps track of current robot state

// Variables for the target of encoders during turn or straight line trajectories
long R_EncoderCurrentTarget = 0; // note these need to be updated in functions with actual values
long L_EncoderCurrentTarget = 0;

//------ Variable for buttons
bool buttonStateA = HIGH;

//------ Variable for counting the number of times the robot has itterated between 'Random Walk' and 'Search'
byte stateCounter = 0;

//Variables for PIDs-------------------
//wheel
float W_Kp_pose = 0.4; //Proportional gain for position controller //
float W_Kd_pose = 0.5; //Derivative gain for position controller // small //
//float W_Ki_pose = 0; //Integral gain for position controller  //really small //

//Angle controller (controls the 'slippage' between wheels)
float A_Kp_pose = 0.2; //Proportional gain for angle controller //


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Class Instances.                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

Kinematics romiKinematics(count_e0, count_e1); //initialise kinematics model with current encoder counts

//-------------PIDs
//right wheel
PID rightPose(W_Kp_pose, W_Kp_pose, 0); //Position controller for right wheel position
PID leftPose(W_Kp_pose, W_Kp_pose, 0); //Position controller for left wheel position
//Angle controller (controls the 'slippage' between wheels)
PID anglePoseStraight(A_Kp_pose, 0, 0); //Angle controller for ROMI wheels (keeps wheels in sync).

//------ Create two motor classes
Motor rightMotor; // set right motor
Motor leftMotor;  // set left motor

//------Classes for line sensor
Line_Sensor lineCentre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor

//Initialise button
Button btnA(buttonPinA);       // define the button on pinA using button class
Button btnB(buttonPinB);

//------Classes from CW2 baseline
SharpIR       DistanceSensor(SHARP_IR_PIN); //Distance sensor
//Imu           Imu; //Class for the gyroscope (IMU)
//Magnetometer  Mag; // Class for the magnetometer
Mapper        Map; //Class for representing the map


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Start of main program                                                          *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup()
{
  //Setup PinModes
  pinMode(LED_BUILTIN, OUTPUT); //set LED default as output
  pinMode(buzzerPin, OUTPUT); //configure buzzerpin as output

  //Setup PID maximums (where applicable)
  rightPose.setMax(22);
  leftPose.setMax(22);
  anglePoseStraight.setMax(30);

  //Set PID thresholds (this is how they return a stop value when goal is reached)
  rightPose.setThresholdActive(1, 4);
  leftPose.setThresholdActive(1, 4);

  //Assign motor pins
  rightMotor.setPins(R_PWM_PIN, R_DIR_PIN); //set right motor pins
  leftMotor.setPins(L_PWM_PIN, L_DIR_PIN); // set left motor pins

  // These two function set up the pin change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Initialise the Serial communication so that we can inspect the values of our encoder using the Monitor.
  Serial.begin( BAUD_RATE ); //used for wired connection to computer
  Serial1.begin(BAUD_RATE); //used for bluetooth

  //delay to give time to start serial monitor
  delay(1000);

  //Print statement and play tone to indicate boot
  Serial.println("Romi boot");
  play_tone(60, 50);
  delay(100);
  play_tone(60, 50);

  // initialize the button objects
  btnA.begin();
  btnB.begin();


}

void loop()
{
  //-------------Code for button functionality --------------------------------------
  //(useful for determining robot state and debugging
  //----------------------------------------------------------------------------------

  btnA.read();               // read the button
  btnB.read();               // read the button

  if (btnA.wasReleased())    // if the button was released, play a tone
  {
    play_tone(150, 100);
    stateRobot++;
    Serial.print("stateRobot = ");
    Serial.println(stateRobot);
  }
  if (btnB.wasReleased())    // if the button was released, print the map
  {
    delay(250);
    Map.printMap();
    Serial.println("");
    Serial.print("Kinematics X =");
    Serial.println(romiKinematics.getPoseX());
    Serial.print("Kinematics Y =");
    Serial.println(romiKinematics.getPoseY());
    Serial.print("Kinematics Theta =");
    Serial.println(romiKinematics.getPoseTheta());
    Serial.println("");
    Serial.println("waiting for button A press");
    romiKinematics.resetEncoderCount();
    stateRobot = 4;
    delay(5000);
  }

  //----------------------------------------------------------------------

  //---------------Don't change this code---------------------------------
  switch (stateRobot) {
    case 1:
      delay(250);

      lineCentre.calibrate();
      play_tone(150, 50); // so user can understand that calibration has finished
      Map.printMap();
      stateRobot++;

      Serial.println("waiting for button A press");

      break;

    case 3:
      Serial.println("Map Erased - Mapping Started");

      Map.resetMap();
      start_time = millis(); //set the time the robot starts moving
      stateRobot++;
      play_tone(150, 50);

      Serial.print("stateRobot = "); //for debugging
      Serial.println(stateRobot);

      delay(250);
      break;

    //---------------Can make changes from here  - RANDOM walk and Exploration behaviour ---------------------------------

    case 4://-->> RANDOM WALK!

      Serial.print("Romi doing random walk"); //for debugging

      doRandomWalk(true); //doRandomWalk until obstacle is detected
      DistanceSensor.obstacleDetected = false; //reset obstacle detected in the IR  library, note this is necessary to allow the ROMI to search for a new unexplored grid tile
      stateCounter++; //keeps track of how many forward movements have been completed by the Romi
      stateRobot = 5; //move to search function

      Serial.print("stateRobot = "); //for debugging
      Serial.println(stateRobot);

      break;

    case 5://-->> SEARCH FOR UNEXPLORED TILE

      //Move to a new unexplored grid tile
      bool myObstacleDetected = true; //this variable is used to denote when an obstacle is NOT detected!
      
      while (myObstacleDetected == true) {
        Serial.print("Romi moving to unexplored tile"); //for debugging
        Map.returnClosestUnexploredCoOrdinates(romiKinematics.getPoseX(), romiKinematics.getPoseY()); //return position of a random unexplored grid tile
        myObstacleDetected = goToPoint(Map.targetXCoordinate, Map.targetYCoordinate); //move to that grid tile position
        delay(250);
        DistanceSensor.obstacleDetected = false; //reset obstacle detected in the IR mapping library, note this is necessary to allow the ROMI to search for a new unexplored grid tile
        delay(250);
      }
      //Then turn towards the centre of the map (to maximise utility of random walk)
      turnTowardsPoint((MAP_X / 2), (MAP_Y / 2));

      stateCounter++; //keeps track of how many forward movements have been completed by the Romi

      if (stateCounter < 3) { //if less than 3 forward movements completed
        stateRobot = 4; //move to random walk function
      }
      else {
        stateRobot = 99; //stop and wait for button press to print map
        stateCounter = 0; //reset number of forward movements
        play_tone(150, 500); //play a tone
        Serial.println("Press button B to print map via serial port");
      }
      Serial.print("stateRobot = "); //for debugging
      Serial.println(stateRobot);
      break;


      //--------------- This is an example of how you can make the Romi move straight or turn during an experiment ---------------------------------
      //--------------- If you want to use these statements you can uncomment this code and comment the code statements above ----------------------

//    case 4://-->> TRAVEL IN STRAIGHT LINE!
//      delay(250);
//      Serial.print("stateRobot test = ");
//      Serial.println(stateRobot);
//      
//      moveStraightLine(-5000, 0); //move forwards for 5000 encoder counts, with obstacle detection turned off
//      stateRobot++;
//      Serial.print("stateRobot = ");
//      Serial.println(stateRobot);
//      break;
//
//    case 5://-->> TURN ON THE SPOT!
//      delay(250);
//      moveTurnOnSpot(90, 0); //turn on spot 90 degrees clockwise, with line detection turned off
//      stateRobot++;
//      Serial.print("stateRobot = ");
//      Serial.println(stateRobot);
//      break;
//
//    case 6://-->> RANDOM WALK!
//      delay(250);
//      doRandomWalk(false); //doRandomWalk forever (because obstacle detection is off)    
//      break;
//
//    default:
//      //pass
//      break;

     
  } // end of switch case statement

  delay(LOOP_DELAY);
  
} // end of main loop

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Functions                                          *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//-----------------Functions for MOVEMENT!--------------------------

// -------STRAIGHT LINE FUNCTION ----------
// Function to move in a straight line for a specified distance
// If findObstacle = true then the function will exit when an obstacle is detected

bool moveStraightLine(long distance, bool findObstacle) 
{
  //note distance is the same as counts in the function currently

  //Set local variables
  bool finishedMovement = false;

  R_EncoderCurrentTarget = count_e0 + distance; //set current target - have just changed from target to encoder value plus additional value
  L_EncoderCurrentTarget = count_e1 + distance;

  //set variable for finding a line
  bool myObstaclePresent = false;

  while (finishedMovement == false) { //loop while the movement is still underway

    // Work out how many milliseconds have gone passed by subtracting our two timestamps.  time_now will always be bigger than the time_of_read (except when millis() overflows after 50 days).
    unsigned long elapsed_time_lineStraightOrTurning = millis(); - time_of_read_lineStraightOrTurning;

    //Do some stuff if you're looking to find a line and stop (CW1, not applicable for CW2)
    if (findObstacle == true)
    {
      // See if 50 milliseconds have ellapsed If not, this block is skipped.
      if ( elapsed_time_lineStraightOrTurning > 50 )
      {
        // Since 50ms elapsed, we update our timestamp so that another 50ms is needed to pass.
        time_of_read_lineStraightOrTurning = millis();

        if (DistanceSensor.obstacleDetected == true)
        {
          rightMotor.setSpeed(0);
          leftMotor.setSpeed(0);
          finishedMovement = true;
          myObstaclePresent = true;
          break;
        }
      }

    }
    //Update PID control and get output for right motor and left motor
    float outputRightMotor = rightPose.update(R_EncoderCurrentTarget, count_e0);
    float outputLeftMotor = leftPose.update(L_EncoderCurrentTarget, count_e1);

    //Update PID control and get output for angle // note left is slave motor, right is master
    float outputAnglePoseStraight = anglePoseStraight.update(R_EncoderCurrentTarget - L_EncoderCurrentTarget, (count_e0 - count_e1));

    //get parameters to indicate whether PID has achieved goal for left and right motor (distance)
    bool R_thresholdReached = rightPose.getThresholdReached();
    bool L_thresholdReached = leftPose.getThresholdReached();
    //int instantaneousAvgError = rightPose.getInstananeousAverageAbsError(); //statement for optional debugging

    //Do mapping (CW2)
    doMapping();

    //Send demands to Motors (Depending on status of reaching goal)
    if (R_thresholdReached == 1) {          //if Right motor has reached goal
      rightMotor.setSpeed(0);               //Set right motor speed to zero
      if (L_thresholdReached == 1) {        //if Left motor has reached goal
        leftMotor.setSpeed(0);              //set motor speed to zero
        finishedMovement = true;
      }
      else {
        leftMotor.setSpeed(outputLeftMotor - outputAnglePoseStraight); //else keep left motor moving towards goal
        romiKinematics.updateStraightMotion(count_e0, count_e1); //update the kinematics function
      }
    }
    else {
      rightMotor.setSpeed(outputRightMotor);
      leftMotor.setSpeed(outputLeftMotor - outputAnglePoseStraight); //else keep both motors moving towards goals
      romiKinematics.updateStraightMotion(count_e0, count_e1); //update the kinematics function
    }
    //printDebugStatements(outputRightMotor, outputLeftMotor, outputAnglePoseStraight); // for debugging
    delay(LOOP_DELAY); //the code doesn't work without this, not sure why? ... question to instructors !

    //debug statements
    //    Serial.print("encoder values   R:");
    //    Serial.print( count_e0 );
    //    Serial.print(", L:");
    //    Serial.print( count_e1 );
    //    Serial.print("  R_EncoderTarget = ");
    //    Serial.print(R_EncoderCurrentTarget);
    //    Serial.print("  L_EncoderTarget = ");
    //    Serial.println(L_EncoderCurrentTarget);
    //    Serial.print("checkIfRomiIsOutsideMap() = ");
    //    Serial.print(checkIfRomiIsOutsideMap());
//    Serial.print("  Kinematics X = "); //note i should move this all into the debug function !
//    Serial.print(romiKinematics.getPoseX());
//    Serial.print("  Kinematics Y = ");
//    Serial.print(romiKinematics.getPoseY());
//    Serial.print("  Kinematics Theta = ");
//    Serial.println(romiKinematics.getPoseTheta());

    unsigned long elapsed_time_general_timer = millis() - general_timer;

    if ( elapsed_time_general_timer > 1000 )
      {
        // update timestamp
        general_timer = millis();

        Serial1.print((millis()-start_time)/1000);
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseX());
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseY());
        Serial1.print(",");
        Serial1.println(romiKinematics.getPoseTheta());
      }

  }

  rightPose.resetThresholdReached(); //reset PIDs for next movement!
  leftPose.resetThresholdReached();
  return myObstaclePresent;
}

//-----END OF STRAIGHT LINE FNCTION------


// -------TURN FUNCTION-------------------

bool moveTurnOnSpot(float angle, bool findLine) { // Function to turn on the spot for a specified angle

  //Set local variable
  bool finishedMovement = false;
  long baselineEncoderValue0 = count_e0; //current right encoder value
  long baselineEncoderValue1 = count_e1; //current left encoder value

  //~~Calculate angle in terms of encoder count~~
  //Note some parameters for this calculation [A] wheels are 145mm apart! [B] there are 8.29 encoder counts per degree revolution of Romi
  long encoderTarget = round(8.29 * angle);

  //set current target
  R_EncoderCurrentTarget = count_e0 + encoderTarget; //have just changed this from taking the target to taking the current encoder count!
  L_EncoderCurrentTarget = count_e1 - encoderTarget;

  //set variable for finding a line
  bool myLinePresent = false;

  //set other variables
  bool R_thresholdReached;
  bool L_thresholdReached;
  float outputAnglePoseStraight;
  float outputRightMotor;
  float outputLeftMotor;

  while (finishedMovement == false) { //loop while the movement is still underway
    //Do some stuff to find the time now

    // Get how much time has passed right now.
    unsigned long time_now = millis();

    // Work out how many milliseconds have gone passed by subtracting
    // our two timestamps.  time_now will always be bigger than the
    // time_of_read (except when millis() overflows after 50 days).
    unsigned long elapsed_time_lineStraightOrTurning = time_now - time_of_read_lineStraightOrTurning;
    unsigned long elapsed_time_lineStraightOrTurningPID = time_now - time_of_read_lineStraightOrTurningPID;
    unsigned long elapsed_time_lineStraightOrTurningMotors = time_now - time_of_read_lineStraightOrTurningMotors;

    //first do some stuff if you're trying to find a line

    //Do some stuff if you're looking to find a line (Only applicable for CW1)
    if (findLine == true)
    {
      // See if 10 milliseconds have ellapsed
      // If not, this block is skipped.
      if ( elapsed_time_lineStraightOrTurning > 10 )
      {
        // Since 10ms elapsed, we update our timestamp
        // so that another 10ms is needed to pass.
        time_of_read_lineStraightOrTurning = millis();

        if (lineCentre.read_filtered() > 90) //if finds line
        {
          rightMotor.setSpeed(0);
          leftMotor.setSpeed(0);
          finishedMovement = true;
          myLinePresent = true;
          play_tone(55, 30);
          delay(50);
          play_tone(55, 30);
          delay(500);
          //romiLineSensorFusion.lineConfidenceValue = 100;
          break;
        }
      }
    }

    //Do mapping (CW2)
    doMapping();

    //~~initiate the 3 PIDS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if ( elapsed_time_lineStraightOrTurningPID > 20 )
    {

      time_of_read_lineStraightOrTurningPID = millis();

      //Update PID control and get output for right motor and left motor
      outputRightMotor = rightPose.update(R_EncoderCurrentTarget, count_e0);
      outputLeftMotor = leftPose.update(L_EncoderCurrentTarget, count_e1);

      //Update PID control and get output for angle // note left is slave motor, right is master
      outputAnglePoseStraight = anglePoseStraight.update(-(baselineEncoderValue0 - count_e0), (baselineEncoderValue1 - count_e1));
      //float outputAnglePoseStraight = 0;

      //get parameters to indicate whether PID has achieved goal for left and right motor (distance)
      R_thresholdReached = rightPose.getThresholdReached();
      L_thresholdReached = leftPose.getThresholdReached();

    }

    // ~~send demand to motors and check for completion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if ( elapsed_time_lineStraightOrTurningMotors > 50 )
    {

      time_of_read_lineStraightOrTurningMotors = millis();

      if (R_thresholdReached == 1) {          //if Right motor has reached goal
        rightMotor.setSpeed(0);               //Set right motor speed to zero
        if (L_thresholdReached == 1) {        //if Left motor has reached goal
          leftMotor.setSpeed(0);              //set motor speed to zero
          finishedMovement = true;
        }
        else {
          leftMotor.setSpeed(outputLeftMotor - outputAnglePoseStraight); //else keep left motor moving towards goal
          romiKinematics.updateTurnOnSport(count_e0, count_e1); //update the kinematics function
        }
      }
      else {
        rightMotor.setSpeed(outputRightMotor);
        leftMotor.setSpeed(outputLeftMotor - outputAnglePoseStraight); //else keep both motors moving towards goals
        romiKinematics.updateTurnOnSport(count_e0, count_e1); //update the kinematics function
      }


      //printDebugStatements(outputRightMotor, outputLeftMotor, outputAnglePoseStraight); // for debugging
      delay(LOOP_DELAY); //the code doesn't work without this, not sure why? ... question to instructors ! - think maybe to do with PID sample rate


      //debugging comments
      //      Serial.print("encoder values   R:");
      //      Serial.print( count_e0 );
      //      Serial.print(", L:");
      //      Serial.print( count_e1 );
      //      Serial.print("  R_EncoderTarget = ");
      //      Serial.print(R_EncoderCurrentTarget);
      //      Serial.print("  L_EncoderTarget = ");
      //      Serial.println(L_EncoderCurrentTarget);

//      Serial.print("Kinematics X =");
//      Serial.println(romiKinematics.getPoseX());
//      Serial.print("Kinematics Y =");
//      Serial.println(romiKinematics.getPoseY());
//      Serial.print("Kinematics Theta =");
//      Serial.println(romiKinematics.getPoseTheta());
      //      Serial.print(romiLineSensorFusion.isLinePresent()*10);
      //      Serial.print(",");
      //      Serial.println(romiLineSensorFusion.lineConfidenceValue);

    unsigned long elapsed_time_general_timer = millis() - general_timer;

    if ( elapsed_time_general_timer > 1000 )
      {
        // update timestamp
        general_timer = millis();

        Serial1.print((millis()-start_time)/1000);
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseX());
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseY());
        Serial1.print(",");
        Serial1.println(romiKinematics.getPoseTheta());
      }


    }
  }

  rightPose.resetThresholdReached(); //reset PIDs for next movement!
  leftPose.resetThresholdReached();
  return myLinePresent;
}

//-----END OF TURN FUNCTION---------------

//-----GO TO POINT FUNCTION (FORWARDS)--------------------

bool goToPoint(float x, float y)
{
  x = int(x);
  y = int(y);

  //Calculate required distance
  float distanceRequired = getDistanceToPoint(x, y);

  //Calculate required angle
  //float headingRequired = atan2(deltaY, deltaX)*180/PI;
  float headingRequired = getAngleToPoint(x, y);
  float turnAngleRequired = headingRequired - romiKinematics.getPoseTheta() + 180;

  //boolean for obstacle detection
  bool myObstacleDetected = false;

  //Convert required turn angle to minimum clockwise or anticlockwise rotation
  if (turnAngleRequired > 180)
  {
    turnAngleRequired -= 360;
  }
  else if (turnAngleRequired < -180)
  {
    turnAngleRequired += 360;
  }

  Serial.println("");
  Serial.println("----------TURNING ANGLE = ---------");
  Serial.println(turnAngleRequired);
  Serial.println("----------DISTANCE = ---------");
  Serial.println(distanceRequired);
  Serial.println((- round(distanceRequired / 0.153)));
  Serial.println("-----------------------------");
  Serial.println("");

  //go home
  moveTurnOnSpot(turnAngleRequired, 0);

  myObstacleDetected = moveStraightLine((- round(distanceRequired / 0.153)), 1);

  return myObstacleDetected;
}

//------------------------------------------

//-----TURN TOWARDS POINT FUNCTION --------------------

void turnTowardsPoint(float x, float y)
{
  x = int(x);
  y = int(y);

  //Calculate required angle
  //float headingRequired = atan2(deltaY, deltaX)*180/PI;
  float headingRequired = getAngleToPoint(x, y);
  float turnAngleRequired = headingRequired - romiKinematics.getPoseTheta() + 180;

  //Convert required turn angle to minimum clockwise or anticlockwise rotation
  if (turnAngleRequired > 180)
  {
    turnAngleRequired -= 360;
  }
  else if (turnAngleRequired < -180)
  {
    turnAngleRequired += 360;
  }

  //go home
  moveTurnOnSpot(turnAngleRequired, 0);
}

//------------------------------------------

//-----GET DISTANCE TO POINT FUNCTION--------------------

float getDistanceToPoint(float pointX, float pointY)
{
  float deltaX = pointX - romiKinematics.getPoseX();
  float deltaY = pointY - romiKinematics.getPoseY();

  //Calculate distance
  float distanceRequired = sqrt(deltaX * deltaX + deltaY * deltaY);

  return distanceRequired;
}
//------------------------------------------

//-----GET ANGLE TO POINT FUNCTION--------------------
float getAngleToPoint(float pointX, float pointY)
{
  float deltaX = pointX - romiKinematics.getPoseX();
  float deltaY = pointY - romiKinematics.getPoseY();

  //Calculate required angle
  float headingRequired = atan2(deltaY, deltaX) * 180 / PI;

  return headingRequired;
}
//------------------------------------------

//-----RANDOM WALK FUNCTION--------------------#
// Function to move with a random walk
// If findObstacle = true then the function will exit when an obstacle is detected

void doRandomWalk(bool findObstacle) {

  //Variables for timer
  static unsigned long walk_update = millis();
  static unsigned long obstacle_update = millis();
  static unsigned long mapping_update = millis();

  //Set local variables
  bool finishedMovement = false;

  // used to control the forward and turn speeds of the robot.
  int forward_bias = -25;
  int turn_bias = 0;
  int left_speed_demand = 0;
  int right_speed_demand = 0;
  //bool myObstaclePresent = false;  //set variable for finding an obstacle //not used

  while (finishedMovement == false) { //loop while the movement is still underway

    // Get how much time has passed right now.
    //unsigned long time_now = millis();

    if ( millis() - mapping_update > 40 ) {
      mapping_update = millis();

      doMapping(); //Complete mapping
      romiKinematics.updateStraightMotion(count_e0, count_e1); //update the kinematics function

    }


    //If obstacle is detected then end the movement
    if ( millis() - obstacle_update > 100 && findObstacle == true) {
      obstacle_update = millis();

      if (DistanceSensor.obstacleDetected == true || checkIfRomiIsOutsideMap() == true)
      {
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
        finishedMovement = true;
        // myObstaclePresent = true; //not used
        break;
      }
    }


    // Periodically set a random turn. Here, gaussian means we most often drive forwards, and occasionally make a big turn.
    if ( millis() - walk_update > 500 ) {
      walk_update = millis();

      // randGaussian(mean, sd).  utils.h
      turn_bias = randGaussian(0, 10);

      // Setting a speed demand with these variables is automatically captured by a speed PID controller in timer3 ISR. Check interrupts.h  for more information.
      left_speed_demand = forward_bias + turn_bias;
      right_speed_demand = forward_bias - turn_bias;
    }

    //keep both motors moving in a random direction
    rightMotor.setSpeed(left_speed_demand);
    leftMotor.setSpeed(right_speed_demand); //else keep both motors moving towards goals

    //Debugging statements
    //    Serial.print("checkIfRomiIsOutsideMap() = ");
    //    Serial.print(checkIfRomiIsOutsideMap());
//    Serial.print("  Kinematics X = "); //note i should move this all into the debug function !
//    Serial.print(romiKinematics.getPoseX());
//    Serial.print("  Kinematics Y = ");
//    Serial.print(romiKinematics.getPoseY());
//    Serial.print("  Kinematics Theta = ");
//    Serial.println(romiKinematics.getPoseTheta());



    unsigned long elapsed_time_general_timer = millis() - general_timer;

    if ( elapsed_time_general_timer > 1000 )
      {
        // update timestamp
        general_timer = millis();

        Serial1.print((millis()-start_time)/1000);
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseX());
        Serial1.print(",");
        Serial1.print(romiKinematics.getPoseY());
        Serial1.print(",");
        Serial1.println(romiKinematics.getPoseTheta());
      }


  }
}

//------------------------------------------

//-----CHECK LINE POSITION FUNCTION--------------
void checkLinePosition()
{
  //romiLineSensorFusion.returnLinePosition(lineLeft.read_calibrated(), lineCentre.read_calibrated(), lineRight.read_calibrated());

}
//------------------------------------------

//-----CHECK FOR END OF LINE--------------
bool checkForEndOfLine(bool myLastDirection)
{

  int signedDirection;
  bool isTheLinePresent = false;

  delay(500);

  if (myLastDirection == 1) //right
  {
    signedDirection = 1;
  }
  else if (myLastDirection == 0) //left
  {
    signedDirection = -1;
  }

  int i = 0;

  isTheLinePresent = moveTurnOnSpot(signedDirection * 90, 1); //turn and search for a line

  if (isTheLinePresent == false)
  {
    delay(500);
    rightMotor.setSpeed(0);
    leftMotor.setSpeed(0);
    signedDirection = signedDirection * -1;
    isTheLinePresent = moveTurnOnSpot(signedDirection * 180, 1); //turn and search for a line
  }

  if (isTheLinePresent == true)
  {
    //Serial.print("found line in search!");
  }

  else
  {
    play_tone(150, 1000);
    //Serial.print("didn't find line in search!");
  }


  return !isTheLinePresent;
}
//------------------------------------------

//-----------------Functions for Positioning !!!--------------------------

//-----MAPPING FUNCTION---------------
void doMapping() {

  //Timing stuff
  static unsigned long irSensor_update = millis();
  static unsigned long emptyCell_update = millis();

  // Read the IR Sensor and determine distance in mm.

  if ( millis() - irSensor_update > 100 ) {
    irSensor_update = millis();

    float distance = DistanceSensor.getDistanceInMM();
    float filteredDistance = DistanceSensor.getFilteredDistanceInMM();


    //Some debugging print statements for calibrating IR sensor
    //    Serial.print("distance = ");
    //    Serial.print(distance);
    //    Serial.print("  |  filteredDistance = ");
    //    Serial.println(filteredDistance);


    if ( filteredDistance < 450 && filteredDistance > 30 ) { //i.e. between approx 30mm and 400mm distance away

      // add distance to  centre of the robot.
      distance += 95;

      // Here we calculate the actual position of the obstacle we have detected
      float projected_x = romiKinematics.getPoseX() + ( distance * cos( romiKinematics.getPoseThetaRadians() ) );
      float projected_y = romiKinematics.getPoseY() + ( distance * sin( romiKinematics.getPoseThetaRadians() ) );
      Map.updateMapFeature( (byte)'O', projected_x, projected_y );

    }



    if ( lineCentre.read_filtered() > 50 ) {
      Map.updateMapFeature( (byte)'L', romiKinematics.getPoseY(), romiKinematics.getPoseX() );
    }
  }

  if ( millis() - emptyCell_update > 200 ) {
    emptyCell_update = millis();

    bool tileExplored = doCheckForUnexploredTile();
    if (tileExplored == false) {
      Map.updateMapFeature( (byte)'.', romiKinematics.getPoseY(), romiKinematics.getPoseX() );
    }
  }
}
//---------------------------------

//-----------------Functions for Utlities !!!--------------------------

//-----DEBUG FUNCTION---------------
void printDebugStatements(float outputRightMotor, float outputLeftMotor, float outputAnglePoseStraight) {

  /*
    //Code for debugging of PID OUTPUT for right motor
    Serial.print("Right wheel PID output is: ");
    Serial.println(outputRightMotor);
  */


  //Code for debugging of PID OUTPUT for left motor
  Serial.print("Left wheel PID output is: ");
  Serial.println(outputLeftMotor);


  /*
    //Code for debugging of PID OUTPUT for angle PID (where left wheel is a slave to right master)
    Serial.print("Left wheel slave angle PID output is: ");
    Serial.println(outputAnglePoseStraight);
  */

  //Code to check encoders values
  // e0-> right, e1-> left
  Serial.print("encoder values   R:");
  Serial.print( count_e0 );
  Serial.print( ", L:");
  Serial.println( count_e1 );

  // short delay so that our plotter graph keeps
  // some history for debugging
  delay(LOOP_DELAY);

}
//---------------------------------


//-----TONE FUNCTION---------------
void play_tone(int frequency, int duration) //plays a tone with specified duration - useful for debugging and understanding state of robot
{
  tone(buzzerPin, frequency, duration);
  delay(duration);
}
//---------------------------------

//-----CHECK buttons function------------

void check_buttons()
{
  btnA.read();               // read the buttons
  btnB.read();

  if (btnA.wasReleased())    // if the button was released, play a tone
  {
    //do nothing
  }

  if (btnB.wasReleased())    // if the button was released, play a tone and do some functionality
  {
    play_tone(200, 150);
    romiKinematics.setPoseTheta(romiKinematics.getPoseTheta() + 30);
  }
}

//---------------------------------

/*-----------------------------------------------------------------------------
   FUNCTION - MAPPING RELATED
  ------------------------------------------------------------------------------*/

//-----CHECK FOR UNEXPLORED TILE FUNCTION------------
bool doCheckForUnexploredTile() {

  //Get current grid position on map
  int x_index = Map.poseToIndex(int(romiKinematics.getPoseX()), MAP_X, MAP_RESOLUTION);
  int y_index = Map.poseToIndex(int(romiKinematics.getPoseY()), MAP_Y, MAP_RESOLUTION);

  //Check if tile is explored?
  bool tileExplored = Map.isTileExplored(x_index, y_index);

  //Check if Romi is still inside map?
  //Map.checkIfRomiIsInsideMap(x_index, y_index);

  //Print values for debugging
  //  Serial.print("x: ");
  //  Serial.print(x_index);
  //  Serial.print("  y: ");
  //  Serial.print(y_index);
  //  Serial.print("  tileExplored = ");
  //  Serial.println(tileExplored);

  //return value
  return tileExplored;
}
//---------------------------------

//-----CHECK FOR ROMI OUTSIDE OF MAP FUNCTION------------
bool checkIfRomiIsOutsideMap() {

  bool romiOutsideOfMap;

  //int x_index = Map.poseToIndex(int(romiKinematics.getPoseX()), MAP_X, MAP_RESOLUTION);
  //int y_index = Map.poseToIndex(int(romiKinematics.getPoseY()), MAP_Y, MAP_RESOLUTION);

  int upperLimitX = MAP_X - MAP_BORDER;
  int upperLimitY = MAP_Y - MAP_BORDER;
  int lowerLimit = 0 + MAP_BORDER;

  if (romiKinematics.getPoseX() <= upperLimitX && romiKinematics.getPoseY() <= upperLimitY && romiKinematics.getPoseX() > lowerLimit && romiKinematics.getPoseY() > lowerLimit) {
    romiOutsideOfMap = false;
  }
  else {
    romiOutsideOfMap = true;
  }

  //return value
  return romiOutsideOfMap;
}
//---------------------------------
