#ifndef _Kinematics
#define _Kinematics_h
#include "mapping.h"

//You may want to use these variables
const int WHEEL_DIAMETER = 70; //(mm)
const int WHEEL_DISTANCE = 139; //(mm) //139 is safe(ish) value -> testing 137
const int GEAR_RATIO = 120;
const int COUNTS_PER_SHAFT_REVOLUTION = 12;
const int COUNTS_PER_WHEEL_REVOLUTION =  1440;
//const float COUNTS_PER_MM = ??;
const float MM_PER_COUNT = 0.153;

class Kinematics
{
  public:
    //Public variables and methods go here
    Kinematics(long, long); // This is the class constructor. It is called whenever we create an instance of the PID class
    
    void setPoseTheta(float); //Set the poseTheta of the robot
    void resetEncoderCount(); //Reset all encoder counts to zero. Used to periodically prevent overflow
    
    float getPoseX(); //Return the pose of the robot
    float getPoseY(); //Return the pose of the robot
    float getPoseTheta(); //Return the pose of the robot
    float getPoseThetaRadians(); //Return the pose of the robot in radians
    float updateStraightMotion(float r_encoderCount, float l_encoderCount); ////This function updates the estimated pose of the robot. It should be called in a loop
    float updateTurnOnSport(float r_encoderCount, float l_encoderCount); ////This function updates the estimated pose of the robot. It should be called in a loop

  private:
    //Private variables and methods go here

    //Representing the position of the robot - note we start in the centre of the map!
    float poseX = MAP_X/2; //position in X global frame (mm)
    float poseY = MAP_Y/2; //position in Y global frame (mm)
    float poseTheta = 0; //orientation in global frame (deg)
    long lastCount_eR; //stores the last value of right encoder count
    long lastCount_eL; //stores the last value of left encoder count
};
/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
 Kinematics::Kinematics(long R_encoderCount, long L_encoderCount)
 {
  lastCount_eR = R_encoderCount; //store current encoder counts for use in calculations
  lastCount_eL = L_encoderCount;
 }

void Kinematics::setPoseTheta(float myPoseTheta)
{
  poseTheta = myPoseTheta;
  if(poseTheta >= 360)
  {
    poseTheta -= 360;
  }
  if(poseTheta <0)
  {
    poseTheta += 360;
  }
  
  Serial.print("poseTheta = ");
  Serial.println(poseTheta);
}

float Kinematics::getPoseX()
{
  return poseX;
}

float Kinematics::getPoseY()
{
  return poseY;
}

float Kinematics::getPoseTheta()
{
  return poseTheta;
}

float Kinematics::getPoseThetaRadians()
{
  return poseTheta * (PI/180);
}



float Kinematics::updateStraightMotion(float r_encoderCount, float l_encoderCount)
{
  //----------complete update of X and Y pose values -----------
  long dRight = count_e0 - lastCount_eR;
  long dLeft = count_e1 - lastCount_eL;

  //convert to average and into mm
  float dTotal = ((dRight + dLeft)/2)* MM_PER_COUNT; //in (mm)
  
  poseX = poseX + dTotal * cos(poseTheta*PI/180);
  poseY = poseY + dTotal * sin(poseTheta*PI/180);

  //----------complete update of heading poseTheta ----------------

  poseTheta = poseTheta + ((dRight - dLeft)* MM_PER_COUNT/WHEEL_DISTANCE)*180/PI; 

  if(poseTheta >= 360)
  {
    poseTheta -= 360;
  }
  if(poseTheta < 0)
  {
    poseTheta += 360;
  }

    
  //---store current encoder values to be used on next update------
  lastCount_eR = r_encoderCount;
  lastCount_eL = l_encoderCount;  
}

float Kinematics::updateTurnOnSport(float r_encoderCount, float l_encoderCount)
{
  //----------complete update of X and Y pose values -----------
  long dRight = count_e0 - lastCount_eR;
  long dLeft = count_e1 - lastCount_eL;

  //convert to average and into mm
  float dTotal = ((abs(dRight) + abs(dLeft))/2)* MM_PER_COUNT; //in (mm)

  //----------complete update of heading poseTheta ----------------

  poseTheta = poseTheta + ((dRight - dLeft)* MM_PER_COUNT/WHEEL_DISTANCE)*180/PI; //not sure why we don't divide this by two !

  if(poseTheta >= 360)
  {
    poseTheta -= 360;
  }
  if(poseTheta < 0)
  {
    poseTheta += 360;
  }

    
  //---store current encoder values to be used on next update------
  lastCount_eR = r_encoderCount;
  lastCount_eL = l_encoderCount;  
}

void Kinematics::resetEncoderCount()
{
  count_e0 = 0;
  count_e1 = 0;
  lastCount_eR = 0;
  lastCount_eL = 0;  
}

#endif
