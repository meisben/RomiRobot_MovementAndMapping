#ifndef _PID_h
#define _PID_h
#include <stdint.h>

#define countStorageSize 10

/*Here, the definition of the PID class begins. This is indicated by the keyword: "class"
This is a general description of the data and functions that the class contains. 
To use a class, we must make a specific instance of the class by declaring it into the same way we declare a variable. 
For example, to create a version of the PID class, in our main file we might write:

PID LeftWheelPID;
PID RightWheelPID;

This will create two instances of the PID class; one for the left wheel and one for the right wheel. 
Each class will have a full copy of all the variables and functions defined for that particular class.
*/ 

class PID
{
  /* Public functions and variables are defined here. A public function / variable can be accessed from outside 
   * the class. 
   * For example, once we have made an instance of the PID class, we can call the update function by writing:
   * 
   * LeftWheelPID.update();
   * 
   * Note that this will only update the LeftWheelPID - RightWheelPID will not be updated unless we also call 
   * RightWheelPID.update()
   */
  public:

    PID(float P, float D, float I); // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float D, float I); // This function updates the values of the gains
    void reset(); //This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement); //This function calculates the PID control signal. It should be called in a loop
    void print_components(); //This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax); //This function sets the maximum output the controller can ask for
    void setDebug(bool state); //This function sets the debug flag;
    //-------added by ben for threshold
    void setThresholdActive(bool state,int threshold); // This function sets a active threshold
    bool getThresholdReached(); //This function tells the loop whether we reached the threshold
    void resetThresholdReached(); //This function enables us to reset the threshold and re-use the PID
    int getInstananeousAverageAbsError(); //useful for debugging
    float error =0; //the error for the PID, in public section for debugging

    //Output components
    //These are used for debugging purposes
    float Kp_output=0; 
    float Ki_output=0;
    float Kd_output=0;
    

  /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
   * For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!) 
   */
  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative

    //We can use this to limit the output to a certain value
    float max_output=255; 

    //Output components
    //These are used for debugging purposes
    float total=0;

    //Values to store
    float last_error=0; //For calculating the derivative term
    float integral_error=0; //For storing the integral of the error
    long last_millis = 0;
    bool debug=false; //This flag controls whether we print the contributions of each component when update is called
    
    //these two variables added by Ben for the PID ending threshold -------------------------------------------------
    int storageArrayErrors[countStorageSize] = {50,50,50,50,50,50,50,50,50,50};
    unsigned char error_counter = 0;
    bool thresholdActive=false;
    int thresholdValue = 20;
    long totalAbsError = 300;
    bool thresholdReached=false;
    
};

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
 PID::PID(float P, float D, float I)
{
  //Store the gains
  setGains(P, D, I);
  //Set last_millis
  last_millis = 0;
}

/*
 * This function prints the individual contributions to the total contol signal
 * You can call this yourself for debugging purposes, or set the debug flag to true to have it called
 * whenever the update function is called.
 */
void PID::print_components()
{
  Serial.print("Proportional component: ");
  Serial.print(Kp_output);
  Serial.print(" Differential component: ");
  Serial.print(Kd_output);
  Serial.print(" Integral component: ");
  Serial.print(Ki_output);
  Serial.print(" Total: ");
  Serial.println(total);
}

/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float D, float I)
{
  Kp = P;
  Kd = D;
  Ki = I;
}

/*
 * This is the update function. 
 * This function should be called repeatedly. 
 * It takes a measurement of a particular quantity and a desired value for that quantity as input
 * It returns an output; this can be sent directly to the motors, 
 * or perhaps combined with other control outputs
 */
float PID::update(float demand, float measurement)
{
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  /*
   * ================================
   * Your code goes implementation of a PID controller should go here
   * ================================
   */

  //This represents the error term
  error = demand - measurement;
  //This represents the error derivative
  float error_delta = (error - last_error)/time_delta;

  //Update storage
  integral_error = ((error + last_error)/2)*time_delta;
  //Note this could also look like this: integral_error += ((error + last_error)/2)*time_delta;

  //------------Bens code for threshold -------

    if (thresholdActive){
      if(1==1){ //statement here to only run periodically
      
        //code to store last variables and calculate threshold
  
        //put current error in storage array
        storageArrayErrors[error_counter] = abs(error);
  
        //calculate running average error
        unsigned char temp;
        if(error_counter == (countStorageSize-1)){
          temp = 0;
        }
        else{
          temp = error_counter + 1;
        }
        totalAbsError-= storageArrayErrors[(temp)];
        totalAbsError+= abs(error);
             
        //check if below threshold
        if(totalAbsError/countStorageSize <= thresholdValue){
          thresholdReached = true;
        }
        
        //update error counter
        error_counter++;
  
        //remap error counter to the size of the storage error array#
        if(error_counter >= countStorageSize){
          error_counter = 0;
        }
      }
    }//end of thresholdActive = True

  //----------------------------

  /*
   * ===========================
   * Code below this point should not need to be changed
   */

  //Calculate components
  Kp_output = Kp*error;
  Kd_output = Kd*error_delta;
  Ki_output = Ki*integral_error;

  //Add the three components to get the total output
  total = Kp_output + Kd_output + Ki_output;

  //store the error value for the next cycle (for the Kd term)
  last_error = error;

  //Make sure we don't exceed the maximum output
  if (total > max_output)
  {
    total = max_output;
  }

  if (total < -max_output)
  {
    total = -max_output;
  }

  //Print debugging information if required
  if (debug)
  {
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" Error Delta");
    Serial.println(error_delta);
    Serial.println("----Threshold values----");
    Serial.print("thresholdReached bool: ");
    Serial.println(thresholdReached);
    Serial.print("Here is the InstAverageAbsError: ");
    Serial.println(totalAbsError/countStorageSize);
    print_components();
  }
  
  return total;
}

void PID::setMax(float newMax)
{
  if (newMax > 0)
  {
    max_output = newMax;
  }
  else
  {
    Serial.println("Max output must be positive");
  }
}

void PID::setDebug(bool state)
{
  debug = state;
}

//-------------functionality added by Ben for PID thresh-hold

void PID::setThresholdActive(bool state,int threshold)
{
  thresholdActive = state;
  thresholdValue = threshold;
}

void PID::resetThresholdReached()
{
  thresholdReached=false;
}

bool PID::getThresholdReached()
{
  return thresholdReached;
}

int PID::getInstananeousAverageAbsError()
{
  return totalAbsError/countStorageSize;
}




#endif
