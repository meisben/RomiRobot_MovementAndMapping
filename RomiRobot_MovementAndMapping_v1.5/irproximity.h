#ifndef _IRProximity_h
#define _IRProximity_h

//Constants
#define OBSTACLE_DISTANCE 90 //used to set the distance that we define an obstacle to be at

class SharpIR
{
    public:
        SharpIR(byte pin);

        bool obstacleDetected = false; //flag which is set to true if obstacle is detected

        float  getFilteredDistanceInMM(); //use this one for CW2 !
        
        //void calibrate(); //not used
        void checkForObstacle(float);

    private:
        byte pin;
        float outputY;
        
        const float alpha = 0.5; //used for low pass filter

};

SharpIR::SharpIR(byte _pin)
{
  pin = _pin;
}


float SharpIR::getFilteredDistanceInMM()
{
    
    float distance = analogRead( pin );
    
    // map this to 0 : 5v range.
    distance *= 0.0048;

    if (distance<0.1){
      distance = 0.1;
    }

    const float exponent = (1/-0.676);
    distance = pow( ( distance / 67.41 ), exponent);
    
    outputY += alpha*(distance-outputY);

    checkForObstacle(outputY);
    
    return outputY;
}


//used to check for an obstacle
void SharpIR::checkForObstacle(float myDistance)
{
    
    if(myDistance <= OBSTACLE_DISTANCE){
      obstacleDetected = true;
    }
}

#endif
