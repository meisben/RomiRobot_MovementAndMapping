#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
const int NUM_CALIBRATIONS = 40;

/*
    Class to represent a single line sensor
*/
class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int pin);
    //Calibrate
    void calibrate();
    //Return the uncalibrated value from the sensor
    int read_raw();
    //Return the calibrated value from the sensor
    int read_calibrated();
    //Return the filtered value
    int read_filtered();

  private:

    int pin;
    int outputY;
    
    const float alpha = 0.8; //used for low pass filter
    
    /*
       Variables needed for calibration here
    */
    unsigned int calibratedBaseline;

};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

int Line_Sensor::read_raw()
{
  return analogRead(pin);
}

void Line_Sensor::calibrate() //This code calibrates each line sensor
{
  //float calibrationArray[NUM_CALIBRATIONS];
  unsigned int calibrationRunningTotal = 0;

  byte i;

  for (i = 1; i <= NUM_CALIBRATIONS; i++) {
    delay(50);
    calibrationRunningTotal+= read_raw();
    //Serial.print("i: ");
    //Serial.print(i);
    //Serial.print("  running total: ");
    //Serial.println(calibrationRunningTotal);
    }

    //Serial.print("i: ");
    //Serial.println(i);
    //Serial.print("calibration running total final ");
    //Serial.println(calibrationRunningTotal);

    calibratedBaseline = calibrationRunningTotal / (i-1); //note we de-crement i by one because it was incremented by an extra one in the loop wtf

    Serial.print("calibrated Baseline ");
    Serial.println(calibratedBaseline);
}

int Line_Sensor::read_calibrated() //This code returns a calibrated reading
{
  return analogRead(pin) - calibratedBaseline;
}

int Line_Sensor::read_filtered() //This code returns a calibrated and filtered reading
{
    int inputX = read_calibrated();        
    outputY += alpha*(inputX-outputY);   
    return outputY;
}


#endif
