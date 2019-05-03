
#ifndef COMP_PFILTER

#include <LIS3MDL.h>
#include <LSM6.h>


class comp_filter
{
  public:
    comp_filter(int T); // Nawid - This is the class constructor - not really sure what parameters to add
    void initDevices();
    void calibrate_mag_auto();
    void set_update_time(int T);
    void max_min_mag_data();
    void calibrate_mag();
    void calibrate_gyro();
    void initial_filtered_reading();
    void calculate_gyro_bias();
    float complementary_filter_calc();
    LSM6 imu;
    LIS3MDL mag; // Nawid - Object to hold the reading
    long t;
    float filtered_mag_heading; // Nawid - This is the filtered heading value
    float initial_mag_heading_value;


  private:

   
//    float mag_x;
//    float mag_y;

//    int sensitivity = 6842; // Nawid -  Conversion factor required

//    int num_calibration_mag = 2000;



    // Nawid - Terms for calibration of the magnetometer
    float offset_x;
    float offset_y;
    float offset_z;
    float range_x;
    float range_y;
    float range_z;
    float avg_range;
    float scale_x;
    float scale_y;
    float scale_z;
    float calibrated_reading_x;
    float calibrated_reading_y;
    
//    float mag_heading; // Nawid - This is the heading angle

//    float mag_alpha = 0.3;

    // Nawid - Gyro calibration
//    int calN = 100; - Changed for a constant 
    float avg;
    

    int Tupdate = 200;
//    long dT = 40;
//    float measurement;
//    float dx;
//    float g = 0.4;
//    float h = 0.15;
//    float estimate = 0;
//    float prediction = 0;
//    float residue;
//    float K_rot = 0.0175; // FS 250: 0.00875 - Nawid - Sensitivity ( This value is divided by 1000 to take into account degrees per second)


    // Nawid - Complementary filter info
//    float filtered_mag_heading; // Nawid - This is the filtered heading value
//    float initial_mag_heading_value;
    float comp_heading;// Nawid - heading for the complementary filter
//    float comp_alpha = 0.9; // Nawid - value for the alpha of the complementary filter
//    float gyro_heading;

    // Nawid - parameters for finding the drift in the gyro.
/*    float starting_time;
    float final_time;
    float total_gyro_heading;
    float first_gyro_heading;
    float bias_per_timestep;
*/
//    float gyro_reading; // Nawid - This is just to show the gyro_reading from each timestep
//    float filtered_mag_heading_comp; // Nawid -  This is another variable to hold the value so processing during the complementary filter stage does not affect it


    
};

comp_filter::comp_filter(int T){
  set_update_time(T);
  t=millis();
}
void comp_filter::initDevices(){ // Nawid - Function to initialise the gyro and the magnetometer
  imu.init();
//  
//  if (!imu.init()) // Nawid - Used to recognise device
//  {
//    while (1)
//    {
//      Serial.println(F("Failed to detect the LSM6."));
//      delay(100);
//    }
//  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL1_XL, 0b01011000); // 208 Hz, +/4 g  // Nawid- sample rate of 208hz and 4g range
  imu.writeReg(LSM6::CTRL2_G,  0b00110100); // 52 Hz, range 500 dps // Nawid - This is for the gyro- Lowered sample rate to 52 Hz
  
  mag.init(); // Nawid - Added this now, which could potentially affect the values of the calibration.
  mag.enableDefault();
//  if (!mag.init())
//  {
//    Serial.println("Failed to detect and initialize magnetometer!");
//    while (1);
//  }
  }
void comp_filter::set_update_time(int T){ // Nawid- Function to set the time between measurements
  Tupdate = T;
}

void comp_filter::calibrate_mag(){ // Nawid - function which performs the manual calibration
   int max_x = -32768; // Nawid - Max X from calibration
   int min_x = 32767; ;// Nawid - Min X from calibration
   int max_y = -32768;// Nawid - Max y from calibration
   int min_y = 32767;// Nawid - Min y from calibration
   int max_z = -32768;// Nawid - Max z from calibration
   int min_z = 32767;// Nawid - Min z from calibration
  for (int i = 0; i <= 2000; i++) // Nawid - Iterates
  {
    if (i == 0) // Natalia-  For the first measurments, the max and min values are the ones collected from data
    {
      mag.read();
      max_x = mag.m.x;
      min_x = mag.m.x;
      max_y = mag.m.y;
      min_y = mag.m.y;
      max_z = mag.m.z;
      min_z = mag.m.z;
    }
    else
    {
      mag.read();
      max_x = max(max_x, mag.m.x);
      min_x = min(min_x, mag.m.x);
      max_y = max(max_y, mag.m.y);
      min_y = min(min_y, mag.m.y);
      max_z = max(max_z, mag.m.z);
      min_z = min(min_z, mag.m.z);

    }
    delay(10);
  }
  offset_x = (max_x + min_x) / 2;
  range_x = (max_x - min_x) / 2;
  offset_y = (max_y + min_y) / 2;
  range_y = (max_y - min_y) / 2;
  offset_z = (max_z + min_z) / 2;
  range_z = (max_z - min_z) / 2;
  avg_range = range_x + range_y + range_z;
  scale_x = avg_range / range_x;
  scale_y = avg_range / range_y;
  scale_z = avg_range / range_z;
}

void comp_filter::calibrate_mag_auto(){//-4559.00,933.00|-2886.00,2237.00|-1239.00,4984.00
  int min_x=-4559;
  int max_x=933;
  int min_y=-2886;
  int max_y=2237;
  int min_z=-1239;
  int max_z=4984;
  
  offset_x = (max_x + min_x) / 2;
  range_x = (max_x - min_x) / 2;
  offset_y = (max_y + min_y) / 2;
  range_y = (max_y - min_y) / 2;
  offset_z = (max_z + min_z) / 2;
  range_z = (max_z - min_z) / 2;
  avg_range = range_x + range_y + range_z;
  scale_x = avg_range / range_x;
  scale_y = avg_range / range_y;
  scale_z = avg_range / range_z;
}

void comp_filter:: initial_filtered_reading(){ // Nawid - Function to get the initial filtered reading in degrees and the initial mag heading is radians
  mag.read();
//  float mag_x = mag.m.x;
  calibrated_reading_x = scale_x * (mag.m.x - offset_x)/6842;
//  float mag_y = mag.m.y;
  calibrated_reading_y = scale_y * (mag.m.y - offset_y)/6842;
  filtered_mag_heading = atan2(calibrated_reading_y,calibrated_reading_x); 
  initial_mag_heading_value = filtered_mag_heading; // Nawid - This is the offset value in radians
  filtered_mag_heading = filtered_mag_heading*(57296 / 1000);  // Nawid - This is the initial value in degrees

}
void comp_filter::calibrate_gyro(){
  //Calibrate over 100 measures
  avg = 0;
  for (int i = 0; i < 100; i++) {
    imu.read();
    avg = avg + imu.g.z;
  }
  avg = avg / 100;
}

/*
void comp_filter:: calculate_gyro_bias(){
  t = millis();
  starting_time = t;
  int i = 0;
  while (i < 200)
  {
    if (millis() > t + Tupdate) {
      dT = millis() - t; // Capture time interval
      t = millis();
      imu.read(); // Nawid - Is there a difference between calling it before or after the if statement
      measurement = -(float) (imu.g.z - avg) * K_rot;

      if (i == 0)
      {
        total_gyro_heading = total_gyro_heading + (measurement * (dT * 0.001));
        first_gyro_heading = total_gyro_heading;
      }
      else
      {
        total_gyro_heading = total_gyro_heading + (measurement * (dT * 0.001));
      }
      i = i + 1; // Nawid - Increases the value for the loop


    }
  }
  final_time = millis();
  bias_per_timestep = (total_gyro_heading - first_gyro_heading) / 10;

  Serial.print(starting_time);
  Serial.print(",");
  Serial.print(final_time);
  Serial.print(",");
  Serial.print(total_gyro_heading);
  Serial.print(",");
  Serial.print(first_gyro_heading);
  Serial.print(",");
  Serial.println(bias_per_timestep);
}
*/

float comp_filter:: complementary_filter_calc() // Nawid - Function to perform entire complementary filter calculation.
{
   
   if (millis() > t + Tupdate) {
   long dT=millis()-t; // Capture time interval
    t = millis();
    imu.read(); // Nawid - Is there a difference between calling it before or after the if statement
    float measurement = -(float) (imu.g.z - avg) * 0.0175;

    mag.read();
//    float mag_x = mag.m.x;
    calibrated_reading_x = scale_x * (mag.m.x - offset_x)/6842;
//    float mag_y = mag.m.y;
    calibrated_reading_y = scale_y * (mag.m.y - offset_y)/6842;
    
    float mag_heading = (atan2(calibrated_reading_y,calibrated_reading_x) -initial_mag_heading_value)*(57296 / 1000); // Nawid - Calculation of the heading angle - THIS IS BETWEEN PI AND - PI but by taking into account the offset and radian to degree conversion, it turns to degrees. 
    mag_heading = fmod((mag_heading) + 360, 360);
     
    if (abs(filtered_mag_heading - mag_heading) > 300) // Nawid - This looks at the difference between the running average(filtered_reading) and the current reading to see if there is a change between 0 and 360
    {
       if (filtered_mag_heading > mag_heading)
       {
        filtered_mag_heading = filtered_mag_heading - 360; // Nawid - If the filtered mag heading is higher, then substract 360 from it
       }

       else if (mag_heading >filtered_mag_heading) // Nawid - If the new reading is higher than the other reading, then subtract 360
       {
          mag_heading = mag_heading - 360; // Nawid - 
          
       }
    }
    filtered_mag_heading = (mag_heading*0.3) +(filtered_mag_heading*(1-0.3)); // Nawid - Filtered Magnetometer reading of the heading

    filtered_mag_heading = fmod((filtered_mag_heading) + 360, 360);

    
    float filtered_mag_heading_comp =  filtered_mag_heading; // Nawid - This is a new variable so any changes to the filtered_mag_heading variable during the comp_filter change wont affect the value beforehand in filtered_mag_heading wont affect the low pass filter
//    gyro_heading = gyro_heading + (measurement*(dT*0.001)); 

    float gyro_reading = measurement*(dT*0.001); // Nawid - This is used to gyro change in a single timestep
    
    if (abs(gyro_reading) < 0.02) // Nawid - Added a threshold to the value if we want to decrease the value
    {
      gyro_reading = 0; 
    }
    

    if (abs((comp_heading + gyro_reading) - filtered_mag_heading_comp ) > 200) // Nawid - This is used to fix the change in the 0 and 360 shift
    {
       if ((comp_heading + gyro_reading) > filtered_mag_heading_comp) // Nawid - If the previous comp_heading and the gyroheading is greater than the other filtered mag heading
       {
        comp_heading = comp_heading - 360; // Nawid - If the filtered mag heading is higher, then substract 360 from it
       }

       else if (filtered_mag_heading_comp  >(comp_heading + gyro_reading))
       {
          filtered_mag_heading_comp  = filtered_mag_heading_comp - 360;
          
       }
    }
    comp_heading = 0.9 *(comp_heading + gyro_reading) + ((1-0.9)*filtered_mag_heading_comp); // Nawid - This is the value in degrees as well as taking into account the initial offset
    
    
    comp_heading = fmod((comp_heading) + 360, 360);
//
//   Serial.print(mag_heading);
//   Serial.print(",");
//   Serial.print(filtered_mag_heading);
//   Serial.print(",");
//   Serial.print(gyro_reading);
//   Serial.print(",");
//   Serial.println(comp_heading);
   }
return comp_heading;
}



#endif
