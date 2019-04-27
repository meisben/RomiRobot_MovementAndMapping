#ifndef _Motor_h
#define _Motor_h
#include <stdint.h>
const int default_max_speed = 255;

class Motor
{
  public:

    void setSpeed(int demand);
    int getSpeed();
    void setMaxSpeed(int newMax);
    void setPins(int pwm, int dir);
    void setDebug(bool state);


  private:

    int pwm_pin;
    int dir_pin;

    int maxSpeed=default_max_speed;
    int speed=0;
    bool debug=false;

};

void Motor::setPins(int pwm, int dir)
{
  pwm_pin = pwm;
  dir_pin = dir;

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  digitalWrite(pwm_pin, 0);
  digitalWrite(dir_pin, 0);
}

void Motor::setSpeed(int demand)
{
  int mag;

  //Handle setting directions
  //For digitalWrite: HIGH => Forward, LOW => Backwards
  if (demand < 0)
  {
    mag = -1 * demand;
    digitalWrite(dir_pin, LOW); //note that i can optimise this code and that we don't have to write to these pins every loop!
  }
  else 
  {
    digitalWrite(dir_pin, HIGH);
    mag = demand;   
  }
  
  if (mag > maxSpeed)
  {
    mag = maxSpeed;
  }
  
  //Write to the H-Bridge
  analogWrite(pwm_pin, mag);

  //Store current speed
  if (mag < 0)
  {
    speed = mag * -1;
  } 
  else
  {
    speed = mag;
  }

  if (debug)
  {
    Serial.print("PWM Value ");
    Serial.println(mag);
  }
}

int Motor::getSpeed()
{
  return speed;
}

void Motor::setMaxSpeed(int newMax)
{
  if (newMax > 255)
  {
    newMax = 255;
  }
  maxSpeed = newMax;

  if (debug)
  {
    Serial.print("Max speed: ");
    Serial.println(maxSpeed);
  }
}

void Motor::setDebug(bool state)
{
  debug = state;
}

#endif
