#include "MotorController.h"



MotorController::MotorController(int pwm, int in1, int in2) : pwm_{pwm}, in1_{in1}, in2_{in2} { }

void MotorController::setPID(const float& kp, const float& ki, const float& kd){
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void MotorController::setMotor() {
   analogWrite(pwm_, pwm_val_);
  
  if(dir_ == 1) {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, HIGH);
  } else if(dir_ == -1) {
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, LOW);
  } else {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
  }
}

float MotorController::getMotorSpeed(){
  return v1Filt;
}

void MotorController::readEncoder(volatile unsigned currentTimeTick) {
  volatile unsigned deltaTick = (double) currentTimeTick - previousTimeTick;
  if(deltaTick > 8) {
    int increment = 0;
    if (dir_ == 1) {
      increment = 1;  
    } else {
      increment = -1;
    }
      pos_i += increment;
      previousTimeTick = currentTimeTick;
  }
}

void MotorController::pidController(float setPoint, float deltaT) {
  dir_ = 1;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
  }
  //Compute velocity with method 1
    
  float velocity = ((float) (pos - posPrev))/deltaT;
  posPrev = pos;
  
  float v1 = velocity/40.0;
  
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  
  float e = setPoint - v1Filt;

  float dedt = (e-eprev)/(deltaT);
  eprev = e;
  
  eintegral += e * deltaT;
  
  float u = kp_ * e + kd_ * dedt + ki_ * eintegral;

  if(u < 0) {
    dir_ = -1;
  }

  pwm_val_ = (int) fabs(u);
  if(pwm_val_ > 255) {
    pwm_val_ = 255;
  }

  setMotor();
}
