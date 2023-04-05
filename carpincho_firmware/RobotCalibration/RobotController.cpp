#include "RobotController.h"


RobotController::RobotController(const float& u, const float& w) : u_input{u}, w_input{w}{ };

void RobotController::setPIDUValues(const float& kp, const float& ki, const float& kd) {
  kpU_ = kp;
  kiU_ = ki;
  kdU_ = kd;
}

void RobotController::setPIDWValues(const float& kp, const float& ki, const float& kd) {
  kpW_ = kp;
  kiW_ = ki;
  kdW_ = kd;
} 

void RobotController::calculateRobotLocomotion(const float& v1rpm, const float& v2rpm) {
  float u = (PI * D * (v1rpm + v2rpm)) / 120;
  float w = (PI * D * (v1rpm - v2rpm)) / (60 * B);
  lowPassFilter(u, w);
}

void RobotController::pidController(const float& v1rpm, const float& v2rpm, float deltaT) {
    calculateRobotLocomotion(v1rpm, v2rpm);
  //Error Proporcional
    Serial.print(u_input);
    Serial.print("      ");
    Serial.print(uFilt);
    Serial.print("      ");
    Serial.print(wFilt);
    Serial.print("      ");
    Serial.println(w_input);
    
    float errorU = u_input - uFilt;
    float errorW = w_input - wFilt;

    //Error Integral
    eintegralU += errorU * deltaT;
    eintegralW += errorW * deltaT;

    //ErrorDerivativo
    float edevU = (errorU - eprevU) / deltaT;
    float edevW = (errorW - eprevW) / deltaT;
    eprevU = errorU;
    eprevW = errorW;
    
    float u_desired = errorU * kpU_ + eintegralU * kiU_ + edevU * kdU_;
    float w_desired = errorW * kpW_ + eintegralW * kiW_ + edevW * kdW_; 

    //Input Reference
    vRPM_A_ = (30 * ( 2 * u_desired + w_desired * B)) / (PI*D);
    vRPM_B_ = (30 * ( 2 * u_desired - w_desired * B)) / (PI*D);
}


void RobotController::lowPassFilter(const float& u, const float& w){
  // Low-pass filter (25 Hz cutoff)
  uFilt = 0.854*uFilt + 0.0728*u + 0.0728*uPrev;
  uPrev = u;
  wFilt = 0.854*wFilt + 0.0728*w + 0.0728*wPrev;
  wPrev = w;
}


float& RobotController::vRPM_A() { return vRPM_A_;}
float& RobotController::vRPM_B() { return vRPM_B_;}
