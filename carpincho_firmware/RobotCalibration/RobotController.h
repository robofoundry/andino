#pragma once
#include <util/atomic.h>
#include <Arduino.h>


class RobotController {

  public:  
    RobotController(const float& u, const float& w);

    void setPIDUValues(const float& kp, const float& ki, const float& kd);
    void setPIDWValues(const float& kp, const float& ki, const float& kd);
    void pidController(const float& v1rpm, const float& v2rpm, float deltaT);
    float& vRPM_A();
    float& vRPM_B();
    
  private:
    //Private Methods
    void calculateRobotLocomotion(const float& v1rpm, const float& v2rpm);
    void lowPassFilter(const float& u, const float& w);
    
    // Robot Constants
    const float D = 65.5;
    const float B = 153.3;
    //OUTPUT
    float vRPM_A_{0.0};
    float vRPM_B_{0.0};
    //InputFilter
    float wFilt{0};
    float wPrev{0};
    float uFilt{0};
    float uPrev{0};
    //Control Errors
    float eintegralU{0};
    float eprevU{0.0};
    float eintegralW{0};
    float eprevW{0.0};
    //Set Point
    float u_input{0.0};
    float w_input{0.0};
    //u Constants controller
    float kpU_{0.0};
    float kdU_{0.0};
    float kiU_{0.0};
    //w Constants controller
    float kpW_{0.0};
    float kdW_{0.0};
    float kiW_{0.0};
};
