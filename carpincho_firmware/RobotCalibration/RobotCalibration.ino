
#include "MotorController.h"
#include "RobotController.h"

//Motor A
#define PWMA 10
#define IN1 9
#define IN2 8

//Motor B
#define IN3 7
#define IN4 6
#define PWMB 5


// Encoders
int  ENC_A = 3;
int  ENC_B = 2;

float kpU = 0.25, kiU = 3.5, kdU = 0;
float kpW = 0.07, kiW = 1.5, kdW = 0;


float kpA = 80, kiA = 250;
float kpB = 35, kiB = 250;


//System stepTime
volatile unsigned currentStepTime = 0;
volatile unsigned previousStepTime = 0;
volatile unsigned deltaStepTime = 0;

//Tick for interruption
volatile unsigned currentTimeTickA = 0;

volatile unsigned currentTimeTickB = 0;


int stepTime = 10;

//Motors
MotorController MotorA{PWMA, IN2,IN1};
MotorController MotorB{PWMB, IN3,IN4};
RobotController robot{5.0, 0.0};

void setup() {
  Serial.begin(115200);
  pinMode(ENC_B, INPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_A, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderB, FALLING);

  MotorA.setPID(kpA, kiA, 0.0);
  MotorB.setPID(kpB, kiB, 0.0);
  robot.setPIDUValues(kpU, kiU, kdU);
  robot.setPIDWValues(kpW, kiW, kdW);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentStepTime = millis();
  currentTimeTickA = currentStepTime;
  currentTimeTickB = currentStepTime;
  deltaStepTime = (double) currentStepTime -  previousStepTime;

  if(deltaStepTime >= stepTime) {
    float deltaT = ((float) deltaStepTime) / 1000;
    
    robot.pidController(MotorA.getMotorSpeed(), MotorB.getMotorSpeed(), deltaT);
    MotorA.pidController(robot.vRPM_A(),deltaT);
    MotorB.pidController(robot.vRPM_B(),deltaT);
    
/*    Serial.print(2.1);
    Serial.print("      ");
    Serial.print(MotorA.getMotorSpeed());
    Serial.print("      ");
    Serial.println(MotorB.getMotorSpeed());*/
    
    
    previousStepTime = currentStepTime;
  }
  
}

void encoderA() {
  MotorA.readEncoder(currentTimeTickA);
}


void encoderB() {
   MotorB.readEncoder(currentTimeTickB);
}
