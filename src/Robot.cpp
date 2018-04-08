//
// Created by tom on 4/8/18.
//
#include "Robot.h"
#include <iostream>
#include <math.h>
using namespace std;
#define likely(x)      __builtin_expect(!!(x), 1)

/*
* TODO: Complete the PID class.
*/

Robot::Robot() {
}

Robot::~Robot() {}

void Robot::Init(double Kp_steering, double Ki_steering, double Kd_steering,
          double Kp_speed, double Ki_speed, double Kd_speed){
  this->speed_pid.Init(Kp_speed, Ki_speed, Kd_speed);
  this->speed_pid.SetIntegralMax(60);
  this->steering_pid.Init(Kp_steering, Ki_steering, Kd_steering);
  this->targetSpeed = 60;

}

/*
* Update the PID error variables given cross track error.
*/
void Robot::UpdateError(double cte_steering, double speed, double steering_angle){
  this->UpdateTargetSpeed(cte_steering, speed, steering_angle);

  double cte_speed = controlSpeed - speed;
  //printf("debug %.3f %.3f %.3f\n ", cte_speed, speed, targetSpeed);
  this->speed_pid.UpdateError(cte_speed);
  this->steering_pid.UpdateError(cte_steering);
}

void Robot::UpdateTargetSpeed(double cte_steering, double speed, double steering_angle){
  double abs_cte_steering = fabs(cte_steering);
  double speed_ = 0;
  if(abs_cte_steering < 0.1)
    speed_ = targetSpeed;
  else if(abs_cte_steering < 0.5)
    speed_ = targetSpeed * 0.9;
  else if(abs_cte_steering < 1)
    speed_ = targetSpeed * 0.65;
  else
    speed_ = targetSpeed * 0.55;

  controlSpeed = 0.7 * controlSpeed + speed_ * 0.3;
}

double Robot::GetThrottle(){
  return -this->speed_pid.TotalError();
}

double Robot::GetSteeringAngle(){
  return this->steering_pid.TotalError();
}