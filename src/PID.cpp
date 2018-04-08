#include "PID.h"
#include <iostream>
using namespace std;
#define likely(x)      __builtin_expect(!!(x), 1)

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  is_initialized = false;
  i_error = 0;
  d_error = 0;
  error = 0;
  integral_max = -1;
}

void PID::UpdateError(double cte) {

  if (likely(is_initialized)) {
    d_error = cte - p_error;
  }
  else{
    d_error = 0;
    is_initialized = true;
  }

  i_error = i_error + cte;

  if(this->integral_max > 0){
    i_error = max(min(i_error, this->integral_max), -this->integral_max);
  }

  p_error = cte;

  error = error + cte * cte;
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::SetIntegralMax(double max_val){
  integral_max = max_val;
}