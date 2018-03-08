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
  p_error = cte;

  error = error + cte * cte;
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

/*
void PID::twiddle(double tol=0.2){
  double p[3] = {0, 0, 0};
  double dp[3] = {1, 1, 1};
  double best_error;
  double it = 0;
  double err = 0;
  //run again
  best_error = yield();

  while (dp[0]+dp[1]+dp[2] > tol){
    std::cout << "Iteration " << it << ", best error = " << best_error << std::endl;

    for(int i=0; i<3; ++i){
      p[i] += dp[i];
      //run again
      best_error = yield();

      if (err < best_error){
        best_error = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] -= 2* dp[i];
        //run again
        best_error = yield();

        if (err < best_error){
          best_error = err;
          dp[i] *= 1.1;
        }
        else{
          p[i] += dp[i];
          dp[i] *= 0.9;
        }

      }
    }
    it +=1;
  }
  std::cout << "Completed parameters are: " << dp[0] << " " << dp[1] << " " << dp[2] << ", best error = " << best_error << std::endl;
}
*/