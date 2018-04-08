//
// Created by tom on 4/8/18.
//

#ifndef PID_ROBOT_H
#define PID_ROBOT_H

#include "PID.h"

class Robot {
public:
  /*
  * Constructor
  */
  Robot();

  /*
  * Destructor.
  */
  virtual ~Robot();

  /*
  * Initialize PID.
  */
  void Init(double Kp_steering, double Ki_steering, double Kd_steering,
            double Kp_speed, double Ki_speed, double Kd_speed);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte_steering, double speed, double steering_angle);

  /*
  * Calculate the total PID error.
  */
  double GetThrottle();

  double GetSteeringAngle();

private:
  double controlSpeed;
  double targetSpeed;
  PID speed_pid;
  PID steering_pid;

  void UpdateTargetSpeed(double cte_steering, double steering_angle, double speed);

};


#endif //PID_ROBOT_H
