#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int move(PID* pid_steering, PID* pid_speed, int timestep, double const_speed, double& error_steering, double& error_speed){

  try{
    uWS::Hub h;

    h.onMessage([&pid_steering, &pid_speed, &timestep, const_speed, &error_steering, &error_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event

      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != "") {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {

            if(timestep > 0){
              // j[1] is the data JSON object
              double cte = std::stod(j[1]["cte"].get<std::string>());
              double speed = std::stod(j[1]["speed"].get<std::string>());
              double angle = std::stod(j[1]["steering_angle"].get<std::string>());
              double steer_value;
              /*
              * TODO: Calcuate steering value here, remember the steering value is
              * [-1, 1].
              * NOTE: Feel free to play around with the throttle and speed. Maybe use
              * another PID controller to control the speed!
              */
              pid_steering->UpdateError(cte);
              steer_value = pid_steering->TotalError();
              error_steering += steer_value * steer_value;

              //speed error
              double cte_speed = const_speed - speed;
              pid_speed -> UpdateError(cte_speed);
              double throttle = -pid_speed->TotalError();
              error_speed += throttle * throttle;

              //steer_value = 0;
              // DEBUG
              //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              timestep--;
            }
            else{
              ws.send("Bye", 3, uWS::OpCode::CLOSE);
            }
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1)
      {
        res->end(s.data(), s.length());
      }
      else
      {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      h.getDefaultGroup<uWS::SERVER>().close();
      std::cout << "Disconnected" << std::endl;
    });


    int port = 4567;
    if (h.listen(port))
    {
      std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
    }
    h.run();
  } catch (...) {
    std::cout << "Exception" << std::endl;
  }


  return 0;
}

void setPIDParameter(PID* pid, double k[3]){
  pid->Init(k[0], k[1], k[2]);
  std::cout << "Kp: " << pid->Kp << "Ki: " << pid->Ki << "Kd: " << pid->Kd << std::endl;
}

void twiddle_steering(){
  PID pid_speed;
  PID pid_steering;

  // TODO: Initialize the pid variable.
  //pid_steering.Init(0.134611, 0.000270736, 3.05349);
  pid_steering.Init(0.2, 0, 3);
  pid_speed.Init(0.271308, 0, 0.0891773);

  double p[3] = {0.134611, 0.000270736, 3.05349};
  setPIDParameter(&pid_steering, p);
  double dp[3] = {0.02, 0.002, 0.02};
  //double dp[3] = {0.0, 0.0, 0.0};
  double best_error;
  double tol = 0.02;
  int it = 1;
  int timestep = 200;
  double const_speed = 70;

  double error_steering, error_speed;

  //run again
  std::cout << "First run." << std::endl;
  error_steering = 0; error_speed=0;
  move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
  best_error = error_steering;

  while (dp[0]+dp[1]+dp[2] > tol){
    std::cout << "Iteration " << it << ", best error = " << best_error << std::endl;

    for(int i=0; i<3; ++i){
      p[i] += dp[i];
      setPIDParameter(&pid_steering, p);

      //run again
      error_steering = 0; error_speed=0;
      move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
      double err=  error_steering;
      std::cout << "Iteration " << it << ":" << i << "a, error = " << err << std::endl;

      if (err < best_error){
        best_error = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] -= 2* dp[i];
        setPIDParameter(&pid_steering, p);
        //run again
        error_steering = 0; error_speed=0;
        move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
        err=  error_steering;
        std::cout << "Iteration " << it << ":" << i << "b, error = " << err << std::endl;

        if (err < best_error){
          best_error = err;
          dp[i] *= 1.1;
        }
        else{
          p[i] += dp[i];
          setPIDParameter(&pid_steering, p);
          dp[i] *= 0.9;
        }

      }
    }
    it++;
  }
  std::cout << "Completed parameters are: " << pid_steering.Kp << " " << pid_steering.Ki << " " << pid_steering.Kd << ", best error = " << best_error << std::endl;
}

void twiddle_speed(){
  PID pid_speed;
  PID pid_steering;

  // TODO: Initialize the pid variable.
  pid_steering.Init(0.15, 0.0, 2.5);
  //pid_steering.Init(0.23, 0, 3);
  pid_speed.Init(0, 0, 0);

  double p[3] = {0.2, 0, 0.02};
  setPIDParameter(&pid_speed, p);

  double dp[3] = {0.05, 0, 0.05};
  double best_error;
  double tol = 0.02;
  int it = 1;
  int timestep = 100;
  double const_speed = 50;

  double error_steering, error_speed;

  //run again
  std::cout << "Twiddle speed: first run." << std::endl;
  error_steering = 0; error_speed=0;
  move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
  best_error = error_speed;

  while (dp[0]+dp[1]+dp[2] > tol){
    std::cout << "Iteration " << it << ", best error = " << best_error << std::endl;

    for(int i=0; i<3; ++i){
      p[i] += dp[i];
      setPIDParameter(&pid_speed, p);

      //run again
      error_steering = 0; error_speed=0;
      move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
      double err=  error_speed;
      std::cout << "Iteration " << it << ":" << i << "a, error = " << err << std::endl;

      if (err < best_error){
        best_error = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] -= 2* dp[i];
        setPIDParameter(&pid_speed, p);
        //run again
        error_steering = 0; error_speed=0;
        move(&pid_steering, &pid_speed, timestep, const_speed, error_steering, error_speed);
        err=  error_speed;
        std::cout << "Iteration " << it << ":" << i << "b, error = " << err << std::endl;

        if (err < best_error){
          best_error = err;
          dp[i] *= 1.1;
        }
        else{
          p[i] += dp[i];
          setPIDParameter(&pid_speed, p);
          dp[i] *= 0.9;
        }

      }
    }
    it++;
  }
  std::cout << "Completed parameters are: " << pid_speed.Kp << " " << pid_speed.Ki << " " << pid_speed.Kd << ", best error = " << best_error << std::endl;
}

int main()
{
  twiddle_steering();

}


