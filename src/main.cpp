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

int main()
{
  uWS::Hub h;

  PID pid_steer;
  PID pid_speed;
  // TODO: Initialize the pid variable.
  static const double Kp_steer0 = 0.02;
  static const double Ki_steer0 = 2e-4 ;
  static const double Kd_steer0 = 0.0;

  static const double Kp_steer1 = Kp_steer0 * 2.0;
  static const double Ki_steer1 = 1e-4;
  static const double Kd_steer1 = 0.0;

  static const double Kp_steer2 = Kp_steer0 * 4.0  ;
  static const double Ki_steer2 = 1e-4;
  static const double Kd_steer2 = 0.050  ;

  static const double Kp_steer3 = Kp_steer0 * 4.0 ;
  static const double Ki_steer3 = 1e-6 * 0;
  static const double Kd_steer3 = 0.10;

  static const double limit0 = 0.3;
  static const double limit1 = 0.8;
  static const double limit2 = 1.0;

  pid_steer.Init(Kp_steer0, Ki_steer0, Kd_steer0);
  pid_steer.set_error_lim(1.0, -1.0);

  double Kp_speed = 1.0;
  double Ki_speed = 1e-4;
  double Kd_speed = 0.0;
  pid_speed.Init(Kp_speed, Ki_speed, Kd_speed);
  pid_speed.set_error_lim(0.6, -0.6);

  h.onMessage([&pid_steer,&pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          /*
          * Calculate steering value here, remember the steering value is
          * [-1, 1].
          */
          PID * pid_steer_p = &pid_steer;
          if(fabs(cte)<=limit0){
            pid_steer_p->setKp(Kp_steer0)->setKi(Ki_steer0)->setKd(Kd_steer0);
          }
          else if(fabs(cte)<=limit1){
            pid_steer_p->setKp(Kp_steer1)->setKi(Ki_steer1)->setKd(Kd_steer1);
          }
          else if(fabs(cte)<=limit2){
            pid_steer_p->setKp(Kp_steer2)->setKi(Ki_steer2)->setKd(Kd_steer2);
          }
          else{
            pid_steer_p->setKp(Kp_steer3)->setKi(Ki_steer3)->setKd(Kd_steer3);
          }
          pid_steer.UpdateError(cte);
          steer_value = -pid_steer.TotalError();


          /*
           * Using another PID controller to control the speed!
          */
          double speed_target = 50;
          if(fabs(steer_value)>0.3)
            speed_target = 30;

          pid_speed.UpdateError(speed_target-speed);
          double throttle = pid_speed.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value <<"  Throttle:" << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
    ws.close();
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
}
