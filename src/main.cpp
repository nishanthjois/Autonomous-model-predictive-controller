#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"



// Fields:

// ptsx (Array) - The global x positions of the waypoints.
// ptsy (Array) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
// psi (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
// psi_unity (float) - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
// x (float) - The global x position of the vehicle.
// y (float) - The global y position of the vehicle.
// steering_angle (float) - The current steering angle in radians.
// throttle (float) - The current throttle value [-1, 1].
// speed (float) - The current velocity in mph.


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          Eigen::VectorXd ptsx(6);
          Eigen::VectorXd ptsy(6);
          //vector<double> ptsx = j[1]["ptsx"];
          //vector<double> ptsy = j[1]["ptsy"];
          ptsx << j[1]["ptsx"][0], j[1]["ptsx"][1], j[1]["ptsx"][2], j[1]["ptsx"][3], j[1]["ptsx"][4], j[1]["ptsx"][5];
          ptsy << j[1]["ptsy"][0], j[1]["ptsy"][1], j[1]["ptsy"][2], j[1]["ptsy"][3], j[1]["ptsy"][4], j[1]["ptsy"][5];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Transform ptsx, ptsy to coordinate system (from world space to car space)
          int sizex = ptsx.size();

          Eigen::VectorXd ptsx_car(sizex);
          Eigen::VectorXd ptsy_car(sizex);

          for(int i=0; i<ptsx.size(); i++){
    
            double dtx = ptsx[i] - px;
            double dty = ptsy[i] - py;
            
            ptsx[i] = dtx * cos(psi) + dty * sin(psi);
            ptsy[i] = dty * cos(psi) - dtx * sin(psi);
            }

          // Find CTE

          // Ref: https://github.com/udacity/CarND-MPC-Quizzes/blob/master/polyfit/solution/main.cpp#L47
          // Pass the x and y waypoint coordinates along the order of the polynomial.
          // In this case, 3.
          auto coeffs = polyfit(ptsx, ptsy, 3);
          double cte = polyeval(coeffs, 0);
         
          // Find orientation error

          // Ref: https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp#L323
          //  Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = atan(coeffs[1]);
          cout << " epsi " << epsi << endl;

          Eigen::VectorXd state(6);
          // current state vector (x,y,psi,v,cte,epsi) from the car point of view
          state << 0, 0, 0, v, cte, -epsi;

          
          // Get data from MPC
          vector<double> path_x, path_y;
          // ref: https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp#L342
          auto vars = mpc.Solve(state, coeffs, path_x, path_y);

          double steer_value=-vars[6];
          double throttle_value=vars[7];
          cout << " steer_value " << steer_value << endl ;

          json msgJson;
          
          vector <double> next_x(ptsx.size());
          vector <double> next_y(ptsy.size());

            for(int i = 0; i < ptsx.size(); i++){
                next_x[i] = ptsx[i];
                next_y[i] = ptsy[i];
            }
             // Yellow line: Display the waypoints/reference line
            msgJson["next_x"] = next_x; 
            msgJson["next_y"] = next_y;

            // Display the MPC predicted trajectory 
            // the points in the simulator are connected by a Green line
            msgJson["mpc_x"] = path_x;
            msgJson["mpc_y"] = path_y;


            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
