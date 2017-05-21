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
//#include "matplotlibcpp.h"
// for convenience
using json = nlohmann::json;
using namespace std;
//namespace plt = matplotlibcpp;
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
        if (event == "telemetry")
        {
          static int iteration =0;
          double steer_value;
          double throttle_value;
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];//6 waypoints
          vector<double> ptsy = j[1]["ptsy"];//6 waypoints
            
          double px = j[1]["x"];// global position of vehicle  should be zero in car coordinate
          double py = j[1]["y"];//global position of the vehicle
          double psi = j[1]["psi"];// orientation in radian
          double v = j[1]["speed"];//speed in mph
          //iteration++;
          //cout << "iteration:" << iteration << endl;
          //cout << "WAYPOINTS x: " << ptsx[0] << " " << ptsx[1] << " " << ptsx[2] << " " << ptsx[3] << " " << ptsx[4] << " " << ptsx[5] << endl;
          //cout << "WAYPOINTS y: " << ptsy[0] << " " << ptsy[1] << " " << ptsy[2] << " " << ptsy[3] << " " << ptsy[4] << " " << ptsy[5] << endl;
          //cout << "Px, Py, psi, v: " << px << " " << py << " " << psi << " " << v << endl;
            
          Eigen::VectorXd xvals = Eigen::VectorXd(6);
          xvals.fill(0);
          Eigen::VectorXd yvals = Eigen::VectorXd(6);
          yvals.fill(0);
 
            
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //KEY POINT #1 transform waypoints to car coordinate before computing the polynomial
          // (1) start by translating
          // (2) use -psi angle
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            for (int i = 0; i<6;i++)
          {
            double translation_x = ptsx[i] - px;
            double translation_y = ptsy[i] - py;
            xvals[i] = translation_x * cos(-psi) - translation_y * sin(-psi);
            yvals[i] = translation_x * sin(-psi) + translation_y * cos(-psi);
          }
            
            
          auto coeffs = polyfit(xvals, yvals, 3);
          
         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         // KEY POINT #2 Mitigate the latency with a speed bias parameter
         ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
         // The cross track error is calculated by evaluating at polynomial at x, f(x)
         // and subtracting y.
         double x_latency_bias = v * 0.1;// 100 milliseconds
   
            
         ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         //KEY POINT #3 evaluate the polynomial at x=0, y=0 since we are in car coordinates
         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          double y = 0;
          double x = 0;
          
          double cte = (polyeval(coeffs, x) - y);
          cout << "main() cte: " << cte << endl;
    
          // Due to the sign starting at 0, the orientation error is -f'(x).
         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         //KEY POINT #4 since we use a polynomial of order 3 compute its derivative
         /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          double epsi = - atan(  coeffs[1] + (2 * coeffs[2] * x) + (3 * coeffs[3]* (x*x)) );//2.19074
    
          cout << "epsi: " << epsi  << endl;
        
          mpc.state[0] = x_latency_bias;
          mpc.state[1] = 0.0;
          mpc.state[2] = 0.0;
          mpc.state[3] = v;
          mpc.state[4] = cte;
          mpc.state[5] = epsi;
            
          auto vars = mpc.Solve(mpc.state, coeffs);
          int N = (vars.size() - 2) / 2;
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          // KEY POINT #5 steer value is negatively signed
          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          steer_value = -vars[0];
          throttle_value = vars[1];
            
        json msgJson;
        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = throttle_value;
            
        //display the MPC predicted trajectory in yellow
        vector<double> mpc_ptsx;
        vector<double> mpc_ptsy;
        for(int i = 0;i < N; i++)
        {
            mpc_ptsx.push_back(vars[2+i]);
            mpc_ptsy.push_back(vars[2+N+i]);
        }

       
        msgJson["mpc_x"] = mpc_ptsx;
        msgJson["mpc_y"] = mpc_ptsy;
        
            
        //Display the waypoints/reference line in green
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        for(int i =0;i < ptsx.size();i++)
        {
            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Green line
            
            next_x_vals.push_back(xvals[i]);
            next_y_vals.push_back(yvals[i]);
        }
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;


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
