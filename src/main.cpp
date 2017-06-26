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
#include "util.h"

// for convenience
using json = nlohmann::json;

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

Eigen::VectorXd CalcDerivative(const Eigen::VectorXd& poly)
{
  const int order = poly.size() - 1;
  Eigen::VectorXd derivative(order);
  for (int i = 0; i < order; ++i)
  {
    derivative[i] = poly[i + 1] * (order + 1);
  }
  return derivative;
}

template<typename T>
void printList(const vector<T>& list, const char* name)
{
  if (name != nullptr)
  {
    cout << name << ":";
  }

  for (const T& t: list)
  {
    cout << t;
  }
  cout << endl;
}

void printVec(const Eigen::VectorXd& v, const char* name)
{
  if (name != nullptr)
  {
    cout << name << ":";
  }

  for (int i = 0; i < v.size(); ++i)
  {
    if (i > 0)
    {
      cout << ',';
    }
    cout << v[i];
  }
  cout << endl;
}

Eigen::VectorXd ConvertToVectorXd(const std::vector<double>& pts)
{
  Eigen::VectorXd vec(pts.size());
  for (size_t i = 0; i < pts.size(); i++)
  {
    vec[i] = pts[i];
  }
  return vec;
}

void CalculateWaypoints(const Eigen::VectorXd& poly, 
        const double xcar,
        const double ycar,
        vector<double>& xway,
        vector<double>& yway)
{
  const int c_waypointCount = 6;
  for (int x = 0; x < c_waypointCount; x++)
  {
    double y = polyeval(poly, static_cast<double>(x) + xcar) - ycar;

    xway.push_back(x);
    yway.push_back(y);
  }
}

void GlobalToCar(vector<double>& ptsx,
                 vector<double>& ptsy,
                 double xshift,
                 double yshift,
                 double psi)
{
  // This is based on driveWell's post at:
  // https://discussions.udacity.com/t/not-able-to-display-trajectory-and-reference-paths-in-the-simulator/248545/9
  for (size_t i = 0; i < ptsx.size(); ++i)
  {
    const double& x = ptsx[i];
    const double& y = ptsy[i];
    double cospsi = cos(psi);
    double sinpsi = sin(psi);
    double xtrans = x - xshift;
    double ytrans = y - yshift;
    double xnew = xtrans * cospsi + ytrans * sinpsi;
    double ynew = -xtrans * sinpsi + ytrans * cospsi;

    ptsx[i] = xnew;
    ptsy[i] = ynew;
  }
}

struct State
{
  const double x;
  const double y;
  const double psi;
  const double v;

  State Predict(double latencySeconds,
                double delta,
                double acceleration) const;
};

State State::Predict(double latencySeconds,
                     double delta,
                     double acceleration) const
{
  return State { x + v * cos(psi) * latencySeconds,
                 y + v * sin(psi) * latencySeconds,
                 psi + v * delta/Lf * latencySeconds,
                 v + acceleration * latencySeconds };
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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          // double psi_unity = j[1]["psi_unity"];
          double mph = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];

          // Workaround unused warning.
          static const double mps_in_mph = 0.44704;
          double v = mps_in_mph * mph;

          vector<double> waypointsx(ptsx);
          vector<double> waypointsy(ptsy);

          GlobalToCar(waypointsx, waypointsy, px, py, psi);

          State current { px, py, psi, v };

          double latencySeconds = 0.1;

          State future = current.Predict(latencySeconds, delta, acceleration);

          GlobalToCar(ptsx, ptsy, future.x, future.y, psi);

          auto poly = polyfit(ConvertToVectorXd(ptsx), ConvertToVectorXd(ptsy), 3);

          double epsi = -atan(poly[1]);

          printVec(poly, "poly");
          // printVec(derivative, "derivative");

          // Polynomial is from car's perspective, so cte is f(0).
          double cte = polyeval(poly, 0);
          cout << "cte: " << cte << "epsi: " << epsi << endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd state(6);
          state << future.x, future.y, future.psi, future.v, cte, epsi;

          auto actuations = mpc.Solve(state, poly);

          double steer_value = - actuations[0] / deg2rad(25);
          double throttle_value = actuations[1];
          cout << "steer: " << steer_value << " throttle: " << throttle_value << endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          // CalculateWaypoints(poly, 0, 0, next_x_vals, next_y_vals);

          msgJson["next_x"] = waypointsx;
          msgJson["next_y"] = waypointsy;


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
