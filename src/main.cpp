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
                       uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    const vector<double> ptsx = j[1]["ptsx"];
                    const vector<double> ptsy = j[1]["ptsy"];
                    const double px = j[1]["x"];
                    const double py = j[1]["y"];
                    const double psi = j[1]["psi"];
                    const double v = j[1]["speed"];
                    
                    // Transform coordinats from map to vehicle CoSy
                    const unsigned n_points = ptsx.size();
                    Eigen::VectorXd ptsx_vehicle(n_points);
                    Eigen::VectorXd ptsy_vehicle(n_points);
                    
                    for(unsigned i = 0U; i < n_points; i++)
                    {
                        ptsx_vehicle(i) = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
                        ptsy_vehicle(i) = cos(psi) * (ptsy[i] - py) - sin(psi) * (ptsx[i] - px);
                    }
                    
                    // Use a polynomial of 3rd order to fit the road
                    const auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);
                    const double cte = polyeval(coeffs, 0);
                    const double epsi = -atan(coeffs[1]);
                    
                    // Read latest throttle and steering requests
                    const double steer_value = j[1]["steering_angle"];
                    const double throttle_value = j[1]["throttle"];
                    
                    // Update current state as the control request is delayed due to latency
                    const double latency = 0.1;
                    const double Lf = 2.67;
                    
                    const double delayed_x = v * latency; // Vehicle will be at this position after control request has been sent
                    const double delayed_y = 0; // no y movement within vehicle CoSy
                    const double delayed_psi = -v * steer_value / Lf * latency; // Predict turn rate
                    const double delayed_v = v + throttle_value * latency; // Estimate velocity (could be improved)
                    const double delayed_cte = cte + v * sin(epsi) * latency; // Predict cross track error with current estimation, velcoity, orientation error  and delay
                    const double delayed_epsi = epsi - v * steer_value / Lf * latency; // Estimate orientation error to be reduced with latest steering value
                    
                    // Set current state
                    Eigen::VectorXd state(6);
                    state << delayed_x, delayed_y, delayed_psi, delayed_v, delayed_cte, delayed_epsi;
                    
                    // Calculate steeering angle and throttle using MPC.
                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    double mpc_steer_value, mpc_throttle_value;
                    
                    mpc.Solve(state, coeffs, mpc_steer_value, mpc_throttle_value, mpc_x_vals, mpc_y_vals);
                    
                    // Set up json message for control request
                    json msgJson;
                    msgJson["steering_angle"] = mpc_steer_value;
                    msgJson["throttle"] = mpc_throttle_value;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                    // Display the waypoints/reference line
                    vector<double> next_x_vals(ptsx_vehicle.data(), ptsx_vehicle.data() + ptsx_vehicle.rows() * ptsx_vehicle.cols());
                    vector<double> next_y_vals(ptsy_vehicle.data(), ptsy_vehicle.data() + ptsy_vehicle.rows() * ptsy_vehicle.cols());
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
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
                    const unsigned latency_in_ms = static_cast<unsigned>(latency * 1000.0);
                    this_thread::sleep_for(chrono::milliseconds(latency_in_ms));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
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
