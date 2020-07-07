#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s; // Frenet s
    vector<double> map_waypoints_dx; // Frenet d
    vector<double> map_waypoints_dy; // Frenet d
    
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    
    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
    
    // Start in lane 1
    // The left lane is lane 0, middle is lane 1, right is lane 2
    int lane = 1;
    
    // Reference velocity to target (speed limit)
    double ref_vel = 0.0; // miles/h
    
    // Remember to pass lane and ref_vel to the function
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                 &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                 uWS::OpCode opCode) {
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
                        
                        auto s = hasData(data);
                        
                        if (s != "") {
                            auto j = json::parse(s);
                            
                            string event = j[0].get<string>();
                            
                            if (event == "telemetry") {
                                // j[1] is the data JSON object
                                
                                // Main car's localization data
                                double car_x = j[1]["x"];
                                double car_y = j[1]["y"];
                                double car_s = j[1]["s"];
                                double car_d = j[1]["d"];
                                double car_yaw = j[1]["yaw"];
                                double car_speed = j[1]["speed"];
                                
                                // Previous path data given to the Planner
                                auto previous_path_x = j[1]["previous_path_x"];
                                auto previous_path_y = j[1]["previous_path_y"];
                                // Previous path's end s and d values
                                double end_path_s = j[1]["end_path_s"];
                                double end_path_d = j[1]["end_path_d"];
                                
                                // Sensor Fusion Data, a list of all other cars on the same side
                                //   of the road with [id,x,y,vx,vy,s,d]
                                auto sensor_fusion = j[1]["sensor_fusion"];
                                
                                json msgJson;
                                
                                vector<double> next_x_vals;
                                vector<double> next_y_vals;
                                
                                /**
                                 * TODO: define a path made up of (x,y) points that the car will visit
                                 *   sequentially every .02 seconds
                                 */
                                int prev_size = previous_path_x.size();
                                
                                // If we have a previous path, we use the last point from it
                                if (prev_size > 0)
                                    car_s = end_path_s;
                                
                                // Flag for changing lane or not
                                bool change_lane = false;
                                
                                // Loop through every car I am tracking
                                for (int i = 0; i < sensor_fusion.size(); ++i){
                                    float d = sensor_fusion[i][6];
                                    // Check d to see if a car is in my lane
                                    if (d < (2+4*lane+2) && d > (2+4*lane-2)){
                                        double vx = sensor_fusion[i][3];
                                        double vy = sensor_fusion[i][4];
                                        double v = sqrt(vx*vx + vy*vy);
                                        double s = sensor_fusion[i][5];
                                        // Adjust s to the future to consider the Simulator delay
                                        s += (double)prev_size * 0.02 * v;
                                        // If the tracked car is in front of me
                                        // and the distance is under a safe value
                                        // and it is driving slower than me
                                        // then I should consider lane changing
                                        if (s > car_s && (s - car_s) < 30 && v < car_speed)
                                            change_lane = true;
                                    }
                                }
                                
                                // Check whether we can change the lane
                                if (change_lane){
                                    int new_lane = 0;
                                    int new_lane2 = 0; // For the middle lane case
                                    if (lane == 0 || lane == 2)
                                        new_lane = 1;
                                    // Loop through every car I am tracking
                                    for (int i = 0; i < sensor_fusion.size(); ++i){
                                        float d = sensor_fusion[i][6];
                                        // Check d to see if a car is in the new lane
                                        if (d < (2+4*new_lane+2) && d > (2+4*new_lane-2)){
                                            double x = sensor_fusion[i][1];
                                            double y = sensor_fusion[i][2];
                                            double vx = sensor_fusion[i][3];
                                            double vy = sensor_fusion[i][4];
                                            double v = sqrt(vx*vx + vy*vy);
                                            double s = sensor_fusion[i][5];
                                            s += (double)prev_size * 0.02 * v;
                                            // Conditions for lane change not allowed:
                                            // distance between two cars is already very small, or
                                            // I don't have a safe distance to the car in the front, or
                                            // I don't have a safe distance to the car behind driving faster than me
                                            if ( distance(car_x,car_y,x,y) < 10 || (s > car_s && s - car_s < 50)  ||
                                                (s < car_s && car_s - s < 30 && v > car_speed) ){
                                                change_lane = false;
                                                break;
                                            }
                                        }
                                    }
                                    // If I am in the middle lane, and changing to the left lane is not feasible
                                    // I want to check whether I can change to the right lane
                                    if (change_lane == false && lane ==1){
                                        new_lane2 = 2;
                                        change_lane = true;
                                        for (int i = 0; i < sensor_fusion.size(); ++i){
                                            float d = sensor_fusion[i][6];
                                            if (d < (2+4*new_lane2+2) && d > (2+4*new_lane2-2)){
                                                double x = sensor_fusion[i][1];
                                                double y = sensor_fusion[i][2];
                                                double vx = sensor_fusion[i][3];
                                                double vy = sensor_fusion[i][4];
                                                double v = sqrt(vx*vx + vy*vy);
                                                double s = sensor_fusion[i][5];
                                                s += (double)prev_size * 0.02 * v;
                                                if ( distance(car_x,car_y,x,y) < 10 || (s > car_s && s - car_s < 50)  ||
                                                    (s < car_s && car_s - s < 30 && v > car_speed) ){
                                                    change_lane = false;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    
                                    // If changing lane is not feasible, reduce speed
                                    // Max allowed jerk is 10 m/s3
                                    // So starting from 0 acceleration to full acceleration of 10 m/s2 is ok
                                    // Max allowed acceleration is 10 m/s2 for change of average speed
                                    // So in 0.02s, a speed difference of 0.2 m/s or 0.447 miles/h is allowed
                                    if (change_lane == false)
                                        ref_vel -= 0.447;
                                    else{
                                        if (new_lane2 == 2)
                                            lane = new_lane2;
                                        else
                                            lane = new_lane;
                                    }
                                }
                                else if (ref_vel < 49.5) // Speed limit is 50 miles/h
                                    ref_vel += 0.447;
                                
                                // Create widely spaced (x,y) waypoints, evenly spaced at 30m
                                // Later we will interpolate these points to get a spline
                                // so that the car can run smoothly
                                vector<double> ptsx, ptsy; // points x, y
                                
                                // Keep track of reference x, y and yaw states
                                // Either referencing the starting point of where the car is now
                                // or the end point of the car's previous path
                                double ref_x = car_x;
                                double ref_y = car_y;
                                double ref_yaw = deg2rad(car_yaw);
                                
                                // If the previous path list is pretty empty
                                // we use the current car state and
                                // calculate the last car state back in the time
                                if (prev_size < 2){
                                    // Use two points to make the path tangent to the car
                                    double prev_car_x = car_x - cos(car_yaw);
                                    double prev_car_y = car_y - sin(car_yaw);
                                    
                                    ptsx.push_back(prev_car_x);
                                    ptsx.push_back(ref_x);
                                    ptsy.push_back(prev_car_y);
                                    ptsy.push_back(ref_y);
                                }
                                // If the previous path list is big enough
                                // we use the last two points
                                else{
                                    ref_x = previous_path_x[prev_size-1];
                                    ref_y = previous_path_y[prev_size-1];
                                    double ref_x_prev = previous_path_x[prev_size-2];
                                    double ref_y_prev = previous_path_y[prev_size-2];
                                    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                                    
                                    ptsx.push_back(ref_x_prev);
                                    ptsx.push_back(ref_x);
                                    ptsy.push_back(ref_y_prev);
                                    ptsy.push_back(ref_y);
                                }
                                
                                // In Frenet, add evenly 30m spaced points ahead of the reference points
                                vector<double> next_wp0 = getXY(car_s+30, 2.0+4*lane, map_waypoints_s, 
                                                                map_waypoints_x, map_waypoints_y);
                                vector<double> next_wp1 = getXY(car_s+60, 2.0+4*lane, map_waypoints_s, 
                                                                map_waypoints_x, map_waypoints_y);
                                vector<double> next_wp2 = getXY(car_s+90, 2.0+4*lane, map_waypoints_s, 
                                                                map_waypoints_x, map_waypoints_y);
                                ptsx.push_back(next_wp0[0]);
                                ptsx.push_back(next_wp1[0]);
                                ptsx.push_back(next_wp2[0]);
                                ptsy.push_back(next_wp0[1]);
                                ptsy.push_back(next_wp1[1]);
                                ptsy.push_back(next_wp2[1]);
                                
                                // Shift our viewpoint to the car to make the spline math easier
                                for (int i = 0; i < ptsx.size(); ++i){
                                    double shift_x = ptsx[i] - ref_x;
                                    double shift_y = ptsy[i] - ref_y;
                                    ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
                                    ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
                                }
                                
                                // Sort ptsx and ptsy for spline function
                                vector<double> ptsx_sorted, ptsy_sorted;
                                for (int i = 0; i < 5; ++i){ // ptsx and ptsy have a size of 5
                                    int x_min_index = std::min_element(ptsx.begin(),ptsx.end()) - ptsx.begin();
                                    ptsx_sorted.push_back(ptsx[x_min_index]);
                                    ptsy_sorted.push_back(ptsy[x_min_index]);
                                    ptsx.erase(ptsx.begin() + x_min_index);
                                    ptsy.erase(ptsy.begin() + x_min_index);
                                }
                                
                                // Create a spline using ptsx and ptsy
                                tk::spline spline;
                                spline.set_points(ptsx_sorted, ptsy_sorted);
                                
                                // Now let's begin to add points to next x and y values
                                // First divide the spline into N points
                                // Every point represent how far the car travels in 0.02s
                                double target_x = 30.0;
                                double target_y = spline(target_x); // Feed an x to s and we get y
                                double target_dist = sqrt(target_x * target_x + target_y * target_y);
                                double N = target_dist / (0.02*ref_vel / 2.24); // miles/h to meters/s
                                double increment = target_x / N;
                                
                                // Use all previous path points to get a smooth path
                                // Previous path size from the simulator should be smaller than 50
                                for (int i = 0; i < prev_size; ++i) {
                                    next_x_vals.push_back(previous_path_x[i]);
                                    next_y_vals.push_back(previous_path_y[i]);
                                }
                                
                                // Fill the path planner to 50 points
                                for (int i = 0; i < 50-prev_size; ++i) {    
                                    double x_point = increment * (i+1);
                                    double y_point = spline(x_point);
                                    
                                    // Go from car local coordinates back to global coordinates
                                    double x_temp = x_point; // Save the x and y point to "clipboard"
                                    double y_temp = y_point;
                                    x_point = (x_temp * cos(ref_yaw) - y_temp * sin(ref_yaw));
                                    y_point = (x_temp * sin(ref_yaw) + y_temp * cos(ref_yaw));
                                    x_point += ref_x;
                                    y_point += ref_y;
                                    
                                    next_x_vals.push_back(x_point);
                                    next_y_vals.push_back(y_point);
                                }
                                // END TODO
                                
                                msgJson["next_x"] = next_x_vals;
                                msgJson["next_y"] = next_y_vals;
                                
                                auto msg = "42[\"control\","+ msgJson.dump()+"]";
                                
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }  // end "telemetry" if
                        } else {
                            // Manual driving
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }  // end websocket if
                }); // end h.onMessage
    
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