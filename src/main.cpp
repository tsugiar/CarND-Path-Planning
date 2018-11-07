#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper_function.h"
#include "vehicle.h"
#include "spline.h"
#include <algorithm>

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  map<int, Vehicle > vehicle_map;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s  = 6945.554;
  double dest_d = 2.0 + 4.0;   // Second lane (middle) in meters
  double max_speed = 22.352;     // max-speed (m/sec), corresponding to 50 miles/hour
  double max_accel = 10;         // max-accel (m/sec^2) 
  double ref_vel = 0;

  double duration = 1.0;   // Duration of planning and trajectory
  int internal_counter;    // Internal counter

  vector<double> GOAL = { max_s, dest_d, max_speed, max_accel };


  int first_time = 1;      

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
#ifdef _MSC_VER
  h.onMessage([&map_waypoints_x, &map_waypoints_y, 
	  &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, 
	  &first_time, &duration, &vehicle_map,
      &GOAL, &internal_counter, &ref_vel](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
	  uWS::OpCode opCode) {
#else
  h.onMessage([&map_waypoints_x, &map_waypoints_y,
          &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
          &first_time, &duration, &vehicle_map,
          &GOAL, &internal_counter, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
#endif

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

			double car_yaw_rad = deg2rad(car_yaw);
			double car_speed_mps = car_speed * 0.44704;   // Car in unit of meter/sec

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

			int prev_size = previous_path_x.size();

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
			// Each element of sensor fusion data consists of these signals [ id, x, y, vx, vy, s, d].
			// where,
			// id ==> ID of target car
			// x,y ==> position of target car in global map
			// vx,vy ==>  x,y velocity of car in global map
			// s,d   ==>  current position of Frenet coordinate
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			vector<double> pts_x;
			vector<double> pts_y;

			if (prev_size > 0)
			{
				car_s = end_path_s;

			}


			// Increment internal counter
			internal_counter = internal_counter + 1;

			// Perform host-lane assignment	
			int host_lane = GetLaneNo(car_d);
			
			if (first_time)
			{
				// Filling up host-property for first-time, then insert it into vehicle_map<int, Vehicle> dictionary
				// key to the map is hostID, value is host-object. 
				// Host ID uses unique number -1

				Vehicle host_obj = Vehicle(host_lane, car_s, car_d, car_speed_mps);
				
				// Set host configuration when code is executed for first-time
				vector<float> road_data = { static_cast<float>(GOAL[2]),   // target_speed  (m/sec)
													3,					   // Number of available lanes
											static_cast<float>(GOAL[0]),   // goal_s     (m)
											static_cast<float>(GOAL[1]),   // goal_d     (m)
											static_cast<float>(GOAL[3]) };  // max_accel (m/sec^2)
				host_obj.configure(road_data);
				vehicle_map.insert(std::pair<int,Vehicle>(-1, host_obj));  

			}
			else
			{
				// If not the first-time, simply update update host position (s,d,sdot,ddot) from vehicle_map
				// dictionary.  Uses -1 for the key
				vehicle_map[-1].s = car_s;
				vehicle_map[-1].d = car_d;
				vehicle_map[-1].v = car_speed_mps;

			}

			// For each target vehicle perform lane assignment
			for (auto target_property : sensor_fusion)
			{
				// Extract target property
				int   id = target_property[0];
				float s = target_property[5];
				float d = target_property[6];
				float xinit = target_property[1];
				float yinit = target_property[2];
				float vx = target_property[3];
				float vy = target_property[4];

				// Compute velocity and projected distance for next time-step
				float v = sqrt(vx*vx + vy * vy);
				s = s + prev_size * 0.02 * v;
				int target_lane = GetLaneNo(d); 
				if (first_time)
				{
					// Note : target_property = [id, x, y, vx, vy, s, d]
					Vehicle tgt_object = Vehicle(target_lane, s, d, v);

					// Insert target ID and its object into dictionary (vehicle_map)
					vehicle_map.insert(std::pair<int, Vehicle>(id, tgt_object));

					first_time = 0;
				}
				else
				{
					// If it isn't first-time vehicle_map updated, just update
					// individual target with latest inforamtion
					vehicle_map[id].lane = target_lane;
					vehicle_map[id].s = s;
					vehicle_map[id].d = d;
					vehicle_map[id].v = v;
                     
				}

			}

			// ==================================================================
			//   Front-target assesment. If there is vehicle ahead of 
			//   host-vehicle that is too close, slow down host vehicle speed.
			//  Otherwise, increase speed up to 50 mph
			// =================================================================

			Vehicle vehicle_ahead;
			bool too_close = false;
			if (vehicle_map[-1].get_vehicle_ahead(vehicle_map, vehicle_map[-1].lane, vehicle_ahead))
			{
				if (vehicle_ahead.s - car_s < 30.0)
				{
					too_close = true;

				}

			}

			if (too_close)
			{
				ref_vel = ref_vel - 0.1;
				// Limite ref_vel not to go below zero
				ref_vel = ref_vel <= 0.0 ? 0.0 : ref_vel;

			}
			else
			{
				ref_vel = ref_vel + 0.1;
				// Limit ref_vel not to exceed speed limit
				ref_vel = ref_vel >= GOAL[2]-0.2 ? GOAL[2]-0.2 : ref_vel;

			}


			// Check if it is appropriate to make shift lane
			vehicle_map[-1].choose_next_stateVer2(vehicle_map);
			double new_lane_center = vehicle_map[-1].lane;

			// =====================================================
			// Trajectory motions                                  =
			// This part, responsible to make smooth trajectory    =
			// for ego-vehicle to follow                           =
			// =====================================================

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);


			if (prev_size < 2)
			{
				double prev_car_x = car_x - cos(ref_yaw);
				double prev_car_y = car_y - sin(ref_yaw);

				pts_x.push_back(prev_car_x); 
				pts_x.push_back(car_x);

				pts_y.push_back(prev_car_y);
				pts_y.push_back(car_y);

			}
			else
			{

				ref_x = previous_path_x[prev_size - 1];
				ref_y = previous_path_y[prev_size - 1];

				double ref_x_prev = previous_path_x[prev_size - 2];
				double ref_y_prev = previous_path_y[prev_size - 2];
				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

				pts_x.push_back(ref_x_prev);
				pts_x.push_back(ref_x);

				pts_y.push_back(ref_y_prev);
				pts_y.push_back(ref_y);

			}

			// Generate waypoints for spline
			// Modulus operi car_s before feeding it into car_s + x
			double car_s_mod;

			car_s_mod = car_s + 30;
			car_s_mod = car_s_mod > GOAL[0] ? car_s_mod - GOAL[0] : car_s_mod;
			vector<double> next_wp0 = getXY(car_s_mod, 2+(new_lane_center-1)*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			
			car_s_mod = car_s + 60;
			car_s_mod = car_s_mod > GOAL[0] ? car_s_mod - GOAL[0] : car_s_mod;
			vector<double> next_wp1 = getXY(car_s + 60, 2+(new_lane_center-1)*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			car_s_mod = car_s + 90;
			car_s_mod = car_s_mod > GOAL[0] ? car_s_mod - GOAL[0] : car_s_mod;
			vector<double> next_wp2 = getXY(car_s + 90, 2+(new_lane_center-1)*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			pts_x.push_back(next_wp0[0]);
			pts_x.push_back(next_wp1[0]);
			pts_x.push_back(next_wp2[0]);


			pts_y.push_back(next_wp0[1]);
			pts_y.push_back(next_wp1[1]);
			pts_y.push_back(next_wp2[1]);



			// Transformation to local coordinate of car
			for (int i = 0; i < pts_x.size(); ++i)
			{
				double shift_x = pts_x[i] - ref_x;
				double shift_y = pts_y[i] - ref_y;

				pts_x[i] = ( shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw));
				pts_y[i] = (-shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw));

			}

			// Sort pts_x so that it is monotonically increasing
			vector<int> pts_x_indexes(pts_x.size());
			{
				std::size_t n(0);
				std::generate(std::begin(pts_x_indexes), std::end(pts_x_indexes), [&] {return n++; });
				std::sort(std::begin(pts_x_indexes), std::end(pts_x_indexes), [&](int i1, int i2) { return pts_x[i1] < pts_x[i2]; });
			}


			vector<double> pts_x_sorted(pts_x.size()), pts_y_sorted(pts_y.size());
			double temp;
			for (int i = 0; i < pts_x.size(); ++i)
			{
				pts_x_sorted[i] = pts_x[pts_x_indexes[i]];
				pts_y_sorted[i] = pts_y[pts_x_indexes[i]];
			   
			}

			pts_x = pts_x_sorted;
			pts_y = pts_y_sorted;

			// Create spline
			tk::spline s;
			s.set_points(pts_x, pts_y);  

			for (int i = 0; i < previous_path_x.size(); ++i)
			{

				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// Start interpolating spline
			double target_x = 30;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
			double x_add_on = 0;
			double x_point, y_point;

			for (int i = 0; i <= 50 - previous_path_x.size(); ++i)
			{

//				double N = target_dist / (0.02 * GOAL[2] * 0.8f);
				double N = target_dist / (0.02 * ref_vel);
				x_point = x_add_on + (target_x) / N;
				y_point = s(x_point);
				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;


				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}



          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));

#ifdef _MSC_VER
			ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";

#ifdef _MSC_VER
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

#ifdef _MSC_VER
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef _MSC_VER
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	  char *message, size_t length) {
#endif

#ifdef _MSC_VER
	  ws->close();
#else
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;

#ifdef _MSC_VER
  string host = "127.0.0.1";
  if (h.listen(host.c_str(),port)) {
#else
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
