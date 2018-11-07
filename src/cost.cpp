#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);


float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, Vehicle> & vehicle_map, map<string, float> & data) {
	/*
	Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
	Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
	*/
	float cost;
	float distance = data["distance_to_goal"];
	if (distance > 0) {
		cost = 1 - 2 * exp(-(abs(2.0*vehicle.goal_d - data["intended_d"] - data["final_d"]) / distance));
	}
	else {
		cost = 1;
	}
	return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, Vehicle> & vehicle_map, map<string, float> & data) {
	/*
	Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
	*/

	float proposed_speed_intended = lane_speed(vehicle_map, data["intended_lane"]);
	if (proposed_speed_intended < 0) {
		proposed_speed_intended = vehicle.target_speed;
	}

	float proposed_speed_final = lane_speed(vehicle_map, data["final_lane"]);
	if (proposed_speed_final < 0) {
		proposed_speed_final = vehicle.target_speed;
	}

	float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;

	return cost;
}

float lane_speed(const map<int, Vehicle> & vehicle_map, int lane) {
	/*
	All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
	we can just find one vehicle in that lane.
	*/
	for (map<int, Vehicle>::const_iterator it = vehicle_map.begin(); it != vehicle_map.end(); ++it) {
		int key = it->first;
		Vehicle vehicle = it->second;
		if (vehicle.lane == lane && key != -1) {
			return vehicle.sdot;
		}
	}
	//Found no vehicle in the lane
	return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, Vehicle> & vehicle_map, const vector<Vehicle> & trajectory) {
	/*
	Sum weighted cost functions to get total cost for trajectory.
	*/
	map<string, float> trajectory_data = get_helper_data(vehicle, trajectory);
	float cost = 0.0;

	//Add additional cost functions here.
	vector< function<float(const Vehicle &, const vector<Vehicle> &, const map<int, Vehicle> &, map<string, float> &)>> cf_list = { goal_distance_cost, inefficiency_cost };
	vector<float> weight_list = { REACH_GOAL, EFFICIENCY };

	for (int i = 0; i < cf_list.size(); i++) {
		float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, vehicle_map, trajectory_data);
		cost += new_cost;
	}

	return cost;

}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory) {
	/*
	Generate helper data to use in cost functions:
	indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
	final_lane: the lane of the vehicle at the end of the trajectory.
	distance_to_goal: the distance of the vehicle to the goal.

	Note that indended_lane and final_lane are both included to help differentiate between planning and executing
	a lane change in the cost functions.
	*/
	map<string, float> trajectory_data;
	Vehicle trajectory_last = trajectory[1];
	float intended_lane;
	float intended_d;
	float final_d;

	if (trajectory_last.state.compare("PLCL") == 0) {
		intended_lane = trajectory_last.lane - 1;
	}
	else if (trajectory_last.state.compare("PLCR") == 0) {
		intended_lane = trajectory_last.lane + 1;
	}
	else {
		intended_lane = trajectory_last.lane;
	}

	float distance_to_goal = vehicle.goal_s - trajectory_last.s;
	float final_lane = trajectory_last.lane;

	intended_d = trajectory_last.lwidth * (-0.5 + (float)intended_lane);
	final_d    = trajectory_last.lwidth * (-0.5 + (float) final_lane);

	trajectory_data["intended_lane"] = intended_lane;
	trajectory_data["final_lane"] = final_lane;
	trajectory_data["distance_to_goal"] = distance_to_goal;
	trajectory_data["intended_d"] = intended_d;
	trajectory_data["final_d"] = final_d;


	return trajectory_data;
}

