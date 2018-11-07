#pragma once
#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle, const map<int, Vehicle> & vehicle_map, const vector<Vehicle> & trajectory);

float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, Vehicle> & vehicle_map, map<string, float> & data);

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, Vehicle> & vehicle_map, map<string, float> & data);

float lane_speed(const map<int, Vehicle> & vehicle_map, int lane);

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory);

#endif