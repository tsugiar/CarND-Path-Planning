#pragma once
#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "helper_function.h"

using namespace std;

class Vehicle {
public:

	int L = 1;
	int lane;

	float s;
	float d;
	float lwidth = 4.0;	 // Default value for lane-width


	float v;

	float target_speed;

	int lanes_available;

	float max_acceleration;

	float goal_d;

	float goal_s;

	string state;

	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int lane, float s, float d, float v);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	void Vehicle::choose_next_stateVer2(map<int, Vehicle> &vehicle_map);

	bool get_vehicle_behind(map<int, Vehicle> &vehicle_map, int lane, Vehicle & rVehicle);

	bool get_vehicle_ahead(map<int, Vehicle> &vehicle_map, int lane, Vehicle & rVehicle);

	void configure(vector<float> road_data);

	bool OkayToLaneChange(map<int, Vehicle> &vehicle_map, int lane);

};

#endif