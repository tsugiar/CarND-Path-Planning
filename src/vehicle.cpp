#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

/**
* Initializes Vehicle
*/

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float d, float v) {

	this->lane = lane;
	this->s = s;
	this->d = d;

	this->v = v;

	max_acceleration = -1;

}

Vehicle::~Vehicle() {}


bool Vehicle::OkayToLaneChange(map<int, Vehicle> &vehicle_map, int laneside)
{
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;
	double TTC;

	// Make sure there is no vehicle in-front within 30 m
	if (this->get_vehicle_ahead(vehicle_map, laneside, vehicle_ahead))
	{

		if (vehicle_ahead.s - this->s < 30.0  )
		{
			return false;

		}
		else
		{   // If vehicle ahead isn't within 30 m, check vehicle behind on other lane
			// Make sure it isn't less than 30 m
			if (this->get_vehicle_behind(vehicle_map, laneside, vehicle_behind))
			{
				TTC = (this->s - vehicle_behind.s) / (vehicle_behind.v - this->v);
				TTC = TTC < 0.0 ? 20 : TTC;

				if ((this->s - vehicle_behind.s < 30.0 && TTC < 2.0) || this->s - vehicle_behind.s < 10.0 )
				{

					return false;

				}
			}

			return true;   // Only vehicle in-front but it is greater than 30 m, vehicle behind doesn't pose a threat

		}

	} 
	else
	{
		// there is no vehicle in-front, only need to check vehicle behind
		if (this->get_vehicle_behind(vehicle_map, laneside, vehicle_behind))
		{
			TTC = (this->s - vehicle_behind.s) / (vehicle_behind.v - this->v);
			TTC = TTC < 0.0 ? 20 : TTC;

			if ((this->s - vehicle_behind.s < 30.0 && TTC < 2.0) || this->s - vehicle_behind.s < 10.0 )
			{
				// Don't do lane-change when vehicle behind is too close
				return false;

			}
			
			// Otherwise, return true
			return true;

		}

		else
		{
			// No vehicle from behind within 30 m
			return true;

		}

	}


}


void Vehicle::choose_next_stateVer2(map<int, Vehicle> &vehicle_map) {
	
	if (this->lane == 1)
	{
		// If we are on lane-1, want to check if condition is appropriate
		// to make a move to lane 2 (middle lane)

		if (this->OkayToLaneChange(vehicle_map, 2))
		{

			this->lane = 2;

		}

	}
	else if (this->lane == 2)
	{
		Vehicle vehicle_ahead;
		
		// Check if speed drop below 85% of target, and there is a vehicle 
		// ahead of ego vehicle that causing congestion
		if (this->v < this->target_speed * 0.85)
		{ 

			if (this->get_vehicle_ahead(vehicle_map, this->lane, vehicle_ahead))
			{
				if (vehicle_ahead.s - this->s < 30.0)
				{
					// Congestion occur in current-lane, assess left-lane (1) first to 
					// to see if it is appropriate for lane-change, then follow by
					// right lane (3)
					if (this->OkayToLaneChange(vehicle_map, 1))
					{
						this->lane = 1;


					}
					else if (this->OkayToLaneChange(vehicle_map, 3))
					{

						this->lane = 3;

					}
					else
					{
						// dont' do antyhing

					}


				}
			}

		}

	// Otherwise don't do anyting

	} // End-condition for cases when ego-vehicle is in lane-2 (middle lane)
	else if (this->lane == 3)
	{

		if (this->OkayToLaneChange(vehicle_map, 2))
		{

			this->lane = 2;

		}

	}

}



bool Vehicle::get_vehicle_behind(map<int, Vehicle> &vehicle_map, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/
	int max_s = -1;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, Vehicle>::iterator it = vehicle_map.begin(); it != vehicle_map.end(); ++it) {
		temp_vehicle = it->second;
		if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
			max_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, Vehicle> &vehicle_map, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/
	int min_s = this->goal_s;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, Vehicle>::iterator it = vehicle_map.begin(); it != vehicle_map.end(); ++it) {
		temp_vehicle = it->second;
		if (temp_vehicle.lane == lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
			min_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}

	return found_vehicle;
}



void Vehicle::configure(vector<float> road_data) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/
	target_speed = road_data[0];
	lanes_available = road_data[1];
	goal_s = road_data[2];
	goal_d = road_data[3];
	max_acceleration = road_data[4];
}