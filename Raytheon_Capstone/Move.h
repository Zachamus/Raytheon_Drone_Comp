#pragma once

#include <mavsdk/plugins/offboard/offboard.h>





namespace Move {
	std::vector<int> MavSDKLocalMove(std::vector<int> local_dist);

	std::vector<int> MavSDKGlobalMove(std::vector<int> global_pos);

	std::vector<int> dist;

	std::vector<int> acceptable;
}