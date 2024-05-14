#pragma once

#include <mavsdk/plugins/offboard/offboard.h>
#include <vector>



namespace Move {
	std::vector<int> MavSDKLocalMove(std::vector<int> local_dist);

	std::vector<int> MavSDKGlobalMove(std::vector<int> global_pos);

}