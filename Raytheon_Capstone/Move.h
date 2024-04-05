#pragma once

#include <mavsdk/mavlink_address.h>
#include <mavsdk/mavlink/common/mavlink.h>
#include <vector>



namespace Move {
	std::vector<int> MavSDKLocalMove(std::vector<int> local_dist);

	std::vector<int> MavSDKGlobalMove(std::vector<int> global_pos);

	std::vector<int> dist;

	std::vector<int> acceptable;
}