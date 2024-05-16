// Raytheon_Capstone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "Header.h"
#include <string>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <mavsdk/geometry.h>


mavsdk::geometry::CoordinateTransformation::LocalCoordinate cv_to_mav(std::vector<float>, double heading) {
	// convert vector to Local Coordinate struct
	//takes x y z vector and compass heading
}


using namespace cv;
using namespace std;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

enum states {
	init,
	search,
	move,
	drop,
	reset
}curr_state;

namespace {
	const char* about = "Basic marker detection";

	//! [aruco_detect_markers_keys]
	const char* keys =
		"{d        | 0     | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
		"DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
		"{cd       |       | Input file with custom dictionary }"
		"{v        |       | Input from video or image file, if ommited, input comes from camera }"
		"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
		"{c        |       | Camera intrinsic parameters. Needed for camera pose }"
		"{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
		"{dp       |       | File of marker detector parameters }"
		"{r        |       | show rejected candidates too }"
		"{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
		"CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";

	//! [aruco_detect_markers_keys]

	const string refineMethods[4] = {
		"None",
		"Subpixel",
		"Contour",
		"AprilTag"
	};

}


int Thread2() { //second thread running OpenCV giving results to shared resource that main thread can only view

}



int main() {
	curr_state = init;

	mavsdk::Mavsdk::ComponentType component_type = mavsdk::Mavsdk::ComponentType::CompanionComputer;
	mavsdk::Mavsdk::Configuration config = mavsdk::Mavsdk::Configuration::Configuration(component_type);
	mavsdk::Mavsdk mavsdk(config);
	mavsdk::ConnectionResult conn_result = mavsdk.add_serial_connection("serial:///dev/ttyAMA0", 57600);

	if (conn_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << conn_result << '\n';
		return 1;
	}

	while (mavsdk.systems().size() == 0) {
		std::cout << "Waiting for system to connect..." << std::endl;
		std::this_thread::sleep_for(1s);
	}
	
	
	std::shared_ptr<mavsdk::System> system = mavsdk.systems().at(0);
	auto offboard = mavsdk::Offboard{ system };
	auto action = mavsdk::Action{ system };
	auto telemetry = mavsdk::Telemetry{ system };

	

	while (telemetry.health_all_ok() == false) {
		std::cout << "Telemetry not healthy";
		std::this_thread::sleep_for(1s);
	}
	std::cout << "System is ready";

	Action::Result set_altitude = action.set_takeoff_altitude(5.0);

	if (set_altitude != Action::Result::Success) {
		std::cerr << "Set altitude failed" << std::endl;
		return 1;
	}

	Action::Result arm_result = action.arm();

	if (arm_result != Action::Result::Success) {
		std::cerr << "Arm failed: " << arm_result << std::endl;
		return 1;
	}

	


	while (1) {
		switch (curr_state) {
		case init:
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else {
				bool gpio_init = Init::Gpio_init();
				bool picam_init = Init::picam_init();
				assert(picam_init && gpio_init); //breaks program here if fail values returned
				curr_state = search;
				break;
			}
		case search:

		{
			while (!Search::targetFound && !reset_in) { //need to make OpenCV class and have a pointer in order to do some read operations on other threads data, might need mutex 
				Search::MavSDKMission();
			}
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else if (Search::targetFound) {
				curr_state = move;
				break;
			}


		}


		case move:
			if (reset_in) {
				curr_state = reset;
				break;
			}

			else {
				if (Move::dist <= Move::acceptable) {
					curr_state = drop;
					break;

				}
				Move::MavSDKLocalMove(Move::dist);
				curr_state = move;
				break;

			}



		case drop:
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else {
				Drop::MavSDKDescend(Drop::HeightToDrop);
				bool drop_success = Drop::WaterGunFire();
				if (drop_success)
					curr_state = search;
				curr_state = drop;
				break;
			}



		case reset:
			std::vector<int> pos = Reset::MavSDKReturnHome();
			bool reset_suc = Reset::Reset();

			if (reset_suc) {
				//figure out reset logic, probably need to reset whole thing
			}

			else {
				reset_suc = Reset::Reset(); //lol try again
			}



		}
	}
}
