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
#include <GeographicLib/Geodesic.hpp>


//using namespace cv;
using namespace std;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace GeographicLib;
const vector<pair<double, double>> searchVec1 = { {0,7.5},{10,0},{10,0},{10,0},{10,0},{10,0},{0,7.5}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {0,7.5}, 
												{10,0}, {10,0}, {10,0}, {10,0}, {10,0}, };

/*mavsdk::geometry::CoordinateTransformation::LocalCoordinate cv_to_mav(std::vector<float>, double heading) {
	// convert vector to Local Coordinate struct
	//takes x y z vector and compass heading
}
*/
//double longitude, double latitude, double altitude

std::vector<std::pair<double,double>> SearchAlgo(double long1, double long2, double long3, double long4, double lat1, double lat2, double lat3, double lat4, double altitude,
												const vector<pair<double,double>> localSearch) {
	Geodesic geod = Geodesic::WGS84();
	vector<pair<double, double>> gpsCoords;
	double distance = 50;
	double new_lat, new_long;
	//obtain verticle angle and horizontal angle, ie need to geod.Inverse bottom left and top left, and bottom left and top right;
	double dist_bottom, az12, az21;
	geod.Inverse(lat1, long1, lat2, long2, dist_bottom, az12, az21); //obtain horizontal angle and distance
	double angle_horizontal = az12;
	double dist_top, az13, az31;
	geod.Inverse(lat1, long1, lat3, long3, dist_top, az13, az31);//obtain vertical angle and distance
	double currlong, currlat;
	double angle_vertical = az13; //now from
	geod.Direct(lat1, long1, az12, 6.858, currlat, currlong);

	gpsCoords.push_back({ currlat, currlong });//append starting point, this is 7.5 yards to the right of bottom left corner.
	double nextlong, nextlat;

	for (auto gpsVec : localSearch) {
		double up = gpsVec.first;
		double right = gpsVec.second;
		if (up) {
			geod.Direct(currlat, currlong, az13, up, nextlat, nextlong);
		}
		else if (right) {
			geod.Direct(currlat, currlong, az12, right, nextlat, nextlong);
		}
		gpsCoords.push_back({ nextlat, nextlong });
		currlat = nextlat;
		currlong = nextlong;

	}
	return gpsCoords;

}



bool offb_ctrl_body(mavsdk::Offboard& offboard)
{
	std::cout << "Starting Offboard velocity control in body coordinates\n";

	// Send it once before starting offboard, otherwise it will be rejected.
	Offboard::VelocityBodyYawspeed stay{};
	offboard.set_velocity_body(stay);

	Offboard::Result offboard_result = offboard.start();
	if (offboard_result != Offboard::Result::Success) {
		std::cerr << "Offboard start failed: " << offboard_result << '\n';
		return false;
	}
	std::cout << "Offboard started\n";

	std::cout << "Turn clock-wise and climb\n";
	Offboard::VelocityBodyYawspeed cc_and_climb{};
	cc_and_climb.down_m_s = -1.0f;
	cc_and_climb.yawspeed_deg_s = 60.0f;
	offboard.set_velocity_body(cc_and_climb);
	sleep_for(seconds(5));

	std::cout << "Turn back anti-clockwise\n";
	Offboard::VelocityBodyYawspeed ccw{};
	ccw.down_m_s = -1.0f;
	ccw.yawspeed_deg_s = -60.0f;
	offboard.set_velocity_body(ccw);
	sleep_for(seconds(5));

	std::cout << "Wait for a bit\n";
	offboard.set_velocity_body(stay);
	sleep_for(seconds(2));

	std::cout << "Fly a circle\n";
	Offboard::VelocityBodyYawspeed circle{};
	circle.forward_m_s = 5.0f;
	circle.yawspeed_deg_s = 30.0f;
	offboard.set_velocity_body(circle);
	sleep_for(seconds(15));

	std::cout << "Wait for a bit\n";
	offboard.set_velocity_body(stay);
	sleep_for(seconds(5));

	std::cout << "Fly a circle sideways\n";
	circle.right_m_s = -5.0f;
	circle.yawspeed_deg_s = 30.0f;
	offboard.set_velocity_body(circle);
	sleep_for(seconds(15));

	std::cout << "Wait for a bit\n";
	offboard.set_velocity_body(stay);
	sleep_for(seconds(8));

	offboard_result = offboard.stop();
	if (offboard_result != Offboard::Result::Success) {
		std::cerr << "Offboard stop failed: " << offboard_result << '\n';
		return false;
	}
	std::cout << "Offboard stopped\n";

	return true;
}






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
	return 0;
}




int main(int argc, char* argv[]) {
	curr_state = init;
	std::cout.precision(15);
	SearchAlgo();
	Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
	mavsdk::ConnectionResult conn_result = mavsdk.add_any_connection("serial:///dev/ttyAMA0:57600");

	if (conn_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << conn_result << '\n';
		return 1;
	}

	auto system = mavsdk.first_autopilot(3.0);
	auto offboard = mavsdk::Offboard{ system.value() };
	auto action = mavsdk::Action{ system.value() };
	auto telemetry = mavsdk::Telemetry{ system.value() };
	




	while (telemetry.health_all_ok() == false) {
		std::cout << "Telemetry not healthy" << std::endl;
		std::this_thread::sleep_for(0.5s);
	}
	std::cout << "System is ready";

	Action::Result set_altitude = action.set_takeoff_altitude(5.0);
	Action::Result set_speed = action.set_maximum_speed(2.0);

	if (set_speed != Action::Result::Success) {
		std::cerr << "Set Speed Failed" << std::endl;
		return 1;
	}

	if (set_altitude != Action::Result::Success) {
		std::cerr << "Set altitude failed" << std::endl;
		return 1;
	}

	Action::Result arm_result = action.arm();

	if (arm_result != Action::Result::Success) {
		std::cerr << "Arm failed: " << arm_result << std::endl;
		return 1;
	}
	//Telemetry::FlightMode curr_flight_mode;

	while (telemetry.flight_mode() != Telemetry::FlightMode::Posctl) {
		sleep_for(1s);
	}

	Action::Result takeoff_result = action.takeoff();

	if (takeoff_result != Action::Result::Success) {
		std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
		return 1;
	}
	

	if (!offb_ctrl_body(offboard)) {
		return 1;
	}

	const auto land_result = action.land();
	if (land_result != Action::Result::Success) {
		std::cerr << "Landing failed: " << land_result << std::endl;
		return 1;
	}
	while (telemetry.in_air()) {
		std::cout << "Currently landing";
		sleep_for(1s);
	}

	std::cout << "Landed";

	sleep_for(seconds(3));

	return 0;



	/*
	while (1) {
		switch (curr_state) {
		case init:
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else {
				bool gpio_init = init::gpio_init();
				bool picam_init = init::picam_init();
				assert(picam_init && gpio_init); //breaks program here if fail values returned
				curr_state = search;
				break;
			}
		case search:

		{
			while (!search::targetfound && !reset_in) { //need to make opencv class and have a pointer in order to do some read operations on other threads data, might need mutex 
				search::mavsdkmission();
			}
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else if (search::targetfound) {
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
				if (move::dist <= move::acceptable) {
					curr_state = drop;
					break;

				}
				move::mavsdklocalmove(move::dist);
				curr_state = move;
				break;

			}



		case drop:
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else {
				drop::mavsdkdescend(drop::heighttodrop);
				bool drop_success = drop::watergunfire();
				if (drop_success)
					curr_state = search;
				curr_state = drop;
				break;
			}



		case reset:
			std::vector<int> pos = reset::mavsdkreturnhome();
			bool reset_suc = reset::reset();

			if (reset_suc) {
				figure out reset logic, probably need to reset whole thing
			}

			else {
				reset_suc = reset::reset(); //lol try again
			}



		}
	}
	*/
}
