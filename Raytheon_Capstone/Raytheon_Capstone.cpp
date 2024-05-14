// Raytheon_Capstone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "Header.h"
#include <string>

using namespace cv;
using namespace std;

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



namespace Move {
	std::vector<int> MavSDKLocalMove(std::vector<int> local_dist);
	std::vector<int> MavSDKGlobalMove(std::vector<int> global_pos);
	std::vector<int> dist;
	std::vector<int> acceptable;
}

namespace Search {
	bool targetFound;
	std::vector<int> OpenCVSearch(std::vector<int> frame);
	std::vector<int> MavSDKMission(void);


}




namespace Drop {
	std::vector<int> MavSDKDescend(int height);
	bool WaterGunFire(void);
	int HeightToDrop;

}

namespace Reset {
	std::vector<int> MavSDKReturnHome(void);
	bool Reset(void);
}


int Thread2() { //second thread running OpenCV giving results to shared resource that main thread can only view

}



int main() {
	curr_state = init;
	bool reset_in; //will need to be an interrupt of some kind 
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

	bool result_gpio = Init::Gpio_init();
	bool result_picam = Init::picam_init();

	assert(result_gpio && result_picam); //break program if init fail








}