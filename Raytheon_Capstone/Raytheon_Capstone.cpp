// Raytheon_Capstone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Windows.h>
#include <cassert>
#include <vector>


enum states {
	init,
	search,
	move,
	drop,
	reset
}curr_state;

namespace Move {
	std::vector<int> MavSDKLocalMove(std::vector<int> local_dist);
	std::vector<int> MavSDKGlobalMove(std::vector<int> global_pos);
	std::vector<int> dist;
	std::vector<int> acceptable;
}

namespace Search {
	BOOL targetFound;
	std::vector<int> OpenCVSearch(std::vector<int> frame);
	std::vector<int> MavSDKMission(void);


}

namespace Init {
	BOOL Gpio_init(void);
	BOOL picam_init(void);
}

namespace Drop {
	std::vector<int> MavSDKDescend(int height);
	BOOL WaterGunFire(void);
	int HeightToDrop;

}

namespace Reset {
	std::vector<int> MavSDKReturnHome(void);
	BOOL Reset(void);
}


int Thread2() { //second thread running OpenCV giving results to shared resource that main thread can only view

}



int main() {
	curr_state = init;
	BOOL reset_in; //will need to be an interrupt of some kind 
	while (1) {
		switch (curr_state) {
		case init:
			if (reset_in) {
				curr_state = reset;
				break;
			}
			else {
				BOOL gpio_init = Init::Gpio_init();
				BOOL picam_init = Init::picam_init();
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
				BOOL drop_success = Drop::WaterGunFire();
				if (drop_success)
					curr_state = search;
				curr_state = drop;
				break;
			}



		case reset:
			std::vector<int> pos = Reset::MavSDKReturnHome();
			BOOL reset_suc = Reset::Reset();

			if (reset_suc) {
				//figure out reset logic, probably need to reset whole thing
			}

			else {
				reset_suc = Reset::Reset(); //lol try again
			}



		}
	}

	BOOL result_gpio = Init::Gpio_init();
	BOOL result_picam = Init::picam_init();

	assert(result_gpio && result_picam); //break program if init fail








}