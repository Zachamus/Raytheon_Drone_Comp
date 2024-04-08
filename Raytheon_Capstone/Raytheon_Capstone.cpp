// Raytheon_Capstone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cassert>
#include <vector>


// ja sam idiot

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
	bool targetFound;
	std::vector<int> OpenCVSearch(std::vector<int> frame);
	std::vector<int> MavSDKMission(void);


}

namespace Init {
	bool Gpio_init(void);
	bool picam_init(void);
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