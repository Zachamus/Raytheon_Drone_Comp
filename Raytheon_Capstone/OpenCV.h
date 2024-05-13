#pragma once
#include <Windows.h> //swap this out lol obv not using windows API 
#include <vector>



class OpenCV
{
	public:
		OpenCV() {
			// create camera obj,
		};

		void startFeed();

		void stopFeed();

		std::vector<double> distances;
		bool targetFound; //if targetFound && targetID != 23 || droppedAlready
		int targetID;

};

