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
#include <unordered_set>
#include <shared_mutex>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace GeographicLib;
const vector<pair<double, double>> searchVec1 = { {0,7.5},{10,0},{10,0},{10,0},{10,0},{10,0},{0,7.5}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {0,7.5}, 
												{10,0}, {10,0}, {10,0}, {10,0}, {10,0}, };
int searchIndex{ 0 };
std::vector<int> markerInfo;
std::unordered_map<int, Vec3d> rmarkerInfo;
std::unordered_set<int> hitMarkers;
//std::mutex markerMutex;
std::vector<int> markers;
std::mutex m; //shared mutex might be best here, lots of read operations and not a bunch of write ops
bool marker_found = false;
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1420.40904, 0, 963.189814,
                                                    0, 1419.58763, 541.813445,
                                                    0, 0, 1);

cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.0147016237, 0.419899926, 0.000778167404, -0.000997227127, -0.844393910);



enum states {
    init,
    searching,
    moving,
    dropping,
    reset
}curr_state;



Telemetry::Quaternion mult(Telemetry::Quaternion a, Telemetry::Quaternion b) {
	Telemetry::Quaternion result;
	result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return result;
}

std::pair<int, Vec3d> markerScan() {
    double lowest = 50;
    std::pair<int, Vec3d> moveVec;

    for (auto [key, val]: rmarkerInfo) {
        if (hitMarkers.find(key) == hitMarkers.end()) {
            if (norm(val) < lowest) {
                lowest = norm(val);
                curr_state = moving;//marker is found and not in the hash set of markers that we've deployed water on
                moveVec.second = val;
                moveVec.first = key;

            }
        }

    }
    return moveVec;
}



std::vector<float> convertToNED(Telemetry::Quaternion q, Vec3d localVec) { //convert to NED frame
	Telemetry::Quaternion localquat;
	localquat.w = 0;					//create dummy quaternion for local vec
	localquat.x = localVec[0];
	localquat.y = localVec[1];
	localquat.z = localVec[2];			//now need to create inverse of NEDquat, 
	Telemetry::Quaternion inverseNed;
	float inverseNedSqrd = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	inverseNed.w = q.w / inverseNedSqrd;
	inverseNed.x = -q.x / inverseNedSqrd;
	inverseNed.y = -q.y / inverseNedSqrd;
	inverseNed.z = -q.z / inverseNedSqrd;

    Telemetry::Quaternion qVec = mult(q, localquat);
    Telemetry::Quaternion finalVec = mult(qVec, inverseNed);
    std::vector<float> returnVec = {finalVec.x, finalVec.y, finalVec.z};
    return returnVec;

	//now perform q * localquat * q^-1 result will also be a dummy quat, the imaginary part will be the vector we want
}



/*mavsdk::geometry::CoordinateTransformation::LocalCoordinate cv_to_mav(std::vector<float>, double heading) {
	// convert vector to Local Coordinate struct
	//takes x y z vector and compass heading
}
*/
//double longitude, double latitude, double altitude

std::vector<std::pair<double,double>> SearchAlgo(double lat1, double long1, double lat2, double long2, double lat3, double long3, double lat4, double long4, double altitude,
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
		double up = gpsVec.first/ ((double)1.094);
		double right = gpsVec.second / ((double)1.094);
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
    cv::VideoCapture inputVideo(0);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    //CommandLineParser parser(argc, argv, keys);
    //parser.about(about);
    bool showRejected = false;
    bool estimatePose = true;
    float markerLength = 1;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    if (!inputVideo.isOpened())
    {
        std::cout << "Problem connecting to cam " << std::endl;
        return -1;
    }
    else
    if (true)
    {
        std::cout << "Successfuly connected to camera " << std::endl;

        inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        int ex = (int)inputVideo.get(cv::CAP_PROP_FOURCC);
        char EXT[] = { (char)(ex & 0XFF),(char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24),0 };
        std::cout << "CAP_PROP_FOURCC: " << EXT << std::endl;

        inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        inputVideo.set(cv::CAP_PROP_FPS, 60);

    }

    int frameCounter = 0;
    int tick = 0;
    int fps;
    std::time_t timeBegin = std::time(0);

    cv::Mat frame;
    double totalTime = 0;
    int totalIterations = 0;

    //! [aruco_pose_estimation2]
    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    //! [aruco_pose_estimation2]

    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        //! [aruco_pose_estimation3]
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;

        // detect markers and estimate pose
        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (estimatePose && !ids.empty()) {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++) {
                std::cout << ids.at(i) << std::endl;
                if (ids.at(i) != 23) {
                    solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                    m.lock(); //take mutex, need to write to hash map
                    rmarkerInfo[ids.at(i)] = tvecs.at(i);
                    m.unlock(); //release, might be a better way to write after getting all tvecs and rvecs but shouldnt matter much
                }
            }
        }
        //! [aruco_pose_estimation3]
        //double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        //totalTime += currentTime;
        //totalIterations++;
        /*if (totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }*/
        //! [aruco_draw_pose_estimation]
        // draw results
        // image.copyTo(imageCopy);
//        if (!ids.empty()) {
//            cv::aruco::drawDetectedMarkers(image, corners, ids);
//
//        }
        //! [aruco_draw_pose_estimation]

        //if (showRejected && !rejected.empty())
            //cv::aruco::drawDetectedMarkers(image, rejected, noArray(), Scalar(100, 0, 255));

        //imshow("out", image);
        char key = (char)waitKey(1);
    }
    //! [aruco_detect_markers]
    return 0;
}




int main(int argc, char* argv[]) {

    curr_state = searching;
    std::cout.precision(15);
    //std::thread t1(Thread2);
    sleep_for(10s);
    //calculate Search gps coordinates
    std::vector<pair<double, double>> out = SearchAlgo(34.4193286, -119.8555100, 34.4193164, -119.8559169, 34.4193286,
                                                       -119.8555100, 34.419237, -119.856216, 30.0, searchVec1);
    for (const auto &joe: out) {
        std::cout << joe.first << " " << joe.second << std::endl; //print gps coords
    }
    


    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    mavsdk::ConnectionResult conn_result = mavsdk.add_any_connection("serial:///dev/ttyAMA0:57600");

    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << conn_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    auto offboard = mavsdk::Offboard{system.value()};
    auto action = mavsdk::Action{system.value()};
    auto telemetry = mavsdk::Telemetry{system.value()};


    while (telemetry.health_all_ok() == false) {
        std::cout << "Telemetry not healthy" << std::endl;
        std::this_thread::sleep_for(0.5s);
    }
    std::cout << "System is ready";

    Action::Result set_altitude = action.set_takeoff_altitude(8.0);
    Action::Result set_speed = action.set_maximum_speed(2.0);

    if (set_speed != Action::Result::Success) {
        std::cerr << "Set Speed Failed" << std::endl;
        return 1;
    }

    if (set_altitude != Action::Result::Success) {
        std::cerr << "Set altitude failed" << std::endl;
        return 1;
    }
    while (telemetry.flight_mode() != Telemetry::FlightMode::Posctl) {
        sleep_for(1s);
    }

    Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arm failed: " << arm_result << std::endl;
        return 1;
    }
    //Telemetry::FlightMode curr_flight_mode;

    Action::Result takeoff_result = action.takeoff();

    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return 1;
    }
    sleep_for(10s);
    Telemetry::Altitude curr_alt = telemetry.altitude();
    double global_alt = curr_alt.altitude_amsl_m;
    Telemetry::ActuatorOutputStatus joe = telemetry.actuator_output_status();
    std::pair<int, Vec3d> moveVec;
    Action::Result gps_res;
    Telemetry::Quaternion qNED;
    std::vector<float> vecNED;
    bool too_far;
    geometry::CoordinateTransformation::LocalCoordinate localCoord;
    geometry::CoordinateTransformation::GlobalCoordinate globalCoord;
    Offboard::Result offboard_result;
    const Offboard::VelocityNedYaw stay{};
    Offboard::PositionNedYaw movePos{};


    while (1) {
        switch (curr_state) {
            case searching:
                if (telemetry.flight_mode() == Telemetry::FlightMode::Altctl) {
                    action.hold();
                    curr_state = reset;
                    break;
                }
                if (searchIndex == out.size() - 1) {
                    curr_state = reset;
                    break;
                }
                gps_res = action.goto_location(out[searchIndex].first, out[searchIndex].second, global_alt, 0.0);
                if (gps_res != Action::Result::Success) {
                    std::cout << "Fucked" << std::endl;
                    curr_state = reset;
                    break;
                }
                


                //need to take mutex right before we check the markerInfo vector
                while ((abs(telemetry.position().latitude_deg - out[searchIndex].first) > 0.00001) ||
                       (abs(telemetry.position().longitude_deg - out[searchIndex].second) > 0.00001)) {
                    /*m.lock();
                    if (((rmarkerInfo.find(22) != rmarkerInfo.end()) && (hitMarkers.find(22) == hitMarkers.end())) || ((rmarkerInfo.find(24) != rmarkerInfo.end()) && (hitMarkers.find(24) == hitMarkers.end()))) {
                        Action::Result hold_res = action.hold();
                        if (hold_res != Action::Result::Success) {
                            curr_state = reset;
                            break;
                        }
                        moveVec  = markerScan();
                    } else
                        m.unlock();
                        */
                    // now release the mutex
                     //not sure if this is needed I think it isnt
                    std::cout << "Drone not at pos yet, we are at: " << telemetry.position().latitude_deg
                              << " latitude, and: " << telemetry.position().longitude_deg << " longitude" << std::endl;
                    std::cout << "We should be at: " << out[searchIndex].first << ", " << out[searchIndex].second
                              << std::endl;
                }
                //if (marker_found) {
                   // marker_found = false;
                    //break; //break out of current state and go into either reset or moving depending on action result
                //}
                searchIndex++;
                std::cout << "We have reached the target location! Checking for markers and then sleeping!"
                          << std::endl;
                m.lock();
                if (((rmarkerInfo.find(22) != rmarkerInfo.end()) && (hitMarkers.find(22) == hitMarkers.end())) || ((rmarkerInfo.find(24) != rmarkerInfo.end()) && (hitMarkers.find(24) == hitMarkers.end()))) {
                    Action::Result hold_res = action.hold();
                    if (hold_res != Action::Result::Success) {
                        curr_state = reset;
                        break;
                    }
                    moveVec = markerScan();
                    marker_found = false;
                }
                m.unlock();
                
                
                sleep_for(1s);
                break;

            case moving:
                //if marker is not already hit, and marker is not ours, move to closest marker
                std::cout << "We are in moving state: " << std::endl;
                if (telemetry.flight_mode() == Telemetry::FlightMode::Altctl) {
                    action.hold();
                    curr_state = reset;
                    break;
                }
                qNED = telemetry.attitude_quaternion();
                vecNED = convertToNED(qNED, moveVec.second);
                for (int i = 0; i < vecNED.size(); i++) {
                    if (vecNED[i] > 30)
                        too_far = true;
                }
                if (too_far) {
                    curr_state = reset;
                    std::cout << "NED vec is too far you fucking RETARD " << std::endl;
                    break;
                }
                std::cout << "Starting Offboard velocity control in NED coordinates\n";

                // Send it once before starting offboard, otherwise it will be rejected.

                offboard.set_velocity_ned(stay);
                offboard_result = offboard.start();
                if (offboard_result != Offboard::Result::Success) {
                    std::cerr << "Offboard start failed: " << offboard_result << '\n';
                    curr_state = reset;
                    break;
                }
                movePos.north_m = vecNED[0];
                movePos.east_m = vecNED[1];
                movePos.down_m = 0;
                movePos.yaw_deg = 0;
                offboard_result = offboard.set_position_ned(movePos);

                if (offboard_result != Offboard::Result::Success) {
                    std::cout << "Offboard move failed " << std::endl;
                    curr_state = reset;
                    break;
                }

                sleep_for(5s);

                offboard_result = offboard.stop();
                if (offboard_result != Offboard::Result::Success) {
                    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
                    curr_state = reset;
                    break;
                }
                curr_state = searching;
                break;



            case reset:
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

        }


    }


}