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
#include <unordered_set>
#include <shared_mutex>
#include "OpenCV.h"
#include "SearchAlgo.h"
#include <unordered_map>


using namespace cv;
using namespace std;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;
const vector<pair<double, double>> searchVec1 = { {0,7.5},{10,0},{10,0},{10,0},{10,0},{10,0},{0,7.5}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {0,7.5}, 
												{10,0}, {10,0}, {10,0}, {10,0}, {10,0}, };
int searchIndex{ 0 };
std::vector<int> markerInfo;
std::unordered_map<int, Vec3d> rmarkerInfo;
std::unordered_set<int> hitMarkers;
std::vector<int> markers;
std::mutex m; //shared mutex might be best here, lots of read operations and not a bunch of write ops
bool marker_found = false;
const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1420.40904, 0, 963.189814,
                                                    0, 1419.58763, 541.813445,
                                                    0, 0, 1);

const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.0147016237, 0.419899926, 0.000778167404, -0.000997227127, -0.844393910);



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



int Thread2() { //second thread running OpenCV giving results to shared resource that main thread can only view
    //CommandLineParser parser(argc, argv, keys);
    //parser.about(about);
    bool showRejected = false;
    bool estimatePose = true;
    float markerLength = 1;
    lccv::PiCamera cam;
    cam.options->video_width=1920;
    cam.options->video_height=1080;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cv::namedWindow("Video",cv::WINDOW_NORMAL);
    cam.startVideo();
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    cv::Mat frame;

    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    while (true) {
        if(!cam.getVideoFrame(frame,1000)){
            std::cout<<"Timeout error"<<std::endl;
        }
        else {
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            detector.detectMarkers(frame, markerCorners, markerIds);

            size_t nMarkers = markerCorners.size();
            vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

            if (estimatePose && !markerIds.empty()) {
                // Calculate pose for each marker
                std::cout << "We have detected a marker" << std::endl;
                for (size_t i = 0; i < nMarkers; i++) {
                    std::cout << markerIds.at(i) << std::endl;
                    if (markerIds.at(i) != 23) {
                        solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                        m.lock(); //take mutex, need to write to hash map
                        rmarkerInfo[markerIds.at(i)] = tvecs.at(i);
                        m.unlock(); //release, might be a better way to write after getting all tvecs and rvecs but shouldnt matter much
                    }
                }
            }
        }

        char key = (char)waitKey(1);
    }
    //! [aruco_detect_markers]
    std::cout << "Failed to grarb image" << std::endl;
    cam.stopVideo();
    cv::destroyWindow("Video");
    return 0;
    //outputVideo.release();
}







int main(int argc, char* argv[]) {
    
    curr_state = searching;
    std::cout.precision(15);
    std::thread t1(Thread2);
    
    //sleep_for(10s);
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

    Action::Result set_altitude = action.set_takeoff_altitude(12.0);
    Action::Result set_speed = action.set_maximum_speed(1.3);

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
    std::vector<double> vecNED;
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
                std::cout << "Going to index " << searchIndex << std::endl;
                gps_res = action.goto_location(out[searchIndex].first, out[searchIndex].second, global_alt, 0.0);
                if (gps_res != Action::Result::Success) {
                    std::cout << "Fucked" << std::endl;
                    curr_state = reset;
                    break;
                }
                


                //need to take mutex right before we check the markerInfo vector
                while ((abs(telemetry.position().latitude_deg - out[searchIndex].first) > 0.00001) ||
                       (abs(telemetry.position().longitude_deg - out[searchIndex].second) > 0.00001)) {
                    m.lock();
		    //std::cout << "We have taken the mutex" << std::endl;
                    if (((rmarkerInfo.find(22) != rmarkerInfo.end()) && (hitMarkers.find(22) == hitMarkers.end())) || ((rmarkerInfo.find(24) != rmarkerInfo.end()) && (hitMarkers.find(24) == hitMarkers.end()))) {
                        Action::Result hold_res = action.hold();
                        if (hold_res != Action::Result::Success) {
                            curr_state = reset;
                            break;
                        }
                        moveVec  = markerScan();
			std::cout << "We are running markerScan" << std::endl;
                        m.unlock();
                        break;
                    } else {
			m.unlock();
                      }
                      
                    // now release the mutex
                     //not sure if this is needed I think it isnt
                    //std::cout << "Drone not at pos yet, we are at: " << telemetry.position().latitude_deg
                       //       << " latitude, and: " << telemetry.position().longitude_deg << " longitude" << std::endl;
                    //std::cout << "We should be at: " << out[searchIndex].first << ", " << out[searchIndex].second
                      //        << std::endl;
                }
                if (marker_found) {
                    marker_found = false;
                    break; //break out of current state and go into either reset or moving depending on action result
                }
                searchIndex++;
                std::cout << "We have reached the target location! Checking for markers and then sleeping!"
                          << std::endl;
                sleep_for(3s);
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
                //qNED = telemetry.attitude_quaternion();
                vecNED.push_back(moveVec.second[0]); // = moveVec.second[0] / 3.281;
		        vecNED.push_back(moveVec.second[1]); // = moveVec.second[1] / 3.281; //convertToNED(qNED, moveVec.second);
		        vecNED[0] = vecNED[0] * -1;
                for (int i = 0; i < vecNED.size(); i++) {
                    if (abs(vecNED[i]) > 10)
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
                movePos.north_m = vecNED[0]/3.281;
                movePos.east_m = vecNED[1]/3.281;
                movePos.down_m = 0;
                movePos.yaw_deg = 0;
                offboard_result = offboard.set_position_ned(movePos);

                if (offboard_result != Offboard::Result::Success) {
                    std::cout << "Offboard move failed " << std::endl;
                    curr_state = reset;
                    break;
                }

                sleep_for(3s);

                offboard_result = offboard.stop();
                if (offboard_result != Offboard::Result::Success) {
                    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
                    curr_state = reset;
                    break;
                }
                curr_state = searching;
                break;



            case reset:
                const auto stop_result = action.hold();
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
