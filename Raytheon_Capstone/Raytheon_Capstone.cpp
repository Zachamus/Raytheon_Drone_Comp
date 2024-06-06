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
#include <lccv.hpp>
#include <atomic>


using namespace cv;
using namespace std;
using namespace mavsdk;
using namespace GeographicLib;
using std::chrono::seconds;
using std::this_thread::sleep_for;
const vector<pair<double, double>> searchVec1 = { {0,7.5},{10,0},{10,0},{10,0},{10,0},{10,0},{0,7.5}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {-10,0}, {0,7.5},
                                                {10,0}, {10,0}, {10,0}, {10,0}, {10,0}, };
int searchIndex{ 0 };
std::vector<int> markerInfo;
std::unordered_map<int, std::pair<double, double>> rmarkerInfo;
std::unordered_map<int, Vec3d> tvecmarkerInfo;
std::unordered_set<int> hitMarkers;
std::vector<int> markers;
std::mutex m; //shared mutex might be best here, lots of read operations and not a bunch of write ops
std::mutex dva; //mutex for entire opencv thread to make sure no context switching while sending offboard
std::unordered_set<int> firstDet;

bool marker_found = false;
const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1420.40904, 0, 963.189814,
    0, 1419.58763, 541.813445,
    0, 0, 1);

const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.0147016237, 0.419899926, 0.000778167404, -0.000997227127, -0.844393910);
condition_variable cv_sync; //sync opencv 
condition_variable cv_tvec_small;
bool tvec_small;
std::atomic<bool> kill_switch{ false };

enum states {
    init,
    searching,
    moving,
    dropping,
    reset
}curr_state;

//get to center

//shoot in area around center

void shootArea(Offboard& offboard) {
    //move in square while shooting, call after starting GPIO, end GPIO after
    //this should probably block OpenCV thread
}



Telemetry::Quaternion mult(Telemetry::Quaternion a, Telemetry::Quaternion b) {
    Telemetry::Quaternion result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

std::pair<int, std::pair<double, double>> markerScan() {
    //double lowest = 50;
    std::pair<int, std::pair<double, double>> moveVec;

    for (const auto& [key, val] : rmarkerInfo) {
        if (hitMarkers.find(key) == hitMarkers.end()) {
            curr_state = moving;//marker is found and not in the hash set of markers that we've deployed water on
            moveVec.second = val;
            moveVec.first = key;


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
    std::vector<float> returnVec = { finalVec.x, finalVec.y, finalVec.z };
    return returnVec;

    //now perform q * localquat * q^-1 result will also be a dummy quat, the imaginary part will be the vector we want
}

int Thread2(Telemetry& telemetry) { //second thread running OpenCV giving results to shared resource that main thread can only view
    //CommandLineParser parser(argc, argv, keys);
    //parser.about(about);
    Geodesic geod = Geodesic::WGS84();
    bool showRejected = false;
    bool estimatePose = true;
    float markerLength = 1;
    bool long_sleep = false;
    bool b_firstDet = false;
    lccv::PiCamera cam;
    cam.options->video_width = 1920;
    cam.options->video_height = 1080;
    cam.options->framerate = 12;
    cam.options->verbose = true;
    cam.options->ev = 0.2;
    cv::namedWindow("Video", cv::WINDOW_NORMAL);
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
    Telemetry::RawGps rawgps;

    double lat1, long1, lat2, long2;
    while (true) {
        dva.lock();
        if (!cam.getVideoFrame(frame, 1000)) {
            std::cout << "Timeout error" << std::endl;
        }
        else {
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            //std::cout << "Scanning for markers" << std::endl;
            detector.detectMarkers(frame, markerCorners, markerIds);

            size_t nMarkers = markerCorners.size();
            vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);
            //aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            if (estimatePose && !markerIds.empty()) {
                // Calculate pose for each marker
                std::cout << "We have detected a marker" << std::endl;
                {
                    std::lock_guard<std::mutex> lock(m);
                    rawgps = telemetry.raw_gps();
                }


                for (size_t i = 0; i < nMarkers; i++) {
                    std::cout << markerIds.at(i) << std::endl;
                    {
                        std::lock_guard<std::mutex> lock(m);
                        if ((markerIds.at(i) != 23) && (hitMarkers.find(markerIds.at(i)) == hitMarkers.end())) {
                            solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                            geod.Direct(rawgps.latitude_deg, rawgps.longitude_deg, 0, tvecs.at(i)[0] / 3.281, lat1, long1);
                            geod.Direct(lat1, long1, 90, tvecs.at(i)[1] / 3.281, lat2, long2);
                            std::pair<double, double> ret = { lat2, long2 };
                            rmarkerInfo[markerIds.at(i)] = ret;
                            tvecmarkerInfo[markerIds.at(i)] = tvecs.at(i);
                            marker_found = true;
                            if ((abs(tvecs.at(i)[0]) < 0.3) && (abs(tvecs.at(i)[1]) < 0.3)) {
                                tvec_small = true;
                                cv_tvec_small.notify_one();
                                long_sleep = true;
                            }
                            else {
                                cv_sync.notify_one();

                            }
                        }
                        std::cout << "Sending notification" << std::endl;

                    }
                    if (long_sleep) {
                        sleep_for(5s);
                        long_sleep = false;
                    }
                    else {
                        sleep_for(0.5s);
                    }
                }
            }
        }
        //imshow("Video", frame);
        char key = (char)waitKey(1);
        dva.unlock();
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


    //sleep_for(10s);
    //calculate Search gps coordinates
    std::vector<pair<double, double>> out = SearchAlgo(34.4188681, -119.8556203, 34.4189528, -119.8556155, 34.4188648,
        -119.8557251, 34.419237, -119.856216, 30.0, searchVec1);
    for (const auto& joe : out) {
        std::cout << joe.first << " " << joe.second << std::endl; //print gps coords
    }



    Mavsdk mavsdk{ Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer} };
    mavsdk::ConnectionResult conn_result = mavsdk.add_any_connection("serial:///dev/ttyAMA0:57600");

    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << conn_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    auto offboard = mavsdk::Offboard{ system.value() };
    auto action = mavsdk::Action{ system.value() };
    auto telemetry = mavsdk::Telemetry{ system.value() };
    std::thread t1([&telemetry]() { Thread2(telemetry); });





    while (telemetry.health_all_ok() == false) {
        std::cout << "Telemetry not healthy" << std::endl;
        std::this_thread::sleep_for(0.5s);
    }
    std::cout << "System is ready";

    telemetry.subscribe_flight_mode([&action](Telemetry::FlightMode flight_mode) {
        if (flight_mode == Telemetry::FlightMode::Land) {
            std::cout << "Changed to land mode, resetting program " << std::endl;
            kill_switch.store(true);
        }
        });




    Action::Result set_altitude = action.set_takeoff_altitude(3.7);
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
    std::pair<int, std::pair<double, double>> moveVec;
    Action::Result gps_res;
    Telemetry::Quaternion qNED;
    std::vector<double> vecNED;
    bool too_far;
    geometry::CoordinateTransformation::LocalCoordinate localCoord;
    geometry::CoordinateTransformation::GlobalCoordinate globalCoord;
    Offboard::Result offboard_result;
    const Offboard::VelocityNedYaw stay{};
    Offboard::PositionNedYaw movePos{};
    Telemetry::RawGps curr_gps;
    std::pair<double, double> marker_location;
    Action::Result move_res;
    bool jadonisaretard = false;
    std::unique_lock<std::mutex> lock(m, std::defer_lock);
    bool signal_det = false;
    bool break_out = false;
    std::vector<float> ned_position{ 0.0, 0.0, 0.0 };
    Offboard::PositionNedYaw initial_ned_setpoint{ 0.0, 0.0, 0.0, 0.0 };
    Offboard::Result offboard_result;
    Vec3d localtvec;

    while (1) {

        switch (curr_state) {
        case searching:
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            if (searchIndex == out.size() - 1) {
                curr_state = reset;
                break;
            }
            lock.lock();
            if (cv_sync.wait_for(lock, std::chrono::seconds(12), [] { return marker_found; })) {
                // Proceed if the condition variable was notified and the condition is true
                if (kill_switch.load()) {
                    std::cout << "Kill switch recognized, killing program" << std::endl;
                    curr_state = reset;
                    break;
                }
                std::cout << "Marker found, moving to move state" << std::endl;
                curr_state = moving;
                signal_det = true;
                marker_found = false;
            }
            else {
                // Timeout occurred
                std::cout << "Timed out, continuing search\n";
            }
            lock.unlock();
            if (signal_det) {
                signal_det = false;
                break;
            }
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            std::cout << "Going to index " << searchIndex << std::endl;
            gps_res = action.goto_location(out[searchIndex].first, out[searchIndex].second, global_alt, 0.0);
            if (gps_res != Action::Result::Success) {
                std::cout << "Failed to move to next search index" << std::endl;
                curr_state = reset;
                break;
            }
            searchIndex++;
            break;
        case moving:
            action.hold();
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            //if marker is not already hit, and marker is not ours, move to closest marker
            std::cout << "We are in moving state: " << std::endl;
            if (telemetry.flight_mode() == Telemetry::FlightMode::Land) {
                action.hold();
                curr_state = reset;
                break;
            }
            lock.lock();
            moveVec = markerScan();
            lock.unlock();

            gps_res = action.goto_location(moveVec.second.first, moveVec.second.second, global_alt, 0.0);
            if (gps_res != Action::Result::Success) {
                std::cout << "Failed to move to next search index" << std::endl;
                curr_state = reset;
                break;
            }
            sleep_for(3s);
            std::cout << "Now entering main control loop" << std::endl;
            offboard.set_position_ned(initial_ned_setpoint);
            offboard_result = offboard.start();
            if (offboard_result != Offboard::Result::Success) {
                std::cerr << "Failed to start Offboard mode: " << offboard_result << std::endl;
                curr_state = reset;
                break;
            }
            while (true) {
                if (kill_switch.load()) {
                    curr_state = reset;
                    break;
                }

                lock.lock();

                if (cv_tvec_small.wait_for(lock, std::chrono::seconds(1), [] {return tvec_small; })) {
                    if (kill_switch.load()) {
                        std::cout << "Kill switch recognized, killing program" << std::endl;
                        curr_state = reset;
                        lock.unlock();
                        break;
                    }
                    else {
                        std::cout << "Detected we are close enough to marker in x and y" << std::endl;
                        tvec_small = false;
                        //curr_state = dropping;
                        lock.unlock();
                        break;
                    }
                }



                if (cv_sync.wait_for(lock, std::chrono::seconds(5), [] { return marker_found; })) { //received signal
                    // Proceed if the condition variable was notified and the condition is true
                    std::cout << "Marker respotted" << std::endl;
                    moveVec = markerScan();
                    localtvec = tvecmarkerInfo[moveVec.first];
                    //signal_det = true;
                    marker_found = false;
                    for (int i = 0; i < 3; i++) {
                        if (localtvec[i] > 10) {
                            break_out = true;
                        }
                    }
                    if (break_out) {
                        break_out = false;
                        std::cout << "Vector was too big" << std::endl;
                        curr_state = reset;
                        break;
                    }
                    ned_position[0] -= localtvec[1] / 3.281;
                    ned_position[1] += localtvec[0] / 3.281;
                    offboard.set_position_ned({
                        ned_position[0],
                        ned_position[1],
                        ned_position[2],
                        0.0 // Keep yaw constant for simplicity
                        });
                    sleep_for(0.5s);

                }
                else {
                    // Timeout occurred
                    std::cout << "Timed out, go up?\n";
                }

                lock.unlock();
            }
            //curr_state = searching;
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            break;
        case dropping:
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            std::cout << "Entered dropping state" << std::endl;
            lock.lock();
            curr_gps = telemetry.raw_gps();
            curr_alt = telemetry.altitude();

            lock.unlock();
            if (kill_switch.load()) {
                std::cout << "Kill switch recognized, killing program" << std::endl;
                curr_state = reset;
                break;
            }
            gps_res = action.goto_location(curr_gps.latitude_deg, curr_gps.longitude_deg, global_alt - curr_alt.altitude_terrain_m + 2.0, 0);
            if (gps_res != Action::Result::Success) {
                std::cout << "Failed to move to next search index" << std::endl;
                curr_state = reset;
                break;
            }
            gps_res = action.hold();
            if (gps_res != Action::Result::Success) {
                std::cout << "Failed to move to next search index" << std::endl;
                curr_state = reset;
                break;
            }
            sleep_for(3s);
            gps_res = action.goto_location(curr_gps.latitude_deg, curr_gps.longitude_deg, global_alt, 0.0);
            if (gps_res != Action::Result::Success) {
                std::cout << "Failed to move to next search index" << std::endl;
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

            sleep_for(seconds(10));

            return 0;

        }
    }
}
