//
// Created by zach on 5/29/24.
//
#include "SearchAlgo.h"

using namespace GeographicLib;
using namespace std;
std::vector<std::pair<double,double>> SearchAlgo(double lat1, double long1, double lat2, double long2, double lat3, double long3, double lat4, double long4, double altitude, const vector<pair<double,double>> localSearch) {
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
    geod.Direct(lat1, long1, az12, 1.975, currlat, currlong);

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


std::pair<double, double> localToGlobal(double lat1, double long1, std::vector<double> vec) {
    Geodesic geod = Geodesic::WGS84();
    std::pair<double,double> outGPS;
    double new_lat, new_long, final_lat, final_long;
    for (int i = 0; i < vec.size(); i++ ) {
        if (abs(vec[i]) > 10) {
            outGPS.first = lat1;
            outGPS.second = long1;
            return outGPS;
        }
    }
    geod.Direct(lat1, long1, 0, vec[0], new_lat, new_long);
    geod.Direct(new_lat, new_long, 90, vec[1], final_lat, final_long);

    outGPS.first = final_lat;
    outGPS.second = final_long;

    return outGPS;

}


