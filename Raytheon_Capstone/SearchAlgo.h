//
// Created by zach on 5/29/24.
//

#ifndef RAYTHEON_CAPSTONE_SEARCHALGO_H
#define RAYTHEON_CAPSTONE_SEARCHALGO_H
#include <vector>
#include <utility>
#include <GeographicLib/Geodesic.hpp>


std::vector<std::pair<double,double>> SearchAlgo(double lat1, double long1, double lat2, double long2, double lat3, double long3, double lat4, double long4, double altitude,
                                                 const std::vector<std::pair<double,double>> localSearch);

std::pair<double, double> localToGlobal(double lat1, double long1, std::vector<float> vec);
#endif //RAYTHEON_CAPSTONE_SEARCHALGO_H
