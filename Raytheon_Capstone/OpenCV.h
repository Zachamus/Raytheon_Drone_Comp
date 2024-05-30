#ifndef RAYTHEON_CAPSTONE_OPENCV_H
#define RAYTHEON_CAPSTONE_OPENCV_H
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <unordered_set>


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

    const std::string refineMethods[4] = {
            "None",
            "Subpixel",
            "Contour",
            "AprilTag"
    };

}



#endif //RAYTHEON_CAPSTONE_OPENCV_H