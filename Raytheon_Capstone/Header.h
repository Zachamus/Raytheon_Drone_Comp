#pragma once
#include <iostream>
#include <ctime>
/*#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
* */
#include <ctime>
#include <stack>
#include <iostream>
#include <math.h>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include <opencv2/video.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <time.h>
#include <mavsdk/plugins/offboard/offboard.h>



#define ASSERT_EQ(val1, val2) \
    if (!((val1) == (val2))) { \
        throw std::runtime_error("Assertion failed: " ); \
    }


#define ASSERT_TRUE(condition) \
    if (!(condition)) { \
        throw std::runtime_error("Assertion failed: " #condition); \
    }
