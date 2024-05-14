#include "OpenCV.h"
#include <iostream>

using namespace std;
using namespace cv;

class OpenCV
{
public:
	OpenCV() {
		
	}
	bool targetFound;
	std::vector<int> ids;
    std::vector<Vec3d> RotVecs, TranVecs;
	
	int test() {
        cv::VideoCapture inputVideo(0 + cv::CAP_DSHOW);
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        bool showRejected = true;
        bool estimatePose = false;
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

                inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
                inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
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
            
            vector<vector<Point2f> > corners, rejected;

            // detect markers and estimate pose
            detector.detectMarkers(image, corners, ids, rejected);

            size_t nMarkers = corners.size();
            vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

            if (estimatePose && !ids.empty()) {
                // Calculate pose for each marker
                for (size_t i = 0; i < nMarkers; i++) {
                    solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                    RotVecs.push_back(rvecs.at(i));
                    TranVecs.push_back(tvecs.at(i));
                }
            }
            //! [aruco_pose_estimation3]
            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            /*if (totalIterations % 30 == 0) {
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                    << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }*/
            //! [aruco_draw_pose_estimation]
            // draw results
           // image.copyTo(imageCopy);
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(image, corners, ids);

                if (estimatePose) {
                    for (unsigned int i = 0; i < ids.size(); i++)
                        break;
                    //cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);

                }
            }
            //! [aruco_draw_pose_estimation]

            if (showRejected && !rejected.empty())
                cv::aruco::drawDetectedMarkers(image, rejected, noArray(), Scalar(100, 0, 255));

            imshow("out", image);
            char key = (char)waitKey(1);
        }
        //! [aruco_detect_markers]
        return 0;
    }


};
