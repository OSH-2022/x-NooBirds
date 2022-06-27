#include "includes.h"
#include "detect_const.h"

using namespace std;
using namespace cv;

int main(int argc, const char** argv) {
	// 1.创建视频采集对象;
	VideoCapture cap;

	// 2.打开默认相机;
	cap.open(0);

	// 3.判断相机是否打开成功;
	if (!cap.isOpened())
		return -1;

	// 4.显示窗口命名;
	namedWindow("Video", 1);
	for (;;) {
		Mat frame;
		cap >> frame;
        if (frame.empty())
			return 0;

        Mat gs_frame;
        GaussianBlur(frame, gs_frame, Size(5, 5), 0);

        Mat hsv;
        cvtColor(gs_frame, hsv, COLOR_BGR2HSV);

        Mat erode_hsv;
        erode(hsv, erode_hsv, Mat());

        Mat mask = isRed(erode_hsv);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        for (auto itr = contours.begin(); itr != contours.end(); itr++) {
			auto rect = minAreaRect(*itr);
			Point2f rect_points[4];
        	rect.points(rect_points);
        	for ( int i = 0; i < 4; i++ ) {
            	line(frame, rect_points[i], rect_points[(i + 1) % 4], Scalar(0, 255, 255));
        	}
		}

		// 显示新的帧;

		cv::namedWindow("Video", 0);
		imshow("Video", frame);
		
		// 按键退出显示;
		if (waitKey(30) >= 0) break;
	}
	cap.release();

	return 0;
}
