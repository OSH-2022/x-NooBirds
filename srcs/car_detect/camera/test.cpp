#include "includes.h"
#include "detect_const.h"
#include "timer.h"

using namespace std;
using namespace cv;

bool areaBigger(const vector<Point> &area1, vector<Point> &area2) {
	return contourArea(area1) > contourArea(area2);
}

int main(int argc, const char** argv) {
	// class camera initialize
	VideoCapture cap;

	// open camera
	cap.open(0);

	// check camera state
	if (!cap.isOpened())
		exit(-1);

	for (;;) {
		Mat frame;
		cap >> frame;
        if (frame.empty())
			exit(-1);

		print_time();

		// Gaussian Blur
        Mat gs_frame;
        GaussianBlur(frame, gs_frame, Size(5, 5), 0);

		// conveter BGR to HSV
        Mat hsv;
        cvtColor(gs_frame, hsv, COLOR_BGR2HSV);

		// erode the frame
        Mat erode_hsv;
        erode(hsv, erode_hsv, Mat());

		// choose the red
        Mat mask = isRed(erode_hsv);

		// choose the contours
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

		if (contours.size() < 4) continue;

		// sort the contours
		sort(contours.begin(), contours.end(), areaBigger);
        for (int idx = 0; idx < 4; idx++) {
			auto rect = minAreaRect(contours[idx]);
			Point2f rect_points[4];
        	rect.points(rect_points);
        	for ( int i = 0; i < 4; i++ ) {
            	line(frame, rect_points[i], rect_points[(i + 1) % 4], Scalar(0, 255, 255));
        	}
		}
		// can be deleted later

		// calculate location in picture
		Point chosenPoints[4];
		for (int idx = 0; idx < 4; idx++) {
			auto rect = minAreaRect(contours[idx]);
			Point2f rectPoints[4];
        	rect.points(rectPoints);
        	chosenPoints[idx] = Point(
				(rectPoints[0].x + rectPoints[1].x
			   + rectPoints[2].x + rectPoints[3].x) / 4,
				(rectPoints[0].y + rectPoints[1].y
			   + rectPoints[2].y + rectPoints[3].y) / 4
			);
		}

		// assign each chosen point
		Point2f basePoints[4];
		for (int idx = 0; idx < 4; idx++) {
			if (chosenPoints[idx].y < FRAME_HEIGHT / 2) {
				if (chosenPoints[idx].x < FRAME_WIDTH / 2)
					basePoints[0] = chosenPoints[idx];
				else basePoints[1] = chosenPoints[idx];
			} else {
				if (chosenPoints[idx].x < FRAME_WIDTH / 2)
					basePoints[2] = chosenPoints[idx];
				else basePoints[3] = chosenPoints[idx];
			}
		}

		Mat convert = getPerspectiveTransform(basePoints, realPoints);
		Mat real;
		warpPerspective(frame, real, convert, real.size());

		print_time();

		// 显示新的帧;
		imshow("Video", frame);
		imshow("Real", real);

		// 按键退出显示;
		if (waitKey(30) >= 0) break;
	}
	cap.release();

	return 0;
}
