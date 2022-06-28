#include "coordinate.h"
#include "detect_const.h"

Coordinate::Coordinate(int id) {
    // open camera
	cap.open(0);
	cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	cap.set(CAP_PROP_FPS, FPS);

	// check camera state
	if (!cap.isOpened())
		exit(-1);

	count = 0;

    Mat frame, gsFrame, hsv;
	AcrtTime time;
    do {
		cap >> frame;
		time.update();

        if (frame.empty())
			continue;

		// Gaussian Blur
        GaussianBlur(frame, gsFrame, Size(5, 5), 0);
		// conveter BGR to HSV
        cvtColor(gsFrame, hsv, COLOR_BGR2HSV);
    } while (!updateBase(hsv));

	while (!trackObject(hsv, time)) {
		cap >> frame;
		time.update();

        if (frame.empty())
			continue;

		// Gaussian Blur
        GaussianBlur(frame, gsFrame, Size(5, 5), 0);
		// conveter BGR to HSV
        cvtColor(gsFrame, hsv, COLOR_BGR2HSV);
	}
}

bool Coordinate::run() {
	bool rst = true;
	count++;

	Mat frame, gsFrame, hsv;
	AcrtTime time;

	cap >> frame;
	time.update();

    if (frame.empty())
		return false;

	// Gaussian Blur
    GaussianBlur(frame, gsFrame, Size(5, 5), 0);
	// conveter BGR to HSV
    cvtColor(gsFrame, hsv, COLOR_BGR2HSV);

	if (!(count & 15)) {
		rst = rst && updateBase(hsv);
	}

	return rst && trackObject(hsv, time);
}

bool areaLarger(const vector<Point> &area1, vector<Point> &area2) {
	return contourArea(area1) > contourArea(area2);
}

Point2f realPoints[4] = {
    Point2f(   0,   0), Point2f(   0, REAL),
	Point2f(REAL,   0), Point2f(REAL, REAL)
};

bool Coordinate::updateBase(const Mat &hsv) {
	vector<vector<Point>> contours;
    Mat mask = isRed(hsv);
	// choose the contours
    findContours(mask, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    if (contours.size() < 4) return false;

	// sort the contours
	std::sort(contours.begin(), contours.end(), areaLarger);

	// update location of base points
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

	convert = getPerspectiveTransform(basePoints, realPoints);
	return true;
}

vector<vector<Point>>::const_iterator areaMax(const vector<vector<Point>> &contours) {
	int max = 0;
	auto rst = vector<vector<Point>>::const_iterator();
	for (auto itr = contours.begin(); itr != contours.end(); ++itr) {
		int area = contourArea(*itr);
		if (area > max) {
			rst = itr;
			max = area;
		}
	}
	return rst;
}

static inline float square(float num) {
	return num * num;
}

static inline float dist(const Point2f &a, const Point2f &b) {
	return sqrt(square(a.x - b.x) + square(a.y - b.y));
}

bool isCar(const Point2f &obj, const Point2f cars[3]) {
	for (int i = 0; i < 3; i++)
		if (dist(obj, cars[i]) > DETECT_TOLERANCE) return false;
	return true;
}

bool Coordinate::trackObject(const Mat &hsv, const AcrtTime &now) {
	Point2f cars[3];

	Mat erodeHsv, colorMask, realMask;
	vector<vector<Point>> contours;
	Point2f rectPoints[4];

	erode(hsv, erodeHsv, Mat());

	for (int color = yellow; color <= blue; ++color) {
		colorMask = isColor(erodeHsv, (color_t)color);
		warpPerspective(colorMask, realMask, convert, Size(REAL, REAL));
		imshow("", realMask);
		waitKey(30);
    	findContours(realMask, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		if (contours.empty()) return false;
		auto rect = minAreaRect(*areaMax(contours));
    	rect.points(rectPoints);
		cars[color - 1] = Point(
			(rectPoints[0].x + rectPoints[1].x
		   + rectPoints[2].x + rectPoints[3].x) / 4,
			(rectPoints[0].y + rectPoints[1].y
		   + rectPoints[2].y + rectPoints[3].y) / 4
		);
	}

	lock_guard<mutex> guard(dataMutex);

	data.time = now;
	for (int id = 0; id < 3; id++) data.cars[id] = cars[id];

	return true;
}

Coordinate::~Coordinate() {
    cap.release();
}

const mutex& Coordinate::getMutex() {
    return dataMutex;
}

const Package& Coordinate::getData() {
    return data;
}