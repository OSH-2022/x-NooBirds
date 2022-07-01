#include "includes.h"
#include "detect_const.h"
#include "timer.h"
#include "coordinate.h"

using namespace std;
using namespace cv;

int main(int argc, const char** argv) {
	Coordinate space(1);
	printTime();
	while (true) {
		space.run();
		auto data = space.getData();
		printf(
			"%d %d, %d %d, %d %d, %d %d\n",
			data.time.sec, data.time.msec,
			data.cars[0].x, data.cars[0].y,
			data.cars[1].x, data.cars[1].y,
			data.cars[2].x, data.cars[2].y
		);
	}
	return 0;
}
