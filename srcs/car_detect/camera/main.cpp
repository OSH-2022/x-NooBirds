#include "includes.h"
#include "camera.h"
#include "timer.h"
#include "detect.h"

int main(int argc, char *argv[]) {
	int i = 0;
	std::string bmp_path = "output.bmp";
	const char *device_path = UVC_DEVICE_PATH;
	struct timeval timeout;
	
	if (1 < argc) device_path = argv[1];
	if (access(device_path, F_OK) != 0) {
		perror("[ERROR] can not find the usb camera device");
		return -1;
	}
	
	camera dev(device_path);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = SHOT_INTERVAL;

	for (i = 0; i < 100; ++i) {
		printf("%d ", i);
		accurate_time_t time;
		get_time(&time);
		printf("%d, %d ", time.sec, time.msec);
		dev.capture(timeout);
		get_time(&time);
		printf("%d, %d\n", time.sec, time.msec);
		dev.create_bmp((std::to_string(i) + bmp_path).c_str());
	}

	// Space space(p_camera->head.start, WIDTH, HEIGHT);
	accurate_time_t time;
	get_time(&time);
	printf("%d, %d ", time.sec, time.msec);
	// space.update(p_camera->head.start);
	get_time(&time);
	printf("%d, %d ", time.sec, time.msec);
	
	return 0;
}