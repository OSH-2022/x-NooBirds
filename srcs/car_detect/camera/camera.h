#ifndef CAMERA_H_
#define CAMERA_H_

#include "includes.h"

#define UVC_DEVICE_PATH "/dev/video0"
#define WIDTH 1920
#define HEIGHT 1080
#define SHOT_INTERVAL 33333

typedef struct {
	uint8_t *start;
	size_t length;
} buffer_t;

class camera {
public:
	camera(const char *device_path);
	~camera();

	int capture(struct timeval timeout);
	void create_bmp(const char *bmp_path);

private:
	int			devfd;
	uint32_t	width;
	uint32_t 	height;
	size_t 		buffer_count;
	buffer_t	*buffers;
	buffer_t	head;
};

#endif // CAMERA_H_
