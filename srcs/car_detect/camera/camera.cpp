#include "camera.h"
#include "bmp.h"

static inline int minmax(int min, int v, int max) {
	return (v < min) ? min : ((max < v) ? max : v);
}

static uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height) {
	uint32_t i = 0, j = 0;
	int32_t y0 = 0, u = 0, y1 = 0, v = 0;
	int index = 0;
	uint8_t* rgb = (uint8_t*)calloc(width * height * 3, sizeof(uint8_t));
	
	for (i = 0; i < height; ++i) {
		for (j = 0; j < width; j += 2) {
			index = i * width + j;
			y0 = yuyv[index * 2 + 0] << 8;
			v = yuyv[index * 2 + 1] - 128;
			y1 = yuyv[index * 2 + 2] << 8;
			u = yuyv[index * 2 + 3] - 128;
			
			rgb[index * 3 + 0] = minmax(0, (y0 + 359 * v) >> 8, 255);
			rgb[index * 3 + 1] = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
			rgb[index * 3 + 2] = minmax(0, (y0 + 454 * u) >> 8, 255);
			rgb[index * 3 + 3] = minmax(0, (y1 + 359 * v) >> 8, 255);
			rgb[index * 3 + 4] = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
			rgb[index * 3 + 5] = minmax(0, (y1 + 454 * u) >> 8, 255);
		}
	}
	
	return rgb;
}

camera::camera(const char *device_path) {
	int fd = open(device_path, O_RDWR | O_NONBLOCK);
	if (fd < 0) exit(-1);
	
	devfd        = fd;
	width        = WIDTH;
	height       = HEIGHT;
	buffer_count = 0;
	buffers      = NULL;
	head.start   = NULL;
	head.length  = 0;

	struct v4l2_capability cap;
	
	memset(&cap, 0, sizeof(cap));
	if (ioctl(devfd, VIDIOC_QUERYCAP, &cap) >= 0) {
		if (! (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
			printf("[ERROR] camera_init: no campture\n");
			exit(-1);
		}
		
		if (! (cap.capabilities & V4L2_CAP_STREAMING)) {
			printf("[ERROR] camera_init: no streaming\n");
			exit(-1);
		}		
	}
	
	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof(cropcap));
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(devfd, VIDIOC_CROPCAP, &cropcap) >= 0) {
		struct v4l2_crop crop;
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;
		
		ioctl(devfd, VIDIOC_S_CROP, &crop);
	}
	
	struct v4l2_format format;
	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	format.fmt.pix.field = V4L2_FIELD_NONE;
	ioctl(devfd, VIDIOC_S_FMT, &format);

	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(req));
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	ioctl(devfd, VIDIOC_REQBUFS, &req);
	buffer_count = req.count;
	buffers = (buffer_t*)calloc(req.count, sizeof(buffer_t));
	
	size_t buf_max = 0;
	struct v4l2_buffer buffer;
	for (size_t i = 0; i < buffer_count; ++i) {
		memset(&buffer, 0, sizeof(buffer));
		
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		
		ioctl(devfd, VIDIOC_QUERYBUF, &buffer);
		if (buffer.length > buf_max)
			buf_max = buffer.length;
		
		buffers[i].length = buffer.length;
		buffers[i].start = (uint8_t*)mmap(
			NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, 
			fd, buffer.m.offset);
		if (NULL == buffers[i].start) {
			perror("mmap error");
			return ;
		}
	}
	
	head.start = (uint8_t *)malloc(buf_max);

	for (size_t i = 0; i < buffer_count; ++i) {
		memset(&buffer, 0, sizeof(buffer));
		
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		
		ioctl(devfd, VIDIOC_QBUF, &buffer);
	}
	
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(devfd, VIDIOC_STREAMON, &type);
}

camera::~camera() {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(devfd, VIDIOC_STREAMOFF, &type);

	size_t i = 0;
	for (i = 0; i < buffer_count; ++i)
		munmap(buffers[i].start, buffers[i].length);
	
	free(buffers);
	free(head.start);
	close(devfd);
}

int camera::capture(struct timeval timeout) {
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(devfd, &fds);
	
	int nready = select(devfd + 1, &fds, NULL, NULL, &timeout);

	if (nready < 0) {
		perror("[ERROR] camera_frame");
		return nready;
	}
	
	if (nready == 0) return nready;

	struct v4l2_buffer buffer;
	
	memset(&buffer, 0, sizeof(buffer));
		
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;

	ioctl(devfd, VIDIOC_DQBUF, &buffer);

	memcpy(head.start, buffers[buffer.index].start, buffer.bytesused);
	head.length = buffer.bytesused;

	ioctl(devfd, VIDIOC_QBUF, &buffer);
	
	return 1;
}

void camera::create_bmp(const char *bmp_path) {
	uint8_t* rgb = yuyv2rgb(head.start, width, height);
	
	int fd = open(bmp_path, O_RDWR | O_CREAT, 0755);
	if (fd < 0) {
		perror("[ERROR] camera_create_bmp open image file");
		free(rgb);
		exit(-1);
	}
	
	if (bmp_create(fd, rgb, width, height) < 0)
		printf("[ERROR] camera_create_bmp create bmp");	
	
	close(fd);	
	free(rgb);
}