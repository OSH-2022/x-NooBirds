#include "detect.h"
#include "camera.h"

Pair::Pair() { }

Pair::Pair(float _x, float _y) {
    x = _x;
    y = _y;
}

Pair& Pair::operator=(const Pair &tar) {
    x = tar.x;
    y = tar.y;
    return *this;
}

Space::Space(const uint8_t* yuyv, unsigned int _width, unsigned int _height) {
    width = _width;
    height = _height;
    // To be done
}

Space::~Space() { }

Pair Space::track(const uint8_t* yuyv, color_set color) {
    return Pair();
}

static inline int minmax(int min, int v, int max) {
	return (v < min) ? min : ((max < v) ? max : v);
}

static inline bool isRed(int r, int g, int b) {
	return r > RED_MIN && r - g > RED_DELTA && r - b > RED_DELTA;
}

static inline bool isColor(color_set color, int r, int g, int b) {
    switch (color) {
    case red:
        return isRed(r, g, b);
    
    case yellow:
        return true;

    default:
        return false;
    }
}

void Space::update(const uint8_t* yuyv) {
    int window_width = width / DETECT_RANGE;
    int window_height = height / DETECT_RANGE;

    bool layer[window_height][window_width];

    int index, y0, v, y1, u, r, g, b;

    // update point near
    for (int i = 0; i < window_height; ++i) {
		for (int j = 0; j < window_width; j += 2) {
			index = i * width + j + window_width;
			y0 = yuyv[index * 2 + 0] << 8;
			v = yuyv[index * 2 + 1] - 128;
			y1 = yuyv[index * 2 + 2] << 8;
			u = yuyv[index * 2 + 3] - 128;
			
			r = minmax(0, (y0 + 454 * u) >> 8, 255);
            g = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
            b = minmax(0, (y0 + 359 * v) >> 8, 255);
            layer[i][j] = isRed(r, g, b);

            r = minmax(0, (y1 + 454 * u) >> 8, 255);
            g = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
            b = minmax(0, (y1 + 359 * v) >> 8, 255);
            layer[i][j + 1] = isColor(POINT_COLOR, r, g, b);
		}
	}

    int count = 0;
    unsigned int x_sum = 0;
    unsigned int y_sum = 0;
    for (int i = 1; i < window_height - 1; i++) {
        for (int j = 1; j < window_width - 1; j++) {
            if (layer[i - 1][j - 1] && layer[i - 1][j] && layer[i - 1][j + 1]
            && layer[i][j - 1] && layer[i][j] && layer[i][j + 1]
            && layer[i + 1][j - 1] && layer[i + 1][j] && layer[i + 1][j + 1]) {
                count++;
                x_sum += j;
                y_sum += i;
            }
        }
    }

    point_near = Pair(
        2 * window_width - (float)x_sum / count,
        HEIGHT - (float)y_sum / count);
    // end of update point near
    printf("%d %f %f\n", count, point_near.x, point_near.y);
    // update point far
    for (int i = 0; i < window_height; ++i) {
		for (int j = 0; j < window_width; j += 2) {
			index = (i + 2 * window_height) * width + j + window_width;
			y0 = yuyv[index * 2 + 0] << 8;
			v = yuyv[index * 2 + 1] - 128;
			y1 = yuyv[index * 2 + 2] << 8;
			u = yuyv[index * 2 + 3] - 128;
			
			r = minmax(0, (y0 + 454 * u) >> 8, 255);
            g = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
            b = minmax(0, (y0 + 359 * v) >> 8, 255);
            layer[i][j] = isRed(r, g, b);

            r = minmax(0, (y1 + 454 * u) >> 8, 255);
            g = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
            b = minmax(0, (y1 + 359 * v) >> 8, 255);
            layer[i][j + 1] = isColor(POINT_COLOR, r, g, b);
		}
	}

    count = 0;
    x_sum = 0;
    y_sum = 0;
    for (int i = 1; i < window_height - 1; i++) {
        for (int j = 1; j < window_width - 1; j++) {
            if (layer[i - 1][j - 1] && layer[i - 1][j] && layer[i - 1][j + 1]
            && layer[i][j - 1] && layer[i][j] && layer[i][j + 1]
            && layer[i + 1][j - 1] && layer[i + 1][j] && layer[i + 1][j + 1]) {
                count++;
                x_sum += j;
                y_sum += i;
            }
        }
    }

    point_far = Pair(
        2 * window_width - (float)x_sum / count,
        window_height - (float)y_sum / count);
    printf("%d %f %f\n", count, point_far.x, point_far.y);
}