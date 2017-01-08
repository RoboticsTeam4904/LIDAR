#ifndef __LidarDatapoint_H__
#define __LidarDatapoint_H__

#include <cstdlib>

struct LidarDatapoint {
	int16_t theta;
	int16_t radius;

	int16_t x;
	int16_t y;
};

#endif // __LidarDatapoint_H__
