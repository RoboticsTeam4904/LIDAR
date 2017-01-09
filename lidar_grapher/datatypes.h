#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <cstdlib>

struct LidarDatapoint {
	int16_t theta;
	int16_t radius;

	int16_t x;
	int16_t y;
};

struct line{
	int16_t start_x;
	int16_t start_y;

	int16_t end_x;
	int16_t end_y;
};

#endif // __DATATYPES_H__
