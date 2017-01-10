#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <cstdlib>

struct lidar_datapoint {
	int16_t theta;
	int16_t radius;

	int16_t x;
	int16_t y;
};

struct line {
	int16_t start_x;
	int16_t start_y;
	int16_t end_x;
	int16_t end_y;
};

struct boiler_location {
	int16_t delta_x;
	int16_t delta_y;
	int16_t delta_theta;
};


#endif // __DATATYPES_H__
