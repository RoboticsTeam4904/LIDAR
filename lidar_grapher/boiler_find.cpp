#include "boiler_find.h"

#include <iostream>
#include <cmath>

using namespace std;

/**
   Calculates the angle of a line.
   Warning: uses expensive trig function
   @return the angle of the line in radians
 */
float calculate_angle(line * line1){
	return atan2(line1->start_y - line1->end_y, line1->start_x - line1->end_x);
}

/**
   Calculates the angle between two lines.
   This function calculates the angle of each line via
   arctangent, then subtracts them.
   It will always return a value between 0.0f and 6.28f
   in radians.
 */
float get_angle(line * line1, line * line2){
	float angle = calculate_angle(line2) - calculate_angle(line1);
	if(angle < 0.0f){
		angle = angle + M_PI*2.0;
	}
	return angle;
}

/**
   Determines if the end of one line is within a certain distance of the start
   of another line.
   @return true if the end of line1 is within 1/256th of the length of line1 squared
   of the start of line2, false otherwise
 */
bool test_distance(line * line1, line * line2){
	uint32_t point_distance = (line1->end_x * line1->end_x + line1->end_y * line1->end_y)
		/ENDPOINT_DIVISOR;

	int16_t x_distance = line1->end_x - line2->start_x;
	int16_t y_distance = line1->end_y - line2->start_y;
	uint32_t distance = x_distance * x_distance + y_distance * y_distance;

	return distance < point_distance;
}

/**
   Measures the length of the line.
   Warning: contains sqrt, expensive to compute
   @return the length of line1, calculated via pythagorean theorum
*/
uint16_t line_length(line * line1){
	int16_t x_delta = line1->start_x - line1->end_x;
	int16_t y_delta = line1->start_y - line1->end_y;
	return sqrt(x_delta*x_delta + y_delta*y_delta);
}

/**
   Calculate the location of the boiler based on line data.
   The algorithm determines the angle between all adjacent lines,
   then checks if the end points of the line are near each other.
   Note that this algorithm will fail if multiple 135* angles
   are within lidar vision.
   @param
   	The first node in a doubly linked list of lines.
	This is not modified.
   @return a boiler_location struct containing the location of the boiler
   	Defaults to all zeros if none is found (please check for this)
 */
boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start){
	doubly_linked_list_node<line> * node = line_data_start;

	boiler_location location;
	location.delta_x = 0;
	location.delta_y = 0;
	location.delta_theta = 0;
	bool finished = false;

	while(!finished){
		uint16_t length = line_length(node->data);
		if(length > MINIMUM_LENGTH){
			float angle = get_angle(node->data, node->next->data);
			// TODO: compare angle, load boiler data
			bool nearby = test_distance(node->data, node->next->data);
			if(nearby){
				location.delta_x = node->data->end_x;
				location.delta_y = node->data->end_y;
				location.delta_theta = calculate_angle(node->data);
			}
		}
		if(node->next == line_data_start){
			finished = true;
		}
		node = node->next;
	}

	return location;
}
