#include "boiler_find.h"

#include <iostream>
#include <cmath>

using namespace std;

float get_angle(line * line1, line * line2){
	float angle1 = atan2(line1->start_y - line1->end_y,
			     line1->start_x - line1->end_x);
	cout << line1->start_x << ","<< line1->start_y << "\t" << line1->end_x << "," << line1->end_y << "\n";
	float angle2 = atan2(line2->start_y - line2->end_y,
			     line2->start_x - line2->end_x);
	cout << line2->start_x << ","<< line2->start_y << "\t" << line2->end_x << "," << line2->end_y << "\n";
	cout << angle1*180/M_PI << "\t" << angle2*180/M_PI << "\n";
	float angle = angle2 - angle1;
	if(angle < 0.0f){
		angle = angle + M_PI*2.0;
	}
	return angle;
}

bool test_distance(line * line1, line * line2){
	uint32_t point_distance = (line1->end_x * line1->end_x + line1->end_y * line1->end_y)/ 256;

	int16_t x_distance = line1->end_x - line2->start_x;
	int16_t y_distance = line1->end_y - line2->start_y;
	uint32_t distance = x_distance * x_distance + y_distance * y_distance;

	return distance < point_distance;
}

uint16_t line_length(line * line1){
	int16_t x_delta = line1->start_x - line1->end_x;
	int16_t y_delta = line1->start_y - line1->end_y;
	return sqrt(x_delta*x_delta + y_delta*y_delta);
}

boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start){
	doubly_linked_list_node<line> * node = line_data_start;

	boiler_location location;
	location.delta_theta = 0;

	while(node != line_data_start->prev){
		uint16_t length = line_length(node->data);
		if(length > MINIMUM_LENGTH){
			float angle = get_angle(node->data, node->next->data);
			cout << angle*180.0/M_PI << "\t";
			cout << test_distance(node->data, node->next->data) << "\n";
		}
		node = node->next;
	}

	return location;
}
