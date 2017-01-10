#include "line_find.h"

#include <iostream>
#include <cmath>

#define PI 3.14159265
#define SLOPE_LIMIT 0.1

using namespace std;

void add_cartesian(lidar_datapoint * point){
	point->x = cos((float) point->theta * PI/180.0f)*point->radius;
	point->y = sin((float) point->theta * PI/180.0f)*point->radius;
}

float get_slope(lidar_datapoint * point1, lidar_datapoint * point2){
	float dy = (float) (point2->y - point1->y);
	float dx = (float) (point2->x - point1->x);

	float slope = (dy / dx);

	if(slope > 1 || slope < -1){ // Maintain accuracy at high angles
		slope = (dx / dy);
	}

	return slope;
}

bool test_distance(lidar_datapoint * point1, lidar_datapoint * point2){
	uint32_t point_distance = (point1->x * point1->x + point1->y * point1->y)/ 64;

	int16_t x_distance = point1->x - point2->x;
	int16_t y_distance = point1->y - point2->y;
	uint32_t distance = x_distance * x_distance + y_distance * y_distance;

	return distance < point_distance;
}

int16_t abs_mod(int16_t i, int16_t mod){
	return ((i % mod) + mod) % mod;
}

bool in_range(int16_t a, int16_t b, int16_t range){
	return (a + range > b) && (a - range < b);
}

bool in_range(float a, float b, float range){
	return (a + range > b) && (a - range < b);
}

void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	doubly_linked_list_node<lidar_datapoint> * node;
	
	for(uint8_t l = 0; l < 10; l++){
		node = lidar_data_start;

		while(node != lidar_data_start->prev){
			int16_t next_datapoint = node->next->data->radius;
			int16_t prev_datapoint = node->prev->data->radius;
			if(in_range(node->data->radius,
				    next_datapoint,
				    node->data->radius / 16) &&
			   in_range(node->data->radius,
				    prev_datapoint,
				    node->data->radius / 16)){
				node->data->radius = (2 * node->data->radius +
							 next_datapoint +
							 prev_datapoint) / 4;
			}

			node = node->next;
		}
	}
	node = lidar_data_start;
	
	while(node != lidar_data_start->prev){
		add_cartesian(node->data);
		node = node->next;
	}
}

vector<line> get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	vector<line> lines;
	blur_points(lidar_data_start);

	doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;

	while(node->data->theta < lidar_data_start->prev->data->theta){
		float slope = get_slope(node->data, node->next->data);

		doubly_linked_list_node<lidar_datapoint> * start_node = node;
		doubly_linked_list_node<lidar_datapoint> * end_node = node;
		uint8_t length = 0;

		while(true){
			start_node = start_node->prev;
			length++;

			float new_slope = get_slope(start_node->data, end_node->data);

			if(!in_range(slope, new_slope, SLOPE_LIMIT)
			   || !test_distance(start_node->next->data, end_node->data)){
				start_node = start_node->next;
				length--;
				break;
			}
		}

		while(true){
			end_node = end_node->next;
			length++;
			if(end_node == start_node){
				break;
			}

			float new_slope = get_slope(start_node->data, end_node->data);

			if(!in_range(slope, new_slope, SLOPE_LIMIT)
			   || !test_distance(end_node->data, end_node->prev->data)){
				end_node = end_node->prev;
				length--;
				break;
			}
		}

		if(length > 4){
			lines.push_back(line(start_node->data, end_node->data));
		}

		node = end_node->next;
	}
	
	return lines;
}
