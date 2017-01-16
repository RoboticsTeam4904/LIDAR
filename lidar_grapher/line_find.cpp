#include "line_find.h"

#include <iostream>
#include <cmath>

using namespace std;

/**
   Sets the cartersian variables of the lidar_datapoint.
   This does not modify the theta or radius variables.
 */
void add_cartesian(lidar_datapoint * point){
	point->x = cos((float) point->theta * M_PI/180.0f)*point->radius;
	point->y = sin((float) point->theta * M_PI/180.0f)*point->radius;
}

/**
   Calculates the slope of the line between two lidar_datapoints.
   This requires that the cartersian variables of the points be
   calculated already.
 */
float get_slope(lidar_datapoint * point1, lidar_datapoint * point2){
	float dy = (float) (point2->y - point1->y);
	float dx = (float) (point2->x - point1->x);

	float slope = (dy / dx);

	if(slope > 1 || slope < -1){ // Maintain accuracy at high angles
		slope = (dx / dy);
	}

	return slope;
}

/**
   Calculates the distance between two points.
   Standard Pythagorean theorum. Uses cartersian variables.
 */
float get_distance(lidar_datapoint * point1, lidar_datapoint * point2){
	float dy = (float) (point2->y - point1->y);
	float dx = (float) (point2->x - point1->x);

	float distance = sqrt(dx*dx + dy*dy);

	return distance;
}

/**
   Determines if two 16 bit ints are within a certain
   range of each other. range should be positive,
   although this function does not check.
 */
bool in_range(int16_t a, int16_t b, int16_t range){
	return (a + range > b) && (a - range < b);
}

/**
   Determines if two floats are within a certain
   range of each other. range should be positive,
   although this function does not check.
 */
bool in_range(float a, float b, float range){
	return (a + range > b) && (a - range < b);
}

/**
   Apply a transform to all the points in a list that "averages" their
   radii with nearby radii. This is similar to a linear approximation
   of a Gaussian blur one point in each direction.
   The transformation is performed in place.
   The cartesian portion of the lidar_datapoint is also filled out at
   the termination of this function.

   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
 */
void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	doubly_linked_list_node<lidar_datapoint> * node;
	
	for(uint8_t l = 0; l < BLUR_COUNT; l++){
		node = lidar_data_start;

		while(node != lidar_data_start->prev){
			int16_t next_datapoint = node->next->data->radius;
			int16_t node_range = node->data->radius / 16;
			if(in_range(node->data->radius, next_datapoint, node_range)){
				int16_t prev_datapoint = node->prev->data->radius;
				if(in_range(node->data->radius, prev_datapoint, node_range)){
					node->data->radius = (node->data->radius +
							      next_datapoint +
							      prev_datapoint) / 3;
				}
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

/**
   Calculate the lines within the dataset.
   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
	Note that the list should be circular in both directions
   @return a doubly linked list of lines
 */
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	doubly_linked_list_node<line> * first_line = NULL;
	doubly_linked_list_node<line> * previous_line = NULL;

	doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;

	while(node->data->theta < lidar_data_start->prev->data->theta){
		float slope = get_slope(node->data, node->next->data);
		int16_t distance = get_distance(node->data, node->next->data);

		doubly_linked_list_node<lidar_datapoint> * start_node = node;
		doubly_linked_list_node<lidar_datapoint> * end_node = node;
		uint8_t length = 0;

		// Run backward (decreasing angle) slope and distance comparison
		while(true){
			start_node = start_node->prev;
			length++;

			float new_slope = get_slope(start_node->data, end_node->data);
			int16_t new_distance = get_distance(start_node->data, start_node->next->data);

			if(!in_range(slope, new_slope, SLOPE_LIMIT)
			   || !in_range(distance, new_distance, DISTANCE_LIMIT)){
				start_node = start_node->next;
				length--;
				break;
			}
		}

		// Run forward (increasing angle) slope and distance comparison
		while(true){
			end_node = end_node->next;
			length++;
			if(end_node == start_node){
				break;
			}

			float new_slope = get_slope(start_node->data, end_node->data);
			int16_t new_distance = get_distance(end_node->prev->data, end_node->data);

			if(!in_range(slope, new_slope, SLOPE_LIMIT)
			   || !in_range(distance, new_distance, DISTANCE_LIMIT)){
				end_node = end_node->prev;
				length--;
				break;
			}
		}

		// Add line if it meets length requirements
		if(length > MIN_LINE_LENGTH){
			if(previous_line == NULL){
				previous_line = new doubly_linked_list_node<line>;
				previous_line->data = new line;
				previous_line->data->start_x = start_node->data->x;
				previous_line->data->start_y = start_node->data->y;
				previous_line->data->end_x = end_node->data->x;
				previous_line->data->end_y = end_node->data->y;
				previous_line->next = NULL;
				previous_line->prev = NULL;
				first_line = previous_line;
			}
			else{
				doubly_linked_list_node<line> * new_line = new doubly_linked_list_node<line>;
				new_line->data = new line;
				new_line->data->start_x = start_node->data->x;
				new_line->data->start_y = start_node->data->y;
				new_line->data->end_x = end_node->data->x;
				new_line->data->end_y = end_node->data->y;
				new_line->prev = previous_line;
				previous_line->next = new_line;
				previous_line = new_line;
			}
		}

		if(end_node->next->data->theta < start_node->next->data->theta){
			break;
		}
		node = end_node->next;
	}

	previous_line->next = first_line;
	first_line->prev = previous_line;
	
	return first_line;
}
