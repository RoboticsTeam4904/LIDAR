#include "point_preprocess.h"

/**
   Adds a single point between several other points based on slope
   and distance comparisons. This is performed in place on the
   entire array, rather than just on the linked list of nonzero points.
 */
void interpolate(uint16_t * distances){
	for(uint16_t i = 0; i < 360; i++){
		if(distances[i] == 0){
			uint16_t last_distance = distances[(i - 1) % 360];
			uint16_t last_last_distance = distances[(i - 2) % 360];
			uint16_t next_distance = distances[(i + 1) % 360];
			uint16_t next_next_distance = distances[(i + 1) % 360];
			if(last_distance != 0 && last_last_distance != 0 &&  (next_distance != 0 || next_next_distance != 0)){
				uint16_t slope = last_last_distance - last_distance;
				if(next_distance != 0){
					if((last_distance + slope * 2) > next_distance - INTERPOLATE_THRESHOLD &&
					   (last_distance - slope * 2) < next_distance + INTERPOLATE_THRESHOLD){
						distances[i] = last_distance + slope;
					}
				}
				else{
					if((last_distance + slope * 3) > next_next_distance - INTERPOLATE_THRESHOLD &&
					   (last_distance - slope * 3) < next_next_distance + INTERPOLATE_THRESHOLD){
						distances[i] = last_distance + slope;
					}
				}
			}
			if(next_distance != 0 && next_next_distance != 0 &&  (last_distance != 0 || last_last_distance != 0)){
				uint16_t slope = next_distance - next_next_distance;
				if(last_distance != 0){
					if((next_distance - slope * 2) < last_distance + INTERPOLATE_THRESHOLD &&
					   (next_distance + slope * 2) > last_distance - INTERPOLATE_THRESHOLD){
						distances[i] = next_distance - slope;
					}
				}
				else{
					if((next_next_distance - slope * 3) < last_distance + INTERPOLATE_THRESHOLD &&
					   (next_next_distance + slope * 3) > last_distance - INTERPOLATE_THRESHOLD){
						distances[i] = next_distance - slope;
					}
				}
			}
		}
	}
}

void load_linked_list(uint16_t * distances, doubly_linked_list_node<lidar_datapoint> * lidar_data_start){

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
	doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;
	bool finished = false;

	while(node != lidar_data_start->prev && !finished){
		int16_t next_datapoint = node->next->data->radius;
		int16_t node_range = node->data->radius / 16;
		if(in_range(node->data->radius, next_datapoint, node_range)){
			int16_t prev_datapoint = node->prev->data->radius;
			if(in_range(node->data->radius, prev_datapoint, node_range)){
				node->data->radius = (2*node->data->radius +
						      next_datapoint +
						      prev_datapoint) / 4;
			}
		}
		if(node == lidar_data_start->prev){
			finished = true;
		}
		node = node->next;
	}
}

/**
   Sets the cartersian variables of the lidar_datapoint.
   This does not modify the theta or radius variables.
 */
void add_cartesian(lidar_datapoint * point){
	point->x = cos((float) point->theta * M_PI/180.0f)*point->radius;
	point->y = sin((float) point->theta * M_PI/180.0f)*point->radius;
}

/**
   Add the cartesian portion of each lidar_datapoint
   
   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
 */
void add_cartesians(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;
	bool finished = false;

	while(node != lidar_data_start && !finished){
		add_cartesian(node->data);
		if(node == lidar_data_start->prev){
			finished = true;
		}
		node = node->next;
	}
}
