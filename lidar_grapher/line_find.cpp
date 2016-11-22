#include "line_find.h"

#include <cmath>

#define PI 3.14159265
#define SLOPE_LIMIT 0.1

using namespace std;

point get_cartesian(point polar){
	int16_t distance = get<1>(polar);
	return point(cos((float) get<0>(polar) * PI/180.0f)*distance,
		     sin((float) get<0>(polar) * PI/180.0f)*distance);
}

float get_slope(point point1, point point2){
	float dy = (float) get<1>(point2) - get<1>(point1);
	float dx = (float) get<0>(point2) - get<0>(point1);

	float slope = (dy / dx);

	if(slope > 1 || slope < -1){ // Maintain accuracy at high angles
		slope = (dx / dy);
	}

	return slope;
}

bool test_distance(point point1, point point2){
	uint32_t point_distance = (get<0>(point1) * get<0>(point1) + get<1>(point1) * get<1>(point1))/ 64;

	int16_t x_distance = get<0>(point1) - get<0>(point2);
	int16_t y_distance = get<1>(point1) - get<1>(point2);
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

vector<point> blur_points(vector<point> lidar_data){
	for(uint8_t l = 0; l < 10; l++){
		for(int16_t i = 0; i < lidar_data.size(); i++){
			int16_t next_datapoint = get<1>(lidar_data[(i + 1) % lidar_data.size()]);
			int16_t prev_datapoint = get<1>(lidar_data[abs_mod(i - 1, lidar_data.size())]);
			if(in_range(get<1>(lidar_data[i]),
				    next_datapoint,
				    get<1>(lidar_data[i]) / 16) &&
			   in_range(get<1>(lidar_data[i]),
				    prev_datapoint,
				    get<1>(lidar_data[i]) / 16)){
				get<1>(lidar_data[i]) = (2 * get<1>(lidar_data[i]) +
							 next_datapoint +
							 prev_datapoint) / 4;
			}
		}
	}
	for(int16_t i = 0; i < lidar_data.size(); i++){
		lidar_data[i] = get_cartesian(lidar_data[i]);
	}

	return lidar_data;
}

vector<line> get_lines(vector<point> lidar_data){
	vector<tuple<point, point>> lines;
	vector<point> blurred_data = blur_points(lidar_data);
	blurred_data.shrink_to_fit();
	int16_t max = blurred_data.size();
	for(int16_t i = 0; i < max; i++){
		float slope = get_slope(blurred_data[i], blurred_data[(i + 1) % blurred_data.size()]);

		int16_t start_idx = i;
		int16_t end_idx = i;

		while(true){
			int16_t last_start = start_idx;
			start_idx = abs_mod(start_idx - 1, blurred_data.size());

			float new_slope = get_slope(blurred_data[start_idx], blurred_data[end_idx]);

			if(!in_range(slope, new_slope, SLOPE_LIMIT) || !test_distance(blurred_data[start_idx], blurred_data[last_start])){
				start_idx = last_start;
				break;
			}
		}

		while(true){
			int16_t last_end = end_idx;
			end_idx = (end_idx + 1) % blurred_data.size();
			if(end_idx == start_idx){
				break;
			}

			float new_slope = get_slope(blurred_data[start_idx], blurred_data[end_idx]);

			if(!in_range(slope, new_slope, SLOPE_LIMIT) || !test_distance(blurred_data[end_idx], blurred_data[last_end])){
				end_idx = last_end;
				break;
			}
		}

		if(abs_mod(end_idx - start_idx, blurred_data.size()) > 4){
			lines.push_back(line(get_cartesian(lidar_data[start_idx]), get_cartesian(lidar_data[end_idx])));

			if(end_idx > i){ // If we have not looped around
				i = end_idx;
			}
			if(i == 0 && start_idx < max){ // If we have not looped around
				max = start_idx;
			}
		}
	}
	return lines;
}
