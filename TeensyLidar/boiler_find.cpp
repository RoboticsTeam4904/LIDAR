#include "boiler_find.h"
#include "math_util.h"

#include <Arduino.h>

/**
   Calculates the angle of a line.
   Warning: uses expensive trig function
   @return the angle of the line in radians
*/
float calculate_angle(line * line1) {
  return atan2(line1->start_y - line1->end_y, line1->start_x - line1->end_x);
}

/**
   Calculates the angle between two lines.
   This function calculates the angle of each line via
   arctangent, then subtracts them.
   It will always return a value between 0.0f and 6.28f
   in radians.
*/
float get_angle(line * line1, line * line2) {
  float angle = calculate_angle(line2) - calculate_angle(line1);
  if (angle < 0.0f) {
    angle = angle + M_PI * 2.0;
  }
  return angle;
}

/**
   Determines if the end of one line is within a certain distance of the start
   of another line.
   @return true if the end of line1 is within 1/256th of the length of line1 squared
   of the start of line2, false otherwise
*/
bool test_distance(line * line1, line * line2) {
  int16_t x_distance = line1->end_x - line2->start_x;
  int16_t y_distance = line1->end_y - line2->start_y;
  uint32_t distance = x_distance * x_distance + y_distance * y_distance;

  return distance < ENDPOINT_DISTANCE * ENDPOINT_DISTANCE;
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
boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start, uint8_t alliance) {
  doubly_linked_list_node<line> * node = line_data_start;

  boiler_location location;
  location.delta_x = 0;
  location.delta_y = 0;
  bool finished = false;

  while (!finished) {
    float angle = get_angle(node->data, node->next->data);
    if (angle < TARGET_ANGLE + ANGLE_RANGE && angle > TARGET_ANGLE - ANGLE_RANGE) {
      bool nearby = test_distance(node->data, node->next->data);
      if (nearby) {
        int16_t delta_x;
        int16_t delta_y;
        if (alliance == BLUE_ALLIANCE) {
          float slope = get_slope(node->data->start_x, node->data->start_y, node->data->end_x, node->data->end_y);
          float boiler_width_delta_x = sqrt(((BOILER_WIDTH / 2.0f) * (BOILER_WIDTH / 2.0f)) / (slope * slope));
          float boiler_width_delta_y = boiler_width_delta_x * slope;
          delta_x = node->next->data->end_x + boiler_width_delta_x;// - (1 / slope) * BOILER_DEPTH;
          delta_y = node->next->data->end_y + boiler_width_delta_y;// - slope * BOILER_DEPTH;
        }
        else if (alliance == RED_ALLIANCE) {
          float slope = get_slope(node->data->start_x, node->data->start_y, node->data->end_x, node->data->end_y);
          delta_x = node->next->data->start_x + slope * BOILER_WIDTH / 2.0f;// + (1 / slope) * BOILER_DEPTH;
          delta_y = node->next->data->start_y + (1 / slope) * BOILER_WIDTH / 2.0f;// + slope * BOILER_DEPTH;
        }
        else {
          break;
        }

        location.delta_x = delta_x;
        location.delta_y = delta_y;
        
        break;
      }
    }
    if (node->next == line_data_start) {
      finished = true;
    }
    node = node->next;
  }

  return location;
}
