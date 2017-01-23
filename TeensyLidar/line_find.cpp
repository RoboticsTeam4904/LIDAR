#include "line_find.h"
#include "math_util.h"

/**
   Calculate the lines within the dataset.
   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
	Note that the list should be circular in both directions
   @return a doubly linked list of lines
*/
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start) {
  doubly_linked_list_node<line> * first_line = NULL;
  doubly_linked_list_node<line> * previous_line = NULL;

  doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;

  while (node->data->theta < lidar_data_start->prev->data->theta) {
    float slope = get_slope(node->data, node->next->data);
    int16_t distance = get_distance_squared(node->data, node->next->data);

    doubly_linked_list_node<lidar_datapoint> * start_node = node;
    doubly_linked_list_node<lidar_datapoint> * end_node = node;
    uint8_t length = 0;

    // Run backward (decreasing angle) slope and distance comparison
    while (true) {
      start_node = start_node->prev;
      length++;

      float new_slope = get_slope(start_node->data, end_node->data);
      int16_t new_distance = get_distance_squared(start_node->data, start_node->next->data);

      if (!in_range(slope, new_slope, SLOPE_LIMIT)
          || !in_range(distance, new_distance, DISTANCE_LIMIT)) {
        start_node = start_node->next;
        length--;
        break;
      }
    }

    // Run forward (increasing angle) slope and distance comparison
    while (true) {
      end_node = end_node->next;
      length++;
      if (end_node == start_node) {
        break;
      }

      float new_slope = get_slope(start_node->data, end_node->data);
      int16_t new_distance = get_distance_squared(end_node->prev->data, end_node->data);

      if (!in_range(slope, new_slope, SLOPE_LIMIT)
          || !in_range(distance, new_distance, DISTANCE_LIMIT)) {
        end_node = end_node->prev;
        length--;
        break;
      }
    }

    // Add line if it meets length requirements
    if (length > MIN_LINE_LENGTH) {
      if (previous_line == NULL) {
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
      else {
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

    if (end_node->next->data->theta < start_node->next->data->theta) {
      break;
    }
    node = end_node->next;
  }

  previous_line->next = first_line;
  first_line->prev = previous_line;

  return first_line;
}
