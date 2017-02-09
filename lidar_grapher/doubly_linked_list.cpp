#include "doubly_linked_list.h"

void lidar_datapoint_list_cleanup(doubly_linked_list_node<lidar_datapoint> * first_node) {
  doubly_linked_list_node<lidar_datapoint> * node = first_node;
  doubly_linked_list_node<lidar_datapoint> * end_node = first_node->prev;

  while (node != end_node) {
    doubly_linked_list_node<lidar_datapoint> * next = node->next;
    delete node->data;
    delete node;
    node = next;
  }

  delete node->data;
  delete node;

  first_node = NULL;
}

void line_list_cleanup(doubly_linked_list_node<line> * first_node) {
  doubly_linked_list_node<line> * node = first_node;
  doubly_linked_list_node<line> * end_node = first_node->prev;

  while (node != end_node) {
    doubly_linked_list_node<line> * next = node->next;
    delete node->data;
    delete node;
    node = next;
  }

  delete node->data;
  delete node;

  first_node = NULL;
}
