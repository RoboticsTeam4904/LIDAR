#ifndef __DOUBLY_LINKED_LIST_H__
#define __DOUBLY_LINKED_LIST_H__

#include <cstdlib>
#include "datatypes.h"

/**
   Struct for a node
   A doubly_linked_list_node is of a certain
   type, T.
*/
template <typename T>
struct doubly_linked_list_node {
  /**
     Pointer to the data
  */
  T * data;
  /**
     Pointer to the next
     node in the list
  */
  doubly_linked_list_node<T> * next;
  /**
     Pointer to the previous
     node in the list
  */
  doubly_linked_list_node<T> * prev;
};

void lidar_datapoint_list_cleanup(doubly_linked_list_node<lidar_datapoint> * first_node);
void line_list_cleanup(doubly_linked_list_node<line> * first_node);

#endif // __DOUBLY_LINKED_LIST_H__
