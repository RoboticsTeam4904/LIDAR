#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <cstdlib>

struct lidar_datapoint {
	int16_t theta;
	int16_t radius;

	int16_t x;
	int16_t y;
};

struct line {
	int16_t start_x;
	int16_t start_y;
	int16_t end_x;
	int16_t end_y;
};

/**
   Struct for a node in a DoublyLinkedList
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

#endif // __DATATYPES_H__
