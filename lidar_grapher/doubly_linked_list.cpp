#include "doubly_linked_list.h"


void lidar_datapoint_list_cleanup(doubly_linked_list_node<lidar_datapoint> * first_node){
        doubly_linked_list_node<lidar_datapoint> * node = first_node;

        while(node != NULL && node != first_node){
		delete node->prev->data;
		delete node->prev;
		node = node->next;
        }

        first_node = NULL;
}

void line_list_cleanup(doubly_linked_list_node<line> * first_node){
        doubly_linked_list_node<line> * node = first_node;

        while(node != NULL && node != first_node){
		delete node->prev->data;
		delete node->prev;
		node = node->next;
        }

        first_node = NULL;
}
