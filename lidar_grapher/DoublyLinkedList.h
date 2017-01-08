#ifndef __DoublyLinkedList_H__
#define __DoublyLinkedList_H__

/**
   Struct for a node in a DoublyLinkedList
   A DoublyLinkedListNode is of a certain
   type, T.
 */
template <typename T>
struct DoublyLinkedListNode {
	/**
	   Pointer to the data
	 */
	T * data;
	/**
	   Pointer to the next
	   node in the list
	*/
	DoublyLinkedListNode<T> * next;
	/**
	   Pointer to the previous
	   node in the list
	*/
	DoublyLinkedListNode<T> * prev;
};

#endif // __DoublyLinkedList_H__
