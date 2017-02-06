#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cstdlib>
#include <cmath>

#ifdef TIME
#include <chrono>
#endif

#ifdef GUI
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

#include "lidar_teensy.h"
#include "line_find.h"
#include "datatypes.h"
#include "doubly_linked_list.h"
#include "boiler_find.h"
#include "point_preprocess.h"

using namespace std;

#ifdef GUI
/**
   Plot lidar data
   @param lidar_data_start the "first" node in a circular doubly linked
   list of lidar datapoints
*/
void plot(doubly_linked_list_node<lidar_datapoint> * lidar_data_start){
	glBegin(GL_POINTS);
	doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start;
	
	while(node != lidar_data_start->prev){ // Caution: skips one point
		glColor3f(0, 1.0f, 0);
		glVertex2i(cos((double) node->data->theta * M_PI/180.0f)*node->data->radius/10,
			   -sin((double) node->data->theta * M_PI/180.0f)*node->data->radius/10);
		node = node->next;
	}
	glEnd();
}
#endif

/**
   Process lidar data
   Draws a lattice, calculates (possibly timing) lines and boiler location,
   then draws lines and boiler location.
*/
void process(doubly_linked_list_node<lidar_datapoint> * lidar_data_start, int alliance){
	// Begin draw lattice 
#ifdef GUI
	if(lidar_data_start != NULL){
		glClear(GL_COLOR_BUFFER_BIT);
	}
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	for(int i = -2; i < 3; i++){
		for(int j = -2; j < 3; j++){
			glVertex2i(i*100, j*100);
		}
	}
	glEnd();
	
	glColor3f(0.5f, 1.0f, 0.5f);
	// End draw lattice

	// Plot raw lidar data
	plot(lidar_data_start);
#endif
	
#ifdef TIME
 	// Time calculate lines data
 	chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
#endif

	// Calculate lines
 	doubly_linked_list_node<line> * first_line =  get_lines(lidar_data_start);
	
	if(first_line != NULL){
#ifdef TIME
		chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();

		chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end - start);
		cout << "Calculate lines:\t" << time_span.count() << "\n";

		start = chrono::high_resolution_clock::now();
#endif

		// Calculate boiler
		boiler_location target = get_boiler(first_line, alliance);
	
#ifdef TIME
		end = chrono::high_resolution_clock::now();

		time_span = chrono::duration_cast<chrono::duration<double>>(end - start);
		cout << "Find the boiler:\t" << time_span.count() << "\n";
#endif
	
#ifdef GUI
		// Draw boiler
		if(target.delta_x != 0 && target.delta_y != 0){
			cout << target.delta_x << "," << target.delta_y << "\n";
		}

		if(target.delta_x != 0 && target.delta_y != 0){
			glBegin(GL_LINES);
			glColor3f(0.5f, 0.5f, 1.0f);
			int16_t target_x = target.delta_x / 10;
			int16_t target_y = -target.delta_y / 10;
			glVertex2i(target_x+10, target_y);
			glVertex2i(target_x-10, target_y);
			glVertex2i(target_x, target_y+10);
			glVertex2i(target_x, target_y-10);
			glEnd();
		}
		// Draw lines
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.5f, 0.5f);

		if(first_line != NULL){
			doubly_linked_list_node<line> * line;

			bool finished = false;
			line = first_line;
			while(!finished){

				glVertex2i(line->data->start_x/10, -line->data->start_y/10);
				glVertex2i(line->data->end_x/10, -line->data->end_y/10);

				if(line->next == first_line){
					finished = true;
				}
				line = line->next;
			}
		
		}

		glEnd();
#endif
		// Delete lines
		line_list_cleanup(first_line);
	}
}

/**
   Setup for GLFW window
   Window is 640 by 480, with internal space from -640 to 640 and -480 to 480
*/
#ifdef GUI
GLFWwindow * setup_window(){
	glfwInit();

	GLFWwindow * window = glfwCreateWindow(640, 480, "Teensy LIDAR grapher", NULL, NULL);

	if(!window){
		cerr << "Could not create window\n";
		glfwTerminate();
		exit(-1);
	}

	glfwMakeContextCurrent(window);
	glOrtho(-640, 640, -480, 480, -10, 10);

	return window;
}
#endif

/**
   Begin cycle to read data from a teensy
   This runs forever if GUI is enabled
*/
int read_teensy(int argc, char * argv[]){
	int alliance = atoi(argv[2]);
	string port = argv[3];
	int baud = 115200; // LIDAR teensy default baud rate
	if(argc > 3){
		baud = atoi(argv[4]);
	}


	int teensy = open_teensy(port, baud);
	if(teensy == -1){
		cerr << "Could not open Teensy\n";
		return -1;
	}

#ifdef GUI
	GLFWwindow * window = setup_window();

	while(!glfwWindowShouldClose(window)){
#endif
		doubly_linked_list_node<lidar_datapoint> * lidar_data_start = get_lidar_data(teensy);

		if(lidar_data_start != NULL){
#ifdef TIME
			// Time calculate lines data
			chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
#endif

			// Blur points
			blur_points(lidar_data_start);
			add_cartesians(lidar_data_start);
	
#ifdef TIME
			chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();

			chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end - start);
			cout << "Blurring points:\t" << time_span.count() << "\n";
#endif
			if(lidar_data_start != NULL){
				process(lidar_data_start, alliance);
			}
		}


		lidar_datapoint_list_cleanup(lidar_data_start);

#ifdef GUI
		glfwSwapBuffers(window);
		glfwPollEvents();
		usleep(100000);
	}

	glfwTerminate();
#endif
	close_teensy(teensy);
	return 0;
}

/**
   Begin cycle to read data from a file
   This runs forever if GUI is enabled
*/
int read_file(int argc, char * argv[]){
	int alliance = atoi(argv[2]);
	fstream file(argv[3]);

	doubly_linked_list_node<lidar_datapoint> * first_node = NULL;
	doubly_linked_list_node<lidar_datapoint> * previous_node = NULL;

	// Load data from file into circular doubly linked list
	while(!file.eof()){
		string datapoint = "";
		getline(file, datapoint);
		uint8_t mode = 0;
		string idx = "";
		string val = "";
		for(int i = 0; i < datapoint.length(); i++){
			if(datapoint[i] == ','){
				mode = 1;
			}
			else{
				if(mode == 0){
					idx += datapoint[i];
				}
				else {
					val += datapoint[i];
				}
			}
		}
		try{
			if(previous_node == NULL){
				previous_node = new doubly_linked_list_node<lidar_datapoint>;
				previous_node->data = new lidar_datapoint;
				previous_node->data->theta = stoi(idx);
				previous_node->data->radius = stoi(val);
				previous_node->next = NULL;
				previous_node->prev = NULL;
				first_node = previous_node;
			}
			else{
				doubly_linked_list_node<lidar_datapoint> * node = new doubly_linked_list_node<lidar_datapoint>;
				node->data = new lidar_datapoint;
				node->data->theta = stoi(idx);
				node->data->radius = stoi(val);
				node->prev = previous_node;
				previous_node->next = node;
				previous_node = node;
			}
		}
		catch(invalid_argument a){
		}
		catch(out_of_range r){
		}
	}
	
	previous_node->next = first_node;
	first_node->prev = previous_node;

	file.close();
	
#ifdef TIME
	// Time calculate lines data
	chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
#endif

	// Blur points
	blur_points(first_node);
	add_cartesians(first_node);
	
#ifdef TIME
	chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();

	chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end - start);
	cout << "Blurring points:\t" << time_span.count() << "\n";
#endif
	
#ifdef GUI
	GLFWwindow * window = setup_window();

	while(!glfwWindowShouldClose(window)){
#endif
		process(first_node, alliance);

#ifdef GUI
		glfwSwapBuffers(window);
		glfwPollEvents();
		usleep(100000);
	}

	glfwTerminate();
#endif

	lidar_datapoint_list_cleanup(first_node);
	
	return 0;
}

int main(int argc, char * argv[]){
	init_trig();
	if(argc < 3){
		cout << "Usage:\n./graph_lidar [type] [alliance] [serial port | file] [baud rate (optional)]\n";
		return -1;
	}
	string type = argv[1];

	if(type == "dev"){
		return read_teensy(argc, argv);
	}
	else if(type == "file"){
		return read_file(argc, argv);
	}
	
	return 0;
}
