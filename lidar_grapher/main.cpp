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
#include <GL/gl.h>
#endif

#include "lidar_teensy.h"
#include "line_find.h"

#define PI 3.14159265

using namespace std;

int16_t plot(vector<point> lidar_data){
#ifdef GUI
	glBegin(GL_POINTS);
#endif
	for(int16_t i = 0; i < lidar_data.size(); i++){
		uint16_t distance = get<1>(lidar_data[i]) / 10;
#ifdef GUI
		glVertex2i(cos((double) get<0>(lidar_data[i]) * PI/180.0f)*distance,
			   -sin((double) get<0>(lidar_data[i]) * PI/180.0f)*distance);
#endif
	}
#ifdef GUI
	glEnd();
#endif
	
	return 0;
}

int draw(vector<point> lidar_data){
#ifdef GUI
	if(lidar_data.size() > 0){
		glClear(GL_COLOR_BUFFER_BIT);
	}
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	glVertex2i(0, 0);
	glEnd();
	
	glColor3f(1.0f, 0.5f, 0.5f);
#endif
#ifdef TIME
	// Time LiDAR data
	chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
#endif
	vector<line> lines =  get_lines(lidar_data);
	lines.shrink_to_fit();
#ifdef TIME
	chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();

	chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end - start);
	cout << time_span.count() << "\n";
#endif

#ifdef GUI
	glColor3f(1.0f, 0.5f, 0.5f);
	glBegin(GL_LINES);
#endif
	for(uint16_t i = 0; i < lines.size(); i++){
		point start_point = get<0>(lines[i]);
		point end_point = get<1>(lines[i]);
#ifdef GUI
		glVertex2i(get<0>(start_point) / 9, -get<1>(start_point) / 9);
		glVertex2i(get<0>(end_point) / 9, -get<1>(end_point) / 9);
#endif
	}
#ifdef GUI
	glEnd();
#endif

#ifdef GUI
	glColor3f(0.5f, 1.0f, 0.5f);
#endif
	plot(lidar_data);

	return 0;
}

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

int read_teensy(int argc, char * argv[]){
	string port = argv[2];
	int baud = 115200;
	if(argc > 3){
		baud = atoi(argv[3]);
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
		vector<point> lidar_data = get_lidar_data(teensy);
		lidar_data.shrink_to_fit();

		draw(lidar_data);

#ifdef GUI
		glfwSwapBuffers(window);
		usleep(100000);
	}

	glfwTerminate();
#endif
	close_teensy(teensy);
}

int read_file(int argc, char * argv[]){
	fstream file(argv[2]);

	vector<point> lidar_data;

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
			lidar_data.push_back(point(stoi(idx), stoi(val)));
		}
		catch(invalid_argument a){
		}
		catch(out_of_range r){
		}
	}
	lidar_data.shrink_to_fit();

	file.close();

#ifdef GUI
	GLFWwindow * window = setup_window();

	while(!glfwWindowShouldClose(window)){
#endif
		draw(lidar_data);

#ifdef GUI
		glfwSwapBuffers(window);
		usleep(100000);
	}

	glfwTerminate();
#endif
}

int main(int argc, char * argv[]){
	if(argc < 3){
		cout << "Usage:\n./graph_lidar [type] [serial port | file] [baud rate (optional)]\n";
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
