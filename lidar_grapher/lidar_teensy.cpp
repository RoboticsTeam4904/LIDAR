#include "lidar_teensy.h"

#include <iostream>
#include <cstdlib>
#include <termios.h> // serial settings
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int open_teensy(string port, int baud){
	struct termios serial_settings;

	int teensy = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if( teensy == -1){ // Could not open
		cerr << "Could not open: " << port << "\n";
		return -1;
	}

	if(tcgetattr(teensy, &serial_settings) == -1){ // Could not get serial settings
		cerr << "Could not get serial settings for: " << port << "\n";
		return -1;
	}

	cfsetispeed(&serial_settings, baud);
	cfsetospeed(&serial_settings, baud);

        serial_settings.c_cflag &= ~PARENB;
        serial_settings.c_cflag &= ~CSTOPB;
        serial_settings.c_cflag &= ~CSIZE;
        serial_settings.c_cflag |= CS8;
        serial_settings.c_cflag &= ~CRTSCTS;
        serial_settings.c_cflag |= CREAD | CLOCAL;
        serial_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
        serial_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        serial_settings.c_oflag &= ~OPOST;

        serial_settings.c_cc[VMIN]  = 1;
        serial_settings.c_cc[VTIME] = 20;

	cfmakeraw(&serial_settings);

	tcflush(teensy, TCIFLUSH);
	if(tcsetattr(teensy, 0, &serial_settings) == -1){ // Could not set serial settings
		cerr << "Could not set serial settings for: " << port << "\n";
		return -1;
	}

	return teensy;
}

doubly_linked_list_node<lidar_datapoint> * get_lidar_data(int teensy){
	doubly_linked_list_node<lidar_datapoint> * first_node = NULL;
	doubly_linked_list_node<lidar_datapoint> * previous_node = NULL;

	char trigger[1];
	trigger[0] = '#';
	write(teensy, trigger, 1);

	string dataset = "";
	char response[1];
	do{
		if(read(teensy, response, 1) > 0){
			dataset += response[0];
		}
		else{
			usleep(1);
		}
	} while(response[0] != '#');

	uint8_t mode = 0;
	string idx = "";
	string val = "";
	for(int i = 0; i < dataset.length(); i++){
		if(dataset[i] == '\n'){
			mode = 0;
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
			idx = "";
			val = "";
		}
		else{
			if(dataset[i] == ','){
				mode = 1;
			}
			else if(mode == 0){
				idx += dataset[i];
			}
			else{
				val += dataset[i];
			}
		}
	}

	if(previous_node != NULL && first_node != NULL){
		previous_node->next = first_node;
		first_node->prev = previous_node;
	}

	return first_node;
}

int close_teensy(int teensy){
	close(teensy);
}
