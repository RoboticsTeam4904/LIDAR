#include "line_find.h"
#include "boiler_find.h"
#include "doubly_linked_list.h"
#include "point_preprocess.h"
#include "datatypes.h"

// Packet loading data
uint8_t current_packet[22];
uint8_t subpacket_idx;
bool start;
uint8_t last_idx;

// Current data
uint16_t distances[360];
doubly_linked_list_node<lidar_datapoint> * lidar_data_start;
doubly_linked_list_node<line> * line_data_start;
boiler_location boiler;

uint8_t calculation_idx;

void setup(){
	Serial.begin(115200);
	Serial1.begin(115200);
	delay(1000); // wait for serial to load
	start = false;
	lidar_data_start = NULL;
	line_data_start = NULL;
	calculation_idx = 0;
}

void try_load_next_byte();
void packet_to_array();
void load_linked_list();

void loop(){
	try_load_next_byte();

	if(!start){
		packet_to_array();
	}

	if(last_idx == 0xFA){
		calculation_idx = 1; // Start calculation
	}
	
	if(calculation_idx == 1){
		interpolate(distances);
		calculation_idx++;
	}
	else if(calculation_idx == 2){
		load_linked_list();
		calculation_idx++;
	}
	else if(calculation_idx == 3){
		blur_points(lidar_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 4){
		blur_points(lidar_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 5){
		blur_points(lidar_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 6){
		add_cartesians(lidar_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 7){
		line_data_start = get_lines(lidar_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 8){
		boiler = get_boiler(line_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 9){
		line_list_cleanup(line_data_start);
		calculation_idx++;
	}
	else if(calculation_idx == 10){
		lidar_datapoint_list_cleanup(lidar_data_start);
		calculation_idx = 1;
	}
}

/**
   Attempt to load the next byte from the LIDAR Serial
   into the packet array
 */
void try_load_next_byte(){
	if(Serial1.available()){
		uint8_t b = Serial1.read();
		if(b == 0xFA && !start){
			subpacket_idx = 0;
			memset(current_packet, 0, 22);
			current_packet[0] = 0xFA;
			start = true;
		}
		else if(start){
			subpacket_idx++;
			current_packet[subpacket_idx] = b;
			if(subpacket_idx == 21){
				start = false;
			}
		}
	}
}

/**
   Update the distance array with the latest packet
 */
void packet_to_array(){
	uint8_t idx = current_packet[1] - 0xA0;
	if(idx != last_idx){
		bool error = false;
		for(uint8_t i = 0; i < 4; i++){
			uint8_t data_start = i * 4 + 4;
			uint16_t angle = idx * 4 + i;
			error = (current_packet[data_start + 1] & 0x80) > 0;
			if(!error){
				uint16_t distance = 0;
				distance = ((current_packet[data_start]) | (current_packet[data_start + 1] & 0x0F) << 8);
				distances[angle] = distance;
			}
			else{
				distances[angle] = 0;
			}
		}
		last_idx = idx;
	}
}

/**
   Transfer the distance array into 
 */
void load_linked_list(){
	doubly_linked_list_node<lidar_datapoint> * previous_node = NULL;
	for(int i = 0; i < 360; i++){
		if(distances[i] != 0){
			if(previous_node == NULL){
				previous_node = new doubly_linked_list_node<lidar_datapoint>;
				previous_node->data = new lidar_datapoint;
				previous_node->data->theta = i;
				previous_node->data->radius = distances[i];
				previous_node->next = NULL;
				previous_node->prev = NULL;
				lidar_data_start = previous_node;
			}
			else{
					doubly_linked_list_node<lidar_datapoint> * node = new doubly_linked_list_node<lidar_datapoint>;
					node->data = new lidar_datapoint;
					node->data->theta = i;
					node->data->radius = distances[i];
					node->prev = previous_node;
					previous_node->next = node;
					previous_node = node;
			}
		}
	}
}
