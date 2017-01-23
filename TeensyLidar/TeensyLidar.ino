#include <FlexCAN.h>
#include <TeensyCANBase.h>

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
uint16_t lidarSpeed;
doubly_linked_list_node<lidar_datapoint> * lidar_data_start;
doubly_linked_list_node<line> * line_data_start;
boiler_location boiler;

uint8_t calculation_idx;

int sendLidarEncoder(byte * msg, byte * resp);

void setup() {
  CAN_begin();
  CAN_add_id(0x601, &sendLidarEncoder);
  CAN_add_id(0x602, &sendLidar);
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000); // wait for serial to load
  init_trig();
  start = false;
  lidar_data_start = NULL;
  line_data_start = NULL;
  calculation_idx = 0;
  lidarSpeed = 0;
  boiler.delta_x = 100;
  boiler.delta_y = 150;
}

void try_load_next_byte();
void packet_to_array();
void load_linked_list();

void loop() {
  CAN_update();
  try_load_next_byte();

  if (!start) {
    packet_to_array();
  }

  if (last_idx == 0x59) {
    if (calculation_idx == 0) {
      calculation_idx = 1; // Start calculation
    }
  }

  if (calculation_idx == 1) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 1");
#endif
    interpolate(distances);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 2) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 2");
#endif
    load_linked_list();
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    if (lidar_data_start == NULL) {
      calculation_idx = 0;
    }
    else {
      calculation_idx++;
    }
    calculation_idx = 10;
  }
  else if (calculation_idx == 3) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 3");
#endif
    blur_points(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 4) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 4");
#endif
    blur_points(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 5) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 5");
#endif
    blur_points(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 6) {
#ifdef TIME
    Serial.println("calculation 6");
    long timing_start = micros();
#endif
    add_cartesians(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 7) {
#ifdef TIME
    Serial.println("calculation 7");
    long timing_start = micros();
#endif
    line_data_start = get_lines(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 8) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 8");
#endif
    boiler = get_boiler(line_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 9) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 9");
#endif
    line_list_cleanup(line_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx++;
  }
  else if (calculation_idx == 10) {
#ifdef TIME
    long timing_start = micros();
    Serial.println("calculation 10");
#endif
    lidar_datapoint_list_cleanup(lidar_data_start);
#ifdef TIME
    Serial.print("calculation finished: ");
    Serial.println(micros() - timing_start);
#endif
    calculation_idx = 1;
  }

  // Logging
  if (Serial.available()) {
    if (Serial.read() == '1') {
      doubly_linked_list_node<lidar_datapoint> * node = lidar_data_start->next;
      bool finished = false;

      while (node != lidar_data_start && !finished) {
        for (uint8_t j = 1; j < 3; j++) {
          if (node->data->theta < pow(10, j)) Serial.print("0");
        }
        Serial.print(node->data->theta);
        Serial.print(",");
        for (uint8_t j = 1; j < 4; j++) {
          if (node->data->radius < pow(10, j)) Serial.print("0");
        }
        Serial.println(node->data->radius);
        delayMicroseconds(2);
        if (node == lidar_data_start->prev) {
          finished = true;
        }
        node = node->next;
      }

      Serial.print("#");
    }
  }
}

/**
   Attempt to load the next byte from the LIDAR Serial
   into the packet array
*/
void try_load_next_byte() {
  if (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (b == 0xFA && !start) {
      subpacket_idx = 0;
      memset(current_packet, 0, 22);
      current_packet[0] = 0xFA;
      start = true;
    }
    else if (start) {
      subpacket_idx++;
      current_packet[subpacket_idx] = b;
      if (subpacket_idx == 21) {
        start = false;
      }
    }
  }
}

/**
   Update the distance array with the latest packet
*/
void packet_to_array() {
  uint8_t idx = current_packet[1] - 0xA0;
  if (idx != last_idx) {
    bool error = false;
    lidarSpeed = current_packet[2] + (current_packet[3] << 8);
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t data_start = i * 4 + 4;
      uint16_t angle = idx * 4 + i;
      error = (current_packet[data_start + 1] & 0x80) > 0;
      if (!error) {
        uint16_t distance = 0;
        distance = ((current_packet[data_start]) | (current_packet[data_start + 1] & 0x0F) << 8);
        distances[angle] = distance;
      }
      else {
        distances[angle] = 0;
      }
    }
    last_idx = idx;
  }
}

/**
   Transfer the distance array into
*/
void load_linked_list() {
  doubly_linked_list_node<lidar_datapoint> * previous_node = NULL;
  for (int i = 0; i < 360; i++) {
    if (distances[i] != 0) {
      if (previous_node == NULL) {
        previous_node = new doubly_linked_list_node<lidar_datapoint>;
        previous_node->data = new lidar_datapoint;
        previous_node->data->theta = i;
        previous_node->data->radius = distances[i];
        previous_node->next = NULL;
        previous_node->prev = NULL;
        lidar_data_start = previous_node;
      }
      else {
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

  if (previous_node != NULL) {
    previous_node->next = lidar_data_start;
    lidar_data_start->prev = previous_node;
  }
}

int sendLidar(byte* msg, byte* resp) {

  resp[0] = boiler.delta_x & 0xff;
  resp[1] = (boiler.delta_x >> 8) & 0xff;
  resp[2] = boiler.delta_y & 0xff;
  resp[3] = (boiler.delta_y >> 8) & 0xff;

  for (int i = 4; i < 8; i++) {
    resp[i] = 0;
  }

  Serial.println(boiler.delta_x);
  
  return 1;
}

int sendLidarEncoder(byte * msg, byte * resp) {
  resp[0] = lidarSpeed & 0xff;
  resp[1] = (lidarSpeed >> 8) & 0xff;
  resp[2] = (lidarSpeed >> 16) & 0xff;
  resp[3] = (lidarSpeed >> 24) & 0xff;

  for (int i = 4; i < 8; i++) {
    resp[i] = 0;
  }

  return 1;
}

