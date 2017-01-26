const uint16_t interpolate_threshold = 32;

uint8_t current_packet[22];
uint8_t subpacket_idx;
bool start;
uint16_t distances[360];
uint16_t signal_strengths[360];
uint16_t slopes[360]; // Slope 0 is 0-1, slope 1 is 1-2, etc
uint8_t last_idx;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // LIDAR Serial
  delay(1000);
  start = false;
}

void loop() {
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

  if (start == false) { // Wait for full packet
    uint8_t idx = current_packet[1] - 0xA0;
    if (idx != last_idx) {
      bool error = false;
      for (uint8_t i = 0; i < 4; i++) {
        uint8_t data_start = i * 4 + 4;
        uint16_t angle = idx * 4 + i;
        error = (current_packet[data_start + 1] & 0x80) > 0;
        if (!error) {
          uint16_t distance = 0;
          distance = ((current_packet[data_start]) | (current_packet[data_start+1] & 0x0F) << 8);
          distances[angle] = distance;
          uint16_t signal_strength = 0;
          signal_strength = ((current_packet[data_start+2]) | (current_packet[data_start+3]) << 8);
          signal_strengths[angle] = signal_strength;
        }
        else {
          distances[angle] = 0;
        }
      }
      last_idx = idx;
    }
  }
  else if (subpacket_idx == 1) { // This will interpolate a single point provided some points before and after it are valid
    for (uint16_t i = 0; i < 360; i++) {
      if (distances[i] == 0) {
        uint16_t last_distance = distances[(i - 1) % 360];
        uint16_t last_last_distance = distances[(i - 2) % 360];
        uint16_t next_distance = distances[(i + 1) % 360];
        uint16_t next_next_distance = distances[(i + 1) % 360];
        if (last_distance != 0 && last_last_distance != 0 &&  (next_distance != 0 || next_next_distance != 0)) {
          uint16_t slope = last_last_distance - last_distance;
          if (next_distance != 0) {
            if ((last_distance + slope * 2) > next_distance - interpolate_threshold &&
                (last_distance - slope * 2) < next_distance + interpolate_threshold) {
              distances[i] = last_distance + slope;
            }
          }
          else {
            if ((last_distance + slope * 3) > next_next_distance - interpolate_threshold &&
                (last_distance - slope * 3) < next_next_distance + interpolate_threshold) {
              distances[i] = last_distance + slope;
            }
          }
        }
        if (next_distance != 0 && next_next_distance != 0 &&  (last_distance != 0 || last_last_distance != 0)) {
          uint16_t slope = next_distance - next_next_distance;
          if (last_distance != 0) {
            if ((next_distance - slope * 2) < last_distance + interpolate_threshold &&
                (next_distance + slope * 2) > last_distance - interpolate_threshold) {
              distances[i] = next_distance - slope;
            }
          }
          else {
            if ((next_next_distance - slope * 3) < last_distance + interpolate_threshold &&
                (next_next_distance + slope * 3) > last_distance - interpolate_threshold) {
              distances[i] = next_distance - slope;
            }
          }
        }
      }
    }
  }

  if (Serial.available()) {
    if (Serial.read() == '1') {
      for (uint16_t i = 0; i < 360; i++) {
        if (distances[i] != 0) {
          for (uint8_t j = 1; j < 3; j++) {
            if (i < pow(10, j)) Serial.print("0");
          }
          Serial.print(i);
          Serial.print(",");
          for (uint8_t j = 1; j < 4; j++) {
            if (distances[i] < pow(10, j)) Serial.print("0");
          }
          Serial.print(distances[i]);
          Serial.print(",");
          for (uint8_t j = 1; j < 5; j++) {
            if (signal_strengths[i] < pow(10, j)) Serial.print("0");
          }
          Serial.println(signal_strengths[i]);
          delayMicroseconds(2);
        }
      }
      Serial.print("#");
    }
  }
}

