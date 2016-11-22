#include <vector>
#include <tuple>

int open_teensy(std::string port, int baud = 115200);
std::vector<std::tuple<int16_t, int16_t>> get_lidar_data(int teensy);
int close_teensy(int teensy);
