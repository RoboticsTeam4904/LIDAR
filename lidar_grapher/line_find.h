#include <vector>
#include <tuple>

typedef std::tuple<int16_t, int16_t> point;
typedef std::tuple<std::tuple<int16_t, int16_t>, std::tuple<int16_t, int16_t>> line;

std::vector<point> blur_points(std::vector<point> lidar_data);
std::vector<line> get_lines(std::vector<point> lidar_data);
