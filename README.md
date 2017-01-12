LIDAR related code for 2017.

TeensyLidarTest is a small program for the Teensy 3.x that reads LIDAR data from the first (non-USB) serial port and outputs csv-style data when a "#" is sent over USB.

lidar_grapher is a small program for a computer that reads LIDAR data from a Teensy running TeensyLidarTest or a file and displays it. It is also used for debugging LIDAR processing algorithms.