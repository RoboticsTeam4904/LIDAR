#!/bin/bash

for file in $(ls ../lidar_dump/); do for i in {0..10000}; do ./lidar_grapher file ../lidar_dump/$file >> $file.time.out; done; done
for file in $(ls *.out); do echo $file; cat $file | grep "Blurring points" | awk '{SUM += $3} END {print 40*SUM/NR}'; cat $file | grep "Calculate lines" | awk '{SUM += $3} END {print 40*SUM/NR}'; cat $file | grep "Find the boiler" | awk '{SUM += $4} END {print 40*SUM/NR}'; done
for file in $(ls ../lidar_dump/); do rm $file.time.out; done

