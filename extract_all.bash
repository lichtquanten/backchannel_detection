#!/usr/bin/env bash
bag_directory=$1
feature_file_directory=$2
log_directory=$3

# Set up log directory
rm -rf $log_directory
mkdir -p $log_directory/out
mkdir -p $log_directory/err

for file in $bag_directory/*; do
	fname=$(basename $file)
	group=${fname%.bag}
	feature_file=$feature_file_directory/$group.csv
	(/home/sean/ws/src/backchannel_detection/extract.bash $file $feature_file 1> $log_directory/out/$group.log 2> $log_directory/err/$group.log && echo "----DONE----" >> /home/sean/.log/out/$group.log) &
done
