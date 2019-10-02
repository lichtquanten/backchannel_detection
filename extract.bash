#!/usr/bin/env bash

if  ! [[ $# -eq 2 ]]
then
  echo "2 arguments required, $# provided."
  echo "./test.bash SOURCE.bag OUT.csv"
  exit 1
fi

audio_features_P1_tmp=$(mktemp /tmp/af1.XXXXXX)
audio_features_P2_tmp=$(mktemp /tmp/af2.XXXXXX)
audio_features_P3_tmp=$(mktemp /tmp/af3.XXXXXX)
bundle_tmp=$(mktemp /tmp/bundle.XXXXXX)
WINDOW_DURATION=0.1

rosrun audio_features features.py \
  _audio_topic:=/pid1/audio/data \
  _features_topic:=/pid1/audio/features \
  _window_duration:=$WINDOW_DURATION \
  _source_bag_path:=$1 \
  _sink_bag_path:=$audio_features_P1_tmp

rosrun audio_features features.py \
  _audio_topic:=/pid2/audio/data \
  _features_topic:=/pid2/audio/features \
  _window_duration:=$WINDOW_DURATION \
  _source_bag_path:=$1 \
  _sink_bag_path:=$audio_features_P2_tmp

rosrun audio_features features.py \
  _audio_topic:=/pid3/audio/data \
  _features_topic:=/pid3/audio/features \
  _window_duration:=$WINDOW_DURATION \
  _source_bag_path:=$1 \
  _sink_bag_path:=$audio_features_P3_tmp


rosrun bundler bundle.py \
  _audio_features_P1_bag_path:=$audio_features_P1_tmp \
  _audio_features_P2_bag_path:=$audio_features_P2_tmp \
  _audio_features_P3_bag_path:=$audio_features_P3_tmp \
  _audio_features_P1_topic:=/pid1/audio/features \
  _audio_features_P2_topic:=/pid2/audio/features \
  _audio_features_P3_topic:=/pid3/audio/features \
  _start_time_bag_path:=$1 \
  _window_duration:=$WINDOW_DURATION \
  _sink_bag_path:=$bundle_tmp

rosrun model tocsv.py \
  _source_bag_path:=$bundle_tmp \
  _csv_path:=$2

# rm $audio_features_P1_tmp $audio_features_P2_tmp $audio_features_P3_tmp $bundle_tmp

