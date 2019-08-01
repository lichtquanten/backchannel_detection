#!/usr/bin/env bash

if  ! [[ $# -eq 2 ]]
then
  echo "2 arguments required, $# provided."
  echo "./test.bash SOURCE.bag OUT.csv"
  return 1
fi

af_tmp=$(mktemp /tmp/af.XXXXXX)
nf_tmp=$(mktemp /tmp/nf.XXXXXX)
bundle_tmp=$(mktemp /tmp/bundle.XXXXXX)

rosrun audio_features features.py \
  _audio_topic:=/mic/data \
  _source_bag_path:=$1 \
  _sink_bag_path:=$af_tmp

rosrun audio_features features.py \
  _audio_topic:=/mic/data \
  _features_topic:=/bc/nod_features \
  _source_bag_path:=$1 \
  _sink_bag_path:=$nf_tmp

rosrun bundler bundle.py \
  _audio_bag_path:=$af_tmp \
  _nod_bag_path:=$nf_tmp \
  _start_time_bag_path:=$1 \
  _window_duration_bag_path:=$1 \
  _sink_bag_path:=$bundle_tmp

rosrun model tocsv.py \
  _source_bag_path:=$bundle_tmp \
  _csv_path:=$2

rm $af_tmp $nf_tmp $bundle_tmp
