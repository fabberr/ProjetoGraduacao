#!/usr/bin/env bash

datasets=( $(ls ./datasets) )
detect=( AKAZE SIFT SURF )
for dataset in "${datasets[@]}"
do
	for f2d in "${detect[@]}"
	do
		./exec_pipeline.sh -y -f $f2d $dataset
	done
done
