#!/bin/bash

# To be run in the root of the kitti dataset directory which should be set up like this:
# kitti_bayesian_segnet/
#    train/
#    trainannot/
#    val/
#    valannot
#
# With val containing all the training images
# and valannot containing all the labeled training images
#
# Then the script will take a random subset (20% of 200 images = 40)
# and move them into the train folders

FILES=$(cat val.txt | shuf)
COUNT=0

for line in $FILES; do
    echo "line: $line"

    if (( COUNT < 80 )); then
	if [[ $line = *"TrainIds"* ]]; then
	    mv "$line" ./trainannot/
	else
	    mv "$line" ./train/
	fi
    	echo $line >> train.txt
    else
    	echo $line >> val-new.txt
    fi
    COUNT=$((COUNT+1))
done

# Put them on the same line
sed 'N;s/\n/ /' train.txt | tee train-new-oneline.txt
sed 'N;s/\n/ /' val-new.txt | tee val-new-oneline.txt

rm train.txt
mv train-new-oneline.txt train.txt
rm val.txt val-new.txt
mv val-new-oneline.txt val.txt
