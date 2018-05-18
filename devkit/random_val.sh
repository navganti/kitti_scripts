#!/bin/bash

# To be run in the root of the kitti dataset directory which should be set up like this:
# kitti_bayesian_segnet/
#    train/
#    trainannot/
#    val/
#    valannot
#
# With train containing all the training images
# and trainannot containing all the labeled training images
#
# Then the script will take a random subset (20% of 200 images = 40)
# and move them into the validation folder

cd train
FILES=$(ls . | shuf -n 40)
for f in $FILES; do
    mv "$f" ../val/
done

cd ../trainannot
for f in $FILES; do
    mv "$f" ../valannot/
done

