#!/usr/bin/python

#
# KITTI Segmentation Resizing Script
# 
# Takes every image and sets it to a size of 352 x 1024 to match the cityscapes data
#


import os
from PIL import Image

#IMAGES_DIR='../../../datasets/kitti/segmentation/training/semantic/'
#OUTPUT_DIR='../../../datasets/kitti_bayesian_segnet/trainannot/'
IMAGES_DIR='../data_semantics/training/semantic/'
OUTPUT_DIR='../data_semantics/training/semantic_cropped/'

NEW_WIDTH=1024
NEW_HEIGHT=352

def resize_image(image_path):
    """Output a resized image of 352 x 1024"""
    try:
        image = Image.open(image_path, 'r')
        width, height = image.size
        print(image_path)
        print(width, height)

        assert(width > NEW_WIDTH)
        assert(height > NEW_HEIGHT)

        left = (width - NEW_WIDTH) / 2
        top = (height - NEW_HEIGHT) / 2
        right = (width + NEW_WIDTH) / 2
        bottom = (height + NEW_HEIGHT) / 2

        output = image.crop((left, top, right, bottom))

        path, image_name = os.path.split(image_path)
        print(image_path, path, image_name)
        output_file = OUTPUT_DIR + image_name
        
        print("Saving...", output_file)
        output.save(output_file, "PNG")

    except:
        print("Couldn't process image: ", image_path)


if __name__ == "__main__":
    images = []
    for file in os.listdir(IMAGES_DIR):
        images.append(IMAGES_DIR + file)

    for image_file in images:
        resize_image(image_file)
