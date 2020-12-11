import os
import yaml             # To import configuration file
import csv              # Format of the annotations
import numpy as np
import cv2 as cv

from RektNetLib import *

##################################################
# TESTING REKTNET DATASET
##################################################
# Explanation:  this script will randomly select an image from the RektNet dataset and display it
#               together with the cone landmarks (7 key points of the cone) with small blue dots.
# Purpose:      demonstrate how to access images, labels and overlay them.
# Usage:        run this script and press 'q' to randomly select another image.


# MAIN
##################################################
with open("config.yaml", "r") as yamlfile:
    config = yaml.load(yamlfile, Loader=yaml.FullLoader)

    rektnetlabels = get_rektnet_annotations(config[0]['dataset']['rektnet_labels'])

    rand = np.random.randint(0, rektnetlabels.shape[0])

    random_img_name = rektnetlabels[rand][0]
    random_img = os.path.join(config[0]['dataset']['rektnet_images'], random_img_name)
    while os.path.isfile(random_img):
        img = cv.imread(random_img)
        img = add_landmarks(img, rektnetlabels[rand])
        cv.imshow('Random', img)
        cv.waitKey(0)
        cv.destroyAllWindows()

        rand = np.random.randint(0, rektnetlabels.shape[0])
        random_img_name = rektnetlabels[rand][0]
        random_img = os.path.join(config[0]['dataset']['rektnet_images'], random_img_name)

