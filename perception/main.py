import os
import yaml             # To import configuration file
import csv              # Format of the annotations
import numpy as np
import cv2 as cv

##################################################
# TESTING REKTNET DATASET
##################################################

# FUNCTIONS
##################################################
def to_int_array(string_of_array):
    mid_idx = string_of_array.find(',')
    first = int(string_of_array[1:mid_idx])
    second = int(string_of_array[mid_idx+2:-1])
    return first, second

def annotate_landmarks(img, label):
    for i in range(1, 8):
        str_coords = label[i]
        x, y = to_int_array(str_coords)
        img = cv.circle(img, (x, y), 1, (255, 0, 0))
    return img


# MAIN
##################################################
with open("config.yaml", "r") as yamlfile:
    config = yaml.load(yamlfile, Loader=yaml.FullLoader)
    rektnet_images = os.listdir(config[0]['dataset']['rektnet_images'])
    #print('Rektnet has: ', len(rektnet_images), ' images.')

# Store RektNet labels in numpy array
with open(config[0]['dataset']['rektnet_labels'], 'r') as f:
    rektnet_labels = list(csv.reader(f, delimiter=","))
rektnet_labels = np.array(rektnet_labels[1:])
rektnet_labels = np.delete(rektnet_labels, [1, 9], axis=1)  # We remove 'URL' and empty last column
# rektnet_labels is a numpy array of strings

rand = np.random.randint(0, rektnet_labels.shape[0])

random_label = rektnet_labels[rand][0]
random_img = os.path.join(config[0]['dataset']['rektnet_images'], random_label)
while os.path.isfile(random_img):
    img = cv.imread(random_img)
    img = annotate_landmarks(img, rektnet_labels[rand])
    cv.imshow('Random', img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    rand = np.random.randint(0, rektnet_labels.shape[0])
    random_label = rektnet_labels[rand][0]
    random_img = os.path.join(config[0]['dataset']['rektnet_images'], random_label)

