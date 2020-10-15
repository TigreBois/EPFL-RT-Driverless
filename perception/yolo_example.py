import yaml
import os

from YoloLib import *


##################################################
# TESTING REKTNET DATASET
##################################################
# Explanation:  this script will randomly select an image from the Yolo dataset and display it
#               together with the cone bounding boxes. There is no other information than bounding boxes.
# Purpose:      demonstrate how to access images, labels and overlay them.
# Usage:        run this script and press 'q' to randomly select another image.


with open("config.yaml", "r") as yamlfile:
    config = yaml.load(yamlfile, Loader=yaml.FullLoader)

    yolo_labels = get_yolo_annotations(config[0]['dataset']['yolo_labels'])

    rand = np.random.randint(0, yolo_labels.shape[0])
    img_name = yolo_labels[rand][0]
    random_img = os.path.join(config[0]['dataset']['yolo_images'], img_name)

    while os.path.isfile(random_img):
        img = cv.imread(random_img)

        img = add_bounding_boxes(img, yolo_labels[rand])

        h0 = img.shape[0]
        w0 = img.shape[1]
        aspect_ratio = w0 / h0

        new_w = 800
        new_h = int(new_w / aspect_ratio)

        dim = (new_w, new_h)
        img = cv.resize(img, dim, interpolation=cv.INTER_AREA)

        cv.imshow('Random', img)
        cv.waitKey(0)
        cv.destroyAllWindows()

        rand = np.random.randint(0, yolo_labels.shape[0])
        img_name = yolo_labels[rand][0]
        random_img = os.path.join(config[0]['dataset']['yolo_images'], img_name)

