import csv
import numpy as np
import cv2 as cv


def get_yolo_annotations(csv_filepath):
    """Retrieve a numpy array with the Yolo annotations

    :param csv_filepath: the path to the csv file with the Yolo annotations.
    :type csv_filepath: string

    Returns:
        A numpy string array of: Image Name, Width, Height, Scale, \
                                 "[X0, Y0, H0, W0]","[X1, Y1, H1, W1]",etc,"

    """
    with open(csv_filepath, 'r') as labels_file:
        yolo_labels = list(csv.reader(labels_file, delimiter=","))
        yolo_labels = np.array(yolo_labels[1:])
        yolo_labels = np.delete(yolo_labels, 0, axis=0)  # Removing CSV header
        yolo_labels = np.delete(yolo_labels, 1, axis=1)  # Removing 'URL' column which is unused
        return yolo_labels


def to_four_ints(string_of_array):
    """Convert a string of four ints to four ints

    :param string_of_array: a string of four ints in the format "[X, Y, H, W]".
    :type string_of_array: string

    Returns:
        The converted four ints X, Y, H, W if existing, else None

    """
    if len(string_of_array) > 0:
        comma1 = string_of_array.find(',')
        comma2 = string_of_array.find(',', comma1+1)
        comma3 = string_of_array.find(',', comma2+1)
        if comma1 != -1 and comma2 != -1 and comma3 != -1:
            X = int(string_of_array[1:comma1])
            Y = int(string_of_array[comma1+1:comma2])
            H = int(string_of_array[comma2+1:comma3])
            W = int(string_of_array[comma3+1:-1])
            return X, Y, H, W
        else:
            return None
    else:
        return None


def add_bounding_boxes(img, annotations):
    """Add bounding boxes to an image

    :param img: the image which will be annotated
    :type img: opencv image
    :param annotations: the yolo annotation array
    :type annotations: list

    Returns:
        An image copy of the provided image with the bounding boxes

    """
    start = 4  # index at which bounding boxes are listed
    i = 0
    while len(annotations[start + i]) > 0:
        x, y, h, w = to_four_ints(annotations[start + i])
        cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        i += 1
    return img
