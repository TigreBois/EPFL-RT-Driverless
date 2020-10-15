import numpy as np
import cv2 as cv
import csv


def get_rektnet_annotations(csv_file):
    """Retrieve a numpy array with the RektNet annotations

    :param csv_file: the path to the csv file with the RektNet annotations.
    :type csv_file: string

    Returns:
        A numpy string array of: Image Name, top, mid_L_top, mid_R_top,
                                 mid_L_bot, mid_R_bot, bot_L, bot_R

    """
    with open(csv_file, 'r') as labels_file:
        rektnet_labels = list(csv.reader(labels_file, delimiter=","))
        rektnet_labels = np.array(rektnet_labels[1:])
        rektnet_labels = np.delete(rektnet_labels, [1, 9], axis=1)  # We remove 'URL' and empty last column
        return rektnet_labels  # rektnet_labels is a numpy array of strings


def to_int_tuple(string_of_array):
    """Convert a string of int tuple to a tuple of int

    :param string_of_array: a string of an int tuple in the format "[X, Y]".
    :type string_of_array: string

    Returns:
        The converted int tuple.

    """
    mid_idx = string_of_array.find(',')
    first = int(string_of_array[1:mid_idx])
    second = int(string_of_array[mid_idx + 2:-1])
    return first, second


def add_landmarks(img, annotations):
    """Add landmarks to an image

    :param img: the image which will be annotated
    :type img: opencv image
    :param annotations: the 7 landmark from the provided RektNet annotation
    :type annotations: numpy array

    Returns:
        An image copy of the provided image with an overlay of the provided annotations

    """
    for i in range(1, 8):
        str_coords = annotations[i]
        x, y = to_int_tuple(str_coords)
        img = cv.circle(img, (x, y), 1, (255, 0, 0))
    return img




