import csv
import numpy as np
import json

# TRAIN
CSV_TRAIN_FILEPATH = '/idiap/user/aunnervik/work_dir/fs/testing/train.csv'
JSON_TRAIN_FILEPATH = '/idiap/user/aunnervik/work_dir/fs/our_project/Yet-Another-EfficientDet-Pytorch-master/Yet-Another-EfficientDet-Pytorch-master/datasets/yolo_cones/annotations/instances_train.json'

# VALIDATE
CSV_VAL_FILEPATH = '/idiap/user/aunnervik/work_dir/fs/testing/validate.csv'
JSON_VAL_FILEPATH = '/idiap/user/aunnervik/work_dir/fs/our_project/Yet-Another-EfficientDet-Pytorch-master/Yet-Another-EfficientDet-Pytorch-master/datasets/yolo_cones/annotations/instances_val.json'


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

def line2items(np_item, image_id, label_id, images, annotations):
    image_item = {}
    image_item["id"] = int(image_id)
    #image_item["license"] = 0
    #image_item["coco_url"] = "https://random_coco_url.com"
    #image_item["flickr_url"] = "https://random_flickr_url.com"
    image_item["width"] = int(np_item[2])
    image_item["height"] = int(np_item[3])
    image_item["file_name"] = np_item[0]
    #image_item["date_captured"] = "1970-00-00 00:00:00"

    images.append(image_item)

    start = 5  # index at which bounding boxes are listed
    i = 0

    while i <= (np_item.shape[0]-start-1) and len(np_item[start + i]) > 0 :
        x, y, h, w = to_four_ints(np_item[start + i])
        annotation_item = {}
        annotation_item["id"] = label_id
        annotation_item["category_id"] = 0
        annotation_item["iscrowd"] = 0
        #annotation_item["segmentation"] = [[]]
        annotation_item["image_id"] = int(image_id)
        #annotation_item["area"] = 0
        annotation_item["bbox"] = [x, y, w, h] # [X Y W H]

        annotations.append(annotation_item)

        label_id += 1
        i += 1

def csv2coco(csv_filepath, json_filepath):

    with open (csv_filepath, 'r') as csv_file:

        csv_labels = list(csv.reader(csv_file, delimiter=","))
        csv_labels = np.array(csv_labels[2:])
        print('CSV File nb_items x len_item:', csv_labels.shape)

        print("First item:\n", csv_labels[0])
        print("Last item:\n", csv_labels[-1])

        complete_annotations = {}

        ######################################
        # INFO
        ######################################
        info = {}
        info["description"] = "Cones Dataset"
        info["url"] = ""
        info["version"] = "0.1"
        info["year"] = "2020"
        info["contributor"] = "Alex"
        info["date_created"] = "2020/11/25"

        complete_annotations["info"] = info

        """
        ######################################
        # LICENSES
        ######################################
        licenses = []

        license_item = {}
        license_item["url"] = 
        license_item["id"] = 
        license_item["name"] = 

        licenses.append(license_item)

        complete_annotations["licenses"] = licenses
        """

        ######################################
        # IMAGES & ANNOTATIONS
        ######################################
        images = []
        annotations = []

        img_idx = 0
        object_idx = 0

        for i in range(csv_labels.shape[0]):
            print("CSV LINE:\t", i)
            line2items(csv_labels[i], img_idx+i, object_idx, images, annotations)

        complete_annotations["images"] = images
        complete_annotations["annotations"] = annotations

        ######################################
        # CATEGORIES
        ######################################
        categories = []

        category_item = {}
        category_item["supercategory"] = "road_infrastructure"
        category_item["id"] = 0
        category_item["name"] = "cone" 

        # Potentially more category_items...

        categories.append(category_item)
        
        complete_annotations["categories"] = categories
    
        with open(json_filepath, "w") as json_file:
            json.dump(complete_annotations, json_file, indent=4)

    return complete_annotations

_ = csv2coco(CSV_TRAIN_FILEPATH, JSON_TRAIN_FILEPATH)
_ = csv2coco(CSV_VAL_FILEPATH, JSON_VAL_FILEPATH)