Source: https://github.com/zylo117/Yet-Another-EfficientDet-Pytorch

In order to convert YOLO-DATASET to COCO format to use the above EfficientDet training framework, you will need to do the following:
* Convert annotations to COCO format `.json` files
* Rearrange dataset to COCO structure
* Create a `dataset_name.yml` project file

# Convert annotations

Here you may use the provided script `csv_to_json.py`, specifically designed to handle `.csv` annotation files, structured as:
`Name,URL,Width,Height,Scale,"X0, Y0, H0, W0","X1, Y1, H1, W1",etc,"`

E.g.:
`frame001.jpg,N/A,2048,1536,0.675925926,"[345, 842, 74, 120]","[505, 716, 60, 34]","[326, 698, 49, 26]","[1725, 789, 111, 79]","[1374, 697, 48, 30]","[1096, 658, 47, 41]","[663, 649, 36, 38]","[471, 648, 37, 32]",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,`

Open up the script, and provide the filepaths to both `.csv` files (both train and val) and the filepaths to the files you want to obtain.


# Rearrange the dataset to COCO structure

The COCO dataset followes the below structure:
```
dataset_name/
	annotations/
	train/
	val/
```
You will need to place the `instances_train.json` and `instances_val.json` files which you generated above, in the `annotations/` subdirectory.
Next, you will need to move the images listed in the `instances_train.json` file to `train/` and the images listed in `instances_val.json` to `val/`.
There is a workaround, if you don't wish to move your dataset. You may create a symlink to the dataset. You may execute the below 2 instructions from the `dataset_name/` root dir:
`ln -s /path/to/your/dataset/ train/` and `ln -s /path/to/your/dataset/ val/`
The linked path may be the same, as long as the dataset directory contains both images mentionned in the `instances_train.json` and `instances_val.json` files.

# Create a `dataset_name.yml` project file

In the `projects/` dir, you will need to create a `dataset_name.yml` file. Make a copy of the original `coco.yml` and make sure to at least update the `project_name` and `obj_list` fields.
You may try with the given `mean`, `std`, `anchors_scale` and `anchors_ratios` but depending on how much your bounding boxes differ from `COCO`, you may need to update them.

In order to get the `anchors_scale` and `anchors_ratios` values, you may want to try to use `https://github.com/Cli98/anchor_computation_tool` or `https://github.com/mnslarcher/kmeans-anchors-ratios`
As for the `mean` and `std` values, you can simply run the following code on your dataset:

Here's an example for the CIFAR10 dataset using pytorch:
```
# load the training data
train_data = torchvision.datasets.CIFAR10(CIFAR10_dir, train=True, download=False)
# use np.concatenate to stick all the images together to form a 1600000 X 32 X 3 array
x = np.concatenate([np.asarray(train_data[i][0]) for i in range(len(train_data))])

print(x.shape)
# calculate the mean and std along the (0, 1) axes
train_mean = np.mean(x, axis=(0, 1))
train_std = np.std(x, axis=(0, 1))
# the the mean and std
print('Mean:',train_mean, ' Std:', train_std)
```
An example output might be:
`Printing: Mean: [125.30691805 122.95039414 113.86538318]  Std: [62.99321928 62.08870764 66.70489964]`
Which you might want to scale down, by dividing all numbers by 255.

# Start training
In order to start training, you may run the following command from the EfficientDet root dir (replacing `dataset_name` with your actual dataset name used everywhere above):
`python train.py -c 0 -p dataset_name --head_only True --lr 1e-3 --batch_size 32 --load_weights weights/efficientdet-d0.pth  --num_epochs 50 --save_interval 100
You may tweek all the parameters to suit your dataset and network. 