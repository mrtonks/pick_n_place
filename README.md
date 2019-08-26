# Pick and Place with Baxter
MSc Summer Project focused on training an neural network to help Baxter on recognizing and picking up objects.

## Overview
This project was developed using the libraries from ROS (kinetic version), an implementation of MaskRCNN<sup>[1]</sup>, Baxter Robot Research and a ZED camera. This repository contains the code to implement a neural network for object recognition in Baxter and execute a "pick and place".

## Code
### src/
This directory contains the code relevant for object recognition and moving Baxter. The results can be seen in this [video](
https://drive.google.com/open?id=1-OxU4u6b8uU-HMpl7ZBVf1AdtI8kVSbB).

**Main files**

`object_recognition.py`: Subscribes to Zed camera feed only once and transforms the image to the format needed. It uses Pillow library to decode the image into RGBA formatm which it's converted to RGB. It will then load the pre-trained model and run it on "inference" mode using MaskRCNN. It will then publish the results containing the name, coordinates and orientation of the objects found using a messenger library called ZeroMQ.

`pick_and_place.py`: Runs in a separate terminal. It subscribes to the Object Detection feed using ZeroMQ. It will also subsribe to the Zed camera feed only once after receiving the message from the Object Detection. It uses OpenCV to collect the depth image. It will process the coordinates from all objects and use them to select the closest object using the depth image. Finally, it will initiate the planning node to start the pick and place.

### src/helpers
This directory contains code to help the main processes during object recognition or pick and place.

**Files**

`const.py`: Constants are stored in this file. Calibration constants can be found in this file. 

`find_contour_angle.py`: Uses OpenCV to find the contours from the mask received and to fit an ellipse within the contour with the biggest area. It processes the angle to return the right angle for Baxter's gripper.

`move_arm.py`: Receives the point where the object is and the angle transformed into quaternions (necessary for Baxter). It commands Baxter to move the right arm, pick and place the object.

`solve_perspective.py`: Uses OpenCV to get the perspective transform matrix than can be used to transform a point from an image into real world coordinates. It requires 4 points from an image (x, y) in pixels and the same 4 points from the real world (x, y). The axes z is not required, since it's known already.

### tests/
This directory contains test files used for learning. It also contains a Jupyter Notebook that was used to train the MaskRCNN. 

**Jupyter Notebook**

`train_mask_rcnn.ipynb`: Uses a COCO dataset that must have the images, masks, definitions file and information file. Runs the training with the provided dataset and the inference with the test images to detect objects. Inference can also be run on a video. 

### mrcnn/
Contains the files necessary for MaskRCNN.

### model/
Folder to place the pre-trained model.

# Getting started
## Requirements

1. Clone this repository
2. Install dependencies
    ```bash
   pip3 install -r requirements-pip.txt
   ```
5. Ensure that you have previously installed ROS Kinetic and have a running environment
    - http://wiki.ros.org/kinetic/Installation
    - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
4. Follow the steps for calibration

## Calibration 
- Start Baxter and untuck arms
- Obtain the four corners coordinates from the table wrt Baxter.
    1. Move right arm to right corners of the table and for each corner, take the pose position of the arm using "rostopic echo -n 1 /robot/limb/right/endpoint_state".
    
    ![Example a](https://github.com/mrtonks/pick_n_place/blob/master/tests/images_calibration/corner_a.jpg)
    ![Example b](https://github.com/mrtonks/pick_n_place/blob/master/tests/images_calibration/corner_b.jpg)
    
    2. Update values from OBJECT_POINTS in helpers/const.py file accordingly (only x and y).
    3. Repeat the same for the left side of the table with the left arm.
- Obtain the pixels coordinates from the four table's corners.
    1. Make sure the ZED camera is connected and you have the ZED SDK and zed_wrapper for python installed.
        - https://www.stereolabs.com/developers/release/
        - https://github.com/stereolabs/zed-python-api 
    2. Publish the ZED node in a terminal 
    ```bash
    roslaunch zed_wrapper zed.launch
    ```
    3. Make sure to uncomment lines 155-156 from `object_detection.py` and save it (visualize.display_instances).
    4. Obtain an image from the table
        ```bash
        python3 object_detection.py
        ```
        - Before running this, download the checkpoint model and move it to the model folder: 
            - https://drive.google.com/open?id=1FZuncg8CphmovfLQxOcAdDZXfBmU0tUR
            - https://www.dropbox.com/s/ll28by2lbbocagq/mask_rcnn_cocosynth_dataset_0300.h5?dl=0 (alternative)
        - Make sure that you already fullfil these requirements in order to use MaskRCNN:
            - https://github.com/matterport/Mask_RCNN#requirements
        - Install any other required library.
    5. Update values from IMAGE_POINTS in helpers/const.py file accordingly (x, y)

## Running the application
There are two scripts that must be executed for the application to start:
- object_detection.py
- pick_and_place.py

Steps for running the application:
1. Open three terminals and execute any necessary commands to run **baxter.sh** as you would normally do.
2. In one terminal, have ZED wrapper node running
    ```bash
    roslaunch zed_wrapper zed.launch
    ```
3. In another terminal, locate the pick and place python file and run it
    ```bash
    python pick_and_place.py
    ```
4. In the last terminal, locate the object detection python file and run it 
    ```bash
    python3 object_detection.py
    ```
5. To stop any of the scripts you need to use ctrl-c and/or ctrl-z.

**Note**: `pick_and_place.py` runs on python 2.7, but `object_detection.py` must run on python 3.5. 

## References:
**[1]** Abdulla, Waleed. "Mask R-CNN for object detection and instance segmentation on Keras and TensorFlow". _Github_.  [https://github.com/matterport/Mask_RCNN](https://github.com/matterport/Mask_RCNN)
