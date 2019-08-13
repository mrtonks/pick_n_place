# pick_n_place
MSc Summer Project 


First, do Calibration 
- Start Baxter and untuck arms
- Obtain the four corners coordinates from the table wrt Baxter
    - Move right arm to right corners of the table and for each corner, take the pose position of the arm using rostopic echo -n 1 /robot/limb/right/endpoint_state 
    - Update values from OBJECT_POINTS in helpers/const.py file accordingly (only x and y)
    - Repeat the same for the left side of the table with the left arm
- Obtain the pixel coordinates from the four corners of the table
    - Make sure the zed camera is connected and you have the zed_wrapper for python installed. Then run in a terminal "roslaunch zed_wrapper zed.launch" 
    - Make sure to uncomment lines 149-150 from object_detection and save
    - Run "python3 object_detection.py" file only to obtain an image from the table
    - Update values from IMAGE_POINTS in helpers/const.py file accordingly (x, y)


