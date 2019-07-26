#!/usr/bin/python3

import pyzed.sl as sl

def main():
    # Create a camera object
    zed = sl.Camera()

    # Create a InitParameters object and set confifuration parameteres
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
    init_params.camera_fps = 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Capture 1 frame and stop
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameteres()

    # Grab an image, a RuntimeParameters object must be given to grab()
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.VIEW_LEFT)
        # Get the timestamp at the time the image was captured
        timestamp = zed.get_timestamp(sl.TIME_REFERENCE.TIME_REFERENCE_CURRENT) 
        print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(), 
            timestamp))
    
    zed.close()

if __name__ == "__main__":
    main()
