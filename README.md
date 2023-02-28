# Tello_ArucoMarker_Navigation
This repository contain scripts to control DJI Tello using Aruco Markers.

1. tello_bigger_frame:  This is the main script for operating the drone(including PID controler for yaw_velocity), using maximum frame size.

2. tello_smaller_frame: This is the secondary script for operating the drone, using minimum frame size for better streaming quality.

3. tello_backup & tello_draft & tello_current: Those scripts are early and more simple implentation of controling the drone. all 3 scripts
                                               are for backup only.

4. The support_scripts folder conatain 3 scripits: color.py for printing options and formating, fps_calculation.py for testing the fps input from a given                                                      camera, tello_videoTheread.py for testing the video thereading.
