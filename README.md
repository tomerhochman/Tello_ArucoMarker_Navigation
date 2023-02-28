# Tello_ArucoMarker_Navigation
This repository contain scripts to control DJI Tello using Aruco Markers.
The main idea is to navigate the Tello based on Aruco marker detection. for any given marker id we can set specific command to tell the Tello what to do.
On top of the above mentiond, we also want to center the Tello in front of the marker, this will help us to minimize the natural drifting of the Tello drone.
The mission steps for drone are: 
  - Takeoff.
  - Stabilze befor continue.
  - Open camera feed.
  - Look for the first marker.
  - If marker detected: then center in front of the marker within the range we set.
  - If in range: continue to the next marker, and repeat steps 5 & 6 until the final marker is detected.
  - If the final marker detected: landing.
  - Close camera feed and exit the program.
  
###############################################################################

1. tello_bigger_frame:  This is the main script for operating the drone(including PID controler for yaw_velocity), using maximum frame size.

2. tello_smaller_frame: This is the secondary script for operating the drone, using minimum frame size for better streaming quality.

3. tello_backup & tello_draft & tello_current: Those scripts are early and more simple implentation of controling the drone. all 3 scripts
                                               are for backup only.

4. The support_scripts folder conatain 3 scripits: color.py for printing options and formating, fps_calculation.py for testing the fps input from a given                                                      camera, tello_videoTheread.py for testing the video thereading.


## Detailed description on how to run the scripts(functions, parameters etc..) are available inside the scripts!    
