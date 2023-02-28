import math
import cv2
import time
from cv2 import aruco
import numpy as np
from djitellopy import tello


# Slope calculation
def slope(x1, y1, x2, y2):
    """
    :param x1:  point1 x value
    :type  x1:  int
    :param y1:  point1 y value
    :type  y1:  int
    :param x2:  point2 x value
    :type  x2:  int
    :param y2:  point2 y value
    :type  y2:  int
    :return:    slope
    :rtype:     float
    """

    try:
        output = (y2-y1)/(x2-x1)
    except ZeroDivisionError:
        output = 0

    return output


# Angle calculation
def angle(m1, m2):
    """
    :param m1: slope of 1st line
    :type  m1: float
    :param m2: slope of 2nd line
    :type  m2: float
    :return:   angle in degrees
    :rtype:    float
    """

    pi = 3.14159265

    # Store the tan value of the angle
    try:
        angle_ = abs(m2 - m1) / (1 + m1 * m2)
    except ZeroDivisionError:
        angle_ = 0

    # Calculate the inverse of the angle
    ret_ = math.atan(angle_)

    # Convert the angle from radians to degree
    val = (ret_ * 180) / pi

    val = round(val, 4)

    return val


# Camera calibration parameters
# MultiMatrix3 = for smaller frame size 480 x 360
# MultiMatrix2 = for original frame size 960 × 720
# MultiMatrix1 = for original frame size 960 × 720
calib_data_path = "/Users/tomerhochman/Desktop/Aruco.tmp/calib_data/MultiMatrix2.npz"
calib_data = np.load(calib_data_path)
# print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# Aruco markers parameters and configuration
MARKER_SIZE = 8  # in CM
cur_id = 0  # Initiate a start marker, current ID.
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
param_markers = aruco.DetectorParameters_create()

# Video recording parameters
fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
out = cv2.VideoWriter('/Users/tomerhochman/Desktop/Aruco.tmp/tello_aruco/tello_video/tello_vid29.avi',
                      fourcc, 30, (960, 720))

# Tello initialization
me = tello.Tello()
me.connect()
print('\x1b[1;91;104m' + "Battery level:" + str(me.get_battery()) + "%" + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 3" + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 2" + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 1" + '\x1b[0m')
me.streamon()
print('\x1b[1;97;41m' + "TAKEOFF!" + '\x1b[0m')
me.takeoff()
me.send_rc_control(0, 0, 0, 0)
time.sleep(1)

# Main loop: while detect marker --> send commands to drone based on the marker position
while True:

    """
    Main loop: tello navigation based on marker detection in real-time.
    1. Read the frame from tello stream.
    2. Apply aruco marker detection.
    3. Estimate pose of Aruco.
    4. Iterate over the Aruco's detected.
    5. Draw box around marker, calculate center of marker, angle, slope, distance to marker.
    6. Draw lines from camera to aruco center.
    7. Add text to frame, include all tello state data.
    8. Control the drone movement based on: x,y,distance.  
    """

    frame = me.get_frame_read().frame

    # get height and width of the frame
    (h, w) = frame.shape[:2]

    # convert to gray scale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # detect aruco marker
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:

        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for corners in marker_corners:
            for ids in marker_IDs:
                for i in total_markers:

                    # draw lines around marker/draw box - in yellow
                    cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
                    # calculate area of aruco marker
                    area = cv2.contourArea(corners)

                    # extract the coordinates(x, y) of each aruco corner(4 in total)
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()

                    # calculate center of aruco marker
                    _centerY = int((corners[0][1] + corners[2][1]) / 2)
                    _centerX = int((corners[0][0] + corners[2][0]) / 2)
                    center = (_centerX, _centerY)
                    # print("Center", center)

                    # calculate x,y,z coordinates of marker
                    x = round(tVec[i][0][0], 1)
                    y = round(tVec[i][0][1], 1)
                    z = round(tVec[i][0][2], 1)

                    # TODO: try to replace int(w / 2), int(h) with tVec[i][0][0], tVec[i][0][1]

                    # Calculate slope
                    slope1 = slope(_centerX, _centerY, int(w / 2), int(h / 2))  # from aruco center to camera center
                    slope2 = slope(_centerX, _centerY, int(w / 2), int(h))  # from aruco center to frame center

                    # Calculate angle
                    angle_btw_line = angle(slope1, slope2)

                    # draw circle in middle of the aruco marker
                    cv2.circle(frame, (_centerX, _centerY), 10, (250, 45, 208), cv2.FILLED)
                    # draw line from image center(drone camera center) to center of marker
                    cv2.line(frame, (int(w / 2), int(h / 2)), (_centerX, _centerY), (0, 255, 255), 3)
                    # draw line from bottom middle of the frame to center of marker
                    cv2.line(frame, (int(w / 2), int(h)), (_centerX, _centerY), (153, 255, 255), 3)

                    # calculate distance from marker to drone camera
                    distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)

                    # draw frame axis of marker(green red blue axis)
                    point = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

                    # Shows id number and distance
                    cv2.putText(frame, f"id: {ids[0]}  Dist: {round(distance, 2)}", top_right,
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 102, 255), 2, cv2.LINE_AA)

                    # Shows x,y,z
                    cv2.putText(frame, f"x: {round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} "
                                       f"z:{round(tVec[i][0][2], 1)}", (30, 110), cv2.FONT_HERSHEY_PLAIN,
                                       1.3, (0, 255, 255), 2, cv2.LINE_AA)

                    # Shows battery level
                    cv2.putText(frame, f"Battery level: {me.get_battery()}", (30, 50),
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)

                    # Shows angle
                    cv2.putText(frame, f"Angle : {angle_btw_line}", (30, 30),
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 102, 102), 2, cv2.LINE_AA)

                    # Shows TOF
                    cv2.putText(frame, f"TOF: {me.get_distance_tof()}", (30, 70),
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 128, 255), 2, cv2.LINE_AA)

                    # Shows height
                    cv2.putText(frame, f"Height: {me.get_height()}", (30, 90),
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (153, 153, 255), 2, cv2.LINE_AA)

                    # Shows distance
                    cv2.putText(frame, f"Distance: {round(distance, 2)}", (30, 130),
                                cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 102, 255), 2, cv2.LINE_AA)

                    # initiate flight parameters for the drone:
                    # tof= height from ground, fb= forward/backward,
                    # lr= left/right, ud= up/down

                    tof = me.get_distance_tof()
                    fb = 0
                    lr = 0
                    ud = 0
                    yaw = 0

                    # Command loop starting here,
                    # depends on the marker id- the drone will take action

                    ##############################

                    # left_right_velocity	int
                    # -100~100 (left/right)
                    #
                    # forward_backward_velocity	int
                    # -100~100 (forward/backward)
                    #
                    # up_down_velocity	int
                    # -100~100 (up/down)
                    #
                    # yaw_velocity	int
                    # -100~100 (yaw)

                    ##############################

                    if ids[0] == cur_id:

                        x_in_range = False
                        y_in_range = False
                        z_in_range = False

                        # LEFT/RIGHT motion
                        if x < -5 or x > 5:
                            lr = x
                        else:
                            x_in_range = True

                        # UP/DOWN motion
                        if y < -10 or y > 10:
                            ud = (y / 2) * (-1)
                        else:
                            y_in_range = True

                        # FORWARD/BACKWARD motion
                        if distance < 75 or distance > 100:
                            fb = ((100 - distance) * (-1)) / 4
                        else:
                            z_in_range = True

                        # Check if Tello is in range on all axes
                        # If true = fly up 15cm and fly forward 80cm
                        # This part makes sure that we are centered on the marker
                        # And only if we are centered, then we can move to the next marker
                        if x_in_range and y_in_range and z_in_range:
                            print('\x1b[1;30;41m' + f'IN RANGE MARKER {ids[0]}' + '\x1b[0m')
                            cur_id += 1
                            print('\x1b[1;30;41m' + 'MOVING UP & FORWARD' + '\x1b[0m')
                            me.send_rc_control(0, 130, 40, 0)
                            time.sleep(0.5)
                            print('\x1b[1;30;41m' + 'WAITING FOR NEXT MARKER' + '\x1b[0m')
                            me.send_rc_control(0, 0, 0, 0)

                        me.send_rc_control(int(lr), int(fb), int(ud), 0)

                        print("lr={}, ud={}, fb={}, yaw={}".format(lr, ud, fb, yaw))
                        print("Battery: {}".format(me.get_battery()))
                        print("Distance: {}cm".format(distance))
                        print("(x={}, y={})".format(x, y))
                        print("---------------------\n---------------------")

    # Write the video file
    out.write(frame)
    # Show the real-time action
    cv2.imshow("Tello-View", frame)

    keys = cv2.waitKey(1) & 0xFF
    if keys == ord('q'):
        print('\x1b[1;35;102m' + 'LANDING!' + '\x1b[0m')
        me.land()
        break


print('\x1b[1;30;43m' + 'Stop video recording' + '\x1b[0m')
out.release()
print('\x1b[1;30;43m' + 'Shutting down camera streaming' + '\x1b[0m')
me.streamoff()
print('\x1b[1;30;43m' + 'Mission finished' + '\x1b[0m')
cv2.destroyAllWindows()
cv2.waitKey(1)


########################################################################################
