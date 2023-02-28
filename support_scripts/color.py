from djitellopy import tello
import os
import time

timestr = time.strftime("%d_%m_%y")
print(timestr)

print('\x1b[1;91;104m' + "battery" + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 3..." + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 2..." + '\x1b[0m')
print('\x1b[1;30;41m' + "Starting Mission in: 1..." + '\x1b[0m')
print('\x1b[1;97;41m' + "TAKEOFF!" + '\x1b[0m\n')

print('\x1b[1;30;41m' + 'IN RANGE' + '\x1b[0m')
print('\x1b[1;30;41m' + 'MOVING UP & FORWARD' + '\x1b[0m\n')

print('\x1b[1;97;41m' + 'LANDING!' + '\x1b[0m')
print('\x1b[1;30;43m' + 'Stop video recording' + '\x1b[0m')
print('\x1b[1;30;43m' + 'Shutting down camera streaming' + '\x1b[0m')
print('\x1b[1;30;43m' + 'Mission finished' + '\x1b[0m')
"style;fg;bg"




# path = '/Users/tomerhochman/Desktop/Aruco.tmp/tello_aruco/tello_video/'
# i = 0
# for filename in os.listdir(path):
#     print(filename)
#     os.rename(os.path.join(path, filename), os.path.join(path, 'tello_vid'+str(i)+'.avi'))
#     i = i +1