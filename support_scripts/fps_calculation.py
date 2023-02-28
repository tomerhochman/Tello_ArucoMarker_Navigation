import time
import cv2
from djitellopy import tello

#cap = cv2.VideoCapture(0)
me = tello.Tello()
me.connect()
me.streamon()


num_frame = 240
#
# print("Capturing {0}".format(num_frame))
#
# start = time.time()
#
# for i in range(0, num_frame):
#     print("Frame number {}".format(i))
#     #ret, frame = cap.read()
#     frame = me.get_frame_read().frame
#
#
# end = time.time()
#
# seconds = (end - start)
#
# print("Time taken: {0} seconds".format(seconds))
#
# fps = num_frame / seconds
# print("Estimated frames per second: {0}".format(fps))
# me.streamoff()

start = time.time()
while True:

    frame = me.get_frame_read().frame
    cv2.imshow("f", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):

        for i in range(0, num_frame):
            print("Frame number {}".format(i))
            # ret, frame = cap.read()
            frame = me.get_frame_read().frame

        end = time.time()

        seconds = (end - start)

        print("Time taken: {0} seconds".format(seconds))

        fps = num_frame / seconds
        print("Estimated frames per second: {0}".format(fps))
        break

me.streamoff()
cv2.destroyAllWindows()
cv2.waitKey(1)






