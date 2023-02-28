import time
import cv2
from threading import Thread
from djitellopy import tello

keep_recording = True

# for Mac camera
# cap = cv2.VideoCapture(0)

# for Tello drone
me = tello.Tello()
me.connect()
me.streamon()
print(me.get_battery())
frame_read = me.get_frame_read()


def video_record():

    global resize
    # For Mac camera
    # ret_, frame = cap.read()
    # h, w = frame.shape[:2]

    # For Tello drone
    h, w, _ = frame_read.frame.shape
    new_h = int(h / 2)
    new_w = int(w / 2)
    resize = cv2.resize(frame_read.frame, (new_w, new_h))
    #(h, w) = frame.shape[:2]

    video = cv2.VideoWriter('thread_test9.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 24.0, (new_w, new_h))

    while keep_recording:

        print("VideoWriting :: Frame")
        video.write(resize)
        time.sleep(1 / 24)


recorder = [Thread(target=video_record)]
recorder[-1].daemon = True
recorder[-1].start()


while True:

    # For Mac camera
    # ret, frame = cap.read()
    # W, H, _ = frame.shape
    #frame_read = me.get_frame_read()
    # For Tello drone
    #frame = me.get_frame_read().frame
    H, W, _ = frame_read.frame.shape
    new_H = int(H / 2)
    new_W = int(W / 2)
    resize = cv2.resize(frame_read.frame, (new_W, new_H))
    # cv2.putText(frame, "Test", (int(cap.get(3)/2), int(cap.get(4)/2)-400), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)
    # cv2.circle(frame, (int(cap.get(3)/2), int(cap.get(4)/2)), 100, (0, 2500, 0), 2, cv2.LINE_AA)

    cv2.putText(resize, "Test", (int(new_W / 2), int(new_H / 2)), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.circle(resize, (int(new_W / 2), int(new_H / 2)), 100, (0, 2500, 0), 2, cv2.LINE_AA)

    cv2.imshow("tello", resize)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        keep_recording = False
        break

# for Mac camera
# cap.release()

# For Tello
me.streamoff()
cv2.destroyAllWindows()
cv2.waitKey(1)
print("video released")

for t in recorder:
    t.join()
