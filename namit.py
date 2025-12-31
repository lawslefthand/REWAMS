import cv2
import time

CAMERA_DEV = "/dev/video0"
WIDTH = 416
HEIGHT = 416

cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

if not cap.isOpened():
    raise RuntimeError("Camera open failed")

print("Camera opened")

start = time.time()
frames = 0

while time.time() - start < 10:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed")
        continue
    frames += 1

cap.release()
print("Frames captured:", frames)
