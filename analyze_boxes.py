import cv2
import numpy as np

img = cv2.imread("latest_result.jpg")
if img is None:
    print("No latest_result.jpg")
    exit()

HSV_LOWER  = np.array([0,   30,  80],  dtype=np.uint8)
HSV_UPPER  = np.array([20, 255, 255],  dtype=np.uint8)
MORPH_K    = 7

blurred = cv2.GaussianBlur(img, (5, 5), 0)
hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
mask    = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_K, MORPH_K))
mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"Total contours: {len(contours)}")
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 100:
        print(f"Area: {area}")
