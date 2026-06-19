import cv2
import numpy as np

img = cv2.imread('/home/hacaton1/Desktop/Catfish/drone_1_capture.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Find the brightest green pixels
# Let's filter for Hue in [40, 80], Saturation > 180, Value > 200
mask = cv2.inRange(hsv, np.array([40, 180, 200]), np.array([80, 255, 255]))
pts = cv2.findNonZero(mask)

if pts is not None:
    print(f"Found {len(pts)} bright green core pixels.")
    h_vals, s_vals, v_vals = [], [], []
    for pt in pts:
        x, y = pt[0]
        h, s, v = hsv[y, x]
        h_vals.append(h)
        s_vals.append(s)
        v_vals.append(v)
    print(f"Hue: {min(h_vals)} - {max(h_vals)}")
    print(f"Saturation: {min(s_vals)} - {max(s_vals)}")
    print(f"Value: {min(v_vals)} - {max(v_vals)}")
else:
    print("No bright core pixels found with S>180, V>200. Let's loosen limits.")
    mask = cv2.inRange(hsv, np.array([40, 100, 150]), np.array([80, 255, 255]))
    pts = cv2.findNonZero(mask)
    if pts is not None:
        print(f"Found {len(pts)} pixels with S>100, V>150.")
        h_vals, s_vals, v_vals = [], [], []
        for pt in pts:
            x, y = pt[0]
            h, s, v = hsv[y, x]
            h_vals.append(h)
            s_vals.append(s)
            v_vals.append(v)
        print(f"Hue: {min(h_vals)} - {max(h_vals)}")
        print(f"Saturation: {min(s_vals)} - {max(s_vals)}")
        print(f"Value: {min(v_vals)} - {max(v_vals)}")
