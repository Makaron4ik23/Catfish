#!/usr/bin/env python3
import cv2
import numpy as np

# Load the captured image
img = cv2.imread('/home/hacaton1/Desktop/Catfish/drone_1_capture.png')
if img is None:
    print("Failed to load image.")
    exit(1)

# Convert to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define a broad green range to find the LED region
# Hue: green is typically 35-85
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

mask = cv2.inRange(hsv, lower_green, upper_green)
green_pixels = cv2.findNonZero(mask)

if green_pixels is not None:
    print(f"Found {len(green_pixels)} green pixels in the mask.")
    
    # Let's extract the actual HSV values of these pixels
    h_vals, s_vals, v_vals = [], [], []
    for pixel in green_pixels:
        x, y = pixel[0]
        h, s, v = hsv[y, x]
        h_vals.append(h)
        s_vals.append(s)
        v_vals.append(v)
        
    print(f"Hue range: {min(h_vals)} - {max(h_vals)}")
    print(f"Saturation range: {min(s_vals)} - {max(s_vals)}")
    print(f"Value range: {min(v_vals)} - {max(v_vals)}")
    
    # Save a debug image with the mask applied
    res = cv2.bitwise_and(img, img, mask=mask)
    cv2.imwrite('/home/hacaton1/Desktop/Catfish/drone_1_mask_debug.png', res)
    print("Saved mask debug image to /home/hacaton1/Desktop/Catfish/drone_1_mask_debug.png")
else:
    print("No green pixels found in the broad range.")
