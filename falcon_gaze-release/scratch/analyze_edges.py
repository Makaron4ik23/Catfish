import cv2
import numpy as np

img = cv2.imread('/home/hacaton1/Desktop/Catfish/drone_1_capture.png')
if img is None:
    print("Failed to load image.")
    exit(1)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blurred, 50, 150)

H, W = img.shape[:2]
CX, CY = W / 2, H / 2
h_crop, w_crop = 160, 160
cy_start, cy_end = int(CY - h_crop/2), int(CY + h_crop/2)
cx_start, cx_end = int(CX - w_crop/2), int(CX + w_crop/2)

center_edges = edges[cy_start:cy_end, cx_start:cx_end]
edge_density = np.sum(center_edges > 0) / (h_crop * w_crop)
print(f"Image dimensions: {W}x{H}")
print(f"Edge density in central {w_crop}x{h_crop} area: {edge_density:.4f}")
