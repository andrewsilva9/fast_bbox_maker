import cv2
import numpy as np
# Object ID in yolo data
object_id = 21
# number of pixels to scale bounding box by (left, up, down, and right)
scaler = 3

# Read in image, resize, blur, binarize / threshold, erode, dilate
image_cv = cv2.imread("test.jpg", 0)
image_resize = cv2.resize(image_cv, dsize=(960, 540))
image_blur = cv2.GaussianBlur(image_resize, ksize=(7, 7), sigmaX=3)
ret, thresh = cv2.threshold(image_blur, 127, 255, cv2.THRESH_BINARY)
kernel = np.ones((5, 5), np.uint8)
opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

# Find lowest left corner, lowest top, highest right, and keep making a detected edge the bottom
left = len(opened[0])
top = -1
bottom = 0
right = 0

for index, row in enumerate(opened):
    ones = np.where(row < 125)[0]
    if len(ones) < 1:
        continue
    if top < 0:
        top = index
    if ones[0] < left:
        left = ones[0]
    if ones[len(ones)-1] > right:
        right = ones[len(ones)-1]
    bottom = index


left -= scaler
top -= scaler
bottom += scaler
right += scaler
left /= float(len(image_resize[0]))
top /= float(len(image_resize))
bottom /= float(len(image_resize))
right /= float(len(image_resize[0]))
width = right-left
height = bottom-top
data = np.array([[object_id, left, top, width, height]]).astype(float)
np.savetxt('test.txt', data, fmt="%i %.8f %.8f %.8f %.8f", newline=' ')
# cv2.rectangle(image_resize, (int(left*len(image_resize[0])), int(top*len(image_resize))),
#               (int((left+width)*len(image_resize[0])), int((top+height)*len(image_resize))), (0, 255, 0), thickness=2)
# cv2.imshow("test", image_resize)
# cv2.waitKey(0)
