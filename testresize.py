import cv2
import numpy as np

import os


wd = os.getcwd()
for img in os.listdir(os.path.join(wd, 'JPEGImages/')):
    if img.startswith('.'):
        continue
    debug = cv2.imread(os.path.join(wd, 'JPEGImages', img))
    image_blur = cv2.GaussianBlur(debug, ksize=(7, 7), sigmaX=3)
    ret, thresh = cv2.threshold(image_blur, 127, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)


    # Find lowest left corner, lowest top, highest right, and keep making a detected edge the bottom
    left = len(opened[0])
    top = -1
    bottom = 0
    right = 0
    scaling_factor = 2
    for index, row in enumerate(opened):
        ones = np.where(row < 125)[0]
        if len(ones) < 1:
            continue
        if top < 0:
            top = index
        if ones[0] < left:
            left = ones[0]
        if ones[len(ones) - 1] > right:
            right = ones[len(ones) - 1]
        bottom = index

    print left, top, right, bottom
    left = max(0, left)
    top = max(0, top)
    bottom = min(bottom, len(debug))
    right = min(right, len(debug[100]))

    mybox = cv2.imread(os.path.join(wd, 'JPEGImages', img))
    mybox = mybox[int(top):int(bottom), int(left):int(right)]
    blank = cv2.imread('blank_background.jpg')
    width = len(mybox[0])/scaling_factor
    height = len(mybox)/scaling_factor
    new_top = np.random.randint(0, len(blank) - height)
    new_left = np.random.randin(0, len(blank[0]) - width)

    blank[new_top:new_top+height, new_left:new_left+width, :] = mybox[::scaling_factor, ::scaling_factor, :]
    file_name = img.split('.')[0]
    image_file_path = os.path.join(wd, 'scaled_imgs', file_name+'scaledby'+str(scaling_factor)+'.jpg')
    text_file_path = os.path.join(wd, 'scaled_labels', file_name+'scaledby'+str(scaling_factor)+'.txt')
    cv2.imwrite(image_file_path, blank)

    if 'block' in file_name:
        obj_num = 16
    elif 'screw' in file_name:
        obj_num = 15
    elif 'beam' in file_name:
        obj_num = 17
    data = np.array([obj_num, new_left, new_top, new_left+width, new_top+height])
    outfile = open(text_file_path, 'w')
    np.savetxt(outfile, data, fmt="%i %.8f %.8f %.8f %.8f", newline=' ')

