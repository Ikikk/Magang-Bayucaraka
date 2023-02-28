import cv2
import numpy as np

import os
os.chdir("finalproject/src")

def scan():
    # read img
    img = cv2.imread('shape1.jpeg')

    # resize img
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))

    # convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # detecting edges
    edges = cv2.Canny(gray, 170, 255)

    # take areas with more intensity
    ret,thresh = cv2.threshold(gray,240,255,cv2.THRESH_BINARY)

    # get all contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # for each contour
    for contour in contours:
        # get length from each contour
        peri = cv2.arcLength(contour, True)
        # get how many dots
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

    # draw contour
    cv2.drawContours(img, [contour], -1, (0,0,0), 2)

    # add label
    img = cv2.putText(img, '{}'.format(len(approx)), (0,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 1, cv2.LINE_AA)
    num = len(approx)
    # print(num)
    return num

# if __name__ == '__main__' :
#     scan()