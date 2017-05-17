# Canny edge detection from image
#
# This program uses edge detection to convert the image to a more basic form
#
# Author: Pearse Swail
# Last Modified: April 17, 2017
#
#------------------------------------------------------------------------------#

import numpy as np
import cv2

img = cv2.imread('smile.png',0)
edges = cv2.Canny(img,100,200)

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

