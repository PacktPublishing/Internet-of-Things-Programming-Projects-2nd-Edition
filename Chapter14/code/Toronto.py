import cv2 as cv

img = cv.imread('images/Toronto.png')
cv.imshow('Downtown Toronto', img)
cv.waitKey(0)
cv.destroyAllWindows()