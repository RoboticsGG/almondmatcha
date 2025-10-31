import cv2
from lane_detector_temp import process_frame

frame = cv2.imread("out.jpg")
theta, b, detected = process_frame(frame)

cv2.waitKey(0)
cv2.destroyAllWindows()