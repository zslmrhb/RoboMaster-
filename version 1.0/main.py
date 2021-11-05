import cv2
import numpy as np
from imutils.video import FPS
import time

cap = cv2.VideoCapture('power_rune.mp4')

# Params needed for preprocessing
blue_range = np.array([[0, 0, 130], [90, 205, 205]], dtype='uint8')
size = (7, 7)
element = cv2.getStructuringElement(cv2.MORPH_RECT, size)

# Kalman Filter Parameters
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kf.processNoiseCov = 1e-8 * np.identity(4, np.float32)
kf.measurementNoiseCov = 1e-6 * np.identity(2, np.float32)
kf.errorCovPost = np.identity(4, np.float32)

# Kalman Filter will make prediction of the shooting coordinate
def KalmanFilter(armor_x, armor_y):
    measurement = np.array([[np.float32(armor_x)], [np.float32(armor_y)]])
    prediction = kf.predict()
    kf.correct(measurement)
    return prediction


# Crop the frame using template matching on the center "R" of the power rune and we will get our area of interest (ROI)
def cropSize(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    templateGray = cv2.imread("template.jpg")
    templateGray = cv2.cvtColor(templateGray, cv2.COLOR_BGR2GRAY)
    w, h = templateGray.shape[::-1]
    # _, imgBinary = cv2.threshold(imgGray, 150, 255, cv2.THRESH_BINARY)
    imgGaus = cv2.adaptiveThreshold(imgGray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, -10)
    radiusRatio = 8
    found = None

    # Multi-scaling template matching
    for scale in np.linspace(0.4, 1, 5)[::-1]:
        rW = int(scale * w)
        rH = int(scale * h)
        # Resize template
        resized = cv2.resize(templateGray, (rW, rH), interpolation=cv2.INTER_AREA)
        # cv2.imshow("Resize", resized)

        # Match templates
        res = cv2.matchTemplate(imgGaus, resized, cv2.TM_CCOEFF_NORMED)
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(res)

        # Save max values
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, scale)

    start = (found[1][0], found[1][1])
    shape = (int(found[2] * w), int(found[2] * h))
    end = (start[0] + shape[0], start[1] + shape[1])

    # Circle calculations
    center = (start[0] + int(0.5 * shape[0]), start[1] + int(0.5 * shape[1]))
    radius = int(radiusRatio * shape[0])

    return [(center[1] - radius), (center[1] + radius), (center[0] - radius), (center[0] + radius)]

# Sort the contours and their respective hierarchy so the one with the smallest area will appear first to
# avoid glitching (when the not desired contour appear first we will lost track of the desired contour for one frame)
def bubbleSort(received_contours, received_hierarchy):
    arr = []
    for k in range(len(received_contours)):
        arr.append([received_contours[k], [k, received_hierarchy[0][k][3]]])

    n = len(arr)
    for i in range(n - 1):

        for j in range(0, n - i - 1):
            if cv2.contourArea(arr[j][0]) > cv2.contourArea(arr[j + 1][0]):
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
    return arr


# fps = FPS().start()

output = cropSize(cap.read()[1])

fps = FPS().start()

while cap.isOpened():
    ret, frame = cap.read()
    frame = frame[output[0]:output[1], output[2]:output[3]]
    contour_frame = frame.copy()
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    mask_img = cv2.cvtColor(blur, cv2.COLOR_RGB2BGR)
    mask = cv2.inRange(mask_img, blue_range[0], blue_range[1])
    morph_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, element)

    contours, hierarchy = cv2.findContours(morph_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        sorted_group = bubbleSort(contours, hierarchy)
        armor_module = []

        for i in range(len(sorted_group)):

            area = cv2.contourArea(sorted_group[i][0])

            # the area is hard-coded, can be more flexible, below is the area of the contour in the video
            if 1430 <= area <= 1600:
                armor_module.append(i)

            elif 3000 <= area <= 4500:
                for j in range(len(armor_module)):
                    if sorted_group[armor_module[j]][1][1] == sorted_group[i][1][0]:
                        cv2.drawContours(contour_frame, sorted_group[armor_module[j]][0], -1, (0, 0, 255), 3)
                        (x, y), radius = cv2.minEnclosingCircle(sorted_group[armor_module[j]][0])
                        center, radius = (int(x), int(y)), int(radius)
                result = KalmanFilter(center[0], center[1])
                cv2.circle(frame, (int(result[0] + 15 * result[2]), int(result[1] + 15 * result[3])), radius,
                                   (0, 255, 0), 2)
    fps.update()
    fps.stop()

    cv2.putText(frame, "{:.2f}".format(fps.fps()), (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
    cv2.imshow("Mask", mask)
    cv2.imshow("Morph", morph_img)
    cv2.imshow("Contour", contour_frame)
    cv2.imshow("Predict", frame)
    #
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    cv2.waitKey(0)
# cv2.waitkey(0)
cap.release()
cv2.destroyAllWindows()
