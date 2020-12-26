import cv2
import numpy as np
from imutils.video import FPS

cap = cv2.VideoCapture('power_rune.mp4')

blue_range = np.array([[0, 0, 130], [90, 205, 205]], dtype='uint8')
size = (7, 7)
element = cv2.getStructuringElement(cv2.MORPH_RECT, size)

kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kf.processNoiseCov = 1e-8 * np.identity(4, np.float32)
kf.measurementNoiseCov = 1e-6 * np.identity(2, np.float32)
kf.errorCovPost = np.identity(4, np.float32)


def kalmanfilter(armor_x, armor_y):
    measurement = np.array([[np.float32(armor_x)], [np.float32(armor_y)]])
    prediction = kf.predict()
    kf.correct(measurement)
    return prediction


def fastSort(received_contours, received_hierarchy):
    arr = []
    for k in range(len(received_contours)):
        arr.append([received_contours[k], [k, received_hierarchy[0][k][3]]])
    arr.sort(key=lambda area: cv2.contourArea(area[0]))
    return arr


fps = FPS().start()

while cap.isOpened():

    ret, frame = cap.read()
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    mask_img = cv2.cvtColor(blur, cv2.COLOR_RGB2BGR)
    mask = cv2.inRange(mask_img, blue_range[0], blue_range[1])
    morph_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, element)
    _, contours, hierarchy = cv2.findContours(morph_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    sorted_group = fastSort(contours, hierarchy)
    armor_module = []

    for i in range(len(sorted_group)):
        area = cv2.contourArea(sorted_group[i][0])
        if 1430 <= area <= 1600:
            armor_module.append(i)
        elif 3000 <= area <= 4500:
            for j in range(len(armor_module)):
                if sorted_group[armor_module[j]][1][1] == sorted_group[i][1][0]:
                    (x, y), radius = cv2.minEnclosingCircle(sorted_group[armor_module[j]][0])
                    center, radius = (int(x), int(y)), int(radius)
                    result = kalmanfilter(center[0], center[1])
                    frame = cv2.circle(frame, (result[0] + 9 * result[2], result[1] + 9 * result[3]), radius,
                                       (0, 255, 0), 2)
    fps.update()
    fps.stop()

    cv2.putText(frame, "{:.2f}".format(fps.fps()), (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
