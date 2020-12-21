import cv2
import numpy as np

cap = cv2.VideoCapture('power rune by Raring_Ringtail.mp4')

blue_range = np.array([[0, 0, 130], [90, 205, 205]], dtype='uint8')
size = (7, 7)
element = cv2.getStructuringElement(cv2.MORPH_RECT, size)


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


while cap.isOpened():
    ret, frame = cap.read()
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    mask_img = cv2.cvtColor(blur, cv2.COLOR_RGB2BGR)
    mask = cv2.inRange(mask_img, blue_range[0], blue_range[1])
    morph_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, element)
    _, contours, hierarchy = cv2.findContours(morph_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    sorted_group = bubbleSort(contours, hierarchy)
    armor_module = []

    for i in range(len(sorted_group)):

        area = cv2.contourArea(sorted_group[i][0])
        if 1430 <= area <= 1600:

            armor_module.append(i)

            # elif 180 <= area <= 200:

        elif 3000 <= area <= 4500:

            for j in range(len(armor_module)):
                if sorted_group[armor_module[j]][1][1] == sorted_group[i][1][0]:
                    cv2.drawContours(frame, sorted_group[armor_module[j]][0], -1, (0, 0, 255), 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()