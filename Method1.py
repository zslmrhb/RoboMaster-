import cv2
import numpy as np

cap = cv2.VideoCapture('power rune by Raring_Ringtail.mp4')

blue_range = np.array([[0, 0, 130], [90, 205, 205]], dtype='uint8')
size = (7, 7)

element = cv2.getStructuringElement(cv2.MORPH_RECT, size)

while cap.isOpened():
    ret, frame = cap.read()
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    mask_img = cv2.cvtColor(blur, cv2.COLOR_RGB2BGR)
    mask = cv2.inRange(mask_img, blue_range[0], blue_range[1])
    img3 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, element)
    _, contours, _ = cv2.findContours(img3, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    armor_angle = []
    armor_module = []
    for i in range(len(contours)):

        area = cv2.contourArea(contours[i])
        if 1430 <= area <= 1600:
            _, _, armorAngle = cv2.minAreaRect(contours[i])
            armor_angle.append(armorAngle)
            armor_module.append(i)

        elif 180 <= area <= 200:
            pass

        elif 3000 <= area <= 4500:

            _, _, panelAngle = cv2.minAreaRect(contours[i])
            for j in range(len(armor_module)):
                if armor_angle[j] - 10 <= panelAngle <= armor_angle[j] + 10:
                    cv2.drawContours(frame, contours[armor_module[j]], -1, (0, 0, 255), 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()