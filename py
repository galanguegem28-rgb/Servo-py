import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

import cv2
from cvzone.HandTrackingModule import HandDetector
import pyfirmata
import numpy as np
import sys

# ---------------- ARDUINO ----------------
my_port = 'COM5'

try:
    board = pyfirmata.Arduino(my_port)
    iterator = pyfirmata.util.Iterator(board)
    iterator.start()
    pin5 = board.get_pin('d:5:s')
    print(f"Connected to Arduino Nano on {my_port}")
except Exception as e:
    print(f"Arduino Error: {e}")
    sys.exit()

# ---------------- HAND DETECTOR ----------------
detector = HandDetector(detectionCon=0.8, maxHands=1)
cap = cv2.VideoCapture(0)

prev_angle = 90
smooth_factor = 0.25

while True:
    success, img = cap.read()
    if not success:
        break

    # 1. Flip the image for natural interaction
    img = cv2.flip(img, 1) 
    
    # 2. Tell the detector NOT to flip the logic (flipType=False)
    hands, img = detector.findHands(img, flipType=False) 

    if hands:
        hand = hands[0]

        # Measure distance between thumb tip and index tip
        length, info, img = detector.findDistance(
            hand["lmList"][4][:2],
            hand["lmList"][8][:2],
            img
        )

        # Map distance to angle
        raw_angle = np.interp(length, [30, 200], [0, 180])
        raw_angle = np.clip(raw_angle, 0, 180)

        # Smooth servo motion
        angle = prev_angle + (raw_angle - prev_angle) * smooth_factor
        prev_angle = angle

        pin5.write(angle)

        cv2.putText(img, f'Servo: {int(angle)} deg', (50, 50),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

    cv2.imshow("CVZone Hand Control", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
board.exit()
