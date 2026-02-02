# ---------------- FIX FOR PYTHON 3.11 + PYFIRMATA ----------------
import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

# ---------------- IMPORTS ----------------
import cv2
from cvzone.HandTrackingModule import HandDetector
import pyfirmata
import numpy as np
import sys
import time

# ---------------- ARDUINO SETUP ----------------
my_port = 'COM6'   # Change if your port is different

try:
    board = pyfirmata.Arduino(my_port)
    time.sleep(2)  # ⭐ Allow Arduino to reset

    iterator = pyfirmata.util.Iterator(board)
    iterator.start()

    pin5 = board.get_pin('d:5:s')  # Servo on D5
    pin5.write(90)  # Start at center position
    time.sleep(1)

    print(f"Connected to Arduino on {my_port}")

except Exception as e:
    print(f"Arduino Error: {e}")
    sys.exit()

# ---------------- HAND DETECTOR ----------------
detector = HandDetector(detectionCon=0.8, maxHands=1)
cap = cv2.VideoCapture(0)

prev_angle = 90
smooth_factor = 0.25

# ---------------- MAIN LOOP ----------------
while True:
    success, img = cap.read()
    if not success:
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)

    if hands:
        hand = hands[0]

        length, info, img = detector.findDistance(
            hand["lmList"][4][:2],
            hand["lmList"][8][:2],
            img
        )

        raw_angle = np.interp(length, [30, 200], [0, 180])
        raw_angle = np.clip(raw_angle, 0, 180)

        angle = prev_angle + (raw_angle - prev_angle) * smooth_factor

        # Only send update if angle changed enough (prevents serial overload)
        if abs(angle - prev_angle) > 1:
            pin5.write(angle)

        prev_angle = angle

        cv2.putText(img, f'Servo: {int(angle)} deg', (50, 50),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

    cv2.imshow("CVZone Hand Control", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------------- CLEANUP ----------------
cap.release()
cv2.destroyAllWindows()
board.exit()
print("Program closed cleanly")
