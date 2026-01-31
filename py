import inspect
# Critical fix for pyfirmata on Python 3.11+
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

import cv2
from cvzone.HandTrackingModule import HandDetector
import pyfirmata
import numpy as np
import sys

# --- Arduino Nano Setup ---
my_port = 'COM5' 
try:
    board = pyfirmata.Arduino(my_port)
    iter8 = pyfirmata.util.Iterator(board)
    iter8.start()
    pin5 = board.get_pin('d:5:s') # Servo on Digital Pin 5
    print(f"Connected to Nano on {my_port}")
except Exception as e:
    print(f"Arduino Error: {e}")
    sys.exit()

# --- Detector Setup ---
# detectionCon: 0.8 (80% confidence), maxHands: 1
detector = HandDetector(detectionCon=0.8, maxHands=1)

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    if not success: break
    
    img = cv2.flip(img, 1)
    # Find the hand and landmarks
    hands, img = detector.findHands(img) 

    if hands:
        hand = hands[0]
        # Landmark 4 (Thumb Tip) and 8 (Index Tip)
        # findDistance calculates the distance and draws the line for you
        length, info, img = detector.findDistance(hand["lmList"][4][:2], hand["lmList"][8][:2], img)
        
        # Map the length (distance) to servo angle (0-180)
        # Adjust 30 and 200 based on your hand distance from camera
        angle = int(np.interp(length, [30, 200], [0, 180]))
        
        # Write to Arduino
        pin5.write(angle)
        
        # Display Angle on screen
        cv2.putText(img, f'Servo: {angle} deg', (50, 50), 
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

    cv2.imshow("CVZone Hand Control", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
board.exit()
