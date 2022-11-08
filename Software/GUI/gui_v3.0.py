import cv2
import numpy as np
import pyautogui as pag
import serial
import math

# Add canvas for demo 1
canvas = np.zeros((800, 1280, 3), np.uint8)
# strate line
cv2.line(canvas, (300, 200), (980, 200), (255, 150, 255), 10)
# curve
for i in range(300, 980):
    cv2.circle(canvas, (i, 600 + int(80 * np.sin((i-300)/100))), 3, (150, 255, 255),3)

# Add canvas for demo 2
canvas2 = np.zeros((800, 1280, 3), np.uint8)
# circle for knob
cv2.circle(canvas2, (640, 400), 200, (150, 255, 255), 10)
# mraker for knob
for i in range(24):
    cv2.line(canvas2, (640 + int(160 * np.sin(np.pi * i / 12)), 400 + int(160 * np.cos(np.pi * i / 12))), (640 + int(180 * np.sin(np.pi * i / 12)), 400 + int(180 * np.cos(np.pi * i / 12))), (255, 150, 255) ,4)

# Add canvas for demo 3
canvas3 = np.zeros((800, 1280, 3), np.uint8)
# circle for knob
cv2.circle(canvas3, (320, 400), 150, (150, 255, 255), 10)
# square
cv2.line(canvas3, (960 - 160, 400 - 160), (960 - 160, 400 + 160),(255, 255, 150) ,4)
cv2.line(canvas3, (960 - 160, 400 - 160), (960 + 160, 400 - 160),(255, 255, 150) ,4)
cv2.line(canvas3, (960 + 160, 400 + 160), (960 - 160, 400 + 160),(255, 255, 150) ,4)
cv2.line(canvas3, (960 + 160, 400 + 160), (960 + 160, 400 - 160),(255, 255, 150) ,4)
# mraker for knob
for i in range(24):
    cv2.line(canvas3, (320 + int(120 * np.sin(np.pi * i / 12)), 400 + int(120 * np.cos(np.pi * i / 12))), (320 + int(135 * np.sin(np.pi * i / 12)), 400 + int(135 * np.cos(np.pi * i / 12))), (255, 150, 255) ,4)

# set the window full screen
out_win = "output_style_full_screen"
cv2.namedWindow(out_win, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(out_win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# open the serial port on Pi
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

# init with demo 0
state = 0

# loop
while True:
    x, y = pag.position()

    # demo 1 logic
    if state == 0:
        img = canvas.copy()
        t = x
        if t < 300:
            t = 300
        elif t > 980:
            t = 980
        cv2.circle(img, (t, 600 + int(80 * np.sin((t-300)/100))), 12, (255, 255, 255), 12)
        cv2.circle(img, (t, 200), 12, (255, 255, 255), 12)
        posStr = 'b' + str(x).rjust(4)+' '+str(y).rjust(4)
        ser.write(bytes(posStr, 'utf-8'))
        cv2.imshow(out_win, img)
        key = cv2.waitKey(1)
        if key == 97:
            state = 0
        elif key == 115:
            state = 1
        elif key == 100:
            state = 2
        elif key == 27:
            break

    # demo 2 logic
    elif state == 1:
        img = canvas2.copy()
        t = math.atan2(400-y, x-640)
        if 0 <= t < np.pi / 2:
            t = np.pi / 2
        elif - np.pi / 2 < t < 0:
            t = - np.pi / 2
        elif t > np.pi / 2:
            t = int(12 * (t + np.pi / 24)  / np.pi) * np.pi / 12
        elif t < - np.pi / 2:
            t = int(12 * (t - np.pi / 24)  / np.pi) * np.pi / 12
        a, b = 640 + int(200 * np.cos(t)), 400 - int(200 * np.sin(t))
        cv2.arrowedLine(img, (640 + int(150 * np.cos(t)), 400 - int(150 * np.sin(t))), (640 + int(190 * np.cos(t)), 400 - int(190 * np.sin(t))), (255, 255, 150) ,6, 0, 0, 0.5)
        posStr = 'k' + str(a).rjust(4)+' '+str(b).rjust(4) + ' 0640 0400'
        ser.write(bytes(posStr, 'utf-8'))
        cv2.imshow(out_win, img)
        key = cv2.waitKey(1)
        if key == 97:
            state = 0
        elif key == 115:
            state = 1
        elif key == 100:
            state = 2
        elif key == 27:
            break
    
    # demo 3 logic
    elif state == 2:
        img = canvas3.copy()s
        t = math.atan2(400-y, x-320)
        if t > 0:
            t = int(12 * (t + np.pi / 24)  / np.pi) * np.pi / 12
        else:
            t = int(12 * (t - np.pi / 24)  / np.pi) * np.pi / 12
        a, b = 320 + int(150 * np.cos(t)), 400 - int(150 * np.sin(t))
        cv2.arrowedLine(img, (320 + int(100 * np.cos(t)), 400 - int(100 * np.sin(t))), (320 + int(140 * np.cos(t)), 400 - int(140 * np.sin(t))), (255, 255, 150) ,6, 0, 0, 0.5)
        c ,d = 0, 0
        if -np.pi / 4 <= t < np.pi / 4:
            c ,d = 1120 , 400 - int(math.tan(t) * 160)
        elif -np.pi * 3 / 4 <= t < -np.pi / 4:
            c ,d = 960 + int(math.tan(t + np.pi / 2) * 160), 560
        elif  np.pi / 4 <= t < np.pi * 3 / 4:
            c, d = 960 - int(math.tan(t - np.pi / 2) * 160) , 240
        else:
            c, d = 800,  400 + int(math.tan(t) * 160)

        cv2.circle(img, (c, d), 12, (255, 255, 255), 12)
        posStr = 'k' + str(a).rjust(4)+' '+str(b).rjust(4) + ' ' +  str(c).rjust(4)+' '+str(d).rjust(4) 
        ser.write(bytes(posStr, 'utf-8'))
        cv2.imshow(out_win, img)
        key = cv2.waitKey(1)
        if key == 97:
            state = 0
        elif key == 115:
            state = 1
        elif key == 100:
            state = 2
        elif key == 27:
            break
