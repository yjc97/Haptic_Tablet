# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 14:03:56 2022

@author: jeff8
"""

import  os
import  time
import pyautogui as pag
try:
    while True:
        print("Press Ctrl-C to end")
        screenWidth, screenHeight = pag.size()  #獲取螢幕的尺寸
        print(screenWidth,screenHeight)
        x,y = pag.position()   #獲取當前滑鼠的位置
        posStr = "Position:" + str(x).rjust(4)+','+str(y).rjust(4)
        print(posStr)
        time.sleep(0.01)
        os.system('cls')   #清楚螢幕
except KeyboardInterrupt:
    print('end....')