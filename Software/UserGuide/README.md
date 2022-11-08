# User Guide

## Firmware setup

### STM-32 Program (already flashed v2.0)

1. Install STM-32Cube IDE
2. From GItHub [Link](https://github.com/sjtuyuxuan/Tablet) download project (v3.0)
3. Flash it into the Micro-controller

### Raspberry Pi Gui

1. From GItHub [Link](https://github.com/sjtuyuxuan/Tablet) download (gui.py for v2.0 STM-32 firmware, gui_v3.0.py for v3.0 STM-32 firmware)

### Odrive

1. Firmware is already setup.



## Software Guide

### Raspberry Pi Gui code

1. Run `python3 gui.py ` or `python3 gui_v3.0.py ` in terminal. 
2. Direct test with finger touch on the screen whether the code is working well.

### STM-32 

The `user` button is broken, so only the `reset` button is on the broad. We will use it for later calibration.

### Odrive

Every time we restart the power of Odrive or one motor is out of control because of collision or other reasons, we need to redo the calibration sequence provide by odrive.

Note that the arm will move CCW for about 60-70 degree. PLEASE reserve enough space or the calibration sequence will not run successfully.



## DEMO Guide

Finish testing the Firmware and Software. Start follow the guides below:

1. OPEN the Gui on Raspberry Pi with `Demo 1` scenario **AND touch the mouse(touch point) to the left up corner (from arm perspective)**.

2. Connect the odrive with USB cable to PC and use python interface to do the calibration sequence for two motor. **Repeat for TWO arms (4  calibrate sequence)**

   1. ```python
      import odrive
      from odrive.enums import *
      
      print("finding an odrive...")
      my_drive = odrive.find_any()
      ```

   2. ```python
      my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
      # wait for big-arm calibrate seq
      ```

   3. ```python
      my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
      # wait for fore-arm calibrate seq
      ```

3. Let the left arm bottom magnet outside and right ram bottom magnet embedded.

4. Drag the arm to the calibrate anchor point. 

   1. Left arm : Match the center point for the hole on the frame work and the hole used to insert the magnet. (We need more accurate for active arm so take off the magnet will help enhance the accuracy)
   2. Right arm : Match the center point for the hole on the frame work and the center of button magnet.

5. Push the `Reset` button on STM-32. Wait for 10 second, until the arm move.

6. You will have 10 second to let left arm magnet into the hole

7. Demo is working!

8. Using 'a'/'s'/'d' on keyboard to switch the demo

   1. 'a' for Demo 1
   2. 's' for Demo 2
   3. 'd' for Demo 3 (only support on v3.0)
   4. 'esc' to quit the Demo

Note !!! switch from Demo 2 to Demo 1 need ensure the left arm is on the left or top (if on the buttom will lead to crash ~_~ you may need calibrate it again!)

