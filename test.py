from keyboard_input import *
from connect import *
import time

r = RobotConnection(SERIAL_PORT='COM6')
k = KeyboardController()

continue_loop = True

while continue_loop:
    
    if 'q' in k.pressed:
        continue_loop = False
    elif Key.up in k.pressed:
        r.write('6 255 255 1 1\n')
    elif Key.right in k.pressed:
        r.write('6 255 255 1 0\n')
    elif Key.left in k.pressed:
        r.write('6 255 255 0 1\n')
    elif Key.down in k.pressed:
        r.write('6 255 255 0 0\n')
    else:
        r.write('6 0 0 1 1\n')

    time.sleep(0.2)
    

k.callback()
r.callback()