from keyboard_input import *
from command import *
import time
import json

r = RobotConnection(SERIAL_PORT='COM4')
k = KeyboardController()

continue_loop = True

while continue_loop:
    
    if KeyCode.from_char('q') in k.pressed:
        continue_loop = False

    elif Key.up in k.pressed:
        command = '1 255 255 1 1 1000\n'
        
    elif Key.right in k.pressed:
        command = '1 255 255 1 0 1000\n'
        
    elif Key.left in k.pressed:
        command = '1 255 255 0 1 1000\n'
        
    elif Key.down in k.pressed:
        command = '1 255 255 0 0 1000\n'
        
    elif KeyCode.from_char('a') in k.pressed:
        command = '5 13 0 0 0 0\n'
        
    elif KeyCode.from_char('d') in k.pressed:
        command = '5 28 0 0 0 0\n'
        
    else:
        pass
        

    print("\n\nDATA READ\n")
    data = r.read_all()
    if len(data) > 0:

        print(data[-1])
        val = json.loads(data[-1])["mode"]

    time.sleep(0.2)
    if val == 0:
        r.write(command)
    
print('ended')
k.callback()
r.callback()