from keyboard_input import *
from command import *
import time
import json
continue_loop = True
c = Commander(serial_port='COM4')
k = KeyboardController()
def end_loop():
    continue_loop = False
#continue_loop = True
k.add(Key.up, lambda: c.add_command(Direct(255, 255, 1, 1, 300)))
k.add(Key.left, lambda: c.add_command(Direct(255, 255, 0, 1, 300)))
k.add(Key.right, lambda: c.add_command(Direct(255, 255, 1, 0, 300)))
k.add(Key.down, lambda: c.add_command(Direct(255, 255, 0, 0, 300)))
k.add(KeyCode.from_char('s'), lambda: c.add_command(Stop()))
k.add(KeyCode.from_char('q'),lambda: end_loop)
while continue_loop:
    
    pass
    
print('ended')
k.callback()
c.callback()