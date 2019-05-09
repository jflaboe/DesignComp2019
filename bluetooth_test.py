from connect import *
import time

bt = RobotConnection(SERIAL_PORT = 'COM4')

bt.read_all()

count = 0
start = time.time()
last_print = start
while time.time() - start < 60:
    if time.time() - last_print > 1:
        last_print = time.time()
        print(count)
        count = 0
    count += len(bt.read_all()) * 3

bt.stop_connection()