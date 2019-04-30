from dc_control_panel import TimeSeries, ControlPanel
import math
import time
import random
from connect import RobotConnection
from keyboard_input import KeyboardController

sonar = TimeSeries('sonar', 50)
keys = KeyboardController()
robot = RobotConnection('COM6')

def forward():
    robot.write("0,1\n")

def backward():
    robot.write("0,-1\n")

def stop():
    robot.write("0,0\n")

keys.add("w",forward)
keys.add("s", backward)
keys.add("q", stop)

cp = ControlPanel(ts = [sonar])
cp.start()

while True:
    data = robot.read_one()
    if not data is None:
        sonar.add(time.time(), float(data.split(' ')[0]))

