from dc_control_panel import TimeSeries, ControlPanel
import math
import time
import random
from original_connect import RobotConnection

sonar = TimeSeries('sonar', 50)

robot = RobotConnection('COM6')


cp = ControlPanel(ts = [sonar])
cp.start()

while True:
    data = robot.read_one()
    if not data is None:
        sonar.add(time.time(), float(data))

