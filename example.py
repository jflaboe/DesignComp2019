from dc_control_panel import TimeSeries, ControlPanel
import math
import time
import random


#creating our time series objects. Use TimeSeries.add((t, x)) to add a new datapoint
sin_series = TimeSeries("sine", 50, fx = 3.6, fy = 1.8)
cos_series = TimeSeries("cosine", 50, fx = 3.6, fy = 1.8)
rand_series = TimeSeries("random", 50, fx = 3.6, fy = 1.8)


#initiates the control panel
cp = ControlPanel(ts = [cos_series, sin_series, rand_series])

###

#Any edits needed to be made to the control panel before starting it would go here

###


#open the panel
cp.start()


#simulates the series for 'time_length' seconds
def update_series(time_length):
    start = time.time()
    time_since = time.time() - start
    while time_since < time_length:
        sin_val = math.sin(time_since * 2 * math.pi / 5)
        cos_val = math.cos(time_since * 2 * math.pi / 5)
        rand_val = random.random() * 10
        
        sin_series.add(time_since, sin_val)
        cos_series.add(time_since, cos_val)
        rand_series.add(time_since, rand_val)
        time.sleep(0.1)
        time_since = time.time() - start



update_series(15)

#closes the panel
cp.callback()



