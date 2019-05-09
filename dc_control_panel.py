import sys
if sys.version_info[0] < 3:
    from Tkinter import *
else:
    from tkinter import *
import threading
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style
import numpy as np

style.use('ggplot')


class Map(Frame):
    def __init__(self, root, *args, **kwargs):
        Frame.__init__(self, root, *args, **kwargs)
        self.c = Canvas(self, width=200, height=200)
        self.c.pack()
        self.loc = (0, 0)
        self.dir = (0, 0)
        self.lines = []
        self.root = root
        self.pack()



class TimeSeries:
    def __init__(self, name, max_entries, fx=5, fy=4, dpi=100, update = 100):
        self.max_entries = max_entries
        self.name = name
    
        self.data = [(0, 0)]
        self.f = Figure(figsize=(fx, fy), dpi=dpi)
        self.a = self.f.add_subplot(111)
        
        self.canvas = None
        self.update_interval = update
        self.lock = False
    
    def add(self, t, x):
        
        
        self.data.append((t, x))
        if len(self.data) > self.max_entries:
            self.data = self.data[1:]
            
        

    def animate(self, i):
    
        self.a.clear()
        self.a.set_title(self.name)
        self.a.plot(*zip(*self.data))
        
    

    def add_to_tk(self, root):
        print("adding " + self.name)
        self.canvas = FigureCanvasTkAgg(self.f, master=root)
        self.canvas.get_tk_widget().pack(pady=3, side = TOP)
        self.ani = animation.FuncAnimation(self.f, self.animate, interval = self.update_interval)
        self.canvas.draw()
    
    
        



class ControlPanel(threading.Thread):
    def __init__(self, ts=None):
        threading.Thread.__init__(self)
        self.time_series = []
        if not ts is None:
            for t in ts:
                self.time_series.append(t)
        

    def callback(self):
        self.root.quit()

    
    

    def run(self):
        self.root = Tk()
        self.root.configure(background='white')
        self.stop = True
        self.root.title("Design Competition 2019 Control Panel")
        width = self.root.winfo_screenwidth()
        height = self.root.winfo_screenheight()
        self.root.geometry('%sx%s' % (width, height))
        self.right_frame = Frame(width = int(width/3), height = height)
        self.right_frame.pack(side=RIGHT)

        for ts in self.time_series:
            ts.add_to_tk(self.right_frame)
        

        self.root.mainloop()


