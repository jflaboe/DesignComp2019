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
        self.a.plot(*zip(*self.data))
        
    

    def add_to_tk(self, root):
        print("adding " + self.name)
        self.canvas = FigureCanvasTkAgg(self.f, master=root)
        self.canvas.get_tk_widget().pack()
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

    def plot (self):
        x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        v= np.array ([16,16.31925,17.6394,16.003,17.2861,17.3131,19.1259,18.9694,22.0003,22.81226])
        p= np.array ([16.23697,     17.31653,     17.22094,     17.68631,     17.73641 ,    18.6368,
            19.32125,     19.31756 ,    21.20247  ,   22.41444   ,  22.11718  ,   22.12453])

        fig = Figure(figsize=(6,6))
        a = fig.add_subplot(111)
        a.scatter(v,x,color='red')
        a.plot(p, range(2 +max(x)),color='blue')
        a.invert_yaxis()

        a.set_title ("Estimation Grid", fontsize=16)
        a.set_ylabel("Y", fontsize=14)
        a.set_xlabel("X", fontsize=14)

        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.get_tk_widget().pack()
        canvas.draw()
    
    

    def run(self):
        self.root = Tk()
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


