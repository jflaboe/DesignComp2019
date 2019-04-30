import serial
import threading
import string

class RobotConnection(threading.Thread):
    def __init__(self, SERIAL_PORT='COM6'):
        threading.Thread.__init__(self)
        self.SERIAL_PORT = SERIAL_PORT
        self.ser = serial.Serial(SERIAL_PORT, 9600)
        self.data_out = []
        self.data_in = []
        self.runnable = True
        self.start()

    def callback(self):
        self.root.quit()

    def run(self):
        while self.runnable:
            if len(self.data_out) > 0 and self.ser.out_waiting == 0:
                self.ser.write(self.data_out.pop(0))
            
            if self.ser.in_waiting > 0:
                data = self.ser.read_until().strip()
                self.data_in.append(data)
    
    def read_one(self):
        if len(self.data_in) > 0:
            return self.data_in.pop(0).decode('utf-8')
        else:
            return None

    def write(self, data):
        self.data_out.append(data.encode('utf-8'))

    def stop_connection(self):
        self.runnable = False