from connect import *
import json
def do_nothing(val):
    pass

def print_executed(val):
    print(val["last_executed"])
class Commander(threading.Thread):
    def __init__(self, serial_port='COM6', use_queue=False, on_read=do_nothing):
        threading.Thread.__init__(self)
        self.bt = RobotConnection(SERIAL_PORT=serial_port)
        self.command_queue = []
        self.last_sent = 0
        self.last_confirmed = 0
        self.use_queue = use_queue
        self.runnable = True
        self.on_read = on_read
        self.start()
        
        

    def callback(self):
        self.runnable = False
        self.join()
        self.bt.stop_connection()

    def run(self):
        while self.runnable is True:
            data = self.bt.read_all()
            for d in data:
                d = json.loads(d)
                self.last_confirmed = d["last_executed"]
                self.on_read(d)
            
            if self.last_confirmed == self.last_sent:
                if len(self.command_queue) > 0:
                    self.last_sent = self.last_sent + 1
                    next_command = self.command_queue.pop(0)
                    next_command.id = self.last_sent
                    self.send(next_command)



    def send(self, command):
        print(str(command))
        self.bt.write(str(command))

    def add_command(self, command):
        if self.use_queue is True:
            self.command_queue.append(command)
        else:
            if len(self.command_queue) == 0:
                self.command_queue.append(command)

    



class Command:
    def __init__(self):
        self.id = 0
        self.command = 0
        self.d1 = 0
        self.d2 = 0
        self.d3 = 0
        self.d4 = 0
        self.d5 = 0

    def __str__(self):
        return " ".join([str(self.id), str(self.command), str(self.d1), str(self.d2), str(self.d3), str(self.d4), str(self.d5)]) + "\n"

    def __repr__(self):
        return self.__str__()


    

class Direct(Command):
    def __init__(self, pwm1, pwm2, dir1, dir2, t):
        Command.__init__(self)
        self.command = 1
        self.d1 = pwm1
        self.d2 = pwm2
        self.d3 = dir1
        self.d4 = dir2
        self.d5 = t


class Stop(Command):
    def __init__(self):
        Command.__init__(self)
        self.command = 0


class Lever(Command):
    def __init__(self, value):
        Command.__init__(self)
        self.command = 5
        self.d1 = value