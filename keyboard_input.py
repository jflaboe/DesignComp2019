from pynput.keyboard import Key, Listener
import threading


class KeyboardController(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.pressed = set([])
        self.functions = {}
        self.start()

    def on_press(self, key):
        
        self.pressed.add(key)
        if key in self.functions:
            self.functions[key]()

    def on_release(self, key):
        try:
            self.pressed.remove(key)
        except:
            pass

    def add(self, key, function):
        self.functions[key] = function

    def run(self):
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.listener.join()
        

    def callback(self):
        self.listener.stop()
