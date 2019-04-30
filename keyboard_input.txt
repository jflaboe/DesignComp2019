from pynput.keyboard import Key, Listener

class KeyboardController:
    def __init__(self):
        self.functions = {}

    def on_press(self, key):
        if key in self.functions:
            self.functions[key]()

    def add(self, key, function):
        self.functions[key] = function