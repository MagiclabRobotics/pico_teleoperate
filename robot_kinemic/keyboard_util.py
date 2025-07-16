from pynput.keyboard import Key, Listener

import threading

import time

class keyboardManage():
    def __init__(self) -> None:
        pass
        self.button_count = 4
        self.status = [0 for _ in range(self.button_count)]

        self.buttons = ["a", "w", "s", "d"]

        self.init_keyboard()
    
    def init_keyboard(self):
        t1 = threading.Thread(target=self.listen)
        t1.start()

    def on_press(self,key):
        try:
            print(f"你按下了 {key} 键")
            for i in range(self.button_count):
                if(key.char == self.buttons[i]):
                    self.status[i] = 1
        except:
            pass

    def on_release(self,key):
        try:
            print(f"你松开了 {key} 键")
            for i in range(self.button_count):
                if(key.char == self.buttons[i]):
                    self.status[i] = 0
        except:
            pass
    def listen(self):
        with Listener(
        on_press=self.on_press,
        on_release=self.on_release
        ) as listener:
            listener.join()


