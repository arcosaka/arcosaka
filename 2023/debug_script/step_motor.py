import time
import threading
from pyPS4Controller.controller import Controller
import pigpio

MOTORA = 11 
MOTORB = 8
ENBL = 18
FREQ = 240
DUTY= 50
RANGE = 100
ON = 1
OFF = 0

class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.up_running = False
        self.output_up_thread = None
        self.down_running = False
        self.output_down_thread = None

        
        # initialize gpio
        self.pi = pigpio.pi()

        self.pi.set_mode(MOTORA, pigpio.OUTPUT)
        self.pi.set_mode(MOTORB, pigpio.OUTPUT)
        self.pi.set_mode(ENBL, pigpio.OUTPUT)
        self.pi.write(MOTORA, OFF)
        self.pi.write(MOTORB, OFF)
        self.pi.write(ENBL, OFF)
        
        self.wait_hl = (1.0 / FREQ * (DUTY / 100.0))
        self.wait_lh = (1.0 / FREQ * (1 - (DUTY / 100.0)))
        

    def on_up_arrow_press(self):
        if not self.up_running:
            self.up_running = True
            self.output_up_thread = threading.Thread(target=self.start_up_output)
            self.output_up_thread.start()
        else:
            self.up_running = False
    
    def start_up_output(self):
        while self.up_running:
            print("出力中...")
            self.pi.write(ENBL, ON)
            
            self.pi.write(MOTORA, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(MOTORB, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(MOTORA, OFF)
            time.sleep(self.wait_lh/2)
            self.pi.write(MOTORB, OFF)
            time.sleep(self.wait_lh/2)

    def stop_up_output(self):
        print("出力を停止しました。")
        self.output_up_thread.join()


    def on_down_arrow_press(self):
        if not self.down_running:
            self.down_running = True
            self.output_down_thread = threading.Thread(target=self.start_down_output)
            self.output_down_thread.start()
        else:
            self.down_running = False

    def start_down_output(self):
        while self.down_running:
            print("出力中...")
            self.pi.write(ENBL, ON)
            
            self.pi.write(MOTORB, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(MOTORA, ON)
            time.sleep(self.wait_hl/2)
            self.pi.write(MOTORB, OFF)
            time.sleep(self.wait_lh/2)
            self.pi.write(MOTORA, OFF)
            time.sleep(self.wait_lh/2)

    def stop_down_output(self):
        print("出力を停止しました。")
        self.output_down_thread.join()


if __name__ == "__main__":
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
