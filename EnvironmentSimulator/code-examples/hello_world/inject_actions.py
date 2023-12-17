'''
   demonstrate injection of actions from an external process using UDP interface

   Python dependencies:
      tkinter (usually included with Python on Windows)
         test if installed: python -m tkinter (or python3 -m tkinter)
         sudo apt-get install python3-tk

   How to run:
   1. Open two terminals in esmini root folder
   2. From terminal 1, run: ./EnvironmentSimulator/code-examples/hello_world/inject_actions.py
   3. From terminal 2, run: ./bin/esmini --window 80 80 800 400 --osc ./EnvironmentSimulator/Unittest/xosc/dummy_mw.xosc --action_server

   Note: Default is local host (127.0.0.1) but IP address can be specified with -a argument, e.g:
       ./EnvironmentSimulator/code-examples/hello_world/inject_actions.py -a 192.168.123.132

   Troubleshooting: If no actions happens it might be that the port (default 48197) need to be
   opened in the firewall of the receiving device. E.g. in Windows create an inbound rule of
   type Port to allow UDP connections on the specific port.
'''


from tkinter import *
from socket import *
import ctypes as ct
import optparse
import time
from enum import IntEnum

class ActionType(IntEnum):
    UNDEFINED          = 0
    SPEED_ACTION       = 1
    LANE_CHANGE_ACTION = 2
    LANE_OFFSET_ACTION = 3
    PLAY               = 4
    PAUSE              = 5
    STEP               = 6
    STEP_DT            = 7
    QUIT               = 8
    NR_OF_ACTIONS      = 9

class SpeedActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
        ("id", ct.c_int),
        ("speed", ct.c_float),
        ("transition_shape", ct.c_int),
        ("transition_dim", ct.c_int),
        ("transition_value", ct.c_float),
    ]

class LaneChangeActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
        ("id", ct.c_int),
        ("mode", ct.c_int),
        ("target", ct.c_int),
        ("transition_shape", ct.c_int),
        ("transition_dim", ct.c_int),
        ("transition_value", ct.c_float),
    ]

class LaneOffsetActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
        ("id", ct.c_int),
        ("offset", ct.c_float),
        ("maxLateralAcc", ct.c_float),
        ("transition_shape", ct.c_int),
    ]

class PlayActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
    ]

class PauseActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
    ]

class StepActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
    ]

class StepDTActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
        ("dt", ct.c_float),
    ]

class QuitActionStruct(ct.Structure):
    _fields_ = [
        ("action_type", ct.c_int),
    ]

class Application(Frame):

    def __init__(self, master, ip):
        self.sock = socket(AF_INET, SOCK_DGRAM)
        self.addr = (ip, 48197)
        self.speed = DoubleVar(value = 50.0)
        self.loop = False
        Frame.__init__(self, master)
        self.createGUI()

    def createGUI(self):
        tab_frame = Frame(self.master)
        tab_frame.grid(row=0, sticky='EW')

        # Label(tab_frame, text="speed").grid(sticky = W, row=0)
        # Scale(tab_frame, from_=0.0, to=180.0, resolution=1.0, orient=HORIZONTAL, variable=self.speed,
        #     command=self.set_speed).grid(sticky = EW, row=1, columnspan=2)
        b = Button(tab_frame, text="Set speed 40 km/h", width=21, command=lambda:self.set_speed(40))
        b.grid(row=1,column=0,sticky=W)
        b = Button(tab_frame, text="Set speed 70 km/h", width=21, command=lambda:self.set_speed(70))
        b.grid(row=1,column=1,sticky=E)
        b = Button(tab_frame, text="Lane change left", width=21, command=lambda:self.lane_change(1))
        b.grid(row=2,column=0,sticky=W)
        b = Button(tab_frame, text="Lane change right", width=21, command=lambda:self.lane_change(-1))
        b.grid(row=2,column=1,sticky=E)
        b = Button(tab_frame, text="Set lane offset = 0.5m left", width=21, command=lambda:self.lane_offset(0.5))
        b.grid(row=3,column=0,sticky=W)
        b = Button(tab_frame, text="Set lane offset = 0.5m right", width=21, command=lambda:self.lane_offset(-0.5))
        b.grid(row=3,column=1,sticky=E)
        b = Button(tab_frame, text="Play", width=21, command=lambda:self.play())
        b.grid(row=4,column=0,sticky=W)
        b = Button(tab_frame, text="Pause", width=21, command=lambda:self.pause())
        b.grid(row=4,column=1,sticky=E)
        b = Button(tab_frame, text="Step 0.05s", width=21, command=lambda:self.step_dt(0.05))
        b.grid(row=5,column=0,sticky=W)
        b = Button(tab_frame, text="Step loop", width=21, command=lambda:self.step())
        b.grid(row=5,column=1,sticky=E)
        b = Button(tab_frame, text="Quit esmini", width=21, command=lambda:self.quit_esmini())
        b.grid(row=6,column=0,sticky=W)
        b = Button(tab_frame, text="Quit", width=21, command=self.quit)
        b.grid(row=6,column=1,sticky=E)

        self.master.update()
        self.master.minsize(self.master.winfo_width(), self.master.winfo_height())

    def set_speed(self, value):
        print('set speed: ', value)
        msg = SpeedActionStruct(ActionType.SPEED_ACTION, 0, value / 3.6, 0, 1, 5.0)  # shape = CUBIC, dim = RATE,  acc = 5.0
        self.sock.sendto(msg, self.addr)

    def lane_change(self, value):
        print('lane change: ', value)
        msg = LaneChangeActionStruct(ActionType.LANE_CHANGE_ACTION, 0, 1, value, 0, 2, 3.0)  # shape = CUBIC, DIM = time, time = 3.0
        self.sock.sendto(msg, self.addr)

    def lane_offset(self, value):
        print('lane offset: ', value)
        msg = LaneOffsetActionStruct(ActionType.LANE_OFFSET_ACTION, 0, value, 2.0, 2)  # maxLatAcc = 2.0 m/s^2, shape = SINUSOIDAL
        self.sock.sendto(msg, self.addr)

    def play(self):
        print('play')
        self.loop = False
        msg = PlayActionStruct(ActionType.PLAY)
        self.sock.sendto(msg, self.addr)

    def pause(self):
        print('pause')
        self.loop = False
        msg = PauseActionStruct(ActionType.PAUSE)
        self.sock.sendto(msg, self.addr)

    def step(self):
        print('step loop')
        self.loop = True
        self.step_loop()

    def step_dt(self, value):
        print('step dt', value)
        self.loop = False
        msg = StepDTActionStruct(ActionType.STEP_DT, value)
        self.sock.sendto(msg, self.addr)

    def quit_esmini(self):
        print('quit esmini')
        msg = QuitActionStruct(ActionType.QUIT)
        self.sock.sendto(msg, self.addr)

    def step_loop(self):
        msg = StepActionStruct(ActionType.STEP)
        self.sock.sendto(msg, self.addr)
        if self.loop == True:
            # Sleep for a while according to fps before next send
            self.after(20, self.step_loop)  # step every 20 ms

    def close(self):
        self.sock.close()

if __name__ == "__main__":

    parser = optparse.OptionParser()
    parser.add_option('-a', '--address',
        action="store", dest="address",
        help="ip address of destination host", default="127.0.0.1")

    options, args = parser.parse_args()

    root = Tk()
    root.attributes('-topmost',True)
    app = Application(master=root, ip=options.address)

    app.mainloop()
    app.close()
    root.destroy()