'''
   demonstrate use of ParameterConditions for triggering of lane changes from an external application

   Python dependencies:
      tkinter (usually included with Python on Windows)
         test if installed: python -m tkinter (or python3 -m tkinter)
         sudo apt-get install python3-tk

   Instruction:
     - copy the esmini shared library (esminiLib.dll or esminiLib.so) to the current folder
     - run: ./trig_lane_change.py
'''


import sys
import ctypes
from tkinter import *
import tkinter.ttk as ttk

STATE = {
  'idle': 0,
  'left': 1,
  'right': 2,
}

fps = 50

class Application(Frame):

    def __init__(self, master=None):

        Frame.__init__(self, master)
        self.state = STATE['idle']
        self.createGUI()

        if sys.platform == "linux" or sys.platform == "linux2":
            self.se = ctypes.CDLL("./libesminiLib.so")
        elif sys.platform == "darwin":
            self.se = ctypes.CDLL("./libesminiLib.dylib")
        elif sys.platform == "win32":
            self.se = ctypes.CDLL("./esminiLib.dll")
        else:
            print("Unsupported platform: {}".format(sys.platform))
            quit()

        self.se.SE_AddPath(b"../../../resources/xosc/Catalogs/Vehicles", 0, 1, 0, 0)
        self.se.SE_Init(b"trig_lane_change.xosc", 0, 1, 0, 0)
        self.step()

    def createGUI(self):
        tab_frame = Frame(self.master)
        tab_frame.grid(row=0, sticky='EW')

        b = Button(tab_frame, text="Left", width=10, command=self.left)
        b.grid(row=0,sticky=W)
        b = Button(tab_frame, text="Right", width=10, command=self.right)
        b.grid(row=0,sticky=E)
        b = Button(tab_frame, text="Quit", width=20, command=self.quit)
        b.grid(row=1,sticky=EW)

        self.master.update()
        self.master.minsize(self.master.winfo_width(), self.master.winfo_height())

    def step(self):
        self.se.SE_Step()

        # need to reset the bool flag value, to prepare for any other lane change
        if self.state == STATE['left']:
            self.se.SE_SetParameterBool(b"ChangeLeft", 0)
        elif self.state == STATE['right']:
            self.se.SE_SetParameterBool(b"ChangeRight", 0)

        self.state == STATE['idle']
        root.after(int(1000.0 / fps), self.step)  # reschedule step

    def left(self):
        self.se.SE_SetParameterBool(b"ChangeLeft", 1)
        self.state = STATE['left']

    def right(self):
        self.se.SE_SetParameterBool(b"ChangeRight", 1)
        self.state = STATE['right']

    def close(self):
        self.se.SE_Close();


if __name__ == "__main__":

    root = Tk()
    root.attributes('-topmost',True)
    app = Application(master=root)

    app.mainloop()
    app.close()
    root.destroy()
