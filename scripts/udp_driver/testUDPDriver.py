'''
   This script shows how to use the esmini UDPDriverController
   it offers three modes of input:
      0. driverInput in terms of throttle, brake and SteeringWheelAngle
      1. state in terms of X, Y, Z, Head, Pitch, Roll, Speed, SteeringWheelAngle
      2. state in terms of X, Y, Head, Speed, SteeringWheelAngle 
   Mode 0 and 2 will align to road, while for 1 all values will be respected, even 
   if vehicle will "hang" in the air.

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/two_cars_in_open_space.xosc
   3. From terminal 2, run: ./scripts/udp_driver/testUDPDriver.py --id 0 --id 1
        or python ./scripts/udp_driver/testUDPDriver.py --id 0 --id 1
        or python3 ./scripts/udp_driver/testUDPDriver.py --id 0 --id 1
        depending on platform and file type associations

   If esmini is running on another host, add argument --ip <ip address of host running esmini>, e.g. --ip 216.58.211.14
   You might need to open ports in firewall. E.g. in Windows add a Inbound Rule for UDP messages on port range 49950-49999.

   Running controllers and esmini on different hosts works as long as they have the same endianess/byte-order, so e.g.
   Mac, Linux and Mac on Intel or the new Mac M1 chip should all work (since they are little-endian). 
   
   For other endianess, e.g. dSPACE runtime platform, you would need to swap byteorder on sender or receiver side.

   For complete message definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriver.hpp
'''


import sys
import os
import argparse
from tkinter import *
import tkinter.ttk as ttk
from udp_osi_common import *

DEFAULT_PORT = 53995

class Object():
    def __init__(self, id, ip_address, base_port):

        self.id = id
        self.version = 1
        self.inputMode = input_modes['driverInput']
        self.inputModeText = StringVar()
        self.inputMode2Text()
        self.objectId = self.id
        self.frameNumber = 0

        self.x = DoubleVar(value = 0.0)
        self.y = DoubleVar(value = 0.0)
        self.z = DoubleVar(value = 0.0)
        self.h = DoubleVar(value = 0.0)
        self.p = DoubleVar(value = 0.0)
        self.r = DoubleVar(value = 0.0)
        self.speed = DoubleVar(value = 0.0)
        self.wheel_angle = DoubleVar(value = 0.0)
        self.throttle = DoubleVar(value = 0.0)
        self.brake = DoubleVar(value = 0.0)
        self.dead_reckon = IntVar(value = 0)

        self.udpSender = UdpSender(ip_address, base_port + id)

    def delete(self):
        self.udpSender.sock.close()

    def inputMode2Text(self):
        if (self.inputMode == input_modes['driverInput']):
            self.inputModeText.set('driverInput')
        elif (self.inputMode == input_modes['stateXYH']):
            self.inputModeText.set('stateXYH')
        elif (self.inputMode == input_modes['stateXYZHPR']):
            self.inputModeText.set('stateXYZHPR')
        elif (self.inputMode == 0):
            self.inputModeText.set('-')
        else:
            print('Unknown mode:', self.inputMode)

    def sendMessage(self):

        # print('sending obj {} inputmode {} frame {} on port {}'.format(self.id, self.inputMode, self.frameNumber, self.udpSender.port))
        if (self.inputMode == input_modes['stateXYZHPR']):
            message = struct.pack('iiiiddddddddB',
                self.version,
                self.inputMode,
                self.objectId,
                self.frameNumber,
                self.x.get(),
                self.y.get(),
                self.z.get(),
                self.h.get(),
                self.p.get(),
                self.r.get(),
                self.speed.get() / 3.6,
                -self.wheel_angle.get(),
                self.dead_reckon.get())
        elif (self.inputMode == input_modes['stateXYH']):
            message = struct.pack('iiiidddddB',
                self.version,
                self.inputMode,
                self.objectId,
                self.frameNumber,
                self.x.get(),
                self.y.get(),
                self.h.get(),
                self.speed.get() / 3.6,
                -self.wheel_angle.get(),
                self.dead_reckon.get())
        elif (self.inputMode == input_modes['driverInput']):
            message = struct.pack('iiiiddd',
                self.version,
                self.inputMode,
                self.objectId,
                self.frameNumber,
                self.throttle.get(),
                self.brake.get(),
                -self.wheel_angle.get())
        
        if (message is not None):
            self.udpSender.send(message)
            self.frameNumber += 1
        else:
            print('none')

    def updateStateXYZHPR(self, value = 0):
        self.setInputMode(input_modes['stateXYZHPR'])
        self.sendMessage()

    def updateStateXYH(self, value = 0):
        self.setInputMode(input_modes['stateXYH'])
        self.sendMessage()

    def updateDriverInput(self, value = 0):
        self.setInputMode(input_modes['driverInput'])
        self.sendMessage()

    def setInputMode(self, mode):
        if (mode < 1 or mode > 3):
            print('Unknown mode:', mode)
        else:
            self.inputMode = mode
        self.inputMode2Text()


class Application(Frame):

    def __init__(self, master=None):

        Frame.__init__(self, master)

        # Create the parser
        parser = argparse.ArgumentParser(description='UDP driver model')

        # Add the arguments
        parser.add_argument('--id', action='append', nargs='+')
        parser.add_argument('--port', help='Start UDP port (default is ' + str(DEFAULT_PORT) + ' objectID)')
        parser.add_argument('--ip', help='IP address of esmini host', default='127.0.0.1')

        args = parser.parse_args()

        if (args.id is None):
            self.obj_id = [0]
        else:
            self.obj_id = [int(item) for args in args.id for item in args]

        if (args.port is None):
            self.base_port = DEFAULT_PORT
        else:
            self.base_port = int(args.port)

        if (args.ip is None):
            self.ip_address = '127.0.0.1'
        else:
            self.ip_address = args.ip

        self.fps = 60
        self.continuous = BooleanVar(value = False)

        self.object = []

        for id in self.obj_id:
            self.object.append(Object(id, self.ip_address, self.base_port))

        self.createGUI()
        self.sendMessages()

    def createGUI(self):
        scalewidth = 12
        areasize = 600
        notebook = ttk.Notebook(self.master)
        notebook.pack(fill=BOTH, expand=True)
        padx = 2

        for obj in self.object:
            tab_frame = Frame(notebook)
            tab_frame.grid(row=0, sticky='EW')
            tab_frame.grid_columnconfigure(0, weight=1)
            notebook.add(tab_frame, text='obj ' + str(obj.id))

            frame1 = Frame(tab_frame, borderwidth=2, relief=GROOVE)
            frame1.grid(row=0,sticky=EW)
            frame1.grid_rowconfigure(0, weight=1)
            frame1.grid_columnconfigure(1, weight=1, minsize=200)

            row = 0
            Label(frame1, text='StateXYZHPR' + str(obj.id)).grid(row=row, sticky=N)

            row += 1
            Label(frame1, text="x").grid(sticky = SE, row = row, column = 0,padx = padx)
            Scale(frame1, from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, variable=obj.x, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="y").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, variable=obj.y, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="h").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-3.14, to=3.14, resolution=0.01, orient=HORIZONTAL, variable=obj.h, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="z").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-20, to=20, resolution=0.1, orient=HORIZONTAL, variable=obj.z, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="p").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-1.5, to=1.5, resolution=0.01, orient=HORIZONTAL, variable=obj.p, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="speed").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-10.0, to=100.0, resolution=1, orient=HORIZONTAL, variable=obj.speed, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame1, text="wheel angle").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame1, from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, variable=obj.wheel_angle, width=scalewidth,
                command=obj.updateStateXYZHPR).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Checkbutton(frame1, text="dead reckoning", variable=obj.dead_reckon, command=obj.updateStateXYZHPR, onvalue=1, offvalue=0).\
                grid(sticky = SW, row = row, column = 1, padx = padx, pady=5)

            frame2 = Frame(tab_frame, borderwidth=2, relief=GROOVE)
            frame2.grid(row=1, sticky='EW')
            frame2.grid_rowconfigure(1, weight=1)
            frame2.grid_columnconfigure(1, weight=1, minsize=200)
            frame2.grid_columnconfigure(0, minsize=2)

            row = 0
            Label(frame2, text='StateXYH' + str(obj.id)).grid(row=row, sticky=N)

            row += 1
            Label(frame2, text="x").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame2, from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, variable=obj.x, width=scalewidth,
                command=obj.updateStateXYH).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame2, text="y").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame2, from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, variable=obj.y, width=scalewidth,
                command=obj.updateStateXYH).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame2, text="h").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame2, from_=-3.14, to=3.14, resolution=0.01, orient=HORIZONTAL, variable=obj.h, width=scalewidth,
                command=obj.updateStateXYH).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame2, text="speed").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame2, from_=-10.0, to=100.0, resolution=1, orient=HORIZONTAL, variable=obj.speed, width=scalewidth,
                command=obj.updateStateXYH).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame2, text="wheel angle").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame2, from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, variable=obj.wheel_angle, width=scalewidth,
                command=obj.updateStateXYH).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Checkbutton(frame2, text="dead reckoning", variable=obj.dead_reckon, command=obj.updateStateXYH, onvalue=1, offvalue=0).\
                grid(sticky = SW, row = row, column = 1, padx = padx, pady=5)

            frame3 = Frame(tab_frame, borderwidth=2, relief=GROOVE)
            frame3.grid(row=2,sticky=EW)
            frame3.grid_rowconfigure(2, weight=1)
            frame3.grid_columnconfigure(1, weight=1, minsize=200)
            frame3.grid_columnconfigure(0, minsize=2)

            row = 0
            Label(frame3, text='Driver input' + str(obj.id)).grid(row=row)

            row += 1
            Label(frame3, text="throttle").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame3, from_=0, to=1, resolution=0.01, orient=HORIZONTAL, variable=obj.throttle, width=scalewidth, 
                command=obj.updateDriverInput).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame3, text="brake").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame3, from_=0, to=1, resolution=0.01, orient=HORIZONTAL, variable=obj.brake, width=scalewidth, 
                command=obj.updateDriverInput).grid(sticky = EW, row = row, column = 1, padx = padx)

            row += 1
            Label(frame3, text="wheel angle").grid(sticky = SE, row = row, column = 0, padx = padx)
            Scale(frame3, from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, variable=obj.wheel_angle, width=scalewidth, 
                command=obj.updateDriverInput).grid(sticky = EW, row = row, column = 1, padx = padx)

            
            bottom_frame = Frame(tab_frame, borderwidth=2, relief=GROOVE)
            bottom_frame.grid(row=3,sticky=EW)
            row = 0
            Label(bottom_frame, text='inputMode: ').grid(sticky = W, row = row, column = 0, padx = padx)
            Entry(bottom_frame, textvariable=obj.inputModeText, state='disabled').grid(sticky = E, row = row, column = 1, padx = padx)
        
        Button(self.master, text="Quit", width=10, command=self.quit).pack(side=RIGHT)

        Checkbutton(self.master, text='continuous mode', variable=self.continuous, command=self.updateContinuousMode, onvalue=True, offvalue=False).pack(side=LEFT)

        self.master.update()
        self.master.minsize(self.master.winfo_width(), self.master.winfo_height())

    def updateContinuousMode(self):
        if self.continuous.get():
            self.sendMessages()

    def sendMessages(self):

        for obj in self.object:
            obj.sendMessage()

        if self.continuous.get():
            # Sleep for a while according to fps before next send
            self.after((int)(1000.0/self.fps), self.sendMessages)


    def close(self):
        for obj in self.object:
            obj.delete()


if __name__ == "__main__":

    root = Tk()
    root.title(os.path.splitext(os.path.basename(sys.argv[0]))[0] + ' ' + ' '.join(sys.argv[1:]))
    app = Application(master=root)

    app.mainloop()
    app.close()
    root.destroy()
