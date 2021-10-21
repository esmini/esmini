'''
   This script shows how to use the esmini UDPDriverModel
   it offers three modes of input:
      0. driverInput in terms of throttle, brake and SteeringWheelAngle
      1. state in terms of X, Y, Z, Head, Pitch, Roll, Speed, SteeringWheelAngle
      2. state in terms of X, Y, Head, Speed, SteeringWheelAngle 
   Mode 0 and 2 will align to road, while for 1 all values will be respected, even 
   if vehicle will "hang" in the air.

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp-driver-model/two_cars_in_open_space.xosc
   3. From terminal 2, run: ./scripts/udp-driver-model/testUDPDriverModel.py --id 0 --id 1
'''


import sys
import os
import argparse
from socket import *
import struct
from tkinter import *
import tkinter.ttk as ttk


DEFAULT_PORT = 61900

class UdpSender():
    def __init__(self, id=0, ip='127.0.0.1', port=DEFAULT_PORT):
        self.id = id
        self.ip_address = ip
        self.port = port
        
        # Create a UDP socket
        self.sock = socket(AF_INET, SOCK_DGRAM)

        # Set a timeout value of 1 second
        self.sock.settimeout(1)
        self.addr = (self.ip_address, self.port)

    def send(self, msg):
        self.sock.sendto(msg, self.addr)

class Object():
    def __init__(self, id, ip_address, base_port):

        self.id = id
        self.version = 1
        self.inputMode = 1  # vehicle stateXYZHPR
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

        self.udpSender = UdpSender(id, ip_address, base_port + id)

    def delete(self):
        self.sock.close()

    def inputMode2Text(self):
        if (self.inputMode == 0):
            self.inputModeText.set('driverInput')
        elif (self.inputMode == 1):
            self.inputModeText.set('stateXYH')
        elif (self.inputMode == 2):
            self.inputModeText.set('stateXYZHPR')
        else:
            print('Unknown mode:', self.inputMode)

    def sendMessage(self):

        # print('sending obj {} inputmode {} frame {} on port {}'.format(self.id, self.inputMode, self.frameNumber, self.udpSender.port))

        if (self.inputMode == 1):
            self.message = struct.pack('iiiidddddddd', 
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
                self.speed.get(),
                -self.wheel_angle.get())
        elif (self.inputMode == 2):
            self.message = struct.pack('iiiiddddd', 
                self.version, 
                self.inputMode,
                self.objectId, 
                self.frameNumber,
                self.x.get(),
                self.y.get(),
                self.h.get(),
                self.speed.get(),
                -self.wheel_angle.get())
        elif (self.inputMode == 0):
            self.message = struct.pack('iiiiddd', 
                self.version, 
                self.inputMode,
                self.objectId, 
                self.frameNumber,
                self.throttle.get(),
                self.brake.get(),
                -self.wheel_angle.get())

        self.udpSender.send(self.message)
        self.frameNumber += 1

    def updateStateXYZHPR(self, value):
        self.setInputMode(1)

    def updateStateXYH(self, value):
        self.setInputMode(2)

    def updateDriverInput(self, value):
        self.setInputMode(0)            

    def setInputMode(self, mode):
        if (mode < 0 or mode > 2):
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

        self.fps = 30

        self.object = []

        for id in self.obj_id:
            self.object.append(Object(id, self.ip_address, self.base_port))
        
        self.createGUI()
        self.sendMessages()

    def createGUI(self):
        scalewidth = 12
        areasize = 600
        frame0 = Frame(self.master)
        frame0.pack(fill= BOTH, expand= True)
        nb = ttk.Notebook(frame0)

        for obj in self.object:
            tab = Frame(nb)
            tab.pack()
            nb.add(tab, text='obj ' + str(obj.id))

            frame1 = Frame(tab, borderwidth=2, relief=GROOVE)
            frame1.pack(fill= BOTH, expand= True, padx= 10, pady=5)
            Label(frame1, text='StateXYZHPR' + str(obj.id)).pack(side=TOP)

            Scale(frame1, label='x', from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.x, width=scalewidth, command=obj.updateStateXYZHPR).pack(fill=X)

            Scale(frame1, label='y', from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.y, command=obj.updateStateXYZHPR).pack(fill=X)

            Scale(frame1, label='h', from_=-3.14, to=3.14, resolution=0.1, orient=HORIZONTAL, \
                variable=obj.h, width=scalewidth, command=obj.updateStateXYZHPR).pack(fill=X)

            Scale(frame1, label='z', from_=-20, to=20, resolution=0.1, orient=HORIZONTAL, \
                variable=obj.z, width=scalewidth, command=obj.updateStateXYZHPR).pack(fill=X)

            Scale(frame1, label='p', from_=-1.5, to=1.5, resolution=0.1, orient=HORIZONTAL, \
                variable=obj.p, width=scalewidth, command=obj.updateStateXYZHPR).pack(fill=X)

            Scale(frame1, label='wheel angle', from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.wheel_angle, width=scalewidth, command=obj.updateStateXYZHPR).pack(fill=X)

            frame2 = Frame(tab, borderwidth=2, relief=GROOVE)
            frame2.pack(fill= BOTH, expand= True, padx= 10, pady=5)
            Label(frame2, text='StateXYH').pack(side=TOP)

            Scale(frame2, label='x', from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.x, width=scalewidth, command=obj.updateStateXYH).pack(fill=X)

            Scale(frame2, label='y', from_=-areasize/2.0, to=areasize/2.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.y, width=scalewidth, command=obj.updateStateXYH).pack(fill=X)

            Scale(frame2, label='h', from_=-3.14, to=3.14, resolution=0.1, orient=HORIZONTAL, \
                variable=obj.h, width=scalewidth, command=obj.updateStateXYH).pack(fill=X)

            Scale(frame2, label='wheel angle', from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.wheel_angle, command=obj.updateStateXYH).pack(fill=X)


            frame3 = Frame(tab, borderwidth=2, relief=GROOVE)
            frame3.pack(fill= BOTH, expand= True, padx= 10, pady=5)
            Label(frame3, text='Driver input').pack(side=TOP)

            Scale(frame3, label='throttle', from_=0, to=1, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.throttle, width=scalewidth, command=obj.updateDriverInput).pack(fill=X)

            Scale(frame3, label='brake', from_=0, to=1, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.brake, width=scalewidth, command=obj.updateDriverInput).pack(fill=X)

            Scale(frame3, label='wheel angle', from_=-1.0, to=1.0, resolution=0.01, orient=HORIZONTAL, \
                variable=obj.wheel_angle, width=scalewidth, command=obj.updateDriverInput).pack(fill=X)

            bottomframe = Frame(tab)
            bottomframe.pack(fill=BOTH, expand= True, padx= 10, pady=5)
            
            label = Label(bottomframe, text='inputMode: ')
            label.pack(side=LEFT)
            text = Entry(bottomframe, textvariable=obj.inputModeText, state='disabled')
            text.pack(side = LEFT)
        
        nb.pack(expand = 1, fill ="both")

        bottomframe = Frame(self.master)
        bottomframe.pack(fill= BOTH, expand= True, padx= 10, pady=5)

        b=Button(bottomframe, text="Quit", width=10, command=self.quit)
        b.pack(side=RIGHT)

    def sendMessages(self):

        for obj in self.object:
            obj.sendMessage()

        # Sleep for a while according to fps before next send
        self.after((int)(1000.0/self.fps), self.sendMessages)


    def close(self):
        for obj in self.object:
            obj.delete()


if __name__ == "__main__":

    root = Tk()
    root.geometry("400x930+60+30")
    root.title(os.path.splitext(os.path.basename(sys.argv[0]))[0] + ' ' + ' '.join(sys.argv[1:]))
    app = Application(master=root)
    app.mainloop()
    app.close()
    root.destroy()
