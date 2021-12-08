'''
   Some useful common classes for the example scripts in this folder
   Prerequisites:
      Python 3
      OSI https://github.com/OpenSimulationInterface/open-simulation-interface
'''

from socket import *
import struct

from osi3.osi_groundtruth_pb2 import GroundTruth

input_modes = {
  'driverInput': 1,
  'stateXYZHPR': 2,
  'stateXYH': 3,
}

base_port = 53995

class UdpSender():
    def __init__(self, ip='127.0.0.1', port=base_port):
        # Create a UDP socket
        self.sock = socket(AF_INET, SOCK_DGRAM)
        self.addr = (ip, port)

    def send(self, msg):
        self.sock.sendto(msg, self.addr)

    def close(self):
        self.sock.close()


class UdpReceiver():
    def __init__(self, ip='127.0.0.1', port=base_port, timeout=-1):
        self.buffersize = 8208  # MAX OSI data size (contract with esmini) + header (two ints)
        # Create a UDP socket
        self.sock = socket(AF_INET, SOCK_DGRAM)
        if timeout >= 0:
            self.sock.settimeout(timeout)
        # Bind to address and ip
        self.sock.bind((ip, port))

    def receive(self):
        bytesAddressPair = self.sock.recvfrom(self.buffersize)
        message = bytesAddressPair[0]
        return message

    def close(self):
        self.sock.close()

class OSIReceiver():
    def __init__(self):
        self.udp_receiver = UdpReceiver(port = 48198)
        self.osi_msg = GroundTruth()

    def receive(self):
        done = False
        next_index = 1
        complete_msg = b''

        # Large nessages might be split in multiple parts
        # esmini will add a counter to indicate sequence number 0, 1, 2...
        # negative counter means last part and message is now complete
        while not done:  
            # receive header
            msg = self.udp_receiver.receive()

            # extract message parts
            header_size = 4 + 4  # counter(int) + size(unsigned int)
            counter, size, frame = struct.unpack('iI{}s'.format(len(msg)-header_size), msg)
            # print('counter {} size {}'.format(counter, size))

            if not (len(frame) == size == len(msg)-8):
                print('Error: Unexpected invalid lengths')
                return
            
            if counter == 1:  # new message
                complete_msg = b''
                next_index = 1

            # Compose complete message
            if counter == 1 or abs(counter) == next_index:
                complete_msg += frame
                next_index += 1
                if counter < 0:  # negative counter number indicates end of message
                    done = True
            else:
                next_index = 1   # out of sync, reset

        # Parse and return message 
        self.osi_msg.ParseFromString(complete_msg)
        return self.osi_msg
        
    def close(self):
        self.udp_receiver.close()
