import os
import can
import time
import struct
import numpy as np



class RadomeCANHandler:
    def __init__(self):

        # This stuff is Linux-specific.
        # See https://www.inno-maker.com/product/usb-can/ for info for other OS

        # The usb2can shows up as the can0 network interface on Linux
        # disable the can0 interface
        os.system('sudo ifconfig can0 down')

        # set it up as can at 125000 bits/s
        os.system('sudo ip link set can0 type can bitrate 125000')

        # internal tx queue length (this doesn't matter very much)
        os.system('sudo ifconfig can0 txqueuelen 1000')

        # enable
        os.system('sudo ifconfig can0 up')

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        self.azimuthID = 1
        self.elevationID = 2


    # send a CAN pakcet to address ID commanding position and velocity in degrees and dps
    def moveMessage(self, position_deg : float, velocity_dps : float, id : int):
        position_deg %= 360
        position = int(round(2.0**24 * (position_deg / 360.0)))
        velocity = min(max(int(round(velocity_dps * (3600 / 3))), -32767), 32767)

        data = struct.pack('>Lh', position, velocity)

        msg = can.Message(arbitration_id=id, data=data[1:])
        self.bus.send(msg)

    # receive a packet, if it is valid, print out its contents
    # TODO actually return the received data
    def recv(self, timeout_s = None):
        if timeout_s is not None:
            msg = self.bus.recv(timeout_s)
        else:
            msg = self.bus.recv()
        
        if msg is not None:
            if msg.is_error_frame:
                print("Received error frame!")
            elif (len(msg.data) == 8):
                pos, vel, cur, vbus = struct.unpack('>LhhB', b'\x00' + msg.data)
                pos_deg = (float(pos) * 360 * 2**-24) % 360
                vel_dps = float(vel) / 1200
                cur_A = float(cur) * 0.001
                vbus_V = float(vbus) * 0.5
                print(f'data from 0x{msg.arbitration_id:03x}: pos = {pos_deg:7.2f}, vel = {vel_dps:7.2f}, cur = {cur_A:6.2f}, vbus = {vbus_V:5.1f}')
            else:
                print(msg)
        return msg

    # Receive and print all packets
    def recvall(self, timeout_s = None):
        msg = []
        while msg is not None:
            msg = self.recv(timeout_s)
        
        return msg
    
    # Generate a trajectory for one axis with constant rotational speed
    # Position and velocity are in degrees and dps
    def rotate(self, pos_init : float, vel : float, addr : int, timeout_s):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            pos = (pos_init + vel * (time.time() - t0)) % 360
            self.moveMessage(pos, vel, addr)
            time.sleep(0.05)
            print(f'Sent position {pos:7.2f} with velocity {vel:7.2f}')
            self.recvall(0)
    
    # Azimuth 0 is at the horizon, 90 is vertical
    def getAzEl(self):
        self.moveMessage(0, 0, 0)
        time.sleep(0.05)
        self.recvall(0)

    def setAzEl(self, az, el, timeout_s):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            self.moveMessage(az, 0, 1)
            # this delay is stupid and related to a firmware problem
            # that is too minor to bother fixing because I already
            # put the covers back on the motor drives
            time.sleep(0.002)
            self.moveMessage(el, 0, 2)
            time.sleep(0.05)
            print(f'Sent az = {az:7.2f}, el = {el:7.2f}')
            self.recvall(0)



if __name__ == '__main__':
    canbus = RadomeCANHandler()
