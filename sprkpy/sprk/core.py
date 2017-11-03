
import time
import serial
import numpy as np
import math as m

"""
The core module defines all of the core control primitives of the sprk
"""


class SPRK:
    """
    This is the primary control object for the platform it define the main API
    """

    def __init__(self, comm, baudrate=57600,timeout=.5):

        """
        The control class is initialized with three platform dependent parameters:

        e.g., MacOSX
        comm="/dev/tty.usbmodem1785631",
        baudrate=57600,
        timeout=.5,

        e.g., Windows
        comm="COM14",
        baudrate=57600,
        timeout=.5,

		e.g., Linux
		comm=/dev/ttyUSB0
		baudrate=57600
		timeout=.5
        """

        # initialize Serial Connection
        self.ser = serial.Serial(comm,baudrate)
        
        time.sleep(.5)

        #homes the platform
        self.home()
    

    def home(self):
        """
        This function resets the platforms state to home
        """
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write(b"h")
        return


    def position(self,requests):
        
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write(b"a")
        
        # write the rest of the requests (xyz, rpy)
        for i in range(0,3):                                        # Positions x,y,z are in the range -0.5 to 0.5
            val = int((requests[i]+0.5)*100)
            self.ser.write(str(val%10).encode())
            self.ser.write(str((int(val/10))%10).encode())

        for i in range(3,6):                                        # Angles r,p,y are in the range -15 to 15
            val = int(requests[i]+15)
            self.ser.write(str(val%10).encode())
            self.ser.write(str((int(val/10))%10).encode())
        return
        
    def mixWave(self,requests):
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write(b"b")
        self.send_wave_requests(requests)
        
    def sinWave(self,requests):
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write(b"c")
        self.send_wave_requests(requests)
        
    def sinBreath(self,requests):
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write(b"d")
        self.send_wave_requests(requests)
    
    def send_wave_requests(self,requests):
        self.ser.flushOutput()
        self.ser.flushInput()
        for i in range(0,3):
            val = int((requests[2*i]+0.5)*100)                            # x,y,z amplitudes
            self.ser.write(str(val%10).encode())
            self.ser.write(str((int(val/10))%10).encode())
            val2 = int((requests[2*i+1])*100)                             # x,y,z frequencies
            self.ser.write(str(val2%10).encode())
            self.ser.write(str((int(val2/10))%10).encode())
        for i in range(3,6):
            val = int(requests[2*i]+15)                                   # xr,yr,zr amplitudes
            self.ser.write(str(val%10).encode())
            self.ser.write(str((int(val/10))%10).encode())
            val2 = int((requests[2*i+1])*100)                             # xr,yr,zr frequencies
            self.ser.write(str(val2%10).encode())
            self.ser.write(str((int(val2/10))%10).encode())
        return


class WaveObj:
    def __init__(self):
        self.xLim = .5
        self.zLim = .5
        self.xrLim = 15
        self.zrLim = 15
        
        #magnitude and frequency
        self.x = [0,0]
        self.xr = [0,0]
        self.y = [0,0]
        self.yr = [0,0]
        self.z = [0,0]
        self.zr = [0,0]
        
    @property
    def request(self):
        arr = self.x+self.y+self.z+self.xr+self.yr+self.zr
        #print arr
        arr[0] = max(-self.xLim,min(arr[0],self.xLim))
        arr[2] = max(-self.xLim,min(arr[2],self.xLim))
        arr[4] = max(-self.zLim,min(arr[4],self.zLim))
        arr[6] = max(-self.xrLim,min(arr[6],self.xrLim))
        arr[8] = max(-self.xrLim,min(arr[8],self.xrLim))
        arr[10] = max(-self.zrLim,min(arr[10],self.zrLim))
        return arr
    
class PosObj:
    def __init__(self,position = [0,0,0,0,0,0]):
        
        self.xLim = .5
        self.zLim = .5
        self.xrLim = 15
        self.zrLim = 15
        
        self.x = 0
        self.xr = 0
        self.y = 0
        self.yr = 0
        self.z = 0
        self.zr = 0

    @property
    def request(self):
        x = max(-self.xLim,min(self.x,self.xLim))
        y = max(-self.xLim,min(self.y,self.xLim))
        z = max(-self.xLim,min(self.z,self.zLim))
        xr = max(-self.xrLim,min(self.xr,self.xrLim))
        yr = max(-self.xrLim,min(self.yr,self.xrLim))
        zr = max(-self.zrLim,min(self.zr,self.zrLim))
        return [x,y,z,xr,yr,zr]
        
        
        
