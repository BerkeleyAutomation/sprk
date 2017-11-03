from sprk.core import *

comm_port = "COM7" #set based on your system

#initialize the python API
sprk = SPRK(comm_port)

#home the robot
sprk.home()

#absolute positional control to z=0.2
p = PosObj()
p.z = 0.2

sprk.position(p.request)


#periodic wave
w = WaveObj()
w.x = [.2,.5] # set [amplitude, frequency] of oscillations about x-axis (or y, z, xr, yr, zr)

sprk.sinWave(w.request)


