# Stewart Platform Research Kit (SPRK)
To simulate body organ motion due to breathing,
heart beats, or peristaltic movements, we designed a lowcost,
miniaturized SPRK (Stewart Platform Research Kit) to
translate and rotate phantom tissue. This platform is 20cm × 20cm × 10cm to fit in the workspace of a da Vinci Research Kit
(DVRK) surgical robot and costs $250. 

## Design
- Dynamixel XL-320 servo motors    (x6)
- OpenCM9.04-C Micro-controller
- USB Downloader (LN-101)
- 1/4 in Acrylic and Delrin Sheets
- Robotis Rivet Set (RS-10)
- Nylon Ball Joint M3 x 24mm x 3mm    (x6)

<p align="center">
  <img src="https://raw.githubusercontent.com/BerkeleyAutomation/sprk/master/design/SPRK2%20with%20DVRK.JPG" width="400">
</p>

CAD models and design docs can be found [here](https://github.com/BerkeleyAutomation/sprk/tree/master/design/).
Instruction on fabricating and assembling the SPRK 2 can be found [here](https://github.com/BerkeleyAutomation/sprk/tree/master/assembly/).

## sprkino
The sprkino package contains the sources for the firmware for the stewart platform. To build and upload these sources, first download the [Robotis OpenCM IDE](http://en.robotis.com/index/product.php?cate_code=131010). The code is uploaded to a OpenCM9.04-C microcontroller using a Serial USB connector (distinct from the LN-101 USB connector that connects to the software interface). 

## sprkpy
The sprk can be controlled from any USB capable device. First, connect the USB Dowloader port (LN-101) to the device. First, enter the appropriate directory and install the requirments:
```
$ cd sprkpy
$ pip install -r requirements.txt
```
Then, run the setup script
```
$ python setup.py
```
Then, install then run the example script from the python API:
```
$ python example.py
```

The example script has a bunch of interesting simple motion examples--and should get you started:
```
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

```


