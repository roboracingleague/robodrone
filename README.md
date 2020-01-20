# Robodrone

## Demo

[![Alt text](https://img.youtube.com/vi/mwLET4B8C2s/0.jpg)](https://www.youtube.com/watch?v=mwLET4B8C2s)


## 1. Drone build

<img src="img/Meaoodrone.jpg" width="400">


### 1.1 Parts
Frame : [![Frame](img/frame.jpeg)](https://www.banggood.com/Realacc-Martian-IV-7-Inch-300mm-Wheelbase-4mm-Arm-Carbon-Fiber-FPV-Racing-Frame-Kit-p-1295851.html?utm_design=41&utm_source=emarsys&utm_medium=Shipoutinform171129&utm_campaign=trigger-emarsys&utm_content=Winna&sc_src=email_2671705&sc_eh=bf6baad03d43ea941&sc_llid=12803824&sc_lid=104858042&sc_uid=yXaVq8Yd1p&cur_warehouse=CN)

Récepteur Frsky XM+ (SBUS) EU LBT : https://www.studiosport.fr/recepteur-frsky-xm-sbus-a13865.html

Motors XNova FS Line 2207 - x4 1700 KV : https://www.studiosport.fr/moteurs-xnova-fs-line-2207-par-4-a17208.html

4 Hélices tripales DALprop TJ6045 Rouge : https://www.studiosport.fr/helices-tripales-dalprop-tj6045-a10781.html

4 ESC AIKON AK 32 35A 6S BLHeli32 : https://www.studiosport.fr/esc-aikon-ak-32-35a-6s-blheli32-a13754.html

UBEC : Matek Systems UBEC DUO 4A/5~12V et 4A/5V 

4S LIPO :   
<a href="https://www.studiosport.fr/batterie-tattu-4s-75c-1550-bf-a14954.html"><img src="img/Tattu4SLIPO.jpg" width="200"></a>

<img src="img/UBEC.jpg" width="200">

Pixhawk 4 mini

<img src="img/Pixhawk4Mini.jpg" width="200">

Radio Controler - Taranis X9D plus

<img src="img/TaranisX9DPlus.jpg" width="200">

Intel realsense T265 & D435

<img src="img/T265-D435.jpeg" width="200">

NVIDIA Jetson nano

<img src="img/nvidia-jetson-nano-developer-kit.jpg" width="200">

3D prints :
- core : <a href="https://cad.onshape.com/documents/5fb94a0090064b109f5c1da5/w/166a6356586f56ed56a74d44/e/b74b3d18538f732dc1240d64"><img src="img/3DPrintCore.png" width="300"></a>
    - STL : [ref](img/Meaoodrone-core.zip)
    - fixings : 
        - d435 : https://www.amazon.fr/gp/product/B07CKZNZRB/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
        - https://www.amazon.fr/gp/product/B07G1B2BJW/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
        - https://www.amazon.fr/gp/product/B07RKTMKGR/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1
        - Support silentbloc M3 2x4 : https://www.dronelec.com/c/p/4480-support-silent-bloc-pour-carte-de-vol-m3-5x5mm-x4/null/
- prop guard x4 : <a href="https://cad.onshape.com/documents/9915b87495d749fffdb4cbc8/w/9cc351269674024398f8d8f9/e/28dcfa34e70ac9709d471085"><img src="img/PropGuard.png" width="200"></a>
    - STL : [ref](img/Prop-guard-v3.stl)

### 1.2 Assembly
#### 1.2.1 Motors and ESC
Mount motors and ESC on each arm and solder the wires. Beware to invert wiring of motor 1 and 2 (they spin counteclockrwise).  

<img src="img/QuadRotorX.svg" width="200">  

#### 1.2.2 Powerboard
Mount powerboard delivered with Pixhawk 4 mini. Solder the wires  
Connect powerboard to Pixhawk with connectors provided with Pixhawk


#### 1.2.3 RC receptor
Connect Frsky XM+ to Pixhawk (use connector privides with Pixhawk and solder with Frsky XM+)

#### 1.2.4 QGround Control settings
Connect your computer to Pixhawk (via micro usb)  
Download, install then open QGround Control on your computer  
You should have now QGC connected to your Pixhawk  
PX4 Firmware version used and tested : 1.10.0  

##### Vehicule setup
Airframe : select 3DR DIY QUad  

Sensor setup : follow instructions  

Radio setup : after having connected your RC (see below), check channel 1 to 12 are active 
and that Roll, Pitch, Yaw and Throttle are correct    

Flight mode : <img src="img/QGC-flightModes.png" width="400">  

Power : <img src="img/QGC-Power.png" width="400">  

Safety : <img src="img/QGC-Safety.png" width="400">  

Parameters :  
- EKF2 :
    - EKF2_AID_MASK : 24
    - EKF2_EVP_NOISE : 0.10 m
    - EKF2_HGT_MODE : vision
- GPS : disabled
- MAVLINK : <img src="img/QGC-MAVLINK.png" width="400">  
- SERIAL : SER_TEL1_BAUD : 921600 8N1
- SYSTEM : <img src="img/QGC-System.png" width="400">  
- Developer : <img src="img/QGC-CB.png" width="400">  


#### 1.2.5 Radio Controller settings
Start and set your taranis so that it connects to your Frsky XM+. Tutorial : https://www.youtube.com/watch?v=ZOBwwNpjNrY  
Set your 12 first channel (first 4 for attitude control). See at end of Readme.md in brain directory for some details


#### 1.2.6 Test your config
Without your propelers mounted, connect your 4S battery to your drone and turn on your Taranis
Arm your drone (from taranis) and check that motors spin in the right way :  
<img src="img/QuadRotorX.svg" width="200">  

You can then test your drone with propelers but outdoor in safety environnement (with no obstacle 10m around and above) and in stabilized mode. Objective is to check that drone can takeoff (no more than 50cm !), yaw, pitch and roll.   
Be ready anytime to disarm if something goes wrong !  

If it's ok, you can move on to the next step : go to autonomous drone build.

#### 1.2.7 Autonoumous drone build
TODO