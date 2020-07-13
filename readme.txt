----------------------------------------------------------------------------

First Time Raspberry Pi Setup:

Setting up internet connection
1. Plug in microcontroller to power source using included cable.
2. Plug in HDMI display and keyboard. Only necessary for internet set-up.
3. Run command "sudo raspi-config".
4. Select option "2".
5. Select option "N2".
6. Enter SSID of network - note that capitals and spaces matter.
7. Enter password for network.
8. Exit menus.

Connecting through a client computer
1. Run command "ifconfig wlan0"
2. Note the first number on the second line - "inet *.*.*.*"
3. Open terminal on client computer.
4. Run "ssh pi@*.*.*.*" with the IP adress from step 2.
5. Enter password chordata (text does not display as you type).

----------------------------------------------------------------------------

Setting up the Physical System:

1. Do not power system until directed. Live plugging k-ceptors is known to blow the buck converter.
2. Affix all sensors to the subject. 
        -Shoe sensors attach the the outside of the shoe, ##_____ side forward with a binder clip
        -All other sensors, whenever possible, get positioned with the "out" port facing upwards
        -If using Blender for full skeleton capture, 15 sensors req.d - see diagram at https://wiki.chordata.cc/wiki/File:DefBipedConf.png
        -Note that the addresses can be changed from Chordata.xml or full_biped.xml
        -If using algorithm, minimum 4 sensors (1_0=thigh, 1_1=shank, 1_2=foot, 2_0=back)
        -Note that the addresses can be changed in the "addressDict" variable at the top of the algorithm
3. Plug sensors in - HUB -- IN/ID_0/OUT -- IN/ID_1/OUT
4. Connect Hub to raspberry pi - see diagram at https://wiki.chordata.cc/wiki/User_Manual/2._Setting_up_the_system
        -If powering the pi, connect pi to hub with 5 jumpers
        -If powering the hub, connect pi to hub with 4 jumpers and USB
5. Attach remaining components (battery, pi, hub) to the back brace with velcro
5. Ensure that all connections are secure, and plug the battery into the pi/hub

----------------------------------------------------------------------------

Running the Notochord:

Notochord Executable Location:
/home/pi/notochord/bin/notochord

Usage:
./notochord [options] [ip*] [port**]
*For running the algorithm, set to localhost
**Port usually unnecessary

Useful Flags:
-h help, displays more written out version of this short section
-y starts program without waiting for user confirmation
-r gives raw values as output (also --raw)
-x for calibration. Only have one K-Ceptor plugged in (also --calibrate)
-c [path] path to config file. Defaults to home/pi/notochord/Chordata.xsd
-v [0-2] more verbose output, default 0
-l [output] where to redirect log messages
-e [output] where to redirect errors
-t [output] where to redirect data (mostly quaternions)
-s [send_rate] data send rate

--no_bundles unbundles code, do not use with algorithm.
--odr=[frequency] changes sample rate, default 50Hz, max 100Hz
--scan creates armature by scanning setup instead of using configuration file.

Updating the Notochord:
Navigate to the Notochord directory

git checkout develop
git pull
scons -j3 debug=1

----------------------------------------------------------------------------

Installing the Algorithm

run "git pull https://github.com/duncan006/notochordAddons.git"

Running the Algorithm:

1. Follow steps above to run the Notochord, using -r --scan localhost
2. Run algorithmSlip.py in ./notochordAddons

Running the Algorithm (Cheat Sheet):

Fill in IP
ssh pi@IP ./notochordAddons/algorithmSlip.py
ssh pi@IP ./notochord/bin/notochord --raw --scan -y localhost

----------------------------------------------------------------------------

Troubleshooting:
