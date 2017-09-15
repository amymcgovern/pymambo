# pymambo
Python interface for Parrot Mambo

This interface was developed to teach kids (K-12) STEM concepts (programming, math, and more) by having them program a drone to fly autonomously.  Anyone can use it who is interested in autonomous drone programming!  

## Requirements

### Hardware

This code was developed and tested on a Parrot Mambo (regular Mambo, without the FPV camera, we will work on that in the next update) and a Raspberry Pi 3B.  It should work on any linux machine with BLE support.

### Software
You will need python, pybluez, and untangle.  All of these are available for the Raspberry Pi 3.  To install these do the following:

```
sudo apt-get update
sudo apt-get install bluetooth
sudo apt-get install bluez
sudo apt-get install python-bluez
pip install untangle
```

To install the pymambo code, download or clone the repository.

## Using the pymambo library

To use the library, you will first need to find the address of your Mambo.  BLE permissions on linux require that this command run in sudo mode.  To this this, from the directory where you installed the pymambo code, type:

```
sudo python findMambo.py
```

This will identify all BLE devices within hearing of the Pi.  The Mambo will be identified at the end.  Save the address and use it in your connection code (discussed below).  If findMambo does not report "FOUND A MAMBO!", then be sure your Mambo is turned on when you run the findMambo code and that your Pi (or other linux box) has its BLE interface turned on.

## Flying

Once you have gotten the address of your mambo from findMambo, you can use it to connect to the Mambo and fly!  We have provided some example scripts and a list of the available commands for writing your own scripts.

Note that you do not need to run any of the flying code in sudo mode!  That was only for discovery.

