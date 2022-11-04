# Lincoln's Robot Car

Don't build this. This is intended as a platform for my son to enjoy and an exercise for myself to see if I could make it. I am putting it up to allow anyone to support and expand the existing building.  It could be useful to harvest components for other projects but the existing build is full of design decisions based on what parts I had available and what I was interested in modeling.  Much better and less complicated designs are available and I recommend using those if you are interested in an RC car or robot.  

## Files
The build includes 2 Autodesk Fusion 360 models.  One contains the bulk of the build while the other just contains the gears which are represented as placeholders in the main model due to model recalculation overhead.  An additional python script is included in the build which runs on Raspbian and requires the evdev library.  Reference the files for the pins used and their functions.

## 3D Printing
All parts were either printed at 100% infill or 50% infill.  Walls were set to 5.  Gears and frame parts were printed at 100%.  These choices are arbitrary. Jigs for hole drilling are also within the model.

## Electrical
No electrical diagrams are included.  The simplicity of it doesn't necessitate one.  Connections go to motor controllers, relays, or the raspberry pi which control the components.  Screws are used as power distribution points which could be improved.  Power is first regulated to 12v and then again regulated down to 5v when necessary.  This is obviously inefficient but convenient.

## Logic
The python script runs on a Raspberry Pi 4.  It is excessive for it's role but could be augmented with computer vision or other features making it more of an automated platform.  It was my first python script.  I could not include some features like acceleration and deceleration curves due to the speed and interaction of event handling.  Again, it might be possible but I am not proficient enough with python.  

## Assembly
The assembly starts with the gearbox and gets somewhat complicated due to the herring bone gears.  There's only one way it can go together and be taken apart.

## Missing
Not depicted in the model:
Some bearings
All nuts and bolts (all m3, m4, or m5)
Wiring
3v relay
Headlight COB LED
Voltage regulators
Battery and socket
Pi



