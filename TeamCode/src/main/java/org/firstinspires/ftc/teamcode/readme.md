# IMPORTANT notes you should read
If you are having issues with something, check here first

## Launcher
- Flywheel connects to ***Control Hub*** motor port 0
- Windmill servo (4-sided ball mover servo) connects to port 2 on the ***control hub***
- If launcher goes too high, **check feedback wires**.
- The motors used in the FIRST Tech Challenge have encoders with a resolution of 28 ticks per revolution. 
  - To convert this to RPM, divide the value by 28, to get to revolutions per second, before 
  - multiplying by 60 to get revolutions per minute.

## Intake
Connects to ***Expansion Hub*** motor port 0

## Drivetrain
- Front motors: Port 2
- Back motors: Port 1
- Left motors: Expansion Hub
- Right motors: Control Hub
- Odometry pods connect to I²C port 0

### Odometry Pod Error Codes
- READY: the device is working as normal
- CALIBRATING: the device is calibrating and outputs are put on hold
- NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
- FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
- FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
- FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
- FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported

## Operation
Control sticks:
- Left control stick is pan
- Right control stick is rotate (sideways)

Buttons:
- Square is field-centric
- Triangle is robot-centric
- Circle is reset position and heading to 0
- Cross is re-calibrate IMU

- Right bumper: Launcher
- Right trigger: Intake in
- Left trigger: Intake reverse

*Note: Position robot facing away from driver at program start or before resetting position*



___
# General Information

## Creating OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
of a particular style of OpMode.  These are bare-bone examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
It is not intended to drive a functioning robot, it is simply showing the minimal code
required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
It may be used to provide a common baseline driving OpMode, or
to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
These may be complex, but their operation should be explained clearly in the comments,
or the comments should reference an external doc, guide or tutorial.
Each OpMode should try to only demonstrate a single concept so they are easy to
locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

1) Locate the desired sample class in the Project/Android tree.

2) Right-click on the sample class and select "Copy"

3) Expand the  TeamCode/java folder

4) Right-click on the org.firstinspires.ftc.teamcode folder and select "Paste"

5) You will be prompted for a class name for the copy.
   Choose something meaningful based on the purpose of this class.
   Start with a capital letter, and remember that there may be more similar classes later.
