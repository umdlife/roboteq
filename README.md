ULife roboteq driver
=======

This is a driver for serial Roboteq motor controllers.
It is based on [g/roboteq](http://github.com/g/roboteq) driver.

### Usage

```bash
roslaunch roboteq example.launch
```

The configuration is given as:

```xml
<rosparam>
  channels:
    - name: left # motor namespace
      encoder_ticks: 24 # number of encoders
      gearbox_divider: 9 # gearbox divider
    - name: right 
      encoder_ticks: 24
      gearbox_divider: 9
</rosparam>
```

The maximal RPM is read from the motor and used inside the script.

### Specs

* Feedback frequency: 111Hz
* Status frequency: 11Hz
* Expect Hall sensor only
* Expect 2 motors
* Tested on SBL2360
* The MBS script sets all the configurations of the motor controller.

### How it works

* The driver aknowledge the controller.
* The controller sends the status message
* The driver checks the status message script version.
	* If the script version is incorrect; the driver sends the new (compiled) version of the mbs script and restart it
* The controller sends Feedback and Status via ASCII handled by the driver and translated to ROS messages.
* The driver receives Float64 rad/s speeds, convert them to RPM on the motor and sends it to the controller. 

### How many ticks?

For a brushless motor without encoder: 

> Ticks = N_Poles \* Hall_Sensor_Number = 8\*3 = 24

### Credits
- AlexisTM (@AlexisTM)
- [g/roboteq](http://github.com/g/roboteq)
