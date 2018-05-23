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

The motor speed is set by sending the **rad/s** as `std_msgs/Float64` to `~/namespace/cmd`

```bash
rostopic pub /left/cmd std_msgs/Float64 "data: 30.0" -r 10
```

You receive the feedback as `roboteq_msgs/Feedback` at `~/namespace/feedback`

```python
Header header
float32 cmd_user     # Rad/s command 
float32 cmd_motor    # % of the motor power used (0.7 is 70%)
float32 measured_vel # Rad/s speed
float32 measured_pos # Total radians rotated
```

You receive the status as `roboteq_msgs/Status` at `~/status`

```python
Header header

uint8 FAULT_OVERHEAT=1
uint8 FAULT_OVERVOLTAGE=2
uint8 FAULT_UNDERVOLTAGE=4
uint8 FAULT_SHORT_CIRCUIT=8
uint8 FAULT_EMERGENCY_STOP=16
uint8 FAULT_SEPEX_EXCITATION_FAULT=32
uint8 FAULT_MOSFET_FAILURE=64
uint8 FAULT_STARTUP_CONFIG_FAULT=128
uint8 fault

uint8 STATUS_SERIAL_MODE=1
uint8 STATUS_PULSE_MODE=2
uint8 STATUS_ANALOG_MODE=4
uint8 STATUS_POWER_STAGE_OFF=8
uint8 STATUS_STALL_DETECTED=16
uint8 STATUS_AT_LIMIT=32
uint8 STATUS_MICROBASIC_SCRIPT_RUNNING=128
uint8 status

# Temperature of main logic chip (C)
float32 ic_temperature

# Internal supply and reference voltage (V)
float32 battery_voltage

# Max RPM of the motors
int32 max_rpm_motor1
int32 max_rpm_motor2

# Battery current sensed
float32 bat_current_1
float32 bat_current_2
```


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
