# SDK and Example for MIT motor control.


### USB2CAN Hardware:Canable

-canable (cantact clone): http://canable.io/ (STM32F042C6)

-canable modified:https://github.com/makerbase-mks/CANable-MKS  --selected

## Dependency:
```shell
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-noetic-can-msgs
sudo apt-get install ros-noetic-socketcan-bridge
```

### Use of slcan in Linux Ubuntu

1. Connect the MKS CANable V2.0 module to the ubuntu system, and send commands to query the device:
   
```shell
ls /dev/ttyACMx
```

2. Map ttyACM0 to can0, and set the bit rate to 1000K:

```shell
sudo slcand -o -c -s8 /dev/ttyACM0 can0
```

The corresponding setting of the bit rate is :
-s8 = 1M

3. Enable CAN:

```shell
sudo ifconfig can0 up
```

4. Configure the storage length of the transmission data buffer of txqueuelen:

```shell
sudo ifconfig can0 txqueuelen 1000
```

5. can0 send data: Enable the motor. For example:ID:0x001，data:0xFFFFFFFFFFFFFFFC：

```shell
cansend can0 001#FFFFFFFFFFFFFFFC
```

### Launch the launch file for the demo

```shell
roslaunch motor_control motor_test.launch
```
