# Modifying to make it __6dof__ with SparkFun OpenLog Artemis (DEV-16832)

Original Official ROS Documentation
--------------------------
https://github.com/ENSTABretagneRobotics/razor_imu_9dof

http://wiki.ros.org/razor_imu_9dof


Install and Configure ROS Package
---------------------------------
1) Install dependencies (for 3D visualization):

```bash
sudo apt-get install python-visual # For Ubuntu 16.04 and before
# For Ubuntu 18.04, install https://github.com/lebarsfa/visual/tree/bionic, or see https://github.com/ENSTABretagneRobotics/razor_imu_9dof/issues/47
sudo apt-get install python3-pip python3-wxgtk4.0 ; pip3 install vpython # From Ubuntu 20.04
```

2) Download code:

```bash
cd ~/catkin_workspace/src
git clone --branch skb-6dof https://github.com/ahmadSum1/razor_imu_9dof.git
cd ..
catkin_make
# For 3D visualization, from Ubuntu 20.04
cd src/razor_imu_9dof/nodes ; wget https://www.glowscript.org/docs/VPythonDocs/VPtoGS.py ; python3 VPtoGS.py ; cp -f Converted/display_3D_visualization.py display_3D_visualization.py ; cd ../../..
```

## Trouble Shooting

Dont forget to *"source YOUR_CATKIN_WORKSPACE/devel/setup.bash"*
```bash
RLException: [razor-pub-and-display.launch] is neither a launch file in package [razor_imu_9dof] nor is [razor_imu_9dof] a launch file name
The traceback for the exception was written to the log file
```
This **MIGHT BE** due to permission error [see](https://get-help.robotigniteacademy.com/t/error-cannot-launch-node-of-type-package-file-py-cant-locate-node-file-py-in-package-package/54)
```bash
chmod +x /path/to/imu_node.py  # replace '/path/to/imu_node.py' by the real path of your file
``` 


## Hardware/Sensor configuration
### Intended Change ..>> make compitable with default SparkFun OpenLog_Artemis firmware(v2.1)
 -- Changing the serial reader file to adopt to the one directional data flow (ros reader will assme data is being delivered in the expected format)
 Sensor configuration shell be done manually before.

https://learn.sparkfun.com/tutorials/openlog-artemis-hookup-guide

## *OLA Menu configuration: (after factory reset)*

  * Menu: Configure Terminal Output
    * "3) set Serial Terminal Baud rate -->> 57600"
    * "7) Output Actual Hertz -->> Disabled"    

  * Menu: Configure Time Stamp
    * "1) Log Time -->> Disabled"
    * "2) Log Date -->> Disabled"

  * Menu: Configure IMU
    * "12) DMP -->> Enabled"
    * "13) -->> Enabled"
    * "15) Accelerometer -->> Enabled"
    * "16) Gyro -->> Enabled"

so the serial data should look like:

 *"q1,q2,q3,Ax,Ay,Az,Gx,Gy,Gz,"*
 
## __DMP9(Quart9) is being avoided since the magnetometer has some issues(physical/SDK bug??) that ruins the Yaw__

##################x######################

Configure
---------
In its default configuration, ``razor_imu_9dof`` expects a yaml config file ``my_razor.yaml`` with:
* USB port to use
* Calibration parameters

An example``razor.yaml`` file is provided.
Copy that file to ``my_razor.yaml`` as follows:

```bash
roscd razor_imu_9dof/config
cp razor.yaml my_razor.yaml
```

Then, edit ``my_razor.yaml`` as needed

Launch
------
Publisher only:

*Apperantly you might need to source in the same terminal tab everytime before launching*

```bash
roslaunch razor_imu_9dof razor-pub.launch
```

Publisher and 3D visualization:

```bash
roslaunch razor_imu_9dof razor-pub-and-display.launch
```

Publisher only with diagnostics:

```bash
roslaunch razor_imu_9dof razor-pub-diags.launch
```

3D visualization only:

```bash
roslaunch razor_imu_9dof razor-display.launch
```


Calibrate
---------
For best accuracy, follow the tutorial to calibrate the sensors:

http://wiki.ros.org/razor_imu_9dof

An updated version of Peter Bartz's magnetometer calibration scripts from https://github.com/ptrbrtz/razor-9dof-ahrs is provided in the ``magnetometer_calibration`` directory.

Update ``my_razor.yaml`` with the new calibration parameters.

Dynamic Reconfigure
-------------------
After having launched the publisher with one of the launch commands listed above, 
it is possible to dynamically reconfigure the yaw calibration.

1) Run:

```bash
rosrun rqt_reconfigure rqt_reconfigure 
```
  
2) Select ``imu_node``. 

3) Change the slider to move the calibration +/- 10 degrees. 
If you are running the 3D visualization you'll see the display jump when the new calibration takes effect.

The intent of this feature is to let you tune the alignment of the AHRS to the direction of the robot driving direction, so that if you can determine that, for example, the AHRS reads 30 degrees when the robot is actually going at 35 degrees as shown by e.g. GPS, you can tune the calibration to make it read 35. It's the compass-equivalent of bore-sighting a camera.
