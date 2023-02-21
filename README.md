# ROS_Status_Page
Purpose: To provide information about the status of sensors and systems to aid test driver get a quick/easy overview during experiments

Modality: custom RVIZ plugin/panel(ros native visualization tool), custom GUI or (to get started) just print statements in the terminal

Programming Language: Your choice of Python2 or C++, check which one’s are supported by ROS melodic and Ubuntu 18.04 if you are unsure

Sensors: Lidar, Camera, GPS/IMU (INS). 
For now just one Lidar, 2 Front Cameras, one INS system

Desired Status: 
Publishing Frequency of sensors 
Camera/Lidar are expected to be around 10 Hz (for now it is acceptable to just check whether the Image topic is published at all) 
INS is expected to be around 50 Hz (for now it is acceptable to just check whether the Lidar topic is published at all) 
Status of positional accuracy (several modes are available, e.g. RTK is the most accurate mode but other modes may be acceptable)
Internet Connectivity
Check if the vehicle’s drive-by-wire system is active, engaged or errors are present
Check whether PTP (time synchronization protocol) is active and all sensors show that synchronization is active
…

In ROS, Data is published as “messages” via separate “Topics”. 
The message defines what data/parameters are published : for example Image, Header (time stamp, sequence etc.). Lidar: Message type: Pointcloud2
The topic is simply the datastream, this can be named as desired, for example Lidar: /lidar_tc/velodyne_points
These topic names are specific to our system, so we will sit down once you have gotten some time to look at this and go over it with actual data from the vehicle. 

By subscribing to a topic you receive the data at the rate it’s published (unless the program takes longer to execute) http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python) 
Other tutorials for ROS are available here: http://wiki.ros.org/ROS/Tutorials 
There are also several books/pdfs online that explain how ROS works. 

<h> How to use the code: </h>

<h2> Frequency test for one topic </h2>

```python
#frequency test for one topic, max_calls= how many consecutive calls before printing warnning,rate= hz, check_period= how many times to check the topic, gps_status= enable gps accurcy testing
a = subscriber("/lidar_tc/velodyne_points",max_calls=4,rate= 10, check_period=-1, gps_status=False)
a.spin() # repeat the node
```
terminal:
```
[`WARN`](#f1c40f) [1677006717.499503]: [/lidar_tc/velodyne_points] No message received for 33 calls on rate of 10 Hz.
[`WARN`](#f1c40f) [1677006717.604131]: [/lidar_tc/velodyne_points] No message received for 34 calls on rate of 10 Hz.
[`INFO`](#2ecc71) [1677006717.710579]: [/lidar_tc/velodyne_points] Is working...

```


<h2> Frequency test for one topic </h2>
