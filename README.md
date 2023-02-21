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


<h2> Code structure: </h2>

```python
#!/usr/bin/env/python2
import rospy
from sensor_msgs.msg import NavSatFix

# Initialize the node with name "subNodestat" and make it anonymous to avoid naming conflicts
rospy.init_node("subNodestat", anonymous=True)

# Define a Subscriber class to subscribe to a topic and receive messages
class subscriber():
    '''
            The subscriber class is defined to subscribe to a topic and receive messages. It takes the following parameters:

                topic: the name of the topic to subscribe to.
                data_type: the type of the messages to receive. It defaults to rospy.AnyMsg.
                max_calls: the number of consecutive calls without receiving a message before logging a warning. It defaults to 4.
                rate: the rate at which to check for messages. It defaults to 10 Hz.
                check_period: the number of times to check for messages for each topic before moving on to the next one. It defaults to 1.
                gps_status: a flag indicating whether to check the GPS accuracy. It defaults to False.

            The subscriber class has the following attributes:

                max_calls: the number of consecutive calls without receiving a message before logging a warning.
                topic: the name of the topic to subscribe to.
                count: the number of consecutive calls without receiving a message.
                data_type: the type of the messages to receive.
                working: a flag indicating whether the subscriber is working correctly.
                rate: the rate at which to check for messages.
                check_period: the number of times to check for messages for each topic before moving on to the next one.
                gps_status: a flag indicating whether to check the GPS accuracy.
                gps_accur: a flag indicating whether the GPS accuracy is sufficient.
                msg: the most recent message received.

            The subscriber class has the following methods:

                run_call: the callback function to run when a message is received on the subscribed topic. It saves the message data in the msg attribute and resets the count.
                get_rate: a helper function to return a rate object based on the rate parameter.
                spin: the main function to run the subscriber. It logs a warning if max_calls messages haven't been received, logs an info message if the subscriber is working correctly, and logs a warning if the GPS accuracy is low.
            '''
    def __init__(self, topic, data_type=rospy.AnyMsg, max_calls=4, rate=10, check_period=1, gps_status=False):
        # Initialize the subscriber with given parameters
        self.max_calls = max_calls
        self.topic = topic
        self.count = 0
        # Set data type to NavSatFix if gps_status is True, otherwise use the provided data_type
        if gps_status:
            self.data_type = NavSatFix
        else:
            self.data_type = data_type
        self.working = True
        self.rate = rate
        self.check_period = check_period
        self.gps_status = gps_status
        self.gps_accur = True
        self.msg = None
        # Subscribe to the specified topic with the defined data type and the run_call function as the callback
        rospy.Subscriber(self.topic, self.data_type, self.run_call)

    # Callback function to run when a message is received on the subscribed topic
    def run_call(self, data):
        # Save the message data in the msg attribute and reset the count
        self.msg = data
        self.count = 0

    # Helper function to return a rate object based on the rate parameter
    def get_rate(self):
        return rospy.Rate(self.rate)

    # Main function to run the subscriber
    def spin(self):
        loop_counter = 0
        #last_time = rospy.Time.now()
        while loop_counter < self.check_period or (self.check_period == -1):
            loop_counter += 1
            self.count += 1
            # If max_calls messages haven't been received, log a warning and set working to False
            if self.count >= self.max_calls:
                rospy.logwarn("[" + self.topic + "] No message received for %0.f calls on rate of " %self.count + "%0.f Hz." %self.rate )
                self.working = False
            # If working is False, log an info message and set it back to True
            elif not self.working:
                rospy.loginfo("[" + self.topic + "] Is working...")
                self.working = True
            # If gps_status is True and a message has been received, check the GPS status and log a warning if it's below 2
            if self.working and self.gps_status and (self.msg != None):
                if self.msg.status.status < 2:
                    rospy.logwarn("[" + self.topic + "] GPS status is %0.1f - low accuracy" %self.msg.status.status)
                    self.gps_accur = False
                elif (not self.gps_accur) and self.msg.status.status == 2:
                    self.gps_accur = True
                    rospy.loginfo("[" + self.topic + "] GPS status is %0.1f - sufficient accuracy" % self.msg.status.status)
            # Sleep based on the rate
            self.get_rate().sleep()
            #intrv = (rospy.Time.now() - last_time).to_sec()
            #print(intrv)
            #last_time = rospy.Time.now()

# Function to spin all the subscriber nodes
def spin_all(nodes):
    '''    
    The spin_all function is used to spin all the subscriber nodes.
    It takes a list of nodes as a parameter and runs spin for each node in the list.
    '''
    while not rospy.is_shutdown():
        for node in nodes:
            node.spin()
```

<h2> How to use the code: </h2>

<h> Frequency test for one topic </h>

```python
#frequency test for one topic, max_calls= how many consecutive calls before printing warnning,rate= hz, check_period= how many times to check the topic, gps_status= enable gps accuracy testing
node = subscriber("/lidar_tc/velodyne_points",max_calls=4,rate= 10, check_period=-1, gps_status=False)
node.spin() # repeat the node
```
```diff
- [WARN] [1677006717.499503]: [/lidar_tc/velodyne_points] No message received for 33 calls on rate of 10 Hz.
- [WARN] [1677006717.604131]: [/lidar_tc/velodyne_points] No message received for 34 calls on rate of 10 Hz.
+ [INFO] [1677006717.710579]: [/lidar_tc/velodyne_points] Is working...
```

<h> Multiple topic monitor </h>

```python
#message detector, max_calls= how many consecutive calls before printing warnning, check_period= how many times to check for each topic before going to the next one, gps_status= enable gps accuracy testing.
        nodes = [subscriber("/camera_fl/camera_info"),
                 subscriber("/camera_fl/image_color"),
                 subscriber("/camera_fr/camera_info"),
                 subscriber("/camera_fr/image_color"),
                 subscriber("/gps/fix", gps_status=True),
                 subscriber("/gps/gps"),
                 subscriber("/gps/imu"),
                 subscriber("/lidar_tc/velodyne_points"),
                 subscriber("/novatel/oem7/odom")]
        spin_all(nodes) # repeat for all nodes
```

```diff
- [WARN] [1677008006.403953]: [/camera_fl/camera_info] No message received for 8 calls on rate of 10 Hz.
- [WARN] [1677008006.508777]: [/camera_fl/image_color] No message received for 8 calls on rate of 10 Hz.
- [WARN] [1677008006.611975]: [/camera_fr/camera_info] No message received for 8 calls on rate of 10 Hz.
- [WARN] [1677008006.715667]: [/camera_fr/image_color] No message received for 9 calls on rate of 10 Hz.
- [WARN] [1677008006.820292]: [/gps/fix] No message received for 9 calls on rate of 10 Hz.
- [WARN] [1677008006.925539]: [/gps/gps] No message received for 9 calls on rate of 10 Hz.
- [WARN] [1677008007.027636]: [/gps/imu] No message received for 9 calls on rate of 10 Hz.
- [WARN] [1677008007.131011]: [/lidar_tc/velodyne_points] No message received for 9 calls on rate of 10 Hz.
- [WARN] [1677008007.237315]: [/novatel/oem7/odom] No message received for 9 calls on rate of 10 Hz.
+ [INFO] [1677008007.341311]: [/camera_fl/camera_info] Is working...
+ [INFO] [1677008007.444135]: [/camera_fl/image_color] Is working...
+ [INFO] [1677008007.546787]: [/camera_fr/camera_info] Is working...
+ [INFO] [1677008007.652143]: [/camera_fr/image_color] Is working...
+ [INFO] [1677008007.755100]: [/gps/fix] Is working...
+ [INFO] [1677008007.857931]: [/gps/gps] Is working...
+ [INFO] [1677008007.965180]: [/gps/imu] Is working...
+ [INFO] [1677008008.067254]: [/lidar_tc/velodyne_points] Is working...
+ [INFO] [1677008008.170661]: [/novatel/oem7/odom] Is working...
```
