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

<h2> One Topic </h2>


```python
#!/usr/bin/env/python2

import rospy
from sensor_msgs.msg import Image

# Initialize a ROS node with the name "subNodestat"
rospy.init_node("subNodestat", anonymous=True)


# Define a class for the node behavior
class detection():
    def __init__(self, topic, dataType, alpha):
        """
        Constructor for the detection class.

        Arguments:
        topic -- ROS topic to subscribe to
        dataType -- type of ROS message to subscribe to
        alpha -- time limit in seconds for which the node waits for a message before sending a warning
        """
        self.alpha = alpha
        self.topic = topic
        self.publisher = None
        self.last_time = rospy.Time.now()
        self.dataType = dataType
        self.working = True

    def runTime(self, data):
        """
        Callback function for the subscriber.

        Arguments:
        data -- ROS message received by the subscriber
        """
        self.last_time = rospy.Time.now()

    def checkTime(self):
        """
        Check the time elapsed since the last message was received.
        Log a warning message if the elapsed time exceeds the time limit.
        """
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_time).to_sec()

        if elapsed_time > self.alpha:
            rospy.logwarn("[" + self.topic + "] No message received for " + str(elapsed_time) + " seconds")
            self.working = False
        elif not self.working:
            rospy.loginfo("[" + self.topic + "] Is working...")
            self.working = True

    def spin(self):
        """
        Loop until the node is shutdown by ROS.
        Call checkTime() repeatedly with a delay of 0.5 seconds.
        """
        try:
            while not rospy.core.is_shutdown():
                rospy.rostime.wallsleep(0.5)
                self.checkTime()
        except rospy.ROSInterruptException:
            pass

    def subscriber(self):
        """
        Subscribe to the ROS topic specified by self.topic.
        Set self.runTime as the callback function.
        Call self.spin().
        """
        image_sub = rospy.Subscriber(self.topic, self.dataType, self.runTime)
        self.spin()


# Main block
if __name__ == "__main__":
    try:
        # Create an instance of the detection class with topic '/camera_fl/image_color', message type 'Image', and time limit of 1 second.
        camera = detection('/camera_fl/image_color', Image, 1)
        # Call the subscriber() method of the instance.
        camera.subscriber()
    except rospy.ROSInterruptException:
        pass

```


<h2> Multiple Topics </h2>


```python
#!/usr/bin/env/python2

# Import the necessary ROS packages and message types
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu, PointCloud2
from gps_common.msg import GPSFix
from nav_msgs.msg import Odometry

# Initialize the ROS node with a unique name
rospy.init_node("subNodestat", anonymous=True)

# Define a class for monitoring message reception from a topic
class detection():
    def __init__(self, topic, dataType, alpha):
        self.alpha = alpha    # Maximum allowed time (in seconds) without receiving a message
        self.topic = topic    # Name of the topic to monitor
        self.publisher = None
        self.count = 0        # Counter for elapsed time (in seconds) since the last message was received
        self.dataType = dataType  # Type of message to expect on the topic
        self.working = True   # Flag indicating whether the topic is currently working
        rospy.Subscriber(self.topic, self.dataType, self.runTime)

    # Callback function to reset the timer when a new message is received
    def runTime(self, data):
        self.count = 0

# Define a function to monitor multiple topics for message reception
def spin_all(nodes):
    rate = rospy.Rate(1)  # Set the loop rate to 1 Hz
    while not rospy.is_shutdown():
        # Update the counters for each monitored topic
        for node in nodes:
            node.count += 1

        # Check if the maximum allowed time has elapsed since the last message for each topic
        for node in nodes:
            if node.count >= node.alpha:
                # Log a warning message if the topic has stopped publishing messages
                rospy.logwarn("[" + node.topic + "] No message received for " + str(node.count) + " seconds")
                node.working = False
            elif not node.working:
                # Log an info message when the topic resumes publishing messages
                rospy.loginfo("[" + node.topic + "] Is working...")
                node.working = True
        rate.sleep()

    # Keep the node running until it is shutdown
    rospy.spin()

# Main program
if __name__ == "__main__":
    try:
        # Create instances of the detection class for each topic to monitor
        nodes = [detection("/camera_fl/camera_info", CameraInfo, 4),
                 detection("/camera_fl/image_color", Image, 4),
                 detection("/camera_fr/camera_info", CameraInfo, 4),
                 detection("/camera_fr/image_color", Image, 4),
                 detection("/gps/fix", NavSatFix, 4),
                 detection("/gps/gps", GPSFix, 4),
                 detection("/gps/imu", Imu, 4),
                 detection("/lidar_tc/velodyne_points", PointCloud2, 4),
                 detection("/novatel/oem7/odom", Odometry, 4)]
        # Start monitoring the topics
        spin_all(nodes)
    except rospy.ROSInterruptException:
        pass
```


<h2> Multiple Topics with Rates </h2>


```python
#!/usr/bin/env/python2
# This is a shebang line that specifies the interpreter to use to run the script.

# Import the necessary ROS packages and message types
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu, PointCloud2
from gps_common.msg import GPSFix
from nav_msgs.msg import Odometry

# Initialize the ROS node
rospy.init_node("subNodestat", anonymous=True)

# Define a subscriber class that keeps track of whether the subscriber is working or not
class subscriber():
    def __init__(self, topic, data_type, max_calls, rate=10):
        # Initialize the subscriber with the given topic, message type, maximum number of calls without receiving a message, and rate
        self.max_calls = max_calls
        self.topic = topic
        self.publisher = None
        self.count = 0
        self.data_type = data_type
        self.working = True
        self.rate = rospy.Rate(rate)
        # Subscribe to the topic and call the run_call method when a message is received
        rospy.Subscriber(self.topic, self.data_type, self.run_call)

    def run_call(self, data):
        # Reset the count when a message is received
        self.count = 0

# Define a function that loops over all subscribers and checks if they are working properly
def spin_all(nodes):
    while not rospy.is_shutdown():
        for node in nodes:
            # Increment the count for the number of calls without receiving a message
            node.count += 1
            if node.count >= node.max_calls:
                # If the maximum number of calls has been reached, log a warning message and mark the subscriber as not working
                rospy.logwarn("[" + node.topic + "] No message received for " + str(node.count) + " calls")
                node.working = False
            elif not node.working:
                # If the subscriber was not working but has started working again, log an info message and mark the subscriber as working
                rospy.loginfo("[" + node.topic + "] Is working...")
                node.working = True
            # Sleep for the specified rate
            node.rate.sleep()

if __name__ == "__main__":
    try:
        # Create a list of subscribers with their respective topics, message types, maximum number of calls, and rate
        nodes = [subscriber("/camera_fl/camera_info", CameraInfo, 3 ),
                 subscriber("/camera_fl/image_color", Image, 3 ),
                 subscriber("/camera_fr/camera_info", CameraInfo, 3 ),
                 subscriber("/camera_fr/image_color", Image, 3 ),
                 subscriber("/gps/fix", NavSatFix, 3, 50),
                 subscriber("/gps/gps", GPSFix, 3, 50),
                 subscriber("/gps/imu", Imu, 3, 100),
                 subscriber("/lidar_tc/velodyne_points", PointCloud2, 3, 10),
                 subscriber("/novatel/oem7/odom", Odometry, 3, 50)]
        # Loop over all subscribers and check if they are working properly
        spin_all(nodes)
    except rospy.ROSInterruptException:
        pass
```
