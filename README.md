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

<h2> Sample 1 </h2>


```python
#!/usr/bin/env python2

# Import the necessary ROS packages and message types
import rospy
from sensor_msgs.msg import Image

# Initialize the node with a unique name
rospy.init_node("subNodestat", anonymous=True)

# Define a class for detecting messages on a topic
class detection():
    
    # Constructor function that takes the topic name and time interval as arguments
    def __init__(self, topic, alpha):
        
        # Store the topic name and time interval as object variables
        self.alpha = alpha
        self.topic = topic
        
        # Set the publisher object to None
        self.publisher = None
        
        # Store the current time as the "last time" a message was received
        self.last_time = rospy.Time.now()

    # Function to run when a message is received on the topic
    def runTime(self, data):
        
        # Get the current time
        current_time = rospy.Time.now()
        
        # Calculate the elapsed time since the last message was received
        elapsed_time = (current_time - self.last_time).to_sec()
        
        # If the elapsed time is greater than the time interval, print a warning message
        if elapsed_time > self.alpha:
            rospy.logwarn("[" + self.topic + "] No message received for " + str(elapsed_time) + " seconds")
        
        # Update the "last time" variable to the current time
        self.last_time = current_time

    # Function to start listening to the topic
    def camera(self):
        
        # Subscribe to the topic and call the runTime function when a message is received
        image_sub = rospy.Subscriber(self.topic, Image, self.runTime)
        
        # If the publisher object is not None and has no connections, print an error message and set the publisher to None
        if self.publisher is not None and self.publisher.get_num_connections() == 0:
            rospy.logerr("[" + self.topic + "] has disconnected")
            self.publisher = None
        
        # Keep the node running and listening for messages
        rospy.spin()

# Check if the script is being run directly (as opposed to being imported as a module)
if __name__ == "__main__":
    
    try:
        # Create an instance of the detection class, passing in the topic name and time interval as arguments
        camera = detection('/camera_fl/image_color', 2)
        
        # Start listening to the topic
        camera.camera()
    
    except rospy.ROSInterruptException:
        # Catch any ROS interrupt exceptions (such as a user interrupting the program with Ctrl-C)
        pass

        
```

<h2> Sample 2 </h2>


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

<h3> Test 1 <\h3>

    
```python
#!/usr/bin/env/python2

import rospy
from sensor_msgs.msg import Image
import multiprocessing as mp
import threading as th
import time as t


class detection():
    def __init__(self, topic, dataType, alpha):
        self.alpha = alpha
        self.topic = topic
        self.publisher = None
        self.last_time = t.time()
        self.dataType = dataType
        self.working = True
        self.lock = th.Lock()
        rospy.init_node("subNodestat", anonymous=True)
        rospy.Subscriber(self.topic, self.dataType, self.runTime)


    def runTime(self, data):
        with self.lock:
            self.last_time = t.time()

    def checkTime(self):
        with self.lock:
            current_time = t.time()
            elapsed_time = current_time - self.last_time

            if elapsed_time > self.alpha:
                rospy.logwarn("[" + self.topic + "] No message received for " + str(elapsed_time) + " seconds")
                self.working = False
            elif not self.working:
                rospy.loginfo("[" + self.topic + "] Is working...")
                self.working = True
                self.last_time = t.time()

    def spin(self):
        try:
            while not rospy.core.is_shutdown():
                rospy.rostime.wallsleep(0.5)
                self.checkTime()
        except rospy.ROSInterruptException:
            pass

    def run(self):
        self.spin()


if __name__ == "__main__":
    try:
        node1 = detection('/camera_fl/image_color', Image, 10)
        node2 = detection('/camera_fr/image_color', Image, 10)

        p1 = mp.Process(target=node1.run)
        p2 = mp.Process(target=node2.run)

        p1.start()
        p2.start()

        p1.join()
        p2.join()

    except rospy.ROSInterruptException:
        pass
```
