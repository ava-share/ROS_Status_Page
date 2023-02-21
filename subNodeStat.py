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

# Main function to run the subscriber nodes
if __name__ == '__main__':
    #Frequency test for one topic
    '''
    # frequency test for one topic, max_calls= how many consecutive calls before printing warnning,rate= hz, check_period= how many times to check the topic, gps_status= enable gps accuracy testing
    node = subscriber("/lidar_tc/velodyne_points", max_calls=4, rate=10, check_period=-1, gps_status=False)
    node.spin()  # repeat the node
    '''
    # Multiple topic monitor
    '''
    # message detector, max_calls= how many consecutive calls before printing warnning, check_period= how many times to check for each topic before going to the next one, gps_status= enable gps accuracy testing.
    nodes = [subscriber("/camera_fl/camera_info"),
             subscriber("/camera_fl/image_color"),
             subscriber("/camera_fr/camera_info"),
             subscriber("/camera_fr/image_color"),
             subscriber("/gps/fix", gps_status=True),
             subscriber("/gps/gps"),
             subscriber("/gps/imu"),
             subscriber("/lidar_tc/velodyne_points"),
             subscriber("/novatel/oem7/odom")]
    spin_all(nodes)  # repeat for all nodes
    '''







