#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import gym_duckietown
from gym_duckietown.simulator import Simulator
import cv2
from cv_bridge import CvBridge

VEHICLE_NAME = os.environ['VEHICLE_NAME']

class ROSWrapper(DTROS):
    '''
    Class that creates a wrapper between the duckietown simulator and the ROS
    interface of a fake robot simulated in the host. The node serves as interface
    for the following parts:
        - Subscribes to the wheels command topic and transforms it into actions in simulation
        - Publishes the rendered observations in the simulator to the camera image topic
    '''

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ROSWrapper, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # construct rendered observation publisher
        self.camera_pub = rospy.Publisher(
            '~/'+ VEHICLE_NAME + '/camera_node/image/compressed',
            CompressedImage,
            queue_size=1
        )

        # Construct subscriber for wheel_cmd topic
        self.wheels_cmd_sub = rospy.Subscriber(
            '~/'+ VEHICLE_NAME + '/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            self.wheels_cmd_cb,
            queue_size=1
        )

        # Initialize the simulator 
        self.env = Simulator(
            seed=123, # random seed
            map_name="loop_empty",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4, # start close to straight
            full_transparency=True,
            distortion=True,
        )
        # Create action variable
        self.action = [0.0,0.0]

    def wheels_cmd_cb(self, msg):
        '''
        Callback that maps wheel commands into actions in the simulator.

            Creates the simulated robot actions based on the received command
            through the main topic in the duckietown infrastructure.

            Args:
                msg (WheelsCmdStamped): Velocity command
        '''

        vel_left = msg.vel_left
        vel_right = msg.vel_right

        self.action = [vel_left,vel_right]

        

    def run(self):
        '''
        Continuously run the wrapper to map ROS interface with simulator.

            This method runs the node, and begins an iterative process where the
            image is retrieved from the simulator and published in the camera image topic,
            and the wheels command received by subscribing to the wheels_cmd topic is translated
            into actions executed in the simulator. The process iterates until the node is terminated

        '''

        while not rospy.is_shutdown():
            
            # Update the simulator
            # the action message is updated every time the wheels_cmd_cb is called
            # that is, every time a message arrives.
            observation, reward, done, misc = self.env.step(self.action)
            # Render the new state
            self.env.render()

            # Set again action to stop, in case no more messages are being received
            self.action = [0.0,0.0]

            if done:
                # Reset the simulator when the robot goes out of the road.
                self.env.reset()

            # Transalte the observation into a CompressedImage message to publish it
            bgr_obs = cv2.cvtColor(observation,cv2.COLOR_RGB2BGR)
            bridge = CvBridge()
            img_msg = bridge.cv2_to_compressed_imgmsg(cvim=bgr_obs,dst_format="jpg")

            # Publish the image in the /image/compressed topic
            self.camera_pub.publish(img_msg)


if __name__ == '__main__':

    # create the node
    node = ROSWrapper(node_name='ros_wrapper_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()