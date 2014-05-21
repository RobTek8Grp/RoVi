#!/usr/bin/env python
"""
    Leon
"""
import rospy,tf
import numpy as np

from geometry_msgs.msg import PoseStamped
from group4_msgs.msg import PointCloudPose

class GPStest():
    def __init__(self):
        self.running = True
        self.calibrating = False
	self.data_received = False
        self.pose_id_max = 12
        
        self.robot_poses = []
        self.calibration_transforms = []
        self.pose_stamped_messages = []
        self.point_cloud_pose_messages = []
        self.pose_error = []
        
        # Init timer
        self.rate = rospy.Rate(10)
        
        # Init subscriber
        self.calibration_transform_sub = rospy.Subscriber('/point_cloud_assembler/calibration', PoseStamped, self.onPoseStamped )
        self.group4_sub = rospy.Subscriber('/robot_rx60b/carmine_pose', PointCloudPose, self.onPointCloudPose )
        
        
    def spin(self):
        while self.running:
	    if self.data_received:
		if self.point_cloud_pose_messages[-1].data == self.pose_id_max:
		  self.calculateTransform()
		  self.running = False

	    if rospy.is_shutdown() :
		    self.running = False
            
            # Block   
            try :
                self.rate.sleep()
            except rospy.ROSInterruptException:
                self.running = False

                
    def onPoseStamped(self,msg):
        matrix4x4 = tf.transformations.quaternion_matrix([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        matrix4x4[0,3] = msg.pose.position.x
        matrix4x4[1,3] = msg.pose.position.y
        matrix4x4[2,3] = msg.pose.position.z
        self.pose_stamped_messages.append(msg.pose_id)
        print msg.pose_id
        print matrix4x4
        
        
    def onPointCloudPose(self, msg):
	self.data_received = True
        matrix4x4 = tf.transformations.quaternion_matrix([msg.bumblebee_pose_left.orientation.x, msg.bumblebee_pose_left.orientation.y, msg.bumblebee_pose_left.orientation.z, msg.bumblebee_pose_left.orientation.w])
        matrix4x4[0,3] = msg.bumblebee_pose_left.position.x
        matrix4x4[1,3] = msg.bumblebee_pose_left.position.y
        matrix4x4[2,3] = msg.bumblebee_pose_left.position.z
        self.calibration_transforms.append(matrix4x4)
        self.point_cloud_pose_messages.append(msg.pose_id)
        #self.pose_id_max = msg.pose_id_max
    
    
    def calculateTransform(self):
        offset = 0
        tf_index = 0
        for pose in range(self.pose_id_max) :
            # offset += 1
            tf_calibration = np.identity(4, dtype=np.float64)
            for transform in range(self.pose_id_max):
                tf_index = (transform + offset) % self.pose_id_max
                tf_calibration = np.dot(tf_calibration,self.calibration_transforms[tf_index])
            self.pose_error.append(tf_calibration)
        
        tcp_to_cam = np.identity(4, dtype=np.float64)    
        for error in range(len(self.pose_error)):
            tcp_to_cam = np.dot(tcp_to_cam, self.pose_error[error])
        
        translation = tf.transformations.translation_from_matrix(tcp_to_cam)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_from_matrix(tcp_to_cam))
        print "x: " + str(translation[0])
        print "y: " + str(translation[1])
        print "z: " + str(translation[2])
        print "R: " + str(roll)
        print "P: " + str(pitch)
        print "Y: " + str(yaw)
        
            
if __name__ == '__main__':
    rospy.init_node('gps_test')
    test = GPStest( )
    test.spin()

