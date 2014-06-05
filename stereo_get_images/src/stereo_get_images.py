#!/usr/bin/env python

## original inspiration for ros message passing:  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys
# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

#sys.path.remove('/opt/ros/hydro/lib/python2.7/dist-packages')
#sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')

# OpenCV
import cv2
         
import time

# Ros libraries
#import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage #, Image
#from cv_bridge import CvImage
#import sensor_msgs.point_cloud2 as pc2
#from sensor_msgs.msg import PointCloud2, PointField
# We do not use cv_bridge it does not support CompressedImage in python
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import PointCloud2, PointField
from group4_msgs.msg import PointCloudPose

print
print "Everything's go!"

class image_get:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/stereo_test_disparity/disparity_image",
        #    CompressedImage)
        #self.pub_cloud = rospy.Publisher("/output/stereo_test_disparity/points", PointCloud2)
        self.pub_group = rospy.Publisher("/robot_rx60b/bumblebee_image_pose", PointCloudPose)
        # self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.LEFT_IMAGE_RECIEVED = False
        self.left_header = 0
        self.right_header = 1
        self.group_4_recieved = False
        self.group_4 = None
        
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/stereo_camera/left/image_rect_color/compressed",
            CompressedImage, self.callback_left,  queue_size = 1)
        self.subscriber = rospy.Subscriber("/stereo_camera/right/image_rect_color/compressed",
            CompressedImage, self.callback_right_with_3D,  queue_size = 1)        
        self.subscriber = rospy.Subscriber("/robot_rx60b/camerapose",
            PointCloudPose, self.callback_PointCloudPose,  queue_size = 1) 

        print "initialized"

    def callback_PointCloudPose(self, ros_data):
	'''Callback function of subscribed topic. 
        Here images get converted and features detected'''  
        print "pose"
        self.group_4 = ros_data
        self.group_4_recieved = True


    def callback_left(self, ros_data):
        '''Callback function of subscribed topic. 
        converts the left image and saves it'''
        self.left_header = ros_data.header.stamp.secs

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, 1) # 1 = cv2.CV_LOAD_IMAGE_COLOR
               
        #set the image as recieved
        self.left_image = image_np
        self.LEFT_IMAGE_RECIEVED = True
 

    #def callback_right(self, ros_data):
    def callback_right_with_3D(self, ros_data):
	'''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        self.right_header = ros_data.header.stamp.secs        
        
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, 1)

        # convert np image to grayscale
        gray_image = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        if self.right_header == self.left_header and self.group_4_recieved:
            s_t = time.clock()
            print self.right_header, self.left_header
            self.group_4_recieved = False

            left_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)

            group_4_out = self.group_4
            
#            PointCloudPose()
#            group_4_out.header = self.group_4.header
#std_msgs/Int16 pose_id
#std_msgs/Int16 pose_id_max
#sensor_msgs/CompressedImage image_left
#sensor_msgs/CompressedImage image_right
#geometry_msgs/Pose spin_center_pose
#sensor_msgs/PointCloud2 carmine_pointcloud 
#geometry_msgs/Pose carmine_pose
#sensor_msgs/PointCloud2 bumblebee_pointcloud
#geometry_msgs/Pose bumblebee_pose_left
#geometry_msgs/Pose bumblebee_pose_right            
            
            

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', left_gray)[1]).tostring()
            # Publish new image
            group_4_out.image_left = msg
            
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', gray_image)[1]).tostring()
            # Publish new image
            group_4_out.image_right = msg
            
            #group_4_out.bumblebee_pointcloud = pcloud            
            self.pub_group.publish(group_4_out)

            print "save time", time.clock() - s_t       

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_get()
    rospy.init_node('save_images_to_group4', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
