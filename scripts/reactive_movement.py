#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist
from tf.transformations import euler_from_quaternion

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sound_play.libsoundplay import SoundClient


class TurtlebotController( object ):

  def __init__( self ):
    self.bridge = CvBridge()
    self.depth_image_np = None
    self.sound_handler = SoundClient( blocking = True )
    self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odom_cb )
    self.depth_img_sub = rospy.Subscriber( '/camera/depth/image', Image , self.depth_image_cb )
    self.cmd_vel_pub = rospy.Publisher( '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size = 10 )

  def odom_cb( self, msg ):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion( ( msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w ) )
    rospy.loginfo( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )

  def depth_image_cb( self, msg ):
    try:
      self.depth_image_np = self.bridge.imgmsg_to_cv2( msg )
    except CvBridgeError as e:
      rospy.logerr( e )

  def obtacle_detected( self ):
    obstacle = False
    if self.depth_image_np is not None:
      column_sample = [self.depth_image_np[x,0:540] for x in [10,250,470]]
      column_sample = np.where( np.isnan( column_sample ), 100.0, column_sample )
      obstacle = np.any( column_sample < 0.5 )
    return obstacle

  def run( self ):
    self.sound_handler.say( 'Hello human, welcome to Robotica Movil' )
    free_space = True
    while not rospy.is_shutdown():
      if self.obtacle_detected():
        # Rotate
        twist = Twist( Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5) )
        if free_space:
          free_space = False
          self.sound_handler.say( 'obstacle detected' )
      else:
        # Go forward
        twist = Twist( Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0) )
        free_space = True
      self.cmd_vel_pub.publish( twist )



if __name__ == '__main__':

  rospy.init_node( 'reactive_movement' )
  turtlebot = TurtlebotController()
  turtlebot.run()


