#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File    :   interactive_teleop_demo.py
@Time    :   2023/04/24 14:33:28
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

# here put the import lib

import rospy
import robot_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


from std_msgs.msg import ColorRGBA,Header
from geometry_msgs.msg import Point,PoseWithCovarianceStamped,Quaternion
from nav_msgs.msg import Odometry

import copy
import math
import tf.transformations as tft

class MarkerServer():
    def __init__(self):
        self.server = InteractiveMarkerServer("/map_annotator/map_poses")
        
        self.fixed_frame = "map" # /map or /odom
        self.poselist = []
        
        
    def CreateInteractiveMarker(self,
                                position = Point(x=1, y=0, z=0),
                                orientation = Quaternion(x=0,y=0,z=0,w=1),
                                arrowcolor = ColorRGBA(r=0.2, g=0.45, b=0.9, a=1.0),
                                ringcolor = ColorRGBA(r=0.1,g=0.9,b=0.05,a=0.6),
                                name = 'nm', 
                                description='desc'):
        '''Create an interactive button
            param color: ColorRGBA from std msg
            param position: Point from geometry_msgs.msg
            param name: Name of the marker
        '''
        # create an arrow marker
        arrowMarker = Marker()
        # set the frame id
        arrowMarker.header.frame_id = self.fixed_frame
        # set the type of marker
        arrowMarker.type = Marker.ARROW # make it an arrow
        # set the scales
        arrowMarker.scale.x = 0.5
        arrowMarker.scale.y = 0.05
        arrowMarker.scale.z = 0.05
        # set orientation
        arrowMarker.pose.orientation= orientation
        # set position
        arrowMarker.pose.position = position
        # set color
        arrowMarker.color = arrowcolor
        
        # create an sylinder marker
        cylinderMarker = Marker()
        # set the frame id
        cylinderMarker.header.frame_id = self.fixed_frame
        # set the type of marker
        cylinderMarker.type = Marker.CYLINDER # make it an cylinder
        # set scale
        cylinderMarker.scale.x = 2*arrowMarker.scale.x
        cylinderMarker.scale.y = cylinderMarker.scale.x
        cylinderMarker.scale.z = 0.01
        # set color
        cylinderMarker.color = ringcolor
        # set position       
        cylinderMarker.pose.position = position
        cylinderMarker.pose.orientation= orientation
        
        # arrowControl
        arrowControl = InteractiveMarkerControl()
        # set control mode
        arrowControl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # set orientation
        arrowControl.orientation.w = 1
        arrowControl.orientation.y = 1
        # set it always visible
        arrowControl.always_visible = True
        # add marker
        arrowControl.markers.append(arrowMarker)
        
        
        # cylinderControl
        cylinderControl = InteractiveMarkerControl()
        # set control mode
        cylinderControl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # set orientation
        cylinderControl.orientation.w = 1
        cylinderControl.orientation.y = 1
        # set it always visible
        cylinderControl.always_visible = True
        # add marker
        cylinderControl.markers.append(cylinderMarker)
                
        
        # create an interactive marker
        ThisInteractiveMarker = InteractiveMarker()
        # set fixed frame
        ThisInteractiveMarker.header.frame_id = self.fixed_frame
        # set time stamp
        ThisInteractiveMarker.header.stamp = rospy.Time.now()
        # set name
        ThisInteractiveMarker.name = name
        # set description
        ThisInteractiveMarker.description = description
        
        # set orientation
        ThisInteractiveMarker.pose.orientation = orientation
        # set scale
        ThisInteractiveMarker.scale = 1
        # set position
        ThisInteractiveMarker.pose.position = position
        # add control method
        ThisInteractiveMarker.controls.append(arrowControl)
        ThisInteractiveMarker.controls.append(cylinderControl)
        
        return ThisInteractiveMarker

    def addMarker(self,posename,pose,controlfunc):
        marker = self.CreateInteractiveMarker(position = pose.position,
                                                orientation = pose.orientation,
                                                name = posename,
                                                description = posename)
        self.server.insert(marker,controlfunc)
        self.server.applyChanges()
    
    def deleteMarker(self,posename):
        if self.server.erase(posename):
            self.server.applyChanges()
        
    def updateMarker(self,posename,pose,controlfunc):
        # self.deleteMarker(posename)
        # self.addMarker(posename,pose,controlfunc)
        if self.server.setPose(posename,pose):
            self.server.applyChanges()
       
    def HandleRvizInput(self,input):
        # handles rviz input
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            name = input.marker_name
            pose = input.pose
            
            rospy.loginfo("Changing marker {} to new position:({},{})".format(
                name,pose.position.x,pose.position.y
            ))

    
    
def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('Interactive_Marker_demo')
    wait_for_time()
    Marker = MarkerServer()
    rospy.sleep(0.5)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()