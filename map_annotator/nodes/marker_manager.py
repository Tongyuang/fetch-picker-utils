#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   marker_manager.py
@Time    :   2023/04/26 18:04:48
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

# here put the import lib


import rospy
from map_annotator.msg import Poses,UserAction
from map_annotator import MarkerServer
from visualization_msgs.msg import InteractiveMarkerFeedback

class MarkerManager(MarkerServer):
    '''
    This class inherits from parent class MarkerServer, add a
    subscriber to the map_annotator/pose_names topic to manage
    markers
    '''
    def __init__(self):
        super().__init__()
        self._posSub = rospy.Subscriber("map_annotator/pose_names", Poses, self._positionCallback)
        self._actionPub = rospy.Publisher("/map_annotator/user_actions", UserAction, queue_size=1)
        
    def _positionCallback(self,msg):
        '''
        The callback function for user action
        '''
        if len(msg.posenames) > len(self.poselist):
            # add poses
            for (posename,posevalue) in zip(msg.posenames,msg.poses):
                if posename not in self.poselist:
                    self.poselist.append(posename)
                    self.addMarker(posename,posevalue,self.HandleRvizInput)
            
        elif len(msg.posenames) < len(self.poselist):
            # delete poses
            for name in self.poselist:
                if name not in msg.posenames:
                    self.deleteMarker(name)
        
                    
    def HandleRvizInput(self,input):
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            name = input.marker_name
            pose = input.pose
            message = UserAction()
            message.command = "relocate"
            message.name = name
            message.updated_pose = pose
            self._actionPub.publish(message)
        


def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
    


def main():
    rospy.init_node('map_annotator_marker_server')
    wait_for_time()
    manager = MarkerManager()
    rospy.sleep(0.5)
    
    rospy.spin()
        
if __name__ == '__main__':
    main()