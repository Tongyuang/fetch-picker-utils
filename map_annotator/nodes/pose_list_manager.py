#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File    :   pose_list_manager.py
@Time    :   2023/04/22 17:24:55
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
@Description : Robot position list manager
'''

import rospy
from map_annotator.msg import Poses,UserAction
from map_annotator import PoseRecorder

class PoseManager(PoseRecorder):
    '''
    This class inherits from parent class poserecorder, add a
    subscriber to the /map_annotator/user_actions topic to manage
    pose lists 
    '''
    def __init__(self):
        super().__init__()
        self._actionSub = rospy.Subscriber("/map_annotator/user_actions", UserAction, self._actionCallback)
        self.posePub = rospy.Publisher('map_annotator/pose_names',
                                Poses,latch=True,queue_size=10)
        self._publishpose()
        
    def _publishpose(self):
        '''
        publish poses
        '''
        message = Poses()
        message.posenames = self.ListPoseName()
        message.poses = self.ListPoseValue()   
        self.posePub.publish(message)   
          
    def _actionCallback(self,msg):
        '''
        The callback function for user action
        '''
        assert msg.command in ['create','goto','delete','rename','relocate']

        if msg.command == 'create':
            # create a new pos
            self.AddPose(msg.name)
            self.save()
            
        elif msg.command == 'goto':
            # goto pos
            self.SetGoal(msg.name)
        
        elif msg.command == "delete":
            # delete pos
            self.DelPose(msg.name)
            self.save()
        
        elif msg.command == "rename":
            # rename pos
            self._poserecord[msg.updated_name] = self._poserecord.pop(msg.name)
            self.save()
            
        elif msg.command == "relocate":
            # relocate
            self._poserecord[msg.name] = msg.updated_pose
            self.save()
        self._publishpose()

        


def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
    


def main():
    rospy.init_node('map_annotator_pose_list_manager')
    wait_for_time()
    
    manager = PoseManager()
    rospy.sleep(0.5)
    rospy.spin()
    # rate = rospy.Rate(2)
    # while not rospy.is_shutdown():
    #     # create a message
    #     message = Poses()
    #     message.posenames = manager.ListPoseName()
    #     message.poses = manager.ListPoseValue()
    #     # send message
    #     annotator_pub.publish(message)
    #     # log
    #     #rospy.loginfo('Publishing pose list: {}'.format(pos_list))
    #     rate.sleep()
        
if __name__ == '__main__':
    main()