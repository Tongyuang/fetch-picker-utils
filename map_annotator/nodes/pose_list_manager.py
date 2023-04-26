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
from map_annotator.msg import PoseNames,UserAction
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
        
    def _actionCallback(self,msg):
        '''
        The callback function for user action
        '''
        assert msg.command in ['create','goto','delete','rename']
        
        if msg.command == 'create':
            # create a new pos
            self.AddPose(msg.name)
            
        elif msg.command == 'goto':
            # goto pos
            self.SetGoal(msg.name)
        
        elif msg.command == "delete":
            # delete pos
            self.DelPose(msg.name)
        
        elif msg.command == "rename":
            # rename pos
            self._poserecord[msg.updated_name] = self._poserecord.pop(msg.name)
        
        


def wait_for_time():
    """
    Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
    


def main():
    rospy.init_node('map_annotator')
    wait_for_time()
    
    annotator_pub = rospy.Publisher('map_annotator/pose_names',
                                PoseNames,queue_size=1,latch=True)
    

    manager = PoseManager()
    rospy.sleep(0.5)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # get pos list
        pos_list = manager.ListPose()
        # create a message
        message = PoseNames()
        message.posenames = pos_list
        # send message
        annotator_pub.publish(message)
        # log
        #rospy.loginfo('Publishing pose list: {}'.format(pos_list))
        rate.sleep()
        
if __name__ == '__main__':
    main()