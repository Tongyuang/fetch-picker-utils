#!/usr/bin/env python3                                                                                  
# -*- encoding: utf-8 -*-
'''
@File    :   recorder.py
@Time    :   2023/04/21 17:15:38
@Author  :   Reon(Yuang) Tong 
@Contact :   yuangtong1999@gmail.com
'''
           
# here put the import lib
                                                                                                       
import rospy
import pickle
import os
from collections import defaultdict
import copy

# msg
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped # this is the position messages used                                                                            
                                                                                                       
class PoseRecorder():                                                                        
    """Listens to /amcl to record the pose of the robot                       
                                                                                                       
    Usage:
        recorder = PoseRecorder()
        recorder.                                                                   
    """                                                                                                
    def __init__(self):                                                                                
        # setup data dir
        self._DatabaseSetup()
        self._LatestPos = None
        # create a publisher
        self._RecordPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        
        
        
    def _DatabaseSetup(self):
        # set up the database, now use pickle instead
        self._defaultDataStoreDir = '/home/yuangtong/catkin_ws/src/fetch_utils/map_annotator/src/data'
        self._defaultDataStoreName = 'poserecord.pkl'
        # load the dataro
        data_path = os.path.join(self._defaultDataStoreDir,self._defaultDataStoreName)
        if not os.path.exists(data_path):
            # needed to create a new file to store the database
            self._poserecord = defaultdict()
        else:
            # load the previous stored data
            with open(data_path,'rb') as rf:
                self._poserecord = pickle.load(rf)     
                rf.close()  
        
    
    def _GetCurrentPos(self):
        try:
            msg = rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped,timeout=1)
            self._LatestPos = msg.pose.pose
        except:
            rospy.logerr('Cannot receive message from /acml_pose topic.')
                                             
    def ListPoseName(self):
        '''
        list the names of all the position stored
        '''
        namelist = list(self._poserecord.keys())
        return namelist
    
    def ListPoseValue(self):
        '''
        list the values of all the position stored
        '''
        posenames = self.ListPoseName()
        poselist = []
        for name in posenames:
            poselist.append(self._poserecord[name])
        return poselist
    
    def AddPose(self,name):
        '''
        Save the robot's current pose as <name>. Overwrites if <name> already exists.
        '''
        self._GetCurrentPos()
        pose = copy.deepcopy(self._LatestPos)
        self._poserecord[name] = pose
    
    def DelPose(self,name):
        '''
        Delete the pose given by <name>
        '''
        if name in self._poserecord.keys():
            del self._poserecord[name]
        else:
            print("No such pose {}".format(name))
    
    def SetGoal(self,name):
        '''
        Sends the robot to the pose given by <name>.
        '''
        if name in self._poserecord.keys():
            goal = PoseStamped()
            goal.header.frame_id = "map"  # Set the frame_id to "map" for a global goal
            goal.header.stamp = rospy.Time.now()
            goal.pose = copy.deepcopy(self._poserecord[name])
            self._RecordPub.publish(goal)
            rospy.loginfo("new position name: {} confirmed. x:{},y:{},z:{}".format(
                name,self._poserecord[name].position.x,self._poserecord[name].position.y,self._poserecord[name].position.z
            ))
    
    def save(self):
        data_path = os.path.join(self._defaultDataStoreDir,self._defaultDataStoreName)
        with open(data_path,'wb') as wf:
            pickle.dump(self._poserecord, wf)   
            wf.close()         
        