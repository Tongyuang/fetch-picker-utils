#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy
from sensor_msgs.msg import JointState                                                                                      
    
TOPIC_NAME = '/joint_states'                                                                                                   
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """ 
                                                                                                   
    def __init__(self):
        
        # store all the names and states
        self.joint_names = ['l_wheel_joint','r_wheel_joint','torso_lift_joint','bellows_joint',
                      'head_pan_joint','head_tilt_joint','shoulder_pan_joint','shoulder_lift_joint',
                      'upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint',
                      'wrist_roll_joint','l_gripper_finger_joint','r_gripper_finger_joint']
        self.joint_states_positions = {name:0.0 for name in self.joint_names}
        self.joint_state_velocities = {name:0.0 for name in self.joint_names}
        self.len_joint = len(self.joint_names)
        # create a subscriber
        rospy.Subscriber(TOPIC_NAME, JointState, self.joint_state_callback)
        
    def get_all_names(self):
        return self.joint_names
     
    def joint_state_callback(self,msg):
        # update joint states
        for i in range(self.len_joint):
            name = msg.name[i]
            self.joint_states_positions[name] = float(msg.position[i])
            self.joint_state_velocities[name] = float(msg.velocity[i])                                                                                           
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                           
        if name not in self.joint_names:
            warn_msg = "name not recognized in joint_state names, expected to in[" + \
                        ", ".join(self.names) + \
                        "], got {}, return 0 instead.".format(name)
            rospy.logwarn(warn_msg)                                                        
            return 0 
        return self.joint_states_positions[name]                                                                                      
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """  
        # check names
        for name in names:
            if name not in self.joint_names:
                warn_msg = "some name not recognized in joint_state names, expected to in[" + \
                        ", ".join(self.names) + "], return some 0s instead."
                rospy.logwarn(warn_msg)
                break   
        # if name not in self.joint_names, return 0
        res = []
        for name in names:
            res.append(self.joint_states_positions[name] if name in self.joint_names else 0)          
        return res                                     