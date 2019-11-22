#!/usr/bin/env python

import rospy
import rospkg
import smach
import time
from sensor_msgs.msg import JointState

# Manipulator 
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

class fileAction(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['input_planning_group',
                                                'input_motionfile_file'])

        self.set_joint_position = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path', SetJointPosition)
        self.set_gripper_position = rospy.ServiceProxy('/open_manipulator/goal_tool_control', SetJointPosition)
        self.count = 0
        self.use_platform = rospy.get_param("~use_platform")

    def execute(self, userdata):
        inputfile = userdata.input_motionfile_file
        self.count = 0
	while 1:
            if rospy.is_shutdown() :
                break
            if self.count > ( len(inputfile) - 1 ):
                break

	    if self.count == 0 :
                position_path_time = 2 
                gripper_path_time = 1 
                operating_time = 2
            else :
                position_path_time = 0.5
                gripper_path_time = 0.5
                operating_time = 0.2

            try: 
	        joint_position = JointPosition()
	        planning_group = userdata.input_planning_group
	        joint_position.joint_name = ['joint1','joint2','joint3','joint4']                
                joint_position.position =  [inputfile[self.count][0], inputfile[self.count][1], \
                                            inputfile[self.count][2], inputfile[self.count][3]]
                path_time = position_path_time
                resp1 = self.set_joint_position(planning_group,joint_position, position_path_time)
	        #print 'resp1 {}'.format(resp1.is_planned) 
	        #rospy.sleep(position_path_time) 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            try: 
	        gripper_position = JointPosition()
	        planning_group = userdata.input_planning_group
	        gripper_position.joint_name = ['gripper']
                if self.use_platform :                    
                    gripper_position.position =  [inputfile[self.count][4]]
                else :
                    if inputfile[self.count][4] < 0.005 :
                        gripper_position.position =  [-0.01]
                    else :
                        gripper_position.position =  [0.01]
           
                path_time = gripper_path_time
                resp1 = self.set_gripper_position(planning_group,gripper_position, gripper_path_time)
	        #print 'resp1 {}'.format(resp1.is_planned) 
	        #rospy.sleep(0.2) 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            #self.count = self.count + 5
            self.count = self.count + 8
            rospy.sleep(operating_time)

	return 'succeeded'
