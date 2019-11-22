#!/usr/bin/env python

import rospy
import rospkg

import smach
from std_msgs.msg import String

# Manipulator 
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.srv import GetJointPosition
from open_manipulator_msgs.srv import GetKinematicsPose

import time
from sensor_msgs.msg import JointState

class trackingAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['input_planning_group'])
	self.object_pose_sub = rospy.Subscriber('/facedetectdataset', String, self.collect)
        #self.cupboundingBox = BoundingBox()

        # this is for Joint Control 
        self.set_joint_position = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path', SetJointPosition)
        # this is for Joint Control from Current Position  
	self.set_joint_position_from_present = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path_from_present', SetJointPosition)
        # to read current joint value
        self.open_manipulator_joint_states_sub_ = rospy.Subscriber('/open_manipulator/joint_states', JointState, self.jointStatesCallback)

        self.command_Key_sub = rospy.Subscriber('command_Key', String, self.commandKeyCallback)
        self.mode = 9
        self.last_detect_time = rospy.get_rostime()
        self.jointStates = [0.0, 0.0, 0.0, 0.0]
        self.gripperStates = [0.0, 0.0]
	self.kinematicsStates = [0.0, 0.0, 0.0]
        self.facedata = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.face_center_x = 320
        self.face_center_y = 240
        self.last_no_detect_time = rospy.get_rostime()
        self.no_detect_count = 0
        self.face_width = 0

    def jointStatesCallback(self, msg):
        self.jointStates[0] = msg.position[0]
        self.jointStates[1] = msg.position[1]
        self.jointStates[2] = msg.position[2]
        self.jointStates[3] = msg.position[3]

    def collect(self, msg):
        #face tracking data
        #rospy.logwarn('camera data %s', msg.data )
        self.facedata = [int (i) for i in msg.data.split(",")]
        self.face_center_x = self.facedata[1]
        self.face_center_y = self.facedata[2]
        self.face_width = self.facedata[3]
        self.last_detect_time = rospy.get_rostime()
        #print "face_center_x: %d"%self.face_center_x
        #print "face_center_x: %d"%self.face_center_x

        '''rospy.logwarn('id %d,CX %d,CY %d,Width %d,Height %d,EulerY %d,EulerZ %d,LeftEyeOpen %d,RightEyeOpen %d,Smile %d ', \
                  self.facedata[0], self.facedata[1], self.facedata[2], self.facedata[3], self.facedata[4], self.facedata[5],\
                  self.facedata[6], self.facedata[7], self.facedata[8], self.facedata[9]) '''

    def commandKeyCallback(self, msg):
        #rospy.logwarn('1commandKeyCallback %d', msg.data) 
        if msg.data == "5" :
            self.mode = 5
        elif msg.data == "6" :
            self.mode = 6
        elif msg.data == "7" :
            self.mode = 7
        elif msg.data == "8" :
            self.mode = 8
        else :
            pass

    def initAction(self,userdata):

        try: 
	    joint_position = JointPosition()
	    planning_group = userdata.input_planning_group
	    joint_position.joint_name = ['joint1','joint2','joint3','joint4']
            joint_position.position =  [0.0, -1.05, 0.35, 0.70]
            path_time = 5
            resp1 = self.set_joint_position(planning_group,joint_position, path_time)
            #print 'resp1 {}'.format(resp1.is_planned) 
            rospy.sleep(5) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print 'init point\n' 

    def facetrackingActionByJoint(self,userdata):
  
        #center_x = 112 #320
        center_x = 182 #320
        center_y = 158 #240

        x_diff = (center_x - self.face_center_x)
        y_diff = (center_y - self.face_center_y)
        #move_x = (x_diff*1000/320)*0.1/1000
        #move_y = (y_diff*1000/320)*-0.05/1000
        move_x = (x_diff*1000/320)*0.3/1000
        move_y = (y_diff*1000/320)*-0.15/1000
        #rospy.logwarn('C_XY %.3f,%.3f , XY_Diff %.3f,%.3f  move_xy %.3f,%.3f)', self.face_center_x ,self.face_center_y , \
                       #x_diff, y_diff, move_x, move_y)
	object_detect_duration = rospy.get_rostime().to_sec() - self.last_detect_time.to_sec()
	#rospy.logwarn("duration %.2f",object_detect_duration ) 	
	
        #check last face detected time 
        if object_detect_duration > 1 :
	    rospy.logwarn(" no face.. %d ", self.no_detect_count)
            self.last_no_detect_time = rospy.get_rostime()
            self.no_detect_count = self.no_detect_count + 1
            if self.no_detect_count > 30 :
                rospy.logwarn(" no_detect_count too long..........................set init!!!!!!!!!!!!!!!!!")

                print 'set init position for face tracking \n' 
                try: 
	            joint_position = JointPosition()
	            planning_group = userdata.input_planning_group
	            joint_position.joint_name = ['joint1','joint2','joint3','joint4']
                    joint_position.position =  [0.0, -1.05, 0.35, 0.70]
                    path_time = 5
                    resp1 = self.set_joint_position(planning_group,joint_position, path_time)
                    #print 'resp1 {}'.format(resp1.is_planned) 
                    rospy.sleep(5) 
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                self.no_detect_count = 0

	    rospy.sleep(0.1) 	
            return ;
        
        self.no_detect_count = 0 

        center_range_x = 50
        center_range_y = 25
        #center range set 
        if abs(x_diff) > center_range_x or abs(y_diff) > center_range_y :
            #operating range set _ x range (-90 degree ~ 90 degree) y range (-45 degree ~ 45 degree)
            if (self.jointStates[0] > 1.57  and move_x > 0) or (self.jointStates[0] < -1.57 and move_x < 0 ) :
                move_x = 0 
                #rospy.logwarn("set  joint0 state %.3f , move_x  %.3f",self.jointStates[0], move_x) 
            if (self.jointStates[3] > 1.57  and move_y > 0) or (self.jointStates[3] < 0 and move_y < 0 ) :
                move_y = 0 
                #rospy.logwarn("set  joint3 state %.3f , move_y  %.3f",self.jointStates[3], move_y) 

            #tracking action
            try:
	        joint_position = JointPosition()
	        planning_group = userdata.input_planning_group
	        joint_position.joint_name = ['joint1','joint2','joint3','joint4']
                joint_position.position =  [move_x, 0, 0, move_y]
                resp1 = self.set_joint_position_from_present(planning_group,joint_position, 0.5)
                rospy.sleep(0.2)  
	        print 'resp2 {}'.format(resp1.is_planned) 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def panoramaShot(self,userdata):
        print 'panoramaShot init point\n' 

        try: 
	    joint_position = JointPosition()
	    planning_group = userdata.input_planning_group
	    joint_position.joint_name = ['joint1','joint2','joint3','joint4']
            joint_position.position =  [-1.57, -1.049, 0.354, 0.715]
            path_time = 5
            resp1 = self.set_joint_position(planning_group,joint_position, path_time)
            #print 'resp1 {}'.format(resp1.is_planned) 
            rospy.sleep(5) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try: 
	    joint_position = JointPosition()
	    planning_group = userdata.input_planning_group
	    joint_position.joint_name = ['joint1','joint2','joint3','joint4']
            joint_position.position =  [-3.14+0.03, -0.700, -0.249, 0.500]
            path_time = 5
            resp1 = self.set_joint_position(planning_group,joint_position, path_time)
            #print 'resp1 {}'.format(resp1.is_planned) 
            rospy.sleep(5) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.sleep(3)

        print 'panoramaShot start\n' 
        try:
	    joint_position = JointPosition()
	    planning_group = userdata.input_planning_group
	    joint_position.joint_name = ['joint1','joint2','joint3','joint4']
            joint_position.position =  [3.14*2-0.06, 0, 0, 0]
            path_time = 20
            resp1 = self.set_joint_position_from_present(planning_group,joint_position, path_time)
            rospy.sleep(21)  
	    #print 'resp1 {}'.format(resp1.is_planned) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print 'panoramaShot end\n' 

        rospy.sleep(3)

        print 'set init position for face tracking \n' 
        try: 
	    joint_position = JointPosition()
	    planning_group = userdata.input_planning_group
	    joint_position.joint_name = ['joint1','joint2','joint3','joint4']
            joint_position.position =  [0.0, -1.05, 0.35, 0.70]
            path_time = 5
            resp1 = self.set_joint_position(planning_group,joint_position, path_time)
            #print 'resp1 {}'.format(resp1.is_planned) 
            rospy.sleep(5) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def execute(self, userdata):
        self.initAction(userdata)
        rospy.sleep(3) 
        self.mode = 9
	while 1:
            if rospy.is_shutdown() :
                return 'aborted'
            elif self.mode == 5 :
                rospy.logwarn(" press 5  release cup ")
                break
            elif self.mode == 6 :
                rospy.logwarn(" press 6  stop ")
                continue
            elif self.mode == 7 :
                rospy.logwarn(" press 7  restart ")
                self.initAction(userdata)
                self.mode = 9
            elif self.mode == 8 :
                rospy.logwarn(" press 8 panorama shot ")
                self.panoramaShot(userdata)
                self.mode = 9

            self.facetrackingActionByJoint(userdata)
        return 'succeeded' 
