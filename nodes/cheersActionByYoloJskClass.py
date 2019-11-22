#!/usr/bin/env python

import rospy
import rospkg
from math import pow, atan2, sqrt
from tf.transformations import *

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
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import JointState

from geometry_msgs.msg import PoseArray, Pose
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
import tf
from enum import Enum

class cheersActionByYoloJsk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted','option1','option2','option3'],
                                    input_keys=['input_planning_group'])


        self.detected = {}
        self.detection_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
	self.object_pose_sub = rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, self.collectJsk)
        self.listener = tf.TransformListener()


	self.object_pose_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.collect)


        self.cupboundingBox = BoundingBox()
        self.set_joint_position = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path', SetJointPosition)
        self.set_kinematics_position = rospy.ServiceProxy('/open_manipulator/goal_task_space_path_position_only', SetKinematicsPose)
	self.set_joint_position_from_present = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path_from_present', SetJointPosition)
        self.open_manipulator_joint_states_sub_ = rospy.Subscriber('/open_manipulator/joint_states', JointState, self.jointStatesCallback)
        self.open_manipulator_kinematics_pose_sub_ = rospy.Subscriber('/open_manipulator/gripper/kinematics_pose', KinematicsPose, self.kinematicsPoseCallback)
        self.open_manipulator_states_ = rospy.Subscriber('/open_manipulator/states', OpenManipulatorState, self.StatesCallback)
        self.last_detect_time = rospy.get_rostime()
        self.jointStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	self.kinematicsStates = [0.0, 0.0, 0.0]
        rospy.logwarn(' cheersActionByYoloJsk ')
        self.trans = [0.0 , 0.0 , 0.0]
        self.command_Key_sub = rospy.Subscriber('command_Key', String, self.commandKeyCallback)
        self.mode = 9
        self.last_detect_time = rospy.get_rostime()
        self.last_yolodetect_time = rospy.get_rostime()
        self.open_manipulator_moving_state = "STOPPED"

        self.StepCupTracking = Enum('StepCupTracking',
                                       'waiting_signal \
                                        go_backward_location \
                                        wait_detecting_tf_of_cup \
                                        go_base_location \
                                        go_tf_location_of_cup \
                                        wait_finish_movement \
                                        close_cup \
                                        kick_cup \
                                        exit')

        self.pre_step = self.StepCupTracking.exit.value
        #self.step = self.StepCupTracking.waiting_signal.value
        self.step = self.StepCupTracking.go_backward_location.value
        self.use_platform = rospy.get_param("~use_platform") 

    def StatesCallback(self, msg):	
        self.open_manipulator_moving_state = msg.open_manipulator_moving_state

    def commandKeyCallback(self, msg):
        #rospy.logwarn('1commandKeyCallback %d', msg.data) 
        if msg.data == "2" :
            self.mode = 2
        elif msg.data == "6" :
            self.mode = 6
        elif msg.data == "7" :
            self.mode = 7
        else :
            pass

    def kinematicsPoseCallback(self, msg):
        self.kinematicsStates[0] = msg.pose.position.x
        self.kinematicsStates[1] = msg.pose.position.y
        self.kinematicsStates[2] = msg.pose.position.z
        #rospy.logwarn(' kinematicsPoseCallback %.2f , %.2f, %.2f  ', self.kinematicsStates[0], self.kinematicsStates[1], self.kinematicsStates[2] )

    def jointStatesCallback(self, msg):
	#rospy.logwarn('jointStatesCallback %d ', len(msg.position) )
        for i, pose in enumerate(msg.position):
            self.jointStates[i] = pose
            #print 'boundingBoxe {} {} '.format(i, pose)

    def collectJsk(self, msg):
        for i, pose in enumerate(msg.poses):
            if pose != Pose():                
                try:
                    #(trans1,rot1) = self.listener.lookupTransform('map', 'yolo_output'+str(i), rospy.Time(0))
		    #(trans,rot) = self.listener.lookupTransform('camera_link', 'yolo_output'+str(i), rospy.Time(0))
		    (trans,rot) = self.listener.lookupTransform('link1', 'yolo_output'+str(i), rospy.Time(0))
		    pos = pose.position
		    val = [round(pos.x,2), round(pos.y,2), round(pos.z,2), round(trans[0],2), round(trans[1],2) , round(trans[2],2)]
                    key = self.detection_names[i]  		
                    #self.update_key(key, val)
		    #print 'Found a {} at {} num{} TF {} '.format(key, val, i, trans)
                    self.trans[0] = round(trans[0],2)
                    self.trans[1] = round(trans[1],2)
                    self.trans[2] = round(trans[2],2)
                    self.last_detect_time = rospy.get_rostime()
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn('there is no tf ')


    def collect(self, msg):
        for i, boundingBox in enumerate(msg.bounding_boxes):
            #print 'boundingBoxe {} {} '.format(i, boundingBox)
	    if boundingBox.Class == "cup" or boundingBox.Class == "wine glass" :
                self.cupboundingBox = boundingBox
		self.last_yolodetect_time = rospy.get_rostime()
		#self.last_detect_time = rospy.get_rostime()
		#rospy.logwarn('cup withd : %d , z : %.2f', self.cupboundingBox.xmax - self.cupboundingBox.xmin, elf.cupboundingBox.xmin.Z )

    def stop_movement(self, userdata):
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        joint_position.position =  [0, 0, 0, 0]
        planning_group = userdata.input_planning_group
        try:    
            path_time = 1                    
            resp1 = self.set_joint_position_from_present(planning_group,joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def execute(self, userdata):
        print "Send Position by Joint Control "
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        joint_position.position =  [0, 0, 0, 0]
        planning_group = userdata.input_planning_group
        end_effector_name = "gripper";     
        kinematics_pose = KinematicsPose()
        cup_tf = [0, 0, 0]   
        self.mode = 9
        self.step = self.StepCupTracking.waiting_signal.value

	while 1:   
            rospy.logwarn("waiting_signal %d ", )         
            if rospy.is_shutdown() :
                return 'aborted'
            elif self.mode == 2 :
                return 'succeeded'
            elif self.mode == 6 :
                self.stop_movement(userdata)
                self.step = self.StepCupTracking.waiting_signal.value
                pass

            if self.step == self.StepCupTracking.waiting_signal.value:
                #rospy.logwarn("waiting_signal")
                if self.mode == 7 :
                    self.pre_step = self.StepCupTracking.waiting_signal.value
                    self.step = self.StepCupTracking.go_backward_location.value
	        else :
                    self.step = self.StepCupTracking.go_backward_location.value
                    pass

            if self.step == self.StepCupTracking.go_backward_location.value:
                rospy.logwarn("go_backward_location")
                #rospy.sleep(3)
                joint_position.position =  [0, -1.551, -0.234, 1.98]
                path_time = 2.5
                try:
                    resp1 = self.set_joint_position(planning_group,joint_position, path_time)
	            #print 'resp1 {}'.format(resp1.is_planned) 
	            rospy.sleep(path_time) 
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                self.pre_step = self.StepCupTracking.go_backward_location.value
                self.step = self.StepCupTracking.wait_detecting_tf_of_cup.value

            elif self.step == self.StepCupTracking.wait_detecting_tf_of_cup.value:
                #rospy.logwarn("wait_detecting_tf_of_cup")
                rospy.sleep(3)
	        object_detect_duration = rospy.get_rostime().to_sec() - self.last_detect_time.to_sec()
	        #rospy.logwarn("duration %.2f",object_detect_duration )
                tmp_x = self.trans[0]
                tmp_y = self.trans[1]
                tmp_z = self.trans[2] 
                distance = math.sqrt(tmp_x**2 + tmp_y**2 + tmp_z**2)
                width = self.cupboundingBox.xmax - self.cupboundingBox.xmin
                if tmp_z < 0.06 :
                    tmp_z = 0.05
 	        #rospy.sleep(0.5)
                if self.use_platform :
	            if object_detect_duration > 1 or tmp_x > 0.5 or distance >0.7 or tmp_x < 0.07 or self.cupboundingBox.Z < 0.01 :
                        rospy.logwarn("object_detect_duration1 %.2f,tmp_x %.2f,distance %.2f,tmp_z %.2f,tmp_x %.2f, self.cupboundingBox.Z %.2f", \
                                       object_detect_duration,tmp_x,distance,tmp_z,tmp_x, self.cupboundingBox.Z)
	                rospy.logwarn(" no object1.. ")
                        pass
                    else : 
                        radian = math.atan(tmp_y/tmp_x)
                        degree = math.degrees(radian)
                        #dist = 0.1	
                        dist = 0.07
                        distX = math.cos(radian)*dist
                        distY = math.sin(radian)*dist
                        rospy.logwarn("tmp_xyz1 %.2f,%.2f,%.2f _ radian %.2f(%.2f) distXY %.3f , %.3f",tmp_x,tmp_y,tmp_z,radian, degree, distX, distY )
                        cup_tf = [tmp_x - distX , tmp_y - distY, tmp_z + 0.05]
                        self.pre_step = self.StepCupTracking.wait_detecting_tf_of_cup.value
                        self.step = self.StepCupTracking.go_base_location.value
                else :
	            if object_detect_duration > 1 or tmp_x > 0.5 or distance >0.7 or self.cupboundingBox.Z < 0.01 :
                        rospy.logwarn("object_detect_duration2 %.2f,tmp_x %.2f,distance %.2f,tmp_z %.2f,tmp_x %.2f, self.cupboundingBox.Z %.2f", \
                                       object_detect_duration,tmp_x,distance,tmp_z,tmp_x, self.cupboundingBox.Z)
	                rospy.logwarn(" no object2.. ")
                        pass
                    else : 
                        radian = math.atan(tmp_y/tmp_x)
                        degree = math.degrees(radian)
                        dist = 0.038	
                        distX = math.cos(radian)*dist
                        distY = math.sin(radian)*dist 
                        rospy.logwarn("tmp_xyz2 %.2f,%.2f,%.2f _ radian %.2f(%.2f) distXY %.3f , %.3f",tmp_x,tmp_y,tmp_z,radian, degree, distX, distY )                       
                        cup_tf = [tmp_x - distX , tmp_y - distY, tmp_z ]
                        self.pre_step = self.StepCupTracking.wait_detecting_tf_of_cup.value
                        self.step = self.StepCupTracking.go_base_location.value

            elif self.step == self.StepCupTracking.go_base_location.value:
                rospy.logwarn("go_base_location")

                joint_position.position =  [0, -1.292, 0.242, 1.342]
                path_time = 1.5
                try:
                    resp1 = self.set_joint_position(planning_group,joint_position, path_time)
	            #print 'resp1 {}'.format(resp1.is_planned) 
	            rospy.sleep(path_time) 
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                self.pre_step = self.StepCupTracking.go_base_location.value
                self.step = self.StepCupTracking.go_tf_location_of_cup.value

            elif self.step == self.StepCupTracking.go_tf_location_of_cup.value:
                rospy.logwarn("go_tf_location_of_cup")

                kinematics_pose.pose.position.x =  cup_tf[0]
                kinematics_pose.pose.position.y =  cup_tf[1]
                kinematics_pose.pose.position.z =  cup_tf[2]
                distance = math.sqrt(cup_tf[0]**2 + cup_tf[1]**2 + cup_tf[2]**2)
                operating_time = distance * 7 / 0.6
                rospy.logwarn("go kinemetics xyz  %.2f,%.2f,%.2f _ time %.3f, dis %.3f",cup_tf[0],cup_tf[1],cup_tf[2],operating_time, distance)
                if operating_time < 3 :
                    operating_time = 3 
                try: 
                    resp1 = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
	            print 'kinemetics resp1 {} time '.format(resp1.is_planned, operating_time) 
                    rospy.sleep(0.1)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                self.pre_step = self.StepCupTracking.go_tf_location_of_cup.value
                self.step = self.StepCupTracking.wait_finish_movement.value

            elif self.step == self.StepCupTracking.wait_finish_movement.value:
                
                object_yolodetect_duration = rospy.get_rostime().to_sec() - self.last_yolodetect_time.to_sec()
                width = self.cupboundingBox.xmax - self.cupboundingBox.xmin
                centX = self.cupboundingBox.X
                centy = self.cupboundingBox.Y
                rospy.logwarn("wait_finish_movement %s , width, %d ", self.open_manipulator_moving_state, width)
                rospy.sleep(1)
                if object_yolodetect_duration < 0.5 and width>300 :
                    self.stop_movement(userdata)
                    self.pre_step = self.StepCupTracking.wait_finish_movement.value
                    self.step = self.StepCupTracking.close_cup.value
                    rospy.logwarn("if .............  1 w %.2f", width)                    
                elif self.open_manipulator_moving_state == OpenManipulatorState.STOPPED :
                    self.pre_step = self.StepCupTracking.wait_finish_movement.value
                    self.step = self.StepCupTracking.close_cup.value
                    rospy.logwarn("if ................................................2 w %.2f", width)
                else :
                    rospy.logwarn("if .............  3 w %.2f", width)

            elif self.step == self.StepCupTracking.close_cup.value: 

                self.stop_movement(userdata)                    
                self.pre_step = self.StepCupTracking.close_cup.value
                self.step = self.StepCupTracking.exit.value
                '''
	        object_yolodetect_duration = rospy.get_rostime().to_sec() - self.last_yolodetect_time.to_sec()            
                width = self.cupboundingBox.xmax - self.cupboundingBox.xmin
                centX = self.cupboundingBox.X
                centY = self.cupboundingBox.Y
                if self.use_platform :
                    cameraCenX =500 # 320
                    cameraCenY =450 # 240
                else :
                    cameraCenX =165 # 320
                    cameraCenY =390 # 240

                range_x = 50
                range_y = 50
                minX = cameraCenX - range_x
                maxX = cameraCenX + range_x
                minY = cameraCenY - range_y
                maxY = cameraCenY + range_y
                move_x = 0
                move_y = 0

	        if object_yolodetect_duration > 1 :
                    self.stop_movement(userdata)                    
                    self.pre_step = self.StepCupTracking.close_cup.value
                    self.step = self.StepCupTracking.exit.value
                    continue

                if minX < centX and centX < maxX and minY < centY and centY < maxY :
                    self.stop_movement(userdata)
                    self.pre_step = self.StepCupTracking.close_cup.value
                    self.step = self.StepCupTracking.kick_cup.value   
                    rospy.logwarn("1close_cup centXY %.2f  ( %.2f, %.2f ), %.2f ( %.2f, %.2f )_%.2f,%.2f_w %.2f",\
                                    centX, minX, maxX ,centY ,minY, maxY,move_x, move_y, width)

                else :
                    if centX < minX : 
                        #turn left
                        move_x = 0.03 
                    elif centX > maxX : 
                        #turn right
                        move_x = -0.03 
                    if centY < minY : 
                        #turn up
                        move_y = -0.03 
                    elif centY > maxY : 
                        #turn down
                        move_y = 0.03   
                    rospy.logwarn("2close_cup centXY %.2f  ( %.2f, %.2f ), %.2f ( %.2f, %.2f )_%.2f,%.2f_w %.2f",\
                                    centX, minX, maxX ,centY ,minY, maxY,move_x, move_y, width)             
                    try:
                        joint_position.position =  [move_x, 0, 0, move_y]
                        resp1 = self.set_joint_position_from_present(planning_group,joint_position, 1)
                        rospy.sleep(0.2)  
	                print 'resp2 {}'.format(resp1.is_planned) 
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    '''

            elif self.step == self.StepCupTracking.kick_cup.value:
                if self.use_platform :
                   #joint2_move = 0.15
                   #joint3_move = 0.1
                   joint2_move = 0.0
                   joint3_move = 0.05
                else :
                   joint2_move = 0.0
                   joint3_move = -0.05
                try:
                    #joint_position.position =  [0, 0.2, -0.3, 0]
                    joint_position.position =  [0, joint2_move, joint3_move, 0]
                    resp1 = self.set_joint_position_from_present(planning_group,joint_position, 0.5)
                    rospy.sleep(0.5)  
	            print 'resp2 {}'.format(resp1.is_planned) 
                    #rospy.sleep(0.5) 
                    #joint_position.position =  [0, -joint2_move , -joint3_move, 0]
                    #resp1 = self.set_joint_position_from_present(planning_group,joint_position, 2)
                    #rospy.sleep(2)  
	            print 'resp2 {}'.format(resp1.is_planned) 
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e    
                self.pre_step = self.StepCupTracking.kick_cup.value
                self.step = self.StepCupTracking.exit.value 

            elif self.step == self.StepCupTracking.exit.value:
                rospy.logwarn("exit")
                #rospy.sleep(3)
                self.pre_step = self.StepCupTracking.exit.value
                self.step = self.StepCupTracking.go_backward_location.value
