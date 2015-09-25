#!/usr/bin/python
import rospy
import rospkg
import smach
import smach_ros
import tf
import sys
import copy
import std_srvs

import matplotlib.pyplot as plt
import numpy

import random
import pdb

from std_srvs.srv import Trigger, TriggerRequest, Empty
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject, DisplayTrajectory
from shape_msgs.msg import SolidPrimitive
from cob_object_detection_msgs.msg import DetectionArray
#from cob_3d_mapping_msgs.msg import TriggerGoal
from ur_msgs.srv import SetIO, SetIORequest
import simple_moveit_interface as smi

from simple_script_server import *
from _ast import Add
from operator import attrgetter, sub
from moveit_msgs.msg._MotionPlanRequest import MotionPlanRequest
from moveit_msgs.msg._PositionIKRequest import PositionIKRequest
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory, PlanningSceneComponents
from moveit_msgs.msg import RobotTrajectory, PlanningScene
from trajectory_msgs.msg._JointTrajectoryPoint import JointTrajectoryPoint
sss = simple_script_server()


class SelectBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['new_box','available_grasps'],
            output_keys=['new_box','available_grasps'])
#         self.transofmaton_frame = "table" # this is the frame where all the grasps will be transformed to
        self.current_box_fix = ""
        self.current_box = ""
        self.rotation_offset = 0
        rospy.Subscriber("/find_suction_grasps/bounding_box_array", DetectionArray, self.callback)
#         rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        # initialize tf listener
        self.listener = tf.TransformListener()
        self.trigger_segmentation = rospy.ServiceProxy('/find_suction_grasps/trigger', std_srvs.srv.Trigger)
        # List of hypothesis
        self.list_of_grasp_hypothesis = []
        self.debug_SelectBox = False
        ### initialize service for gripper on universal arm 
        rospy.sleep(1)
        

    def execute(self, userdata):
        start_detect = rospy.Time.now()
        rospy.wait_for_service('/find_suction_grasps/trigger')
        if self.debug_SelectBox:
            rospy.logwarn(">>>>>>>>>>>>>>>>>>>>>>>>>>")
            rospy.logwarn("the new box flag is: %s",userdata.new_box)
            rospy.logwarn(">>>>>>>>>>>>>>>>>>>>>>>>>>")
        
        if userdata.new_box:# this is set to true during the construction of the State machine, before the first iteration then it is overwritten every iteration
            self.current_box_fix = ""
#             print "trigger perception manually"
#            sss.wait_for_input()
            print "trigger new detection"
            try:
                self.trigger_segmentation()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return "failed"
            print "I have requested the service"
            start_time = rospy.Time.now()
            while (len(self.list_of_grasp_hypothesis) == 0 or self.current_box_fix == "")and not rospy.is_shutdown():#waiting for call back to return the selected box
                if rospy.Time.now() > start_time + rospy.Duration(5):
                    print "timeout while waiting for result. Repeating select"
                    return "failed" #results in a loop here until a grasp is found 
                #print "waiting for result"
                #print self.current_box_fix
                rospy.sleep(0.01)
            userdata.new_box = False
            if self.debug_SelectBox:
                print self.current_box_fix
            self.current_box = copy.deepcopy(self.current_box_fix)
            userdata.available_grasps = self.list_of_grasp_hypothesis
            print "\n##### detection time", (rospy.Time.now() - start_detect).to_sec()

            ############UGLY
            #rospy.sleep(0.5)
            
        else:
            # here the detected grasp is rotated by rotation_offset this is to obtain a viable  grasp
            rospy.logwarn("YOU SHOULD NOT BE HERE")
            userdata.new_box = True
            if self.rotation_offset >= 2*math.pi:
                rospy.loginfo("resetting rotation")
                self.rotation_offset = 0.0
                userdata.new_box = True
                return "failed"
            self.rotation_offset += 60/180.0*math.pi
            #print "self.current_box_fix.pose="
            #print self.current_box_fix.pose
            print "current box frame id before update: ", self.current_box.pose.header.frame_id


            pose_offset = PoseStamped()
            pose_offset.header.frame_id = "current_box_fix"
            pose_offset.header.stamp = rospy.Time(0)
            quat = tf.transformations.quaternion_from_euler(0, 0, self.rotation_offset)
            pose_offset.pose.orientation.x = quat[0]
            pose_offset.pose.orientation.y = quat[1]
            pose_offset.pose.orientation.z = quat[2]
            pose_offset.pose.orientation.w = quat[3]
            ps = self.listener.transformPose("current_box_fix", pose_offset)
            self.current_box = copy.deepcopy(self.current_box_fix)
            self.current_box.pose = ps

            #print "self.current_box.pose="
            #print self.current_box.pose

        #rospy.sleep(0.05)#HACK
        return "succeeded"

    def callback(self, bba):
        print "Found", len(bba.detections), "bounding boxes"
        if self.debug_SelectBox:
            print "the frame obtained from the topic is", bba.header.frame_id
        #####FIX wrong bounding box lwh calculations (needs to be fixed in box_detection and perception_common/cob_vision_utils (boundingBoxToMarker)###
        bba_out = DetectionArray()
        for bb in bba.detections:
            bb.bounding_box_lwh.x = bb.bounding_box_lwh.x*2
            bb.bounding_box_lwh.y = bb.bounding_box_lwh.y*2
            bba_out.detections.append(bb)
        bba = bba_out
        #####END FIX
        
        self.grasp_hypothesis_filter(bba)

    
    def grasp_hypothesis_filter(self, bba):
        ########## defines ########
        # common
        distance_from_center = 0.2          # [m]
        upright_offset = 70/180.0*math.pi # [rad]
        edge_offset = 0.03                  # [m]
        relation_factor = 1.5
        height_threshold = 0.04             # [m]
        # cromo
        edge_length_short = 0.075           # [m]
        edge_length_long = 0.17             # [m]
        
        # ambroxol
        #edge_length_short = 0.05
        #edge_length_long = 0.135
        
        ########## end defines ######


        # upright
        #bba = self.filter_for_upright(bba, upright_offset)
        #print "--> after upright filter:", len(bba.detections), "bounding boxes"
        
        # too close to table
        bba = self.filter_low_grasps(bba, height_threshold)
        
        bba = self.filter_for_upright(bba, upright_offset)

        # center from_container
        bba = self.filter_far_away_from_center_of_container(bba, distance_from_center)
        if self.debug_SelectBox:
            print "--> after filtration:", len(bba.detections), "bounding boxes"

        # area
        #min_area = (edge_length_short - edge_offset) * (edge_length_long - edge_offset)
        #max_area = (edge_length_short + edge_offset) * (edge_length_long + edge_offset)
        #print "min_area, max_area", min_area, ",", max_area
        #bba = self.filter_for_area_size(bba, min_area, max_area)
        #print "--> after area filter:", len(bba.detections), "bounding boxes"
        
        # edge length
        #min_length = edge_length_short - edge_offset
        #max_length = edge_length_long + edge_offset
        #print "min_length, max_length", min_length, ",", max_length
        #bba = self.filter_for_edge_length(bba, min_length, max_length)
        #print "--> after edge filter:", len(bba.detections), "bounding boxes"

        # edge relation
        #min_edge_relation = edge_length_short/edge_length_long / relation_factor
        #max_edge_relation = edge_length_short/edge_length_long * relation_factor
        #print "min_edge_relation, max_edge_relation", min_edge_relation, max_edge_relation
        #bba = self.filter_for_edge_relation(bba, min_edge_relation, max_edge_relation)
        #print "--> after edge relation filter:", len(bba.detections), "bounding boxes"

        if len(bba.detections) == 0:
            return

#         self.broadcast_bba(bba)# rebroadcast on tf all the grasps in the distance defined by distance_from_center
        # select highest plane
        
        height = ""
        for bb in bba.detections:
            bb.pose.header.stamp = rospy.Time(0)
            bb_pose_in_table = self.listener.transformPose("table",bb.pose)# output is PoseStamed
            bb.pose = bb_pose_in_table
#             detObj = Detection()
#             detObj.pose = bb_pose_in_table
#             self.list_of_grasp_hypothesis.append(bb_pose_in_table)        
#             self.list_of_grasp_hypothesis.append(detObj)
        self.list_of_grasp_hypothesis = bba.detections # all detections are now in the table frame
        self.list_of_grasp_hypothesis.sort(key = lambda ps: ps.pose.pose.position.z, reverse=True) # Sort Poses according to height from table
        self.broadcast_bba(bba)
        if self.debug_SelectBox:
            rospy.logwarn("Heights of grasps after sorting: ")
            height_after_sort = []
            for ps in self.list_of_grasp_hypothesis:
                height_after_sort.append(ps.pose.pose.position.z)
            rospy.logwarn(height_after_sort)
        height = self.list_of_grasp_hypothesis[0].pose.pose.position.z # the heighest grasp
        if(height< height_threshold):
            print "The frame has been rejected due to not being high enough, the threshold is: ",height_threshold
            return
        else:
            self.current_box_fix = self.list_of_grasp_hypothesis[0] # this is the detection array entry of the selected box this is defined in the table frame
            if self.debug_SelectBox:
                print "height =", height
                print "bb_area =", self.list_of_grasp_hypothesis[0].bounding_box_lwh.x * self.list_of_grasp_hypothesis[0].bounding_box_lwh.y
                print "bb_x, bb_y = ", self.list_of_grasp_hypothesis[0].bounding_box_lwh.x, ",", self.list_of_grasp_hypothesis[0].bounding_box_lwh.y
    
    def filter_for_area_size(self, bba_in, lower_limit, upper_limit):
        if upper_limit < lower_limit:
            rospy.log_err("upper_limit is smaller than lower_limit")
            sys.exit()
        
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            area = bb.bounding_box_lwh.x * bb.bounding_box_lwh.y
            #print "area =", area
            if area < upper_limit and area > lower_limit:
                bba_out.detections.append(bb)
        return bba_out

    def filter_for_upright(self, bba_in, offset):
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            bb_pose_in_table = self.listener.transformPose("table",bb.pose)
            (roll, pitch, yaw) = euler_from_quaternion([bb_pose_in_table.pose.orientation.x, bb_pose_in_table.pose.orientation.y, bb_pose_in_table.pose.orientation.z, bb_pose_in_table.pose.orientation.w])
            roll = abs(roll)
            pitch = abs(pitch)
            yaw = abs(yaw)
            #print roll, pitch, yaw
            if roll < offset and pitch < offset:
                bba_out.detections.append(bb)
        return bba_out
        
    def filter_for_edge_length(self, bba_in, lower_limit, upper_limit):
        if upper_limit < lower_limit:
            rospy.log_err("upper_limit is smaller than lower_limit")
            sys.exit()
        
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            #print "bba_x =", bb.bounding_box_lwh.x, "bba_y =", bb.bounding_box_lwh.y
            if bb.bounding_box_lwh.x < upper_limit and bb.bounding_box_lwh.x > lower_limit and bb.bounding_box_lwh.y < upper_limit and bb.bounding_box_lwh.y > lower_limit:
                bba_out.detections.append(bb)
        return bba_out

    def filter_for_edge_relation(self, bba_in, lower_limit, upper_limit):
        if upper_limit < lower_limit:
            rospy.log_err("upper_limit is smaller than lower_limit")
            sys.exit()
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            xy_relation = bb.bounding_box_lwh.x / bb.bounding_box_lwh.y
            if xy_relation > 1:
                xy_relation = 1/xy_relation
            #print "edge_relation =", xy_relation
            if xy_relation > lower_limit and xy_relation < upper_limit:
                bba_out.detections.append(bb)
        return bba_out
    
    def filter_low_grasps(self, bba_in, min_height):
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            try:
                height_above_table = self.listener.transformPose("table", bb.pose).pose.position.z
            except Exception, e:
                rospy.logerr("could not transform pose. Exception: %s", str(e))
                sys.exit()
            if height_above_table >= min_height:
                bba_out.detections.append(bb)
        return bba_out

    def filter_far_away_from_center_of_container(self, bba_in, max_distance):
        bba_out = DetectionArray()
        for bb in bba_in.detections:
            try:
                trafo = self.listener.transformPose("container_link", bb.pose)
            except Exception, e:
                rospy.logerr("could not transform pose. Exception: %s", str(e))
                sys.exit()
            distance = math.sqrt(trafo.pose.position.x**2 + trafo.pose.position.y**2)
            #print "distance", distance
            if distance < max_distance:
                bba_out.detections.append(bb)
        return bba_out

    def broadcast_bba(self, bba):
        counter = 0
        for bb in bba.detections:
            name = "bba_" + str(counter)
            #print "broadcast " + name
            counter += 1
            self.br.sendTransform(
                    (bb.pose.pose.position.x, bb.pose.pose.position.y, bb.pose.pose.position.z),
                    #(0, 0, 0, 1),
                    (bb.pose.pose.orientation.x, bb.pose.pose.orientation.y, bb.pose.pose.orientation.z, bb.pose.pose.orientation.w),
                    rospy.Time.now(),
                    name,
                    bb.pose.header.frame_id)
        

    def broadcast_tf(self, event):
        if self.current_box != "":
            self.br.sendTransform(
                    (self.current_box.pose.pose.position.x, self.current_box.pose.pose.position.y, self.current_box.pose.pose.position.z),
                    #(0, 0, 0, 1),
                    (self.current_box.pose.pose.orientation.x, self.current_box.pose.pose.orientation.y, self.current_box.pose.pose.orientation.z, self.current_box.pose.pose.orientation.w),
                    event.current_real,
                    "current_box",
                    self.current_box.pose.header.frame_id)

        if self.current_box_fix != "":
            self.br.sendTransform(
                    (self.current_box_fix.pose.pose.position.x, self.current_box_fix.pose.pose.position.y, self.current_box_fix.pose.pose.position.z),
                    #(0, 0, 0, 1),
                    (self.current_box_fix.pose.pose.orientation.x, self.current_box_fix.pose.pose.orientation.y, self.current_box_fix.pose.pose.orientation.z, self.current_box_fix.pose.pose.orientation.w),
                    event.current_real,
                    "current_box_fix",
                    self.current_box_fix.pose.header.frame_id)


class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['available_grasps'],
            output_keys=['new_box'])

        self.debug = True
        self.execute_motions = True
        self.simulation = False
        self.gripper_type = 2#1 metalbracket and large suction, #2 new 3d printed gripper
        self.eef_step = 0.1
        self.jump_threshold = 3.0 # this defines the maximum allowable jump between interpolated points in the cartesian path
        self.plan_mode = 1 # 1 = cartesian path with intermediate point, 2 =  collision free planning using plan from moveit
        
        # initialize tf listener
        self.listener = tf.TransformListener()
        
        # initialize tf broadcaster
        self.br = tf.TransformBroadcaster()
        
        ### Create a handle for the Robot Commander
        self.rc = RobotCommander()
        
        ### Create a handle for the Move Group Commander
        self.mgc = MoveGroupCommander("manipulator")
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        
        ### create handle for octomap clear service
        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.clear_octomap_service = rospy.ServiceProxy('/clear_octomap', std_srvs.srv.Empty)
        
        ### create handle for obtaining the planning scene
        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.get_planning_scene_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene) # publisher for planning scene
        
        ### create octo map update handel that uses the agile grasp PC filtration node
        rospy.wait_for_service('/PC_filter/trigger_PC_filter', 10.0)
        self.trigger_octo_map_update = rospy.ServiceProxy('/PC_filter/trigger_PC_filter', std_srvs.srv.Trigger)

        ### initialize service for gripper on universal arm 
        if(not self.simulation):
            self.io_srv = rospy.ServiceProxy('set_io', SetIO)
        

        rospy.logwarn("Initializing Grasp")
        
        #Collision objects
        self._pub_co = rospy.Publisher('/collision_object', CollisionObject)
        self._pub_aco = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.collision_objs = []
        self.attached_collsion_objs = []
        
        
    def execute(self, userdata):
        if not self.simulation:
            self.open_gripper()
        # Define plane to prevent motion out of the table
        plane_mount_frame_id = "table"
        plane_mount_dim = [0.01,1.8,1.5] # [m]
        
        plane_mount_pose = PoseStamped()
        plane_mount_name = "plane_mount_long"
        plane_mount_pose.header.frame_id = plane_mount_frame_id
        plane_mount_pose.pose.position.x = 0.78/2
        plane_mount_pose.pose.position.y = -1.56/2
        plane_mount_pose.pose.position.z = 0
        
#         self.manage_collison_obj(plane_mount_name, plane_mount_pose, plane_mount_dim, 1, True, True)
        
        # Define the Kinect mount
        kinect_mount_frame_id = "table"
        
        kinect_mount_long_side_dim = [0.045,0.045,1.33] # [m]
        kinect_mount_short_dim = [0.08,0.33,0.045] # [m]
        
        kinect_mount_long_pose = PoseStamped()
        kinect_mount_short_pose = PoseStamped()
        
        kinect_mount_long_name = "kinect_mount_long"
        kinect_mount_long_pose.header.frame_id = kinect_mount_frame_id
        kinect_mount_long_pose.pose.position.x = 0.373
        kinect_mount_long_pose.pose.position.y = 0.945
        kinect_mount_long_pose.pose.position.z = 0
        
        kinect_mount_short_name = "kinect_mount_short"
        kinect_mount_short_pose.header.frame_id = kinect_mount_frame_id
        kinect_mount_short_pose.pose.position.x = kinect_mount_long_pose.pose.position.x - (kinect_mount_short_dim[0] - kinect_mount_long_side_dim[0])
        kinect_mount_short_pose.pose.position.y = kinect_mount_long_pose.pose.position.y -  kinect_mount_short_dim[1]
        kinect_mount_short_pose.pose.position.z = 0.955
        
        #adding kineckt mounts to collision space
        self.manage_collison_obj(kinect_mount_long_name, kinect_mount_long_pose, kinect_mount_long_side_dim, 1, True, True)
        self.manage_collison_obj(kinect_mount_short_name, kinect_mount_short_pose, kinect_mount_short_dim, 1, True, True)
        
        
        # Define the 4 planes of the box
        box_frame_id = "table" #the coordinate system in which the box sides are defined
        box_long_side_dim = [0.005*2,0.41,0.165*1.2] # [m]
        box_short_side_dim = [0.29,0.005*2,0.165*1.2] # [m]
        
        box_left_pose = PoseStamped()
        box_right_pose = PoseStamped()
        box_back_pose = PoseStamped()
        box_front_pose = PoseStamped()
        
        
        box_left_name = "box_side_left"
        box_left_pose.header.frame_id = box_frame_id
        box_left_pose.pose.position.x = 0.0305
        box_left_pose.pose.position.y = 0.481
        box_left_pose.pose.position.z = 0
        
        box_right_name = "box_side_right"
        box_right_pose.header.frame_id = box_frame_id
        box_right_pose.pose.position.x = 0.311 
        box_right_pose.pose.position.y = 0.476        
        box_right_pose.pose.position.z = 0
        
        box_back_name = "box_side_back"
        box_back_pose.header.frame_id = box_frame_id
        box_back_pose.pose.position.x = 0.0305
        box_back_pose.pose.position.y = 0.481        
        box_back_pose.pose.position.z = 0
        
        box_front_name = "box_side_front"
        box_front_pose.header.frame_id = box_frame_id
        box_front_pose.pose.position.x = 0.0336
        box_front_pose.pose.position.y = 0.884        
        box_front_pose.pose.position.z = 0
        
        # adding the boxes to the collision space
#         self.manage_collison_obj(box_left_name, box_left_pose, box_long_side_dim, 1, True, True)
#         self.manage_collison_obj(box_right_name, box_right_pose, box_long_side_dim, 1, True, True)
#         self.manage_collison_obj(box_back_name, box_back_pose, box_short_side_dim, 1, True, True)
#         self.manage_collison_obj(box_front_name, box_front_pose, box_short_side_dim, 1, True, True)
#         self.clear_octomap_service()

        if self.debug:    
            rospy.logwarn("The static scene  have been added as collision objects")



#         # Define the box and cylinder defining the end effector
#         # end effector collision object
#         link_attached_to_ef = "ee_link"
#         mb_ef_collisonobj_touch_links = [link_attached_to_ef, 'wrist_3_link']
#           
#         mb_ef_collisonobj_name = "metal_bracket"
#         mb_ef_pose = PoseStamped()
#         mb_ef_pose.header.frame_id =  link_attached_to_ef
#         mb_ef_pose.pose.position.x = 0.0
#         mb_ef_pose.pose.position.y = -0.06/2
#         mb_ef_pose.pose.position.z = -0.06/2
#         mb_ef_size = (0.065,0.06,0.06)
#         
#                 
#         sucker_ef_collisonobj_name = "suction_gripper"
#         sucker_ef_pose = PoseStamped()
#         sucker_ef_pose.header.frame_id =  link_attached_to_ef
#         sucker_ef_pose.pose.position.x = mb_ef_size[0] + 0.04/2
#         sucker_ef_pose.pose.position.y = 0.0
#         sucker_ef_pose.pose.position.z = 0.0
#         sucker_ef_size = (0.04,0.03,0.03)
#  
#         box_dummy_pose = PoseStamped()
#         box_dummy_pose.header.frame_id =  "table"
#         box_dummy_pose.pose.position.x = 0.147
#         box_dummy_pose.pose.position.y = 0.636
#         box_dummy_pose.pose.position.z = 0 
#         self.manage_collison_obj("dummy_box", box_dummy_pose, [0.09,0.09,0.165], 1, True, True)
#         raw_input("Added box")
#         print self.collision_objs
#         self.manage_collison_obj("dummy_box", box_dummy_pose, [0.09,0.09,0.165], 1, False, True)
#         raw_input("removed box")        

#         self.manage_attached_collision_obj(link_attached_to_ef, mb_ef_collisonobj, mb_ef_pose, list(mb_ef_size), 1, True, [link_attached_to_ef, 'wrist_3_link'])
#         raw_input("added attached collision obj")
#         self.manage_attached_collision_obj(link_attached_to_ef, mb_ef_collisonobj, mb_ef_pose, list(mb_ef_size), 1, False, [link_attached_to_ef, 'wrist_3_link'])
#         raw_input("removed attached collision obj")
# #         self.psi.add_box(mb_ef_collisonobj, mb_ef_pose, mb_ef_size)# #size x,y,z x is always to the left viewing the robot from the PC 
#         self.mgc.attach_object(mb_ef_collisonobj, link_attached_to_ef)
#         aco = AttachedCollisionObject()
#         co = CollisionObject()
#         co.operation = CollisionObject.ADD
#         co.id = mb_ef_collisonobj
#         co.header = mb_ef_pose.header
#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         box.dimensions = list(mb_ef_size)
#         co.primitives = [box]
#         co.primitive_poses = [mb_ef_pose.pose]
#         aco.object = co
#         aco.touch_links = [link_attached_to_ef, 'wrist_3_link']
#         aco.link_name = link_attached_to_ef
        
        
#         sucker_ef_collisonobj = "suction_gripper"
#         sucker_ef_pose = PoseStamped()
#         sucker_ef_pose.header.frame_id =  link_attached_to_ef
#         sucker_ef_pose.pose.position.x = 0.065+0.04/2
#         sucker_ef_pose.pose.position.y = 0.0
#         sucker_ef_pose.pose.position.z = 0.0
#         sucker_ef_size = (0.04,0.03,0.03)
#         self.psi.attach_box(link_attached_to_ef, sucker_ef_collisonobj, sucker_ef_pose, sucker_ef_size)
        
        #if sss.wait_for_input() != "":
        #    print "not executing grasp"
        #    return "failed"

        #max_retries = 5
        #counter = 0
        #while not self.plan_and_execute(userdata):
        #    if counter >= max_retries:
        #        return "failed"
        #    counter+=1
        rospy.sleep(0.5)# time for processing
        if not self.plan_and_execute(userdata):
#             userdata.new_box = False
            return "failed"
        
        return "succeeded"
    
    def manage_attached_collision_obj(self, link, name, pose, size, type, add_remove, remove_from_environment = True, touch_links = []):
        """
        creates a collision object of either box, cylinder or sphere
        depending on the type. 1 = box, 2 = cylinder, 3 = sphere. 
        Add = true, remove = false
        @param link: the link to which the object will be attached
        @type link: String
        @param touch_links: the list of links that the obj is allowed to touch
        @param name: the name given to the collision obj
        @param pose: the pose of the obj
        @type pose: PoseStamped
        @param size: the size of the obj
        @type size: List
        @param type: the shape type of the obj
        @param add_remove: the flag to remove or add the obj
        @param remove_from_environment: A flag determining if the detached objcet should also be removed from the environmnt
        """
        aco = AttachedCollisionObject()
        aco.object = self.manage_collison_obj(name, pose, size, type, add_remove, False)
        aco.link_name = link
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        else:
            aco.touch_links = [link]
                     
        obj = None
        obj_index = -1
            
        for index, object_current in enumerate(self.attached_collsion_objs):
            if object_current.object.id == name:
                obj =  object_current
                obj_index = index

        if add_remove:
            if obj == None:#append if non existent
                self.attached_collsion_objs.append(aco)
            else:# overwrite if exists
                self.attached_collsion_objs[obj_index] = aco
        else:
            #remove from list if exists
            if obj != None:
                 self.attached_collsion_objs.remove(obj)
            #remove from environment
                 
        self._pub_aco.publish(aco)
        if add_remove == False and remove_from_environment:
            rospy.sleep(0.1)
            self._pub_co.publish(aco.object)
        return aco
    
    def manage_collison_obj(self, name, pose_stmpd, size, type, add_remove, publish = True):
        """
        creates a collision object of either box, cylinder or sphere
        depending on the type. 1 = box, 2 = cylinder, 3 = sphere. 
        Add = true, remove = false
        @param name: the name given to the collision obj
        @param pose_stmpd: the pose_stmpd of the obj
        @type pose_stmpd: PoseStamped
        @param size: the size of the obj
        @type size: List
        @param type: the shape type of the obj
        @param add_remove: the flag to remove or add the obj
        @todo: add cylinder to the planning interface
        """
        co = CollisionObject()
        if name == "":
            rospy.logwarn("no name given to collision obj")
            return co
        co.id = name
        
        obj = None
        obj_index = -1
            
        for index, object_current in enumerate(self.collision_objs):
            if object_current.id == name:
                obj =  object_current
                obj_index = index

        if add_remove:
            co.header = pose_stmpd.header
            co.primitive_poses = [pose_stmpd.pose]
            co.operation = CollisionObject.ADD
            if type!= 1 and type!= 2 and type!= 3:
                rospy.logerr("No proper type given to create collision obj")
            elif type == 1:# box
                co.primitives = [self.creat_primitive_box(size)]
                co.primitive_poses[0].position.x += size[0]/2
                co.primitive_poses[0].position.y += size[1]/2
                co.primitive_poses[0].position.z += size[2]/2
            elif type == 2:# cylinder
                co.primitives = [self.creat_primitive_cylinder(size)]
                co.primitive_poses[0].position.z += size[0]/2
            elif type == 3:# sphere
                co.primitives = [self.creat_primitive_sphere(size)]
            if obj == None:#append if non existent
                self.collision_objs.append(co)
            else:# overwrite if exists
                self.collision_objs[obj_index] = co
        else:
            co.operation = CollisionObject.REMOVE
            #remove from list if exists
            if obj != None:
                 self.collision_objs.remove(obj)
        if publish:
            self._pub_co.publish(co)
        return co
    
    
    def creat_primitive_box (self, size):
        """
        create a primitive box
        @param size: size of the box in x,y,z
        @type size: list
        @note: the box size starts from the center
        """
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        return box
        
    def creat_primitive_cylinder(self, size): 
        """
        create a primitive cylinder
        @type size[0]: double height 
        @type size[1]: double radius
        """   
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = size
#         cylinder.CYLINDER_HEIGHT = size[1]
#         cylinder.CYLINDER_RADIUS = size[0]
        return cylinder
    
    def creat_primitive_sphere(self, Radius): 
        """
        create a primitive sphere
        @type Radius: list double
        """   
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.size = Radius
        return sphere
    
    def open_gripper(self):
        r = SetIORequest()
        r.fun = 1
        r.pin = 0
        r.state = 0.0
        self.io_srv(r)

    def close_gripper(self):
        r = SetIORequest()
        r.fun = 1
        r.pin = 0
        r.state = 1.0
        self.io_srv(r)

    def plan_and_execute(self, userdata):
    
        ### DEBUG PERCEPTION ONLY
        #sss.wait_for_input()
        #userdata.new_box = True
        #return True
        ### END DEBUG
#         print "The pose reference frame is",self.mgc.get_pose_reference_frame()
#         self.mgc.set_pose_reference_frame("camera_rgb_optical_frame")
#         print "The pose reference frame is",self.mgc.get_pose_reference_frame()
        
        start_plan = rospy.Time.now()
        grasp_number = 0
        successful_trajectory_comp = False
        if self.debug:
            rospy.logwarn("*******************************")
            rospy.logwarn("the number of grasps detected after filtration are %i", len(userdata.available_grasps))
            rospy.logwarn("*******************************")
        while grasp_number<len(userdata.available_grasps):
            self.mgc.set_planner_id("KPIECEkConfigDefault") #self.mgc.set_planner_id("LBKPIECEkConfigDefault")
            # Define the box and cylinder defining the end effector
            # end effector collision object

            if self.debug:
                rospy.logwarn("*******************************")
                rospy.logwarn("currently processing grasp %i of %i", grasp_number+1, len(userdata.available_grasps))
                rospy.logwarn("*******************************")
            
            
            ### edits here were done due to the change of the gripper type 
            if self.gripper_type == 1:
                link_attached_to_ef = "ee_link"
                mb_ef_collisonobj_touch_links = [link_attached_to_ef, 'wrist_3_link']
                  
                mb_ef_collisonobj_name = "metal_bracket"
                mb_ef_pose = PoseStamped()
                mb_ef_pose.header.frame_id =  link_attached_to_ef
                mb_ef_pose.pose.position.x = 0.0
                mb_ef_pose.pose.position.y = -0.06/2
                mb_ef_pose.pose.position.z = -0.06/2
                mb_ef_size = [0.065,0.06,0.06]
                
                        
                sucker_ef_collisonobj_name = "suction_gripper"
                sucker_ef_pose = PoseStamped()
                sucker_ef_pose.header.frame_id =  link_attached_to_ef
                sucker_ef_pose.pose.position.x = mb_ef_size[0]
                sucker_ef_pose.pose.position.y = -0.03/2
                sucker_ef_pose.pose.position.z = -0.03/2
                sucker_ef_size = [0.05,0.03, 0.03]
         
                self.manage_attached_collision_obj(link_attached_to_ef, mb_ef_collisonobj_name, mb_ef_pose, mb_ef_size, 1, True, True, mb_ef_collisonobj_touch_links)
                self.manage_attached_collision_obj(link_attached_to_ef, sucker_ef_collisonobj_name, sucker_ef_pose, sucker_ef_size, 1, True, True, mb_ef_collisonobj_touch_links)
            
            elif self.gripper_type == 2:
                link_attached_to_ef = "ee_link"
                ef_collisonobj_touch_links = [link_attached_to_ef, 'wrist_3_link']
                  
                top_half_ef_collisonobj_name = "top_half_ef"
                top_half_ef_pose = PoseStamped()
                top_half_ef_pose.header.frame_id =  link_attached_to_ef
                top_half_ef_pose.pose.position.x = 0.0
                top_half_ef_pose.pose.position.y = -0.0125
                top_half_ef_pose.pose.position.z = -0.0125
#                 quat = tf.transformations.quaternion_from_euler(0, 90/180*math.pi, 0)
#                 top_half_ef_pose.pose.orientation.x = quat[0]
#                 top_half_ef_pose.pose.orientation.y = quat[1]
#                 top_half_ef_pose.pose.orientation.z = quat[2]
#                 top_half_ef_pose.pose.orientation.w = quat[3]
                top_half_ef_size = [0.11, 0.0125*2, 0.0125*2]
                
                        
                lower_half_ef_collisonobj_name = "lower_half_ef"
                lower_half_ef_pose = PoseStamped()
                lower_half_ef_pose.header.frame_id =  link_attached_to_ef
                lower_half_ef_pose.pose.position.x = top_half_ef_size[0]
                lower_half_ef_pose.pose.position.y = -0.0125/2
                lower_half_ef_pose.pose.position.z = -0.0125/2
#                 quat = tf.transformations.quaternion_from_euler(2, 90/180*math.pi, 2)
#                 lower_half_ef_pose.pose.orientation.x = quat[0]
#                 lower_half_ef_pose.pose.orientation.y = quat[1]
#                 lower_half_ef_pose.pose.orientation.z = quat[2]
#                 lower_half_ef_pose.pose.orientation.w = quat[3]
                lower_half_ef_size = [0.11, 0.0125, 0.0125]
         
                self.manage_attached_collision_obj(link_attached_to_ef, top_half_ef_collisonobj_name, top_half_ef_pose, top_half_ef_size, 1, True, True, ef_collisonobj_touch_links)
                self.manage_attached_collision_obj(link_attached_to_ef, lower_half_ef_collisonobj_name, lower_half_ef_pose, lower_half_ef_size, 1, True, True, ef_collisonobj_touch_links)
            
            self.clear_octomap_service()
            if(not self.trigger_octo_map_update().success):
                rospy.logwarn("The Octomap update has failed")
                raw_input("there is a problem updating the octo map program has paused")
            if self.debug:
                raw_input("updating octo map after reset")
            
            if self.debug:
                rospy.loginfo("*** MGC parameters***")
                rospy.loginfo("MGC planning frame is : "+ str(self.mgc.get_planning_frame()))
                rospy.loginfo("MGC goal tolerance is : "+str(self.mgc.get_goal_tolerance()))
                rospy.loginfo("MGC planning pose reference frame is : "+str(self.mgc.get_pose_reference_frame()))
                rospy.loginfo("*** End of MGC parameters***")
#                 raw_input("please press a key to continue")
                
            # publish current frame
#             bogus = userdata.available_grasps[grasp_number].pose
#             bogus = self.listener.transformPose("table", bogus)
#             bogus.pose.position.z += 0.05
#             self.broadcast_tf_frame(bogus,"current_box")
            self.broadcast_tf_frame(userdata.available_grasps[grasp_number].pose,"current_box")
            
            # this is used instead of the sss move to save time if the robots configuration has changed
            start_pose = PoseStamped()
            start_pose.pose.position.x = -0.152242
            start_pose.pose.position.y = 0.496314
            start_pose.pose.position.z = 0.366784
            
            start_pose.pose.orientation.x = 0.00119264
            start_pose.pose.orientation.y = 0.000560835
            start_pose.pose.orientation.z = 0.999998
            start_pose.pose.orientation.w = 0.00170356
            start_pose.header.frame_id = "table"

#             over_ride_frame = PoseStamped()
#             over_ride_frame.pose.position.x = 0.24413990093
#             over_ride_frame.pose.position.y = 0.665933668938
#             over_ride_frame.pose.position.z = 0.0455009931801
#             
#             over_ride_frame.pose.orientation.x = 0.0146537424109
#             over_ride_frame.pose.orientation.y = 0.0251395433852
#             over_ride_frame.pose.orientation.z = -0.707517848633
#             over_ride_frame.pose.orientation.w = 0.706096144344
#             over_ride_frame.header.frame_id = "table"
#             self.broadcast_tf_frame(over_ride_frame, "current_box")
            
    
            ### Plan Approach
            #parameters
            approach_height_above_grasp = 0.10
            approach_offset_in_table = 0#-0.01
            # Old
            approach_pose_offset = PoseStamped()
            approach_pose_offset.header.frame_id = "current_box"
            approach_pose_offset.header.stamp = rospy.Time(0)
            approach_pose_offset.pose.position.z += approach_height_above_grasp
            # alternative but in table coordinates
#             approach_pose_offset = copy.deepcopy(userdata.available_grasps[grasp_number].pose)
#             approach_pose_offset.header.stamp = rospy.Time(0)
#             rospy.logwarn(approach_pose_offset)
#             raw_input("above is the approach pose")
#             approach_pose_offset.pose.position.z += 0.05
            #approach_pose_offset.pose.position.x -= 0.1
            #quat = tf.transformations.quaternion_from_euler(0, 1.57, 1.57)
            #approach_pose_offset.pose.orientation.x = quat[0]
            #approach_pose_offset.pose.orientation.y = quat[1]
            #approach_pose_offset.pose.orientation.z = quat[2]
            #approach_pose_offset.pose.orientation.w = quat[3]
            
            try:
                approach_pose = self.listener.transformPose("table", approach_pose_offset)
                approach_pose.pose.position.x += approach_offset_in_table
                if self.debug:
                    rospy.loginfo("The height of the approach is  %f", approach_pose.pose.position.z - 0.05)
            except Exception, e:
                rospy.logerr("could not transform pose (approach). Exception: %s", str(e))
                return False
            
            
            # planning configuration (approach)
            self.mgc.allow_replanning(False)
            self.mgc.set_planning_time(10)
            max_number_of_planning_trials = 5
            
#             (pre_grasp_config, error_code) = sss.compose_trajectory("arm","pre_grasp")
#             if error_code != 0:
#                 rospy.logerr("unable to parse pre_grasp configuration")
#                 return False
#             start_state = RobotState()
#             start_state.joint_state.name = pre_grasp_config.joint_names
#             start_state.joint_state.position = pre_grasp_config.points[0].positions
#             start_state.attached_collision_objects = self.attached_collsion_objs
#             start_state.is_diff = True
#             self.mgc.set_start_state(start_state)
            
            #override for trial run
            start_state = self.rc.get_current_state()
            start_state.attached_collision_objects = self.attached_collsion_objs
            self.mgc.set_start_state(start_state)
            
            if self.debug:
                self.print_aco_names()
                self.print_co_names()
                self.broadcast_tf_frame(approach_pose, "Debug_Grasp_Frame")
            approach_success = False
            n_attempts = 0
            rotation_offset = 0
            
            if self.debug:
                rotations = 0
#                 while rotations <= 2*math.pi :
#                     pss = self.creat_pose_offset("current_box", [0, 0, rotations],[0, 0, approach_height_above_grasp])
#                     rospy.logwarn("this is the pose")
#                     rospy.logwarn(pss)
#                     psst = self.listener.transformPose("table", pss)
#                     rospy.logwarn("this is the pose in the tabel")
#                     rospy.logwarn(psst)
#                     self.broadcast_tf_frame(psst, "Debug_Grasp_Frame")
#                     rotations += 30/180.0*math.pi
#                     rospy.sleep(1.0)
#                 index_of_grasp = 0
#                 while index_of_grasp <len(userdata.available_grasps):
#                     rospy.logwarn("The number of grasps are: %s", len(userdata.available_grasps))
#                     self.broadcast_tf_frame(userdata.available_grasps[index_of_grasp].pose,"current_box")
#                     rospy.logwarn("published")
#                     rospy.sleep(2.0)
#                     index_of_grasp += 1
#                 sys.exit()
            
            if self.plan_mode == 1:
                approach_pose_intermediate = copy.deepcopy(approach_pose)
                if approach_pose_intermediate.pose.position.z < 0.3:
                    approach_pose_intermediate.pose.position.z = 0.3 # offset by 20 cm from the frame
                while approach_success == False and n_attempts<max_number_of_planning_trials:
                    (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([approach_pose_intermediate.pose,approach_pose.pose], self.eef_step, self.jump_threshold, True)
                    if not (frac_approach == 1.0):
                        rotation_offset += 60/180.0*math.pi
                        approach_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset],[0, 0, approach_height_above_grasp])
                        approach_pose = self.listener.transformPose("table", approach_pose_offset)
                        approach_pose.pose.position.x += approach_offset_in_table
                        approach_pose_intermediate = copy.deepcopy(approach_pose)
                        if approach_pose_intermediate.pose.position.z < 0.2:
                            approach_pose_intermediate.pose.position.z = 0.2 # offset by 20 cm from the frame
                        if self.debug:
                            # publish the new frame
                            rospy.logwarn("The Rotation offset is: %f degrees", rotation_offset*180.0/math.pi)
                            self.broadcast_tf_frame(approach_pose, "Debug_Grasp_Frame")
                            rospy.logerr("Unable to plan approach trajectory during approach before grasp. frac = %f", frac_approach)
                            rospy.loginfo("Pick attempt: " +  str(n_attempts))
                        n_attempts += 1
                        rospy.sleep(0.1)
                    else:
                        approach_success = True
                        rospy.logwarn("Pick attempt: " +  str(n_attempts) +" Has worked using (cartesian_path)" )
                        break
                if approach_success == False:
                    grasp_number +=1
                    continue
                
            elif self.plan_mode == 2:
                while approach_success == False and n_attempts<max_number_of_planning_trials:
                    self.mgc.clear_pose_targets()
                    self.mgc.set_pose_target(approach_pose.pose)
                    traj_approach = self.mgc.plan()
                    if traj_approach.joint_trajectory.header.frame_id:
                        if self.debug:
                            rospy.logwarn("Pick attempt: " +  str(n_attempts) +" Has worked using (plan)" )
                        approach_success = True
                        frac_approach = 1.0
                        break
                    else:
                        if self.debug:
                            rospy.loginfo("Pick attempt: " +  str(n_attempts))
                            self.broadcast_tf_frame(approach_pose, "Debug_Grasp_Frame")
                        rotation_offset += 60/180.0*math.pi
                        approach_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset],[0, 0, approach_height_above_grasp])
                        approach_pose = self.listener.transformPose("table", approach_pose_offset)
                        approach_pose.pose.position.x += approach_offset_in_table
                        rospy.sleep(0.1)
                        n_attempts += 1
                if not approach_success:
                    grasp_number +=1
                    continue
            else:
                rospy.logwarn("Planning mode does not exist please check and try again")
                sys.exit()
    
            ### start of grasping
            ### remove gripper collision objects
#             self.manage_attached_collision_obj(link_attached_to_ef, mb_ef_collisonobj_name, mb_ef_pose, mb_ef_size, 1, False, True, mb_ef_collisonobj_touch_links)
#             self.manage_attached_collision_obj(link_attached_to_ef, sucker_ef_collisonobj_name, sucker_ef_pose, sucker_ef_size, 1, False, True, mb_ef_collisonobj_touch_links)
            if self.debug:
#                 rospy.logwarn("Gripper collision objects have been removed")
                raw_input("Finished planining approach view in rviz")
            ### Set next (virtual) start state
            traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
            start_state.joint_state.position = traj_approach_endpoint.positions
            start_state.is_diff = True
            start_state.attached_collision_objects = self.attached_collsion_objs
            self.mgc.set_start_state(start_state)
    
            ### Plan Grasp
            # Parameters
            grasp_height_above_grasp = -0.005 #go deeper than grasp for grip
            grasp_offset_in_table = 0#-0.01
            
            grasp_pose_offset = PoseStamped()
            grasp_pose_offset.header.frame_id = "current_box"
            grasp_pose_offset.header.stamp = rospy.Time(0)
            grasp_pose_offset.pose.position.z += grasp_height_above_grasp
    
#             grasp_pose_offset = copy.deepcopy(userdata.available_grasps[grasp_number].pose)
#             grasp_pose_offset.header.stamp = rospy.Time(0)
#             grasp_pose_offset.pose.position.z += -0.005
            
            grasp_pose = self.listener.transformPose("table", grasp_pose_offset)
            
            # these lines have been added to clear the Octamap around the object to be grasped
            if self.debug:
                raw_input("The cylinder will be added press enter to add")
            cylinder_pose = grasp_pose
            """
            @todo: make the cylinder_dimentions parameters to be read and not constants
            """
            cylinder_dim = [0.004,0.03] # [m]
            cylinder_name = "cylinder_1"
            self.manage_collison_obj(cylinder_name, cylinder_pose, cylinder_dim, 2, True, True)
            self.clear_octomap_service()
            if(not self.trigger_octo_map_update().success):
                rospy.logwarn("The Octomap update has failed")
                raw_input("there is a problem updating the octo map program has paused")
            if self.debug:
                raw_input("updating octo map after reset")
            
            # add cylinder to allowed collision matrix
            request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
            response = self.get_planning_scene_service(request)
            acm = response.scene.allowed_collision_matrix
            if self.debug:
                rospy.logwarn(acm)
                raw_input("above is the ACM matrix")
            if not 'cylinder_1' in acm.default_entry_names:
                acm.default_entry_names += ['cylinder_1']
                acm.default_entry_values += [True]
            planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
            self._pubPlanningScene.publish(planning_scene_diff)
            rospy.sleep(0.1)
            
            grasp_pose.pose.position.x += grasp_offset_in_table # the correct grasp in the tabel's coordinate frame static offset till we calibrate
            self.broadcast_tf_frame(grasp_pose, "Debug_Grasp_Frame")
            
            grasp_success = False
            rotation_offset = 0
            n_attempts = 0
            frac_grasp = 0

            if self.debug:
                self.print_aco_names()
                print ""
                self.print_co_names()
                print ""
            if self.plan_mode == 1:
                while (frac_grasp != 1.0 and n_attempts<max_number_of_planning_trials):
                    (traj_grasp,frac_grasp) = self.mgc.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
                    if frac_grasp != 1.0:
                            rotation_offset += 60/180.0*math.pi
                            grasp_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset], [0, 0, grasp_height_above_grasp])
                            grasp_pose = self.listener.transformPose("table", grasp_pose_offset)
                            if self.debug:
                                self.broadcast_tf_frame(grasp_pose, "Debug_Grasp_Frame")
                                rospy.loginfo("Grasp attempt: " +  str(n_attempts))
                            rospy.sleep(0.1)
                            n_attempts += 1
                    else:
                        grasp_success == True
                        break
            elif self.plan_mode == 2:
                while grasp_success == False and n_attempts<max_number_of_planning_trials:
                    self.mgc.clear_pose_targets()
                    self.mgc.set_pose_target(grasp_pose.pose)
                    traj_grasp = self.mgc.plan()
                    if traj_grasp.joint_trajectory.header.frame_id:
                        if self.debug:
                            rospy.logwarn("Grasp attempt: " +  str(n_attempts) +" Has worked using (plan)" )
                            raw_input("The grasp has worked check Rviz and press any key to execute")
                            sss.move("arm", "pre_grasp")
                            print self.mgc.execute(traj_approach)
                            raw_input("press any key to grasp")
                            print self.mgc.execute(traj_grasp)
                        grasp_success = True
                        frac_grasp = 1.0
                        break
                    else:
                        if self.debug:
                            rospy.loginfo("Grasp attempt: " +  str(n_attempts) + " using (plan)")
                            self.broadcast_tf_frame(grasp_pose, "Debug_Grasp_Frame")
#                             raw_input("The grasp has not worked check press any key to try again")
                        rotation_offset += 60/180.0*math.pi
                        grasp_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset],[0, 0, grasp_height_above_grasp])
                        grasp_pose = self.listener.transformPose("table", grasp_pose_offset)
                        grasp_pose.pose.position.x += grasp_offset_in_table
                        rospy.sleep(0.1)
                        n_attempts += 1
                if not grasp_success:
                    grasp_number +=1
                    raw_input("The cylinder has been added press enter to remove")
                    self.manage_collison_obj(cylinder_name, cylinder_pose, cylinder_dim, 2, False, True)
                    raw_input("The cylinder has been removed press enter to continue")
                    continue
            else:
                rospy.logwarn("Planning mode does not exist please check and try again")
                sys.exit()
            
            if not (frac_grasp == 1.0):# this is used only if we do not find a path using plan or cartesian path plan
                if self.debug:
                    rospy.logwarn("Unable to plan (grasp) trajectory using the selected planning method approach. frac = %f", frac_grasp)
                    rospy.loginfo("Will attempt to find alternative configuration using IK")
                grasp_pose_alt_config = copy.deepcopy(grasp_pose)
                grasp_pose_alt_config.pose.position.z = 0.25 #[m]#box_long_side_dim[2]*1.1

                # replan approach
                start_state = self.rc.get_current_state()
                start_state.attached_collision_objects = self.attached_collsion_objs
                self.mgc.set_start_state(start_state)
                (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([grasp_pose_alt_config.pose], self.eef_step, self.jump_threshold, True)
                traj_grasp, status_flag = self.find_alternative_configuration_path(grasp_pose, grasp_pose_alt_config, 10, 30/180.0*math.pi, 60/180.0*math.pi, start_state)
                # display part
                if status_flag:
                    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        DisplayTrajectory)
                    display_trajectory = DisplayTrajectory()
    
                    display_trajectory.trajectory_start = start_state
                    display_trajectory.trajectory.append(traj_grasp)
                    display_trajectory_publisher.publish(display_trajectory)
                
                if status_flag == False or frac_approach!= 1.0:
                    if self.debug:
                        rospy.logerr("Unable to plan grasp trajectory using both IK and the selected plan method")
                    grasp_number +=1
                    self.manage_collison_obj(cylinder_name, cylinder_pose, cylinder_dim, 2, False, True)
                    continue
                else:
                    frac_grasp = 1.0
                    if self.debug:
                        rospy.loginfo("plan (grasp) trajectory worked using IK")
            if self.debug:
                raw_input("calculated grasp")
            
            ### since the grasp planning was successful it makes sense to attach the object to the gripper ( this means later it will be possible to adjust the objects size and remove it from the scene
            # by adding to the ACO it is automatically removed from the CO
            self.manage_attached_collision_obj(link_attached_to_ef, cylinder_name, cylinder_pose, cylinder_dim, 2, True, False, ef_collisonobj_touch_links)
            
            if self.debug:
                self.print_aco_names()
                self.print_co_names()
                raw_input("the link has been attach, verify the coherency of the collision objects")
            
            ### Set next (virtual) start state
            traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
            start_state.joint_state.position = traj_grasp_endpoint.positions
            start_state.is_diff = True
            start_state.attached_collision_objects = self.attached_collsion_objs
            self.mgc.set_start_state(start_state)
    
            ### Plan Lift
            #parameters
            lift_height_above_grasp = 0.3
            
            
            lift_pose_offset = PoseStamped()
            lift_pose_offset.header.frame_id = "current_box"
            lift_pose_offset.header.stamp = rospy.Time(0)
    
#             lift_pose_offset = copy.deepcopy(userdata.available_grasps[grasp_number].pose)
#             lift_pose_offset.header.stamp = rospy.Time(0)
            
            #quat = tf.transformations.quaternion_from_euler(0, 1.57, 1.57)
            #lift_pose_offset.pose.orientation.x = quat[0]
            #lift_pose_offset.pose.orientation.y = quat[1]
            #lift_pose_offset.pose.orientation.z = quat[2]
            #lift_pose_offset.pose.orientation.w = quat[3]
            lift_pose = self.listener.transformPose("table", lift_pose_offset)
            lift_pose.pose.position.z = lift_height_above_grasp
    #         lift_pose = self.listener.transformPose("camera_rgb_optical_frame", lift_pose_offset)
            #DELETE START
            #lift_pose.pose.orientation.x = 0
            #lift_pose.pose.orientation.y = 0
            #lift_pose.pose.orientation.z = 0
            #lift_pose.pose.orientation.w = 1
            #DELETE END
            if self.debug:
                self.print_aco_names()
                self.print_co_names()
                self.broadcast_tf_frame(lift_pose, "Debug_Grasp_Frame")
            rotation_offset = 0
            n_attempts = 0
            frac_lift = 0
            while (frac_lift != 1.0 and n_attempts<max_number_of_planning_trials):
                (traj_lift,frac_lift) = self.mgc.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
                if frac_lift != 1.0:
                        rotation_offset += 60/180.0*math.pi
                        lift_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset])
                        lift_pose = self.listener.transformPose("table", lift_pose_offset)
                        lift_pose.pose.position.z = lift_height_above_grasp
                        if self.debug:
                            self.broadcast_tf_frame(lift_pose, "Debug_Grasp_Frame")
                        rospy.sleep(0.1)
                        n_attempts += 1
                else:
                    break

            #traj_lift = self.smooth_cartesian_path(traj_lift)
            
            # here is the detachment of the attached box (it is detached regardless of whether the lift succeeds or not
            self.manage_attached_collision_obj(link_attached_to_ef, cylinder_name, cylinder_pose, cylinder_dim, 2, False, True, ef_collisonobj_touch_links)
            
            if not (frac_lift == 1.0):
                rospy.logerr("Unable to plan lift trajectory. frac = %f", frac_lift)
                grasp_number +=1
                continue
            
            if self.debug:
                raw_input("calculated lift")
                
            #override for trial run
            ### Set next (virtual) start state
#             traj_lift_endpoint = traj_lift.joint_trajectory.points[-1]
#             start_state = RobotState()
#             start_state.joint_state.name = traj_lift.joint_trajectory.joint_names
#             start_state.joint_state.position = traj_lift_endpoint.positions
#             start_state.is_diff = True
#             start_state.attached_collision_objects = self.attached_collsion_objs
#             self.mgc.set_start_state(start_state)            
#             
#             rotation_offset = 0
#             n_attempts = 0
#             frac_return = 0
#             while (frac_return != 1.0 and n_attempts<max_number_of_planning_trials):
#                 (traj_return,frac_return) = self.mgc.compute_cartesian_path([start_pose.pose], self.eef_step, self.jump_threshold, True)
#                 if frac_return != 1.0:
#                         rotation_offset += 60/180.0*math.pi
#                         lift_pose_offset = self.creat_pose_offset("current_box", [0, 0, rotation_offset])
#                         lift_pose = self.listener.transformPose("table", lift_pose_offset)
#                         lift_pose.pose.position.z = lift_height_above_grasp
#                         if self.debug:
#                             self.broadcast_tf_frame(start_pose, "Debug_Grasp_Frame")
#                         rospy.sleep(0.1)
#                         n_attempts += 1
#                 else:
#                     break
# 
#             #traj_lift = self.smooth_cartesian_path(traj_lift)
#             
#             if not (frac_return == 1.0):
#                 rospy.logerr("Unable to plan return trajectory. frac = %f", frac_return)
#                 raw_input("prompt")
#                 grasp_number +=1
#                 continue
#             
#             if self.debug:
#                 raw_input("calculated return")

            #if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0 and not traj_pre_grasp == None):
            if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):# and frac_return == 1.0):
                rospy.logerr("Unable to plan whole grasping trajectory")
                rospy.logerr("Approach: ", frac_approach, " Grasp: ", frac_grasp, " Lift: ")#, frac_lift, " Return: ", frac_return )
                grasp_number +=1
                continue
            else: 
                if self.debug:
                    rospy.logwarn("The grasp (in list of grasps) number: %i is successful", grasp_number)
                successful_trajectory_comp = True
                break
        
        if successful_trajectory_comp != True:
            if self.debug:
                rospy.logwarn("there were no feasible trajectories in the list of available_grasps")
            userdata.new_box = True
            rospy.sleep(0.1)
            return False       
        
        if self.debug:
                self.print_aco_names()
                self.print_co_names()
                raw_input("the link has been attach, verify the coherency of the collision objects before next planning cycle ")        

# #         fix trajectories with zero velocities
#         for i in range(len(traj_approach.joint_trajectory.points)):
#             traj_approach.joint_trajectory.points[i].velocities = [0]*7
#         for i in range(len(traj_grasp.joint_trajectory.points)):
#             traj_grasp.joint_trajectory.points[i].velocities = [0]*7
#         for i in range(len(traj_lift.joint_trajectory.points)):
#             traj_lift.joint_trajectory.points[i].velocities = [0]*7
# 
#         # fix trajectories to stop at the end
#         #traj_approach.joint_trajectory.points[-1].velocities = [0]*7
#         #traj_grasp.joint_trajectory.points[-1].velocities = [0]*7
#         #traj_lift.joint_trajectory.points[-1].velocities = [0]*7
#         
#         # fix trajectories to be slower/faster
#         speed_factor = 1.00
#         for i in range(len(traj_approach.joint_trajectory.points)):
#             traj_approach.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor
#         for i in range(len(traj_grasp.joint_trajectory.points)):
#             traj_grasp.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor
#         for i in range(len(traj_lift.joint_trajectory.points)):
#             traj_lift.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor

        traj_approach = self.traj_fix_and_change_velocity(traj_approach, 1.0)
        traj_grasp = self.traj_fix_and_change_velocity(traj_grasp, 1.0)
        traj_lift  = self.traj_fix_and_change_velocity(traj_lift, 1.0)



        print "\n##### planning time", (rospy.Time.now() - start_plan).to_sec()
        if self.execute_motions:
            ### execute
            start_execute = rospy.Time.now()
            #override for trial run
#             if self.debug:
#                 raw_input("moveing to pre_grasp")

#             handle_arm = sss.move("arm", "pre_grasp")
#             print handle_arm.get_error_code()
            
            
            rospy.loginfo("approach")
            traj_approach = self.smooth_cartesian_path(traj_approach)
            if self.debug:
                raw_input( "pelase press enter to contiune to grasp")
            print self.mgc.execute(traj_approach)
            
            rospy.loginfo("grasp")
            traj_grasp = self.smooth_cartesian_path(traj_grasp)
            if self.debug:
               raw_input("please press enter to continue to grasp and turn on suction")
            if not self.simulation:
                self.close_gripper()
            print self.mgc.execute(traj_grasp)
            rospy.sleep(0.1)       
    
                        
            rospy.loginfo("lift")
            traj_lift = self.smooth_cartesian_path(traj_lift)
            if self.debug:
                raw_input("pelase press enter to continue to lift")
    #         sss.wait_for_input()#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            print self.mgc.execute(traj_lift)
            
#             rospy.loginfo("Return")
#             traj_lift = self.smooth_cartesian_path(traj_return)
#             if self.debug:
#                 raw_input("pelase press enter to continue to return")
#                 rospy.logwarn(traj_lift)
#                 rospy.loginfo(traj_return)
#     #         sss.wait_for_input()#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#             print self.mgc.execute(traj_return)
            
#             override for trial run
            if self.debug:
                raw_input( "pelase press enter to contiune to retreat")
            handle_arm = sss.move("arm", ["retreat"])
            print handle_arm.get_error_code()
            
            if not self.simulation:
                self.open_gripper()
        
            userdata.new_box = True
    
            print "\n##### execution time", (rospy.Time.now() - start_execute).to_sec()
            if self.debug:
                raw_input("End of grasp please press enter to continue to next object")
    #         rospy.sleep(3)
    #         sss.wait_for_input()
    
            return True
        else:
            userdata.new_box = True
            return True
        
    def creat_pose_offset(self, name_of_parent_frame, angle = [], position = []):
        """
        @summary: creates a PoseStamed message who's pose is defined by the inputs in the parent frame 
        @param angle: A list of angles (RPY) that will be used to rotate the frame
        @param position: A list of coordinates (x,y,z) that will be used to translate the frame
        @return: PoseStamed
        """
        if len(angle) == 0:
            angle = [0]*3
        if len(position) == 0:
            position = [0]*3
        pose_offset = PoseStamped()
        pose_offset.header.frame_id = name_of_parent_frame
        pose_offset.header.stamp = rospy.Time(0)
        quat = tf.transformations.quaternion_from_euler(angle[0],angle[1], angle[2])
        pose_offset.pose.orientation.x = quat[0]
        pose_offset.pose.orientation.y = quat[1]
        pose_offset.pose.orientation.z = quat[2]
        pose_offset.pose.orientation.w = quat[3]
        
        pose_offset.pose.position.x = position[0]
        pose_offset.pose.position.y = position[1]
        pose_offset.pose.position.z = position[2]
        
        return pose_offset
            
        
    def rotate_pose_stamped(self, frame_name_pose_change, frame_output_id, frame_name_publish="",angle = [], position = []):
        """
        @deprecated: This function is currently not in use
        @summary: this function takes a sting (frame_name_pose_change)
        and changes its pose according to the inputs, after the update the new frame is published
        @param frame_name_pose_change: String of the frame name
        @param frame_output_id: the name of the frame to which the pose will be transformed in the end
        @param angle: A list of angles (RPY) that will be used to rotate the frame
        @param position: A list of coordinates (x,y,z) that will be used to translate the frame
        """
        if len(angle) == 0:
            angle = [0]*3
        if len(position) ==0:
            position = [0]*3
        if frame_name_publish =="":# if the name is empty then update the original frame
            frame_name_publish = frame_name_pose_change
        
        pose_offset = PoseStamped()
        pose_offset.header.frame_id = frame_name_pose_change
        pose_offset.header.stamp = rospy.Time(0)
        quat = tf.transformations.quaternion_from_euler(angle[0],angle[1], angle[2])
        pose_offset.pose.orientation.x = quat[0]
        pose_offset.pose.orientation.y = quat[1]
        pose_offset.pose.orientation.z = quat[2]
        pose_offset.pose.orientation.w = quat[3]
        
        pose_offset.pose.orientation.x = position[0]
        pose_offset.pose.orientation.y = position[1]
        pose_offset.pose.orientation.z = position[2]
        
        ps = self.listener.transformPose(frame_output_id, pose_offset)# transform the new pose to the frame id (reference frame)
        self.broadcast_tf_frame(ps,frame_name_publish)# update the old frame or publish a new one with the frame ID being the frame used to change the pose

    def find_alternative_configuration_path(self, pose_final, pose_initial, number_of_steps, maximum_ang_vel, maxdiff_between_joints, seed_point = "",collision_avoidance = True, move_group_commander = ""):
        """
        @param pose_final: The destination pose (posestamed)
        @param pose_initial: The Initial pose from which the movemnet will be calculated
        @param number_of_steps: The number of steps the move from pose_initial to pose_final will be descrietized to
        @param maximum_ang_velparam: The maximum allowable velocity for which joints are allowed to move [rad/s]
        @param maxdiff_between_joints: The maximum angular displacement allowed between IK solutions [rad]. used to check continuity of the solution
        @param seed_pointparam: The initial point for which the first IK is calculated [RobotState].
        @summary: This method attempts to plan between the pose_final and pose_inital a collision free path using a linear path. 
        The IK solution so calculated backwards form goal pose to initial pose and then reversed, this means that the initial point for the produced for the trajectory might be different in joint space,
        compared to the given seed for instance, this is desirable but during this motion collisions are not detected 
        The orientation information is not included, meaning that the the movement should always maintain the same orientation.
        @note: collisions are only avoided at the calculated points, any points in between could contain collisions and will not be detected.
        """
        
        if move_group_commander == "":
            move_group_commander = self.mgc
        if seed_point == "":
            seed_point = move_group_commander.get_current_state()
        seed_point_c = copy.deepcopy(seed_point)
        
        # internal variables used
        frame_id = ""
        success_flag = False
        joint_Positions = [] # list of the IK solutions
        
        # create an IK request
        rospy.wait_for_service("/compute_ik")
        PIKRservice = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        
        # complete difference
        delta_list = [pose_final.pose.position.x - pose_initial.pose.position.x,  pose_final.pose.position.y - pose_initial.pose.position.y, pose_final.pose.position.z - pose_initial.pose.position.z]
        # incremental difference
        delta_list = [element/number_of_steps for element in delta_list]
        
        
        for step_current in xrange(0, number_of_steps):
            
            pose_current = copy.deepcopy(pose_final)
            # updating the location based on cartesian  movements backward from grasp to approach starting with grasp
            pose_current.pose.position.x -= step_current*delta_list[0]
            pose_current.pose.position.y -= step_current*delta_list[1]
            pose_current.pose.position.z -= step_current*delta_list[2]
            
            PIKR = PositionIKRequest()
            PIKR.attempts = 5
            PIKR.avoid_collisions = collision_avoidance
            PIKR.group_name = move_group_commander.get_name()
            PIKR.pose_stamped = pose_current
            PIKR.robot_state = seed_point_c # start_state
            PIKR.timeout = rospy.Duration(1, 0)
            IK_output = PIKRservice(PIKR)
            if self.debug:
                rospy.loginfo("The invers Kinematic solution for step %s is: ", step_current)
                rospy.loginfo(IK_output.solution.joint_state.position)
            joint_Positions.append(list(IK_output.solution.joint_state.position))
            seed_point_c.joint_state.position = list(IK_output.solution.joint_state.position)
            
            if IK_output.error_code.val != MoveItErrorCodes.SUCCESS:
                rospy.logerr("IK encountered a problem Error code: %s", IK_output.error_code)
                success_flag = False
                return None, success_flag
        joint_Positions.reverse() # the calculation was done from the final pose to the initial pose, this should be reversed before execution
        
        if self.debug:
            rospy.loginfo("the list of the IK solutuions is as follows: ")
            rospy.loginfo(joint_Positions)
            rospy.loginfo("The length is: %s", len(joint_Positions))
        
        if self.check_continuity(joint_Positions, maxdiff_between_joints):
            frame_id = pose_final.header.frame_id
            success_flag = True
        else:
            success_flag = False
        return self.create_RobotTrajectory(joint_Positions, move_group_commander.get_active_joints(), frame_id, maximum_ang_vel, seed_point.joint_state.position), success_flag
    
    def check_continuity(self, input_list, Threshold):
        """
        @summary: checks the continuity of a list element wise compared to a threshold
        @return: Bool
        """
        for index in xrange(0,len(input_list)-1):
            diff = map(sub, input_list[index], input_list[index + 1])
            if any((diff_entry> Threshold or diff_entry < -Threshold) for diff_entry in diff):
                rospy.logwarn("Discontinuity detected ...")
                rospy.logwar("The threshold is: %s and the max diff is: %s the min diff is: %s",(Threshold, max(diff),min(diff)))
                return False
            else:
                return True
        
    def create_RobotTrajectory(self, list_of_joint_angles, names_of_joints, frame_in_which_traj_was_planed, maximum_ang_vel, state_from_previous_planning_step):
        """
        @summary: Creates a robot trajectory from a given list of joint angels. The time between points is calculated according to the maximum_ang_vel
        @note: The inital time calculation for the trajectory (the first element in the list), is done using the state of the robot, state_from_previous_planning_step
        """
        accumelated_time = rospy.Duration()
        Trial_traj = RobotTrajectory()
        Trial_traj.joint_trajectory.header.frame_id = frame_in_which_traj_was_planed
        Trial_traj.joint_trajectory.joint_names = names_of_joints
        for count in xrange(0,len(list_of_joint_angles)):
            current_point = JointTrajectoryPoint()
            current_point.positions = list_of_joint_angles[count]
            if(count == 0):
                diff = map(sub, state_from_previous_planning_step, current_point.positions)
            else:
                diff = map(sub, list_of_joint_angles[count-1], current_point.positions)
            max_diff = max(abs(value) for value in diff)
            min_time_needed = max_diff/maximum_ang_vel # in secs float
            duration_needed = rospy.Duration.from_sec(min_time_needed)
            complete_duration = duration_needed + accumelated_time
            accumelated_time = complete_duration # updated for next loop
            current_point.time_from_start = complete_duration
            Trial_traj.joint_trajectory.points.append(current_point)
        
        if self.debug:
            rospy.loginfo("The generated Trajectory is: %s", Trial_traj)
        
        return Trial_traj

    def broadcast_tf_frame(self, Frame, name):
        """
        @type Frame: PoseStamped
        @type name: String
        @summary: publishes a new frame or updates an old frame with the name and pose which are inputs
        """
        if Frame != "":
            self.br.sendTransform(
                    (Frame.pose.position.x, Frame.pose.position.y, Frame.pose.position.z),
                    #(0, 0, 0, 1),
                    (Frame.pose.orientation.x, Frame.pose.orientation.y, Frame.pose.orientation.z, Frame.pose.orientation.w),
                    rospy.Time.now(),
                    name,
                    Frame.header.frame_id)
            if name == "current_box":
                self.br.sendTransform(
                    (Frame.pose.position.x, Frame.pose.position.y, Frame.pose.position.z),
                    #(0, 0, 0, 1),
                    (Frame.pose.orientation.x, Frame.pose.orientation.y, Frame.pose.orientation.z, Frame.pose.orientation.w),
                    rospy.Time.now(),
                    "current_box_fix",
                    Frame.header.frame_id)
        
    def traj_fix_and_change_velocity(self, input_traj, velocty_multiplyer = 1.0 ,fix = True):
        for i in range(len(input_traj.joint_trajectory.points)):
            if fix:
                input_traj.joint_trajectory.points[i].velocities = [0]*7
            input_traj.joint_trajectory.points[i].time_from_start *= 1.0/velocty_multiplyer
        return input_traj
    
    def smooth_cartesian_path(self, traj):
        #print traj
        time_offset = 0.2
        
        # smooth first point
        for i in range(len(traj.joint_trajectory.points)):
            if i == 0:
                continue
            else:
                traj.joint_trajectory.points[i].time_from_start += rospy.Duration(time_offset)
        
        # smooth last point
        traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(time_offset)
        
        return traj

    def print_co_names(self):
        rospy.logwarn("The names of the collision objects are: ")
        for index, co in enumerate(self.collision_objs):
            rospy.logwarn("%s  %s  ", index, co.id)
    def print_aco_names(self):
        rospy.logwarn("The names of the attached collision objects are: ")
        for index, aco in enumerate(self.attached_collsion_objs):
            rospy.logwarn("%s  %s  ", index, aco.object.id)

    def plot_traj(self, traj):
        number_of_points = len(traj.joint_trajectory.points)
        min_y = 0
        max_y = 0
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm","pre_grasp")
        last_pos = pre_grasp_config.points[-1].positions

        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[0]]*6, 'bo')
        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[1]]*6, 'bo')
        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[2]]*6, 'bo')
        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[3]]*6, 'bo')
        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[4]]*6, 'bo')
        plt.plot([-0.5, -0.4, -0.3, -0.2, -0.1, -0.0], [last_pos[5]]*6, 'bo')

        traj = self.smooth_cartesian_path(traj)

        for point in traj.joint_trajectory.points:
            x = [point.time_from_start.to_sec()]*6
            y = point.positions
            print last_pos
            print y
            vel = numpy.subtract(last_pos, y)
            print "x =", x
            print "y =", y
            print "vel =", vel
            plt.plot(x, vel, 'ro')
            min_y = min(min(y),min_y)
            max_y = max(max(y),max_y)
        plt.axis([-0.5, traj.joint_trajectory.points[-1].time_from_start.to_sec(),min_y, max_y])
        plt.show()


class SM(smach.StateMachine):# this braket showes that SM inherits smach.StateMachine
    def __init__(self):        
        smach.StateMachine.__init__(self,outcomes=['ended'])
        
        self.userdata.new_box = True
        self.userdata.available_grasps = []
        
        with self:

#            smach.StateMachine.add('GRASP',Grasp(),
#                transitions={'succeeded':'GRASP',
#                    'failed':'GRASP'})

            smach.StateMachine.add('SELECT',SelectBox(),
                transitions={'succeeded':'GRASP',
                    'failed':'SELECT'})

            smach.StateMachine.add('GRASP',Grasp(),
                transitions={'succeeded':'SELECT',
                    'failed':'SELECT'})

#            smach.StateMachine.add('GRASP2',Grasp(),
#                transitions={'succeeded':'SELECT',
#                    'failed':'GRASP3'})

#            smach.StateMachine.add('GRASP3',Grasp(),
#                transitions={'succeeded':'SELECT',
#                    'failed':'SELECT'})


if __name__=='__main__':
    rospy.init_node('grasp')
    print "please crop the Pointcloud before continuing"
    sss.move("arm", "retreat")
    sss.wait_for_input()
    sm = SM() #craete the state machine
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')#(server_name, state, path )
    sis.start()# start the server for communication with the State machine
    outcome = sm.execute()# start the sate machine and prevent the addition of new states
    rospy.spin()
    sis.stop()
