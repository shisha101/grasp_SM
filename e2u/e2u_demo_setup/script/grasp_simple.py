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

from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject


from shape_msgs.msg import SolidPrimitive
from cob_object_detection_msgs.msg import DetectionArray
#from cob_3d_mapping_msgs.msg import TriggerGoal
from ur_msgs.srv import SetIO, SetIORequest
import simple_moveit_interface as smi

from simple_script_server import *
sss = simple_script_server()





class SelectBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['new_box'],
            output_keys=['new_box'])

        self.current_box_fix = ""
        self.current_box = ""
        self.rotation_offset = 0
        rospy.Subscriber("/find_suction_grasps/bounding_box_array", DetectionArray, self.callback)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        # initialize tf listener
        self.listener = tf.TransformListener()
        self.trigger_segmentation = rospy.ServiceProxy('/find_suction_grasps/trigger', std_srvs.srv.Trigger)
        rospy.sleep(1)
        
    def execute(self, userdata):
        start_detect = rospy.Time.now()
        rospy.wait_for_service('/find_suction_grasps/trigger')
                
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
            while self.current_box_fix == "" and not rospy.is_shutdown():#waiting for call back to return the selected box
                if rospy.Time.now() > start_time + rospy.Duration(5):
                    print "timeout while waiting for result. Repeating select"
                    return "failed" #results in a loop here until a grasp is found 
                #print "waiting for result"
                #print self.current_box_fix
                rospy.sleep(0.01)
            userdata.new_box = False
            print self.current_box_fix
            self.current_box = copy.deepcopy(self.current_box_fix)

            print "\n##### detection time", (rospy.Time.now() - start_detect).to_sec()

            ############UGLY
            #rospy.sleep(0.5)
            
        else:
            # here the detected grasp is rotated by rotation_offset this is probably to solve the gripper collision problem
            if self.rotation_offset >= 2*math.pi:
                rospy.loginfo("resetting rotation")
                self.rotation_offset = 0.0
                userdata.new_box = True
                return "failed"
            self.rotation_offset += 30/180.0*math.pi
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
        print "the frame obtained from the topic is", bba.header.frame_id
        #####FIX wrong bounding box lwh calculations (needs to be fixed in box_detection and perception_common/cob_vision_utils (boundingBoxToMarker)###
        bba_out = DetectionArray()
        for bb in bba.detections:
            bb.bounding_box_lwh.x = bb.bounding_box_lwh.x*2
            bb.bounding_box_lwh.y = bb.bounding_box_lwh.y*2
            bba_out.detections.append(bb)
        bba = bba_out
        #####END FIX
        
        ########## defines ########
        # common
        distance_from_center = 0.2          # [m]
        upright_offset = 45.0/180.0*math.pi # [rad]
        edge_offset = 0.03                  # [m]
        relation_factor = 1.5
        height_threshold = 0.02             # [m]
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

        # center from_container
        bba = self.filter_far_away_from_center_of_container(bba, distance_from_center)
        print "--> after center filter:", len(bba.detections), "bounding boxes"

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

        self.broadcast_bba(bba)# rebroadcast on tf all the grasps in the distance defined by distance_from_center

        # select highest plane
        height = ""
        for bb in bba.detections:
            bb.pose.header.stamp = rospy.Time(0)
            height_above_table = self.listener.transformPose("table",bb.pose).pose.position.z
            #print "height_above_table", height_above_table
            #if height == 0:
            #    sss.wait_for_input()
            if height == "":
                bb_tmp = bb
                height = height_above_table
            elif height_above_table >= height:
                bb_tmp = bb
                height = height_above_table
            else:
                # not higher than highest plane
                pass
            if(height< height_threshold):
                print "The frame has been rejected due to not being high enough, the threshold is: ",height_threshold
                return
            else:
                print "height =", height
                print "bb_area =", bb_tmp.bounding_box_lwh.x * bb_tmp.bounding_box_lwh.y
                print "bb_x, bb_y = ", bb_tmp.bounding_box_lwh.x, ",", bb_tmp.bounding_box_lwh.y
                self.current_box_fix = bb_tmp # this is the detection array entry of the selected box this is defined in the table frame


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
            input_keys=[],
            output_keys=['new_box'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Move Group Commander
        self.mgc = MoveGroupCommander("manipulator")
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
#         
        
        ### initialize service for gripper on universal arm 
        self.io_srv = rospy.ServiceProxy('set_io', SetIO)
        
        self.eef_step = 0.01
        self.jump_threshold = 2
        rospy.logwarn("Initializing Grasp")
        rospy.sleep(1)
        
    def execute(self, userdata):
        self.open_gripper()
 
        box_dummy_pose = PoseStamped()
        box_dummy_pose.header.frame_id =  "table"
        box_dummy_pose.pose.position.x = 0.147
        box_dummy_pose.pose.position.y = 0.636
        box_dummy_pose.pose.position.z = 0
        self.psi.add_box("dummy_box", box_dummy_pose, (0.18,0.09,0.26))# #size x,y,z x is always to the left viewing the robot from the PC  
        rospy.logwarn("I have added the box")
         
#         
        # end effector collision object
        link_attached_to_ef = "ee_link"
          
        mb_ef_collisonobj = "metal_bracket"
        mb_ef_pose = PoseStamped()
        mb_ef_pose.header.frame_id =  link_attached_to_ef
        mb_ef_pose.pose.position.x = 0.065/2.0
        mb_ef_pose.pose.position.y = 0.0
        mb_ef_pose.pose.position.z = 0.0
        mb_ef_size = (0.065,0.06,0.06)
#         self.psi.attach_box(link_attached_to_ef, mb_ef_collisonobj, mb_ef_pose, mb_ef_size,[link_attached_to_ef, 'wrist_3_link'])

#         self.psi.add_box(mb_ef_collisonobj, mb_ef_pose, mb_ef_size)# #size x,y,z x is always to the left viewing the robot from the PC 
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
        rospy.sleep(0.5)
        if not self.plan_and_execute(userdata):
            userdata.new_box = False
            return "failed"
        
        return "succeeded"
           
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
        


        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm","pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return False
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
#         self.mgc.set_start_state(start_state)
        self.mgc.set_start_state_to_current_state()

        ### Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_box"
        approach_pose_offset.header.stamp = rospy.Time(0)
#         approach_pose_offset.pose.position.x -= 0.1
        approach_pose_offset.pose.position.z += 0.05
        #quat = tf.transformations.quaternion_from_euler(0, 1.57, 1.57)
        #approach_pose_offset.pose.orientation.x = quat[0]
        #approach_pose_offset.pose.orientation.y = quat[1]
        #approach_pose_offset.pose.orientation.z = quat[2]
        #approach_pose_offset.pose.orientation.w = quat[3]
        
        try:
            approach_pose = self.listener.transformPose("table", approach_pose_offset)
            approach_pose.pose.position.x += -0.01
#             approach_pose = self.listener.transformPose("camera_rgb_optical_frame", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False
            
        rospy.logwarn("Approach planninig without collision object")
        (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        
        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory during approach before grasp. frac = %f", frac_approach)
            #for i in range(len(traj_approach.joint_trajectory.points)):
            #    traj_approach.joint_trajectory.points[i].velocities = [0]*7
            #sss.wait_for_input()
            #sss.move("arm","pre_grasp")#DELETE
            #self.mgc.execute(traj_approach)#DELETE
#             return False
        else:
            rospy.logwarn("Planning without collision objects Success")
        sss.wait_for_input()
        

        ### Set next (virtual) start state
        traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
        start_state.joint_state.position = traj_approach_endpoint.positions
        start_state.is_diff = True
        self.mgc.set_start_state(start_state)

        ### Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_box"
        grasp_pose_offset.header.stamp = rospy.Time(0)
#         grasp_pose_offset.pose.position.x -= 0.1
        grasp_pose_offset.pose.position.z += -0.003
        #quat = tf.transformations.quaternion_from_euler(0, 1.57, 1.57)
        #grasp_pose_offset.pose.orientation.x = quat[0]
        #grasp_pose_offset.pose.orientation.y = quat[1]
        #grasp_pose_offset.pose.orientation.z = quat[2]
        #grasp_pose_offset.pose.orientation.w = quat[3]
        
        grasp_pose = self.listener.transformPose("table", grasp_pose_offset)
        grasp_pose.pose.position.x += -0.01
#         grasp_pose = self.listener.transformPose("camera_rgb_optical_frame", grasp_pose_offset)

        #DELETE START
        #grasp_pose.pose.orientation.x = 0
        #grasp_pose.pose.orientation.y = 0
        #grasp_pose.pose.orientation.z = 0
        #grasp_pose.pose.orientation.w = 1
        #DELETE END

        (traj_grasp,frac_grasp) = self.mgc.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        sss.wait_for_input()
        #traj_grasp = self.smooth_cartesian_path(traj_grasp)
        
        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory during grasp after approach. frac = %f", frac_grasp)
            #for i in range(len(traj_approach.joint_trajectory.points)):
            #    traj_approach.joint_trajectory.points[i].velocities = [0]*7
            #for i in range(len(traj_grasp.joint_trajectory.points)):
            #    traj_grasp.joint_trajectory.points[i].velocities = [0]*7
            #sss.move("arm","pre_grasp")#DELETE
            #self.mgc.execute(traj_approach)#DELETE
            #self.mgc.execute(traj_grasp)#DELETE
            return False

        ### Set next (virtual) start state
        traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
        start_state.joint_state.position = traj_grasp_endpoint.positions
        start_state.is_diff = True
        # start_state.attached_collision_objects =
        self.mgc.set_start_state(start_state)

        ### Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_box"
        lift_pose_offset.header.stamp = rospy.Time(0)
        #lift_pose_offset.pose.position.x -= 0.1
        lift_pose_offset.pose.position.z += 0.1
        #quat = tf.transformations.quaternion_from_euler(0, 1.57, 1.57)
        #lift_pose_offset.pose.orientation.x = quat[0]
        #lift_pose_offset.pose.orientation.y = quat[1]
        #lift_pose_offset.pose.orientation.z = quat[2]
        #lift_pose_offset.pose.orientation.w = quat[3]
        lift_pose = self.listener.transformPose("table", lift_pose_offset)
#         lift_pose = self.listener.transformPose("camera_rgb_optical_frame", lift_pose_offset)
        #DELETE START
        #lift_pose.pose.orientation.x = 0
        #lift_pose.pose.orientation.y = 0
        #lift_pose.pose.orientation.z = 0
        #lift_pose.pose.orientation.w = 1
        #DELETE END

        (traj_lift,frac_lift) = self.mgc.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        
        #traj_lift = self.smooth_cartesian_path(traj_lift)
        
        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory. frac = %f", frac_lift)
            return False

        #if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0 and not traj_pre_grasp == None):
        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            return False

        # fix trajectories with zero velocities
        for i in range(len(traj_approach.joint_trajectory.points)):
            traj_approach.joint_trajectory.points[i].velocities = [0]*7
        for i in range(len(traj_grasp.joint_trajectory.points)):
            traj_grasp.joint_trajectory.points[i].velocities = [0]*7
        for i in range(len(traj_lift.joint_trajectory.points)):
            traj_lift.joint_trajectory.points[i].velocities = [0]*7

        # fix trajectories to stop at the end
        #traj_approach.joint_trajectory.points[-1].velocities = [0]*7
        #traj_grasp.joint_trajectory.points[-1].velocities = [0]*7
        #traj_lift.joint_trajectory.points[-1].velocities = [0]*7
        
        # fix trajectories to be slower/faster
        speed_factor = 3.0
        for i in range(len(traj_approach.joint_trajectory.points)):
            traj_approach.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor
        for i in range(len(traj_grasp.joint_trajectory.points)):
            traj_grasp.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor
        for i in range(len(traj_lift.joint_trajectory.points)):
            traj_lift.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor

        print "\n##### planning time", (rospy.Time.now() - start_plan).to_sec()

        ### execute
        sss.wait_for_input()
        start_execute = rospy.Time.now()
        handle_arm = sss.move("arm", "pre_grasp")
        print handle_arm.get_error_code()
        
        #sss.wait_for_input()
        
        rospy.loginfo("approach")
        #self.plot_traj(traj_approach)
        traj_approach = self.smooth_cartesian_path(traj_approach)
#         self.plot_traj(traj_approach)
        print "pelase press enter to contiune to approach"
        print self.mgc.execute(traj_approach)
        #sss.wait_for_input()
        
        rospy.loginfo("grasp")
        #self.plot_traj(traj_grasp)
        traj_grasp = self.smooth_cartesian_path(traj_grasp)
        print "pelase press enter to contiune to grasp and turn on suction"
#         sss.wait_for_input()#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
#         rospy.sleep(2)
#         self.close_gripper()
        print self.mgc.execute(traj_grasp)
        rospy.sleep(0.1)
#         print"Please press enter to turn on suction"
#         sss.wait_for_input()            
#         self.close_gripper()
                    
        rospy.loginfo("lift")
        traj_lift = self.smooth_cartesian_path(traj_lift)
        print "pelase press enter to contiune to lift"
#         sss.wait_for_input()#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        print self.mgc.execute(traj_lift)
        #sss.wait_for_input()
        handle_arm = sss.move("arm", ["pre_grasp","retreat"])
        print handle_arm.get_error_code()

        self.open_gripper()
    
        userdata.new_box = True

        print "\n##### execution time", (rospy.Time.now() - start_execute).to_sec()
        print "End of grasp please press enter to continue to next object"
#         rospy.sleep(3)
#         sss.wait_for_input()

        return True
        
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
def attCollObjCb(data):
    print("attCollObjCb")
    print(data)



def simple_function():
        rc = RobotCommander()
        mgc = MoveGroupCommander("manipulator")
        # print(rc.get_group_names())
        # print(rc.get_group('manipulator'))
        
        
        # exit()
        
        eef_step = 0.01
        jump_threshold = 2
        ### Create a handle for the Planning Scene Interface
        psi = PlanningSceneInterface()
        
        
        rc.get_current_state()
        rospy.logwarn(rc.get_current_state())
        sss.wait_for_input()
        #rate = rospy.Rate(0.1) # 10hz
        rate = rospy.Rate(1) # 10hz
        rospy.sleep(1)
        
        
        theSub = rospy.Subscriber('/attached_collision_object', AttachedCollisionObject, attCollObjCb, queue_size = 1)
        
        while not rospy.is_shutdown():
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = "table"
            approach_pose.header.stamp = rospy.Time(0)
            approach_pose.pose.position.x = 0.146
            approach_pose.pose.position.y = 0.622
            approach_pose.pose.position.z = 0.01
            quat = tf.transformations.quaternion_from_euler(0, 0, 1.57/2)
            approach_pose.pose.orientation.x = quat[0]
            approach_pose.pose.orientation.y = quat[1]
            approach_pose.pose.orientation.z = quat[2]
            approach_pose.pose.orientation.w = quat[3]
#             mgc.set_start_state_to_current_state()
#             (traj_approach,frac_approach) = mgc.compute_cartesian_path([approach_pose.pose], eef_step, jump_threshold, True)
#             if(frac_approach != 1):
#                 rospy.logwarn("Planning did not succeed before adding collision objects")
#             else:
#                 rospy.logwarn("Planning succeeded before adding collision objects")
# 
#             rospy.logwarn("waiting for input to add dummy box")
#             sss.wait_for_input()
#             
            box_dummy_pose = PoseStamped()
            box_dummy_pose.header.frame_id =  "table"
            box_dummy_pose.pose.position.x = 0.147
            box_dummy_pose.pose.position.y = 0.636
            box_dummy_pose.pose.position.z = 0
            psi.add_box("dummy_box", box_dummy_pose, (0.18,0.09,0.26))# #size x,y,z x is always to the left viewing the robot from the PC  
#             rospy.logwarn("I have added the box")
#             rospy.sleep(1)
#             rospy.logwarn("waiting for input to try planning with dummy box")
#             sss.wait_for_input()
#             
#             (traj_approach,frac_approach) = mgc.compute_cartesian_path([approach_pose.pose], eef_step, jump_threshold, True)
#             if(frac_approach != 1):
#                 rospy.logwarn("Planning did not succeed after adding collision objects (dummy box)")
#             else:
#                 rospy.logwarn("Planning succeeded after adding collision objects (dummy box)")
#                 
            rospy.logwarn("waiting for input to add end effector box box")
            sss.wait_for_input()
            # end effector collision object
            link_attached_to_ef = "ee_link"
            mb_ef_collisonobj = "metal_bracket"
            mb_ef_pose = PoseStamped()
            mb_ef_pose.header.frame_id =  link_attached_to_ef
            mb_ef_pose.pose.position.x = 0.065/2.0
            mb_ef_pose.pose.position.y = 0.0
            mb_ef_pose.pose.position.z = 0.0
            mb_ef_size = (0.065,0.06,0.06)



            psi.attach_box(link_attached_to_ef, mb_ef_collisonobj, mb_ef_pose, mb_ef_size, [link_attached_to_ef, "wrist_3_link"])
            
            
            raw_input("0 hi")
            
            mgc.attach_object("dummy_box", link_attached_to_ef, [link_attached_to_ef, "wrist_3_link"])
            

            
            
            rospy.logwarn("I have added the attached box to the end effector")
            
            
            rospy.sleep(1)
            raw_input("1 hi")           
            
            rospy.logwarn(rc.get_current_state())
            # mgc.attach_object(object_name, link_name, touch_links)
            mgc.set_start_state_to_current_state()
            rospy.logwarn(rc.get_current_state())
            raw_input("2 hi")
            rospy.logwarn("waiting for input to try planning with end effector box")
            sss.wait_for_input()
            
            (traj_approach,frac_approach) = mgc.compute_cartesian_path([approach_pose.pose], eef_step, jump_threshold, True)
            if(frac_approach != 1):
                rospy.logwarn("Planning did not succeed after adding collision objects (dummy box)")
            else:
                rospy.logwarn("Planning succeeded after adding collision objects (dummy box)")
            
            rospy.logwarn("waiting for input to try planning next loop")
            sss.wait_for_input()
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('grasp')    
    print "please crop the Pointcloud before continuing"
#     sss.move("arm", "retreat")
#     sss.wait_for_input()
#     sm = SM() #craete the state machine
#     sis = smach_ros.IntrospectionServer('sm', sm, 'SM')#(server_name, state, path )
#     sis.start()# start the server for communication with the State machine
#     outcome = sm.execute()# start the sate machine and prevent the addition of new states
    rospy.logwarn("calling simple_function")
    simple_function()
    rospy.spin()
#     sis.stop()

