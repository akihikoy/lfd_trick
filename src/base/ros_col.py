#! /usr/bin/env python
#ROS tools (collision detection).
#DEPRECATED: This package is not maintained anymore.  Use ros_col_mi.
import roslib
roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('geometry_msgs')
import geometry_msgs.msg
import trajectory_msgs.msg
#For collision check
roslib.load_manifest('arm_navigation_msgs')
import arm_navigation_msgs.srv  #WARNING: Only supported in ROS<=Groovy
import arm_navigation_msgs.msg  #WARNING: Only supported in ROS<=Groovy

from base_geom import *
from base_const import *


#There must be a planning scene or FK / IK crashes
def SetupPlanningScene():
  print 'Waiting for /environment_server/set_planning_scene_diff...'
  rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  set_scene= rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
  req= arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
  set_scene(req)
  print 'OK'
  return set_scene


class TStateValidityChecker:
  def __init__(self):
    self.set_scene_client= SetupPlanningScene()
    self.planning_scene_req= arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()

    rospy.wait_for_service("planning_scene_validity_server/get_state_validity")
    self.check_state_validity_client= rospy.ServiceProxy("planning_scene_validity_server/get_state_validity", arm_navigation_msgs.srv.GetStateValidity)

    rospy.wait_for_service("planning_scene_validity_server/get_trajectory_validity")
    self.check_trajectory_validity_client= rospy.ServiceProxy("planning_scene_validity_server/get_trajectory_validity", arm_navigation_msgs.srv.GetJointTrajectoryValidity)

    #rospy.wait_for_service("environment_server/get_robot_state")
    #self.get_robot_state_client= rospy.ServiceProxy("environment_server/get_robot_state", arm_navigation_msgs.srv.GetRobotState)

    #PR2 arm joint names:
    self.joint_names= [[],[]]
    self.joint_names[RIGHT]= [
        'r_shoulder_pan_joint',
        'r_shoulder_lift_joint',
        'r_upper_arm_roll_joint',
        'r_elbow_flex_joint',
        'r_forearm_roll_joint',
        'r_wrist_flex_joint',
        'r_wrist_roll_joint']
    self.joint_names[LEFT]= [
        'l_shoulder_pan_joint',
        'l_shoulder_lift_joint',
        'l_upper_arm_roll_joint',
        'l_elbow_flex_joint',
        'l_forearm_roll_joint',
        'l_wrist_flex_joint',
        'l_wrist_roll_joint']
    #PR2 all link names:
    #obtained from res.planning_scene.allowed_collision_matrix.link_names
    self.links= {}
    self.links['base']= ['base_footprint', 'base_link', 'base_bellow_link', 'bl_caster_rotation_link', 'bl_caster_l_wheel_link', 'bl_caster_r_wheel_link', 'br_caster_rotation_link', 'br_caster_l_wheel_link', 'br_caster_r_wheel_link', 'fl_caster_rotation_link', 'fl_caster_l_wheel_link', 'fl_caster_r_wheel_link', 'fr_caster_rotation_link', 'fr_caster_l_wheel_link', 'fr_caster_r_wheel_link']
    self.links['torso']= ['torso_lift_link']
    self.links['head']= ['head_pan_link', 'head_tilt_link', 'head_plate_frame', 'head_mount_link', 'head_mount_kinect_ir_link', 'head_mount_kinect_rgb_link', 'head_mount_prosilica_link', 'sensor_mount_link', 'double_stereo_link']
    self.links['l_arm']= ['l_shoulder_pan_link', 'l_shoulder_lift_link', 'l_upper_arm_roll_link', 'l_upper_arm_link', 'l_elbow_flex_link', 'l_forearm_roll_link', 'l_forearm_link', 'l_wrist_flex_link', 'l_wrist_roll_link']
    self.links['l_gripper']= ['l_gripper_palm_link', 'l_gripper_l_finger_link', 'l_gripper_l_finger_tip_link', 'l_gripper_motor_accelerometer_link', 'l_gripper_r_finger_link', 'l_gripper_r_finger_tip_link']
    self.links['laser_mount']= ['laser_tilt_mount_link']
    self.links['r_arm']= ['r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link', 'r_upper_arm_link', 'r_elbow_flex_link', 'r_forearm_roll_link', 'r_forearm_link', 'r_wrist_flex_link', 'r_wrist_roll_link']
    self.links['r_gripper']= ['r_gripper_palm_link', 'r_gripper_l_finger_link', 'r_gripper_l_finger_tip_link', 'r_gripper_motor_accelerometer_link', 'r_gripper_r_finger_link', 'r_gripper_r_finger_tip_link']
    self.links['robot']= self.links['base'] + self.links['torso'] + self.links['head'] + self.links['l_arm'] + self.links['l_gripper'] + self.links['laser_mount'] + self.links['r_arm'] + self.links['r_gripper']
    #Gripper padding (originally they are 0.02)
    self.padding_links= self.links['l_gripper'] + self.links['r_gripper']
    self.padding_values= [0.002]*len(self.padding_links)

  #Prepare for making a scene.
  def InitToMakeScene(self):
    self.planning_scene_req= arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()

  #Send the planning_scene_diff requiest to server.
  #NOTE: After executing self.Add* and IgnoreCollision, this method must be called.
  def SendToServer(self):
    for i in range(len(self.padding_links)):
      padding= arm_navigation_msgs.msg.LinkPadding()
      padding.link_name= self.padding_links[i]
      padding.padding= self.padding_values[i]
      self.planning_scene_req.planning_scene_diff.link_padding.append(padding)
    #Disable to return (it will be faster?)
    self.set_scene_client(self.planning_scene_req)

  #Add an object to the planning scene at pose x
  def AddToScene(self, x,
      shape_type=arm_navigation_msgs.msg.Shape.BOX,
      shape_dims=[],
      padding=0.002,
      name='scene_obj1', frame_id='torso_lift_link'):
    scene_obj= arm_navigation_msgs.msg.CollisionObject()
    scene_obj.id= name
    scene_obj.operation.operation= arm_navigation_msgs.msg.CollisionObjectOperation.ADD
    scene_obj.header.frame_id= frame_id
    scene_obj.header.stamp= rospy.Time.now()

    shape= arm_navigation_msgs.msg.Shape()
    shape.type= shape_type
    shape.dimensions= shape_dims

    pose= XToGPose(x)
    scene_obj.shapes= [shape]
    scene_obj.poses= [pose]
    scene_obj.padding= padding

    self.planning_scene_req.planning_scene_diff.collision_objects.append(scene_obj)

  #Add a box to the planning scene at pose x
  def AddBoxToScene(self, x,
      shape_dims=[],
      padding=0.002,
      name='scene_obj1', frame_id='torso_lift_link'):
    self.AddToScene(x=x,
      shape_type=arm_navigation_msgs.msg.Shape.BOX,
      shape_dims=shape_dims, padding=padding, name=name, frame_id=frame_id)

  #Attach an object to a robot hand at pose x
  def AddToRobotHand(self, x,
      shape_type=arm_navigation_msgs.msg.Shape.BOX,
      shape_dims=[],
      padding=0.002,
      name='attached_obj1', arm=RIGHT):
    attached_obj= arm_navigation_msgs.msg.AttachedCollisionObject()
    if arm==RIGHT:
      attached_obj.link_name= 'r_wrist_roll_link'
      #The end-effector CAN TOUCH the object
      attached_obj.touch_links= [
          'r_end_effector',
          'r_wrist_roll_link',
          'r_wrist_flex_link']
    elif arm==LEFT:
      attached_obj.link_name= 'l_wrist_roll_link'
      #The end-effector CAN TOUCH the object
      attached_obj.touch_links= [
          'l_end_effector',
          'l_wrist_roll_link',
          'l_wrist_flex_link']

    attached_obj.object.id= name
    attached_obj.object.operation.operation= arm_navigation_msgs.msg.CollisionObjectOperation.ADD
    attached_obj.object.header.frame_id= attached_obj.link_name
    attached_obj.object.header.stamp= rospy.Time.now()

    shape= arm_navigation_msgs.msg.Shape()
    shape.type= shape_type
    shape.dimensions= shape_dims

    pose= XToGPose(x)
    attached_obj.object.shapes= [shape]
    attached_obj.object.poses= [pose]
    attached_obj.object.padding= padding

    self.planning_scene_req.planning_scene_diff.attached_collision_objects.append(attached_obj)

  #Attach a box to a robot hand at pose x
  def AddBoxToRobotHand(self, x,
      shape_dims=[],
      padding=0.002,
      name='attached_obj1', arm=RIGHT):
    self.AddToRobotHand(x=x,
      shape_type=arm_navigation_msgs.msg.Shape.BOX,
      shape_dims=shape_dims, padding=padding, name=name, arm=arm)

  #Ignore collision between name1 and name2, which can be a name of object or:
  #  '.robot': all links of the robot
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_ALL='all'
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_OBJECTS='objects'
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS='attached'
  def IgnoreCollision(self,name1,name2):
    assert(name1!=name2)
    #Expand names
    set_link_names= ('base', 'torso', 'head', 'l_arm', 'l_gripper', 'laser_mount', 'r_arm', 'r_gripper', 'robot')
    if name1[0]=='.' and name1[1:] in set_link_names:
      links1= self.links[name1[1:]]
    else:
      links1= [name1]
    if name2[0]=='.' and name2[1:] in set_link_names:
      links2= self.links[name2[1:]]
    else:
      links2= [name2]
    for link1 in links1:
      for link2 in links2:
        col_obj= arm_navigation_msgs.msg.CollisionOperation()
        col_obj.object1= link1
        col_obj.object2= link2
        col_obj.operation= arm_navigation_msgs.msg.CollisionOperation.DISABLE
        self.planning_scene_req.operations.collision_operations.append(col_obj)

  #Remove added/attached objects from the scene and the robot
  def RemoveFromScene(self):
    self.planning_scene_req= arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    return self.SendToServer()


  #Check the validity of a state joint_positions
  #which is an array of joint positions [float]
  def IsValidState(self, joint_positions, arm=RIGHT):
    req= arm_navigation_msgs.srv.GetStateValidityRequest()
    req.robot_state.joint_state.name= self.joint_names[arm]
    req.robot_state.joint_state.position= joint_positions
    #req.robot_state= self.get_robot_state_client(arm_navigation_msgs.srv.GetRobotStateRequest()).robot_state
    #req.robot_state.joint_state.position= Vec(req.robot_state.joint_state.position)
    #indexes= [req.robot_state.joint_state.name.index(name) for name in self.joint_names[arm]]
    #req.robot_state.joint_state.position[indexes]= joint_positions

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions= True
    res= self.check_state_validity_client(req)
    #return res.error_code.val == res.error_code.SUCCESS
    return res

  #Check the validity of a trajectory joint_traj
  #which is a trajectory_msgs/JointTrajectory
  def IsValidTrajectory(self, joint_traj):
    #'''#using check_trajectory_validity_client:
    req= arm_navigation_msgs.srv.GetJointTrajectoryValidityRequest()
    #req.robot_state.joint_state.name= joint_traj.joint_names
    #if len(joint_traj.points)>0:
      #req.robot_state.joint_state.position= joint_traj.points[0].positions
    req.trajectory= joint_traj

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions= True
    res= self.check_trajectory_validity_client(req)
    #return res.error_code.val == res.error_code.SUCCESS
    return res
    #'''
    '''#using check_state_validity_client:
    req= arm_navigation_msgs.srv.GetStateValidityRequest()
    req.robot_state.joint_state.name= joint_traj.joint_names
    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions= True

    res= None
    for point in joint_traj.points:
      req.robot_state.joint_state.position= point.positions
      res= self.check_state_validity_client(req)
      if res.error_code.val<>res.error_code.SUCCESS:
        return res
    return res
    #'''



#Extract a list of colliding object pairs [set([o1,o2]),..] from contacts
#  where contacts: arm_navigation_msgs/ContactInformation[]
#ignored: if an object is included in this list, that collision is ignored.
def GetCollidingObjects(contacts, ignored=[]):
  res= []
  for c in contacts:
    o1= c.contact_body_1
    o2= c.contact_body_2
    if len(ignored)>0 and (o1 in ignored or o2 in ignored):
      continue
    pair= set([o1,o2])
    res.append(pair)
  return res

