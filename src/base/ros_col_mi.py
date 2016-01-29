#! /usr/bin/env python
#ROS tools (collision detection with MoveIt!).
import roslib
roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('geometry_msgs')
import geometry_msgs.msg
import trajectory_msgs.msg
#For collision check
roslib.load_manifest('moveit_msgs')
import moveit_msgs.srv
import moveit_msgs.msg
import shape_msgs.msg
import lfd_trick.srv

from base_geom import *
from base_const import *
from ros_base import *


##There must be a planning scene or FK / IK crashes
#def SetupPlanningScene():
  #print 'Waiting for /environment_server/set_planning_scene_diff...'
  #rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  #set_scene= rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
  #req= arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
  #set_scene(req)
  #print 'OK'
  #return set_scene

#Dummy class for old version.
class TStateValidityChecker:
  pass

class TStateValidityCheckerMI(TROSUtil):
  class TConfig:
    def __init__(self):
      self.JointNames= [[],[]]
      self.Links= {}
      self.PaddingLinks= []
      self.PaddingValues= []
      self.DefaultBaseFrame= None
      self.HandLinkToGrasp= [[],[]]
      self.IgnoredLinksInGrasp= [[],[]]

  def __init__(self):
    TROSUtil.__init__(self)
    self.c= self.TConfig()
    self.planning_scene_req= lfd_trick.srv.SetPlanningSceneDiffRequest()

  #Initialize.  robot should have ConfigureSVC function to setup self.c.
  def Init(self, robot):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    robot.ConfigureSVC(self.c)

    ra(self.AddSrvP('set_scene_client', '/state_validity_checker/set_planning_scene_diff',
                    lfd_trick.srv.SetPlanningSceneDiff, persistent=False, time_out=3.0))

    ra(self.AddSrvP('check_state_validity_client', '/state_validity_checker/get_state_validity',
                    lfd_trick.srv.GetStateValidity, persistent=False, time_out=3.0))
    ra(self.AddSrvP('check_trajectory_validity_client', '/state_validity_checker/get_trajectory_validity',
                    lfd_trick.srv.GetTrajectoryValidity, persistent=False, time_out=3.0))
    ra(self.AddSrvP('get_allowed_collision_matrix_client', '/state_validity_checker/get_allowed_collision_matrix',
                    lfd_trick.srv.GetAllowedCollisionMatrix, persistent=False, time_out=3.0))

    #rospy.wait_for_service("environment_server/get_robot_state")
    #self.get_robot_state_client= rospy.ServiceProxy("environment_server/get_robot_state", arm_navigation_msgs.srv.GetRobotState)

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  #Prepare for making a scene.
  def InitToMakeScene(self):
    self.planning_scene_req= lfd_trick.srv.SetPlanningSceneDiffRequest()
    #We need to get the current allowed_collision_matrix for self.IgnoreCollision().
    #  Otherwise, it seems that an existing collision ignoring matrix
    #  (e.g. collision btwn connected links) is overwritten.
    #  So we need to start from an existing matrix.
    self.planning_scene_req.planning_scene_diff.allowed_collision_matrix= self.srvp.get_allowed_collision_matrix_client().allowed_collision_matrix

  #Send the planning_scene_diff requiest to server.
  #NOTE: After executing self.Add* and IgnoreCollision, this method must be called.
  def SendToServer(self):
    for i in range(len(self.c.PaddingLinks)):
      padding= moveit_msgs.msg.LinkPadding()
      padding.link_name= self.c.PaddingLinks[i]
      padding.padding= self.c.PaddingValues[i]
      self.planning_scene_req.planning_scene_diff.link_padding.append(padding)
    #self.planning_scene_req.planning_scene_diff.robot_model_root= self.c.DefaultBaseFrame
    #Disable to return (it will be faster?)
    self.srvp.set_scene_client(self.planning_scene_req)
    '''DEBUG'''
    #print '------------------------'
    #self.planning_scene_req.planning_scene_diff.robot_state.is_diff= True
    #print self.planning_scene_req
    #res= self.srvp.set_scene_client(self.planning_scene_req)
    #print '------------------------'
    #print res
    #print '------------------------'

  #Add an object to the planning scene at pose x
  def AddToScene(self, x,
      shape_type=shape_msgs.msg.SolidPrimitive.BOX,
      shape_dims=[],
      padding=0.002,
      name='scene_obj1', frame_id=None):
    scene_obj= moveit_msgs.msg.CollisionObject()
    scene_obj.id= name
    scene_obj.operation= moveit_msgs.msg.CollisionObject.ADD
    scene_obj.header.frame_id= self.c.DefaultBaseFrame if frame_id is None else frame_id
    scene_obj.header.stamp= rospy.Time.now()

    primitive= shape_msgs.msg.SolidPrimitive()
    primitive.type= shape_type
    primitive.dimensions= shape_dims

    pose= XToGPose(x)
    scene_obj.primitives= [primitive]
    scene_obj.primitive_poses= [pose]
    #scene_obj.padding= padding

    self.planning_scene_req.planning_scene_diff.world.collision_objects.append(scene_obj)

  #Add a box to the planning scene at pose x.
  #shape_dims= [lx,ly,lz]
  def AddBoxToScene(self, x,
      shape_dims=[],
      padding=0.002,
      name='scene_obj1', frame_id=None):
    self.AddToScene(x=x,
      shape_type=shape_msgs.msg.SolidPrimitive.BOX,
      shape_dims=shape_dims, padding=padding, name=name, frame_id=frame_id)

  #Add a cylinder (aligned along z) to the planning scene at pose x.
  #shape_dims= [height,radius]
  def AddCylinderToScene(self, x,
      shape_dims=[],
      padding=0.002,
      name='scene_obj1', frame_id=None):
    self.AddToScene(x=x,
      shape_type=shape_msgs.msg.SolidPrimitive.CYLINDER,
      shape_dims=shape_dims, padding=padding, name=name, frame_id=frame_id)

  #Attach an object to a robot hand at pose x
  def AddToRobotHand(self, x,
      shape_type=shape_msgs.msg.SolidPrimitive.BOX,
      shape_dims=[],
      padding=0.002,
      name='attached_obj1', arm=RIGHT):
    attached_obj= moveit_msgs.msg.AttachedCollisionObject()
    attached_obj.link_name= self.c.HandLinkToGrasp[arm]
    #The end-effector CAN TOUCH the object:
    attached_obj.touch_links= self.c.IgnoredLinksInGrasp[arm]

    attached_obj.object.id= name
    attached_obj.object.operation= moveit_msgs.msg.CollisionObject.ADD
    attached_obj.object.header.frame_id= attached_obj.link_name
    attached_obj.object.header.stamp= rospy.Time.now()

    primitive= shape_msgs.msg.SolidPrimitive()
    primitive.type= shape_type
    primitive.dimensions= shape_dims

    pose= XToGPose(x)
    attached_obj.object.primitives= [primitive]
    attached_obj.object.primitive_poses= [pose]
    #attached_obj.object.padding= padding

    self.planning_scene_req.planning_scene_diff.robot_state.attached_collision_objects.append(attached_obj)

  #Attach a box to a robot hand at pose x.
  #shape_dims= [lx,ly,lz]
  def AddBoxToRobotHand(self, x,
      shape_dims=[],
      padding=0.002,
      name='attached_obj1', arm=RIGHT):
    self.AddToRobotHand(x=x,
      shape_type=shape_msgs.msg.SolidPrimitive.BOX,
      shape_dims=shape_dims, padding=padding, name=name, arm=arm)

  #Attach a cylinder (aligned along z) to a robot hand at pose x.
  #shape_dims= [height,radius]
  def AddCylinderToRobotHand(self, x,
      shape_dims=[],
      padding=0.002,
      name='attached_obj1', arm=RIGHT):
    self.AddToRobotHand(x=x,
      shape_type=shape_msgs.msg.SolidPrimitive.CYLINDER,
      shape_dims=shape_dims, padding=padding, name=name, arm=arm)

  #Ignore collision between name1 and name2, which can be a name of object or:
  #  '.robot': all links of the robot
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_ALL='all'
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_OBJECTS='objects'
  #  arm_navigation_msgs.msg.CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS='attached'
  def IgnoreCollision(self,name1,name2):
    assert(name1!=name2)
    #Expand names
    if name1[0]=='.' and name1[1:] in self.c.Links:
      links1= self.c.Links[name1[1:]]
    else:
      links1= [name1]
    if name2[0]=='.' and name2[1:] in self.c.Links:
      links2= self.c.Links[name2[1:]]
    else:
      links2= [name2]
    acm= self.planning_scene_req.planning_scene_diff.allowed_collision_matrix
    for link1 in links1:
      for link2 in links2:
        link12= (link1,link2)
        idx12= [-1,-1]
        for link,i in zip(link12,(0,1)):
          if link in acm.entry_names:
            idx12[i]= acm.entry_names.index(link)
          else:
            idx12[i]= len(acm.entry_names)
            acm.entry_names.append(link)
            for e in acm.entry_values:  e.enabled.append(False)
            acm.entry_values.append(moveit_msgs.msg.AllowedCollisionEntry())
            acm.entry_values[-1].enabled= [False]*len(acm.entry_names)
        acm.entry_values[idx12[0]].enabled[idx12[1]]= True
        acm.entry_values[idx12[1]].enabled[idx12[0]]= True

  #Remove added/attached objects from the scene and the robot
  def RemoveFromScene(self):
    self.planning_scene_req= lfd_trick.srv.SetPlanningSceneDiffRequest()
    return self.SendToServer()


  #Check the validity of a state joint_positions
  #which is an array of joint positions [float]
  def IsValidState(self, joint_positions, arm=RIGHT):
    req= lfd_trick.srv.GetStateValidityRequest()
    req.contacts_frame_id= self.c.DefaultBaseFrame
    req.robot_state.joint_state.name= self.c.JointNames[arm]
    req.robot_state.joint_state.position= joint_positions
    if 'is_diff' in dir(req.robot_state):
      req.robot_state.is_diff= True  #NOTE: this is necessary due to MoveIt's bug.
    #See more information: https://groups.google.com/forum/#!topic/moveit-users/34cvq4mtCoE
    #req.robot_state= self.get_robot_state_client(arm_navigation_msgs.srv.GetRobotStateRequest()).robot_state
    #req.robot_state.joint_state.position= Vec(req.robot_state.joint_state.position)
    #indexes= [req.robot_state.joint_state.name.index(name) for name in self.c.JointNames[arm]]
    #req.robot_state.joint_state.position[indexes]= joint_positions

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    res= self.srvp.check_state_validity_client(req)
    #return res.error_code.val == res.error_code.SUCCESS
    return res

  #Check the validity of a trajectory joint_traj
  #which is a trajectory_msgs/JointTrajectory
  def IsValidTrajectory(self, joint_traj):
    #'''#using srvp.check_trajectory_validity_client:
    req= lfd_trick.srv.GetTrajectoryValidityRequest()
    if 'is_diff' in dir(req.start_state):
      req.start_state.is_diff= True  #NOTE: this is necessary due to MoveIt's bug.
    req.trajectory.joint_trajectory= joint_traj
    res= self.srvp.check_trajectory_validity_client(req)
    return res
    #'''
    '''#using srvp.check_state_validity_client:
    req= lfd_trick.srv.GetStateValidityRequest()
    req.contacts_frame_id= self.c.DefaultBaseFrame
    req.robot_state.joint_state.name= joint_traj.joint_names
    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    if 'is_diff' in dir(req.robot_state):
      req.robot_state.is_diff= True  #NOTE: this is necessary due to MoveIt's bug.

    res= None
    for point in joint_traj.points:
      req.robot_state.joint_state.position= point.positions
      res= self.srvp.check_state_validity_client(req)
      if not res.valid:
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

