#! /usr/bin/env python
#Robot controller for PR2.
from base_const import *
if ROS_ROBOT not in ('PR2','PR2_SIM'):
  raise ImportError('Stop importing: ROS_ROBOT is not PR2')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

import roslib
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
import kinematics_msgs.srv  #WARNING: may not work with ROS_DISTRO>='hydro'
import trajectory_msgs.msg
import pr2_controllers_msgs.msg
import pr2_mechanism_msgs.srv

from base_geom import *
from base_traj import *
from ros_robot import *
from ros_base import *

'''Robot control class for PR2.'''
class TRobotPR2(TDualArmRobot):
  def __init__(self):
    TDualArmRobot.__init__(self, name='PR2')

    self.curr_pos= [[0.0]*self.DoF()]*2
    self.curr_vel= [[0.0]*self.DoF()]*2

    self.ik_timeout= rospy.Duration(5.0)

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

    self.head_joint_names= ['head_pan_joint', 'head_tilt_joint']

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

    #Mannequin controller:
    self.mann_ctrl= TPR2Mannequin()


  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddSub('r_arm_state', '/r_arm_controller/state',
                   pr2_controllers_msgs.msg.JointTrajectoryControllerState,
                   lambda msg:self.ArmStateCallback(msg,RIGHT)))
    ra(self.AddSub('l_arm_state', '/l_arm_controller/state',
                   pr2_controllers_msgs.msg.JointTrajectoryControllerState,
                   lambda msg:self.ArmStateCallback(msg,LEFT)))

    ra(self.AddPub('head_ctrl', '/head_traj_controller/command',
                   trajectory_msgs.msg.JointTrajectory))

    ra(self.AddSrvP('r_get_fk', '/pr2_right_arm_kinematics/get_fk',
                    kinematics_msgs.srv.GetPositionFK, persistent=False, time_out=3.0))
    ra(self.AddSrvP('r_get_ik', '/pr2_right_arm_kinematics/get_ik',
                    kinematics_msgs.srv.GetPositionIK, persistent=False, time_out=3.0))
    ra(self.AddSrvP('l_get_fk', '/pr2_left_arm_kinematics/get_fk',
                    kinematics_msgs.srv.GetPositionFK, persistent=False, time_out=3.0))
    ra(self.AddSrvP('l_get_ik', '/pr2_left_arm_kinematics/get_ik',
                    kinematics_msgs.srv.GetPositionIK, persistent=False, time_out=3.0))

    ra(self.AddActC('r_traj', '/r_arm_controller/joint_trajectory_action',
                    pr2_controllers_msgs.msg.JointTrajectoryAction, time_out=3.0))
    ra(self.AddActC('l_traj', '/l_arm_controller/joint_trajectory_action',
                    pr2_controllers_msgs.msg.JointTrajectoryAction, time_out=3.0))

    ra(self.AddActC('r_grip', '/r_gripper_controller/gripper_action',
                    pr2_controllers_msgs.msg.Pr2GripperCommandAction, time_out=3.0))
    ra(self.AddActC('l_grip', '/l_gripper_controller/gripper_action',
                    pr2_controllers_msgs.msg.Pr2GripperCommandAction, time_out=3.0))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    TDualArmRobot.Cleanup(self)

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    #Gripper padding (originally they are 0.02)
    c.PaddingLinks= c.Links['l_gripper'] + c.Links['r_gripper']
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'torso_lift_link'
    c.HandLinkToGrasp[RIGHT]= 'r_wrist_roll_link'
    c.HandLinkToGrasp[LEFT]= 'l_wrist_roll_link'
    c.IgnoredLinksInGrasp[RIGHT]= [
          'r_end_effector',
          'r_wrist_roll_link',
          'r_wrist_flex_link'] + c.Links['r_gripper']
    c.IgnoredLinksInGrasp[LEFT]= [
          'l_end_effector',
          'l_wrist_roll_link',
          'l_wrist_flex_link'] + c.Links['l_gripper']

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('PR2','PR2_SIM'):  return True
    return TDualArmRobot.Is(self,q)

  '''Mannequin controller.'''
  @property
  def Mannequin(self):
    return self.mann_ctrl

  @property
  def BaseFrame(self):
    return 'torso_lift_link'

  '''End link of an arm.'''
  def EndLink(self, arm):
    if   arm==RIGHT:  return 'r_wrist_roll_link'
    elif arm==LEFT:   return 'l_wrist_roll_link'

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
    return self.joint_names[arm]

  '''Return limits of joint angular velocity.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    return [0.8, 0.8, 2.0, 2.0, 3.0, 3.0, 10.0]


  def ArmStateCallback(self, msg, arm):
    with self.sensor_locker:
      self.curr_pos[arm]= msg.actual.positions
      self.curr_vel[arm]= msg.actual.velocities


  '''Return joint angles of an arm.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def Q(self, arm=None):
    if arm==None:  arm= self.Arm
    with self.sensor_locker:
      q= copy.deepcopy(self.curr_pos[arm])
    return q

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (None if failure), res: FK status.
    arm: LEFT, RIGHT, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    if arm==None:  arm= self.Arm
    if q==None:  q= self.Q(arm)

    fk_req= kinematics_msgs.srv.GetPositionFKRequest()
    fk_req.header.frame_id= self.BaseFrame
    fk_req.fk_link_names= [self.EndLink(arm)]
    fk_req.robot_state.joint_state.name= self.JointNames(arm)
    fk_req.robot_state.joint_state.position= q
    try:
      res= None
      with self.sensor_locker:
        if   arm==RIGHT:  res= self.srvp.r_get_fk(fk_req)
        elif arm==LEFT:   res= self.srvp.l_get_fk(fk_req)
    except rospy.ServiceException as e:
      CPrint(4,'FK failed:',str(e))
      CPrint(4,'Request:',fk_req)
      CPrint(4,'#FK failed:',str(e))
      raise e  #Forward the exception

    x= GPoseToX(res.pose_stamped[0].pose)
    x_res= x if x_ext==None else Transform(x,x_ext)
    if res.error_code.val==1:  return (x_res, res) if with_st else x_res
    else:  return (None, res) if with_st else None

  '''Compute an inverse kinematics of an arm.
  Return joint angles for a target self.EndLink(arm) pose on self.BaseFrame.
    return: q, res;  q: joint angles (None if failure), res: IK status.
    arm: LEFT, RIGHT, or None (==currarm).
    x_trg: target pose.
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned q satisfies self.FK(q,x_ext,arm)==x_trg.
    start_angles: initial joint angles for IK solver, or None (==self.Q(arm)).
    with_st: whether return IK status. '''
  def IK(self, x_trg, x_ext=None, start_angles=None, arm=None, with_st=False):
    if arm==None:  arm= self.Arm
    if start_angles==None:  start_angles= self.Q(arm)

    x_trg[3:]/= la.norm(x_trg[3:])  #Normalize the orientation:
    xw_trg= x_trg if x_ext==None else TransformRightInv(x_trg,x_ext)

    ik_req= kinematics_msgs.srv.GetPositionIKRequest()
    ik_req.ik_request.ik_seed_state.joint_state.name= self.JointNames(arm)
    ik_req.timeout= self.ik_timeout
    ik_req.ik_request.ik_link_name= self.EndLink(arm)
    ik_req.ik_request.pose_stamped.header.frame_id= self.BaseFrame
    ik_req.ik_request.pose_stamped.pose= XToGPose(xw_trg)
    ik_req.ik_request.ik_seed_state.joint_state.position= start_angles
    try:
      res= None
      with self.sensor_locker:
        if   arm==RIGHT:  res= self.srvp.r_get_ik(ik_req)
        elif arm==LEFT:   res= self.srvp.l_get_ik(ik_req)
    except rospy.ServiceException as e:
      CPrint(4,'IK failed:',str(e))
      CPrint(4,'Request:',ik_req)
      CPrint(4,'#IK failed:',str(e))
      raise e  #Forward the exception

    q= res.solution.joint_state.position
    if res.error_code.val==1:  return (q, res) if with_st else q
    else:  return (None, res) if with_st else None


  '''Follow a joint angle trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    assert(len(q_traj)==len(t_traj))
    if arm==None:  arm= self.Arm

    #copy q_traj, t_traj to goal
    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj)

    with self.control_locker:
      actc= self.actc.r_traj if arm==RIGHT else self.actc.l_traj
      actc.send_goal(goal)
      BlockAction(actc, blocking=blocking, duration=t_traj[-1])


  '''Open a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def OpenGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.09, max_effort=50.0, arm=arm, blocking=blocking)

  '''Close a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def CloseGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.00, max_effort=50.0, arm=arm, blocking=blocking)

  '''Control a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    pos: target position; 0.09 (open), 0.0 (close).
    max_effort: maximum effort to control; 12~15 (weak), 50 (strong), -1 (maximum).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def MoveGripper(self, pos, max_effort, arm=None, blocking=False):
    if arm==None:  arm= self.Arm

    goal= pr2_controllers_msgs.msg.Pr2GripperCommandGoal()
    goal.command.position= pos
    goal.command.max_effort= max_effort

    with self.control_locker:
      actc= self.actc.r_grip if arm==RIGHT else self.actc.l_grip
      actc.send_goal(goal)
      BlockAction(actc, blocking=blocking, duration=0.0)

  '''Control the head.
    pan, tilt: target angles.
    dt: duration to reach the target.  '''
  def MoveHead(self, pan, tilt, dt=4.0):
    jp= trajectory_msgs.msg.JointTrajectoryPoint()
    jp.positions= [pan,tilt]
    jp.velocities= [0.0]*2
    jp.time_from_start= rospy.Duration(dt)

    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= self.head_joint_names
    traj.points.append(jp)
    traj.header.stamp= rospy.Time.now()
    self.pub.head_ctrl.publish(traj)



'''Mannequin controller for PR2.
    Based on pr2_lfd_utils/src/recordInteraction.py implemented by Scott Niekum. '''
class TPR2Mannequin(TROSUtil):
  def __init__(self):
    TROSUtil.__init__(self)
    joint_thresh= 0.001
    self.joint_bounds= [joint_thresh]*10
    self.standard_controllers= ['r_arm_controller', 'l_arm_controller']
    self.mannequin_controllers= ['r_arm_controller_loose', 'l_arm_controller_loose']
    self.is_mannequin= [False,False]

  def __del__(self):
    TROSUtil.__del__(self)

  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub('r_mann_ctrl', '/r_arm_controller_loose/command',
                   trajectory_msgs.msg.JointTrajectory))
    ra(self.AddPub('l_mann_ctrl', '/l_arm_controller_loose/command',
                   trajectory_msgs.msg.JointTrajectory))

    ra(self.AddSrvP('switch_control', '/pr2_controller_manager/switch_controller',
                    pr2_mechanism_msgs.srv.SwitchController, persistent=False, time_out=3.0))

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Activate(self, arm):
    if self.is_mannequin[arm]:
      print '%s arm is already mannequin mode.' % LRToStr(arm)
      return

    if not self.Init():  return
    if arm==RIGHT and 'r_mann_state' not in self.sub:
      self.sub.r_mann_state= rospy.Subscriber(
        '/r_arm_controller_loose/state',
        pr2_controllers_msgs.msg.JointTrajectoryControllerState,
        lambda msg:self.JointStateCallback(msg,arm=RIGHT))
    if arm==LEFT and 'l_mann_state' not in self.sub:
      self.sub.l_mann_state= rospy.Subscriber(
        '/l_arm_controller_loose/state',
        pr2_controllers_msgs.msg.JointTrajectoryControllerState,
        lambda msg:self.JointStateCallback(msg,arm=LEFT))

    switch_req= pr2_mechanism_msgs.srv.SwitchControllerRequest()
    switch_req.strictness= pr2_mechanism_msgs.srv.SwitchControllerRequest.BEST_EFFORT
    switch_req.stop_controllers= [self.standard_controllers[arm]]
    switch_req.start_controllers= [self.mannequin_controllers[arm]]
    res= self.srvp.switch_control(switch_req)
    self.is_mannequin[arm]= True
    print '%s arm is mannequin mode.' % LRToStr(arm)

  def Deactivate(self, arm):
    if not self.is_mannequin[arm]:
      print '%s arm is not mannequin mode.' % LRToStr(arm)
      return

    if arm==RIGHT and 'r_mann_state' in self.sub:
      self.sub.r_mann_state.unregister()
      del self.sub.r_mann_state
    if arm==LEFT and 'l_mann_state' in self.sub:
      self.sub.l_mann_state.unregister()
      del self.sub.l_mann_state

    if not self.Init():  return
    switch_req= pr2_mechanism_msgs.srv.SwitchControllerRequest()
    switch_req.strictness= pr2_mechanism_msgs.srv.SwitchControllerRequest.BEST_EFFORT
    switch_req.stop_controllers= [self.mannequin_controllers[arm]]
    switch_req.start_controllers= [self.standard_controllers[arm]]
    res= self.srvp.switch_control(switch_req)
    self.is_mannequin[arm]= False
    print '%s arm have finished mannequin mode.' % LRToStr(arm)

  def JointStateCallback(self, msg, arm):
    if self.is_mannequin[arm]:
      #max_error= max([abs(x) for x in msg.error.positions])
      exceeded= [abs(x) > y for x,y in zip(msg.error.positions, self.joint_bounds)]

      if any(exceeded):
        # Copy our current state into the commanded state
        cmd= trajectory_msgs.msg.JointTrajectory()
        cmd.header.stamp= msg.header.stamp
        cmd.joint_names= msg.joint_names
        cmd.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        cmd.points[0].time_from_start= rospy.Duration(0.125)
        cmd.points[0].positions= msg.actual.positions
        (self.pub.r_mann_ctrl if arm==RIGHT else self.pub.l_mann_ctrl).publish(cmd)

