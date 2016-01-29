#! /usr/bin/env python
#Robot controller for Baxter.
from base_const import *
if ROS_ROBOT not in ('Baxter','Baxter_SIM'):
  raise ImportError('Stop importing: ROS_ROBOT is not Baxter')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

import roslib
import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import baxter_interface
from baxter_pykdl import baxter_kinematics
import time, math, sys, copy
import cv2
import cv_bridge

from base_geom import *
from base_traj import *
from ros_robot import *
from ros_base import *

'''Robot control class for Baxter.'''
class TRobotBaxter(TDualArmRobot):
  def __init__(self):
    TDualArmRobot.__init__(self, name='Baxter')
    self.is_sim= (ROS_ROBOT=='Baxter_SIM')

    #Gripper command-position conversions.
    #epg: electric parallel gripper (narrow, pos#4).
    #self.epg_cmd2pos= lambda cmd: 0.00042*cmd+0.05317    #effective cmd in [0,100] ([0,100])
    #self.epg_pos2cmd= lambda pos: (pos-0.05317)/0.00042  #pos in [0.054,0.094] meter
    #epg: electric parallel gripper (narrow, pos#1).
    self.epg_cmd2pos= lambda cmd: 0.00037*cmd            #effective cmd in [0,100] ([0,100])
    self.epg_pos2cmd= lambda pos: (pos)/0.00037          #pos in [0.00,0.037] meter
    #rqg: Robotiq gripper.
    self.rqg_cmd2pos= lambda cmd: -0.00041*cmd+0.09249   #effective cmd in [12,230] ([0,255])
    self.rqg_pos2cmd= lambda pos: -(pos-0.09249)/0.00041 #pos in [0.0,0.0855] meter

    self.joint_names= [[],[]]
    self.joint_names[RIGHT]= ['right_'+joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    self.joint_names[LEFT]=  ['left_' +joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    #Baxter all link names:
    #obtained from /gazebo/link_states
    #obtained from self.planning_scene_req.planning_scene_diff.allowed_collision_matrix.entry_names
    self.links= {}
    self.links['base']= ['base', 'pedestal', 'torso']
    self.links['head']= ['collision_head_link_1', 'collision_head_link_2', 'display', 'head', 'screen', 'sonar_ring']
    self.links['l_arm']= ['left_upper_shoulder', 'left_lower_shoulder', 'left_upper_elbow', 'left_lower_elbow', 'left_upper_forearm', 'left_lower_forearm', 'left_wrist', 'left_upper_elbow_visual', 'left_upper_forearm_visual']
    self.links['l_gripper']= ['left_gripper', 'left_gripper_base', 'left_hand', 'left_hand_accelerometer', 'left_hand_camera', 'left_hand_range']
    self.links['r_arm']= ['right_upper_shoulder', 'right_lower_shoulder', 'right_upper_elbow', 'right_lower_elbow', 'right_upper_forearm', 'right_lower_forearm', 'right_wrist', 'right_upper_elbow_visual', 'right_upper_forearm_visual']
    self.links['r_gripper']= ['right_gripper', 'right_gripper_base', 'right_hand', 'right_hand_accelerometer', 'right_hand_camera', 'right_hand_range']
    self.links['robot']= self.links['base'] + self.links['head'] + self.links['l_arm'] + self.links['l_gripper'] + self.links['r_arm'] + self.links['r_gripper']

    #Robotiq controller:
    self.robotiq= TRobotiq()

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.default_face= 'config/baymax.jpg'
    self.ChangeFace(self.default_face)

    self.limbs= [None,None]
    self.limbs[RIGHT]= baxter_interface.Limb(LRTostr(RIGHT))
    self.limbs[LEFT]=  baxter_interface.Limb(LRTostr(LEFT))

    #self.joint_names= [[],[]]
    #self.joint_names[RIGHT]= self.limbs[RIGHT].joint_names()
    #self.joint_names[LEFT]=  self.limbs[LEFT].joint_names()

    self.kin= [None,None]
    self.kin[RIGHT]= baxter_kinematics(LRTostr(RIGHT))
    self.kin[LEFT]=  baxter_kinematics(LRTostr(LEFT))  #tip_link=_gripper(default),_wrist,_hand

    self.head= baxter_interface.Head()

    ra(self.AddActC('r_traj', '/robot/limb/right/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))
    ra(self.AddActC('l_traj', '/robot/limb/left/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    self.epgripper= baxter_interface.Gripper('right', baxter_interface.CHECK_VERSION)
    self.grippers= [self.epgripper, self.robotiq]

    print 'Enabling the robot...'
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    print 'Calibrating electric parallel gripper...'
    ra(self.epgripper.calibrate())

    print 'Initializing and activating Robotiq gripper...'
    ra(self.robotiq.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    TDualArmRobot.Cleanup(self)

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= c.Links['l_gripper'] + c.Links['r_gripper']
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'torso'
    c.HandLinkToGrasp[RIGHT]= 'right_gripper'
    c.HandLinkToGrasp[LEFT]= 'left_gripper'
    c.IgnoredLinksInGrasp[RIGHT]= ['right_gripper'] + c.Links['r_gripper']
    c.IgnoredLinksInGrasp[LEFT]= ['left_gripper'] + c.Links['l_gripper']

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Baxter','Baxter_SIM'):  return True
    return TDualArmRobot.Is(self,q)

  @property
  def BaseFrame(self):
    return 'torso'

  '''End link of an arm.'''
  def EndLink(self, arm):
    if   arm==RIGHT:  return 'right_gripper'
    elif arm==LEFT:   return 'left_gripper'

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
    return self.joint_names[arm]

  '''Return limits of joint angular velocity.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]


  '''Return joint angles of an arm.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def Q(self, arm=None):
    if arm==None:  arm= self.Arm
    with self.sensor_locker:
      angles= self.limbs[arm].joint_angles()
      q= [angles[joint] for joint in self.joint_names[arm]]  #Serialize
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

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      x= self.kin[arm].forward_position_kinematics(joint_values=angles)

    x_res= x if x_ext==None else Transform(x,x_ext)
    return (x_res, True) if with_st else x_res

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

    with self.sensor_locker:
      q= self.kin[arm].inverse_kinematics(xw_trg[:3], xw_trg[3:], seed=start_angles)

    if q is not None:  return (q, True) if with_st else q
    else:  return (None, False) if with_st else None


  '''Follow a joint angle trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    assert(len(q_traj)==len(t_traj))
    if arm==None:  arm= self.Arm

    #Insert current position to beginning.
    if t_traj[0]>1.0e-2:
      t_traj.insert(0,0.0)
      q_traj.insert(0,self.Q(arm=arm))

    #copy q_traj, t_traj to goal
    goal= control_msgs.msg.FollowJointTrajectoryGoal()
    goal.goal_time_tolerance= rospy.Time(0.1)
    goal.trajectory.joint_names= self.joint_names[arm]
    goal.trajectory= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj)

    with self.control_locker:
      actc= self.actc.r_traj if arm==RIGHT else self.actc.l_traj
      actc.send_goal(goal)
      BlockAction(actc, blocking=blocking, duration=t_traj[-1])
      #actc.wait_for_result(timeout=rospy.Duration(t_traj[-1]+5.0))  WARNING: Maybe it's better to set timeout


  '''Open a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.1, arm=arm, blocking=blocking)

  '''Close a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.0, arm=arm, blocking=blocking)

  '''High level interface to control a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if self.is_sim:  return  #WARNING:We do nothing if the robot is on simulator.
    if arm==None:  arm= self.Arm

    gripper= self.grippers[arm]
    if isinstance(gripper, baxter_interface.Gripper):
      clip= lambda c: max(0.0,min(100.0,c))
      cmd= clip(self.epg_pos2cmd(pos))
      with self.control_locker:
        gripper.set_velocity(clip(speed))
        gripper.set_moving_force(clip(max_effort))
        gripper.set_holding_force(clip(max_effort))
        gripper.command_position(cmd,block=blocking)

    elif isinstance(gripper, TRobotiq):
      clip= lambda c: max(0.0,min(255.0,c))
      cmd= clip(self.rqg_pos2cmd(pos))
      max_effort= clip(max_effort*(255.0/100.0))
      speed= clip(speed*(255.0/100.0))
      with self.control_locker:
        gripper.MoveGripper(cmd, max_effort, speed, blocking=blocking)

  '''Control the head around z.
    angle: target angle in radian.
    speed: movement speed; 0 (minimum), 100 (maximum).
    timeout: timeout in seconds. '''
  def MoveHeadPan(self, angle, speed=100, timeout=10):
    self.head.set_pan(angle, speed, timeout)

  '''Nod head (no controllable parameters). '''
  def NodHead(self):
    self.head.command_nod()

  '''Change the face image to file_name.'''
  def ChangeFace(self, file_name):
    img= cv2.imread(file_name)
    msg= cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub= rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True, queue_size=1)
    pub.publish(msg)


roslib.load_manifest('robotiq_c_model_control')
import robotiq_c_model_control.msg as robotiq_msgs

'''Robotiq Gripper utility class'''
class TRobotiq(TROSUtil):
  def __init__(self):
    TROSUtil.__init__(self)
    self.status= None
    self.SensorCallback= None

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self, cmd_topic='/rq1/command', st_topic='/rq1/status'):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub('grip', cmd_topic, robotiq_msgs.CModel_robot_output))
    ra(self.AddSub('grip', st_topic, robotiq_msgs.CModel_robot_input, self.SensorHandler))

    rospy.sleep(0.2)
    self.Activate()

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    self.Deactivate()
    TROSUtil.Cleanup(self)

  def SensorHandler(self,msg):
    self.status= msg
    if self.SensorCallback is not None:
      self.SensorCallback(self.status)

  @staticmethod
  def PrintStatus(st):
    print 'Flags(ACT,GTO,STA,OBJ,FLT):',st.gACT,st.gGTO,st.gSTA,st.gOBJ,st.gFLT,
    print 'State(PR,PO,CU):',st.gPR,st.gPO,st.gCU

  def Activate(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 1
    cmd.rSP= 255  #SPeed
    cmd.rFR= 150  #FoRce
    self.pub.grip.publish(cmd)

  def Deactivate(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 0
    self.pub.grip.publish(cmd)

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def OpenGripper(self, blocking=False):
    self.MoveGripper(pos=0, max_effort=100, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def CloseGripper(self, blocking=False):
    self.MoveGripper(pos=255, max_effort=100, blocking=blocking)

  '''Control a gripper.
    pos: target position; 0 (open), 255 (close).
    max_effort: maximum effort to control; 0~50 (weak), 200 (strong), 255 (maximum).
    speed: speed of the movement; 0 (minimum), 255 (maximum).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def MoveGripper(self, pos, max_effort, speed=255, blocking=False):
    pos= max(0,min(255,int(pos)))
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 1
    cmd.rPR= pos  #Position Request
    cmd.rSP= speed
    cmd.rFR= max_effort
    self.pub.grip.publish(cmd)
    if blocking:
      while pos!=self.status.gPR and not rospy.is_shutdown():
        #self.PrintStatus(self.status)
        rospy.sleep(0.001)
      prev_PO= None
      CMAX= 500
      counter= CMAX
      while not (self.status.gGTO==0 or self.status.gOBJ==3) and not rospy.is_shutdown():
        #self.PrintStatus(self.status)
        if self.status.gPO==prev_PO:  counter-= 1
        else:  counter= CMAX
        if counter==0:  break
        prev_PO= self.status.gPO
        rospy.sleep(0.001)
      #self.StopGripper()

  '''Stop the gripper motion. '''
  def StopGripper(self):
    cmd= robotiq_msgs.CModel_robot_output();
    cmd.rACT= 1
    cmd.rGTO= 0
    self.pub.grip.publish(cmd)

