#! /usr/bin/env python
#Common robot interface for PR2 and Baxter.
from base_const import *
from base_util import *
from base_traj import *
from ros_base import *


'''Common robot control class for PR2 and Baxter.
  Pose: Cartesian pose (position x,y,z and orientation in quaternion qx,qy,qz,qw).
  End-effector: wrist or something.
  Robot frame: base link, torso link, or something.
'''
class TDualArmRobot(TROSUtil):
  def __init__(self, name):
    TROSUtil.__init__(self)

    self._name= name
    self.currarm= LEFT

    #Thread locker for self.currarm:
    self.currarm_locker= threading.RLock()

    #NOTE: the sub classes should use the following lockers during controlling/sensing.
    #Thread locker for control:
    self.control_locker= threading.RLock()
    #Thread locker for sensor:
    self.sensor_locker= threading.RLock()

  def __del__(self):
    self.Cleanup()
    print '%s: bye.'%self.Name

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    #NOTE: assign True into self._is_initialized after init.

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    #Check the thread lockers status:
    print 'Count of currarm_locker:',self.currarm_locker._RLock__count

    #Check the thread lockers status:
    print 'Count of control_locker:',self.control_locker._RLock__count
    print 'Count of sensor_locker:',self.sensor_locker._RLock__count

    TROSUtil.Cleanup(self)

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    pass

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    return False

  @property
  def Name(self):
    return self._name

  @property
  def Arm(self):
    with self.currarm_locker:
      arm= self.currarm
    return arm

  @Arm.setter
  def Arm(self, arm):
    with self.currarm_locker:
      self.currarm= arm

  @property
  def ArmStr(self):
    return LRToStr(self.Arm)

  @property
  def ArmStrS(self):
    return LRToStrS(self.Arm)

  @property
  def ArmStrs(self):
    return LRToStrs(self.Arm)

  @property
  def BaseFrame(self):
    pass

  '''End link of an arm.'''
  def EndLink(self, arm):
    pass

  '''Names of joints of an arm.'''
  def JointNames(self, arm):
    pass

  def DoF(self, arm=None):
    return 7

  '''Return limits of joint angular velocity.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    pass

  '''Return joint angles of an arm.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def Q(self, arm=None):
    pass

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (None if failure), res: FK status.
    arm: LEFT, RIGHT, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (None if failure), res: status.
    arm: LEFT, RIGHT, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose (i.e. offset) on self.EndLink(arm) frame.
      If not None, we do not consider an offset.
    with_st: whether return the solver status. '''
  def J(self, q=None, x_ext=None, arm=None, with_st=False):
    pass

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
    pass

  '''Follow a joint angle trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    pass

  '''Follow a self.EndLink(arm)-pose trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    x_traj: self.EndLink(arm)-pose trajectory [x,y,z,qx,qy,qz,qw]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def FollowXTraj(self, x_traj, t_traj, x_ext=None, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    q_curr= self.Q(arm)
    q_traj= XTrajToQTraj(lambda x,q_start: self.IK(x, x_ext=x_ext, start_angles=q_start, arm=arm),
                         x_traj, start_angles=q_curr)
    if q_traj is None:
      raise ROSException('ik','FollowXTraj: IK failed')

    self.FollowQTraj(q_traj, t_traj, arm=arm, blocking=blocking)

  '''Control an arm to the target joint angles.
    arm: LEFT, RIGHT, or None (==currarm).
    q_trg: target joint angles.
    dt: duration time in seconds.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def MoveToQ(self, q_trg, dt=4.0, arm=None, blocking=False):
    self.FollowQTraj(q_traj=[q_trg], t_traj=[dt], arm=arm, blocking=blocking)

  '''Control an arm to the target self.EndLink(arm) pose.
    arm: LEFT, RIGHT, or None (==currarm).
    x_trg: target pose.
    dt: duration time in seconds.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def MoveToX(self, x_trg, dt=4.0, x_ext=None, arm=None, blocking=False):
    self.FollowXTraj(x_traj=[x_trg], t_traj=[dt], x_ext=x_ext, arm=arm, blocking=blocking)

  '''Control an arm to the target self.EndLink(arm) pose with a linearly interpolated trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    x_trg: target pose.
    dt: duration time in seconds.
    inum: number of interpolation points.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.
    x_ext: a local pose on the self.EndLink(arm) frame.
      If not None, the final joint angles q satisfies self.FK(q,x_ext,arm)==x_trg. '''
  def MoveToXI(self, x_trg, dt=4.0, x_ext=None, inum=30, arm=None, blocking=False):
    if arm is None:  arm= self.Arm

    x_curr= self.FK(q=None, x_ext=x_ext, arm=arm)
    x_traj= XInterpolation(x_curr,x_trg,inum)
    t_traj= TimeTraj(dt,inum)

    self.FollowXTraj(x_traj, t_traj, x_ext=x_ext, arm=arm, blocking=blocking)

  '''Open a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def OpenGripper(self, arm=None, blocking=False):
    pass

  '''Close a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def CloseGripper(self, arm=None, blocking=False):
    pass


class TFakeRobot(TDualArmRobot):
  def __init__(self):
    TDualArmRobot.__init__(self, name='NoRobot')


