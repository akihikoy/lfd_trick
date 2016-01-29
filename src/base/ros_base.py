#! /usr/bin/env python
#ROS basic tools.
import rospy
import actionlib as al
import trajectory_msgs.msg

from base_util import *

'''Block the execution of action client.
  act_client: action client that is executing an action.
  blocking: False: not block, True: wait until action ends, 'time': wait until duration.
  duration: duration in seconds.
  accuracy: accuracy to check duration.
'''
def BlockAction(act_client, blocking, duration, accuracy=0.02):
  if blocking==False:  return
  if blocking=='time':
    end_time= rospy.Time.now() + rospy.Duration(duration)
    dt= duration*accuracy
    while rospy.Time.now() < end_time:
      time.sleep(dt)
    return
  if blocking==True:
    act_client.wait_for_result()
    return
  raise Exception('BlockAction: invalid blocking type: %r'%blocking)


def SetupServiceProxy(name, srv_type, persistent=False, time_out=None):
  print 'Waiting for %s... (t/o: %r)' % (name, time_out)
  try:
    rospy.wait_for_service(name, time_out)
  except rospy.exceptions.ROSException as e:
    print 'Failed to connect the service %s' % name
    print '  Error:',str(e)
    return None
  srvp= rospy.ServiceProxy(name, srv_type, persistent=persistent)
  return srvp


'''Setup an instance of SimpleActionClient.
  e.g.
    SimpleActionClient(name, act_type, time_out=5.0, num_wait=None)
      Waiting for [name]... (up to 5.0s)
      Waiting for [name]... (up to 5.0s)
      (until connection is established)
    SimpleActionClient(name, act_type, time_out=5.0)
      Waiting for [name]... (up to 5.0s)
      (return None if connection is not established)
'''
def SetupSimpleActionClient(name, act_type, time_out=None, num_wait=1):
  actc= al.SimpleActionClient(name, act_type)
  if time_out==None:  time_out= rospy.Duration()
  elif not isinstance(time_out, rospy.Duration):  time_out= rospy.Duration(time_out)
  while num_wait==None or num_wait>0:
    print 'Waiting for %s... (t/o: %r, #: %r)' % (name, time_out.to_sec(), num_wait)
    if actc.wait_for_server(time_out):
      return actc
    num_wait-= 1
  print 'Failed to connect the action service %s' % name


#Support function to generate trajectory_msgs/JointTrajectoryPoint.
def ROSGetJTP(q,t):
  jp= trajectory_msgs.msg.JointTrajectoryPoint()
  jp.positions= q
  jp.time_from_start= rospy.Duration(t)
  return jp

'''Get trajectory_msgs/JointTrajectory from a joint angle trajectory.
  joint_names: joint names.
  q_traj: joint angle trajectory [q0,...,qD]*N.
  t_traj: corresponding times in seconds from start [t1,t2,...,tN]. '''
def ToROSTrajectory(joint_names, q_traj, t_traj):
  assert(len(q_traj)==len(t_traj))
  traj= trajectory_msgs.msg.JointTrajectory()
  traj.joint_names= joint_names
  traj.points= [ROSGetJTP(q,t) for q,t in zip(q_traj, t_traj)]
  traj.header.stamp= rospy.Time.now()
  return traj


'''Basic ROS utility class.'''
class TROSUtil(object):
  def __init__(self):
    self._is_initialized= False

    #Container for Publishers
    self.pub= TContainer()
    #Container for Subscribers
    self.sub= TContainer()
    #Container for Service proxies
    self.srvp= TContainer()
    #Container for SimpleActionClient
    self.actc= TContainer()

  def __del__(self):
    self.Cleanup()

  def Init(self):
    self._is_initialized= False
    '''Example:
    res= []
    ra= lambda r: res.append(r)

    ra(self.AddPub(...))
    ra(self.AddPub(...))
    ra(self.AddSrvP(...))

    if False not in res:  self._is_initialized= True
    return self._is_initialized
    '''

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    for k in self.sub.keys():
      print 'Stop subscribing %r...' % k,
      self.sub[k].unregister()
      del self.sub[k]
      print 'ok'

    for k in self.pub.keys():
      print 'Stop publishing %r...' % k,
      self.pub[k].publish()
      self.pub[k].unregister()
      del self.pub[k]
      print 'ok'

    for k in self.srvp.keys():
      print 'Delete service proxy %r...' % k,
      del self.srvp[k]
      print 'ok'

    for k in self.actc.keys():
      print 'Delete action client %r...' % k,
      del self.actc[k]
      print 'ok'

  @property
  def IsInitialized(self):
    return self._is_initialized

  def AddSub(self, name, port_name, port_type, call_back):
    if name not in self.sub:
      self.sub[name]= rospy.Subscriber(port_name, port_type, call_back)
    return True

  def AddPub(self, name, port_name, port_type):
    if name not in self.pub:
      self.pub[name]= rospy.Publisher(port_name, port_type)
    return True

  def AddSrvP(self, name, port_name, port_type, persistent=False, time_out=None):
    if name not in self.srvp:
      srvp= SetupServiceProxy(port_name, port_type, persistent, time_out)
      if srvp==None:  return False
      else:  self.srvp[name]= srvp
    return True

  def AddActC(self, name, port_name, port_type, time_out=None, num_wait=1):
    if name not in self.actc:
      actc= SetupSimpleActionClient(port_name, port_type, time_out, num_wait)
      if actc==None:  return False
      else:  self.actc[name]= actc
    return True

  def DelSub(self, name):
    if name in self.sub:
      self.sub[name].unregister()
      del self.sub[name]

  def DelPub(self, name):
    if name in self.pub:
      self.pub[name].publish()
      self.pub[name].unregister()
      del self.pub[name]

  def DelSrvP(self, name):
    if name in self.srvp:
      del self.srvp[name]

  def DelActC(self, name):
    if name in self.actc:
      del self.actc[name]

