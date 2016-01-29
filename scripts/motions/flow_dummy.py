#!/usr/bin/python
from core_tool import *
def Help():
  return '''Virtually generate a /color_occupied_ratio topic i.e. material amount.
  WARNING: THIS IS NOT MAINTAINED.
  Usage: flow_dummy C_ID
    C_ID: Source container ID.'''

class TFlowDyn:
  def __init__(self, a_src=1.0, a_rcv=0.0, get_theta=None, time_step=0.03):
    self.a_src= a_src
    self.a_rcv= a_rcv
    self.get_theta= get_theta
    self.time_step= time_step
    self.flow= 0.0
    self.flow_obs= 0.0
    self.t_start= rospy.Time.now()
  def Step(self):
    self.theta= self.get_theta()
    #theta where flow starts:
    #theta_fs= -0.5*math.pi*self.a_src + 0.6666*math.pi
    theta_fs= -0.5*math.pi*self.a_src + 0.8*math.pi
    if self.theta>theta_fs:
      self.flow= min(8.0*(self.theta-theta_fs), self.a_src/self.time_step)
    else:
      self.flow= 0.0
    self.flow_obs= self.flow + 0.01*(random.random()-0.5)
    self.a_src+= self.time_step * (-self.flow)
    self.a_rcv+= self.time_step * self.flow
    self.time= rospy.Time.now()-self.t_start

def GetTheta(t, source):
  t.ExecuteMotion('infer2', {},[source],'x',[],False)
  x_src= t.GetAttr(source,'x')
  p,R= XToPosRot(x_src)
  return GetAngle([0.0,0.0,1.0],R[:,2])

def FlowDummyLoop(th_info, t, source):
  time_step=0.03  #33Hz
  flow_dyn= TFlowDyn(get_theta= lambda:GetTheta(t,source), time_step=time_step)
  loop_rate= rospy.Rate(1.0/time_step)
  t.AddPub('dummy_amount', '/color_occupied_ratio', std_msgs.msg.Float64)
  while th_info.IsRunning():
    flow_dyn.Step()
    t.pub.dummy_amount.publish(flow_dyn.a_rcv)
    #rospy.sleep(time_step)
    loop_rate.sleep()
  t.pub.dummy_amount.unregister()
  del t.pub['dummy_amount']

def Run(t,*args):
  if len(args)>0:
    t.thread_manager.Add(name='dummy_amount', target=lambda th_info: FlowDummyLoop(th_info, t, args[0]))
  else:
    t.thread_manager.Stop(name='dummy_amount')

