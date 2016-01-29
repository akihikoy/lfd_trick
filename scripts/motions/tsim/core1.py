#!/usr/bin/python
from core_tool import *
def Help():
  return '''Core of ODE grasping and pouring simulation.
  Usage: do not call this directly.'''

def SetupServiceProxy(t,l):
  if 'ode_get_config' not in t.srvp:
    print 'Waiting for /ode_grpour_sim/get_config...'
    rospy.wait_for_service('/ode_grpour_sim/get_config',3.0)
    t.srvp.ode_get_config= rospy.ServiceProxy('/ode_grpour_sim/get_config', lfd_sim.srv.ODEGetConfig, persistent=False)
  if 'ode_reset2' not in t.srvp:
    print 'Waiting for /ode_grpour_sim/reset2...'
    rospy.wait_for_service('/ode_grpour_sim/reset2',3.0)
    t.srvp.ode_reset2= rospy.ServiceProxy('/ode_grpour_sim/reset2', lfd_sim.srv.ODEReset2, persistent=False)
  if 'ode_pause' not in t.srvp:
    print 'Waiting for /ode_grpour_sim/pause...'
    rospy.wait_for_service('/ode_grpour_sim/pause',3.0)
    t.srvp.ode_pause= rospy.ServiceProxy('/ode_grpour_sim/pause', std_srvs.srv.Empty, persistent=False)
  if 'ode_resume' not in t.srvp:
    print 'Waiting for /ode_grpour_sim/resume...'
    rospy.wait_for_service('/ode_grpour_sim/resume',3.0)
    t.srvp.ode_resume= rospy.ServiceProxy('/ode_grpour_sim/resume', std_srvs.srv.Empty, persistent=False)

def SetupPubSub(t,l):
  StopPubSub(t,l)
  if 'ode_ppour' not in t.pub:
    t.pub.ode_ppour= rospy.Publisher("/ode_grpour_sim/ppour", std_msgs.msg.Float64MultiArray)
  if 'ode_theta' not in t.pub:
    t.pub.ode_theta= rospy.Publisher("/ode_grpour_sim/theta", std_msgs.msg.Float64)
  if 'ode_viz' not in t.pub:
    t.pub.ode_viz= rospy.Publisher("/ode_grpour_sim/viz", lfd_sim.msg.ODEViz)
  if 'ode_sensors' not in t.sub:
    t.sub.ode_sensors= rospy.Subscriber("/ode_grpour_sim/sensors", lfd_sim.msg.ODESensor, lambda msg:ODESensorCallback(msg,t,l))
  if 'sensor_callback' not in l:
    l.sensor_callback= None

def StopPubSub(t,l):
  if 'ode_sensors' in t.sub:
    t.sub.ode_sensors.unregister()
    del t.sub.ode_sensors

def ODESensorCallback(msg,t,l):
  l.sensors= msg
  print l.sensors.num_src, l.sensors.num_rcv, l.sensors.num_flow, l.sensors.num_spill, l.sensors.num_bounce
  if l.sensor_callback!=None:
    l.sensor_callback()

def GetConfig(t):
  return t.srvp.ode_get_config().config

def ResetConfig(t,config):
  t.pub.ode_viz.publish(lfd_sim.msg.ODEViz())  #Clear visualization
  req= lfd_sim.srv.ODEReset2Request()
  req.config= config
  t.srvp.ode_reset2(req)

def SimSleep(t,l,dt):
  tc0= l.sensors.time
  while l.sensors.time-tc0<dt:
    time.sleep(dt*0.02)

def MoveDPPour(t,l,dp):
  dt= l.config.TimeStep
  p_pour0= l.sensors.p_pour
  p_pour_msg= std_msgs.msg.Float64MultiArray()
  p_pour_msg.data= Vec(p_pour0) + Vec(dp)
  t.pub.ode_ppour.publish(p_pour_msg)
  SimSleep(t,l,dt)

def MoveDTheta(t,l,dth):
  dt= l.config.TimeStep
  theta0= l.sensors.theta
  theta_msg= std_msgs.msg.Float64()
  theta_msg.data= theta0 + dth
  t.pub.ode_theta.publish(theta_msg)
  SimSleep(t,l,dt)

def MoveToTrgPPour(t,l,p_pour_trg,spd):
  dt= l.config.TimeStep
  p_pour0= l.sensors.p_pour
  diff= Vec(p_pour_trg) - Vec(p_pour0)
  diff_norm= la.norm(diff)
  p_pour_msg= std_msgs.msg.Float64MultiArray()
  if diff_norm>spd*dt:
    p_pour_msg.data= Vec(p_pour0) + diff*(spd*dt)/diff_norm
    reached= False
  else:
    p_pour_msg.data= Vec(p_pour_trg)
    reached= True
  t.pub.ode_ppour.publish(p_pour_msg)
  SimSleep(t,l,dt)
  return reached

def Run(t,*args):
  print Help()
