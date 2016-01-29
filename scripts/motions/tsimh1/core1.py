#!/usr/bin/python
from core_tool import *
def Help():
  return '''Core of ODE hopping robot simulation.
  Usage: do not call this directly.'''

def SetupServiceProxy(t,l):
  t.AddSrvP('ode_get_config', '/ode_hopper1/get_config',
            lfd_sim.srv.ODEGetConfigHp1, persistent=False, time_out=3.0)
  t.AddSrvP('ode_reset2', '/ode_hopper1/reset2',
            lfd_sim.srv.ODESetConfigHp1, persistent=False, time_out=3.0)
  t.AddSrvP('ode_pause', '/ode_hopper1/pause',
            std_srvs.srv.Empty, persistent=False, time_out=3.0)
  t.AddSrvP('ode_resume', '/ode_hopper1/resume',
            std_srvs.srv.Empty, persistent=False, time_out=3.0)

def SetupPubSub(t,l):
  StopPubSub(t,l)
  t.AddPub('ode_ctrl', '/ode_hopper1/control', lfd_sim.msg.ODEControlHp1)
  t.AddPub('ode_viz', '/ode_hopper1/viz', lfd_sim.msg.ODEViz)
  t.AddSub('ode_sensors', '/ode_hopper1/sensors',
           lfd_sim.msg.ODESensorHp1,
           lambda msg:ODESensorCallback(msg,t,l))
  if 'sensor_callback' not in l:
    l.sensor_callback= None

def StopPubSub(t,l):
  t.DelSub('ode_sensors')

def ODESensorCallback(msg,t,l):
  l.sensors= msg
  #print l.sensors.Time
  if l.sensor_callback!=None:
    l.sensor_callback()

def GetConfig(t):
  return t.srvp.ode_get_config().config

def ResetConfig(t,config):
  t.pub.ode_viz.publish(lfd_sim.msg.ODEViz())  #Clear visualization
  req= lfd_sim.srv.ODESetConfigHp1Request()
  req.config= config
  t.srvp.ode_reset2(req)

def SimSleep(t,l,dt):
  tc0= l.sensors.Time
  while l.sensors.Time-tc0<dt:
    time.sleep(dt*0.02)

def MoveDTheta(t,l,dth):
  dt= l.config.TimeStep
  theta0= Vec(l.sensors.JointAngles)
  ctrl_msg= lfd_sim.msg.ODEControlHp1()
  ctrl_msg.Angles= theta0 + dth
  t.pub.ode_ctrl.publish(ctrl_msg)
  SimSleep(t,l,dt)

def TrackTraj(t,l,q_traj,t_traj):
  assert(len(q_traj)==len(t_traj))
  t_prev= 0.0
  ctrl_msg= lfd_sim.msg.ODEControlHp1()
  for q_curr,t_curr in zip(q_traj,t_traj):
    ctrl_msg.Angles= q_curr
    t.pub.ode_ctrl.publish(ctrl_msg)
    SimSleep(t,l,t_curr-t_prev)
    t_prev= t_curr

def Run(t,*args):
  print Help()
