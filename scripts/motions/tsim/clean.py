#!/usr/bin/python
from core_tool import *
def Help():
  return '''Clean the ROS connections used in tsim.
  Usage: tsim.clean'''
def Run(t,*args):
  t.DelSub('ode_sensors')

  t.DelPub('ode_ppour')
  t.DelPub('ode_theta')
  t.DelPub('ode_viz')

  t.DelSrvP('ode_get_config')
  t.DelSrvP('ode_reset2')
  t.DelSrvP('ode_pause')
  t.DelSrvP('ode_resume')
