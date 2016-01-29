#!/usr/bin/python
from core_tool import *
def Help():
  return '''Change config of ODE grasping and pouring simulation.
  Usage: config1'''
def Run(t,*args):
  l= TContainer(debug=True)
  sim= t.LoadMotion('tsim.core1')

  sim.SetupServiceProxy(t,l)
  sim.SetupPubSub(t,l)

  t.srvp.ode_resume()
  l.config= sim.GetConfig(t)
  print 'Current config:',l.config

  #Setup config
  l.config.MaxContacts= 2
  l.config.TimeStep= 0.025
  l.config.Gravity= -1.0
  l.config.BallType= 0  #Sphere particles
  #l.config.BallType= 1  #Box particles
  l.config.SrcSize2H= 0.03  #Mouth size; Default: 0.03
  #Bounce balls:
  l.config.ContactBounce= 0.7
  l.config.ContactBounceVel= 0.2
  l.config.ViscosityParam1= 0.0  #Default: 0.0
  #Natto:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 1.5e-6  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.1  #Default: 0.1
  #Ketchup:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 2.5e-7  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.2  #Default: 0.1
  #Slime???:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 1.0e-5  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.1  #Default: 0.1

  #Reset to get state for plan
  sim.ResetConfig(t,l.config)
  time.sleep(0.1)  #Wait for l.sensors is updated
  t.srvp.ode_pause()  #Pause to wait grasp plan
  print '---------------'
  print 'New config:',sim.GetConfig(t)
  print '---------------'
  print 'Run tsim.clean to disconnect'
