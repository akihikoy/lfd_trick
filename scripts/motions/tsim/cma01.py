#!/usr/bin/python
from core_tool import *
def Help():
  return '''CMA for grasping and pouring in ODE simulation.
  Usage: tsim.cma01'''

def PlanLearnCallback(t,l,sim,context):
  if context=='infer_grab':
    #Plan l.config.GripperHeight
    gh= l.learn_grab.Select()
    l.config.GripperHeight= gh[0]*l.sensors.p_pour[2]  #Should be in [0,l.sensors.p_pour[2]]
    l.exec_status= SUCCESS_CODE

  elif context=='infer_pour':
    #Plan l.p_pour_trg, l.p_pour_trg0, l.theta_init
    pp= l.learn_ppour.Select()
    l.p_pour_trg= [l.sensors.x_rcv.position.x+pp[0], 0.0, l.sensors.x_rcv.position.z+pp[1]]
    l.p_pour_trg0= Vec(l.p_pour_trg)+[0.0,0.0,0.2]
    l.theta_init= DegToRad(45.0)
    l.exec_status= SUCCESS_CODE

  elif context=='end_of_flowc_gen':
    if l.sensors.num_spill>0:
      CPrint(3,'Failure: num_spill=',l.sensors.num_spill)
      l.learn_grab.Update(-0.1*l.sensors.num_spill)
      l.learn_ppour.Update(-0.1*l.sensors.num_spill)
    else:
      l.learn_grab.Update(1.0)
      l.learn_grab.Update(-0.1*l.sensors.num_spill)

def Run(t,*args):
  l= TContainer(debug=True)
  l.planlearn_callback= PlanLearnCallback
  m_sm= t.LoadMotion('tsim.sm1')

  #Setup learners
  l.learn_grab= TContOptNoGrad()
  options= {}
  options['bounds']= [[0.0],[1.0]]
  options['tolfun']= 1.0e-4
  options['scale0']= 0.5
  options['parameters0']= [0.5]
  l.learn_grab.Init({'options':options})

  l.learn_ppour= TContOptNoGrad()
  options= {}
  options['bounds']= [[-0.2,0.2],[0.2,1.0]]
  options['tolfun']= 1.0e-4
  options['scale0']= 0.2
  options['parameters0']= [-0.1,0.4]
  l.learn_ppour.Init({'options':options})

  for i in range(1):
    res= m_sm.PourSM(t,l)

  l= None
  return res
