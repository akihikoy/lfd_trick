#!/usr/bin/python
from core_tool import *
def Help():
  return '''Spline trajectory control of ODE hopping robot simulation.
  Usage: template'''

#Generate key points: [t,x-1,x-2,...,x-Nd]*Ns
def GenKeyPoints(x0=[0.0],xf=[1.0],tf=1.0,param=[[0.0,0.0]]):
  Nd= len(x0)
  data= [[0.0]+[None]*Nd,
         [0.333*tf]+[None]*Nd,
         [0.666*tf]+[None]*Nd,
         [tf]+[None]*Nd]
  for d in range(Nd):
    a= (xf[d]-x0[d])/tf
    data[0][1+d]= x0[d]
    data[1][1+d]= x0[d]+a*data[1][0] + param[d][0]
    data[2][1+d]= x0[d]+a*data[2][0] + param[d][1]
    data[3][1+d]= xf[d]
  return data

def Run(t,*args):
  l= TContainer(debug=True)

  sim= t.LoadMotion('tsimh1.core1')

  try:
    sim.SetupServiceProxy(t,l)
    sim.SetupPubSub(t,l)

    t.srvp.ode_resume()
    l.config= sim.GetConfig(t)
    print 'Current config:',l.config

    #Setup config
    #l.config.BaseDensity= 500.0

    #Reset with sending config
    sim.ResetConfig(t,l.config)
    time.sleep(0.1)  #Wait for l.sensors is updated

    print 'l.sensors:',l.sensors

    #Jump motion:
    #key_points= [[0.0, 0.0,0.0,0.0,0.0],
                 #[0.5, 0.0,0.0,1.8,-1.9],
                 #[0.6, 0.0,0.0,0.0,0.0]]
    key_points= GenKeyPoints(x0=[0.0]*4, xf=[0.0]*4, tf=0.7,
                  param=[[0.0,0.0],
                         [0.0,0.0],
                         RandN([-2.0,-2.0],[2.0,2.0]),
                         RandN([-2.0,-2.0],[2.0,2.0])])
    splines= [TCubicHermiteSpline() for d in range(len(key_points[0])-1)]
    for d in range(len(splines)):
      data_d= [[p[0],p[d+1]] for p in key_points]
      splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)
    q_traj= []
    t_traj= []
    t_curr= key_points[0][0]
    while True:
      q= [splines[d].Evaluate(t_curr) for d in range(len(splines))]
      q_traj.append(q)
      t_traj.append(t_curr)
      if t_curr>key_points[-1][0]:  break
      t_curr+= 0.02
    sim.TrackTraj(t,l,q_traj,t_traj)

    time.sleep(2.0)  #Observe

  except Exception as e:
    PrintException(e, ' in tsim.test1')

  finally:
    sim.StopPubSub(t,l)
    t.srvp.ode_pause()
