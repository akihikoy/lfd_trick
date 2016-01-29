#! /usr/bin/env python
#Basic tools (trajectory).
import numpy as np
import numpy.linalg as la
import math
import random
import copy
from base_util import *
from base_const import *
from base_geom import *

#Get a sequence of times, from 0 to dt including inum points (0 is not included).
def TimeTraj(dt, inum):
  return FRange1(0.0, dt, inum)[1:]

#Remove radian jumping in a joint angle trajectory.
def SmoothQTraj(q_traj):
  if len(q_traj)==0:  return
  q_prev= np.array(q_traj[0])
  q_offset= np.array([0]*len(q_prev))
  for q in q_traj:
    q_diff= np.array(q) - q_prev
    for d in range(len(q_prev)):
      if q_diff[d]<-math.pi:  q_offset[d]+=1
      elif q_diff[d]>math.pi:  q_offset[d]-=1
    q_prev= copy.deepcopy(q)
    q[:]= q+q_offset*2.0*math.pi

'''Limit joint velocities.
Sequence of times t_traj will be directly modified.
  q_start: joint angles at t=0.
  q_traj: joint angle trajectory [q0,...,qD]*N.
  t_traj: corresponding times in seconds from start [t1,t2,...,tN].
  qvel_limits: limit velocities. '''
def LimitQTrajVel(q_start, q_traj, t_traj, qvel_limits, termlen=9, dv=0.08):
  assert(len(q_traj)==len(t_traj))
  if len(q_traj)==0:  return

  def sub_proc(qt, i, t_offset, t_prev, q_prev, vel_limits):
    q= qt[0][i]
    qt[1][i]+= t_offset
    qd= [AngleMod1(q[d]-q_prev[d]) / (qt[1][i]-t_prev) for d in range(len(q))]
    s= max([abs(qd[d])/vel_limits[d] for d in range(len(qd))])
    if s>1.0:
      diff= (qt[1][i]-t_prev)*float(s-1.0)
      t_offset+= diff
      qt[1][i]+= diff
    CPrint(2,'s=',s,qt[1][i]-t_prev,qd)
    return t_offset, qt[1][i], q

  t_offset= 0.0
  t_prev= 0.0
  q_prev= q_start
  qt= (q_traj,t_traj)

  offset=0
  i_term= termlen if len(q_traj)>2*termlen else len(q_traj)/2
  i_middle= len(q_traj)-2*i_term
  for i in range(i_term):
    vel_limits= [v*dv*float(i+1+offset) for v in qvel_limits]
    t_offset,t_prev,q_prev= sub_proc(qt, i, t_offset, t_prev, q_prev, vel_limits)
  for i in range(i_middle):
    t_offset,t_prev,q_prev= sub_proc(qt, i_term+i, t_offset, t_prev, q_prev, qvel_limits)
  for i in range(i_term):
    vel_limits= [v*dv*float(i_term-i+offset) for v in qvel_limits]
    t_offset,t_prev,q_prev= sub_proc(qt, i_term+i_middle+i, t_offset, t_prev, q_prev, vel_limits)

#Check the velocity consistency between x_traj and q_traj
#  where x_traj is target Cartesian trajectory,
#  q_traj is corresponding joint angle trajectory solved by IK.
#If the norm of inconsistency is greater than err_thresh,
#  return None immediately, otherwise, return the maximum of the norms.
def IKTrajCheck(x_traj, q_traj, err_thresh=0.01):
  #Compute velocities
  xd_traj= [Vec(x_traj[i])-Vec(x_traj[i-1]) for i in range(1,len(x_traj))]
  qd_traj= [Vec(q_traj[i])-Vec(q_traj[i-1]) for i in range(1,len(q_traj))]
  max_err= 0.0
  for i in range(3,len(xd_traj)):
    #Roughly estimate the next Cartesian velocity
    xd_est= np.dot(Vec([xd_traj[i-3],xd_traj[i-2],xd_traj[i-1]]).T,
                   np.dot(la.pinv(Vec([qd_traj[i-3],qd_traj[i-2],qd_traj[i-1]]).T),
                          qd_traj[i]) )
    err= la.norm(xd_traj[i]-xd_est)
    if err>err_thresh:  return None
    if err>max_err:  max_err= err
  return max_err

'''DEPRECATED
#Interpolate (p1)-(p2) of duration with time_step and store into traj_points
#(p1) is not contained.  Velocities are automatically computed
#If rot_adjust is true, each dimension is considered as a revolute joint, and a shorter direction is automatically chosen
#If vel_limits is specified, the velocity is adjusted so that it does not exceed vel_limits.
#When the velocity is modified, the duration is also modified, which is returned
def InterpolateLinearly2(traj_points,p1,p2,duration,time_step,rot_adjust=False, vel_limits=[]):
  traj_points[:]= []
  v= (p2-p1)/duration
  if rot_adjust:
    for d in range(len(p1)):
      delta= p2[d]-p1[d]
      if math.fabs(delta)>math.pi:
        v[d]= AngleMod1(delta)/duration
  if len(vel_limits)>0:
    #Borrowed Scott's velocity modification
    ratios= [math.fabs(v[d]) / vel_limits[d] for d in range(len(v))]
    max_r= max(ratios)
    if max_r>1.0:
      v= v/max_r
      duration= duration*max_r
  t= 0.0
  jp= trajectory_msgs.msg.JointTrajectoryPoint()
  t+= time_step
  while t<duration:
    p= p1+v*t
    jp.time_from_start= rospy.Duration(t)
    jp.positions= p.copy()
    #jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
    t+= time_step
  if t-time_step<=duration:
    p= p1+v*duration
    jp.time_from_start= rospy.Duration(duration)
    jp.positions= p.copy()
    #jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
  return duration
'''

#Return the interpolation from x1 to x2 with N points
#p1 is not included
def XInterpolation(x1,x2,N):
  p1,R1= XToPosRot(x1)
  p2,R2= XToPosRot(x2)
  dp= (p2-p1)/float(N)
  trans_R= np.dot(R2,R1.T)
  w= InvRodrigues(trans_R)
  traj=[]
  for t in range(N):
    R= np.dot(Rodrigues(float(t+1)/float(N)*w),R1)
    p1= p1+dp
    traj.append(PosRotToX(p1,R))
  return traj

#Return the interpolation from q1 to q2 with N points
#q1 is not included
def QInterpolation(q1,q2,N):
  R1= QToRot(q1)
  R2= QToRot(q2)
  trans_R= np.dot(R2,R1.T)
  w= InvRodrigues(trans_R)
  traj=[]
  for t in range(N):
    R= np.dot(Rodrigues(float(t+1)/float(N)*w),R1)
    traj.append(RotToQ(R))
  return traj

'''Transform a Cartesian trajectory to joint angle trajectory.
  func_ik: IK function (x, q_start).
  x_traj: pose sequence [x1, x2, ...].
  start_angles: joint angles used for initial pose of first IK.  '''
def XTrajToQTraj(func_ik, x_traj, start_angles):
  N= len(x_traj)
  q_prev= start_angles
  q_traj= None
  for x,n in zip(x_traj, range(N)):
    q= func_ik(x, q_prev)
    if q==None:  return None
    if q_traj==None:  q_traj= [[0.0]*len(q) for i in range(N)]
    q_traj[n][:]= q
    q_traj[n][:]= q
    q_prev= q
  SmoothQTraj(q_traj)
  return q_traj

#Generate a cubic Hermite spline from a key points.
#Key points: [[t0,x0],[t1,x1],[t2,x2],...].
class TCubicHermiteSpline:
  class TKeyPoint:
    T= 0.0  #Input
    X= 0.0  #Output
    M= 0.0  #Gradient
    def __str__(self):
      return '['+str(self.T)+', '+str(self.X)+', '+str(self.M)+']'

  class TParam: pass

  def __init__(self):
    self.idx_prev= 0
    self.Param= self.TParam()

  def FindIdx(self, t, idx_prev=0):
    idx= idx_prev
    if idx>=len(self.KeyPts): idx= len(self.KeyPts)-1
    while idx+1<len(self.KeyPts) and t>self.KeyPts[idx+1].T:  idx+=1
    while idx>=0 and t<self.KeyPts[idx].T:  idx-=1
    return idx

  #Return interpolated value at t
  def Evaluate(self, t):
    idx= self.FindIdx(t,self.idx_prev)
    if abs(t-self.KeyPts[-1].T)<1.0e-6:  idx= len(self.KeyPts)-2
    if idx<0 or idx>=len(self.KeyPts)-1:
      print 'WARNING: Given t= %f is out of the key points (index: %i)' % (t,idx)
      if idx<0:
        idx= 0
        t= self.KeyPts[0].T
      else:
        idx= len(self.KeyPts)-2
        t= self.KeyPts[-1].T

    h00= lambda t: t*t*(2.0*t-3.0)+1.0
    h10= lambda t: t*(t*(t-2.0)+1.0)
    h01= lambda t: t*t*(-2.0*t+3.0)
    h11= lambda t: t*t*(t-1.0)

    self.idx_prev= idx
    p0= self.KeyPts[idx]
    p1= self.KeyPts[idx+1]
    tr= (t-p0.T) / (p1.T-p0.T)
    return h00(tr)*p0.X + h10(tr)*(p1.T-p0.T)*p0.M + h01(tr)*p1.X + h11(tr)*(p1.T-p0.T)*p1.M

  #Compute a phase information (n, tp) for a cyclic spline curve.
  #n:  n-th occurrence of the base wave
  #tp: phase (time in the base wave)
  def PhaseInfo(self, t):
    t0= self.KeyPts[0].T
    te= self.KeyPts[-1].T
    T= te-t0
    mod= Mod(t-t0,T)
    tp= t0+mod  #Phase
    n= (t-t0-mod)/T
    return n, tp

  #Return interpolated value at t (cyclic version).
  #pi: Phase information.
  def EvaluateC(self, t, pi=None):
    if pi==None:
      n, tp= self.PhaseInfo(t)
    else:
      n, tp= pi
    return self.Evaluate(tp) + n*(self.KeyPts[-1].X - self.KeyPts[0].X)

  #data= [[t0,x0],[t1,x1],[t2,x2],...]
  FINITE_DIFF=0  #Tangent method: finite difference method
  CARDINAL=1  #Tangent method: Cardinal spline (c is used)
  ZERO= 0  #End tangent: zero
  GRAD= 1  #End tangent: gradient (m is used)
  CYCLIC= 2  #End tangent: treating data as cyclic (KeyPts[-1] and KeyPts[0] are considered as an identical point)
  def Initialize(self, data, tan_method=CARDINAL, end_tan=GRAD, c=0.0, m=1.0):
    if data != None:
      self.KeyPts= [self.TKeyPoint() for i in range(len(data))]
      for idx in range(len(data)):
        self.KeyPts[idx].T= data[idx][0]
        self.KeyPts[idx].X= data[idx][1]

    #Store parameters for future use / remind parameters if not given
    if tan_method==None:  tan_method= self.Param.TanMethod
    else:                 self.Param.TanMethod= tan_method
    if end_tan==None:  end_tan= self.Param.EndTan
    else:              self.Param.EndTan= end_tan
    if c==None:  c= self.Param.C
    else:        self.Param.C= c
    if m==None:  c= self.Param.M
    else:        self.Param.M= m

    grad= lambda idx1,idx2: (self.KeyPts[idx2].X-self.KeyPts[idx1].X)/(self.KeyPts[idx2].T-self.KeyPts[idx1].T)

    if tan_method == self.FINITE_DIFF:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= 0.5*grad(idx,idx+1) + 0.5*grad(idx-1,idx)
    elif tan_method == self.CARDINAL:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= (1.0-c)*grad(idx-1,idx+1)

    if end_tan == self.ZERO:
      self.KeyPts[0].M= 0.0
      self.KeyPts[-1].M= 0.0
    elif end_tan == self.GRAD:
      self.KeyPts[0].M= m*grad(0,1)
      self.KeyPts[-1].M= m*grad(-2,-1)
    elif end_tan == self.CYCLIC:
      if tan_method == self.FINITE_DIFF:
        grad_p1= grad(0,1)
        grad_n1= grad(-2,-1)
        M= 0.5*grad_p1 + 0.5*grad_n1
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M
      elif tan_method == self.CARDINAL:
        T= self.KeyPts[-1].T - self.KeyPts[0].T
        X= self.KeyPts[-1].X - self.KeyPts[0].X
        grad_2= (X+self.KeyPts[1].X-self.KeyPts[-2].X)/(T+self.KeyPts[1].T-self.KeyPts[-2].T)
        M= (1.0-c)*grad_2
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M

  def Update(self):
    self.Initialize(data=None, tan_method=None, end_tan=None, c=None, m=None)

#Modify the velocity of a given trajectory (base routine).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#traj: Function to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
#norm: Function to compute norm of two vectors
#diff: Function to compute difference of two vectors
#is_samedir: Function to judge if two vectors have the same direction
def modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir):
  num_iter_l= int(num_iter_l)
  num_iter_e= int(num_iter_e)
  ddt= time_step/(float(num_iter_l)/5.0)
  v_dt= v*time_step
  x0= traj(t0)
  t1= t0
  x1= x0
  s1= None
  cross= False
  #print '-----------'
  while num_iter_l>0 and num_iter_e>0 and t1<T:
    t2= t1+ddt
    if t2>T:  t2= T
    x2= traj(t2)
    dx= norm(x2,x0)
    s2= diff(x2,x1)
    if s1==None:  s1= s2
    #print t2,dx/time_step,s1,s2
    samedir= is_samedir(s1,s2)
    if dx<v_dt and samedir:
      t1= t2
      x1= x2
      num_iter_l-= 1
    else:
      if dx<v_dt and not samedir:
        #print 'cross#',t2,x2
        cross= True
        over_t2= t2
        over_x2= x2
      ddt*= 0.5
      num_iter_e-= 1
  if cross:
    #print 'cross',over_t2,over_x2,num_iter_l,num_iter_e
    t1= over_t2
    x1= over_x2
  #print t1,t1-t0, x1, norm(x1,x0)/time_step, v,num_iter_l, num_iter_e
  return t1, x1, norm(x1,x0)/time_step

#Modify the velocity of a given trajectory (1-d version).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#traj: Function to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
def ModifyTrajVelocity(t0, v, traj, time_step, T, num_iter_l=50, num_iter_e=6):
  norm= lambda x2,x1: abs(x2-x1)
  diff= lambda x2,x1: x2-x1
  is_samedir= lambda s1,s2: Sign(s1)==Sign(s2)
  return modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir)

#Modify the velocity of a given trajectory (N-d version).
#t0: Current internal time maintaining a playing point of the trajectory
#v: Target speed
#trajs: List of functions to map time t --> point x
#time_step: Control time step (actual value)
#T: Maximum internal time
#num_iter_l: Limit of iteration number (linear search)
#num_iter_e: Limit of iteration number (exponential search)
def ModifyTrajVelocityV(t0, v, traj, time_step, T, num_iter_l=50, num_iter_e=6):
  diff= lambda x2,x1: [x2d-x1d for (x2d,x1d) in zip(x2,x1)]
  norm= lambda x2,x1: la.norm(diff(x2,x1))
  is_samedir= lambda s1,s2: np.dot(s1,s2) > 0
  return modify_traj_velocity_base(t0, v, traj, time_step, T, num_iter_l, num_iter_e, norm, diff, is_samedir)
