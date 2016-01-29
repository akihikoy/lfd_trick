#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer a spreading trajectory.
  Usage: infer_spread SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)
  '''

#NOTE: This will decide the length of a single phase wave but actual duration
#  is decided through an actual execution where the velocity controller is applied
MAX_TIME= 1.0

#x,y wave pattern generator
class TWaveGenerator:
  def __init__(self,vx=0.1):
    self.data= [[0.0 *MAX_TIME,vx*0.0 , 0.0],
                [0.25*MAX_TIME,vx*0.25, 1.0],
                [0.75*MAX_TIME,vx*0.75,-1.0],
                [1.0 *MAX_TIME,vx*1.0 , 0.0]]
    #FINITE_DIFF, CARDINAL
    self.splines= [TCubicHermiteSpline() for d in range(len(self.data[0])-1)]
    for d in range(len(self.splines)):
      data_d= [[x[0],x[d+1]] for x in self.data]
      self.splines[d].Initialize(data_d, tan_method=self.splines[d].CARDINAL, end_tan=self.splines[d].CYCLIC, c=0.0, m=0.0)
    self.params= []  #Set of [m1,m2] used in Evaluate

  def Evaluate(self, t, m1=None, m2=None):
    if None in (m1,m2):
      n,tp= self.splines[0].PhaseInfo(t)
      n= int(n)
      if m1==None:  m1= self.params[n][0] if n<len(self.params) else 1.0
      if m2==None:  m2= self.params[n][1] if n<len(self.params) else 1.0
    self.splines[1].KeyPts[1].X= self.data[1][2] * m1
    self.splines[1].KeyPts[2].X= self.data[2][2] * m2
    self.splines[1].Update()
    return [self.splines[d].EvaluateC(t) for d in range(len(self.splines))]

#Index sampler good for searching
def IdSampler(N):
  if N==0:  return []
  if N==1:  return [0]
  if N==2:  return [0,1]
  src= range(N)
  res= []
  res.append(src.pop(0))
  res.append(src.pop(-1))
  d= 2
  while True:
    for i in range(1,d,2):
      res.append(src.pop(len(src)*i/d))
      if len(src)==0:  return res
    d*= 2

def FSampler(xmin,xmax,num_div):
  data= FRange1(xmin,xmax,num_div)
  return [data[i] for i in IdSampler(num_div)]

#Return the evaluation e1,e2 of trajectory p=func(t), t in [0,MAX_TIME].
#e1= whether the first half is inside the polygon.
#e2= whether the last half is inside the polygon.
def EvalWaveFunc(func, points, resolution=20):
  e1= True
  e2= True
  for t in FSampler(0.0*MAX_TIME,0.5*MAX_TIME,resolution/2):
    p= func(t)
    if not PointInPolygon2D(points,p):
      e1= False
      break
  for t in FSampler(0.5*MAX_TIME,1.0*MAX_TIME,resolution/2):
    p= func(t)
    if not PointInPolygon2D(points,p):
      e2= False
      break
  return e1, e2

#Optimize a single phase of wave trajectory p= func(t,p1,p2), t in [0,MAX_TIME],
#parameterized by p1,p2,
#so that p1,p2 satisfies the evaluation function eval_func.
#True/False,True/False= eval_func(lambda t:func(t,p1,p2))
#Greater p1,p2 are better.
#Optimization starts from p1_0, p2_0 that are the maximum values of the parameters.
#search_factor must be in (0.0,1.0); 0.99:slow,accurate, 0.01:fast,inaccurate.
def OptimizeWaveFunc1(func, p1_0, p2_0, eval_func, search_factor=0.9):
  #Check the existence of the solution:
  e1,e2= eval_func(lambda t:func(t,0.0,0.0))
  if not e1 or not e2:  return None,None

  p1= p1_0
  p2= p2_0
  while True:
    e1,e2= eval_func(lambda t:func(t,p1,p2))
    #print p1,p2,e1,e2
    if e1 and e2:  return p1,p2
    if not e1:
      p1*= search_factor
      if p1<1.0e-6:
        p2*= search_factor
    if not e2:
      p2*= search_factor
      if p2<1.0e-6:
        p1*= search_factor

#Plan spreading wave for points.
#points: x,y data.
#Return start, u_dir, wave
#where wave at time t is given by:
#  rot= np.array([[u_dir[0],-u_dir[1]],[u_dir[1],u_dir[0]]])
#  p= np.array(start) + np.dot(rot, wave.Evaluate(t))
def PlanSpreadingWave(points, x_step=0.03, start_search_factor=0.01, search_factor=0.9, eval_resolution=20, verbose=True):
  pca= TPCA(points)
  u_dir= pca.EVecs[0]
  if verbose:  print 'direction=',u_dir
  start= pca.Mean
  while True:
    start2= np.array(start) - start_search_factor*u_dir
    if not PointInPolygon2D(points, start2):  break
    start= start2
  if verbose:  print 'start=',start
  u_dir/= math.sqrt(u_dir[0]**2+u_dir[1]**2)
  rot= np.array([[u_dir[0],-u_dir[1]],[u_dir[1],u_dir[0]]])

  wave= TWaveGenerator(vx=x_step)

  #Planning spreading wave (planning only)
  pstart= copy.deepcopy(start)
  while True:
    func= lambda ti,p1,p2: np.array(pstart) + np.dot(rot, np.array(wave.Evaluate(ti,m1=p1,m2=p2)))
    p1o,p2o= OptimizeWaveFunc1(func, p1_0=2.0, p2_0=2.0, eval_func=lambda f:EvalWaveFunc(f,points,eval_resolution), search_factor=search_factor)
    #p1o,p2o= OptimizeWaveFunc2(func, p1_0=2.0, p2_0=2.0, eval_func=lambda f:EvalWaveFunc(f,points,eval_resolution))
    if None in (p1o,p2o):  break
    wave.params.append([p1o, p2o])
    if verbose:  print p1o, p2o, EvalWaveFunc(lambda t:func(t,p1o,p2o),points,eval_resolution), func(0.0,p1o,p2o), PointInPolygon2D(points,func(0.0,p1o,p2o))
    pstart= func(MAX_TIME,p1o,p2o)

  return start, u_dir, wave


def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  inferred_keys= args[2] if len(args)>2 else []
  verbose= args[3] if len(args)>3 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))

  rcv= situation['receiver']

  t.ExecuteMotion('infer2', {},[rcv],'x')
  x_r= a_get(rcv,'x')

  #For data_base:
  InsertDict(situation, {'receiver_attr':{'x':x_r}})

  inferred_keys.append([CURR,'spread','z_avr'])  #Average of Z of the spreading area
  inferred_keys.append([CURR,'spread','z_max'])  #Max of Z of the spreading area
  inferred_keys.append([CURR,'spread','xy_start'])  #X,Y of the start point
  inferred_keys.append([CURR,'spread','xy_dir'])  #X,Y direction of the spreading pattern
  inferred_keys.append([CURR,'spread','key_points'])  #Key points of the spreading pattern
  inferred_keys.append([CURR,'spread','wave_modifier'])  #Modifying parameters for the spreading pattern
  inferred_keys.append([CURR,'spread','duration'])  #Duration of the pattern
  #NOTE: 'wave' is also inferred, but it is not added to inferred_keys
  # in order to avoid to be stored in data_base
  #[CURR,'spread','wave']  #X,Y spreading pattern

  #Container to store the variables derived from situation
  dsit= TContainer(debug=True)
  dsit.x_r= x_r

  #Edge points in local frame:
  if a_has(rcv,'l_p_pour_e_set'):
    dsit.l_p_rcv_set= a_get(rcv,'l_p_pour_e_set')
  else:
    return False

  #Edge points in torso frame:
  dsit.p_rcv_set= [Transform(dsit.x_r, p) for p in dsit.l_p_rcv_set]
  #X,Y of edge points in torso frame:
  dsit.xy_rcv_set= [[x,y] for (x,y,z) in dsit.p_rcv_set]
  #Average and max Z of edge points in torso frame:
  dsit.z_rcv_avr= sum([z for (x,y,z) in dsit.p_rcv_set])/float(len(dsit.p_rcv_set))
  dsit.z_rcv_max= max([z for (x,y,z) in dsit.p_rcv_set])

  start,u_dir,wave= PlanSpreadingWave(dsit.xy_rcv_set, x_step=0.05, start_search_factor=0.01, search_factor=0.9, eval_resolution=20, verbose=verbose)
  duration= MAX_TIME*float(len(wave.params))

  a_set(CURR,'spread','z_avr',  dsit.z_rcv_avr)
  a_set(CURR,'spread','z_max',  dsit.z_rcv_max)
  a_set(CURR,'spread','xy_start',  start)
  a_set(CURR,'spread','xy_dir',  u_dir)
  a_set(CURR,'spread','wave',  wave)
  a_set(CURR,'spread','key_points',  copy.deepcopy(wave.data))
  a_set(CURR,'spread','wave_modifier',  copy.deepcopy(wave.params))
  a_set(CURR,'spread','duration',  duration)

  ##Generate spreading trajectory
  #fp= file('/tmp/spread2.dat','w')
  #rot= np.array([[u_dir[0],-u_dir[1]],[u_dir[1],u_dir[0]]])
  #for t in FRange1(0.0,duration,500):
    #p= np.array(start) + np.dot(rot, wave.Evaluate(t))
    #fp.write(' '.join(map(str,p))+'\n')
  #fp.close()
  viz= TSimpleVisualizer(rospy.Duration(20.0), name_space='visualizer_spread')
  viz.viz_frame= t.robot.BaseFrame
  rot= np.array([[u_dir[0],-u_dir[1]],[u_dir[1],u_dir[0]]])
  for t in FRange1(0.0,duration,100):
    p= np.array(start) + np.dot(rot, wave.Evaluate(t))
    viz.AddSphere(list(p)+[dsit.z_rcv_max, 0,0,0,1], scale=[0.01]*3, rgb=viz.ICol(5), alpha=0.5)

  return True

