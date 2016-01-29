#!/usr/bin/python
#Numerical methods.
import os,time
from base_util import *

#Search [[f,x]*num] such that f=fobj(x) is not None and x is in bound
def InitialGuess(bound, fobj, num=1, max_count=None):
  assert(len(bound)==2)
  assert(len(bound[0])==len(bound[1]))
  xmin= bound[0]
  xmax= bound[1]
  dim= len(xmin)
  dx= [xmax[d]-xmin[d] for d in range(dim)]
  fx_data= []
  if num<=0:  return fx_data
  while max_count==None or max_count>0:
    x= [xmin[d]+dx[d]*random.random() for d in range(dim)]
    f= fobj(x)
    if f is not None:
      fx_data.append([f,x])
      if len(fx_data)>=num:  return fx_data
    if max_count is not None:  max_count-= 1
  return fx_data

'''Define a space.
min and max should be list or None. '''
class TSpaceDef(object):
  def __init__(self,dim=0,min=None,max=None):
    self.D= dim
    self.Min= min
    self.Max= max

  @property
  def Bounds(self):
    return [self.Min if self.Min is not None else [], self.Max if self.Max is not None else []]

#Concatenate spaces.
def CatSpaces(*spaces):
  res= TSpaceDef(dim=0, min=[], max=[])
  for sp in spaces:
    if sp.D>0:
      res.D+= sp.D
      res.Min+= sp.Min
      res.Max+= sp.Max
  return res

#Get an expanded bounds (Min,Max).
def ExpandBounds(space, f_scale, len_min):
  expanded= TSpaceDef(dim=space.D)
  if expanded.D==0:  return expanded
  mi,ma= space.Bounds
  if mi is None and ma is not None:  mi= ma
  elif mi is not None and ma is None:  ma= mi
  elif mi is None and ma is None:  mi= [0.0]*space.D; ma= [0.0]*space.D
  lens= [max(len_min, f_scale*(ma[d]-mi[d])) for d in range(space.D)]
  expanded.Min= [0.5*(ma[d]+mi[d]) - 0.5*lens[d] for d in range(space.D)]
  expanded.Max= [0.5*(ma[d]+mi[d]) + 0.5*lens[d] for d in range(space.D)]
  return expanded

#Maintain and learn a bounding box (bounds) of a D dimensional vector.
class TBoundingBox(TSpaceDef):
  def __init__(self, dim):
    TSpaceDef.__init__(self, dim, [None]*dim, [None]*dim)

  #Add a point: the bounding box is automatically adjusted so that it contains the point.
  def Add(self, p):
    for d in range(self.D):
      if self.Min[d]==None or p[d]<self.Min[d]:  self.Min[d]= p[d]
      if self.Max[d]==None or p[d]>self.Max[d]:  self.Max[d]= p[d]

  #Add another bounding box: the bounding box contains both the old one and a requested one.
  def AddBB(self, bb):
    self.Add(bb.Min)
    self.Add(bb.Max)

  #Return an expanded bounding box with a scaling factor f_scale.
  def GetExpanded(self, f_scale, len_min=0.0):
    #D= len(self.Min)
    #lens= [max(len_min, f_scale*(self.Max[d]-self.Min[d]) ) for d in range(D)]
    #bb= TBoundingBox(D)
    #bb.Min= [0.5*(self.Max[d]+self.Min[d]) - 0.5*lens[d] for d in range(D)]
    #bb.Max= [0.5*(self.Max[d]+self.Min[d]) + 0.5*lens[d] for d in range(D)]
    bb= ExpandBounds(self, f_scale, len_min)  #bb is a TSpaceDef
    #bb.__class__= TBoundingBox  #downcast
    return bb

#Do backtracing line search max f(x2=x+l*direc) where x2 satisfies is_in(x2)
def LineSearch(f,x0,direc,grad=None,l0=1.0,rho=0.5,eps=0.5,n_max=5,f0=None,is_in=None,constrain=None):
  l_min= l0*rho**n_max
  d= np.mat(direc)
  if grad!=None:  g= np.mat(grad)
  else:           g= d
  t= eps * (g.T * d)[0,0]
  x= np.mat(x0)
  l= l0
  if f0==None:  f0= f(x)
  if is_in==None:  is_in= lambda x:True
  if constrain==None:  constrain= lambda x:x
  #print 0.0, x.T, f(x)-f0, is_in(x), (f(x) - f0 >= l*t)
  if not is_in(constrain(x+l_min*d)):  return x
  while n_max>0:
    x2= constrain(x+l*d)
    #print l, x.T, d.T, x2.T, f(x2)-f0, is_in(x2), (f(x2) - f0 >= l*t)
    if is_in(x2) and f(x2) - f0 >= l*t:  return x2
    l*= rho
    n_max-=1
  return x

#Do golden line search
golden_ratio= (math.sqrt(5.0)-1.0)*0.5
def LineSearchG(f,x0,direc,l_max=1.0,n_max=5,tol=1.0e-3,constrain=None):  #is_in=None,
  #if is_in==None:  is_in= lambda x:True
  if constrain==None:  constrain= lambda x:x
  a= np.mat(x0)
  b= constrain(np.mat(x0)+np.mat(direc)*l_max)

  c= constrain(b - golden_ratio*(b-a))
  d= constrain(a + golden_ratio*(b-a))
  fc= f(c); fd= f(d)
  while la.norm(c-d)>tol and n_max>0:
    if fc>fd:
      b= d
      d= c
      c= constrain(b - golden_ratio*(b-a))
      fd=fc; fc=f(c)
    else:
      a= c
      c= d
      d= constrain(a + golden_ratio*(b-a))
      fc=fd; fd=f(d)
    n_max-= 1
  return (b+a)*0.5



'''First order gradient based optimizer (interface).'''
class TFirstOrderOptimizer(object):
  def __init__(self):
    pass

  #Return an initial internal state.
  def Init(self, param):
    return []

  #Progress the optimization one step, and return new parameter and internal state.
  def Step(self, param, grad, state):
    return param, state

class TGradientAscent(TFirstOrderOptimizer):
  def __init__(self, alpha=0.2, normalize_grad=False):
    self.alpha= alpha
    self.normalize_grad= normalize_grad
  #Progress the optimization one step, and return new parameter and internal state.
  def Step(self, param, grad, state):
    if self.normalize_grad:  grad= grad/la.norm(grad)
    param= param + self.alpha*grad
    return param, state

#ref. http://qiita.com/skitaoka/items/e6afbe238cd69c899b2a
# http://www.matthewzeiler.com/pubs/googleTR2012/googleTR2012.pdf
def StepAdaDeltaMax(x, g, r, s, rho, eps):
  r= rho*r + (1.0-rho)*g*g
  v= math.sqrt((s+eps)/(r+eps)) * g
  s= rho*s + (1.0-rho)*v*v
  x= x + v
  return x, r, s

StepAdaDeltaMaxV= np.vectorize(StepAdaDeltaMax)

class TAdaDeltaMax(TFirstOrderOptimizer):
  def __init__(self, rho=0.95, eps=1.0e-3, normalize_grad=False):
    self.rho= rho
    self.eps= eps
    self.normalize_grad= normalize_grad

  #Return an initial internal state.
  def Init(self, param):
    r= np.zeros_like(param)
    s= np.zeros_like(param)
    return (r,s)

  #Progress the optimization one step, and return new parameter and internal state.
  def Step(self, param, grad, state):
    r,s= state
    if self.normalize_grad:  grad= grad/la.norm(grad)
    param,r,s= StepAdaDeltaMaxV(param,grad,r,s, self.rho,self.eps)
    return param, (r,s)




#Helper for Taylor series expansion:

#Return a vector [x_0,...,x_D-1], x_d=0 (d!=i), x_i=h
def Delta1(D,i,h):
  delta= np.mat([0.0]*D).T
  delta[i]= h
  return delta

#Return a vector [x_0,...,x_D-1], x_d=0 (d!=i1,i2), x_i1=h1, x_i2=h2
def Delta2(D,i1,i2,h1,h2):
  delta= np.mat([0.0]*D).T
  if i1==i2:
    delta[i1]= h1+h2
  else:
    delta[i1]= h1
    delta[i2]= h2
  return delta

'''First order Taylor series expansion of f around x0.
  f(x) ~ a  +  b.T * (x-x0)
Returns a,b.
h: Used for numerical derivative computation.
'''
def TaylorExp1(f, x0, h=0.01, maxd1=5.0):
  x0= np.mat(x0)
  if x0.shape[0]==1:  x0= x0.T
  Dx= x0.shape[0]
  a= f(x0)
  b= np.mat([0.0]*Dx).T
  maxb= 0.0
  for d in range(Dx):
    b[d]= ( f(x0+Delta1(Dx,d,h)) - f(x0-Delta1(Dx,d,h)) ) / (2.0*h)
    if abs(b[d])>maxb:  maxb= abs(b[d])
  if maxb>maxd1:  b*= maxd1/maxb
  return a,b

'''Second order Taylor series expansion of f around x0.
  f(x) ~ a  +  b.T * (x-x0)  +  1/2 * (x-x0).T * c * (x-x0)
Returns a,b,c.
h: Used for numerical derivative computation.
'''
def TaylorExp2(f, x0, h=0.01, maxd1=5.0, maxd2=1.0):
  x0= np.mat(x0)
  if x0.shape[0]==1:  x0= x0.T
  Dx= x0.shape[0]
  a= f(x0)
  b= np.mat([0.0]*Dx).T
  maxb= 0.0
  for d in range(Dx):
    b[d]= ( f(x0+Delta1(Dx,d,h)) - f(x0-Delta1(Dx,d,h)) ) / (2.0*h)
    if abs(b[d])>maxb:  maxb= abs(b[d])
  if maxb>maxd1:  b*= maxd1/maxb
  h= 0.5*h
  c= np.mat([[0.0]*Dx]*Dx)
  maxc= 0.0
  for d1 in range(Dx):
    c[d1,d1]= ( f(x0+Delta1(Dx,d1,2.0*h)) - 2.0*f(x0) + f(x0-Delta1(Dx,d1,2.0*h)) ) / (4.0*h**2)
    if abs(c[d1,d1])>maxc:  maxc= abs(c[d1,d1])
    for d2 in range(d1+1,Dx):
      c[d1,d2]= ( f(x0+Delta2(Dx,d1,d2,h,h)) - f(x0+Delta2(Dx,d1,d2,h,-h)) - f(x0+Delta2(Dx,d1,d2,-h,h)) + f(x0-Delta2(Dx,d1,d2,h,h)) ) / (4.0*h**2)
      c[d2,d1]= c[d1,d2]
      if abs(c[d1,d2])>maxc:  maxc= abs(c[d1,d2])
  if maxc>maxd2:  c*= maxd2/maxc
  return a,b,c


#Exponential moving average filter for one-dimensional variable.
class TExpMovingAverage1(object):
  #mean: initial mean. If None, the first value is used.
  #init_sd: initial standard deviation.
  #alpha: weight of new value.
  def __init__(self, mean=None, init_sd=0.0, alpha=0.5):
    self.Mean= mean
    self.SqMean= None
    self.InitSD= init_sd
    self.Alpha= alpha
    self.sd_= None

  def Update(self, value):
    if self.Mean==None:  self.Mean= value
    else:  self.Mean= self.Alpha*value + (1.0-self.Alpha)*self.Mean
    if self.SqMean==None:  self.SqMean= self.InitSD*self.InitSD + self.Mean*self.Mean
    else:  self.SqMean= self.Alpha*(value*value) + (1.0-self.Alpha)*self.SqMean
    self.sd_= None

  @property
  def StdDev(self):
    if self.sd_==None:  self.sd_= math.sqrt(max(0.0,self.SqMean-self.Mean*self.Mean))
    return self.sd_



'''
Interface class of a function approximator.
We assume a data takes a form like:
  X=[[x1^T],  Y=[[y1^T],
     [x2^T],     [y2^T],
     [... ]]     [... ]]
where xn is an input vector, yn is an output vector (n=1,2,...).
'''
class TFunctionApprox(object):
  @staticmethod
  def DefaultOptions():
    Options= {}
    return Options
  @staticmethod
  def DefaultParams():
    Params= {}
    return Params

  #Number of samples
  @property
  def NSamples(self):
    return len(self.DataX)

  #Number of x-dimensions
  @property
  def Dx(self):
    return len(self.DataX[0]) if self.NSamples>0 else 0

  #Number of y-dimensions
  @property
  def Dy(self):
    return len(self.DataY[0]) if self.NSamples>0 else 0

  def __init__(self):
    self.Options= {}
    self.Params= {}
    self.Load(data={'options':self.DefaultOptions(), 'params':self.DefaultParams()})
    self.is_predictable= False
    self.load_base_dir= None

  #Load options and parameters from a dictionary.
  #base_dir: where external data file(s) are stored; None for a default value.
  #  Note: data may contain a filename like '{base}/func.dat'
  #        where {base} is supposed be replaced by base_dir.
  #        Use self.Locate to get the actual path (e.g. self.Locate('{base}/func.dat')).
  def Load(self, data=None, base_dir=None):
    if data!=None and 'options' in data: InsertDict(self.Options, data['options'])
    if data!=None and 'params' in data: InsertDict(self.Params, data['params'])
    self.load_base_dir= base_dir

  def Locate(self, filename):
    if filename.find('{base}')>=0 and self.load_base_dir==None:
      raise Exception('Use Load with specifying base_dir argument. Otherwise Locate() can not return the correct location for the filename: %s'%filename)
    return filename.format(base=self.load_base_dir)

  #Save options and parameters into a dictionary.
  #base_dir: used to store data into external data file(s); None for a default value.
  #  Note: returned dict may contain file path(s) containing data.
  #        Such path(s) my contain {base} which is actually base_dir.
  #        Those {base} will be replaced by base_dir when using Load().
  #        This is useful to move the data files and load them.
  def Save(self, base_dir=None):
    self.SyncParams(base_dir)
    data= {}
    data['options']= ToStdType(self.Options)
    data['params']= ToStdType(self.Params)
    return copy.deepcopy(data)

  #Synchronize Params (and maybe Options) with an internal learner to be saved.
  #base_dir: used to store data into external data file(s); None for a default value.
  def SyncParams(self, base_dir):
    pass

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return self.is_predictable

  #Initialize approximator.  Should be executed before Update/UpdateBatch.
  def Init(self):
    self.DataX= []
    self.DataY= []
    self.is_predictable= False

  #Incrementally update the internal parameters with a single I/O pair (x,y).
  #If x and/or y are None, only updating internal parameters is done.
  def Update(self, x=None, y=None, not_learn=False):
    if x!=None or y!=None:
      self.DataX.append(list(x))
      self.DataY.append(list(y))
    if not_learn:  return

  #Incrementally update the internal parameters with I/O data (X,Y).
  #If x and/or y are None, only updating internal parameters is done.
  def UpdateBatch(self, X=None, Y=None, not_learn=False):
    if X!=None or Y!=None:
      self.DataX.extend(X)
      self.DataY.extend(Y)
    if not_learn:  return

  #Prediction result class.
  class TPredRes:
    def __init__(self):
      self.Y= None  #Prediction result.
      self.Var= None  #Covariance matrix.
      self.Grad= None  #Gradient.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    raise Exception('FIXME: Implement')


#Dump function approximator (subclass of TFunctionApprox) to file for plot.
def DumpPlot(fa, f_reduce=lambda xa:xa, f_repair=lambda xa,mi,ma,me:xa, file_prefix='/tmp/f', x_var=0.0, n_div=50, bounds=None):
  #if len(fa.DataX)==0:  print 'DumpPlot: No data'; return
  if not fa.IsPredictable():  print 'DumpPlot: Not predictable'; return
  if bounds!=None:
    xamin0,xamax0= bounds
  else:
    xamin0= [min([x[d] for x in fa.DataX]) for d in range(fa.Dx)]
    xamax0= [max([x[d] for x in fa.DataX]) for d in range(fa.Dx)]
  xamin= f_reduce(xamin0)
  xamax= f_reduce(xamax0)
  xmed= [Median([x[d] for x in fa.DataX]) for d in range(fa.Dx)]
  if len(xamin)>=3 or len(xamin)!=len(xamax) or len(xamin)<=0:
    print 'DumpPlot: Invalid f_reduce function'
    return

  fp= open('%s_est.dat'%(file_prefix),'w')
  if len(xamin)==2:
    for xa1_1 in FRange1(xamin[0],xamax[0],n_div):
      for xa1_2 in FRange1(xamin[1],xamax[1],n_div):
        xa1r= [xa1_1,xa1_2]
        xa1= f_repair(xa1r, xamin0, xamax0, xmed)
        fp.write('%s\n' % ToStr(xa1r,xa1,ToList(fa.Predict(xa1,x_var).Y)))
      fp.write('\n')
  else:  #len(xamin)==1:
    for xa1_1 in FRange1(xamin[0],xamax[0],n_div):
      xa1r= [xa1_1]
      xa1= f_repair(xa1r, xamin0, xamax0, xmed)
      fp.write('%s\n' % ToStr(xa1r,xa1,ToList(fa.Predict(xa1,x_var).Y)))
  fp.close()
  fp= open('%s_smp.dat'%(file_prefix),'w')
  for xa1,x2 in zip(fa.DataX, fa.DataY):
    fp.write('%s\n' % ToStr(f_reduce(xa1),xa1,x2))
  fp.close()


'''Function y=F(x) object where dim(y)==0, x: n-dim vector. '''
class TZeroFunc(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    Options= {}
    return Options

  #Number of x-dimensions
  @property
  def Dx(self):
    return self.dx

  #Number of y-dimensions
  @property
  def Dy(self):
    return 0

  def __init__(self, dx):
    TFunctionApprox.__init__(self)
    self.dx= dx

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class do not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    y= np.zeros((0,1))
    dy= np.zeros((self.dx,0))
    var= np.zeros((0,0))
    res= self.TPredRes()
    res.Y= y
    if with_var:  res.Var= var
    if with_grad:  res.Grad= dy
    return res

'''For any function y=F(x), y: m-dim vector, x: n-dim vector, this gives E[dy/dx], E[y], cov[y].
We use a local linear form of F (dF(x)=dy/dx is given):
  E[y]= F(x)
  E[dy/dx]= dF(x)
  cov[y]= dF(x).T * cov[x] * dF(x)
If dF is None (not provided), we numerically compute it.
FdF is an alternative form of F and dF:
  y,dy/dx = FdF(x, with_grad=True)
  y = FdF(x, with_grad=False)'''
class TLocalLinear(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    Options= {}
    Options['h']= 0.01     #Numerical derivative window.
    Options['maxd1']= 5.0  #Maximum value of 1st order derivative.
    return Options

  #Number of x-dimensions
  @property
  def Dx(self):
    return self.dx

  #Number of y-dimensions
  @property
  def Dy(self):
    return self.dy

  def __init__(self, dx, dy, F=None, dF=None, FdF=None):
    TFunctionApprox.__init__(self)
    self.dx= dx
    self.dy= dy
    self.F= F
    if dF is not None:
      self.dF= dF
    else:
      #self.MF= lambda x:self.F(ToList(x))
      self.dF= lambda x:self.NumDeriv(x)
    self.FdF= FdF

  #Compute derivative at x numerically.
  def NumDeriv(self,x):
    h=self.Options['h']
    maxd1=self.Options['maxd1']
    delta= lambda dd: np.array([0.0 if d!=dd else h for d in range(self.Dx)])
    dy= np.zeros((self.Dx,self.Dy))
    for d in range(self.Dx):
      dy[d,:]= (np.array(self.F(x+delta(d))) - self.F(x-delta(d))).T/(2.0*h)
      maxd=abs(dy[d,:]).max()
      if maxd>maxd1:  dy[d,:]*= maxd1/maxd
    return dy

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class do not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    res= self.TPredRes()
    if self.FdF is None:
      res.Y= MCVec(self.F(x))
      if var_is_zero:
        if with_var:  res.Var= np.zeros((self.dy,self.dy))
        if with_grad: res.Grad= Mat(self.dF(x))
      else:  #i.e. not var_is_zero
        if with_var or with_grad:  grad= Mat(self.dF(x))
        if with_var:  res.Var= grad.T*x_var*grad
        if with_grad: res.Grad= grad
    else:  #with self.FdF
      if with_grad or (not var_is_zero and with_var):
        res.Y,grad= self.FdF(x,with_grad=True)
        grad= Mat(grad)
      else:
        res.Y= self.FdF(x,with_grad=False)
      if with_var:  res.Var= np.zeros((self.dy,self.dy)) if var_is_zero else grad.T*x_var*grad
      if with_grad: res.Grad= grad
    return res

'''For any function y=F(x), y: scalar (1-dim), x: n-dim vector, this gives E[dy/dx], E[y], var[y].
To obtain E[y] and var[y], we use a local quadratic form of F obtained by Taylor series expansion.'''
class TLocalQuad(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    Options= {}
    Options['h']= 0.01     #Numerical derivative window.
    Options['maxd1']= 5.0  #Maximum value of 1st order derivative.
    Options['maxd2']= 1.0  #Maximum value of 2nd order derivative.
    return Options

  #Number of x-dimensions
  @property
  def Dx(self):
    return self.dx

  #Number of y-dimensions
  @property
  def Dy(self):
    return 1  #This class works only when dim(y)==1.

  def __init__(self, dx, F):
    TFunctionApprox.__init__(self)
    self.dx= dx
    self.F= F
    self.MF= lambda x:self.F(ToList(x))

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class do not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    res= self.TPredRes()
    if var_is_zero:
      if with_var:  res.Var= np.mat([0.0])
      if not with_grad:
        res.Y= np.mat([self.F(x)])
      else:  #i.e. with_grad
        y,dy= TaylorExp1(self.MF, x, h=self.Options['h'], maxd1=self.Options['maxd1'])
        res.Y= np.mat([y])
        res.Grad= dy
    else:  #i.e. not var_is_zero
      y,dy,ddy= TaylorExp2(self.MF, x, h=self.Options['h'], maxd1=self.Options['maxd1'], maxd2=self.Options['maxd2'])
      #print 'TaylorExp2',TaylorExp2
      #print 'ddy,x_var',ddy,x_var
      #print '(ddy*x_var).trace()',(ddy*x_var).trace()
      #print 'self.F(x)',self.F(x)
      #print 'y',y
      #print 'np.mat([y])',np.mat([y])
      res.Y= np.mat([y]) + (ddy*x_var).trace()
      if with_var:  res.Var= 2.0*(ddy*x_var*ddy*x_var).trace() + dy.T*x_var*dy
      if with_grad:  res.Grad= dy
    return res

'''For a quadratic function y=F(x)=(x-a).T*W*(x-a),
y: scalar (1-dim), x,a: n-dim column vector, this gives E[dy/dx], E[y], var[y]. '''
class TQuadratic(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    Options= {}
    return Options

  #Number of x-dimensions
  @property
  def Dx(self):
    return self.dx

  #Number of y-dimensions
  @property
  def Dy(self):
    return 1  #This class works only when dim(y)==1.

  def F(self, x):
    dxa= MCVec(x)-self.a
    return dxa.T*self.W*dxa

  def dF(self, x):
    dxa= MCVec(x)-self.a
    return (self.W+self.W.T)*dxa

  def ddF(self, x):
    dxa= MCVec(x)-self.a
    return self.W+self.W.T

  def __init__(self, a, W):
    TFunctionApprox.__init__(self)
    if a is not None:
      self.dx= Len(a)
      self.a= MCVec(a)
    if isinstance(W, (float, np.float_, np.float16, np.float32, np.float64)):
      self.W= np.diag([W]*self.dx)
    elif W is not None:
      self.W= Mat(W)

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class do not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    res= self.TPredRes()
    if var_is_zero:
      if with_var:  res.Var= np.mat([0.0])
      if not with_grad:
        res.Y= self.F(x)
      else:  #i.e. with_grad
        res.Y= self.F(x)
        res.Grad= self.dF(x)
    else:  #i.e. not var_is_zero
      y,dy,ddy= self.F(x),self.dF(x),self.ddF(x)
      res.Y= np.mat([y]) + (ddy*x_var).trace()
      if with_var:  res.Var= 2.0*(ddy*x_var*ddy*x_var).trace() + dy.T*x_var*dy
      if with_grad:  res.Grad= dy
    return res


'''Another form of quadratic function: y=F(x)=(x[:D]-x[D:]).T*W*(x[:D]-x[D:]),
y: scalar (1-dim), x: D*2-dim column vector, this gives E[dy/dx], E[y], var[y]. '''
class TQuadratic2(TQuadratic):
  def F(self, x):
    dxa= MCVec(x[:self.D])-MCVec(x[self.D:])
    return dxa.T*self.W*dxa

  def dF(self, x):
    dxa= MCVec(x[:self.D])-MCVec(x[self.D:])
    dF1= (self.W+self.W.T)*dxa
    return np.bmat([[dF1],[-dF1]])

  def ddF(self, x):
    dxa= MCVec(x[:self.D])-MCVec(x[self.D:])
    ddF1= self.W+self.W.T
    return np.bmat([[ddF1,-ddF1],[-ddF1,ddF1]])

  def __init__(self, D, W):
    TQuadratic.__init__(self,a=[0.0]*D,W=W)
    self.D= D
    self.dx= 2*D
    self.a= None
