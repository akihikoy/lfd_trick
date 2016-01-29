#!/usr/bin/python
'''
Incremental version of LWR, locally weighted regression.
- Gaussian kernel with max norm is available.
- Subclass of TFunctionApprox.
'''
from base_util import *
from base_ml import *
import six.moves.cPickle as pickle

#Gaussian function
def Gaussian(xd, var, a=1.0):
  assert(xd.shape[0]==1)
  return a * np.exp(-0.5*xd / var * xd.T)

#Gaussian function with max norm
def GaussianM(xd, var, a=1.0):
  assert(xd.shape[0]==1)
  return a * np.exp(-0.5*(np.multiply(xd,xd)/var).max())  #Max norm

def AddOne(x):
  if isinstance(x,list):
    if len(x)==0:  return [1.0]
    if isinstance(x[0],list): return [x[0]+[1.0]]
    return x+[1.0]
  if isinstance(x,np.ndarray):
    return x.tolist()+[1.0]
  if x.shape[0]==1:  return Mat(x.tolist()[0]+[1.0])
  if x.shape[1]==1:  return Mat(x.T.tolist()[0]+[1.0]).T
  return None

def AddOnes(X):
  if isinstance(X,list):  Y= X
  else:  Y= X.tolist()
  for x in Y:
    x.append(1.0)
  return Y

class TLWR(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    Options= {}
    '''Kernel type.
      'l2g': Standard Gaussian with L2 norm.
      'maxg': Gaussian with max norm. '''
    Options['kernel']= 'l2g'

    Options['add_one']= True  #Whether automatically stack one to x for learning constant component.
    Options['c_min']= 0.01   #Minimum kernel width.
    Options['c_max']= 1.0e6  #Maximum kernel width.
    Options['c_gain']= 0.7   #Kernel width = c_gain * distance-to-closest-point.
    Options['f_reg']= 0.01   #Regularization parameter in weight inverse.

    Options['diff_query_x']= False  #Shift the sample x with a query x.

    Options['base_dir']= '/tmp/lwr/'  #Base directory.  Last '/' is matter.
    '''Some data (DataX, DataY)
        are saved into this file name when Save() is executed.
        label: 'data_x', or 'data_y'.
        base: Options['base_dir'] or base_dir argument of Save method.'''
    Options['data_file_name']= '{base}lwr_{label}.dat'
    return Options
  @staticmethod
  def DefaultParams():
    Params= {}
    Params['C']= None
    Params['Closests']= None
    Params['CDists']= None  #Distance to the closest point
    Params['data_x']= None
    Params['data_y']= None
    return Params

  def __init__(self):
    TFunctionApprox.__init__(self)
    self.Importance= None  #Weight modifier given externally.  Map of {index:weight}.

  #Synchronize Params (and maybe Options) with an internal learner to be saved.
  #base_dir: used to store data into external data file(s); None for a default value.
  def SyncParams(self, base_dir):
    TFunctionApprox.SyncParams(self, base_dir)
    if base_dir==None:  base_dir= self.Options['base_dir']
    L= lambda f: f.format(base=base_dir)
    if self.NSamples>0:
      self.Params['C']= ToStdType(self.C)
      self.Params['Closests']= ToStdType(self.Closests)
      self.Params['CDists']= ToStdType(self.CDists)

      self.Params['data_x']= self.Options['data_file_name'].format(label='data_x',base='{base}')
      #fp= OpenW(L(self.Params['data_x']), 'w')
      #for x in self.DataX:
        #fp.write('%s\n'%(' '.join(map(str,x))))
      #fp.close()
      pickle.dump(ToStdType(self.DataX), OpenW(L(self.Params['data_x']), 'wb'), -1)

      self.Params['data_y']= self.Options['data_file_name'].format(label='data_y',base='{base}')
      #fp= OpenW(L(self.Params['data_y']), 'w')
      #for y in self.DataY:
        #fp.write('%s\n'%(' '.join(map(str,y))))
      #fp.close()
      pickle.dump(ToStdType(self.DataY), OpenW(L(self.Params['data_y']), 'wb'), -1)

  #Initialize approximator.  Should be executed before Update/UpdateBatch.
  def Init(self):
    TFunctionApprox.Init(self)
    L= self.Locate
    if self.Params['data_x'] != None:
      self.DataX= pickle.load(open(L(self.Params['data_x']), 'rb'))
    if self.Params['data_y'] != None:
      self.DataY= pickle.load(open(L(self.Params['data_y']), 'rb'))

    self.C= []
    self.Closests= []
    self.CDists= []  #Distance to the closest point

    if self.Params['C'] != None:
      self.C= copy.deepcopy(self.Params['C'])
    if self.Params['Closests'] != None:
      self.Closests= copy.deepcopy(self.Params['Closests'])
    if self.Params['CDists'] != None:
      self.CDists= copy.deepcopy(self.Params['CDists'])

    if self.Options['kernel']=='l2g':  #L2 norm Gaussian
      self.kernel= Gaussian
      self.dist= Dist
    elif self.Options['kernel']=='maxg':  #Max norm Gaussian
      self.kernel= GaussianM
      self.dist= DistM
    else:
      raise Exception('Undefined kernel type:',self.Options['kernel'])

    self.lazy_copy= True  #Assign True when DataX or DataY is updated.
    self.CheckPredictability()


  #Update self.is_predictable.
  def CheckPredictability(self):
    self.is_predictable= (len(self.DataX)>=2)

  #Get weights W(x)=diag([k(x,x1),k(x,x2),...])
  def Weights(self,x,x_var):
    N= self.X.shape[0]
    D= self.X.shape[1] - (1 if self.Options['add_one'] else 0)
    W= np.diag([1.0]*N)
    for n in range(N):
      W[n,n]= self.kernel((x-self.X[n])[:,:D], np.array([self.C[n]*self.C[n]]*D)+x_var)
    if self.Importance!=None:
      for k,v in self.Importance.iteritems():
        W[k,k]*= v
    return W


  #Incrementally update the internal parameters with a single I/O pair (x,y).
  #If x and/or y are None, only updating internal parameters is done.
  def Update(self, x=None, y=None, not_learn=False):
    TFunctionApprox.Update(self, x, y, not_learn)
    #FIXME: it's better to use an efficient nearest neighbor like KD tree.
    if len(self.DataX)==1:
      self.Closests.append(-1)
      self.C.append(None)
      self.CDists.append(None)
      return
    dist_to_c= lambda dist: min( max(self.Options['c_min'], self.Options['c_gain']*dist), self.Options['c_max'] )
    if len(self.Closests)==1:
      self.Closests.append(0)
      self.Closests[0]= 1
      self.CDists= [self.dist(self.DataX[0],self.DataX[1])]*2
      self.C= [dist_to_c(self.CDists[0])]*2
    else:
      n= len(self.Closests)
      dc= 1.0e100
      nc= None #Closest point
      for k in range(n):
        d= self.dist(x,self.DataX[k])
        if d<dc:
          dc=d
          nc=k
        if d<self.CDists[k]:
          self.Closests[k]= n
          self.CDists[k]= d
          self.C[k]= dist_to_c(d)
      self.Closests.append(nc)
      self.CDists.append(dc)
      self.C.append(dist_to_c(dc))
      #for k in range(n+1):
        #self.C[k]= min(self.C[k], self.C[self.Closests[k]])
    self.lazy_copy= True
    self.CheckPredictability()

  #Incrementally update the internal parameters with I/O data (X,Y).
  #If x and/or y are None, only updating internal parameters is done.
  def UpdateBatch(self, X=None, Y=None, not_learn=False):
    TFunctionApprox.UpdateBatch(self, X, Y, not_learn)
    for x,y in zip(X, Y):
      self.Update(x,y,not_learn)
    #self.C= self.AutoWidth(c_min)
    self.lazy_copy= True
    self.CheckPredictability()

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    if self.lazy_copy:
      self.X= Mat(AddOnes(copy.deepcopy(self.DataX)) if self.Options['add_one'] else self.DataX)
      self.Y= Mat(self.DataY)
      self.lazy_copy= False
    if self.Options['diff_query_x']:
      subx= [x[d] for d in range(len(x))] + ([0.0] if self.Options['add_one'] else [])
      X= self.X - subx
    else:
      X= self.X
    xx= AddOne(x) if self.Options['add_one'] else x
    x_var, var_is_zero= RegularizeCov(x_var, len(x))
    res= self.TPredRes()
    D= X.shape[1]  #Num of dimensions of x
    W= self.Weights(xx,np.diag(x_var))
    beta= (X.T*(W*X) + self.Options['f_reg']*np.eye(D,D)).I * (X.T*(W*self.Y))
    if self.Options['diff_query_x']:
      res.Y= beta[-1].T
    else:
      res.Y= (xx * beta).T
    grad= beta[:-1] if self.Options['add_one'] else beta
    if with_var:
      N= X.shape[0]  #Num of samples
      div= W.trace()
      div*= (1.0 - float(D)/float(N)) if N>D else 1.0e-4
      Err= X * beta - self.Y
      div= max(div,1.0e-4)
      res.Var= (Err.T * W * Err) / div  #Covariance of prediction error.
      res.Var+= grad.T * x_var * grad  #Covariance propagated from input.
    if with_grad:
      res.Grad= grad
      #res.Grad= self.NumDeriv(x,x_var)
    return res

  #Compute derivative at x numerically.
  def NumDeriv(self,x,x_var=0.0,h=0.01):
    Dx= Len(x)
    Dy= Len(self.DataY[0])
    delta= lambda dd: np.array([0.0 if d!=dd else h for d in range(Dx)])
    dy= Mat([[0.0]*Dy]*Dx)
    for d in range(Dx):
      dy[d,:]= (self.Predict(x+delta(d),x_var).Y - self.Predict(x-delta(d),x_var).Y).T/(2.0*h)
      maxd=abs(dy[d,:]).max()
      if maxd>1.0:  dy[d,:]*= 1.0/maxd
    return dy

  #Compute Gaussian kernel width for each data point automatically.
  def AutoWidth(self, c_min=0.01, c_max=1.0e6, c_gain=0.7):
    N= len(self.DataX)
    C= [0.0]*N
    for n in range(N):
      C[n]= max( c_min, min([c_gain*self.dist(self.DataX[n],self.DataX[d]) if d!=n else c_max for d in range(N)]) )
    #print C
    return C


