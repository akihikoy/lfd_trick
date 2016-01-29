#!/usr/bin/python
'''
Incremental version of LWR, locally weighted regression.
- Gaussian kernel with max norm is available.

FIXME:TODO: TLWR should inherit TFunctionApprox (see base_ml_dnn for example).
Use global DumpPlot instead of TLWR.DumpPlot
DEPRECATED: This issue is resolved in the newer version: base_ml_lwr2.py
'''
from base_util import *
from base_ml import *

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

class TLWR:
  def __init__(self,kernel='l2g'):
    self.AddOne= True  #Automatically add one to the end of x
    self.LazyCopy= False
    self.Importance= None
    if   kernel=='l2g':  #L2 norm Gaussian
      self.kernel= Gaussian
      self.dist= Dist
    elif kernel=='maxg':  #Max norm Gaussian
      self.kernel= GaussianM
      self.dist= DistM

  #Get weights W(x)=diag([k(x,x1),k(x,x2),...])
  def Weights(self,x,x_var):
    N= self.X.shape[0]
    D= self.X.shape[1] - (1 if self.AddOne else 0)
    W= np.diag([1.0]*N)
    for n in range(N):
      W[n,n]= self.kernel((x-self.X[n])[:,:D], np.array([self.C[n]*self.C[n]]*D)+x_var)
    if self.Importance!=None:
      for k,v in self.Importance.iteritems():
        W[k,k]*= v
    return W

  #Whether prediction is available (False if the model is not learned at all).
  def Available(self):
    return len(self.DataX)>=2

  #Train with data X,Y.
  #   [x1^T]    [y1^T]
  # X=[x2^T]  Y=[y2^T]
  #   [... ]    [... ]
  # c: Gaussian kernel width.
  # f_reg: Reguralization factor.
  def Train(self,X,Y,c_min=0.01,f_reg=0.01):
    self.DataX= X
    self.DataY= Y
    self.C= self.AutoWidth(c_min)
    self.FReg= f_reg
    self.LazyCopy= True

  #Initialization for incremental learning
  def Init(self,c_min=0.01, c_max=1.0e6, c_gain=0.7, f_reg=0.01):
    self.DataX= []
    self.DataY= []
    self.C= []
    self.Closests= []
    self.CDists= []  #Distance to the closest point
    self.CMin= c_min
    self.CMax= c_max
    self.CGain= c_gain
    self.FReg= f_reg
    self.LazyCopy= False

  #Incrementally update the internal parameters
  def Update(self,x,y):
    #FIXME: it's better to use an efficient nearest neighbor like KD tree.
    self.DataX.append(list(x))
    self.DataY.append(list(y))
    if len(self.DataX)==1:
      self.Closests.append(-1)
      self.C.append(None)
      self.CDists.append(None)
      return
    dist_to_c= lambda dist: min( max(self.CMin, self.CGain*dist), self.CMax )
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
    self.LazyCopy= True

  #Prediction result class.
  class TPredRes:
    def __init__(self):
      self.Y= None  #Prediction result.
      self.Var= None  #Variance matrix.
      self.Grad= None  #Gradient.
  #Do prediction.
  # Return a TPredRes instance.
  # x_var: Variance of x (TEST).
  # with_var: Whether compute a variance matrix of error at the query point as well.
  # with_grad: Whether compute a gradient at the query point as well.
  def Predict(self,x,x_var=0.0,with_var=False,with_grad=False):
    if self.LazyCopy:
      self.X= Mat(AddOnes(copy.deepcopy(self.DataX)) if self.AddOne else self.DataX)
      self.Y= Mat(self.DataY)
      self.LazyCopy= False
    self.DiffQueryX= False
    if self.DiffQueryX:
      subx= [x[d] for d in range(len(x))] + ([0.0] if self.AddOne else [])
      X= self.X - subx
    else:
      X= self.X
    xx= AddOne(x) if self.AddOne else x
    res= self.TPredRes()
    D= X.shape[1]  #Num of dimensions of x
    W= self.Weights(xx,x_var)
    beta= (X.T*(W*X) + self.FReg*np.eye(D,D)).I * (X.T*(W*self.Y))
    if self.DiffQueryX:
      res.Y= beta[-1].T
    else:
      res.Y= (xx * beta).T
    if with_var:
      N= X.shape[0]  #Num of samples
      div= W.trace()
      div*= (1.0 - float(D)/float(N)) if N>D else 1.0e-4
      Err= X * beta - self.Y
      div= max(div,1.0e-4)
      res.Var= (Err.T * W * Err) / div
    if with_grad:
      res.Grad= beta[:-1]
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

  #Dump data to file for plot.
  def DumpPlot(self,bounds,f_reduce,f_repair,file_prefix='/tmp/f',x_var=0.0):
    if len(self.DataX)==0:
      print 'No data'
      return
    xamin0,xamax0= bounds
    xamin= f_reduce(xamin0)
    xamax= f_reduce(xamax0)
    xmed= [Median([x[d] for x in self.DataX]) for d in range(len(self.DataX[0]))]
    if len(xamin)>=3 or len(xamin)!=len(xamax) or len(xamin)<=0:
      print 'DumpPlot: Invalid f_reduce function'
      return

    fp= OpenW('%s_est.dat'%(file_prefix),'w')
    if len(xamin)==2:
      for xa1_1 in FRange1(xamin[0],xamax[0],50):
        for xa1_2 in FRange1(xamin[1],xamax[1],50):
          xa1r= [xa1_1,xa1_2]
          xa1= f_repair(xa1r, xamin0, xamax0, xmed)
          fp.write('%s\n' % ToStr(xa1r,xa1,ToList(self.Predict(xa1,x_var).Y)))
        fp.write('\n')
    else:  #len(xamin)==1:
      for xa1_1 in FRange1(xamin[0],xamax[0],50):
        xa1r= [xa1_1]
        xa1= f_repair(xa1r, xamin0, xamax0, xmed)
        fp.write('%s\n' % ToStr(xa1r,xa1,ToList(self.Predict(xa1,x_var).Y)))
    fp.close()
    fp= OpenW('%s_smp.dat'%(file_prefix),'w')
    for xa1,x2 in zip(self.DataX, self.DataY):
      fp.write('%s\n' % ToStr(f_reduce(xa1),xa1,x2))
    fp.close()

