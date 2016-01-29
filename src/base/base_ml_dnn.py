#!/usr/bin/python
'''
Deep neural networks for regression using chainer.
'''
from base_util import *
from base_ml import *
from base_chn import loss_for_error2

from chainer import cuda, Variable, FunctionSet, optimizers
import chainer.functions  as F
import six.moves.cPickle as pickle


#Tools for NNs

#ReLU whose input is a normal distribution variable.
#  mu: mean, var: variance (square of std-dev).
#  cut_sd: if abs(mu)>cut_sd*sigma, an approximation is used.  Set None to disable this.
def ReLUGauss(mu, var, epsilon=1.0e-6, cut_sd=4.0):
  cast= type(mu)
  sigma= math.sqrt(var)
  if sigma<epsilon:  return cast(max(0.0,mu)), cast(0.0)
  #Approximation to speedup for abs(mu)>cut_sd*sigma.
  if cut_sd!=None and mu>cut_sd*sigma:   return cast(mu), cast(var)
  if cut_sd!=None and mu<-cut_sd*sigma:  return cast(0.0), cast(0.0)
  sqrt2= math.sqrt(2.0)
  sqrt2pi= math.sqrt(2.0*math.pi)
  z= mu/(sqrt2*sigma)
  E= math.erf(z)
  X= math.exp(-z*z)
  mu_out= sigma/sqrt2pi*X + mu/2.0*(1.0+E)
  var_out= (1.0+E)/4.0*(mu*mu*(1.0-E)+2.0*var) - sigma*X/sqrt2pi*(sigma*X/sqrt2pi+mu*E)
  if var_out<0.0:
    if var_out>-epsilon:  return mu_out, 0.0
    else:
      msg= 'ERROR in ReLUGauss: %f, %f, %f, %f'%(mu, sigma, mu_out, var_out)
      print msg
      raise Exception(msg)
  return cast(mu_out), cast(var_out)

#Vector version of ReLUGauss
ReLUGaussV= np.vectorize(ReLUGauss)

#Gradient of ReLU whose input is a normal distribution variable.
#  mu: mean, var: variance (square of std-dev).
#  cut_sd: if abs(mu)>cut_sd*sigma, an approximation is used.  Set None to disable this.
def ReLUGaussGrad(mu, var, epsilon=1.0e-6, cut_sd=4.0):
  cast= type(mu)
  sigma= math.sqrt(var)
  if sigma<epsilon:  return cast(1.0 if mu>0.0 else 0.0)
  #Approximation to speedup for abs(mu)>cut_sd*sigma.
  if cut_sd!=None and mu>cut_sd*sigma:   return cast(1.0)
  if cut_sd!=None and mu<-cut_sd*sigma:  return cast(0.0)
  sqrt2= math.sqrt(2.0)
  z= mu/(sqrt2*sigma)
  return cast(0.5*(1.0+math.erf(z)))

ReLUGaussGradV= np.vectorize(ReLUGaussGrad)  #Vector version



'''
Interface class of a function approximator.
We assume a data takes a form like:
  X=[[x1^T],  Y=[[y1^T],
     [x2^T],     [y2^T],
     [... ]]     [... ]]
where xn is an input vector, yn is an output vector (n=1,2,...).
'''
class TNNRegression(TFunctionApprox):
  @staticmethod
  def DefaultOptions():
    '''Some options are defined both for a mean model and an error model (_err).
        If the value of an error model option is None,
        the value of corresponding mean model is used.'''
    Options= {}
    Options['name']= '' #Arbitrary name.
    Options['gpu']= -1  #Device ID of GPU (-1: not use).
    Options['n_units']= [1,200,200,1]  #Number of input/hidden/output units.
    Options['n_units_err']= None  #Number of input/hidden/output units for error model.

    Options['num_min_predictable']= 3  #Number of minimum samples necessary to train NNs.

    Options['init_bias_randomly']= True  #Initialize bias of linear models randomly.
    Options['bias_rand_init_bound']= [-1.0, 1.0]  #Bound used in random bias initialization.
    Options['bias_rand_init_bound_err']= [0.0, 0.5]  #Bound used in random bias initialization of error model.

    Options['dropout']= True  #If use dropout.
    Options['dropout_ratio']= 0.01  #Ratio of dropout.
    Options['dropout_err']= None  #If use dropout for error model.
    Options['dropout_ratio_err']= None  #Ratio of dropout for error model.

    Options['error_loss_neg_weight']= 0.1  #Weight of negative loss for error model.

    Options['AdaDelta_rho']= 0.9  #Parameter for AdaDelta.
    Options['AdaDelta_rho_err']= None  #Parameter for AdaDelta for error model.

    Options['batchsize']= 10  #Size of mini-batch.
    Options['batchsize_err']= None  #Size of mini-batch for error model.
    Options['num_max_update']= 5000  #Maximum number of updates with mini-batch.
    Options['num_max_update_err']= None  #Maximum number of updates with mini-batch for error model.
    Options['num_check_stop']= 50  #Stop condition is checked for every this number of updates w mini-batch.
    Options['num_check_stop_err']= None  #Stop condition is checked for every this number of updates w mini-batch (for error model).
    Options['loss_maf_alpha']= 0.4  #Update ratio of moving average filter for loss.
    Options['loss_maf_alpha_err']= None  #Update ratio of moving average filter for loss for error model.
    Options['loss_stddev_init']= 2.0  #Initial value of loss std-dev (unit (1.0) is 'loss_stddev_stop').
    Options['loss_stddev_init_err']= None  #Initial value of loss std-dev (unit (1.0) is 'loss_stddev_stop') for error model.
    Options['loss_stddev_stop']= 1.0e-3  #If std-dev of loss is smaller than this value, iteration stops.
    Options['loss_stddev_stop_err']= None  #If std-dev of loss is smaller than this value, iteration stops (for error model).

    Options['base_dir']= '/tmp/dnn/'  #Base directory.  Last '/' is matter.
    '''Some data (model.parameters, model_err.parameters, DataX, DataY)
        are saved into this file name when Save() is executed.
        label: 'model_mean', 'model_err', 'data_x', or 'data_y'.
        base: Options['base_dir'] or base_dir argument of Save method.'''
    Options['data_file_name']= '{base}nn_{label}.dat'
    '''Template of filename to store the training log.
        name: Options['name'].
        n: number of training executions.
        code: 'mean' or 'err'.
        base: Options['base_dir'].'''
    Options['train_log_file']= '{base}train/nn_log-{n:05d}-{name}{code}.dat'

    Options['verbose']= True

    return Options
  @staticmethod
  def DefaultParams():
    Params= {}
    Params['nn_params']= None
    Params['nn_params_err']= None
    Params['nn_data_x']= None
    Params['nn_data_y']= None
    Params['num_train']= 0  #Number of training executions.
    return Params

  @staticmethod
  def ToVec(x):
    if x==None:  return np.array([],np.float32)
    elif isinstance(x,list):  return np.array(x,np.float32)
    elif isinstance(x,(np.ndarray,np.matrix)):
      return x.ravel().astype(np.float32)
    raise Exception('ToVec: Impossible to serialize:',x)

  def __init__(self):
    TFunctionApprox.__init__(self)

  '''
  NOTE
  In order to save and load model parameters,
    save: p=ToStdType(model.parameters)
    load: model.copy_parameters_from(map(lambda e:np.array(e,np.float32),p))
  '''

  #Synchronize Params (and maybe Options) with an internal learner to be saved.
  #base_dir: used to store data into external data file(s); None for a default value.
  def SyncParams(self, base_dir):
    TFunctionApprox.SyncParams(self, base_dir)
    if base_dir==None:  base_dir= self.Options['base_dir']
    L= lambda f: f.format(base=base_dir)
    if self.IsPredictable():
      #self.Params['nn_params']= ToStdType(self.model.parameters)
      #self.Params['nn_params_err']= ToStdType(self.model_err.parameters)
      self.Params['nn_params']= self.Options['data_file_name'].format(label='model_mean',base='{base}')
      pickle.dump(ToStdType(self.model.parameters), OpenW(L(self.Params['nn_params']), 'wb'), -1)
      self.Params['nn_params_err']= self.Options['data_file_name'].format(label='model_err',base='{base}')
      pickle.dump(ToStdType(self.model_err.parameters), OpenW(L(self.Params['nn_params_err']), 'wb'), -1)

    if self.NSamples>0:
      self.Params['nn_data_x']= self.Options['data_file_name'].format(label='data_x',base='{base}')
      #fp= OpenW(L(self.Params['nn_data_x']), 'w')
      #for x in self.DataX:
        #fp.write('%s\n'%(' '.join(map(str,x))))
      #fp.close()
      pickle.dump(ToStdType(self.DataX), OpenW(L(self.Params['nn_data_x']), 'wb'), -1)

      self.Params['nn_data_y']= self.Options['data_file_name'].format(label='data_y',base='{base}')
      #fp= OpenW(L(self.Params['nn_data_y']), 'w')
      #for y in self.DataY:
        #fp.write('%s\n'%(' '.join(map(str,y))))
      #fp.close()
      pickle.dump(ToStdType(self.DataY), OpenW(L(self.Params['nn_data_y']), 'wb'), -1)

  #Initialize approximator.  Should be executed before Update/UpdateBatch.
  def Init(self):
    TFunctionApprox.Init(self)
    L= self.Locate
    if self.Params['nn_data_x'] != None:
      self.DataX= np.array(pickle.load(open(L(self.Params['nn_data_x']), 'rb')), np.float32)
    else:
      self.DataX= np.array([],np.float32)
    if self.Params['nn_data_y'] != None:
      self.DataY= np.array(pickle.load(open(L(self.Params['nn_data_y']), 'rb')), np.float32)
    else:
      self.DataY= np.array([],np.float32)

    self.CreateNNs()

    if self.Params['nn_params'] != None:
      #self.model.copy_parameters_from(map(lambda e:np.array(e,np.float32),self.Params['nn_params']))
      self.model.copy_parameters_from(map(lambda e:np.array(e,np.float32),pickle.load(open(L(self.Params['nn_params']), 'rb')) ))
      self.is_predictable= True
    else:
      if self.Options['init_bias_randomly']:
        self.InitBias(m='mean')

    if self.Params['nn_params_err'] != None:
      #self.model_err.copy_parameters_from(map(lambda e:np.array(e,np.float32),self.Params['nn_params_err']))
      self.model_err.copy_parameters_from(map(lambda e:np.array(e,np.float32),pickle.load(open(L(self.Params['nn_params_err']), 'rb')) ))
    else:
      if self.Options['init_bias_randomly']:
        self.InitBias(m='error')

    if self.Options['gpu'] >= 0:
      cuda.init(self.Options['gpu'])
      self.model.to_gpu()
      self.model_err.to_gpu()

    self.optimizer= optimizers.AdaDelta(rho=self.Options['AdaDelta_rho'])
    self.optimizer.setup(self.model.collect_parameters())
    self.optimizer_err= optimizers.AdaDelta(rho=IfNone(self.Options['AdaDelta_rho_err'], self.Options['AdaDelta_rho']))
    self.optimizer_err.setup(self.model_err.collect_parameters())

  #Create neural networks.
  def CreateNNs(self):
    assert(len(self.Options['n_units'])>=2)
    assert(self.Options['n_units_err']==None or len(self.Options['n_units_err'])>=2)
    #Mean model
    n_units= self.Options['n_units']
    self.f_names= ['l%d'%i for i in range(len(n_units)-1)]
    funcs= {}
    for i in range(len(n_units)-1):
      funcs[self.f_names[i]]= F.Linear(n_units[i],n_units[i+1])
    self.model= FunctionSet(**funcs)
    #Error model
    if self.Options['n_units_err']!=None:  n_units= self.Options['n_units_err']
    self.f_names_err= ['l%d'%i for i in range(len(n_units)-1)]
    funcs= {}
    for i in range(len(n_units)-1):
      funcs[self.f_names_err[i]]= F.Linear(n_units[i],n_units[i+1])
    self.model_err= FunctionSet(**funcs)

  #Randomly initialize bias of linear models.
  def InitBias(self, m='both'):
    if m in ('both','mean'):
      for l in self.f_names:
        getattr(self.model,l).b[:]= [Rand(*self.Options['bias_rand_init_bound'])
                                     for d in range(getattr(self.model,l).b.size)]
    if m in ('both','error'):
      for l in self.f_names_err:
        getattr(self.model_err,l).b[:]= [Rand(*self.Options['bias_rand_init_bound_err'])
                                         for d in range(getattr(self.model_err,l).b.size)]


  #Compute output (mean) for a set of x.
  def Forward(self, x_data, train):
    if not self.Options['dropout']:  train= False
    dratio= self.Options['dropout_ratio']
    x= Variable(x_data)
    h0= x
    for l in self.f_names[:-1]:
      h1= F.dropout(F.relu(getattr(self.model,l)(h0)), ratio=dratio, train=train)
      h0= h1
    y= getattr(self.model,self.f_names[-1])(h0)
    return y

  #Compute output (mean) and loss for sets of x and y.
  def FwdLoss(self, x_data, y_data, train):
    y= self.Forward(x_data, train)
    t= Variable(y_data)
    return F.mean_squared_error(y, t), y

  #Compute output (error) for a set of x.
  def ForwardErr(self, x_data, train):
    dropout= IfNone(self.Options['dropout_err'], self.Options['dropout'])
    if not dropout:  train= False
    dratio= IfNone(self.Options['dropout_ratio_err'], self.Options['dropout_ratio'])
    x= Variable(x_data)
    h0= x
    for l in self.f_names_err[:-1]:
      h1= F.dropout(F.relu(getattr(self.model_err,l)(h0)), ratio=dratio, train=train)
      h0= h1
    y= getattr(self.model_err,self.f_names_err[-1])(h0)
    return y

  #Compute output (error) and loss for sets of x and y.
  def FwdLossErr(self, x_data, y_data, train):
    y= self.ForwardErr(x_data, train)
    t= Variable(y_data)
    return loss_for_error2(y, t, self.Options['error_loss_neg_weight']), y


  #Forward computation of neural net considering input distribution.
  def ForwardX(self, x, x_var=None, with_var=False, with_grad=False):
    zero= np.float32(0)
    x= np.array(x,np.float32); x= x.reshape(x.size,1)

    #Error model:
    if with_var:
      h0= x
      for ln in self.f_names_err[:-1]:
        l= getattr(self.model_err,ln)
        hl1= l.W.dot(h0) + l.b.reshape(l.b.size,1)  #W h0 + b
        h1= np.maximum(zero, hl1)  #ReLU(hl1)
        h0= h1
      l= getattr(self.model_err,self.f_names_err[-1])
      y_err0= l.W.dot(h0) + l.b.reshape(l.b.size,1)
      y_var0= np.diag((y_err0*y_err0).ravel())
    else:
      y_var0= None

    x_var, var_is_zero= RegularizeCov(x_var, x.size, np.float32)
    if var_is_zero:
      g= None  #Gradient
      h0= x
      for ln in self.f_names[:-1]:
        l= getattr(self.model,ln)
        hl1= l.W.dot(h0) + l.b.reshape(l.b.size,1)  #W h0 + b
        h1= np.maximum(zero, hl1)  #ReLU(hl1)
        if with_grad:
          g2= l.W.T.dot(np.diag((hl1>0.0).ravel().astype(np.float32)))  #W diag(step(hl1))
          g= g2 if g==None else g.dot(g2)
        h0= h1
      l= getattr(self.model,self.f_names[-1])
      y= l.W.dot(h0) + l.b.reshape(l.b.size,1)
      if with_grad:
        g= g2 if g==None else g.dot(l.W.T)
      return y, y_var0, g

    else:
      g= None  #Gradient
      h0= x
      h0_var= x_var
      for ln in self.f_names[:-1]:
        l= getattr(self.model,ln)
        hl1= l.W.dot(h0) + l.b.reshape(l.b.size,1)  #W h0 + b
        hl1_dvar= np.diag( l.W.dot(h0_var.dot(l.W.T)) ).reshape(hl1.size,1)  #diag(W h0_var W^T)
        h1,h1_dvar= ReLUGaussV(hl1,hl1_dvar)  #ReLU_gauss(hl1,hl1_dvar)
        h1_var= np.diag(h1_dvar.ravel())  #To a full matrix
        if with_grad:
          g2= l.W.T.dot(np.diag(ReLUGaussGradV(hl1,hl1_dvar).ravel()))
          g= g2 if g==None else g.dot(g2)
        h0= h1
        h0_var= h1_var
      l= getattr(self.model,self.f_names[-1])
      y= l.W.dot(h0) + l.b.reshape(l.b.size,1)
      y_var= None
      if with_var:
        y_var= l.W.dot(h0_var.dot(l.W.T)) + y_var0
      if with_grad:
        g= g2 if g==None else g.dot(l.W.T)
      return y, y_var, g

  #Training code common for mean model and error model.
  @staticmethod
  def TrainNN(**opt):
    N= len(opt['x_train'])
    loss_maf= TExpMovingAverage1(init_sd=opt['loss_stddev_init']*opt['loss_stddev_stop'],
                                 alpha=opt['loss_maf_alpha'])
    batchsize= min(opt['batchsize'], N)  #Adjust mini-batch size for too small N
    num_max_update= opt['num_max_update']
    n_epoch= num_max_update/(N/batchsize)+1
    is_updating= True
    n_update= 0
    sum_loss= 0.0
    fp= OpenW(opt['log_filename'],'w')
    for epoch in xrange(n_epoch):
      perm= np.random.permutation(N)
      # Train model per batch
      for i in xrange(0, N, batchsize):
        x_batch= opt['x_train'][perm[i:i+batchsize]]
        y_batch= opt['y_train'][perm[i:i+batchsize]]
        if opt['gpu'] >= 0:
          x_batch= cuda.to_gpu(x_batch)
          y_batch= cuda.to_gpu(y_batch)

        opt['optimizer'].zero_grads()
        loss, pred= opt['fwd_loss'](x_batch, y_batch, train=True)
        loss.backward()  #Computing gradients
        opt['optimizer'].update()
        n_update+= 1

        sum_loss+= float(cuda.to_cpu(loss.data))
        if n_update % opt['num_check_stop'] == 0:
          #loss_maf.Update(float(cuda.to_cpu(loss.data)))
          loss_maf.Update(sum_loss / opt['num_check_stop'])
          sum_loss= 0.0
          if opt['verb']:  print 'Training %s:'%opt['code'], epoch, n_update, loss_maf.Mean, loss_maf.StdDev
          fp.write('%d %d %f %f\n' % (epoch, n_update, loss_maf.Mean, loss_maf.StdDev))
          if loss_maf.StdDev < opt['loss_stddev_stop']:
            is_updating= False
            break
        if n_update >= num_max_update:
          is_updating= False
          break
      if not is_updating:  break
    fp.close()

  #Main update code in which we train the mean model, generate y-error data, train the error model.
  def UpdateMain(self):
    if self.NSamples < self.Options['num_min_predictable']:  return

    #Train mean model
    opt={
      'code': '{code}-{n:05d}'.format(n=self.Params['num_train'], code=self.Options['name']+'mean'),
      'log_filename': self.Options['train_log_file'].format(n=self.Params['num_train'], name=self.Options['name'], code='mean', base=self.Options['base_dir']),
      'verb': self.Options['verbose'],
      'gpu': self.Options['gpu'],
      'fwd_loss': self.FwdLoss,
      'optimizer': self.optimizer,
      'x_train': self.DataX,
      'y_train': self.DataY,
      'batchsize': self.Options['batchsize'],
      'num_max_update': self.Options['num_max_update'],
      'num_check_stop': self.Options['num_check_stop'],
      'loss_maf_alpha': self.Options['loss_maf_alpha'],
      'loss_stddev_init': self.Options['loss_stddev_init'],
      'loss_stddev_stop': self.Options['loss_stddev_stop'],
      }
    self.TrainNN(**opt)

    # Generate training data for error model
    preds= []
    x_batch= self.DataX[:]
    if self.Options['gpu'] >= 0:
      x_batch= cuda.to_gpu(x_batch)
    pred= self.Forward(x_batch, train=False)
    D= self.DataY.shape[1]
    self.DataYErr= np.abs(cuda.to_cpu(pred.data) - self.DataY)

    #Train error model
    opt={
      'code': '{code}-{n:05d}'.format(n=self.Params['num_train'], code=self.Options['name']+'err'),
      'log_filename': self.Options['train_log_file'].format(n=self.Params['num_train'], name=self.Options['name'], code='err', base=self.Options['base_dir']),
      'verb': self.Options['verbose'],
      'gpu': self.Options['gpu'],
      'fwd_loss': self.FwdLossErr,
      'optimizer': self.optimizer_err,
      'x_train': self.DataX,
      'y_train': self.DataYErr,
      'batchsize': IfNone(self.Options['batchsize_err'], self.Options['batchsize']),
      'num_max_update': IfNone(self.Options['num_max_update_err'], self.Options['num_max_update']),
      'num_check_stop': IfNone(self.Options['num_check_stop_err'], self.Options['num_check_stop']),
      'loss_maf_alpha': IfNone(self.Options['loss_maf_alpha_err'], self.Options['loss_maf_alpha']),
      'loss_stddev_init': IfNone(self.Options['loss_stddev_init_err'], self.Options['loss_stddev_init']),
      'loss_stddev_stop': IfNone(self.Options['loss_stddev_stop_err'], self.Options['loss_stddev_stop']),
      }
    self.TrainNN(**opt)

    self.Params['num_train']+= 1

    #End of training NNs
    self.is_predictable= True


  #Incrementally update the internal parameters with a single I/O pair (x,y).
  #If x and/or y are None, only updating internal parameters is done.
  def Update(self, x=None, y=None, not_learn=False):
    #TFunctionApprox.Update(self, x, y, not_learn)
    if x!=None or y!=None:
      if len(self.DataX)==0:
        self.DataX= np.array([self.ToVec(x)],np.float32)
        self.DataY= np.array([self.ToVec(y)],np.float32)
      else:
        self.DataX= np.vstack((self.DataX, self.ToVec(x)))
        self.DataY= np.vstack((self.DataY, self.ToVec(y)))
    if not_learn:  return
    self.UpdateMain()

  #Incrementally update the internal parameters with I/O data (X,Y).
  #If x and/or y are None, only updating internal parameters is done.
  def UpdateBatch(self, X=None, Y=None, not_learn=False):
    #TFunctionApprox.UpdateBatch(self, X, Y, not_learn)
    if X!=None or Y!=None:
      if len(self.DataX)==0:
        self.DataX= np.array(X, np.float32)
        self.DataY= np.array(Y, np.float32)
      else:
        self.DataX= np.vstack((self.DataX, np.array(X, np.float32)))
        self.DataY= np.vstack((self.DataY, np.array(Y, np.float32)))
    if not_learn:  return
    self.UpdateMain()

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    res= self.TPredRes()
    #x_batch= np.array([self.ToVec(x)],np.float32)
    #if self.Options['gpu'] >= 0:
      #x_batch= cuda.to_gpu(x_batch)
    #pred= self.Forward(x_batch, train=False)
    #res.Y= cuda.to_cpu(pred.data)[0]
    #if with_var:
      #pred_err= self.ForwardErr(x_batch, train=False)
      #res.Var= np.diag(cuda.to_cpu(pred_err.data)[0])
      #res.Var= res.Var*res.Var
    y, y_var, g= self.ForwardX(x, x_var, with_var, with_grad)
    res.Y= y
    res.Var= y_var
    res.Grad= g
    return res



def TNNRegressionExample1():
  #TrueFunc= lambda x: 0.5*x
  #TrueFunc= lambda x: 1.2+math.sin(x)
  #TrueFunc= lambda x: 1.2+math.sin(3*x)
  #TrueFunc= lambda x: 2.0*x**2
  #TrueFunc= lambda x: 4.0-x if x>0.0 else 0.0
  #TrueFunc= lambda x: 4.0 if 0.0<x and x<2.5 else 0.0
  #TrueFunc= lambda x: 0.0 if x<0.0 else (2.0 if x<2.5 else 4.0)
  TrueFunc= lambda x: 0.0 if x<1.0 else 4.0

  #TEST: NN's estimation is bad where |x| is far from zero.
  Bound= [-3.0,5.0]
  #Bound= [-5.0,3.0]
  #Bound= [-3.0,3.0]
  #Bound= [1.0,5.0]
  #Bound= [-5.0,-1.0]
  #Bound= [-5.0,5.0]

  def GenData(n, noise):
    #data_x= [[x+1.0*Rand()] for x in FRange1(*Bound,num_div=n)]
    data_x= [[Rand(*Bound)] for k in range(n)]
    data_y= [[TrueFunc(x[0])+noise*Rand()] for x in data_x]
    #data_y= [[TrueFunc(x[0])+(noise if abs(x[0])<2.0 else 0.0)*Rand()] for x in data_x]
    return data_x, data_y

  #load_model,train_model= False,True
  load_model,train_model= True,False

  if train_model:
    x_train,y_train= GenData(100, noise=0.2)  #TEST: n samples, noise

    print 'Num of samples for train:',len(y_train)
    # Dump data for plot:
    fp1= file('/tmp/dnn/smpl_train.dat','w')
    for x,y in zip(x_train,y_train):
      fp1.write('%s #%i# %s\n' % (' '.join(map(str,x)),len(x)+1,' '.join(map(str,y))))
    fp1.close()

  x_test= np.array([[x] for x in FRange1(*Bound,num_div=100)]).astype(np.float32)
  y_test= np.array([[TrueFunc(x[0])] for x in x_test]).astype(np.float32)

  # Dump data for plot:
  fp1= file('/tmp/dnn/smpl_test.dat','w')
  for x,y in zip(x_test,y_test):
    fp1.write('%s #%i# %s\n' % (' '.join(map(str,x)),len(x)+1,' '.join(map(str,y))))
  fp1.close()

  batch_train= True
  #batch_train= False
  options= {}
  #options['AdaDelta_rho']= 0.5
  #options['AdaDelta_rho']= 0.9
  #options['dropout']= False
  #options['dropout_ratio']= 0.01
  options['loss_stddev_stop']= 1.0e-4
  options['loss_stddev_stop_err']= 1.0e-4
  options['num_max_update']= 20000
  #options['batchsize']= 5
  #options['batchsize']= 10
  #options['num_check_stop']= 50
  #options['loss_maf_alpha']= 0.4
  options['loss_stddev_stop']= 1.0e-4
  options['loss_stddev_stop_err']= 1.0e-4
  model= TNNRegression()
  #print 'model.Options=',model.Options
  model.Load({'options':options})
  if load_model:
    model.Load(LoadYAML('/tmp/dnn/nn_model.yaml'), '/tmp/dnn/')
  model.Init()
  #print 'model.Options=',model.Options
  if train_model:
    if not batch_train:
      for x,y,n in zip(x_train,y_train,range(len(x_train))):
        print '========',n,'========'
        model.Update(x,y,not_learn=((n+1)%min(10,len(x_train))!=0))
      #model.Update()
    else:
      #model.Options['dropout_ratio']= options['dropout_ratio']
      model.UpdateBatch(x_train,y_train)
      #model.Options['dropout_ratio']= 0.0
      #model.UpdateBatch()

  if not load_model:
    SaveYAML(model.Save('/tmp/dnn/'), '/tmp/dnn/nn_model.yaml')

  # Dump data for plot:
  fp1= file('/tmp/dnn/nn_test%04i.dat'%1,'w')
  for x in x_test:
    with_var,with_grad= True, True
    pred= model.Predict(x,x_var=0.0**2,with_var=with_var,with_grad=with_grad)
    y= pred.Y.ravel()
    y_err= np.sqrt(np.diag(pred.Var)) if with_var else [0.0]
    grad= pred.Grad.ravel() if with_grad else [0.0]
    fp1.write('%s #%i# %s %s %s\n' % (' '.join(map(str,x)), len(x)+1, ' '.join(map(str,y)), ' '.join(map(str,y_err)), ' '.join(map(str,grad)) ))
  fp1.close()

  # Dump data for plot:
  fp1= file('/tmp/dnn/nn_test%04i.dat'%2,'w')
  for x in x_test:
    with_var,with_grad= True, True
    pred= model.Predict(x,x_var=0.5**2,with_var=with_var,with_grad=with_grad)
    y= pred.Y.ravel()
    y_err= np.sqrt(np.diag(pred.Var)) if with_var else [0.0]
    grad= pred.Grad.ravel() if with_grad else [0.0]
    fp1.write('%s #%i# %s %s %s\n' % (' '.join(map(str,x)), len(x)+1, ' '.join(map(str,y)), ' '.join(map(str,y_err)), ' '.join(map(str,grad)) ))
  fp1.close()

