#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of CompOptInference.
  Usage: test_opt1'''
def Run(t,*args):

  #Discrete parameter is a type of function
  #Note that each function assumes different size of x
  def fobj1(x,f_none=None):
    assert len(x)==2
    if not IsIn(x[0], [-3.0,-0.5]):  return f_none
    if (x[0]+2.0)**2+(x[1]+0.5)**2<0.2:  return f_none
    return -0.01*(3.0*(x[0]-1.2)**2 + 2.0*(x[1]+2.0)**2)

  def fobj2(x,f_none=None):
    assert len(x)==2
    if not IsIn(x[0], [0.0,1.0]):  return f_none
    return -0.01*(0.5*(x[0]+2.0)**2 + 0.5*(x[1]+2.0)**2 + 20.0)

  def fobj3(x,f_none=None):
    assert len(x)==1
    if not IsIn(x[0], [1.5,3.0]):  return f_none
    return -0.01*(20.0*(x[0]-2.0)**2 - 15.0)

  #x[0]: discrete parameter to select function
  #x[1]: continuous parameters for each function; size depends on x[0]
  def fobj(x,f_none=None):
    if x[0]==0:    f= fobj1(x[1])
    elif x[0]==1:  f= fobj2(x[1])
    elif x[0]==2:  f= fobj3(x[1])
    if f is None:  return f_none
    return f

  opt= TCompositeOpt()
  options= {}
  options['param_struct']= ['x1','x',[ ['c1','c'], ['c2','c'], ['c3','c'] ]]
  options['x1']= {}
  #options['x1']['ucb_nsd']= 2.0
  options['x1']['alpha']= 0.2
  options['c1']= {}
  options['c1']['bounds']= [[-3.0,-3.0],[3.0,3.0]]
  options['c1']['tolfun']= 1.0e-4
  options['c1']['scale0']= 1.0
  options['c1']['parameters0']= [0.0,0.0]
  #options['c1']['parameters0']= [-2.0,0.0]
  options['c2']= {}
  options['c2']['bounds']= [[-3.0,-3.0],[3.0,3.0]]
  options['c2']['tolfun']= 1.0e-4
  options['c2']['scale0']= 1.0
  options['c2']['parameters0']= [0.0,0.0]
  #options['c2']['parameters0']= [0.5,0.0]
  options['c3']= {}
  options['c3']['bounds']= [[-3.0],[3.0]]
  options['c3']['tolfun']= 1.0e-4
  options['c3']['scale0']= 1.0
  options['c3']['parameters0']= [0.0]
  #options['c3']['parameters0']= [2.0]

  parameters_res, score_res=  CompOptInference(fobj, options,
                                               t.database, db_search_key='xxx', infer_type='xxx',
                                               param_dist=lambda p1,p2:CompDist(p1,p2,1.0e6))
  print parameters_res, score_res

