#!/usr/bin/python
#\file    toy1.py
#\brief   Toy problem for a dynamic programming and learning dynamics.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.20, 2015

'''
We consider 2-D flow from a source container and a receiving container.
Pouring point of the source container is (y,z) which is an action parameter.
z >= 0.3: height constraint.
We consider the orientation of the source container is fixed.
y is horizontal and z is vertical.
The position of the receiving container is (yl,0), and its width is wl.
Pouring with an action (y,z) produces a flow; its center is (cy,cz) and its variance is w.
They are given by a simplified dynamics model:
  cy= y+z^(1/2)
  cz= 0
  w= z/2
Then score e is computed.  If the flow is inside the receiving container, e is 1.
Otherwise, negative penalty is given.
Our purpose is to optimize the action (y,z) w.r.t. the score.
'''

import numpy as np

class TContainerToy1:
  pass

def Dynamics(tsys,y,z):
  cy= y + 0.1*z**0.5
  cz= 0.0
  w=  0.4*z
  tsys.n_dyn+= 1
  return cy,cz,w


#def Assess(tsys,cy,cz,w):
  #if (cy+0.5*w) >= (tsys.yl+0.5*tsys.wl):
    #return -1
  #if (cy-0.5*w) <= (tsys.yl-0.5*tsys.wl):
    #return -1
  #return +1.0

def Assess(tsys,cy,cz,w):
  if (cy+0.5*w) >= (tsys.yl+0.5*tsys.wl):
    return (tsys.yl+0.5*tsys.wl) - (cy+0.5*w)  #Penalty
    #return -1
  if (cy-0.5*w) <= (tsys.yl-0.5*tsys.wl):
    return (cy-0.5*w) - (tsys.yl-0.5*tsys.wl)  #Penalty
    #return -1
  e= +1.0 - 50.0*(cy-tsys.yl)**2
  return e if e>0.0 else e
  #return +1.0


#Dynamics+Assess system
def Whole(tsys,y,z):
  if tsys.DEBUG:  print 'y,z:',y,z,
  cy,cz,w= Dynamics(tsys,y,z)
  if tsys.DEBUG:  print '  cy,cz,w:',cy,cz,w,
  e= Assess(tsys,cy,cz,w)
  if tsys.DEBUG:  print '  e:',e
  return e

def Constrain(bound,v):
  return [min(max(v[d],bound[0][d]),bound[1][d]) for d in range(len(v))]

tsys= TContainerToy1()
tsys.yl= 0.5  #Location of receiving container.
tsys.wl= 0.3  #Size of receiving container
tsys.DEBUG= False
tsys.bound= [[0.2,0.3],[0.8,0.8]]  #Boundary of y,z
tsys.bound2= [[0.2,0.0,0.0],[0.8,0.0,0.8]]  #Boundary of cy,cz,w
tsys.n_dyn= 0  #Number of Dynamics computation

'''
x1= [y,z].T
x2= [cy,cz,w].T
y= [e]f
x2= F1(x1)
y= F2(x2)
y= F12(x1) == F2(F1(x1))
'''

def F1(x1):
  return np.array(Dynamics(tsys,x1[0],x1[1])).T

def F2(x2):
  return np.array([Assess(tsys,x2[0],x2[1],x2[2])]).T

def F12(x1):
  return F2(F1(x1))

#Constrain x1
def C1(x1):
  return np.array(Constrain(tsys.bound, x1)).T

#Constrain x2
def C2(x2):
  return np.array(Constrain(tsys.bound2, x2)).T

