#! /usr/bin/env python
#Basic tools (advanced stuff).
import numpy as np
import numpy.linalg as la
import math
import random
import tf
from base_util import *
from base_geom import *

#Closest point on a line (p1,p2) from a reference point
def LineClosestPoint(p1, p2, point_ref):
  a= Vec(p2)-Vec(p1)
  t_max= la.norm(a)
  a= a/t_max
  t= np.dot(Vec(point_ref)-p1, a)
  if t>=t_max:  return p2
  if t<=0:      return p1
  return Vec(p1) + t*a

#Closest point on a polygon from a reference point
def PolygonClosestPoint(points, point_ref):
  if len(points)==0:  return None
  if len(points)==1:  return points[0]
  p_closest= None
  d_closest= 1.0e20
  N= len(points)
  for i in range(N):
    p= LineClosestPoint(points[i], points[i+1 if i+1!=N else 0], point_ref)
    d= la.norm(Vec(point_ref)-Vec(p))
    if p_closest==None or d<d_closest:
      p_closest= p
      d_closest= d
  return p_closest

#ref. http://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
#Ray-casting algorithm (http://en.wikipedia.org/wiki/Point_in_polygon)
def PointInPolygon2D(points, point):
  c= False
  j= len(points)-1
  for i in range(len(points)):
    if ((points[i][1]>point[1]) != (points[j][1]>point[1])) and (point[0] < (points[j][0]-points[i][0]) * (point[1]-points[i][1]) / (points[j][1]-points[i][1]) + points[i][0]) :
      c= not c
    j= i
  return c

class TPCA:
  def __init__(self,points,calc_projected=True):
    self.Mean= np.mean(points,axis=0)
    data= points-self.Mean
    cov= np.cov(data.T)
    evals, evecs= la.eig(cov)
    idx= evals.argsort()[::-1]  #Sort by eigenvalue in decreasing order
    self.EVecs= evecs[:,idx]
    self.EVals= evals[idx]
    self.Projected= None
    if calc_projected:
      self.Projected= np.dot(data, self.EVecs)

  def Project(self,points):
    return np.dot(points-self.Mean, self.EVecs)

  def Reconstruct(self,proj,idx=None):
    if idx==None:  idx= range(len(self.EVecs))
    return np.dot(proj, self.EVecs[idx]) + self.Mean

#Centroid of a polygon
#ref. http://en.wikipedia.org/wiki/Centroid
def PolygonCentroid(points, pca_default=None, only_centroid=True):
  if len(points)==0:  return None
  if len(points)==1:  return points[0]
  assert(len(points[0])==3)
  if pca_default==None:
    pca= TPCA(points)
  else:
    pca= pca_default
  xy= pca.Projected[:,[0,1]]
  N= len(xy)
  xy= np.vstack((xy,[xy[0]]))  #Extend so that xy[N]==xy[0]
  A= 0.5*sum([xy[n,0]*xy[n+1,1] - xy[n+1,0]*xy[n,1] for n in range(N)])
  Cx= sum([(xy[n,0]+xy[n+1,0])*(xy[n,0]*xy[n+1,1]-xy[n+1,0]*xy[n,1]) for n in range(N)]) / (6.0*A)
  Cy= sum([(xy[n,1]+xy[n+1,1])*(xy[n,0]*xy[n+1,1]-xy[n+1,0]*xy[n,1]) for n in range(N)]) / (6.0*A)
  centroid= pca.Reconstruct([Cx,Cy],[0,1])
  if only_centroid:  return centroid
  else:  return centroid, [Cx,Cy]

#Get a parameterized polygon
#  closed: Closed polygon or not.  Can take True,False,'auto' (automatically decided)
#  center_modifier: Function (centroid of polygon --> new center) to modify the center.
class TParameterizedPolygon:
  def __init__(self, points, center_modifier=None, closed='auto'):
    assert(len(points)>=3);
    pca= TPCA(points)
    self.Center, self.Center2d= PolygonCentroid(points, pca, only_centroid=False)
    if center_modifier is not None:
      self.Center= np.array(center_modifier(self.Center))
      self.Center2d= pca.Project(self.Center)[:2]
    self.PCAAxes= pca.EVecs
    self.PCAValues= pca.EVals
    self.PCAMean= pca.Mean
    angles= [0]*len(pca.Projected)
    dirc= 0
    for i in range(len(points)):
      diff= pca.Projected[i,[0,1]] - np.array(self.Center2d)
      angles[i]= math.atan2(diff[1],diff[0])
      if angles[i]>math.pi:  angles[i]-= 2.0*math.pi
      if i>0:
        if AngleDisplacement(angles[i-1],angles[i])>0: dirc+=1
        else: dirc-=1
    #print dirc
    dirc= Sign(dirc)
    self.Direction= dirc
    self.Angles= []
    self.Points= []
    self.Points2D= []
    aprev= angles[0]
    for i in range(len(points)):
      if i==0 or Sign(AngleDisplacement(aprev,angles[i]))==dirc:
        self.Angles.append(angles[i])
        self.Points.append(points[i])
        self.Points2D.append(pca.Projected[i,[0,1]])
        aprev= angles[i]
    #print dirc,self.Angles[0],self.Angles[-1]
    if   dirc>0 and self.Angles[-1]>self.Angles[0]:  self.Offset= -math.pi-self.Angles[0]
    elif dirc<0 and self.Angles[-1]<self.Angles[0]:  self.Offset= -math.pi-self.Angles[-1]
    elif dirc>0 and self.Angles[-1]<self.Angles[0]:  self.Offset= math.pi-self.Angles[0]+1.0e-12
    elif dirc<0 and self.Angles[-1]>self.Angles[0]:  self.Offset= math.pi-self.Angles[-1]+1.0e-12
    self.Angles= [AngleMod1(a+self.Offset) for a in self.Angles]
    if closed=='auto':
      avr_a_diff= sum([abs(AngleDisplacement(self.Angles[i-1],self.Angles[i])) for i in range(1,len(self.Angles))])/float(len(self.Angles)-1)
      self.IsClosed= (abs(AngleDisplacement(self.Angles[-1],self.Angles[0])) < 2.0*avr_a_diff)
      #print dirc,self.Angles[0],self.Angles[-1]
      #print 'IsClosed:', self.IsClosed,avr_a_diff, abs(AngleDisplacement(self.Angles[-1],self.Angles[0]))
    else:
      self.IsClosed= closed
    if self.IsClosed:
      self.Angles.append(self.Angles[0])
      self.Points.append(points[0])
      self.Points2D.append(pca.Projected[0,[0,1]])
    self.IdxAngleMin= min(range(len(self.Angles)), key=lambda i: self.Angles[i])
    self.IdxAngleMax= max(range(len(self.Angles)), key=lambda i: self.Angles[i])
    #print 'angles:',angles
    #print 'self.Angles:',self.Angles
    self.Bounds= self.ComputeBounds()
    self.EstimateAxes(dtheta=2.0*math.pi/50.0)

  #Return a boundary of possible angle for AngleToPoint.
  #NOTE: AngleToPoint(angle) will return None even if angle is in this bound.
  def ComputeBounds(self):
    if self.IsClosed:
      return [-math.pi, math.pi]
    if Sign(self.Angles[-1]-self.Angles[0])==self.Direction:
      if self.Angles[0]<self.Angles[-1]:  return [self.Angles[0], self.Angles[-1]]
      else:                               return [self.Angles[-1], self.Angles[0]]
    raise Exception('Bug in TParameterizedPolygon::ComputeBounds')
    #if self.Direction>0:
      #return [[self.Angles[0],math.pi],[-math.pi,self.Angles[-1]]]
    #else:
      #return [[self.Angles[-1],math.pi],[-math.pi,self.Angles[0]]]

  def AngleToPoint(self, angle):
    angle= AngleMod1(angle)
    if not IsIn(angle, self.Bounds):  return None
    if angle<=self.Angles[self.IdxAngleMin]:
      i_closest= self.IdxAngleMin
      i_closest2= self.IdxAngleMax
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= 2.0*math.pi-self.Angles[i_closest2]+angle
    elif angle>=self.Angles[self.IdxAngleMax]:
      i_closest= self.IdxAngleMax
      i_closest2= self.IdxAngleMin
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= 2.0*math.pi+self.Angles[i_closest2]-angle
    else:
      i_closest= filter(lambda i: IsAngleIn(angle,[self.Angles[i],self.Angles[i+1]]), range(len(self.Angles)-1))
      if len(i_closest)==0:  return None
      i_closest= i_closest[0]
      i_closest2= i_closest+1
      alpha= abs(angle-self.Angles[i_closest])
      alpha2= abs(self.Angles[i_closest2]-self.Angles[i_closest]) - alpha
    if abs(alpha)<1.0e-6:  return np.array(self.Points[i_closest])
    if abs(alpha2)<1.0e-6:  return np.array(self.Points[i_closest2])
    pi= np.array(self.Points2D[i_closest])
    pi2= np.array(self.Points2D[i_closest2])
    c= self.Center2d
    ratio= la.norm(pi-c)/la.norm(pi2-c) * math.sin(alpha)/math.sin(alpha2)
    t= ratio/(1.0+ratio)
    if t<0.0 or t>1.0:  return None
    assert(t>=0.0 and t<=1.0)
    return t*np.array(self.Points[i_closest2]) + (1.0-t)*np.array(self.Points[i_closest])

  def PointToAngle(self, point):
    diff= np.dot(point-self.PCAMean, self.PCAAxes)[[0,1]] - np.array(self.Center2d)
    return AngleMod1(math.atan2(diff[1],diff[0])+self.Offset)

  def EstimateAxes(self,dtheta):
    theta= 0.0
    points= []
    while theta<2.0*math.pi:
      p= self.AngleToPoint(theta)
      if p is not None: points.append(p)
      theta+= dtheta
    pca= TPCA(points)
    self.Axes= pca.EVecs
    self.AxValues= pca.EVals

#Fitting a circle to the data XY, return the center [x,y] and the radius
def CircleFit2D_SVD(XY):
  centroid= np.average(XY,0) # the centroid of the data set

  X= [XY[d][0]-centroid[0] for d in range(len(XY))] # centering data
  Y= [XY[d][1]-centroid[1] for d in range(len(XY))] # centering data
  Z= [X[d]**2 + Y[d]**2 for d in range(len(XY))]
  ZXY1= np.matrix([Z, X, Y, [1.0]*len(Z)]).transpose()
  U,S,V= la.svd(ZXY1,0)
  if S[3]/S[0]<1.0e-12:  # singular case
    print "SINGULAR"
    A= (V.transpose())[:,3]
  else:  # regular case
    R= np.average(np.array(ZXY1),0)
    N= np.matrix([[8.0*R[0], 4.0*R[1], 4.0*R[2], 2.0],
                  [4.0*R[1], 1.0, 0.0, 0.0],
                  [4.0*R[2], 0.0, 1.0, 0.0],
                  [2.0,      0.0, 0.0, 0.0]])
    W= V.transpose()*np.diag(S)*V
    D,E= la.eig(W*la.inv(N)*W)  # values, vectors
    idx= D.argsort()
    Astar= E[:,idx[1]]
    A= la.solve(W, Astar)

  A= np.array(A)[:,0]
  center= -A[1:3].transpose()/A[0]/2.0+centroid
  radius= math.sqrt(A[1]**2+A[2]**2-4.0*A[0]*A[3])/abs(A[0])/2.0
  return center, radius

#Fitting a circle to the data marker_data, return the center [x,y,z] and the radius
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
def CircleFit3D(marker_data):
  #Compute the normal vector
  p_mean= np.array([0.0,0.0,0.0])
  n_z= np.array([0.0,0.0,0.0])
  for x in marker_data:
    p,R= XToPosRot(x)
    n_z+= R[:,2]
    p_mean+= p
  n_z= n_z/float(len(marker_data))
  n_z= n_z/la.norm(n_z)
  p_mean= p_mean/float(len(marker_data))
  n_x= np.array([-n_z[1],n_z[0],0.0])
  n_x= n_x/la.norm(n_x)
  n_y= np.cross(n_z,n_x)

  print 'p_mean= ',p_mean
  print 'n_x= ',n_x
  print 'n_y= ',n_y
  print 'n_z= ',n_z

  #Project the data onto the plane n_x, n_y
  p_data= []
  for x in marker_data:
    p= np.array([x[0],x[1],x[2]])
    lx= np.dot(p-p_mean,n_x)
    ly= np.dot(p-p_mean,n_y)
    p_data.append([lx,ly])
    #print 'lx,ly= ',lx,ly

  #print p_data
  p_file= file('/tmp/original.dat','w')
  for x in marker_data:
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  #Compute the circle parameters
  lcx, radius= CircleFit2D_SVD(p_data)
  x_center= lcx[0]*n_x + lcx[1]*n_y + p_mean
  #print x_center, radius

  p_file= file('/tmp/circle.dat','w')
  for ith in range(1000):
    th= (2.0*math.pi)/1000.0*float(ith)
    lx= lcx[0]+radius*math.cos(th)
    ly= lcx[1]+radius*math.sin(th)
    x= lx*n_x + ly*n_y + p_mean
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  return x_center, radius

#Compute the sensor pose x_sensor on the robot frame from data
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
#  gripper_data: corresponding gripper pose sequence (on the robot frame)
#  x_g2m: marker pose on the gripper's local frame
def CalibrateSensorPose(marker_data, gripper_data, x_g2m):
  assert(len(marker_data)==len(gripper_data))
  #Marker poses on the robot frame
  robot_marker_data= map(lambda x: Transform(x,x_g2m), gripper_data)
  #Sensor poses
  x_sensor_data= [TransformRightInv(robot_marker_data[d], marker_data[d]) for d in range(len(marker_data))]
  #Average sensor poses to get x_sensor
  x_sensor= AverageXData(x_sensor_data)
  #x_sensor= x_sensor_data[0]
  print '##--------------##'
  print 'la.norm(x_sensor[3:]):',la.norm(x_sensor[3:])
  print '##gripper_data[0]:',gripper_data[0]
  print '##robot_marker_data[0]:',robot_marker_data[0]
  print '##Transform(gripper_data[0],x_g2m):',Transform(gripper_data[0],x_g2m)
  print '##marker_data[0]:',marker_data[0]
  print '##x_sensor_data[0]:',x_sensor_data[0]
  print '##Transform(x_sensor_data[0],marker_data[0]):',Transform(x_sensor_data[0],marker_data[0])
  print '##Transform(x_sensor,marker_data[0]):',Transform(x_sensor,marker_data[0])
  print '##--------------##'
  #for i in range(len(x_sensor_data)):
    #x= x_sensor_data[i]
    #print x, tf.transformations.euler_from_quaternion(x[3:7]), marker_data[i]
  print '##--------------##'
  #print x_sensor
  err= [0.0]*7
  for d in range(len(robot_marker_data)):
    err= [err[k]+abs(Transform(x_sensor,marker_data[d])[k]-robot_marker_data[d][k]) for k in range(7)]
    #err+= la.norm(np.array(Transform(x_sensor,marker_data[d]))-np.array(robot_marker_data[d]))
  err= [err[k]/float(len(robot_marker_data)) for k in range(7)]
  print 'Error:',err
  return x_sensor
