#!/usr/bin/python
import roslib; roslib.load_manifest('lfd_trick')
from base import *

if __name__ == '__main__':
  urand= lambda: 2.0*(random.random()-0.5)

  p1= [urand(),urand(),urand()]
  p2= [urand(),urand(),urand()]
  #p1= [urand(),urand(),0.]
  #p2= [urand(),urand(),0.]

  axis,angle= GetAxisAngle(p1,p2)

  reconstructed= np.dot(RFromAxisAngle(axis,angle),p1)
  reconstructed*= Norm(p2)/Norm(p1)

  print 'p1=',p1
  print 'p2=',p2
  print 'axis=',axis,Norm(axis)
  print 'angle=',angle
  print 'reconstructed=',reconstructed
  print 'error=',Norm(Vec(reconstructed)-Vec(p2))

