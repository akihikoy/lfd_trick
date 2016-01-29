#!/usr/bin/python
#\file    test_lquad.py
#\brief   certain python script
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.30, 2015
import roslib; roslib.load_manifest('lfd_trick')
from base import *

def Main():
  #def PrintEq(s):  print '%s= %r' % (s, eval(s))

  def func(x):
    if abs(x[0])<0.5:  return 0.0
    return -abs(x[0])-x[1]*x[1]

  #def func(x):
    #return -(np.dot(x[:2],x[:2]))

  load_model= False

  options= {}
  #options['h']= 0.1
  #model= TLocalQuad(2,lambda x:-(x[0]*x[0]+x[1]*x[1]))
  model= TLocalQuad(2,lambda x:func(x))
  model.Load({'options':options})
  model.Init()

  mi= [-1.0,-1.0]
  ma= [+1.0,+1.0]
  me= [0.0,0.0]
  f_reduce=lambda xa:xa
  f_repair=lambda xa,mi,ma,me:xa
  DumpPlot(model, f_reduce=f_reduce, f_repair=f_repair, file_prefix='/tmp/lquad', x_var=[0.2,0.2], bounds=[mi,ma])

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa -3d
          -s 'set xlabel "x";set ylabel "y";set title "r";set ticslevel 0;'
          -cs 'u 1:2:5' /tmp/lquad_est.dat w l &''',
    '''''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines())
      print '###',cmd
      os.system(cmd)

  print '##########################'
  print '###Press enter to close###'
  print '##########################'
  raw_input()
  os.system('qplot -x2kill aaa')

if __name__=='__main__':
  import sys
  if len(sys.argv)>1 and sys.argv[1] in ('p','plot','Plot','PLOT'):
    PlotGraphs()
    sys.exit(0)
  Main()
