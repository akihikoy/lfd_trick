#!/usr/bin/python
#\file    test_lwr2a.py
#\brief   Test src/base/base_ml_lwr2.py
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.20, 2015
import roslib; roslib.load_manifest('lfd_trick')
from base import *

def Main():
  #def PrintEq(s):  print '%s= %r' % (s, eval(s))

  load_model= False

  options= {}
  options['kernel']= 'maxg'
  options['c_min']= 0.01
  options['f_reg']= 0.0001
  model= TLWR()
  model.Load({'options':options})
  if load_model:
    model.Load(LoadYAML('/tmp/lwr/lwr_model.yaml'), '/tmp/lwr/')
  model.Init()

  src_file= 'data/ode_f3_smp.dat'; dim= [2,3,2]
  assess= lambda y: 5.0*y[0]+y[1]

  if not load_model:
    fp= file(src_file)
    while True:
      line= fp.readline()
      if not line: break
      data= line.split()
      model.Update(map(float,data[sum(dim[0:1]):sum(dim[0:2])]),
                   map(float,data[sum(dim[0:2]):sum(dim[0:3])]))
    SaveYAML(model.Save('/tmp/lwr/'), '/tmp/lwr/lwr_model.yaml')

  mi= [min([x[d] for x in model.DataX]) for d in range(len(model.DataX[0]))]
  ma= [max([x[d] for x in model.DataX]) for d in range(len(model.DataX[0]))]
  me= [Median([x[d] for x in model.DataX]) for d in range(len(model.DataX[0]))]
  f_reduce=lambda xa:[xa[0],xa[2]]
  f_repair=lambda xa,mi,ma,me:[xa[0],me[1],xa[1]]
  DumpPlot(model, f_reduce=f_reduce, f_repair=f_repair, file_prefix='/tmp/lwr/f3', x_var=[0.,0.,0.], bounds=[mi,ma])
  fp= open('/tmp/lwr/f3_ideals.dat','w')
  for xa1,x2 in zip(model.DataX, model.DataY):
    if assess(x2)>0.5:
      fp.write('%s\n' % ToStr(f_reduce(xa1),xa1,[assess(x2)]))
  fp.close()

def PlotGraphs():
  print 'Plotting graphs..'
  import os
  commands=[
    '''qplot -x2 aaa -3d
          -s 'set xlabel "flow_x";set ylabel "flow_var";set title "amount, -spill";set ticslevel 0;'
          -cs 'u 1:2:($6*5)' /tmp/lwr/f3_est.dat w l /tmp/lwr/f3_smp.dat -cs '' /tmp/lwr/f3_ideals.dat u 1:2:'($6)' ps 3
          -cs 'u 1:2:7'      /tmp/lwr/f3_est.dat w l /tmp/lwr/f3_smp.dat -cs '' /tmp/lwr/f3_ideals.dat u 1:2:'($6)' ps 3 &''',
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
