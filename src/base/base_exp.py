#!/usr/bin/python
#Experiments tools.
from base_util import *

'''
Experiments tool to repeat a program with conditions.
main= function(logdir,options)
basedir='{home}/data/ode_grpour/e2015_06_19/'
logdir='{base}/{cond}/run{n:03}/'
  {home}: home directory.
  {base}: base directory.
  {cond}: condition name.
  {n}: run index.
dirs [subdir1, subdir2, ..]
num_runs: number of runs.
conditions {'cond1':options1, 'cond2':options2, ...}
'''
def Exp(main, basedir, logdir, dirs, num_runs, conditions):
  import os,time,filecmp
  exists= os.path.exists
  mkdirs= os.makedirs
  rename= os.rename
  remove= os.remove
  homedir= os.environ['HOME']

  basedir1= basedir.format(home=homedir)
  if exists(basedir1):
    print '########################################'
    print 'Base directory exists:',basedir1
    print '########################################'
    for i in range(3):
      print 'Start in %i sec... (Ctrl+C to stop)'%(3-i)
      time.sleep(1.0)
  else:
    mkdirs(basedir1)

  #Save setup into a file.
  setup= {'basedir':basedir,
          'logdir':logdir,
          'dirs':dirs,
          'num_runs':num_runs,
          'conditions':conditions}
  tmpfilename= '/tmp/exp%s.txt' % os.getpid()
  SaveYAML(setup, tmpfilename)
  exp_file= basedir1+'/setup.yaml'
  if exists(exp_file):
    if filecmp.cmp(exp_file, tmpfilename):
      remove(tmpfilename)
    else:
      i= 0
      while True:
        exp_file_bk= basedir1+'/setup%s.yaml'%i
        if not exists(exp_file_bk):  break
        i+= 1
      rename(exp_file, exp_file_bk)
      rename(tmpfilename, exp_file)
  else:
    rename(tmpfilename, exp_file)

  error_flag= False
  for n in range(num_runs):
    for cond,options in conditions.iteritems():
      logdir1= logdir.format(home=homedir, base=basedir1, cond=cond, n=n)
      if exists(logdir1):
        print 'Log directory exists:',logdir1
        print 'Skip...'
        continue
      mkdirs(logdir1)
      for d in dirs:  mkdirs(logdir1+d)
      res= main(logdir1, options)
      if res!=None and not res:
        error_flag= True
        break
    if error_flag:  break

