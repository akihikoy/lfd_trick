#!/usr/bin/python
from core_tool import *
def Help():
  return '''Thread tool.
  Usage:
    th [INDEXES]
    th 'list'
      Display the threads.
    th 'stop', TH_NAME
      Stop a thread named TH_NAME.
      TH_NAME: Thread name.
    th 'stopall'
      Stop all threads.
  '''
def Run(t,*args):
  c1,c2= ACol.X2(1)

  if len(args)==0 or not isinstance(args[0],str):
    command= 'list'
  else:
    command= args[0]
    args= args[1:]

  if command=='list':
    assert(len(args)==0)
    for name,th in t.thread_manager.thread_list.iteritems():
      print '%s%s%s: %s, running=%s, thread=%s' % (
          c1,name,c2, hex(id(th)), str(th.running), hex(id(th.thread)))

  elif command=='stop':
    assert(len(args)==1)
    print 'Stop thread %r...' % args[0],
    t.thread_manager.Stop(args[0])
    print 'ok'

  elif command=='stopall':
    assert(len(args)==0)
    t.thread_manager.StopAll()

  else:
    raise Exception('Invalid command: %r'%command)
