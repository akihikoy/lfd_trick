#!/usr/bin/python
from core_tool import *
def Help():
  return '''Event-driven state machine test.
  Usage: test_smev'''
def Run(t,*args):
  import time,sys

  l= TContainer(debug=True)
  l.messenger= Queue.Queue()
  l.thread_name= 'SMEv'

  def ReadEvent():
    while t.thread_manager.IsRunning(l.thread_name):
      try:
        l.event= l.messenger.get(timeout=5.0)
        break
      except Queue.Empty:
        pass

  sm= TStateMachine()
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].EntryAction= lambda: CPrint(1,'Hello state machine!')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'switch'

  sm.NewState('switch')
  sm['switch'].EntryAction= ReadEvent
  sm['switch'].NewAction()
  sm['switch'].Actions[-1].Condition= lambda: l.event=='q'
  sm['switch'].Actions[-1].Action= lambda: CPrint(1,'Bye bye')
  sm['switch'].Actions[-1].NextState= 'stop'
  sm['switch'].NewAction()
  sm['switch'].Actions[-1].Condition= lambda: not t.thread_manager.IsRunning(l.thread_name)
  sm['switch'].Actions[-1].NextState= 'stop'
  sm['switch'].NewAction()
  sm['switch'].Actions[-1].Condition= lambda: l.event=='p'
  sm['switch'].Actions[-1].Action= lambda: CPrint(1,'p is pushed!!!')
  sm['switch'].Actions[-1].NextState= 'buffer'
  sm['switch'].ElseAction.Condition= lambda: True
  sm['switch'].ElseAction.Action= lambda: CPrint(3,'Unrecognized event:',l.event)
  sm['switch'].ElseAction.NextState= 'buffer'

  #A buffer state to reenter to switch; if we use ORIGIN_STATE in switch
  sm.NewState('buffer')
  sm['buffer'].ElseAction.Condition= lambda: True
  sm['buffer'].ElseAction.NextState= 'switch'

  sm.NewState('stop')
  sm['stop'].EntryAction= lambda: CPrint(1,'Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Show()
  #sm.Run()
  #Run the state machine as a thread:
  t.thread_manager.Add(name='SMEv', target=lambda th_info: sm.Run())

  t.kbhit.Activate()
  print ' q to stop or >'
  while t.thread_manager.IsRunning('SMEv'):
    #print t.thread_manager.IsRunning('SMEv')  #FIXME: remove thread object if exception happens
    key= t.kbhit.KBHit()
    #key= raw_input('input > ')
    if key!=None:
      l.messenger.put(key)
      if key=='q':  break
      print ' q to stop or >'
  t.kbhit.Deactivate()

  time.sleep(0.5)

  if t.thread_manager.IsRunning('SMEv'):
    #t.thread_manager.Stop('SMEv')
    t.thread_manager.StopRequest('SMEv')

  l= None
