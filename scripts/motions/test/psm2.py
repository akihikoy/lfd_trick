#!/usr/bin/python
from core_tool import *
def Help():
  return '''Parallel state machine test with core_tool's SM execution interface.
  Usage: test_psm'''

def DefSubSM(t, signal):
  sm= TStateMachine(local_obj=TContainer(debug=True))
  #sm.Debug= True
  sm.l.event_messenger= signal.NewQueue()

  def ReadEvent():
    #sm.l.event= sm.l.event_messenger.get()
    while True:
      try:
        sm.l.event= sm.l.event_messenger.get(timeout=5.0)
        break
      except Queue.Empty:
        if not sm.ThreadInfo.IsRunning():
          sm.SetExceptionFlag()
          break

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].EntryAction= lambda: CPrint(1,'I am sub state machine!')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'event_read'

  sm.NewState('event_read')
  sm['event_read'].ElseAction.Condition= lambda: True
  sm['event_read'].ElseAction.Action= ReadEvent
  sm['event_read'].ElseAction.NextState= 'event_handle'

  sm.NewState('event_handle')
  sm['event_handle'].NewAction()
  sm['event_handle'].Actions[-1].Condition= lambda: sm.l.event==(EVENT_STATE_ENTRY,'special')
  sm['event_handle'].Actions[-1].Action= lambda: CPrint(1,'x')
  sm['event_handle'].Actions[-1].NextState= 'special'
  sm['event_handle'].NewAction()
  sm['event_handle'].Actions[-1].Condition= lambda: sm.l.event==(EVENT_STATE_ENTRY,'stop')
  sm['event_handle'].Actions[-1].Action= lambda: CPrint(1,'q')
  sm['event_handle'].Actions[-1].NextState= 'stop'
  sm['event_handle'].ElseAction.Condition= lambda: True
  #sm['event_handle'].ElseAction.Action= lambda: (CPrint(1, sm.l.event), time.sleep(0.1))
  sm['event_handle'].ElseAction.NextState= 'event_read'

  sm.NewState('special')
  sm['special'].EntryAction= lambda: (CPrint(3,'SPECIAL EVENT!!!'), time.sleep(0.5))
  sm['special'].ElseAction.Condition= lambda: True
  sm['special'].ElseAction.NextState= 'event_read'

  sm.NewState('stop')
  sm['stop'].EntryAction= lambda: (time.sleep(0.5), CPrint(1,'Bye-bye sub state machine'))
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm

def SMCallback(sm, event_type, state, action):
  if event_type==EVENT_STATE_ENTRY:
    #CPrint(3,(event_type,state),'-->')
    sm.l.event_messenger.put((event_type,state))

def Run(t,*args):
  import time,sys

  sm= TStateMachine(local_obj=TContainer(debug=True))
  sm.Debug= True
  sm.EventCallback= SMCallback
  sm.l.event_messenger= TSignal()
  sm.l.sub_sm= DefSubSM(t, sm.l.event_messenger)

  def Ask():
    print ' q to stop or >'
    while True:
      key= t.kbhit.KBHit()
      if key!=None:
        sm.l.key= key
        break

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].EntryAction= lambda: CPrint(1,'Hello parallel state machine!')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: t.RunSMAsThread(sm.l.sub_sm, 'SubSM')
  sm['start'].ElseAction.NextState= 'ask'

  sm.NewState('ask')
  sm['ask'].ElseAction.Condition= lambda: True
  sm['ask'].ElseAction.Action= Ask
  sm['ask'].ElseAction.NextState= 'response'

  sm.NewState('response')
  sm['response'].NewAction()
  sm['response'].Actions[-1].Condition= lambda: sm.l.key=='x'
  sm['response'].Actions[-1].NextState= 'special'
  sm['response'].NewAction()
  sm['response'].Actions[-1].Condition= lambda: sm.l.key=='q'
  sm['response'].Actions[-1].NextState= 'stop'
  sm['response'].ElseAction.Condition= lambda: True
  sm['response'].ElseAction.Action= lambda: CPrint(4,'I do not know',sm.l.key)
  sm['response'].ElseAction.NextState= 'ask'

  sm.NewState('special')
  sm['special'].EntryAction= lambda: (CPrint(1,'=============='), time.sleep(0.5))
  sm['special'].ElseAction.Condition= lambda: True
  sm['special'].ElseAction.NextState= 'ask'

  sm.NewState('stop')
  sm['stop'].EntryAction= lambda: CPrint(1,'Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  t.kbhit.Activate()

  t.RunSM(sm,'test_psm')

  t.kbhit.Deactivate()

  #while t.thread_manager.IsRunning('SubSM'):
    #print 'sm.l.sub_sm.Running',sm.l.sub_sm.Running
    #print 'sm.l.sub_sm.curr_state',sm.l.sub_sm.curr_state
    #print 't.thread_manager.IsRunning("SubSM")',t.thread_manager.IsRunning('SubSM')
    #print 'sm.l.event_messenger.queues=',sm.l.event_messenger.queues
    #if t.thread_manager.IsRunning('SubSM'):
      #t.thread_manager.Stop('SubSM')
      ##t.thread_manager.StopRequest('SubSM')

  if sm.l.sub_sm.ThreadInfo.IsRunning():
    t.thread_manager.Stop('SubSM')

  print 'sm.l.event_messenger.queues=',sm.l.event_messenger.queues
  sm.l.sub_sm.Cleanup()
  print 'sm.l.event_messenger.queues=',sm.l.event_messenger.queues
  sm.Cleanup()

  return sm.ExitStatus
