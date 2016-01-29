#! /usr/bin/env python
import os
import sys
import time
import numpy as np
import numpy.linalg as la
import roslib; roslib.load_manifest('lfd_trick')
import rospy
import tf
import base
import core_tool as ct
#For CUI
import readline
import threading

#Reload dependent modules
def ReloadModules():
  reload(base)
  reload(ct)

#Convert a vector to string
def VecToStr(vec,delim=' '):
  return delim.join(map(str,vec))


class TCUITool:

  def __init__(self):

    self.hist_file= '.trick_hist'
    try:
      readline.read_history_file(self.hist_file)
    except IOError:
      pass
    readline.parse_and_bind('tab: complete')
    self.write_history_file= readline.write_history_file

    self.t= ct.TCoreTool()

    self.done_exit_proc= False


  def __del__(self):
    self._exit('TCUITool.__del__')
    self.thread_cui.join()
    del self.t


  def _setup(self):
    self.t.Setup()

    self._exec_default()

  def _exit(self,where=''):
    with self.exit_locker:
      print 'Exiting at %r...' % where
      if not self.done_exit_proc:
        print 'Running exit process...'
        self.SaveHistory()
        self._exec_exit()
        print 'TCoreTool.Cleanup...'
        self.t.Cleanup()
        self.done_exit_proc= True
      else:
        print 'Exit process is skipped'

  def _exec_default(self):
    print 'Running _default...'
    self.t.ExecuteMotion('_default')

  def _exec_exit(self):
    print 'Running _exit...'
    self.t.ExecuteMotion('_exit')

  def SaveHistory(self):
    print 'Save history into ',self.hist_file
    self.write_history_file(self.hist_file)

  def Start(self):
    ct.CPrint(0,'Setup...')
    #self._setup()
    thread_setup= threading.Thread(name='_setup', target=self._setup)
    thread_setup.start()

    self.thread_cui= threading.Thread(name='thread_cui', target=self.Interface)
    self.thread_cui.start()

    self.exit_locker= threading.RLock()

    thread_setup.join()
    ct.CPrint(0,'Done: setup')


  def Eval(self,slist):
    xglobals= {}
    for k,v in globals().items():  xglobals[k]= v
    for k,v in ct.__dict__.items():  xglobals[k]= v
    return eval(' '.join(slist),xglobals,self.__dict__)


  def Interface(self):
    self.lastx= [0.,0.,0., 0.,0.,0.,1.]
    running= True
    while running and not rospy.is_shutdown():
      self.t.kbhit.Deactivate()
      cmd= raw_input('%s:trick or quit|%s> '%(self.t.robot.Name,self.t.robot.ArmStrS)).split()

      try:
        if len(cmd)==0:
          continue
        elif cmd[0] == 'quit' or cmd[0] == 'exit':
          running= False
          rospy.signal_shutdown('quit...')
        elif cmd[0] == 'init':
          self.t.Setup()
        elif cmd[0] == 'reload' or cmd[0] == 'reloadf':
          if cmd[0] == 'reloadf': self._exec_exit()
          self.t.Cleanup()
          old_t= self.t
          old_dict= self.t.__dict__
          ReloadModules()
          self.t= ct.TCoreTool()
          if cmd[0] == 'reload':
            for k,v in old_dict.items():
              self.t.__dict__[k]= v
          elif cmd[0] == 'reloadf':
            #self.t.Setup()
            self.t.robot= old_dict['robot']
            self.t.state_validity_checker= old_dict['state_validity_checker']
            #self.t.dparam_client_tof= old_dict['dparam_client_tof']
            self._exec_default()
          print 'Delete',old_t
          del old_t
        elif cmd[0]=='r':
          self.t.robot.Arm= ct.RIGHT
          print '%s arm is selected' % ct.LRToStr(self.t.robot.Arm)
        elif cmd[0]=='l':
          self.t.robot.Arm= ct.LEFT
          print '%s arm is selected' % ct.LRToStr(self.t.robot.Arm)
        elif cmd[0]=='set':
          vid= cmd[1]
          value= self.Eval(cmd[2:])
          self.t.__dict__[vid]= value
          print 'Set t.%s= %r' % (vid,value)
        elif cmd[0]=='calc':
          if cmd[1]=='e2q':
            args= self.Eval(cmd[2:])
            rot= tf.transformations.quaternion_from_euler(args[0], args[1], args[2])
            print 'Quaternion: ',rot
          elif cmd[1]=='e2qd':
            args= self.Eval(cmd[2:])
            e= np.radians(args)
            rot= tf.transformations.quaternion_from_euler(e[0], e[1], e[2])
            print 'Quaternion: ',rot
          elif cmd[1]=='q2e':
            args= self.Eval(cmd[2:6])
            e= tf.transformations.euler_from_quaternion(args)
            print 'Euler: ',e
          elif cmd[1]=='q2ed':
            args= self.Eval(cmd[2:])
            e= tf.transformations.euler_from_quaternion(args)
            print 'Euler: ',np.degrees(e)
          else:
            res= self.Eval(cmd[1:])
            print res
        elif cmd[0]=='c':
          res= self.Eval(cmd[1:])
          print res
        elif cmd[0]=='var':
          val= self.Eval(cmd[2:])
          self.__dict__[cmd[1]]= val
          print 'New variable ',cmd[1],' : ',self.__dict__[cmd[1]]
        elif cmd[0]=='bp':
          if len(cmd)==1 or cmd[1]=='show':
            print 'Base points are: ',self.t.base_x
          elif cmd[1]=='addx':
            x= self.t.robot.FK()
            self.t.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',self.t.base_x[cmd[2]]
          elif cmd[1]=='addxe':
            xe= self.t.robot.FK(x_ext=self.t.GetAttr('wrist_%s'%self.t.robot.ArmStrs,'lx'))
            self.t.base_x[cmd[2]]= xe
            print 'Added to base points[',cmd[2],']: ',self.t.base_x[cmd[2]]
          elif cmd[1]=='add':
            x=[0.0]*7
            x[0:7]= self.Eval(cmd[3:])
            self.t.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',self.t.base_x[cmd[2]]
          else:
            print 'Invalid bp-command line: ',' '.join(cmd)
        elif cmd[0]=='lastx':
          if len(cmd)==1 or cmd[1]=='show':
            print 'Last x: ',ct.ToList(self.lastx)
          elif cmd[1]=='set':
            args= self.Eval(cmd[2:])
            if len(args)>=3:  self.lastx[0:3]= args[0:3]
            if len(args)>=7:  self.lastx[3:7]= args[3:7]
          elif cmd[1]=='arlocal':
            id= int(cmd[2])
            self.t.UpdateAR(id)
            if self.t.IsARAvailable(id):
              l_x= ct.TransformLeftInv(self.t.ARX(id), self.lastx)
              print 'Local pose of last x on AR ',id,': ',ct.ToList(l_x)
            else:
              print 'Error: AR marker not found: ',id
          elif cmd[1]=='bplocal':
            id= cmd[2]
            if id in self.t.base_x:
              l_x= ct.TransformLeftInv(self.t.BPX(id), self.lastx)
              print 'Local pose of last x on base point ',id,': ',ct.ToList(l_x)
            else:
              print 'Error: base point not found: ',id
          else:
            print 'Invalid lastx-command line: ',' '.join(cmd)
        elif cmd[0]=='q':
          q= self.t.robot.Q()
          print self.t.robot.ArmStr,'arm joint angles: ',ct.ToList(q)
        elif cmd[0]=='x':
          self.lastx= self.t.robot.FK()
          print self.t.robot.ArmStr,'arm endeffector position: ',ct.ToList(self.lastx)
        elif cmd[0]=='xe':
          self.lastx= self.t.robot.FK(x_ext=self.t.GetAttr('wrist_%s'%self.t.robot.ArmStrs,'lx'))
          print self.t.robot.ArmStr,'arm extended-endeffector position: ',ct.ToList(self.lastx)
        elif cmd[0]=='ext':
          print self.t.robot.ArmStr,'arm extension: ',self.t.GetAttr('wrist_%s'%self.t.robot.ArmStrs,'lx')
        elif cmd[0]=='moveq':
          args= self.Eval(cmd[1:])
          if len(args)==2:
            dt= 2.0
            q_trg= [0.0]*7
            dt= float(args[0])
            q_trg[0:7]= args[1]
            self.t.robot.MoveToQ(q_trg,dt)
          else:
            print 'Invalid moveq-arguments: ',' '.join(cmd)
        elif cmd[0]=='movex' or cmd[0]=='imovex':
          args= self.Eval(cmd[1:])
          if len(args)==2 and (len(args[1])==3 or len(args[1])==7):
            dt= 2.0
            x_trg= self.t.robot.FK()
            dt= float(args[0])
            if len(args[1])>=3:  x_trg[0:3]= args[1][0:3]
            if len(args[1])==7:  x_trg[3:7]= args[1][3:7]
            if cmd[0]=='movex':
              self.t.robot.MoveToX(x_trg,dt)
            else:
              self.t.robot.MoveToXI(x_trg,dt)
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='movexe' or cmd[0]=='imovexe':
          args= self.Eval(cmd[1:])
          if len(args)==2 and (len(args[1])==3 or len(args[1])==7):
            dt= 2.0
            x_trg= self.t.robot.FK()
            dt= float(args[0])
            if len(args[1])>=3:  x_trg[0:3]= args[1][0:3]
            if len(args[1])==7:  x_trg[3:7]= args[1][3:7]
            if cmd[0]=='movexe':
              self.t.robot.MoveToX(x_trg,dt,self.t.GetAttr('wrist_%s'%self.t.robot.ArmStrs,'lx'))
            else:
              self.t.robot.MoveToXI(x_trg,dt,self.t.GetAttr('wrist_%s'%self.t.robot.ArmStrs,'lx'))
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='grip':
          args= self.Eval(cmd[1:])
          pos= float(args[0])
          max_effort= 20
          if len(args)>=2:  max_effort= float(args[1])
          self.t.robot.MoveGripper(pos, max_effort)
        elif cmd[0]=='head':
          args= self.Eval(cmd[1:])
          dt= float(args[0])
          pan= float(args[1])
          tilt= float(args[2])
          self.t.robot.MoveHead(pan, tilt, dt)
        elif cmd[0]=='ar':
          args= self.Eval(cmd[1:])
          id= int(args)
          self.t.UpdateAR(id)
          if self.t.IsARAvailable(id):
            print 'AR ',id,' pose in torso-frame: ',self.t.ARX(id)
        elif cmd[0]=='arraw':
          args= self.Eval(cmd[1:])
          id= int(args)
          self.t.UpdateAR(id)
          if self.t.IsARObserved(id):
            print 'AR ',id,' pose in raw: ',self.t.ar_x[id]
        #elif cmd[0]=='m':
          #if len(cmd)>2:  args= self.Eval(cmd[2:])
          #else:  args= ()
          #if not isinstance(args,tuple):  args= (args,)
          #res= self.t.ExecuteMotion(cmd[1], *args)
          #if res!=None:  print 'Result:',res
        #elif cmd[0]=='shake':
          #if len(cmd)>=3:    self.t.ShakeGripper(shake_Hz=float(cmd[1]),shake_width=float(cmd[2]))
          #elif len(cmd)==2:  self.t.ShakeGripper(shake_Hz=float(cmd[1]))
          #else:              self.t.ShakeGripper()
        else:
          if len(cmd)==2 and (cmd[1]=='-help' or cmd[1]=='--help'):
            sub= self.t.LoadMotion(cmd[0])
            if sub:
              print sub.Help()
          else:
            if len(cmd)>1:  args= self.Eval(cmd[1:])
            else:  args= ()
            if not isinstance(args,tuple):  args= (args,)
            res= self.t.ExecuteMotion(cmd[0], *args)
            if res!=None:  print 'Result:',res
      except Exception as e:
        ct.PrintException(e,' caught in CUI')
        c1= ct.ACol.I(4)
        c2= ct.ACol.I(1)
        ce= ct.ACol.I()
        print '%sCheck the command line: %s%s %s' % (c1, c2,' '.join(cmd), ce)
        if len(self.t.state_context)>0:
          print '%sstate_context: %r%s' % (c1,self.t.state_context,ce)
          self.t.state_context= []
        if len(self.t.assessment_list)>0:
          print '%sassessment_list: %r%s' % (c1,self.t.assessment_list,ce)
          self.t.RemoveAllAssessments()
        self.t.kbhit.Deactivate()

    self._exit('the end of TCUITool.Interface')


if __name__ == '__main__':
  rospy.init_node('cuiToolNode')
  #joy_kind = rospy.get_param('~joy_kind', 'default')
  #base_path = rospy.get_param('~base_path', 'data/bagfiles')
  cui= TCUITool()
  cui.Start()
  #r = rospy.Rate(150)
  #while not rospy.is_shutdown():
    #cui.controlCart()
    #print '.',
    #r.sleep()
  rospy.spin()

