#! /usr/bin/env python
import numpy as np
import numpy.linalg as la
import roslib; roslib.load_manifest('lfd_trick')
import rospy
import std_msgs.msg
import std_srvs.srv
import ar_track_alvar_msgs.msg
import lfd_vision.msg
import lfd_vision.srv
import lfd_sim.msg
import lfd_sim.srv
from base import *


class TCoreTool(TROSUtil):

  def __init__(self):
    TROSUtil.__init__(self)

    self.robot= None
    if   ROS_ROBOT in ('PR2','PR2_SIM'):  self.robot= TRobotPR2()
    elif ROS_ROBOT in ('Baxter','Baxter_SIM'):  self.robot= TRobotBaxter()
    else:  self.robot= TFakeRobot()

    #self.state_validity_checker= TStateValidityChecker()
    self.state_validity_checker= TStateValidityCheckerMI()

    self.motion_path_prefix= 'motions.'
    self.loaded_motions= []

    #Thread locker for self.attributes:
    self.attr_locker= threading.RLock()

    self.base_x= {}

    #Attributes of objects
    self.attributes= {}

    #Parameter database
    self.database= []

    #Dictionary of name: active state machine:
    self.active_sm= {}

    #Dictionary of name: assessment model:
    self.assessment_list= {}
    #List of current state machine's states:
    #  Note: Hierarchal state machine is considered.
    #  In order to distinguish a motion script (e.g. 'pour') and
    #  a state machine inside the script, we add 'sm' to context
    #  when we are in the state machine.
    self.state_context= []

    #Detecting user's keyboard hit
    #assuming to be used in executing a motion script
    #to interact with the robot (e.g. pointing out its bad manner)
    self.kbhit= TKBHit(activate=False)

    #Thread list
    self.thread_manager= TThreadManager()
    #Visualization list; element should be a TSimpleVisualizer
    self.viz= TContainer()

    #Cache of modules loaded by LoadMotion  (Is this still used???)
    self.m= TContainer()

    #Store callback functions
    self.callback= TContainer()


    #Communication with flow_analyzer node
    self.AddPub('fla_bb', '/flow_analyzer/indexed_bb', lfd_vision.msg.IndexedBoundingBox)
    self.callback.fla_bb_counts= None
    self.AddSub('fla_counts', '/flow_analyzer/bb_counts', lfd_vision.msg.Int32Array, self.FLABBCountsCallback)

    #[id]=[pose x,y,z,qx,qy,qz,qw]
    self.ar_markers= {}
    self.ar_marker_frame_id= ""
    self.callback.ar_marker_observer= None
    self.using_ar_markers_filter= True
    self.ar_markers_filter_ratio= 0.2
    #Sensor pose in robot frame
    self.x_sensor= []
    #Marker ID attached on the (left)-wrist link to adjust the marker error
    self.ar_adjust_m_id= 3
    self.ar_adjust_arm= 1  #Left arm
    self.l_x_m_wrist= []  #Marker pose in the wrist frame
    self.default_ar_adjust_ratio= 0.02  #Update ratio
    self.ar_adjust_ratio= self.default_ar_adjust_ratio  #Update ratio (actually used)
    self.ar_adjust_err= 0.0  #Error norm is stored
    self.br= tf.TransformBroadcaster()
    self.AddSub('ar', '/ar_pose_marker', ar_track_alvar_msgs.msg.AlvarMarkers, self.ARMarkerObserver)

    #self.viz= TSimpleVisualizer()
    self.viz.marker= TSimpleVisualizer(rospy.Duration(1.0), name_space='visualizer_marker')


  def __del__(self):
    #self.stopRecord()
    self.Cleanup()
    TROSUtil.__del__(self)
    print 'TCoreTool: done',self

  def Setup(self):
    res= []
    ra= lambda r: res.append(r)
    ra(self.robot.Init())
    if self.robot.Name!='NoRobot':
      ra(self.state_validity_checker.Init(self.robot))

    if False in res:
      CPrint(4, 'Failed to setup. Run init again.')

    #Dynamically reconfigure the sentis m100 ToF sensor
    #self.dparam_client_tof= dynamic_reconfigure.client.Client('/sentis_tof_m100_1')

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    #Check the thread lockers status:
    print 'Count of attr_locker:',self.attr_locker._RLock__count

    self.thread_manager.StopAll()

    for k in self.callback.keys():
      print 'Stopping callback %r...' % k,
      self.callback[k]= None  #We do not delete
      print 'ok'

    for k in self.viz.keys():
      print 'Stop visualizing %r...' % k,
      del self.viz[k]
      print 'ok'

    TROSUtil.Cleanup(self)

    for k in self.m.keys():
      print 'Deleting motion script cache %r...' % k,
      del self.m[k]
      print 'ok'

  @staticmethod
  def DataBaseDir():
    return '%s/data/' % (os.environ['HOME'])

  @staticmethod
  def LogFileName(prefix, timestamp=None, suffix='.dat'):
    if timestamp==None:  timestamp= TimeStr('short2')
    location= '{base}tmp'.format(base=TCoreTool.DataBaseDir())
    if not os.path.exists(location):
      CPrint(2,'Directory for log files does not exist:',location)
      CPrint(2,'Want to create? (if No, we will use /tmp)')
      if AskYesNo():  os.makedirs(location)
      else:  location= '/tmp'
    return '%s/%s%s%s' % (location, prefix, timestamp, suffix)


  #Wrapping AskYesNo to be compatible with kbhit:
  def AskYesNo(self):
    if self.kbhit.IsActive():
      self.kbhit.Deactivate()
      res= AskYesNo()
      self.kbhit.Activate()
      return res
    else:
      return AskYesNo()

  #Wrapping AskGen to be compatible with kbhit:
  def AskGen(self,*argv):
    if self.kbhit.IsActive():
      self.kbhit.Deactivate()
      res= AskGen(*argv)
      self.kbhit.Activate()
      return res
    else:
      return AskGen(*argv)


  def GetAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        d= d[n]
      if isinstance(d,(int,bool,float)):  return d
      #return copy.deepcopy(d)  #TODO: if we have a 'lazydeepcopy', the performance will be improved
      return d  #TODO: if we have a 'lazydeepcopy', the performance will be improved
  def GetAttrOr(self,default,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        if type(d)!=dict or (not n in d):
          return default
        d= d[n]
      if isinstance(d,(int,bool,float)):  return d
      #return copy.deepcopy(d)  #TODO: if we have a 'lazydeepcopy', the performance will be improved
      return d  #TODO: if we have a 'lazydeepcopy', the performance will be improved
  def HasAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        if type(d)!=dict or (not n in d):
          return False
        d= d[n]
      return True
  #Set a value to attributes; last element of keys_value is the value
  def SetAttr(self,*keys_value):
    assert(len(keys_value)>=2)
    with self.attr_locker:
      d= self.attributes
      for n in keys_value[0:-1]:
        if type(d)==dict and n in d:
          pass
        elif type(d)==dict:
          d[n]= {}
        else:  #i.e. type(d)!=dict (Note: at first, d==self.attributes should be a dict)
          last_d[last_n]= {}
          d= last_d[last_n]
          d[n]= {}
        last_d= d
        last_n= n
        d= d[n]
      last_d[last_n]= keys_value[-1]
  #Add a dictionary value to attributes (only the assignment behavior is different from SetAttr);
  #last element of keys_dvalue is the (dict) value
  def AddDictAttr(self,*keys_dvalue):
    dict_value= keys_dvalue[-1]
    assert(type(dict_value)==dict)
    assert(len(keys_dvalue)>=1)
    with self.attr_locker:
      d= self.attributes
      if len(keys_dvalue)==1:
        InsertDict(d, dict_value)
        return
      for n in keys_dvalue[0:-1]:
        if type(d)==dict and n in d:
          pass
        elif type(d)==dict:
          d[n]= {}
        else:  #i.e. type(d)!=dict (Note: at first, d==self.attributes should be a dict)
          last_d[last_n]= {}
          d= last_d[last_n]
          d[n]= {}
        last_d= d
        last_n= n
        d= d[n]
      InsertDict(last_d[last_n], dict_value)
  def DelAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      if len(keys)==0:  return False
      for n in keys:
        if type(d)!=dict or (not n in d):
          return False
        last_d= d
        last_n= n
        d= d[n]
      del last_d[last_n]
      return True


  #Activate a state machine
  def RunSM(self,state_machine,sm_name):
    self.active_sm[sm_name]= state_machine
    try:
      #state_machine.EventCallback= self.SMCallback
      state_machine.Run()
    except Exception as e:
      PrintException(e,' caught in RunSM')
      state_machine.Running= False
      state_machine.SetExitStatus(FAILURE_FORCEDQUIT)  #WARNING:a bit strange (maybe it will overwrite an existing failure flag which is more informative than this flag)
      for name,sm in self.active_sm.iteritems():
        CPrint('Stopping state machine %s ..'%name)
        sm.SetExceptionFlag()
    del self.active_sm[sm_name]

  #Activate a state machine as a thread
  #TThreadInfo object is assigned to state_machine.ThreadInfo
  def RunSMAsThread(self,state_machine,sm_name):
    target= lambda th_info: (
        state_machine.__dict__.__setitem__('ThreadInfo',th_info),
        self.RunSM(state_machine,sm_name) )
    self.thread_manager.Add(name=sm_name, target=target)


  #Memorize a database data as a bad data to a memory.
  #Return memorized or not.
  #a: TAssessment object that should contain DBIndex, MarkedBad additionally.
  #mem_key: List of keys of memory.
  #assess_key: Key to register in database.
  #assess_info: Dictionary containing assessment information.
  #context: Context where the assessment is done.
  #ask: Ask to the operator if we memorize the data.
  def MemorizeAssessment(self, a, mem_key, assess_key, assess_info, context, ask=True):
    if not a.MarkedBad:
      CPrint(3, 'Memorizing No. %i data into [memory][%s]' % (a.DBIndex, ']['.join(mem_key)))
      if ask:  CPrint(3, 'OK?')
      if not ask or self.AskYesNo():
        assess_info['context']= copy.deepcopy(context)
        self.database[a.DBIndex][DB_ASSESSMENT][assess_key]= assess_info
        new_data= self.GetAttrOr([], 'memory',*mem_key)+[self.database[a.DBIndex]]
        self.SetAttr(*(['memory']+mem_key+[new_data]))
        a.MarkedBad= True
        return True
    return False


  def RegisterAssessment(self, assessment):
    if assessment.Name in self.assessment_list.iteritems():
      CPrint(4, 'Assessment %r already exists' % assessment.Name)
      raise
    assessment.RegisteredContext= copy.deepcopy(self.state_context)
    self.assessment_list[assessment.Name]= assessment
    if assessment.CallbackType&TAssessment.TIMER_CALLBACK:
      thread_name= 'timer_assessment_'+assessment.Name
      assessment.TimerThreadName= thread_name
      target= lambda th_info: self.TimerCallback(th_info, assessment)
      self.thread_manager.Add(name=thread_name, target=target)

  def RemoveAssessment(self,name):
    if name in self.assessment_list:
      if self.assessment_list[name].CallbackType&TAssessment.TIMER_CALLBACK:
        self.thread_manager.Stop(self.assessment_list[name].TimerThreadName)
      del self.assessment_list[name]

  def RemoveAllAssessments(self):
    for name,a in self.assessment_list.iteritems():
      print 'Removing assessment %r= %r...' % (a.Name, a)
      if a.CallbackType&TAssessment.TIMER_CALLBACK:
        self.thread_manager.Stop(a.TimerThreadName)
    self.assessment_list= {}

  def TimerCallback(self, th_info, assessment):
    while th_info.IsRunning():
      time.sleep(assessment.TimeStep)
      res= assessment.Assess(assessment, TAssessment.EVENT_TIMER, self.state_context, None)
      if res==TAssessment.COMPLETED:
        self.RemoveAssessment(assessment.Name)
      elif res==TAssessment.EXCEPTION:
        CPrint(3, 'Requesting exception..')
        #FIXME
        CPrint(4, 'FIXME: exception in a thread does not stop the main thread!!!')
        raise Exception('Exception requested from assessment(s) in TCoreTool.TimerCallback')

  def SMCallback(self, sm, event_type, state, action):
    if event_type==EVENT_SM_ENTRY:
      self.state_context.append('sm')
      event_type2= TAssessment.EVENT_ENTRY
    elif event_type==EVENT_SM_EXIT:
      assert(self.state_context[-1]=='sm')
      event_type2= TAssessment.EVENT_EXIT
    elif event_type==EVENT_STATE_ENTRY:
      self.state_context.append(state)
      event_type2= TAssessment.EVENT_ENTRY
    elif event_type==EVENT_STATE_STAY:
      assert(self.state_context[-1]==state)
      event_type2= TAssessment.EVENT_STAY
    elif event_type==EVENT_STATE_EXIT:
      assert(self.state_context[-1]==state)
      event_type2= TAssessment.EVENT_EXIT
    else:
      CPrint(4, 'Unrecognized event:',event_type)
      raise

    #kbhit_active= self.kbhit.IsActive()
    #key= self.kbhit.KBHit() if kbhit_active else None
    #if kbhit_active:  self.kbhit.Deactivate()

    del_list= []
    exception_req= False
    for name,a in self.assessment_list.iteritems():
      if a.CallbackType&TAssessment.STATE_CALLBACK:
        res= a.Assess(a, event_type2, self.state_context, sm)
        if res==TAssessment.COMPLETED:  del_list.append(name)
        elif res==TAssessment.EXCEPTION:  exception_req= True
    for name in reversed(del_list):
      self.RemoveAssessment(name)

    if exception_req:
      CPrint(3, 'Requesting exception..')
      raise Exception('Exception requested from assessment(s) in TCoreTool.SMCallback')

    #if kbhit_active:  self.kbhit.Activate()

    if event_type==EVENT_SM_EXIT:
      self.state_context.pop()
    elif event_type==EVENT_STATE_EXIT:
      self.state_context.pop()


  def FLABBCountsCallback(self, msg):
    if self.callback.fla_bb_counts<>None:
      self.callback.fla_bb_counts(msg)

  def ARMarkerObserver(self, msg):
    for m in msg.markers:
      is_new= False
      if m.id not in self.ar_markers:
        self.ar_markers[m.id]= copy.deepcopy(m.pose)
        is_new= True
      self.ar_markers[m.id].header= copy.deepcopy(m.header)
      x_raw= copy.deepcopy(m.pose.pose)
      xp= x_raw.position
      xq= x_raw.orientation
      pose= [xp.x, xp.y, xp.z,  xq.x, xq.y, xq.z, xq.w]
      if (not is_new) and self.using_ar_markers_filter:
        self.ar_markers[m.id].pose= AverageX(self.ar_markers[m.id].pose, pose, self.ar_markers_filter_ratio)
      else:
        self.ar_markers[m.id].pose= pose
      self.ar_marker_frame_id= m.header.frame_id

      #Visualize the marker
      if len(self.x_sensor)==7:
        self.viz.marker.AddMarker(self.ARX(m.id), scale=[0.04,0.04,0.012], mid=m.id, rgb=self.viz.marker.ICol(m.id))
        self.viz.marker.AddArrow(self.ARX(m.id), scale=[0.05,0.004,0.004], mid=10000+m.id, rgb=self.viz.marker.ICol(m.id))

      #Adjustment with the marker on the wrist
      if len(self.l_x_m_wrist)==7:
        x_m_wrist= self.robot.FK(x_ext=self.l_x_m_wrist, arm=self.ar_adjust_arm)

        #Visualize the marker on the wrist
        self.viz.marker.AddMarker(x_m_wrist, scale=[0.04,0.04,0.012], mid=9999, rgb=self.viz.marker.ICol(1), alpha=0.5)
        self.viz.marker.AddArrow(x_m_wrist, scale=[0.05,0.004,0.004], mid=10000+9999, rgb=self.viz.marker.ICol(1), alpha=0.5)

        if m.id==self.ar_adjust_m_id:
          x_sensor2= TransformRightInv(x_m_wrist,self.ARXraw(self.ar_adjust_m_id))
          del self.ar_markers[self.ar_adjust_m_id]

          n_x= QToRot(self.x_sensor[3:7])[:,0]
          n_x2= QToRot(x_sensor2[3:7])[:,0]
          ang_nx= math.acos(np.dot(n_x,n_x2)/(la.norm(n_x)*la.norm(n_x2)))
          #Acceptable angle is 30 [deg], otherwise it is considered as a noise
          if ang_nx<30.0/180.0*math.pi:
            self.ar_adjust_err= la.norm(np.array(self.x_sensor)-x_sensor2)
            self.x_sensor= AverageX(self.x_sensor, x_sensor2, self.ar_adjust_ratio)
            #print 'self.ar_adjust_err=',self.ar_adjust_err

    #Broadcast the tf between the self.robot.BaseFrame and the external camera
    if len(self.x_sensor)==7:
      self.br.sendTransform(self.x_sensor[0:3],self.x_sensor[3:],
          rospy.Time.now(),
          self.ar_marker_frame_id,
          self.robot.BaseFrame)

    if self.callback.ar_marker_observer:
      self.callback.ar_marker_observer()




  def ARXraw(self,id):
    #x_raw= self.ar_markers[id].pose
    #xp= x_raw.position
    #xq= x_raw.orientation
    #return [xp.x, xp.y, xp.z,  xq.x, xq.y, xq.z, xq.w]
    return self.ar_markers[id].pose

  def ARXtime(self,id):
    return self.ar_markers[id].header.stamp

  def ARX(self,id):
    x= self.ARXraw(id)
    return Transform(self.x_sensor, x)

  def BPX(self,id):
    return self.base_x[id]


  #Load external motion script written in python,
  #which is imported as a module to this script, so we can share the memory
  def LoadMotion(self, fileid):
    modname= self.motion_path_prefix+fileid
    try:
      sub= __import__(modname,globals(),locals(),modname,-1)
      if modname in self.loaded_motions:
        reload(sub)
      else:
        self.loaded_motions.append(modname)
    except ImportError:
      print 'Cannot import motion file: ',modname
      sub= None
    return sub

  #Execute external motion script written in python,
  #which is imported as a module to this script, so we can share the memory
  def ExecuteMotion(self, fileid, *args):
    sub= self.LoadMotion(fileid)
    if sub:
      return sub.Run(self,*args)


if __name__ == '__main__':
  print 'Note: run cui_tool.py'



