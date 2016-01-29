#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow sensor using the ToF depth camera (sentis m100).  lfd_vision/flow_analyzer_node should be running.
  Usage: flow_sensor_depth'''

#Get IndexedBoundingBox
def GetBB(x_o, bb_o, x_tof):
  x_o_bb= Transform(x_o, bb_o['center'])
  bb= lfd_vision.msg.IndexedBoundingBox()
  bb.active= True
  bb.pose= XToGPose(TransformLeftInv(x_tof, x_o_bb))
  bb.dimensions= bb_o['dim']
  return bb, x_o_bb
#Get IndexedBoundingBox that is shifted above (z-axis) the given bounding box
def GetAboveBB(x_o, bb_o, x_tof, new_height):
  dim_o_bb= Vec(bb_o['dim'])
  x_o_bb= Transform(x_o, bb_o['center'])
  p_o_bb,R_o_bb= XToPosRot(x_o_bb)
  x_bb= PosRotToX(p_o_bb+(0.5*dim_o_bb[2]+0.5*new_height)*R_o_bb[:,2], R_o_bb)

  bb= lfd_vision.msg.IndexedBoundingBox()
  bb.active= True
  bb.pose= XToGPose(TransformLeftInv(x_tof, x_bb))
  bb.dimensions= [dim_o_bb[0],dim_o_bb[1],new_height]
  return bb, x_bb

class TFlowSensorDepth:
  #source: source container id.
  #receiver: receiving container id.
  #get_x_tof: function to get x_tof (pose of tof sensor in torso frame)
  def __init__(self, t, source, receiver, get_x_tof):
    self.t= t  #TCoreTool
    self.source= source
    self.receiver= receiver
    self.get_x_tof= get_x_tof
    self.x_tof= []
    self.callback_running= False
  def __del__(self):
    self.Deactivate()
    print 'Deleted TFlowSensorDepth',self
  def Activate(self, frame_rate=40):
    #TODO: 1. use AddSrvP.  2. return False if failure in AddSrvP.

    #self.t.dparam_client_tof.update_configuration({'Frame_Rate':frame_rate})
    if 'tof_set_frame_rate' not in self.t.srvp:
      print 'Waiting for service flow_analyzer/set_frame_rate.'
      rospy.wait_for_service('flow_analyzer/set_frame_rate')
      self.t.srvp.tof_set_frame_rate= rospy.ServiceProxy('flow_analyzer/set_frame_rate', lfd_vision.srv.SetFrameRate, persistent=False)
    self.t.srvp.tof_set_frame_rate(frame_rate)

    if 'tof_set_bbeq' not in self.t.srvp:
      print 'Waiting for service flow_analyzer/set_bbeq.'
      rospy.wait_for_service('flow_analyzer/set_bbeq')
      self.t.srvp.tof_set_bbeq= rospy.ServiceProxy('flow_analyzer/set_bbeq', lfd_vision.srv.SetBBEquation, persistent=False)

    #Get bounding box (receiver)
    if self.t.HasAttr(self.receiver):
      self.t.ExecuteMotion('infer2', {},[self.receiver],'bound_box')
      self.rcv_bound_box= self.t.GetAttr(self.receiver, 'bound_box')

    #Get bounding box (source)
    if self.t.HasAttr(self.source):
      self.t.ExecuteMotion('infer2', {},[self.source],'bound_box')
      self.src_bound_box= self.t.GetAttr(self.source, 'bound_box')

    #source container (above) - receiving container
    bbeq= lfd_vision.msg.IndexedBBEquation()
    bbeq.index= 4
    bbeq.active= True
    bbeq.op= bbeq.OP_DIFFERENCE
    bbeq.idx_l= 2
    bbeq.idx_r= 1
    self.t.srvp.tof_set_bbeq(bbeq)

    self.viz= TSimpleVisualizer(rospy.Duration(1.0), name_space='visualizer_flow_sensor')

    self.t.callback.fla_bb_counts= self.Callback

  def Deactivate(self):
    self.t.callback.fla_bb_counts= None
    while self.callback_running:  time.sleep(0.01)
    if 'viz' in self.__dict__:  del self.viz
    #self.t.dparam_client_tof.update_configuration({'Frame_Rate':1})
    if 'tof_set_frame_rate' in self.t.srvp:  self.t.srvp.tof_set_frame_rate(1)
    self.UnsetBB()

  def SetBB(self):
    if not self.t.HasAttr(self.receiver):  return
    if len(self.x_tof)<>7:  return

    self.t.ExecuteMotion('infer2', {},[self.receiver],'x',[],False)
    x_rcv= self.t.GetAttr(self.receiver,'x')
    self.t.ExecuteMotion('infer2', {},[self.source],'x',[],False)
    x_src= self.t.GetAttr(self.source,'x')

    bb,x_bb= GetAboveBB(x_o=x_rcv, bb_o=self.rcv_bound_box, x_tof=self.x_tof,
                  new_height=0.04)
    bb.index= 0
    self.t.pub.fla_bb.publish(bb)
    if 'viz' in self.__dict__:
      self.viz.AddCube(x_bb, bb.dimensions, rgb=self.viz.ICol(2), alpha=0.4, mid=bb.index)

    bb,x_bb= GetBB(x_o=x_rcv, bb_o=self.rcv_bound_box, x_tof=self.x_tof)
    bb.index= 1
    self.t.pub.fla_bb.publish(bb)
    if 'viz' in self.__dict__:
      self.viz.AddCube(x_bb, bb.dimensions, rgb=self.viz.ICol(2), alpha=0.4, mid=bb.index)

    bb,x_bb= GetAboveBB(x_o=x_src, bb_o=self.src_bound_box, x_tof=self.x_tof,
                  new_height=0.04)
    bb.index= 2
    self.t.pub.fla_bb.publish(bb)
    if 'viz' in self.__dict__:
      self.viz.AddCube(x_bb, bb.dimensions, rgb=self.viz.ICol(2), alpha=0.4, mid=bb.index)

    bb,x_bb= GetBB(x_o=x_src, bb_o=self.src_bound_box, x_tof=self.x_tof)
    bb.index= 3
    self.t.pub.fla_bb.publish(bb)
    if 'viz' in self.__dict__:
      self.viz.AddCube(x_bb, bb.dimensions, rgb=self.viz.ICol(2), alpha=0.4, mid=bb.index)

  def UnsetBB(self):
    for i in range(4):
      bb= lfd_vision.msg.IndexedBoundingBox()
      bb.active= False
      bb.index= i
      self.t.pub.fla_bb.publish(bb)
  def UpdateXtof(self):
    if self.get_x_tof<>None:
      self.x_tof= self.get_x_tof()
  #Broadcast the tf between the self.t.robot.BaseFrame and the tof sensor
  def SetTF(self):
    if len(self.x_tof)==7:
      #print 'Broadcasting...',self.x_tof
      self.t.br.sendTransform(self.x_tof[0:3],self.x_tof[3:],
          rospy.Time.now(),
          'tf_sentis_tof',
          self.t.robot.BaseFrame)
  def Callback(self,msg):
    self.callback_running= True
    self.UpdateXtof()
    self.SetTF()
    self.SetBB()
    self.bb_counts= msg.data
    print self.bb_counts
    self.callback_running= False

def Run(t,*args):
  #-0.5  0.5  0.5  0.5
  get_x_tof= lambda: t.robot.FK(x_ext=[0.20,0.0,0.075, 0.5,0.5,0.5,0.5],arm=RIGHT)
  flow_sensor= TFlowSensorDepth(t, source='b53', receiver='b51', get_x_tof=get_x_tof)
  flow_sensor.Activate()
  time.sleep(20)
  flow_sensor.Deactivate()
