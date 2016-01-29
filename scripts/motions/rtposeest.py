#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control pose estimator (adjuster) with ray tracing.
  Usage:
    rtposeest COMMAND, SENSOR_TYPE [, OPTIONS]
      General usage.
      COMMAND: See following.
      SENSOR_TYPE: 'ext' (external xtion) or 'lgrip' (m100 on left gripper).
      OPTIONS: Depends on COMMAND.
    rtposeest 'create', SENSOR_TYPE, SCENE_NAME, OBJS
      Create a ray tracing scene.
      SCENE_NAME: Name of the scene.
      OBJS: List of object labels to be added.
    rtposeest 'remove', SENSOR_TYPE, SCENE_NAME
      Remove a ray tracing scene.
      SCENE_NAME: Name of the scene.
    rtposeest 'clear', SENSOR_TYPE
      Clear every ray tracing scenes.
    rtposeest
    rtposeest 'stop', SENSOR_TYPE
      Stop the pose estimator.
    rtposeest 'run', SENSOR_TYPE, OBJS [, METHODS [, UPDATE_RATIO]]
      Run the pose estimator.  Execute 'stop' to quit.
      OBJS: List of object labels whose poses are estimated.
      METHODS: List of how to optimize (default: ['xyz']*len(OBJS)).
      UPDATE_RATIO: Ratio to update the pose (default: 0.2).
    rtposeest 'once', SENSOR_TYPE, OBJ [, COUNT [, METHOD [, UPDATE_RATIO]]]
      Execute the pose estimator only a specific times.
      OBJ: Object label whose pose is estimated.
      COUNT: How many times do we execute (default: 1).
      METHOD: How to optimize (default: 'xyz').
      UPDATE_RATIO: Ratio to update the pose (default: 1.0).
  '''

def SetupServiceProxy(t, l):
  if l.sensor=='xtion':
    t.AddSrvP('xtion_remove_scene', '/rt_pose_estimator_xtion/remove_scene', lfd_vision.srv.RemoveScene, persistent=False, time_out=3.0)
    t.AddSrvP('xtion_create_scene', '/rt_pose_estimator_xtion/create_scene', lfd_vision.srv.CreateScene, persistent=False, time_out=3.0)
    t.AddSrvP('xtion_lpose_opt', '/rt_pose_estimator_xtion/lpose_opt', lfd_vision.srv.RTLabeledPoseOpt, persistent=False, time_out=3.0)
  elif l.sensor=='m100':
    t.AddSrvP('m100_remove_scene', '/rt_pose_estimator_m100/remove_scene', lfd_vision.srv.RemoveScene, persistent=False, time_out=3.0)
    t.AddSrvP('m100_create_scene', '/rt_pose_estimator_m100/create_scene', lfd_vision.srv.CreateScene, persistent=False, time_out=3.0)
    t.AddSrvP('m100_lpose_opt', '/rt_pose_estimator_m100/lpose_opt', lfd_vision.srv.RTLabeledPoseOpt, persistent=False, time_out=3.0)
    t.AddSrvP('m100_set_frame_rate', '/sentis_m100/set_frame_rate', lfd_vision.srv.SetFrameRate, persistent=False, time_out=3.0)

def LoadShapePrimitives(obj):
  primitives= []
  if 'shape_primitives' in obj:
    for prim in obj['shape_primitives']:
      primitive= lfd_vision.msg.RayTracePrimitive()
      primitive.kind= prim['kind']
      primitive.param= prim['param']
      primitive.pose= XToGPose(prim['pose'])
      primitives.append(primitive)
  return primitives

def UpdateFromLPoseOptResult(msg, t, l, update_ratio, threshold, eval_desc=None):
  x_o1= Transform(l.rtposeest_x_sensor(),GPoseToX(msg.lpose.pose))
  if eval_desc==None:  eval_desc= TContainer()
  eval_desc.quality= msg.errors[2]
  eval_desc.depth_diff= math.sqrt(msg.errors[0])
  eval_desc.normal_diff= math.sqrt(msg.errors[1])
  #print msg.lpose.label, x_o1, msg.errors
  if eval_desc.quality>=threshold['thq'] and eval_desc.depth_diff<=threshold['thd'] and eval_desc.normal_diff<=threshold['thn']:
    eval_desc.prev_good= True
    CPrint(2,'Revision performed',msg.errors,update_ratio)
    if update_ratio!=None:
      x_o0= t.GetAttr(msg.lpose.label,'x')
      x_o2= AverageX(x_o0, x_o1, update_ratio)
      t.SetAttr(msg.lpose.label,'x', x_o2)
      return True,x_o2
    else:
      return True,x_o1
  else:
    eval_desc.prev_good= False
    CPrint(3,'Revision is ignored because of the large errors; error, threshold=',eval_desc,threshold)  #msg.errors
    return False,x_o1

#def LPoseRevisionCallback(msg, t, update_ratio, revised_trigger):
  #x_o= UpdateFromLPoseOptResult(msg, t, l, update_ratio)
  #if x_o!=None:
    #revised_trigger.put((msg.lpose.label,x_o))

#def StopPoseEstimator(t):
  #if 'lpose_rev' in t.sub:
    #t.sub.lpose_rev.unregister()
    #del t.sub.lpose_rev
  ##if 'lpose' in t.pub:
    ##t.pub.lpose.unregister()
    ##del t.pub.lpose
  ##if 'lpose_optreq' in t.pub:
    ##t.pub.lpose_optreq.unregister()
    ##del t.pub.lpose_optreq

#def SetupPoseEstimator(t):
  #StopPoseEstimator(t)
  #if 'lpose' not in t.pub:
    #t.pub.lpose= rospy.Publisher("/rt_pose_estimator/labeled_pose", lfd_vision.msg.LabeledPose)
  #if 'lpose_optreq' not in t.pub:
    #t.pub.lpose_optreq= rospy.Publisher("/rt_pose_estimator/labeled_pose_optreq", lfd_vision.msg.LabeledPoseOptReq)

#Construct an optimizer request
def OptStageA(tp,thq,thd,thn):
  ax0= PToGPoint([0.,0.,0.])
  return OptStage2(tp,ax0,ax0,[],0,0.,0.,thq,thd,thn)
def OptStage1(tp,ax1,rng,ndiv,wd,wn,thq,thd,thn):
  ax0= PToGPoint([0.,0.,0.])
  return OptStage2(tp,ax1,ax0,rng,ndiv,wd,wn,thq,thd,thn)
def OptStage2(tp,ax1,ax2,rng,ndiv,wd,wn,thq,thd,thn):
  stage= lfd_vision.msg.RayTraceOptReq1()
  stage.type= tp
  stage.axis1= ax1
  stage.axis2= ax2
  stage.range= rng
  stage.num_div= ndiv
  stage.weight_depth= wd
  stage.weight_normal= wn
  stage.th_quality= thq
  stage.th_depth_diff= thd
  stage.th_normal_diff= thn
  return stage

def PoseEstOnce(t, l, obj, update_ratio, method, prev_res=None):
  lpose_opt_request= lfd_vision.srv.RTLabeledPoseOptRequest()
  lpose_optreq= lpose_opt_request.req
  if t.robot.sensor_locker._RLock__count==0:  #FIXME: accessing row-level info. is this correct?
    t.m.infer.Run(t, {},[obj],'x',[],False)
  x_b= t.GetAttr(obj,'x')
  lsensor_x_b= TransformLeftInv(l.rtposeest_x_sensor(),x_b)  #Pose in sensor frame
  lpose_optreq.lpose.label= obj
  lpose_optreq.lpose.pose= XToGPose(lsensor_x_b)
  #if prev_res!=None and 'quality' in prev_res and prev_res.quality>=th['thq'] and prev_res.depth_diff<=th['thd'] and prev_res.normal_diff<=th['thn']:
  if prev_res!=None and 'prev_good' in prev_res and prev_res.prev_good:
    prev_good= True
    th= l.th_high1
  else:
    prev_good= False
    th= l.th_low1
  if method=='xyz':
    #lpose_optreq.stages.append(OptStageA('xyz_auto'))
    axx= PToGPoint([1.,0.,0.])
    axy= PToGPoint([0.,1.,0.])
    axz= PToGPoint([0.,0.,1.])
    if prev_good:
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high3))
      lpose_optreq.stages.append(OptStage1('lin1d',axz,    [0.05],     40,wd=5.0,wn=0.5,**l.th_high2))
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high1))
    else:
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low3))
      lpose_optreq.stages.append(OptStage1('lin1d',axz,    [0.15],     40,wd=5.0,wn=0.5,**l.th_low2))
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low1))
  elif method=='lin2dxy' or method=='lin2dxz':
    R= QToRot(lsensor_x_b[3:])
    if method=='lin2dxy':
      ax1= PToGPoint(R[:,0])  #e_x
      ax2= PToGPoint(R[:,1])  #e_y
    elif method=='lin2dxz':
      ax1= PToGPoint(R[:,0])  #e_x
      ax2= PToGPoint(R[:,2])  #e_z
    if prev_good:
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high3))
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high2))
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high1))
    else:
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low3))
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low2))
      lpose_optreq.stages.append(OptStage2('lin2d',ax1,ax2,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low1))
  elif method=='rot2dxy':
    R= QToRot(lsensor_x_b[3:])
    ax1= PToGPoint(R[:,0])  #e_x
    ax2= PToGPoint(R[:,1])  #e_y
    if prev_good:
      lpose_optreq.stages.append(OptStage2('rot2d',ax1,ax2,[0.35,0.35],40,wd=5.0,wn=1.0,**l.th_high1))
    else:
      lpose_optreq.stages.append(OptStage2('rot2d',ax1,ax2,[0.7,0.7],40,wd=5.0,wn=1.0,**l.th_low3))
  elif method=='xyzrot2dxy':
    axx= PToGPoint([1.,0.,0.])
    axy= PToGPoint([0.,1.,0.])
    axz= PToGPoint([0.,0.,1.])
    R= QToRot(lsensor_x_b[3:])
    ax1= PToGPoint(R[:,0])  #e_x
    ax2= PToGPoint(R[:,1])  #e_y
    if prev_good:
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.05,0.05],40,wd=5.0,wn=1.0,**l.th_high3))
      lpose_optreq.stages.append(OptStage1('lin1d',axz,    [0.05],     40,wd=5.0,wn=0.5,**l.th_high2))
      lpose_optreq.stages.append(OptStage2('rot2d',ax1,ax2,[0.35,0.35],40,wd=5.0,wn=1.0,**l.th_high1))
    else:
      lpose_optreq.stages.append(OptStage2('lin2d',axx,axy,[0.15,0.15],40,wd=5.0,wn=1.0,**l.th_low3))
      lpose_optreq.stages.append(OptStage1('lin1d',axz,    [0.15],     40,wd=5.0,wn=0.5,**l.th_low2))
      lpose_optreq.stages.append(OptStage2('rot2d',ax1,ax2,[0.7,0.7],40,wd=5.0,wn=1.0,**l.th_low1))
  else:
    CPrint(4,'Unknown method:',method)
    return -1
  if   l.sensor=='xtion': lpose_opt_result= t.srvp.xtion_lpose_opt(lpose_opt_request)
  elif l.sensor=='m100':  lpose_opt_result= t.srvp.m100_lpose_opt(lpose_opt_request)
  CPrint(3,'DEBUG:',method,prev_good)
  return UpdateFromLPoseOptResult(lpose_opt_result.res, t, l, update_ratio, th, prev_res)

def PoseEstLoop(th_info, t, l, objs, update_ratio, methods):
  #print '[DEBUG]StopPoseEstimator'
  #SetupPoseEstimator(t)
  #print '[DEBUG]Connecting to topics'
  #t.sub.lpose_rev= rospy.Subscriber("/rt_pose_estimator/labeled_pose_revision", lfd_vision.msg.LabeledPoseRevision, lambda msg:LPoseRevisionCallback(msg,t,l,update_ratio,revised_trigger))
  prev_res= TContainer(debug=True)
  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      for i_o in range(len(objs)):
        #print '[DEBUG]OptReq',objs[i_o]
        obj= objs[i_o]
        method= methods[i_o]
        PoseEstOnce(t, l, obj, update_ratio, method, prev_res)
        #time.sleep(10.0e-3)  #WARNING:If the pose estimator is improved much, 10ms may be too big.
  except Exception as e:
    PrintException(e, ' in rtposeest')
  #finally:
    #StopPoseEstimator(t)

def Run(t,*args):
  res= True
  sensor_type= None
  if len(args)==0:
    command= 'stop'
  else:
    command= args[0]
    if len(args)>1:
      sensor_type= args[1]
      args= args[2:]
    else:
      args= args[1:]

  l= TContainer(debug=True)
  if sensor_type=='ext':  #External Xtion:
    l.sensor= 'xtion'
    #Function to get the sensor pose
    l.rtposeest_x_sensor= lambda: t.x_sensor
    #Acceptable error threshold
    l.th_low3=  {'thq':4,'thd':math.sqrt(0.05),'thn':math.sqrt(0.5)}
    l.th_low2=  {'thq':4,'thd':math.sqrt(0.04),'thn':math.sqrt(0.4)}
    l.th_low1=  {'thq':4,'thd':math.sqrt(0.035),'thn':math.sqrt(0.35)}
    l.th_high3= {'thq':4,'thd':math.sqrt(0.03),'thn':math.sqrt(0.3)}
    l.th_high2= {'thq':4,'thd':math.sqrt(0.025),'thn':math.sqrt(0.25)}
    l.th_high1= {'thq':4,'thd':math.sqrt(0.02),'thn':math.sqrt(0.2)}
  elif sensor_type=='lgrip':  #Sentis m100 on gripper:
    l.sensor= 'm100'
    #Function to get the sensor pose
    lx_sensor_lg= t.GetAttr('wl_m100','lx')
    l.rtposeest_x_sensor= lambda: t.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
    t.viz.marker.AddMarker(l.rtposeest_x_sensor(), scale=[0.06,0.06,0.012])
    #t.viz.marker.AddArrow(l.rtposeest_x_sensor(), scale=[0.05,0.004,0.004])
    t.viz.marker.AddCoord(l.rtposeest_x_sensor(), scale=[0.07,0.002], alpha=1.0)
    #Acceptable error threshold
    l.th_low3=  {'thq':3,'thd':math.sqrt(0.25),'thn':math.sqrt(0.6)}
    l.th_low2=  {'thq':3,'thd':math.sqrt(0.15),'thn':math.sqrt(0.5)}
    l.th_low1=  {'thq':3,'thd':math.sqrt(0.10),'thn':math.sqrt(0.4)}
    l.th_high3= {'thq':4,'thd':math.sqrt(0.05),'thn':math.sqrt(0.32)}
    l.th_high2= {'thq':4,'thd':math.sqrt(0.03),'thn':math.sqrt(0.27)}
    l.th_high1= {'thq':4,'thd':math.sqrt(0.02),'thn':math.sqrt(0.25)}
    #FIXME:TODO: 'thq' should be 4 if there is no occlusion. Estimate it by occlusion estimation.

  if command=='create':
    scene_name= args[0]
    objs= args[1]
    SetupServiceProxy(t, l)
    create_scene_req= lfd_vision.srv.CreateSceneRequest()
    create_scene_req.name= scene_name
    for obj in objs:
      x_o= t.GetAttr(obj,'x')
      lsensor_x_o= TransformLeftInv(l.rtposeest_x_sensor(),x_o)  #Pose in sensor frame
      create_scene_req.models.append(lfd_vision.msg.RayTraceModel())
      create_scene_req.models[-1].label= obj
      create_scene_req.models[-1].primitives= LoadShapePrimitives(t.GetAttr(obj))
      create_scene_req.models[-1].initial_pose= XToGPose(lsensor_x_o)
    if   l.sensor=='xtion': t.srvp.xtion_create_scene(create_scene_req)
    elif l.sensor=='m100':  t.srvp.m100_create_scene(create_scene_req)

  elif command=='remove':
    scene_name= args[0]
    remove_scene_req= lfd_vision.srv.RemoveSceneRequest()
    remove_scene_req.name= scene_name
    SetupServiceProxy(t, l)
    if   l.sensor=='xtion': t.srvp.xtion_remove_scene(remove_scene_req)
    elif l.sensor=='m100':  t.srvp.m100_remove_scene(remove_scene_req)

  elif command=='clear':
    SetupServiceProxy(t, l)
    if   l.sensor=='xtion': t.srvp.xtion_remove_scene(lfd_vision.srv.RemoveSceneRequest())
    elif l.sensor=='m100':  t.srvp.m100_remove_scene(lfd_vision.srv.RemoveSceneRequest())

  elif command=='stop':
    CPrint(3, 'Stopping the pose estimator.')
    t.thread_manager.Stop(name='rtposeest_'+l.sensor)
    SetupServiceProxy(t, l)
    if l.sensor=='m100':  t.srvp.m100_set_frame_rate(2)

  elif command=='run':
    CPrint(3, 'Activating the pose estimator.')
    objs= args[0]
    methods= args[1] if len(args)>1 else ['xyz']*len(objs)
    update_ratio= args[2] if len(args)>2 else 0.2
    SetupServiceProxy(t, l)
    if l.sensor=='m100':
      t.srvp.m100_set_frame_rate(10);
      time.sleep(1.5)  #Wait sensor update
    #Delete base_marker_id registration because it conflicts with this pose estimator
    for obj in objs:
      if t.HasAttr(obj,'base_marker_id'):
        t.DelAttr(obj,'base_marker_id')
    t.m.infer= t.LoadMotion('infer2')
    t.thread_manager.Add(name='rtposeest_'+l.sensor, target=lambda th_info: PoseEstLoop(th_info, t, l, objs, update_ratio, methods))

  elif command=='once':
    obj= args[0]
    count= args[1] if len(args)>1 else 1
    method= args[2] if len(args)>2 else 'xyz'
    update_ratio= args[3] if len(args)>3 else 1.0
    SetupServiceProxy(t, l)
    if l.sensor=='m100':
      t.srvp.m100_set_frame_rate(10);
      time.sleep(1.5)  #Wait sensor update
    #Delete base_marker_id registration because it conflicts with this pose estimator
    if t.HasAttr(obj,'base_marker_id'):
      t.DelAttr(obj,'base_marker_id')
    t.m.infer= t.LoadMotion('infer2')
    x_o_data= []
    n_request= count
    n_executed= 0
    prev_res= TContainer(debug=True)
    while n_request>0:
      if n_executed>count*10:
        CPrint(4,'Number of pose estimation executions has reached a limit.')
        if l.sensor=='m100':  t.srvp.m100_set_frame_rate(2)
        #return None
        break
      print 'rtposeest.once: n_request,n_executed=',n_request,n_executed
      res,x_o= PoseEstOnce(t, l, obj, update_ratio=None, method=method, prev_res=prev_res)
      n_executed+=1
      if res==-1:  return None
      if res==True:
        x_o_data.append(x_o)
        n_request-=1
    if l.sensor=='m100':  t.srvp.m100_set_frame_rate(2)
    if len(x_o_data)>1:
      x_o1= AverageXData(x_o_data)
    elif len(x_o_data)==1:
      x_o1= x_o_data[0]
    else:
      return None
    if update_ratio!=None:
      x_o0= t.GetAttr(obj,'x')
      x_o2= AverageX(x_o0, x_o1, update_ratio)
      t.SetAttr(obj,'x', x_o2)
      return x_o2
    else:
      return x_o1

  else:
    raise Exception('Invalid command: %r'%command)
