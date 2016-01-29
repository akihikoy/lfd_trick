#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualize a specified object.
  Usage-1: viz OBJ_ID1, OBJ_ID2, ...
    Start visualization.
    OBJ_ID*: identifier of object. e.g. 'b1'
  Usage-2: viz
    Stop visualization. '''
def VizLoop(th_info, t, objs):
  if len(objs)==0:
    #th_info.Manager.StopRequest(th_info.Name)
    return
  #tmpfp= file('/tmp/m_viz_state','w')
  viz= TSimpleVisualizer(rospy.Duration(1.0), name_space='visualizer_viz')
  m_infer= t.LoadMotion('infer')
  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      oid= 0
      #tmpfp.write('########Time: %r\n' % rospy.Time.now())
      for obj in objs:
        #tmpfp.write('obj: %r\n' % obj)
        #oid= id(t.GetAttr(obj))%(2**31)
        oid+= 100
        mid= oid
        #tmpfp.write('oid: %r\n' % oid)
        #print 'p0',t.robot.sensor_locker._is_owned(),t.robot.sensor_locker._RLock__count
        if t.robot.sensor_locker._RLock__count==0:  #FIXME: accessing row-level info. is this correct?
          m_infer.Run(t, obj,'x',False)
          xw=[None,None]
          xw[RIGHT]= t.robot.FK(arm=RIGHT)
          xw[LEFT]= t.robot.FK(arm=LEFT)
        else:
          xw=[None,None]
        x_o= t.GetAttrOr(None, obj,'x')
        if x_o==None:
          #tmpfp.write('Failed to infer x_o\n')
          pass
        else:
          #tmpfp.write('x_o: %r\n' % x_o)
          primitives= t.GetAttrOr(None, obj,'grab_primitives')
          if primitives!=None:
            alpha= 0.5
            for prm in primitives:
              if prm['kind']=='pkCylinder':
                p_1= Transform(x_o,prm['p1'])
                p_2= Transform(x_o,prm['p2'])
                mid= viz.AddCylinder(p_1,p_2, prm['width'], alpha=alpha, rgb=viz.ICol(5), mid=mid)
              elif prm['kind']=='pkCube':
                x_center= Transform(x_o,prm['x_center'])
                mid= viz.AddCube(x_center, scale=prm['dims'], alpha=alpha, rgb=viz.ICol(5), mid=mid)
          primitives= t.GetAttrOr(None, obj,'shape_primitives')
          if primitives!=None:
            alpha= 0.6
            for prm in primitives:
              if prm['kind'] in ('rtpkCylinder','rtpkHalfCylinder'):
                #p_1= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,-0.5*prm['param'][1]])
                #p_2= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,+0.5*prm['param'][1]])
                p_1= Transform(x_o,Transform(prm['pose'],[0.,0.,-0.5*prm['param'][1]]))
                p_2= Transform(x_o,Transform(prm['pose'],[0.,0.,+0.5*prm['param'][1]]))
                mid= viz.AddCylinder(p_1,p_2, 2.0*prm['param'][0], alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind'] in ('rtpkTube','rtpkHalfTube'):
                #p_1= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,-0.5*prm['param'][2]])
                #p_2= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,+0.5*prm['param'][2]])
                p_1= Transform(x_o,Transform(prm['pose'],[0.,0.,-0.5*prm['param'][2]]))
                p_2= Transform(x_o,Transform(prm['pose'],[0.,0.,+0.5*prm['param'][2]]))
                mid= viz.AddCylinder(p_1,p_2, 2.0*prm['param'][0], alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind']=='rtpkCuboid':
                x_center= Transform(x_o,prm['pose'])
                mid= viz.AddCube(x_center, scale=2.0*Vec(prm['param']), alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind']=='rtpkRectTube':
                x_center= Transform(x_o,prm['pose'])
                mid= viz.AddCube(x_center, scale=2.0*Vec(prm['param'][:3]), alpha=alpha, rgb=viz.ICol(oid), mid=mid)
          l_x_pour_e= t.GetAttrOr(None, obj,'l_x_pour_e')
          if l_x_pour_e!=None:
            x_pour_e= Transform(x_o,l_x_pour_e)
            mid= viz.AddSphere(x_pour_e, scale=[0.008]*3, rgb=viz.ICol(1), alpha=0.7, mid=mid)
            mid= viz.AddCoord(x_pour_e, scale=[0.05,0.002], alpha=1.0, mid=mid)
          l_x_pour_l= t.GetAttrOr(None, obj,'l_x_pour_l')
          if l_x_pour_l!=None:
            x_pour_l= Transform(x_o,l_x_pour_l)
            mid= viz.AddSphere(x_pour_l, scale=[0.008]*3, rgb=viz.ICol(2), alpha=0.7, mid=mid)
            mid= viz.AddCoord(x_pour_l, scale=[0.05,0.002], alpha=1.0, mid=mid)
          l_p_pour_e_set= t.GetAttrOr(None, obj,'l_p_pour_e_set')
          if l_p_pour_e_set!=None:
            p_pour_e_set= map(lambda p: Transform(x_o,p), l_p_pour_e_set)
            #print p_pour_e_set
            mid= viz.AddPoints(p_pour_e_set, scale=[0.008,0.008], rgb=viz.ICol(0), alpha=1.0, mid=mid)
            mid= viz.AddPolygon(p_pour_e_set, scale=[0.004], rgb=[1,0.5,0.5], alpha=1.0, mid=mid)
          l_x_grab= t.GetAttrOr(None, obj,'l_x_grab')
          if l_x_grab!=None:
            x_g= Transform(x_o,l_x_grab)
            mid= viz.AddCube(x_g, scale=[0.06,0.04,0.01], rgb=viz.ICol(5), alpha=0.7, mid=mid)
            mid= viz.AddArrow(x_g, scale=[0.06,0.003,0.003], rgb=viz.ICol(5), alpha=0.7, mid=mid)
          #End of: if x_o==None else
        #tmpfp.write('mid: %r\n' % mid)
        #End of: for obj in objs
      #Visualize gripper models:
      for arm in (LEFT,RIGHT):
        #xw= t.robot.FK(arm=arm)
        if xw[arm] is None:  continue
        mid= viz.AddCoord(xw[arm], scale=[0.03,0.002], alpha=1.0, mid=mid)
        lw_xe= t.GetAttr('wrist_'+LRToStrs(arm),'lx')
        xe= Vec(Transform(xw[arm],lw_xe))
        ex,ey,ez= RotToExyz(QToRot(xe[3:]))
        #mid= viz.AddCube(xe, [0.1,0.06,0.04], rgb=viz.ICol(arm), alpha=0.5, mid=mid)
        mid= viz.AddCube(xe-((0.06*ex).tolist()+[0.0]*4), [0.12,0.08,0.02], rgb=viz.ICol(3), alpha=0.5, mid=mid)
        mid= viz.AddCube(xe+((0.04*ey).tolist()+[0.0]*4), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
        mid= viz.AddCube(xe-((0.04*ey).tolist()+[0.0]*4), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
        mid= viz.AddCoord(xe, scale=[0.05,0.002], alpha=1.0, mid=mid)
      #Sentis M100 on Gripper:
      if xw[LEFT] is not None:
        lx_sensor_lg= t.GetAttr('wl_m100','lx')
        #x_sensor= t.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
        x_sensor= Transform(xw[LEFT],lx_sensor_lg)
        mid= viz.AddMarker(x_sensor, scale=[0.06,0.06,0.012], alpha=0.8, mid=mid)
        mid= viz.AddCoord(x_sensor, scale=[0.07,0.002], alpha=1.0, mid=mid)
      #Sleep until next visualization frame...
      time.sleep(100.0e-3)
      #End of: while Running...
  except Exception as e:
    PrintException(e, ' in viz')
  #tmpfp.write('########Finished: %r\n' % rospy.Time.now())
  #tmpfp.close()
def Run(t,*args):
  if len(args)>0:
    t.thread_manager.Add(name='viz', target=lambda th_info: VizLoop(th_info, t,args))
  else:
    t.thread_manager.Stop(name='viz')
