#!/usr/bin/python
from core_tool import *
def Help():
  return '''Checking parameter consistency by visualizing.
  Usage: test_viz'''
def Run(t,*args):
  t.ExecuteMotion('objs2')

  t.viz.test= TSimpleVisualizer(name_space='visualizer_test')
  t.viz.test.viz_frame= t.robot.BaseFrame

  def Close(t):
    if 'test' in t.viz:
      del t.viz['test']

  for arm in (LEFT,RIGHT):
    xw= t.robot.FK(arm=arm)
    t.viz.test.AddCoord(xw, scale=[0.05,0.002], alpha=1.0)
    lw_xe= t.GetAttr('wrist_'+LRToStrs(arm),'lx')
    xe= Vec(Transform(xw,lw_xe))
    ex,ey,ez= RotToExyz(QToRot(xe[3:]))
    #t.viz.test.AddCube(xe, [0.1,0.06,0.04], rgb=t.viz.test.ICol(arm), alpha=0.5)
    t.viz.test.AddCube(xe-((0.05*ex).tolist()+[0.0]*4), [0.1,0.06,0.02], rgb=t.viz.test.ICol(3), alpha=0.5)
    t.viz.test.AddCube(xe+((0.03*ey).tolist()+[0.0]*4), [0.02,0.005,0.04], rgb=t.viz.test.ICol(2), alpha=0.5)
    t.viz.test.AddCube(xe-((0.03*ey).tolist()+[0.0]*4), [0.02,0.005,0.04], rgb=t.viz.test.ICol(2), alpha=0.5)
    t.viz.test.AddCoord(xe, scale=[0.05,0.002], alpha=1.0)

    bound_box= t.GetAttr('wrist_'+LRToStrs(arm), 'bound_box')
    lx_bb= bound_box['center']
    dim_bb= Vec(bound_box['dim'])*1.1
    x_bb= Transform(xe, lx_bb)
    t.viz.test.AddCube(x_bb, dim_bb, rgb=t.viz.test.ICol(1), alpha=0.5)

  #Marker on the wrist
  x_m_wrist= t.robot.FK(x_ext=t.l_x_m_wrist, arm=t.ar_adjust_arm)
  #Visualize the marker on the wrist
  t.viz.test.AddMarker(x_m_wrist, scale=[0.04,0.04,0.012], mid=9999, rgb=t.viz.test.ICol(2), alpha=0.5)
  t.viz.test.AddMarker(x_m_wrist, scale=[0.05,0.05,0.02], mid=9999, rgb=t.viz.test.ICol(4), alpha=0.5)
  t.viz.test.AddArrow(x_m_wrist, scale=[0.05,0.004,0.004], mid=10000+9999, rgb=t.viz.test.ICol(2), alpha=0.5)

  #Sentis M100 on Gripper
  #lx1= [0.082, 0.00, 0.08315]+MultiplyQ(QFromAxisAngle([0,1,0],DegToRad(88.)),QFromAxisAngle([0,0,1],DegToRad(90.))).tolist()
  #lx1= [-0.08315, 0.00, 0.0]+MultiplyQ(QFromAxisAngle([0.,1.,0.],-math.pi/2.0),MultiplyQ(QFromAxisAngle([0,1,0],DegToRad(88.)),QFromAxisAngle([0,0,1],DegToRad(90.)))).tolist()
  #lx_sensor_lg= lx1
  lx_sensor_lg= t.GetAttr('wl_m100','lx')
  x_sensor= t.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
  print 'wl_m100 x_sensor=',x_sensor
  t.viz.test.AddMarker(x_sensor, scale=[0.06,0.06,0.012], alpha=0.8)
  t.viz.test.AddCoord(x_sensor, scale=[0.07,0.002], alpha=1.0)

  bound_box= t.GetAttr('wl_m100', 'bound_box')
  lx_bb= bound_box['center']
  dim_bb= Vec(bound_box['dim'])*1.1
  x_bb= Transform(x_sensor, lx_bb)
  t.viz.test.AddCube(x_bb, dim_bb, rgb=t.viz.test.ICol(1), alpha=0.5)

  ##Local observation pose of Sentis M100
  #lx_obs= copy.deepcopy(t.GetAttr('wl_m100','lx'))
  #ex,ey,ez= RotToExyz(XToPosRot(lx_obs)[1])
  #invq= MultiplyQ(QFromAxisAngle(ez,DegToRad(-90.)),QFromAxisAngle(ey,DegToRad(-90.)))
  #lx_obs[3:]= Transform(invq, lx_obs[3:])
  #x_obs= t.robot.FK(x_ext=lx_obs, arm=LEFT)
  #t.viz.test.AddMarker(x_obs, scale=[0.06,0.06,0.012], rgb=t.viz.test.ICol(4), alpha=0.8)
  #t.viz.test.AddCoord(x_obs, scale=[0.10,0.004], alpha=0.5)

  raw_input('done > ')
  Close(t)
