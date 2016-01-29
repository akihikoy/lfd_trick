#!/usr/bin/python
from core_tool import *
def Help():
  return '''Calibrate the RGB-D sensor pose using the pose estimator with ray tracing.
  We use an object whose pose is known in some way other than rtposeest.
  E.g. we can use an object grasped by robot, a gripper, etc.
  Usage: rtcalib_x REF_OBJ [, COUNT [, METHOD]]
    REF_OBJ: Object ID used as a reference, such as 'wrist_l', 'b53'.
    COUNT: How many times do we sample the object pose (default: 5).
    METHOD: Method of the ray tracing pose estimator (default: 'xyz').
  '''
def Run(t,*args):
  ref_obj= args[0]
  #ref_obj= 'wrist_l'
  #ref_obj= 'b53'
  rtpe_count= args[1] if len(args)>1 else 5
  rtpe_method= args[2] if len(args)>2 else 'xyz'

  #Since ref_obj may be already registered in the rtposeest scene,
  #we copy ref_obj as a new entry
  t.SetAttr(ref_obj+'_calib', copy.deepcopy(t.GetAttr(ref_obj)))
  ref_obj= ref_obj+'_calib'

  t.ExecuteMotion('infer2', {},[ref_obj],'x',[],True)
  x_w_1= t.GetAttr(ref_obj,'x')

  t.ExecuteMotion('rtposeest','create','ext','scene_calib_x',[ref_obj])

  CPrint(3,'Turning off x_sensor calibration with AR marker: t.ar_adjust_ratio=',t.ar_adjust_ratio,'--> 0.0')
  t.ar_adjust_ratio= 0.0

  #x_sensor_data= []
  #for k in range(rtpe_count):
    #x_w_2= t.ExecuteMotion('rtposeest','once','ext',ref_obj,1,rtpe_method,None)
    #x_sensor= TransformRightInv(x_w_1,TransformLeftInv(t.x_sensor,x_w_2))
    #x_sensor_data.append(x_sensor)
  #t.x_sensor[:3]= AverageXData(x_sensor_data)[:3]

  CPrint(3,'Running the ray tracing pose estimator...')
  x_w_2= t.ExecuteMotion('rtposeest','once','ext',ref_obj,rtpe_count,rtpe_method,None)
  if x_w_2!=None:
    x_sensor= TransformRightInv(x_w_1,TransformLeftInv(t.x_sensor,x_w_2))
    t.x_sensor[:3]= x_sensor[:3]
    CPrint(3,'Done calibration: t.x_sensor=',t.x_sensor)
  else:
    CPrint(4,'Error in rtcalib_x:',ref_obj)

  #t.ExecuteMotion('rtposeest','remove','ext','scene_calib_x')

  t.DelAttr(ref_obj)
