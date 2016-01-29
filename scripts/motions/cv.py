#!/usr/bin/python
from core_tool import *
def Help():
  return '''Computer vision based sensor tool (color_detector) manager.
  Usage:
    cv
    cv 'clear'
      Clear connection.
    cv 'setup' [, INDEX]
      Setup connection.
      INDEX: sensor index (0, 1, 2; 0: all; default: 1).
      Return: whether the setup is success (True or False).
    cv 'pause' [, INDEX]
      Pause the sensor node.
      INDEX: sensor index (0, 1, 2; 0: all; default: 1).
    cv 'resume' [, INDEX]
      Resume the sensor node.
      INDEX: sensor index (0, 1, 2; 0: all; default: 1).
  '''

##DEPRECATED: use ColDetObserver
##Observes amount of the material in the cup... just observing the color occupied ration
#def AmountObserver(t, msg):
  #t.material_amount_observed= True
  #t.material_amount= msg.data
  #if t.callback.amount_observer<>None:
    #t.callback.amount_observer(msg)
##DEPRECATED: use ColDetObserver
#def FlowSpeedObserver(t, msg):
  #if t.callback.flow_speed_observer<>None:
    #t.callback.flow_speed_observer(msg)

def ColDetObserver(t, msg):
  t.material_amount_observed= True  #WILL BE DEPRECATED
  t.material_amount= msg.col_filled_ratio[0]  #WILL BE DEPRECATED
  #For backward compatibility:
  if t.callback.amount_observer!=None:
    msg2= std_msgs.msg.Float64()
    msg2.data= msg.col_filled_ratio[0]
    t.callback.amount_observer(msg2)
  if t.callback.flow_speed_observer!=None:
    msg2= std_msgs.msg.Float64MultiArray()
    msg2.data= msg.flow_avr_spddir
    t.callback.flow_speed_observer(msg2)
  if t.callback.cv!=None:
    t.callback.cv(msg)

def ColDetObserver2(t, msg):
  if t.callback.cv2!=None:
    t.callback.cv2(msg)

def Run(t,*args):
  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'clear'
    args= []

  if command=='setup':
    index= args[0] if len(args)>0 else 1

    if 'cv' not in t.callback:
      t.material_amount= 0  #WILL BE DEPRECATED
      t.material_amount_observed= False  #WILL BE DEPRECATED
      t.callback.amount_observer= None  #Backward compatibility
      t.callback.flow_speed_observer= None  #Backward compatibility
      t.callback.cv= None
      t.callback.cv2= None

    #DEPRECATED; use /color_detector/sensor
    #if 'amount' not in t.sub:
      #t.sub.amount= rospy.Subscriber("/color_occupied_ratio", std_msgs.msg.Float64, lambda msg:AmountObserver(t,msg))
    #if 'flow_speed' not in t.sub:
      #t.sub.flow_speed= rospy.Subscriber("/flow_speed_angle", std_msgs.msg.Float64MultiArray, lambda msg:FlowSpeedObserver(t,msg))

    if index in (0,1):
      res= (t.AddPub('coldet_viz', '/color_detector/viz', lfd_vision.msg.ColDetViz))
      if not res:  return False
    if index in (0,2):
      res= (t.AddPub('coldet_viz2', '/color_detector2/viz', lfd_vision.msg.ColDetViz))
      if not res:  return False

    if index in (0,1):
      res= (t.AddSub('coldet_sensor', '/color_detector/sensor', lfd_vision.msg.ColDetSensor, lambda msg:ColDetObserver(t,msg)))
      if not res:  return False
    if index in (0,2):
      res= (t.AddSub('coldet_sensor2', '/color_detector2/sensor', lfd_vision.msg.ColDetSensor, lambda msg:ColDetObserver2(t,msg)))
      if not res:  return False

    if index in (0,1):
      res= (t.AddSrvP('cv_pause', '/color_detector/pause', std_srvs.srv.Empty, persistent=False, time_out=3.0))
      if not res:  return False
    if index in (0,2):
      res= (t.AddSrvP('cv_pause2', '/color_detector2/pause', std_srvs.srv.Empty, persistent=False, time_out=3.0))
      if not res:  return False

    if index in (0,1):
      res= (t.AddSrvP('cv_resume', '/color_detector/resume', std_srvs.srv.Empty, persistent=False, time_out=3.0))
      if not res:  return False
    if index in (0,2):
      res= (t.AddSrvP('cv_resume2', '/color_detector2/resume', std_srvs.srv.Empty, persistent=False, time_out=3.0))
      if not res:  return False

    return True

  elif command=='clear':
    t.callback.amount_observer= None  #Backward compatibility
    t.callback.flow_speed_observer= None  #Backward compatibility
    t.callback.cv= None
    t.callback.cv2= None
    raise Exception('Implement this. (delete subscriptions and proxies, etc.)')

  elif command=='pause':
    index= args[0] if len(args)>0 else 1
    if index in (0,1):  t.srvp.cv_pause()
    if index in (0,2):  t.srvp.cv_pause2()

  elif command=='resume':
    index= args[0] if len(args)>0 else 1
    if index in (0,1):  t.srvp.cv_resume()
    if index in (0,2):  t.srvp.cv_resume2()


