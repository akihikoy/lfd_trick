#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test pose estimation with ray tracing (2).
  Usage: test_poseest2 BOTTLE_ID
    BOTTLE_ID: label of bottle.
  '''

def LPoseRevisionCallback(msg,t):
  x_o= Transform(t.x_sensor,GPoseToX(msg.lpose.pose))
  print msg.lpose.label, x_o, msg.errors
  if msg.errors[0]<0.02 and msg.errors[1]<0.1:
    t.SetAttr(msg.lpose.label,'x', x_o)
  else:
    CPrint(4,'Revision is ignored because of the large errors')

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

def Run(t,*args):

  bottle1= args[0] if len(args)>0 else 'b53'

  #Update attribute information
  t.ExecuteMotion('objs2')

  #bottle1= 'b54'  #Coke can test
  #bottle1= 'b53'

  def EstPoseFromMarker():
    t.SetAttr(bottle1,'base_marker_id', 1)
    t.ExecuteMotion('infer2', {},[bottle1],'x',[],True)
    t.DelAttr(bottle1,'base_marker_id')

  EstPoseFromMarker()

  t.srvp.remove_scene= rospy.ServiceProxy('/rt_pose_estimator/remove_scene', lfd_vision.srv.RemoveScene, persistent=False)
  t.srvp.create_scene= rospy.ServiceProxy('/rt_pose_estimator/create_scene', lfd_vision.srv.CreateScene, persistent=False)
  t.pub.lpose= rospy.Publisher("/rt_pose_estimator/labeled_pose", lfd_vision.msg.LabeledPose)
  t.sub.lpose_rev= rospy.Subscriber("/rt_pose_estimator/labeled_pose_revision", lfd_vision.msg.LabeledPoseRevision, lambda msg:LPoseRevisionCallback(msg,t))

  #Remove existing scene:
  t.srvp.remove_scene(lfd_vision.srv.RemoveSceneRequest())

  #Create scene for bottle1
  create_scene_req= lfd_vision.srv.CreateSceneRequest()
  create_scene_req.name= 'scene1'
  create_scene_req.models.append(lfd_vision.msg.RayTraceModel())
  create_scene_req.models[-1].label= bottle1
  create_scene_req.models[-1].primitives= LoadShapePrimitives(t.GetAttr(bottle1))
  create_scene_req.models[-1].initial_pose= XToGPose(t.GetAttr(bottle1,'x'))
  t.srvp.create_scene(create_scene_req)

  t.kbhit.Activate()

  def Close():
    t.kbhit.Deactivate()
    t.sub.lpose_rev.unregister()
    del t.sub.lpose_rev
    t.pub.lpose.unregister()
    del t.pub.lpose
    del t.srvp.remove_scene
    del t.srvp.create_scene

  while True:
    if t.kbhit.IsActive():
      key= t.kbhit.KBHit()
      if key=='q':
        break;
      elif key==' ':
        EstPoseFromMarker()
    else:
      break

    t.ExecuteMotion('infer2', {},[bottle1],'x',[],False)
    x_b= t.GetAttr(bottle1,'x')
    lsensor_x_b= TransformLeftInv(t.x_sensor,x_b)  #Pose in sensor frame
    lpose= lfd_vision.msg.LabeledPose()
    lpose.label= bottle1
    lpose.pose= XToGPose(lsensor_x_b)
    t.pub.lpose.publish(lpose)


  Close()
