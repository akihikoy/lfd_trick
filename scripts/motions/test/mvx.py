#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test move_to_x, a collision free reaching motion generator.
  Usage: test_mvx'''
def Run(t,*args):
  conservative= False
  arm= LEFT
  lw_xe= [0.0,0.0,0.0,  0.0, -0.70710678, 0.0, 0.70710678]
  with_wall= False
  with_obj= True

  q0= [-1.110782068313732, -0.8585888942618372, -0.024897070310493596, 1.8091441982766785, -0.41351721873277825, -0.9970549994780731, 0.2511101055899809]
  t.robot.MoveToQ(q0,dt=2.0, arm=arm,blocking=True)
  xe_0= t.robot.FK(x_ext=lw_xe, arm=arm)
  xe_trg= copy.deepcopy(xe_0)
  #xe_trg[:2]= [0.73, 0.52]
  xe_trg[:2]= [0.75, 0.12]
  print ToList(xe_0), ToList(xe_trg)

  #Setup the collision check scene
  scene_objs= []
  if with_wall:
    wdim= [0.15,0.15,2.0]
    wall_attr={
        #'x': [0.8,0.25,0.0] if not with_obj else [0.85,0.25,0.0],
        'x': [0.85,0.25,0.0],
        #'x': [0.85,0.0,0.0],
        'bound_box': {'dim':wdim,'center':[0.0,0.0,0.0, 0.0,0.0,0.0,1.0]},
        'shape_primitives': [
          {
            'kind': 'rtpkCuboid',
            'param': [l/2.0 for l in wdim],  #[half_len_x,half_len_y,half_len_z]
            'pose': [0.0,0.0,0.0, 0.0,0.0,0.0,1.0],
            },
          ],
      }
    t.SetAttr('wall',wall_attr)
    scene_objs+= ['wall']

  if with_obj:
    t.ExecuteMotion('objs2')
    t.SetAttr('b53','l_x_grab', [0.0,0.0,0.08, 0,0,0,1])
    t.SetAttr('b53','g_width', 0.05)
    scene_objs+= ['b53']
    t.ExecuteMotion('grab', 'b53', LRToStrs(arm))

  t.SetAttr(CURR,'scene', scene_objs)
  t.ExecuteMotion('viz', *scene_objs)

  #Check the initial condition:
  t.ExecuteMotion('scene', 'make')
  isvalid, res= t.ExecuteMotion('scene','isvalidq',arm,[],t.robot.Q(arm),True)
  time.sleep(1.0)
  print 'scene validity: ',isvalid
  t.ExecuteMotion('scene', 'clear')
  if not isvalid:
    print 'Initial state is invalid.'
    print 'Colliding objects:',GetCollidingObjects(res.contacts)
    return False

  print t.ExecuteMotion('move_to_x', xe_trg, 3.0, lw_xe, arm, {}, conservative)
  print t.ExecuteMotion('move_to_x', xe_0, 3.0, lw_xe, arm, {}, conservative)

  if with_obj:
    t.ExecuteMotion('release', 'b53')
