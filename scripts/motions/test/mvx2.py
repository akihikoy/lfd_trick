#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test move_to_x with a setup of previous failure (now fixed).
  Usage: mvx2'''
def Run(t,*args):
  conservative= True

  t.ExecuteMotion('objs2')

  #Setup of the failure case
  q_curr= [-1.0734030550964355, -0.2239619477539064, 0.26231071442871096, 1.5654273922485353, -0.6047719249328614, -1.695535931152344, 0.6642136803955079]
  l_x_ext= [0.1220011136234492, 1.9071793840377826e-17, 0.11498665099872901, 0.49950990312532106, -0.5043495896854451, -0.49956456101266927, 0.50048971807830656]
  objs_attr= {
    'table': {'x': [0.95, 0.0, -0.17393322833847408, 0.0, 0.0, 00, 1.0]},
    'table0': {'x': [0.80951779228395992, 0.00036268322148388102, -0.1733322833847408, 0.0, 0.0, 0.0, 1.0]},
    'b56': {'x': [0.91882024332689316, 0.4210116993892362, 0.040557153806231275, 0.29379794437407319, 0.23237888597166714, 0.50051320252724452, 0.78049302068119586],
      'grabbed': {'grabber_handid': 1, 'grabber_wrist':'wrist_l'},
      'l_x_grab': [-0.0001831709998704294, -0.098999830547253, 0.12200111437264902, 3.864896278952064e-05, -3.8720537844668266e-05, 0.706452328502289, 0.7077606265967835],
      },
    'b65': {'x': [0.93558438005740463, 0.26446420886548533, -0.17385039842827682, -.1684043449710081e-19, -8.6736173798840326e-19, 0.0093418947148062575, 0.9999536354949879]}
    }
  x_target= [0.88005797271632624, -0.042687073282167746, -0.16052950981145744, -0.00022601564861422777, -0.003255077722311605, -0.65310945210576898, 0.75725649165667364]
  arm= LEFT

  t.ExecuteMotion('setup_pour_scene', 'b56','b65')

  for key,value in objs_attr.iteritems():
    t.AddDictAttr(key,value)
  scene_objs= objs_attr.keys()
  t.SetAttr(CURR,'scene', scene_objs)
  t.ExecuteMotion('viz', *scene_objs)

  t.robot.MoveToQ(q_curr,dt=3.0, arm=arm,blocking=True)
  CPrint(1,'Start move_to_x.  Ready?')
  if not t.AskYesNo():  return

  print t.ExecuteMotion('move_to_x', x_target, 4.0, l_x_ext, arm, {}, conservative)

