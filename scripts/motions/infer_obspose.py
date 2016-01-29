#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer observation pose l_x_obs.
  Usage: infer_obspose SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''

#Evaluate an observation pose.
#x_obs_trg: observation pose
#length: length of observation pose from a target object
def EvalObsPose(dsit, x_obs_trg, length, verbose=False):
  if length<0.25 or length>0.60:  return None
  #Check the validity of grab pose
  x_w_trg= TransformRightInv(x_obs_trg,dsit.l_x_obs)  #Target of wrist x
  ignore= []
  vg,res2,res3= dsit.t.ExecuteMotion('scene','isvalidx',dsit.arm,x_w_trg,ignore,dsit.q_curr)
  #if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
  #if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
  if verbose:  print 'res2=',res2
  if verbose:  print 'res3=',res3
  if verbose and res3:
    #print 'res3:\n',res3
    VisualizeContacts(res3.contacts,dt=rospy.Duration(0.5))
  if not vg:  return None

  #Check the occlusion by other objects
  for p_occl in dsit.set_p_occl:
    v1= Vec(x_obs_trg[:3])-dsit.p_o
    v2= p_occl-dsit.p_o
    if abs(GetAngle([v1[0],v1[1],0.0], [v2[0],v2[1],0.0])) < dsit.angle_occl:  #Consider only xy angle
      return None

  score= 1.0

  #Score from current control pose and observation pose:
  d_x_obs= DiffX(dsit.x_obs, x_obs_trg)  #[dx,dy,dz, dwx,dwy,dwz]
  score-= 0.3*Norm(d_x_obs[:3]) + 0.1*Norm(d_x_obs[3:])

  #Score from observation length:
  score-= 10.0*(length-0.25)**2

  return score

#Expand observation-pose parameters.
#parameters[0]= length from a target object [0.25,0.60]
#parameters[1]= yaw angle of observation pose [-pi,pi]
def ExpandObsPoseParameters(parameters, dsit):
  length= parameters[0]
  phi_yaw= parameters[1]
  phi_pitch= dsit.phi_pitch
  d= Transform(QFromAxisAngle([0.0,1.0,0.0],phi_pitch), [-length,0.0,0.0, 0.0,0.0,0.0,1.0])
  d= Transform(QFromAxisAngle([0.0,0.0,1.0],phi_yaw), d)
  x_obs_trg= [0.0]*7
  x_obs_trg[:3]= dsit.p_o+d[:3]
  x_obs_trg[3:]= d[3:]
  return x_obs_trg, length

#Distance of two parameters
def DistOfObsPoseParams(p1,p2):
  return Dist(p1,p2)

#Distance to modify the score function from examples
def DistOfObsPoseExs(parameters, dsit, db_situation):
  if db_situation['infer_info']['type']!='ver1':  return None
  dists= []
  dists.append(DistOfObsPoseParams(parameters, db_situation['infer_info']['param']))
  dists.append(Dist(dsit.p_o, db_situation['obj_attr']['p']))
  dists.append(0.5*MaxNorm(Vec(dsit.q_curr)-Vec(db_situation['q_curr'])))
  dists.append(0.0 if dsit.obj==db_situation['obj'] else 0.5)
  #print dists
  dist= max(dists)
  return dist

def EvalObsPoseForFMax(parameters, dsit, f_none=100):
  x_obs_trg, length= ExpandObsPoseParameters(parameters, dsit)

  score= EvalObsPose(dsit, x_obs_trg, length)
  score= ModifyScoreFromExamples(
      score= score,
      bad_exs= dsit.t.GetAttrOr([], 'memory','obspose','bad'),
      good_exs= dsit.t.GetAttrOr([], 'memory','obspose','good'),
      dist_func= lambda db_situation, inferred_data, assessment: DistOfObsPoseExs(parameters, dsit, db_situation),
      dist_threshold= 0.2)  #FIXME: dist_threshold is too big?
  #print score, parameters
  if score!=None:  return score
  else:            return f_none


def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  inferred_keys= args[2] if len(args)>2 else []
  verbose= args[3] if len(args)>3 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))

  obj= situation['obj']
  arm= situation['handid']
  q_curr= GetOrSet(situation, 'q_curr', lambda:t.robot.Q(arm))
  l_x_obs= situation['l_x_obs']
  objs_occl= GetOrSet(situation, 'objs_occl', lambda:[])  #Objects possibly causing occlusion
  angle_occl= GetOrSet(situation, 'angle_occl', lambda:DegToRad(45.0))  #We consider occlusion happens if the angle is inside this value

  x_obs= t.robot.FK(q=q_curr, x_ext=l_x_obs, arm=arm)

  #Current object pose, bounding box, and the center
  t.ExecuteMotion('infer2', {},[obj],'x')
  x_o= t.GetAttr(obj,'x')
  t.ExecuteMotion('infer2', {},[obj],'bound_box')
  p_o= Transform(x_o, t.GetAttr(obj,'bound_box','center')[:3])

  InsertDict(situation, {'obj_attr':{'p':p_o}})

  set_p_occl= []
  dict_p_occl={}
  for obj_occl in objs_occl:
    t.ExecuteMotion('infer2', {},[obj_occl],'x')
    p_occl= t.GetAttr(obj_occl,'x')[:3]
    set_p_occl.append(Vec(p_occl))
    dict_p_occl[obj_occl]= p_occl
  InsertDict(situation, {'set_p_occl':dict_p_occl})

  inferred_keys.append([obj,'x_obs_trg'])

  t.ExecuteMotion('scene','make')

  #Container to store the variables derived from situation
  dsit= TContainer(debug=True)
  dsit.t= t
  dsit.arm= arm
  dsit.obj= obj
  dsit.p_o    = Vec(p_o)  #center of target object
  dsit.q_curr = q_curr
  dsit.l_x_obs= l_x_obs  #control pose in the wrist frame
  dsit.x_obs= x_obs  #current control pose
  dsit.phi_pitch= DegToRad(30.0)  #pitch angle which is not optimized
  dsit.objs_occl= objs_occl
  dsit.set_p_occl= set_p_occl
  dsit.angle_occl= angle_occl

  options= {}
  options['param_struct']= ['c0','c']
  options['c0']= {}
  options['c0']['bounds']= [[0.25,-math.pi],[0.60,math.pi]]
  options['c0']['tolfun']= 1.0e-2 # 1.0e-4
  options['c0']['scaling_of_variables']= np.array([1.0,0.1])
  options['c0']['parameters0']= [0.25,0.0]
  options['c0']['scale0']= 0.2

  options['maxfevals']= 200
  options['scale0_ratio1']= 0.005
  options['scale0_ratio2']= 0.2
  options['init_guess_num']= 3
  options['max_init_guess_num']= [200,4000]
  options['db_search_num']= 3
  options['max_db_search_count']= 1000
  options['db_param_novelty']= 0.1
  fobj= lambda x: EvalObsPoseForFMax(x,dsit,None)

  parameters_res, score_res= CompOptInference(fobj, options,
                      t.database, db_search_key='x_obs_trg', infer_type='ver1',
                      param_dist=DistOfObsPoseParams)
  if score_res is None:
    t.ExecuteMotion('scene', 'clear')
    CPrint(4,'No solution found')
    return False
  InsertDict(situation, {'infer_info':{'type':'ver1', 'param':parameters_res, 'score':score_res}})

  x_obs_trg, length= ExpandObsPoseParameters(parameters_res, dsit)
  a_set(obj,'x_obs_trg',  x_obs_trg)

  viz= TSimpleVisualizer(rospy.Duration(10.0), name_space='visualizer_obspose')
  viz.viz_frame= t.robot.BaseFrame
  viz.AddMarker(x_obs_trg, scale=[0.012,0.04,0.04], rgb=[0.5,0.5,0.5], alpha=0.7)
  viz.AddCoord(x_obs_trg, scale=[0.05,0.002], alpha=0.7)

  t.ExecuteMotion('scene', 'clear')

  return a_has(obj,'x_obs_trg')

