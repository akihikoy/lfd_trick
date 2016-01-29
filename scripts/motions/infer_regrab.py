#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer a good pose to put an object to re-grab for pouring.
  Usage: infer_regrab SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)
  '''

#Evaluate a putting pose.
def EvalXPut(x_put, parameters, situation, dsit, verbose=False):
  score= 1.0

  def IsValidXTrg(x_w_target, ign_objs):
    vg,res2,res3= dsit.t.ExecuteMotion('scene','isvalidx',dsit.arm,x_w_target,ign_objs,dsit.q_curr)
    if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
    if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
    if verbose and res3:
      #print 'res3:\n',res3
      VisualizeContacts(res3.contacts,dt=rospy.Duration(0.5))
    if not vg:  return False
    return True

  #TODO: fix the weights
  score-= abs(parameters[0])+abs(parameters[1])+1.0*abs(parameters[2])

  #Check the validity of x_put0 (above x_put)
  x_put0= Vec(copy.deepcopy(x_put))
  x_put0[2]+= 0.1  #10cm above x_put #FIXME: magic parameter
  x_w_trg= TransformRightInv(x_put0, dsit.lw_x_ext)  #Target of wrist x
  ignore= []  #Collisions of every objects are considered
  if not IsValidXTrg(x_w_trg, ignore):  return None

  #Check the validity of x_put
  x_w_trg= TransformRightInv(x_put, dsit.lw_x_ext)  #Target of wrist x
  ignore= [dsit.obj,'table']  #Ignoring collision with obj
  if not IsValidXTrg(x_w_trg, ignore):  return None

  #Check the validity of x_put1 (i.e. capability of withdrawing the hand)
  x_put1= Vec(copy.deepcopy(x_put))
  x_put1[:3]-= 2.5*dsit.g_width*XToPosRot(x_put)[1][:,0]  #Withdraw along ex of Rot of x_put #FIXME:the magic number
  x_w_trg= TransformRightInv(x_put1, dsit.lw_x_ext)  #Target of wrist x
  ignore= [dsit.obj,'table']  #Ignoring collision with obj
  if not IsValidXTrg(x_w_trg, ignore):  return None

  #Check the validity of re-grab pose which is roughly estimated
  x_g= copy.deepcopy(dsit.x_ext_curr)
  x_g[0]+= parameters[0]
  x_g[1]+= parameters[1]
  #Note: use the same oriantation
  x_w_trg= TransformRightInv(x_g, dsit.lw_x_ext)  #Target of wrist x
  ignore= [dsit.obj,'table']  #Ignoring collision with obj
  if not IsValidXTrg(x_w_trg, ignore):  return None

  return score

#Expand trajectory parameters.
#parameters[0,1]: displacement of x- and y-axis
#parameters[2]: rotation around z-axis, relative angle from dsit.theta_trg
#return: x_put
def ExpandXPutParameters(parameters, situation, dsit):
  x_put= copy.deepcopy(dsit.x_ext_curr)
  x_put[0]+= parameters[0]
  x_put[1]+= parameters[1]
  x_put[3:]= MultiplyQ(QFromAxisAngle([0.,0.,1.], dsit.theta_trg+parameters[2]), x_put[3:])
  return x_put

def DistOfXPutExs(parameters, situation, dsit, db_situation):
  if db_situation['infer_info']['type']!='ver1':  return None
  dists= []
  #FIXME: CONSIDER THIS AGAIN
  dists.append(10.0*Norm(Vec(parameters[0:2]) - Vec(db_situation['infer_info']['param'][0:2])))  #NOTE: 10.0 is for regularization
  dists.append(abs(parameters[2] - db_situation['infer_info']['param'][2]))
  d_x_o= DiffX(dsit.x_o, db_situation['obj_attr']['x'])
  dists.append(Norm(d_x_o[:3]))
  dists.append(0.2*Norm(d_x_o[3:]))
  dist= max(dists)
  return dist

def EvalXPutForFMin(parameters, situation, dsit, f_none=100):
  x_put= ExpandXPutParameters(parameters, situation, dsit)
  score= EvalXPut(x_put, parameters, situation, dsit)
  score= ModifyScoreFromExamples(
      score= score,
      bad_exs= dsit.t.GetAttrOr([], 'memory','regrab','bad'),
      good_exs= dsit.t.GetAttrOr([], 'memory','regrab','good'),
      dist_func= lambda db_situation, inferred_data, assessment: DistOfXPutExs(parameters, situation, dsit, db_situation),
      dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

  #print score, parameters
  if score<>None:  return -score
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
  lw_x_ext= situation['l_x_ext']  #Control point in wrist frame
  q_curr= GetOrSet(situation, 'q_curr', lambda:t.robot.Q(arm))

  t.ExecuteMotion('infer2', {},[obj],'x')
  x_o= a_get(obj,'x')

  InsertDict(situation, {'obj_attr':{'x':x_o}})
  inferred_keys.append([CURR,'regrab','x_put'])  #Pose of l_x_ext to put the obj


  #Container to store the variables derived from situation
  dsit= TContainer(debug=True)
  dsit.t= t
  dsit.obj= obj
  dsit.arm= arm
  dsit.lw_x_ext= lw_x_ext
  dsit.q_curr= q_curr
  dsit.x_o= x_o
  dsit.x_ext_curr= t.robot.FK(q=q_curr, x_ext=lw_x_ext, arm=arm)
  dsit.g_width= t.GetAttr(obj,'g_width')

  #Compute ideal theta for pouring...

  #xe_ey: e_y of Rot of dsit.x_ext_curr
  #  Note: -xe_ey is the best pouring direction
  xe_ey= XToPosRot(dsit.x_ext_curr)[1][:,1]

  #hc: container(obj)'s pouring point axis
  if a_has(obj,'l_p_pour_e_set'):
    edge_o_center= TParameterizedPolygon(a_get(obj,'l_p_pour_e_set')).Center
  elif a_has(obj,'l_x_pour_e'):
    edge_o_center= a_get(obj,'l_x_pour_e')
  p_ec= Transform(x_o,edge_o_center)
  e_z= XToPosRot(x_o)[1][:,2]  #e_z of Rot of x_o
  hc= (p_ec-x_o[:3]) - np.dot(p_ec-x_o[:3], e_z)*e_z

  #dsit.theta_trg: ideal theta (rotation around z-axis) for pouring
  axis,angle= GetAxisAngle([hc[0],hc[1],0.0], [(-xe_ey)[0],(-xe_ey)[1],0.0])
  dsit.theta_trg= angle if axis[2]>=0.0 else -angle
  CPrint(3,'###infer_regrab: theta_trg=',dsit.theta_trg)

  viz= TSimpleVisualizer(rospy.Duration(10.0), name_space='visualizer_regrab')
  viz.viz_frame= t.robot.BaseFrame

  #Visualize ideal putting location
  x_put= ExpandXPutParameters([0.0,0.0,0.0], situation, dsit)
  viz.AddCube(x_put, scale=[0.06,0.04,0.01], rgb=Vec(viz.ICol(4))*0.5, alpha=0.4)
  viz.AddArrow(x_put, scale=[0.06,0.003,0.003], rgb=Vec(viz.ICol(4))*0.5, alpha=0.4)


  bb_margin= 1.15  #TEST; bigger than margin for infer_traj
  t.ExecuteMotion('scene', 'make',[],bb_margin)


  fobj= lambda x: EvalXPutForFMin(x,situation,dsit,None)
  options= {'CMA_diagonal':1, 'verb_time':0}
  options['bounds']= [
      [-0.04,-0.04,-math.pi],
      [ 0.04, 0.04, math.pi]]
  options['tolfun']= 1.0e-4 # 1.0e-4
  options['maxfevals']= 500
  options['verb_log']= False
  options['scaling_of_variables']= np.array([0.005,0.005,1.0])
  parameters0= [0.0, 0.0, 0.0]
  scale0= 0.5*math.pi

  options['scale0_ratio1']= 0.005
  options['scale0_ratio2']= 0.2
  options['init_guess_num']= 5
  options['max_init_guess_num']= [100,4000]
  options['db_search_num']= 5
  options['max_db_search_count']= 200
  options['db_param_novelty']= 0.1

  parameters_res,score_res= ExecInference(fobj, parameters0, scale0, options,
                      t.database, db_search_key='regrab', infer_type='ver1')
  if score_res is None:
    t.ExecuteMotion('scene', 'clear')
    CPrint(4,'No solution found')
    return False
  InsertDict(situation, {'infer_info':{'type':'ver1', 'param':parameters_res, 'score':score_res}})

  x_put= ExpandXPutParameters(parameters_res, situation, dsit)
  a_set(CURR,'regrab','x_put',  x_put)

  t.ExecuteMotion('scene', 'clear')

  viz.AddCube(x_put, scale=[0.06,0.04,0.01], rgb=viz.ICol(4), alpha=0.7)
  viz.AddArrow(x_put, scale=[0.06,0.003,0.003], rgb=viz.ICol(4), alpha=0.7)

  #print '##DEBUG##',dsit.theta_trg
  #t.AskYesNo()


  return a_has(CURR,'regrab','x_put')


