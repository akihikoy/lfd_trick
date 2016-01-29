#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer pouring edge point l_x_pour_e on the source frame for spreading, with considering the source's initial pouring angle pour_start_angle.
  Usage: infer_poursp SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''
def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  inferred_keys= args[2] if len(args)>2 else []
  verbose= args[3] if len(args)>3 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))


  source= situation['source']
  receiver= situation['receiver']
  arm= situation['handid']
  lb_x_grab= GetOrSet(situation, 'lb_x_grab', lambda:t.GetAttr(source,'l_x_grab'))

  t.ExecuteMotion('infer2', {},[source],'x')
  x_b= a_get(source,'x')
  t.ExecuteMotion('infer2', {},[receiver],'x')
  x_c= a_get(receiver,'x')

  spread_start= TransformLeftInv(x_c, situation['spread_start'])

  InsertDict(situation, {'source_attr':{'x':x_b}, 'receiver_attr':{'x':x_c}})
  inferred_keys.append([source,'l_x_pour_e'])
  inferred_keys.append([receiver,'l_x_pour_l'])
  inferred_keys.append([receiver,'l_x_pour_l0'])



  '''Estimating pouring edge point l_x_pour_e,
      and pouring location l_x_pour_l on the receiver frame,
      with considering initial pouring angle pour_start_angle'''

  #Evaluate a pouring poses.
  #lb_x_pe: pouring edge point on the source container frame
  #lc_x_pl: pouring location on the receiving container frame
  #x_b: source container pose
  #x_c: receiving container pose
  def EvalPourSpPoses(lb_x_pe, lc_x_pl, lc_x_pl0, x_b, x_c, arm, q_curr, x_w, lb_x_grab, verbose=False):
    score= 1.0

    x_pe= Transform(x_b, lb_x_pe)
    x_pl= Transform(x_c, lc_x_pl)
    x_pl0= Transform(x_c, lc_x_pl0)
    x_grab= Transform(x_b, lb_x_grab)

    #Requirement on angle between gripper and pouring axis:
    p_pe, R_pe= XToPosRot(x_pe)
    e_x_pe= R_pe[:,0]
    p_g, R_g= XToPosRot(x_grab)
    e_x_g= R_g[:,0]
    a2= abs(GetAngle(e_x_pe, e_x_g))  #[0,pi]
    if verbose:  print 'a2=',RadToDeg(a2)
    if not IsIn(a2,DegToRad([0,90])):
      return None
    score-= 1.0*a2

    #Check the validity of pouring location-0 when moving the pouring edge point
    lw_x_pe= TransformLeftInv(x_w, x_pe)  #Pouring edge point in wrist frame
    x_w_trg= TransformRightInv(x_pl0,lw_x_pe)  #Target of wrist x
    vg,res2,res3= t.ExecuteMotion('scene','isvalidx',arm,x_w_trg,[],q_curr)
    if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
    if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
    if not vg:  return None

    return score

  #Expand pouring parameters.
  #parameters[0]= angle of source container's edge
  def ExpandPourSpParameters(parameters, edge_b, spread_start, pour_start_angle):
    lb_x_pe= [0]*7
    p1= edge_b.AngleToPoint(parameters[0])
    if p1 is None:  return None,None,None
    lb_x_pe[0:3]= list(p1)
    R= Eye()
    #e_z:
    R[:,2]= edge_b.Axes[:,2] if edge_b.Axes[2,2]>0 else -edge_b.Axes[:,2]
    #e_y:
    R[:,1]= GetOrthogonalAxisOf(R[:,2], preferable=edge_b.Center-Vec(lb_x_pe[0:3]))
    #e_x:
    R[:,0]= np.cross(R[:,1], R[:,2])
    lb_x_pe[3:]= RotToQ(R)

    lc_x_pl= [0]*7
    lc_x_pl[0:3]= list(spread_start)
    #FIXME: a magic parameter; 7cm above of the point
    lc_x_pl[2]+= 0.07
    #lc_x_pl[3:]= lb_x_pe[3:]
    lc_x_pl[3:]= [0.0,0.0,0.0,1.0]  #FIXME: a magic parameter

    #Rotate lc_x_pl with pour_start_angle
    q_pour_start= QFromAxisAngle(QToRot(lc_x_pl[3:])[:,0], pour_start_angle)
    lc_x_pl[3:]= MultiplyQ(q_pour_start, lc_x_pl[3:])

    lc_x_pl0= copy.deepcopy(lc_x_pl)
    #13cm above; FIXME: magic parameter
    lc_x_pl0[:3]= Vec(lc_x_pl[:3]) + 0.13*Vec([0.,0.,1.])
    return  lb_x_pe, lc_x_pl, lc_x_pl0

  def DistOfPourSpExs(parameters, x_b, x_c, arm, q_curr, x_w, edge_b, spread_start, pour_start_angle,  db_situation):
    if db_situation['infer_info']['type']!='ver_sp1':  return None
    dists= []
    #FIXME: CONSIDER THE CONTAINER TYPES
    dists.append(Norm(Vec(parameters) - Vec(db_situation['infer_info']['param'])))
    d_x_b= DiffX(x_b, db_situation['source_attr']['x'])
    dists.append(Norm(d_x_b[:3]))
    dists.append(0.2*Norm(d_x_b[3:]))
    d_spread_start= Norm(Vec(spread_start), Vec(db_situation['spread_start']))
    dists.append(d_spread_start)
    #print dists
    dist= max(dists)
    return dist

  def EvalPourSpPosesForFMin(parameters, x_b, x_c, arm, q_curr, x_w, lb_x_grab, edge_b, spread_start, pour_start_angle, f_none=100):
    lb_x_pe, lc_x_pl, lc_x_pl0= ExpandPourSpParameters(parameters, edge_b, spread_start, pour_start_angle)
    if None in (lb_x_pe, lc_x_pl, lc_x_pl0):  return f_none
    score= EvalPourSpPoses(lb_x_pe, lc_x_pl, lc_x_pl0, x_b, x_c, arm, q_curr, x_w, lb_x_grab)
    score= ModifyScoreFromExamples(
        score= score,
        bad_exs= t.GetAttrOr([], 'memory','poursp','bad'),
        good_exs= t.GetAttrOr([], 'memory','poursp','good'),
        dist_func= lambda db_situation, inferred_data, assessment: DistOfPourSpExs(parameters, x_b, x_c, arm, q_curr, x_w, edge_b, spread_start, pour_start_angle,  db_situation),
        dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

    #print score, parameters
    if score!=None:  return -score
    else:            return f_none

  if t.HasAttr(source,'l_p_pour_e_set') and t.HasAttr(source,'pour_start_angle'):
    bb_margin= 1.15  #TEST; bigger than margin for infer_traj
    t.ExecuteMotion('scene', 'make',[],bb_margin)

    t.ExecuteMotion('infer', source,'x')
    x_b= np.array(t.GetAttr(source,'x'))
    t.ExecuteMotion('infer', receiver,'x')
    x_c= np.array(t.GetAttr(receiver,'x'))
    q_curr= t.robot.Q(arm)
    x_w= t.robot.FK(q=q_curr,arm=arm)
    edge_b= TParameterizedPolygon(t.GetAttr(source,'l_p_pour_e_set'), center_modifier=lambda c:[0.0,0.0,c[2]])
    pour_start_angle= t.GetAttr(source,'pour_start_angle')

    fobj= lambda x: EvalPourSpPosesForFMin(x,x_b,x_c,arm,q_curr,x_w,lb_x_grab,edge_b,spread_start,pour_start_angle,None)
    options= {'CMA_diagonal':1, 'verb_time':0}
    #options['bounds']= [[-math.pi,-math.pi,0.5],[math.pi,math.pi,0.7]]
    options['bounds']= [[edge_b.Bounds[0]],[edge_b.Bounds[1]]]
    options['tolfun']= 1.0e-2 # 1.0e-4
    options['maxfevals']= 1000
    options['verb_log']= False
    options['scaling_of_variables']= np.array([(edge_b.Bounds[1]-edge_b.Bounds[0])/(2.0*math.pi)])
    parameters0= [0.5*(edge_b.Bounds[0]+edge_b.Bounds[1])]
    scale0= 0.5*math.pi

    options['scale0_ratio1']= 0.005
    options['scale0_ratio2']= 0.2
    options['init_guess_num']= 5
    options['max_init_guess_num']= [200,1000]
    options['db_search_num']= 5
    options['max_db_search_count']= 1000
    options['db_param_novelty']= 0.1

    parameters_res,score_res= ExecInference(fobj, parameters0, scale0, options,
                        t.database, db_search_key='l_x_pour_e', infer_type='ver_sp1')
    if score_res is None:
      t.ExecuteMotion('scene', 'clear')
      CPrint(4,'No solution found')
      return False
    InsertDict(situation, {'infer_info':{'type':'ver_sp1', 'param':parameters_res, 'score':score_res}})

    lb_x_pe, lc_x_pl, lc_x_pl0= ExpandPourSpParameters(parameters_res, edge_b, spread_start, pour_start_angle)

    t.SetAttr(source,'l_x_pour_e',  lb_x_pe)
    t.SetAttr(receiver,'l_x_pour_l',  lc_x_pl)
    t.SetAttr(receiver,'l_x_pour_l0',  lc_x_pl0)


  t.ExecuteMotion('scene', 'clear')

  return t.HasAttr(source,'l_x_pour_e') and t.HasAttr(receiver,'l_x_pour_l')

