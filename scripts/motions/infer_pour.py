#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer pouring edge point l_x_pour_e on the source frame and pouring location l_x_pour_l on the receiver frame, with considering the source's initial pouring angle pour_start_angle.
  Usage: infer_pour SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''

'''Estimating pouring edge point l_x_pour_e,
    and pouring location l_x_pour_l on the cup frame,
    with considering initial pouring angle pour_start_angle
    (ver.2)'''

#Evaluate a pouring poses.
#lb_x_pe: pouring edge point on the source container frame
#lc_x_pl: pouring location on the receiving container frame
#x_b: source container pose
#x_c: receiving container pose
def EvalPourPoses(dsit, lb_x_pe, lc_x_pl, lc_x_pl0, verbose=False):
  score= 1.0

  x_pe= Transform(dsit.x_b, lb_x_pe)
  x_pl= Transform(dsit.x_c, lc_x_pl)
  x_pl0= Transform(dsit.x_c, lc_x_pl0)

  #TEST
  ##Score from moving the source container:
  #d1= la.norm(Vec(x_pe[0:3])-Vec(x_pl[0:3]))
  #score-= d1
  #if verbose:  print 'd1=',d1

  #TEST
  ##Score from rotating the source container (considering only ex):
  #d2= 1.0-np.dot(QToRot(x_pe[3:])[:,0],QToRot(x_pl[3:])[:,0])
  #score-= d2
  #if verbose:  print 'd2=',d2

  #Requirement on pouring axis from left gripper's pouring movement:
  p_pl, R_pl= XToPosRot(x_pl)
  e_x_pl= R_pl[:,0]
  #a1= math.atan2(abs(e_x_pl[1]), abs(e_x_pl[0]))
  a1= abs(GetAngle(e_x_pl, [1.0,0.0,0.0]))  #[0,pi]
  if verbose:  print 'a1=',RadToDeg(a1)
  if not IsIn(a1,DegToRad([0,70])):  #WARNING:CRITICAL_CHANGE
    return None
  score-= 0.5*a1

  #Requirement on angle between gripper and pouring axis:
  p_pe, R_pe= XToPosRot(x_pe)
  e_x_pe= R_pe[:,0]
  p_g, R_g= XToPosRot(dsit.x_grab)
  e_x_g= R_g[:,0]
  a2= abs(GetAngle(e_x_pe, e_x_g))  #[0,pi]
  if verbose:  print 'a2=',RadToDeg(a2)
  if not IsIn(a2,DegToRad([0,45])):  #WARNING:CRITICAL_CHANGE
    return None
  score-= 1.0*a2

  #Check the validity of pouring location-0 when moving the pouring edge point
  lw_x_pe= TransformLeftInv(dsit.x_w, x_pe)  #Pouring edge point in wrist frame
  x_w_trg= TransformRightInv(x_pl0,lw_x_pe)  #Target of wrist x
  vg,res2,res3= dsit.t.ExecuteMotion('scene','isvalidx',dsit.arm,x_w_trg,[],dsit.q_curr)
  #if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
  #if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
  if verbose:  print 'res2=',res2
  if verbose:  print 'res3=',res3
  if not vg:  return None

  return score

#Expand pouring parameters.
#parameters[0]= angle of source container's edge
#parameters[1]= angle of receiving container's edge
#parameters[2]= ratio of receiver's edge and its center
def ExpandPourParameters(parameters, dsit):
  lb_x_pe= [0]*7
  p1= dsit.edge_b.AngleToPoint(parameters[0])
  if p1 is None:  return None,None,None
  lb_x_pe[0:3]= list(p1)
  R= Eye()
  #e_z:
  R[:,2]= dsit.edge_b.Axes[:,2] if dsit.edge_b.Axes[2,2]>0 else -dsit.edge_b.Axes[:,2]
  #e_y:
  R[:,1]= GetOrthogonalAxisOf(R[:,2], preferable=dsit.edge_b.Center-Vec(lb_x_pe[0:3]))
  #e_x:
  R[:,0]= np.cross(R[:,1], R[:,2])
  lb_x_pe[3:]= RotToQ(R)

  lc_x_pl= [0]*7
  ratio= parameters[2] if dsit.x_pl_center_ratio==None else dsit.x_pl_center_ratio
  edge_c_p= dsit.edge_c.AngleToPoint(parameters[1])
  if edge_c_p is None:  return None,None,None
  lc_x_pl[0:3]= list(ratio*edge_c_p + (1.0-ratio)*dsit.edge_c.Center)
  #lc_x_pl[0:3]= list(dsit.edge_c.AngleToPoint(parameters[1]))
  #FIXME: a magic parameter; 3cm above of the point
  lc_x_pl[2]+= dsit.x_pl_displacement_z  #0.03
  R= Eye()
  #e_z:
  R[:,2]= dsit.edge_c.Axes[:,2] if dsit.edge_c.Axes[2,2]>0 else -dsit.edge_c.Axes[:,2]
  #e_y:
  R[:,1]= GetOrthogonalAxisOf(R[:,2], preferable=edge_c_p-dsit.edge_c.Center)
  #e_x:
  R[:,0]= np.cross(R[:,1], R[:,2])
  lc_x_pl[3:]= RotToQ(R)

  #Rotate lc_x_pl with pour_start_angle
  q_pour_start= QFromAxisAngle(R[:,0], dsit.pour_start_angle)
  lc_x_pl[3:]= MultiplyQ(q_pour_start, lc_x_pl[3:])

  lc_x_pl0= copy.deepcopy(lc_x_pl)
  #lc_x_pl0[:3]= Vec(lc_x_pl[:3]) + 0.12*Normalize(Vec(lc_x_pl[:3])-dsit.edge_c.Center)
  #12cm above; FIXME: magic parameter
  lc_x_pl0[:3]= Vec(lc_x_pl[:3]) + 0.12*Vec([0.,0.,1.])
  return  lb_x_pe, lc_x_pl, lc_x_pl0

def DistOfPourExs(parameters, dsit,  situation):
  if situation['infer_info']['type']!=dsit.infer_type:  return None
  dists= []
  #FIXME: CONSIDER THE CONTAINER TYPES
  dists.append(Norm(Vec(parameters) - Vec(situation['infer_info']['param'])))
  d_x_b= DiffX(dsit.x_b, situation['source_attr']['x'])
  dists.append(Norm(d_x_b[:3]))
  dists.append(0.2*Norm(d_x_b[3:]))
  d_x_c= DiffX(dsit.x_c, situation['receiver_attr']['x'])
  dists.append(Norm(d_x_c[:3]))
  dists.append(0.2*Norm(d_x_c[3:]))
  #print dists
  dist= max(dists)
  return dist

def EvalPourPosesForFMin(parameters, dsit, f_none=100):
  lb_x_pe, lc_x_pl, lc_x_pl0= ExpandPourParameters(parameters, dsit)
  if None in (lb_x_pe, lc_x_pl, lc_x_pl0):  return f_none
  score= EvalPourPoses(dsit, lb_x_pe, lc_x_pl, lc_x_pl0)
  score= ModifyScoreFromExamples(
      score= score,
      bad_exs= dsit.t.GetAttrOr([], 'memory','pour','bad'),
      good_exs= dsit.t.GetAttrOr([], 'memory','pour','good'),
      dist_func= lambda situation, inferred_data, assessment: DistOfPourExs(parameters, dsit,  situation),
      dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

  #print score, parameters
  if score!=None:  return -score
  else:            return f_none

def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  inferred_keys= args[2] if len(args)>2 else []
  verbose= args[3] if len(args)>3 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))


  bottle= situation['source']
  cup= situation['receiver']
  arm= situation['handid']
  lb_x_grab= GetOrSet(situation, 'lb_x_grab', lambda:t.GetAttr(bottle,'l_x_grab'))

  t.ExecuteMotion('infer2', {},[bottle],'x')
  x_b= a_get(bottle,'x')
  t.ExecuteMotion('infer2', {},[cup],'x')
  x_c= a_get(cup,'x')

  InsertDict(situation, {'source_attr':{'x':x_b}, 'receiver_attr':{'x':x_c}})
  inferred_keys.append([bottle,'l_x_pour_e'])
  inferred_keys.append([cup,'l_x_pour_l'])
  inferred_keys.append([cup,'l_x_pour_l0'])

  #Container to store the variables derived from situation
  dsit= TContainer(debug=True)
  dsit.t= t
  dsit.x_b= x_b
  dsit.x_c= x_c
  dsit.arm= arm
  dsit.lb_x_grab= lb_x_grab
  dsit.x_grab= Transform(x_b, lb_x_grab)
  dsit.x_pl_displacement_z= GetOrSet(situation, 'x_pl_displacement_z', lambda:0.03)
  dsit.x_pl_center_ratio= GetOrSet(situation, 'x_pl_center_ratio', lambda:None)

  dsit.infer_type= 'ver1' if dsit.x_pl_center_ratio==None else 'ver2'

  if not t.HasAttr(bottle,'l_p_pour_e_set') or not t.HasAttr(cup,'l_p_pour_e_set') or not t.HasAttr(bottle,'pour_start_angle'):
    return False


  bb_margin= 1.15  #TEST; bigger than margin for infer_traj
  t.ExecuteMotion('scene', 'make',[],bb_margin)

  t.ExecuteMotion('infer', bottle,'x')
  x_b= np.array(t.GetAttr(bottle,'x'))
  t.ExecuteMotion('infer', cup,'x')
  x_c= np.array(t.GetAttr(cup,'x'))
  q_curr= t.robot.Q(arm)
  x_w= t.robot.FK(q=q_curr,arm=arm)
  edge_b= TParameterizedPolygon(t.GetAttr(bottle,'l_p_pour_e_set'), center_modifier=lambda c:[0.0,0.0,c[2]])
  edge_c= TParameterizedPolygon(t.GetAttr(cup,'l_p_pour_e_set'), center_modifier=lambda c:[0.0,0.0,c[2]])
  pour_start_angle= t.GetAttr(bottle,'pour_start_angle')

  dsit.q_curr= q_curr
  dsit.x_w= x_w
  dsit.edge_b= edge_b
  dsit.edge_c= edge_c
  dsit.pour_start_angle= pour_start_angle

  fobj= lambda x: EvalPourPosesForFMin(x,dsit,None)
  options= {'CMA_diagonal':1, 'verb_time':0}
  #options['bounds']= [[-math.pi,-math.pi,0.5],[math.pi,math.pi,0.7]]
  options['bounds']= [[edge_b.Bounds[0],edge_c.Bounds[0],0.1],[edge_b.Bounds[1],edge_c.Bounds[1],0.5]]
  options['tolfun']= 1.0e-2 # 1.0e-4
  options['maxfevals']= 1000
  options['verb_log']= False
  #options['scaling_of_variables']= np.array([1.0,1.0,0.01])
  options['scaling_of_variables']= np.array([(edge_b.Bounds[1]-edge_b.Bounds[0])/(2.0*math.pi),(edge_c.Bounds[1]-edge_c.Bounds[0])/(2.0*math.pi),0.01])
  #options['scaling_of_variables']= np.array([1.0,1.0,1.0])
  #typical_x= [0.0,0.0,0.6]
  #options['typical_x']= typical_x
  #options['fixed_variables']= {2:0.5}
  parameters0= [0.5*(edge_b.Bounds[0]+edge_b.Bounds[1]),0.5*(edge_c.Bounds[0]+edge_c.Bounds[1]),0.3]
  scale0= 0.5*math.pi
  if dsit.x_pl_center_ratio!=None:  #parameters[2] is given externally
    options['bounds']= [options['bounds'][0][:2], options['bounds'][1][:2]]
    options['scaling_of_variables']= options['scaling_of_variables'][:2]
    parameters0= parameters0[:2]

  options['scale0_ratio1']= 0.005
  options['scale0_ratio2']= 0.2
  options['init_guess_num']= 5
  options['max_init_guess_num']= [200,1000]
  options['db_search_num']= 5
  options['max_db_search_count']= 1000
  options['db_param_novelty']= 0.1

  parameters_res,score_res= ExecInference(fobj, parameters0, scale0, options,
                      t.database, db_search_key='l_x_pour_e', infer_type=dsit.infer_type)
  if score_res is None:
    t.ExecuteMotion('scene', 'clear')
    CPrint(4,'No solution found')
    return False
  InsertDict(situation, {'infer_info':{'type':dsit.infer_type, 'param':parameters_res, 'score':score_res}})

  lb_x_pe, lc_x_pl, lc_x_pl0= ExpandPourParameters(parameters_res, dsit)

  t.SetAttr(bottle,'l_x_pour_e',  lb_x_pe)
  t.SetAttr(cup,'l_x_pour_l',  lc_x_pl)
  t.SetAttr(cup,'l_x_pour_l0',  lc_x_pl0)


  t.ExecuteMotion('scene', 'clear')

  return t.HasAttr(bottle,'l_x_pour_e') and t.HasAttr(cup,'l_x_pour_l')

